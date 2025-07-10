#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <pthread.h>
#include <errno.h>

#define RET_SUCCESS         0
#define RET_ERROR           -1

/* Command definitions - these match the kernel module */
#define CMD_START_MONITOR   1
#define CMD_STOP_MONITOR    2
#define CMD_SET_FREQUENCY   3
#define CMD_SET_THRESHOLD   4
#define CMD_RESET_COUNTERS  5
#define CMD_GET_STATUS      6

/* Status defintions */
#define STATUS_IDLE         0
#define STATUS_MONITORING   1
#define STATUS_ALARM        2
#define STATUS_ERROR        3

/* RT-FIFO device paths */
#define CMD_FIFO_DEV        "/dev/rtf0"
#define STATUS_FIFO_DEV     "/dev/rtf1"

/* Command Structure */
typedef struct 
{
    int cmd_type; 
    int param1;
    int param2; 
    long long timestamp; 
} rt_command_t; 

/* Status Structure */
typedef struct
{
    int status; 
    int frequency_hz; 
    int threshold; 
    unsigned long counter; 
    unsigned long alarm_count; 
    long long last_update; 
    int cpu_load_percent; 
} rt_status_t;

/* Global variables */
static int cmd_fd = -1; 
static int status_fd = -1; 
static int running = 1; 
static int display_mode = 0; /* 0=compact, 1=detailed */
static rt_status_t last_status; 
static pthread_t status_thread; 
static pthread_mutex_t status_mutex = PTHREAD_MUTEX_INITIALIZER; 

/* Function prototypes */
static int open_fifos(void);
static void close_fifos(void);
static int send_command(int cmd_type, int param1, int param2);
static void* status_monitor_thread(void* arg); 
static void display_status(const rt_status_t* status); 
static void display_help(void); 
static void signal_handler(int sig);
static const char* status_to_string(int status);
static void clear_screen(void);
static long long get_timestamp_ns(void);

static long long get_timestamp_ns(void)
{
    struct timeval tv; 
    gettimeofday(&tv, NULL);
    long long retVal = (long long)tv.tv_sec * 1000000000LL + (long long)tv.tv_usec* 1000LL; 
    return retVal;
}

static int open_fifos(void)
{
    /* open fifo */
    cmd_fd = open(CMD_FIFO_DEV, O_WRONLY | O_NONBLOCK);
    if (cmd_fd < 0)
    {
        perror("Failed to open command FIFO"); 
        return RET_ERROR; 
    }

    status_fd = open(STATUS_FIFO_DEV, O_RDONLY | O_NONBLOCK);
    if (status_fd < 0)
    {
        perror("Failed to open status FIFO");
        close (cmd_fd); 
        cmd_fd = -1;  
        return RET_ERROR; 
    }

    puts("RT-FIFO devices opened successfully");
    return RET_SUCCESS;
}

static void close_fifos(void)
{
    if (cmd_fd >= 0)
    {
        close(cmd_fd);
        cmd_fd = -1; 
    }
    if (status_fd >= 0)
    {
        close(status_fd);
        status_fd = -1; 
    }
}

static int send_command(int cmd_type, int param1, int param2)
{
    rt_command_t cmd; 
    ssize_t bytes_written; 

    if (cmd_fd < 0)
    {
        puts("[ERROR] Command FIFO not open");
        return RET_ERROR;
    }

    cmd.cmd_type = cmd_type; 
    cmd.param1 = param1; 
    cmd.param2 = param2; 
    cmd.timestamp = get_timestamp_ns(); 

    bytes_written = write(cmd_fd, &cmd, sizeof(rt_command_t));
    if (bytes_written != sizeof(rt_command_t))
    {
        perror("Failed to send command or sent corrupted command");
        return RET_ERROR; 
    }

    return RET_SUCCESS;
}

static const char* status_to_string(int status)
{
    switch(status)
    {
        case STATUS_IDLE:           return "IDLE";
        case STATUS_MONITORING:     return "MONITORING";
        case STATUS_ALARM:          return "ALARM";
        case STATUS_ERROR:          return "ERROR";
        default:                    return "UNKNOWN"; 
    }
}

static void clear_screen(void)
{
    printf("\033[2J\033[H");
}

static void display_status(const rt_status_t* status)
{
    static int update_count = 0; 
    time_t now = time(NULL);

    if (display_mode == 0)
    {
        /* compact display */
        printf("\r[%s] Count: %6lu | Alarms: %3lu | CPU: %2d%% | Freq: %2dHz | Thr: %2d | Sensor:  %3d",
            status_to_string(status->status),
            status->counter,
            status->alarm_count, 
            status->cpu_load_percent, 
            status->frequency_hz,
            status->threshold,
            status->cpu_load_percent);
        fflush(stdout);
    }
    else
    {
        /* TODO: Detailed Display Values*/
        printf("[ERROR] Detailed display support to be added in the future...");
    }

    update_count++; 
}

static void* status_monitor_thread(void* arg)
{
    rt_status_t status; 
    ssize_t bytes_read;
    fd_set readfds; 
    struct timeval timeout; 

    printf("Status Monitor thread started\n");

    while (running)
    {
        FD_ZERO(&readfds);
        FD_SET(status_fd, &readfds);

        timeout.tv_sec = 1;
        timeout.tv_usec = 0;

        int result = select(status_fd + 1, &readfds, NULL, NULL, &timeout);

        if (result > 0 && FD_ISSET(status_fd, &readfds))
        {
            bytes_read = read(status_fd, &status, sizeof(rt_status_t));
            if (bytes_read == sizeof(rt_status_t))
            {
                pthread_mutex_lock(&status_mutex);
                last_status = status;
                pthread_mutex_unlock(&status_mutex);
                display_status(&status);
            }
            else if (bytes_read > 0)
            {
                printf("WARNING: Received partial status data (%zd bytes)\n", bytes_read);
            }
        }
        else if (result < 0 && errno != EINTR)
        {
            perror("[ERROR] status select failed");
            break; 
        }
    }

    printf("\n[ERROR] Status monitor thread terminated\n");
    return NULL; 
}

static void display_help(void)
{
    printf("\n RTLinux Monitor Commands\n");
    printf("start                       - start monitoring\n");
    printf("stop                        - stop monitoring\n");
    printf("freq <hz>                   - set update freq (1 - 1000 Hz)\n");
    printf("threshold <value>           - set alarm threshold (0 - 100)\n");
    printf("reset                       - reset counters\n");
    printf("status                      - get current status\n");
    printf("display [compact|detail]    - set display mode (detail mode to be added in future)\n");
    printf("help                        - show this help\n");
    printf("quit                        - exit application\n");
}

static void signal_handler(int sig)
{
    printf("\nReceived signal %d, shutting down...\n", sig);
    running = 0; 
}

int main(int argc, char* argv[])
{
    char input[256]; 
    char command[64]; 
    int param1, param2; 
    int ret; 

    printf("\nRTLinux Real-time Command & Status Monitor\n");
    printf("============================================\n\n");

    /* setup signal handlers */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* Open RT-FIFO devices */
    if (open_fifos() < 0)
    {
        printf("Failed to open RT-FIFO devices. Is the kernel module loaded?\n");
        printf("Try: insmod rt_monitor.ko\n");
        return 1; 
    }

    /* Create status monitorting thread */
    ret = pthread_create(&status_thread, NULL, status_monitor_thread, NULL);
    if (ret != 0)
    {
        printf("Failed to create status thread %s\n", strerror(ret));
        close_fifos();
        return 1; 
    }

    /* Get initial status */
    send_command(CMD_GET_STATUS, 0, 0);

    display_help();

    /* main command loop */
    printf("rt_monitor> ");
    fflush(stdout);
    while (running && fgets(input, sizeof(input), stdin))
    {
        /* parse command */
        param1 = param2 = 0; 
        int args = sscanf(input, "%63s %d %d", command, &param1, &param2);
        if (args < 1)
        {
            printf("rt_monitor> ");
            fflush(stdout);
            continue;
        }

        /* process command */
        if (strcmp(command, "start") == 0)
        {
            send_command(CMD_START_MONITOR, 0, 0);
            printf("[DEBUG] start command sent\n");
        }
        else if (strcmp(command, "stop") == 0)
        {
            send_command(CMD_STOP_MONITOR, 0, 0);
            printf("[DEBUG] stop command sent\n");
        }
        else if (strcmp(command, "freq") == 0)
        {
            if (args >= 2 && param1 > 0 && param1 <= 1000)
            {
                send_command(CMD_SET_FREQUENCY, param1, 0);
                printf("[DEBUG] Frequency set to %d Hz\n", param1);
            }
            else 
            {
                printf("Usage: freq <hz> (1-1000)\n");
            }
        }
        else if (strcmp(command, "threshold") == 0)
        {
            if (args >= 2 && param1 > 0 && param1 <= 100)
            {
                send_command(CMD_SET_THRESHOLD, param1, 0);
                printf("[DEBUG] Threshold set to %d Hz\n", param1);
            }
            else 
            {
                printf("Usage: threshold <value> (1-100)\n");
            }
        }
        else if (strcmp(command, "reset") == 0)
        {
            send_command(CMD_RESET_COUNTERS, 0, 0);
            printf("[DEBUG] reset command sent\n");
        }
        else if (strcmp(command, "status") == 0)
        {
            send_command(CMD_GET_STATUS, 0, 0);
            printf("[DEBUG] get status command sent\n");
        }
        else if (strcmp(command, "display") == 0)
        {
            if (args >= 2)
            {
                if (strcmp(input + strlen(command) + 1, "compact\n") == 0)
                {
                    display_mode = 0; 
                    printf("[DEBUG] Display Mode: compact\n");
                }
                else 
                {
                    printf("[ERROR] only compact mode support currently\n");
                }
            }
            else
            {
                printf("Current display mode: %s\n", display_mode ? "detailed" : "compact");
            }
        }
        else if (strcmp(command, "help") == 0)
        {
            display_help();
        }
        else if (strcmp(command, "quit") == 0 || strcmp(command, "exit") == 0)
        {
            break; 
        }
        else
        {
            printf("[ERROR] unknown command '%s' (type 'help')\n", command);
        }

        if (running)
        {
            printf("rt_monitor> ");
            fflush(stdout);
        }
    }

    /* cleanup */
    running = 0; 
    pthread_join(status_thread, NULL);
    close_fifos();
    printf("\nRTLinux monitor application terminated\n");

    return RET_SUCCESS;
}
