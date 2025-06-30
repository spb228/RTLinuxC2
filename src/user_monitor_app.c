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
    int threhold; 
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

int main(void)
{
    printf("Hello World!\n");
    return 0;
}