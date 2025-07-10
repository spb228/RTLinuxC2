#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/sched/rt.h>

/* Module Info */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Swapnil Barot");
MODULE_DESCRIPTION("Real time command and status monitor.");
MODULE_VERSION("1.0");

/* FIFO Definitions */
#define CMD_FIFO            0       /* user to kernel status */
#define STATUS_FIFO         1       /* kernel to user status */
#define FIFO_SIZE           1024

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

/* Command Structure */
typedef struct 
{
    int cmd_type; 
    int param1;
    int param2; 
    ktime_t timestamp; 
} rt_command_t; 

/* Status Structure */
typedef struct
{
    int status; 
    int frequency_hz; 
    int threshold; 
    unsigned long counter; 
    unsigned long alarm_count; 
    ktime_t last_update; 
    int cpu_load_percent; 
} rt_status_t;

/* Global variables */
static struct task_struct *rt_monitor_thread;   /* previous rt_task but renamed due to naming conflict */
static struct hrtimer timer; 
static rt_status_t system_status; 
static int task_running = 0;                /* 0 implies task not active */
static int monitoring_active = 0;           /* 0 implies monitoring not enabled */
static ktime_t task_period;                 /* period between monitoring cycles */

/* device driver variables */
static int major_cmd = 240, major_status = 241;     /* fixed major device numbers */
static struct cdev cmd_cdev, status_cdev; 
static struct class *rtf_class; 
static struct device *cmt_device, *status_device; 

/* simulated sensor data */
static int current_sensor_value = 50; 
static int sensor_direction = 1; 

/* synchronization primitives - thread safe access to shared data */
static DEFINE_MUTEX(cmd_mutex); 
static DEFINE_MUTEX(status_mutex);
static DECLARE_WAIT_QUEUE_HEAD(status_wait);
static int status_ready = 0; 

/* high-resolution timer calllback function */
static enum hrtimer_restart timer_func(struct hrtimer *timer)
{
    if (monitoring_active && task_running)
    {
        wake_up_process(rt_monitor_thread);
        hrtimer_forward_now(timer, task_period);    /* schedule next timer event */
        return HRTIMER_RESTART;                     /* tell timer subsystem to restart */
    }
    return HRTIMER_NORESTART;                       /* stop the timer */                    
}

/* simulate sensor reading function */
static int simulate_sensor_reading(void)
{
    static int noise = 0; 
    /* create sawtooth pattern */
    current_sensor_value += sensor_direction * 2; 
    if (current_sensor_value >= 90)
    {
        sensor_direction = -1;
    } 
    else if (current_sensor_value <= 10)
    {
        sensor_direction = 1;
    }
    noise = (noise + 7) % 11 - 5;

    return current_sensor_value + noise;
}

/* update system status information */
static void update_status(void)
{
    system_status.counter++;
    system_status.last_update = ktime_get(); 
    //system_status.cpu_load_percent = (current_sensor_value * 100) / 100; 
    system_status.cpu_load_percent = current_sensor_value;
    if (system_status.cpu_load_percent > 100)
    {
        system_status.cpu_load_percent = 100; /* clamp to 100 */
    }
}

/* send status update to user space app */
static void send_status_update(void)
{
    mutex_lock(&status_mutex); 
    status_ready = 1;
    mutex_unlock(&status_mutex);
    wake_up_interruptible(&status_wait); 
}

/* process incoming command from user space */
static void process_command(rt_command_t *cmd)
{
    pr_debug("Processing command: %d, params: %d, %d\n", cmd->cmd_type, cmd->param1, cmd->param2); 

    switch(cmd->cmd_type)
    {
        case CMD_START_MONITOR:
            if (!monitoring_active)
            {
                monitoring_active = 1; 
                system_status.status = STATUS_MONITORING; 
                hrtimer_start(&timer, task_period, HRTIMER_MODE_REL);  /* start timer */
                pr_info("Monitoring started\n");
            }
        break; 
        
        case CMD_STOP_MONITOR:
            if (monitoring_active)
            {
                monitoring_active = 0; 
                system_status.status = STATUS_IDLE; 
                hrtimer_cancel(&timer);     /* stop timer */
                pr_info("Monitoring stopped\n");
            }
        break; 
        
        case CMD_SET_FREQUENCY:
            if (cmd->param1 > 0 && cmd->param1 <= 1000)
            {
                system_status.frequency_hz = cmd->param1; 
                /* convert freq to timer period (nsec)*/
                task_period = ktime_set(0, NSEC_PER_SEC / cmd->param1);         
                pr_info("Frequency set to %d Hz\n", cmd->param1);
            }
        break;

        case CMD_SET_THRESHOLD: 
            if (cmd->param1 >= 0 && cmd->param2 <= 100)
            {
                system_status.threshold = cmd->param1; 
                pr_info("Threshold set to %d\n", cmd->param1);
            }
        break; 
        
        case CMD_RESET_COUNTERS:
            system_status.counter = 0; 
            system_status.alarm_count = 0; 
            pr_info("Counters reset\n");
        break; 

        default:
            pr_warn("Unknown command: %d\n", cmd->cmd_type); 
        break;
    }
}





static int __init simple_module_init(void)
{
    printk(KERN_INFO "Hello from simple module!\n");
    return 0; 
}

static void __exit simple_module_exit(void)
{
    printk(KERN_INFO "Goodbye from simple module\n");
}

module_init(simple_module_init);
module_exit(simple_module_exit);