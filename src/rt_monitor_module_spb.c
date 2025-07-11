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
static struct device *cmd_device, *status_device; 

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

/* real-time monitor task function */
static int rt_monitor_task(void *arg)
{
    /* set this thread to real-time priority (SCHED FIFO with highest priority) */
    struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 }; 
    //sched_setscheduler(current, SCHED_FIFO, &param); /* getting error "sched_setscheduler not defined"
    sched_set_fifo(current);

    pr_info("RT Monitor task started, period %lld ns\n", ktime_to_ns(task_period));

    while (!kthread_should_stop() && task_running)
    {
        /* put thread to sleep until woken by timer */
        set_current_state(TASK_UNINTERRUPTIBLE);
        schedule();

        if (monitoring_active)
        {
            current_sensor_value = simulate_sensor_reading();
            update_status();

            /* check for alarm condition */
            if (current_sensor_value > system_status.threshold)
            {
                system_status.status = STATUS_ALARM; 
                system_status.alarm_count++; 
            }
            else if (system_status.status == STATUS_ALARM)
            {
                system_status.status = STATUS_MONITORING; 
            }

            send_status_update(); /* notify user space app of status update */
        }
    }

    pr_info("RT monitor task terminated\n");
        return 0; 
}

/* Command device file ops - handle user space access to devices (/dev/rtf0) */
static int cmd_open(struct inode *inode, struct file *file) {  return 0;  } /* open cmd device for writing */

static int cmd_release(struct inode *inode, struct file *file) { return 0; } /* relinquish cmd device */

static ssize_t cmd_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    rt_command_t cmd; 

    /* validate write size */
    if (count != sizeof(rt_command_t)) return -EINVAL; 

    /* copy command from user space to kernel space. if fail, return error */
    if (copy_from_user(&cmd, buf, sizeof(rt_command_t))) return -EFAULT;
    
    cmd.timestamp = ktime_get();

    /* process command */
    mutex_lock(&cmd_mutex);
    process_command(&cmd); 
    mutex_unlock(&cmd_mutex); 

    return count; /* return num of bytes written */
}

/* file ops structure for command device */
static struct file_operations cmd_fops = 
{
    .open = cmd_open,
    .release = cmd_release,
    .write = cmd_write,
};

/* Command device file ops - handle user space access to devices (/dev/rtf0) */
static int status_open(struct inode *inode, struct file *file) { return 0; }

static int status_release(struct inode *inode, struct file *file) { return 0; }

static ssize_t status_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    if (count < sizeof(rt_status_t)) return -EINVAL; 

    if (file->f_flags & O_NONBLOCK)
    {
        /* if non-blocking, return immediately if no data available */
        if (!status_ready) return -EAGAIN; 
    }
    else
    {
        /* if blocking, wait for status data to be available */
        if (wait_event_interruptible(status_wait, status_ready)) return -ERESTARTSYS; 
    }

    mutex_lock(&status_mutex);
    if (copy_to_user(buf, &system_status, sizeof(rt_status_t)))
    {
        /* TODO: investigate if need to put status ready to 0 here as well ? */
        mutex_unlock(&status_mutex);
        return -EFAULT; 
    }
    status_ready = 0; 
    mutex_unlock(&status_mutex);

    return sizeof(rt_status_t); 
}

/* poll for status device */
static unsigned int status_poll(struct file *file, poll_table *wait)
{
    /* register this file with wait queue */
    poll_wait(file, &status_wait, wait);

    /* return appropriate poll flags */
    return status_ready ? (POLLIN | POLLRDNORM) : 0; 
}

/* file ops structure for status device */
static struct file_operations status_fops = 
{
    .open = status_open,
    .release = status_release, 
    .read = status_read, 
    .poll = status_poll,
};


static int __init rt_monitor_init(void)
{
    int ret; 
    pr_info("RTLinux Real-Time Monitor Initializing\n");

    /* initialize system status with def values */
    memset(&system_status, 0, sizeof(rt_status_t)); 
    system_status.status = STATUS_IDLE; 
    system_status.frequency_hz = 10; 
    system_status.threshold = 75; 
    task_period = ktime_set(0, NSEC_PER_SEC / 10); /* 100ms period (10 Hz) */

    /* register character devices */
    ret = register_chrdev(major_cmd, "rtf0", &cmd_fops); 
    if (ret < 0)
    {
        pr_err("Failed to register command device: %d\n", ret); 
    }
    ret = register_chrdev(major_status, "rtf1", &status_fops); 
    if (ret < 0)
    {
        pr_err("Failed to register status device: %d\n", ret);
        unregister_chrdev(major_cmd, "rtf0");
        return ret;
    }

    /* create device class for udev */
    rtf_class = class_create("rtf"); 
    if (IS_ERR(rtf_class))
    {
        pr_err("Failed to create device class\n");
        unregister_chrdev(major_cmd, "rtf0");
        unregister_chrdev(major_status, "rtf1");
        return PTR_ERR(rtf_class);
    }

    /* create device nodes */
    cmd_device = device_create(rtf_class, NULL, MKDEV(major_cmd, 0), NULL, "rtf0");
    status_device = device_create(rtf_class, NULL, MKDEV(major_status, 0), NULL, "rtf1");

    /* initialize high-res timer which will periodically wake up RT task */
    hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    timer.function = timer_func; 

    /* create and start RT monitorting task */
    task_running = 1; 
    rt_monitor_thread = kthread_create(rt_monitor_task, NULL, "rt_monitor");
    if (IS_ERR(rt_monitor_thread))
    {
        pr_err("Failed to create RT Task: %d\n", PTR_ERR(rt_monitor_thread));
        task_running = 0; 
        device_destroy(rtf_class, MKDEV(major_status, 0)); 
        device_destroy(rtf_class, MKDEV(major_cmd, 0));
        class_destroy(rtf_class);
        unregister_chrdev(major_cmd, "rtf0");
        unregister_chrdev(major_status, "rtf1");
        return PTR_ERR(rt_monitor_thread);
    }

    wake_up_process(rt_monitor_thread); 
    pr_info("RTLinux Real-Time Monitor initialized successfully\n");
    pr_info("Command fifo: /dev/rtf0\n");
    pr_info("Status fifo: /dev/rtf1\n");

    return 0; 
}

static void __exit rt_monitor_exit(void)
{
    pr_info("RTLinux Real-time Monitor shutting down\n");
    task_running = 0; 
    monitoring_active = 0; 
    hrtimer_cancel(&timer); 

    if (rt_monitor_thread) 
    {
        kthread_stop(rt_monitor_thread);
    }

    device_destroy(rtf_class, MKDEV(major_status, 0)); 
    device_destroy(rtf_class, MKDEV(major_cmd, 0));
    class_destroy(rtf_class);
    unregister_chrdev(major_cmd, "rtf0");
    unregister_chrdev(major_status, "rtf1");

    pr_info("RTLinux Real-time Monitor shutdown complete\n");
}

module_init(rt_monitor_init);
module_exit(rt_monitor_exit);