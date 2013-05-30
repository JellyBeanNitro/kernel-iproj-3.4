/*
 * u_atcmd.c - AT Command Handler
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/device.h>

#include <asm/uaccess.h>
#include <asm/io.h>

/* for debug... */
//#define ATCMD_DBG

void atcmd_connect(void *port);
void atcmd_disconnect(void);
int atcmd_queue(const char *buf, size_t count);

#include "u_sdio.c"

static const char *atcmd_ap[] = {
    "+MTC", "%ACS", "%BTTM", "%READY", "%AVR", "%BATL", "%BOFF", "%CAM", "%CHARGE",
    "%CHCOMP", "%ECALL", "%EMT", "%FBOOT", "%FLIGHT", "%FMR", "%FUELRST", "%FUELVAL", "%GKPD",
    "%LANG", "%LCD", "%MACCK", "%MAC", "%MMCFORMAT", "%MMCTOTALSIZE", "%MMCUSEDSIZE", "%MOT",
    "%MPT", "%NCM", "%OSVER", "%PMRST", "%SURV", "%ACCEL", "%COMPASS", "%GYRO", "%ALC", "%PROXIMITY",
    "%RESTART", "%SPM", "%VLC", "%WLANR", "%WLANT", "%WLAN", "%MLT",
    "%PTNCLR", "%LGANDROID", "%LGATSERVICE", "%LGPWD", "%PNUM", "%OSPPWDINIT", "%QFUSE", "%USB","%KCAL",//KCAL AT Command for 325

#if defined(CONFIG_MACH_LGE_I_BOARD_LGU) || defined(CONFIG_MACH_LGE_325_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU)
    "%MTV", "%NFC", "$LGAPP", "%SIMPWDINIT", "%SYNCTYPE",
#elif defined(CONFIG_MACH_LGE_I_BOARD_VZW) || defined(CONFIG_MACH_LGE_IJB_BOARD_VZW)
    "+VZWAPNE", "+CSS", "+CSO", "+CTSA",
#elif defined(CONFIG_MACH_LGE_325_BOARD_VZW)
    "+VZWAPNE", "+CSS", "+CSO", "+CTSA", "%NFC",
#elif defined (CONFIG_MACH_LGE_I_BOARD_SKT) || defined (CONFIG_MACH_LGE_325_BOARD_SKT) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
    "%MTV", "%NFC", "$LGAPP", "%SIMPWDINIT", "*SYNC*UMS", "+SYNCUMS",
#elif defined (CONFIG_MACH_LGE_I_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_DCM) || defined(CONFIG_MACH_LGE_IJB_BOARD_DCM)
    "%MTV","%MTVD", "%IMA", "%IDM", "%EXTIDM", "%FELICATX", "%SWITCH", "%FREQCAL", "%RFIDCK", "%RFREGCAL", "%SWTABLE", "%CFREQ",
#endif

    NULL
};

#ifdef ATCMD_DBG
#define ATCMD_DBG_DATA_ROW 16
#define ATCMD_DBG_DATA(data, len) \
    do { \
        char _atcmd_dbg_hex[3*ATCMD_DBG_DATA_ROW+1]; \
        char _atcmd_dbg_ascii[ATCMD_DBG_DATA_ROW+1]; \
        int idx = 0, row, i; \
        pr_info("%s: ATCMD_DBG_DATA = len(%d)\n", __func__, len); \
        while (idx < len) { \
            memset(_atcmd_dbg_ascii, ' ', ATCMD_DBG_DATA_ROW); \
            _atcmd_dbg_ascii[ATCMD_DBG_DATA_ROW] = '\0'; \
            row = len - idx; \
            row = (row > ATCMD_DBG_DATA_ROW) ? ATCMD_DBG_DATA_ROW: row; \
            for (i = 0; i < row; i++, idx++) { \
                sprintf(&_atcmd_dbg_hex[i*3], "%02X ", data[idx]); \
                _atcmd_dbg_ascii[i] = (data[idx] > 0x20 && data[idx] < 0x7f) ? data[idx] : '?'; \
            } \
            pr_info("%s : %s\n", _atcmd_dbg_ascii, _atcmd_dbg_hex); \
        } \
    } while(0)
#else
#define ATCMD_DBG_DATA() \
    do {} while(0)
#endif

enum {
    ATCMD_TO_AP,
    ATCMD_TO_CP
};

enum {
    ATCMD_OP_MODE_ATCMD,
    ATCMD_OP_MODE_PCSYNC,
};
static char *ATCMD_OP_MODE_PCSYNC_str = "pc sync mode";
static char *ATCMD_OP_MODE_ATCMD_str  = "at cmd  mode";

static struct {
    struct gsdio_port *port;
    int op_mode;
} atcmd_info = {
    .op_mode = ATCMD_OP_MODE_ATCMD,
};

ssize_t atcmd_sdio_write(const char *buf, size_t count);

#define list_last_entry(ptr, type, member) \
    list_entry((ptr)->prev, type, member)

static int atcmd_major;
static DECLARE_WAIT_QUEUE_HEAD(atcmd_read_wait);
static int atcmd_modem = 0;

LIST_HEAD(atcmd_pool);
struct atcmd_request {
    char buf[2048];
    unsigned length;
    unsigned status;

    struct list_head list;
};

static struct class *atcmd_class;
static struct device *atcmd_dev;

struct device *get_atcmd_dev(void)
{
    return atcmd_dev;
}
EXPORT_SYMBOL(get_atcmd_dev);

static char atcmd_name[2048];
static char atcmd_state[2048];

struct atcmd_request *atcmd_alloc_req(void)
{
    struct atcmd_request *req;

    req = kmalloc(sizeof(struct atcmd_request), GFP_ATOMIC);
    if (req == NULL)
        return NULL;

    req->length = 0;
    req->status = 0;

    return req;
}

void atcmd_free_req(struct atcmd_request *req)
{
    kfree(req);
}

int atcmd_to(const char *buf, size_t count)
{
    int i, len;
    char *p;

    if (atcmd_info.op_mode == ATCMD_OP_MODE_PCSYNC)
    {
#ifdef ATCMD_DBG
        pr_info("%s: ATCMD_TO_AP: PCSYNC\n", __func__);
#endif
        return ATCMD_TO_AP;
    }

    strncpy(atcmd_name, buf, count);
    if ((p = strchr(atcmd_name, '=')) || (p = strchr(atcmd_name, '?')))
    {
        *p = '\0';
    }
    else
    {
        p = strchr(atcmd_name, '\r');
        *p = '\0';
    }

    for (i = 0; atcmd_ap[i] != NULL; i++)
    {
        len = strlen(atcmd_ap[i]);

        if (!strcasecmp(&atcmd_name[2], atcmd_ap[i]))
        {

// P2BT kwanseok.kim 20120404 - VBatt for Bluetooth; P930 HW dependancy [START] 
		#ifdef CONFIG_MACH_LGE_I_BOARD_ATNT
		if(!strcasecmp(&atcmd_name[2] , "%BTTM")){
			extern int pm_chg_vbatt_fet_on(int );
			pm_chg_vbatt_fet_on(1); //Vbatt on
		}
		#endif
// P2BT kwanseok.kim 20120404 - VBatt for Bluetooth; P930 HW dependancy [END]

            if ((p = strchr(buf, '=')) || (p = strchr(buf, '?')))
            {
                strncpy(atcmd_state, p, count);
                p = strchr(atcmd_state, '\r');
                *p = '\0';
            }
            else
            {
                atcmd_state[0] = '\0';
            }

#ifdef ATCMD_DBG
            pr_info("%s: ATCMD_TO_AP: matching!!! %s, %s\n", __func__, atcmd_name, atcmd_state);
#endif
            return ATCMD_TO_AP;
        }
    }

#ifdef ATCMD_DBG
    pr_info("%s: ATCMD_TO_CP\n", __func__);
#endif
    return ATCMD_TO_CP;
}

int atcmd_queue(const char *buf, size_t count)
{
    struct gsdio_port *port = atcmd_info.port;
    struct atcmd_request *req;

    if (count <= 0)
    {
        pr_info("%s: count <= 0\n", __func__);
        return -1;
    }

    /* atcmd_pool is empty or new pool */
    if (list_empty(&atcmd_pool) ||
        (req = list_last_entry(&atcmd_pool, struct atcmd_request, list))->status == 1)
    {
        if (atcmd_info.op_mode == ATCMD_OP_MODE_PCSYNC)
        {
            /* do nothing */
        }
        else if (count >= 3 &&
            strncasecmp(buf, "at%", 3) &&
            strncasecmp(buf, "at+", 3) &&
            strncasecmp(buf, "at$", 3) &&
            strncasecmp(buf, "at*", 3))
        {
            return 0;
        }
        else if (count >= 2 && strncasecmp(buf, "at", 2))
        {
            return 0;
        }
        else if (buf[0] != 'a' && buf[0] != 'A')
        {
            return 0;
        }

        req = atcmd_alloc_req();
        if( req == NULL )
        {
            pr_err("can't alloc for req\n" ) ;
            return -ENOMEM ;
        }

        list_add_tail(&req->list, &atcmd_pool);
    }

    memcpy(&req->buf[req->length], buf, count);
    req->length += count;
    req->buf[req->length] = '\0';

#ifdef ATCMD_DBG
    ATCMD_DBG_DATA(req->buf, req->length);
#endif

    if (buf[count-1] == '\r' ||
        buf[count-1] == '\n' ||
        buf[count-1] == '\0' ||
        atcmd_info.op_mode == ATCMD_OP_MODE_PCSYNC)
    {
        if (atcmd_to(req->buf, req->length) == ATCMD_TO_AP)
        {
#ifdef ATCMD_DBG
            pr_info("%s: op_mode = %s\n", __func__,
                    (atcmd_info.op_mode == ATCMD_OP_MODE_ATCMD) ? "ATCMD_OP_MODE_ATCMD" : "ATCMD_OP_MODE_PCSYNC");
#endif

            if (atcmd_info.op_mode == ATCMD_OP_MODE_ATCMD)
            {
                char *envp[3];
                char name[120], state[120];

                snprintf(name, 120, "AT_NAME=%s", atcmd_name);
                snprintf(state, 120, "AT_STATE=%s", atcmd_state);
                envp[0] = name;
                envp[1] = state;
                envp[2] = NULL;

                spin_unlock_irq(&port->port_lock);
                kobject_uevent_env(&atcmd_dev->kobj, KOBJ_CHANGE, envp);
                spin_lock_irq(&port->port_lock);
            }

            if (atcmd_modem)
            {
#ifdef ATCMD_DBG
                pr_info("%s: wake_up! atcmd_read_wait\n", __func__);
#endif
                req->status = 1;
                wake_up_interruptible(&atcmd_read_wait);
            }
            else
            {
#ifdef ATCMD_DBG
                pr_info("%s: throw away! free_req\n", __func__);
#endif
                list_del(&req->list);
                atcmd_free_req(req);
            }
        }
        else
        {
            /* write to sdio */
            atcmd_sdio_write(req->buf, req->length);
            list_del(&req->list);
            atcmd_free_req(req);
        }
    }

    return 1;
}

ssize_t atcmd_usb_write(const char *buf, size_t count)
{
    struct gsdio_port *port = atcmd_info.port;
    struct list_head *pool = &port->write_pool;
    struct usb_ep *in;
    struct usb_request *req;
    int ret;

    if (!port->port_usb) {
        pr_err("%s: usb disconnected\n", __func__);

        /* take out all the pending data from sdio */
        gsdio_read_pending(port);

        return -EIO;
    }

    in = port->port_usb->in;

    spin_lock_irq(&port->port_lock);
    while (list_empty(pool))
    {
        spin_unlock_irq(&port->port_lock);
        usleep(10);
        spin_lock_irq(&port->port_lock);
    }
    req = list_entry(pool->next, struct usb_request, list);
    list_del(&req->list);
    spin_unlock_irq(&port->port_lock);

    memcpy(req->buf, buf, count);
    req->length = count;

    ret = usb_ep_queue(in, req, GFP_KERNEL);
    spin_lock_irq(&port->port_lock);
    if (ret) {
        pr_info("%s: usb ep out queue failed"
                "port:%p, port#%d err:%d\n",
                __func__, port, port->port_num, ret);

        /* could be usb disconnected */
        if (!port->port_usb)
        {
            pr_info("%s: could be usb disconnected\n", __func__);
            gsdio_free_req(in, req);
        }
        else
        {
            list_add(&req->list, pool);
        }
        spin_unlock_irq(&port->port_lock);  // it was locked up without release knk
        return -EAGAIN;
    }

//    port->nbytes_tolaptop += count;
    spin_unlock_irq(&port->port_lock);
    return count;
}

ssize_t atcmd_sdio_write(const char *buf, size_t count)
{
    struct gsdio_port *port = atcmd_info.port;
    int ret = 0;

    if (!port->sdio_open)
    {
        pr_err("%s: sio channel is not open\n", __func__);
        return -1;
    }

    spin_unlock_irq(&port->port_lock);
    ret = sdio_write(port->sport_info->ch, buf, count);
    spin_lock_irq(&port->port_lock);

    return ret;
}

int atcmd_open(struct inode *inode, struct file *filp)
{
    atcmd_modem++;
    return 0;
}

ssize_t atcmd_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
    struct atcmd_request *req;
    unsigned len=0;

    if (list_empty(&atcmd_pool))
    {
        if (filp->f_flags & O_NONBLOCK)
            return -EAGAIN;

#ifdef ATCMD_DBG
        pr_info("%s:1 sleeping?\n", __func__);
#endif
        interruptible_sleep_on(&atcmd_read_wait);
#ifdef ATCMD_DBG
        pr_info("%s:1 awake\n", __func__);
#endif

        if (signal_pending(current))
            return -ERESTARTSYS;

        if (list_empty(&atcmd_pool))
            return 0;

        req = list_first_entry(&atcmd_pool, struct atcmd_request, list);
    }
    else
    {
        req = list_first_entry(&atcmd_pool, struct atcmd_request, list);
#ifdef ATCMD_DBG
        pr_info("%s:2 sleeping?\n", __func__);
#endif
        wait_event_interruptible(atcmd_read_wait, req->status);
#ifdef ATCMD_DBG
        pr_info("%s:2 awake\n", __func__);
#endif
    }

    list_del(&req->list);
    memcpy(buf, req->buf, req->length);
    len = req->length;

#ifdef ATCMD_DBG
    ATCMD_DBG_DATA(buf, len);
#endif

    atcmd_free_req(req);

    return len;
}

ssize_t atcmd_write(struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{
    ssize_t size = 0;

    if(!strncasecmp(buf, ATCMD_OP_MODE_ATCMD_str, sizeof(ATCMD_OP_MODE_ATCMD_str))) {
        pr_info("%s : mode changed to %s", __func__, ATCMD_OP_MODE_ATCMD_str);
        atcmd_info.op_mode = ATCMD_OP_MODE_ATCMD;
        return 0;
    }
    else if(!strncasecmp(buf, ATCMD_OP_MODE_PCSYNC_str, sizeof(ATCMD_OP_MODE_PCSYNC_str))) {
        pr_info("%s : mode changed to %s", __func__, ATCMD_OP_MODE_PCSYNC_str);
        atcmd_info.op_mode = ATCMD_OP_MODE_PCSYNC;
        return 0;
    }

    size = atcmd_usb_write(buf, count);
#if 0 //                                                                                                         
    if (!(count & 0x7FF))
#else
    if (!(count & 0x800))
#endif	
        atcmd_usb_write(NULL, 0);

    return size;
}

int atcmd_release(struct inode *inodes, struct file *filp)
{
    atcmd_modem--;
    if (atcmd_modem < 0) atcmd_modem = 0;
    return 0;
}

void atcmd_connect(void *port)
{
    atcmd_info.port = (struct gsdio_port *)port;
}

void atcmd_disconnect(void)
{
	/*                                        */
    //                                                                      
    if (atcmd_info.op_mode == ATCMD_OP_MODE_PCSYNC)
    {
        pr_info("pcsync disconnect");
        wake_up_interruptible(&atcmd_read_wait);
    }
    //                                                                   
}

struct file_operations atcmd_fops =
{
    .owner      = THIS_MODULE,
    .read       = atcmd_read,
    .write      = atcmd_write,
    .open       = atcmd_open,
    .release    = atcmd_release,
};

static char *atcmd_dev_node(struct device *dev, mode_t *mode)
{
    *mode = 0666;
    return NULL;
}

static ssize_t atcmd_show_name(struct class *class, struct class_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", atcmd_name);
}
static ssize_t atcmd_store_name(struct class *class, struct class_attribute *attr, const char *buf, size_t count)
{
    return atcmd_usb_write(buf, count);
}
static CLASS_ATTR(name, 0644, atcmd_show_name, atcmd_store_name);

static ssize_t atcmd_show_state(struct class *class, struct class_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", atcmd_state);
}
static CLASS_ATTR(state, 0444, atcmd_show_state, NULL);

static __init int atcmd_init(void)
{
    int err;

    atcmd_major = register_chrdev(0, "modem", &atcmd_fops);
    if (atcmd_major < 0)
        return atcmd_major;

    pr_info("%s: ATCMD Handler /dev/lge_atcmd major=%d\n", __func__, atcmd_major);

    atcmd_class = class_create(THIS_MODULE, "atcmd");
    atcmd_class->devnode = atcmd_dev_node;
    atcmd_dev = device_create(atcmd_class, NULL, MKDEV(atcmd_major, 0), NULL, "lge_atcmd");
    atcmd_dev->class = atcmd_class;

    err = class_create_file(atcmd_class, &class_attr_name);
    err = class_create_file(atcmd_class, &class_attr_state);

    return 0;
}

static __exit void atcmd_exit(void)
{
    pr_info("%s\n", __func__);
    class_remove_file(atcmd_class, &class_attr_name);
    class_remove_file(atcmd_class, &class_attr_state);
    unregister_chrdev(atcmd_major, "modem");
}

module_init(atcmd_init);
module_exit(atcmd_exit);

MODULE_LICENSE("Dual BSD/GPL");
