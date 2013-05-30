/*
 *
 *
 *
 */

#if 1 /*                                               */
#include <linux/module.h>   /* kernel module definitions */
#endif
#include <linux/platform_device.h>
#include <asm/io.h>
#include <mach/restart.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

void wdog_test_function(struct work_struct *work);
DECLARE_DELAYED_WORK(wdog_test_work_struct, wdog_test_function);

void wdog_test_function(struct work_struct *work) {
	mdelay(15000);
}

static int panic_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i=0;

	return sprintf(buf, "%d\n", i);
}

static int panic_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int test_result=0;
	
	sscanf(buf, "%d\n", &test_result);


	if (test_result == 2 )
		BUG();
	else if (test_result == 10 )
#if defined(CONFIG_LGE_ERROR_HANDLER)
	//Kernel panic
		*(volatile int*)(0x00000000) = 0x0;
	else if (test_result == 3 )
		msm_set_restart_mode(SUB_THD_F_PWR);
	else if (test_result == 4 )
		msm_set_restart_mode(SUB_UNKNOWN);
	//Watchdog TZ crash
	else if (test_result == 5 ) {
		schedule_delayed_work_on(0, &wdog_test_work_struct, msecs_to_jiffies(100));
		return count;
	}

#endif

	panic("test panic");
	
	return count;
}
DEVICE_ATTR(gen_panic, 0770, panic_show, panic_store);

static int lge_tempdevice_probe(struct platform_device *pdev)
{
	int err;

	err = device_create_file(&pdev->dev, &dev_attr_gen_panic);
	if (err < 0) 
		printk("%s : Cannot create the sysfs\n", __func__);

	return 0;
	
}

static struct platform_device lgetemp_device = {
	.name = "lge_test",
	.id		= -1,
};

static struct platform_driver this_driver = {
	.probe = lge_tempdevice_probe,
	.driver = {
		.name = "lge_test",
	},
};

int __init lge_tempdevice_init(void)
{
	printk("%s\n", __func__);
	platform_device_register(&lgetemp_device);

	return platform_driver_register(&this_driver);
}

module_init(lge_tempdevice_init);
MODULE_DESCRIPTION("Just temporal code");
MODULE_AUTHOR("MoonCheol Kang <neo.kang@lge.com>");
MODULE_LICENSE("GPL");
