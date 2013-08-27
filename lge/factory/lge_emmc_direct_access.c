/*
 * arch/arm/mach-msm/lge/lge_emmc_direct_access.c
 *
 * Copyright (C) 2010 LGE, Inc
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <asm/div64.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/kmod.h>
#include <linux/workqueue.h>
#include <lg_backup_items.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <lg_diag_cfg.h>
#include <mach/oem_rapi_client.h>
#include <linux/mutex.h>

/* Some useful define used to access the MBR/EBR table */
//#define BLOCK_SIZE                0x200
#define TABLE_ENTRY_0             0x1BE
#define TABLE_ENTRY_1             0x1CE
#define TABLE_ENTRY_2             0x1DE
#define TABLE_ENTRY_3             0x1EE
#define TABLE_SIGNATURE           0x1FE
#define TABLE_ENTRY_SIZE          0x010

#define OFFSET_STATUS             0x00
#define OFFSET_TYPE               0x04
#define OFFSET_FIRST_SEC          0x08
#define OFFSET_SIZE               0x0C
#define COPYBUFF_SIZE             (1024 * 16)
#define BINARY_IN_TABLE_SIZE      (16 * 512)
#define MAX_FILE_ENTRIES          20

#define MMC_BOOT_TYPE 0x48
#define MMC_SYSTEM_TYPE 0x78
#define MMC_USERDATA_TYPE 0x79

#define MMC_RCA 2

#define MAX_PARTITIONS 64

#define GET_LWORD_FROM_BYTE(x)    ((unsigned)*(x) | \
        ((unsigned)*((x)+1) << 8) | \
        ((unsigned)*((x)+2) << 16) | \
        ((unsigned)*((x)+3) << 24))

#define PUT_LWORD_TO_BYTE(x, y)   do{*(x) = (y) & 0xff;     \
    *((x)+1) = ((y) >> 8) & 0xff;     \
    *((x)+2) = ((y) >> 16) & 0xff;     \
    *((x)+3) = ((y) >> 24) & 0xff; }while(0)

#define GET_PAR_NUM_FROM_POS(x) ((((x) & 0x0000FF00) >> 8) + ((x) & 0x000000FF))

#define MMC_BOOT_TYPE 0x48
#define MMC_EXT3_TYPE 0x83
#define MMC_VFAT_TYPE 0xC

//                                              
// MOD 0010090: [FactoryReset] Enable Recovery mode FactoryReset
#define MMC_RECOVERY_TYPE		0x60
#define MMC_MISC_TYPE 0x77
#define MMC_XCALBACKUP_TYPE 0x6E
//                                            


typedef struct MmcPartition MmcPartition;

static unsigned ext3_count = 0;

// LG_FW : 2011.07.06 moon.yongho : saving webdload status variable to eMMC. ----------[[
#ifdef LG_FW_WEB_DOWNLOAD	
static char *ext3_partitions[] = {"persist", "bsp", "blb", "tombstones", "drm", "fota", "system", "cache", "userdata","NONE"};
#else	
static char *ext3_partitions[] = {"system", "userdata", "cache", "NONE"};
#endif /*LG_FW_WEB_DOWNLOAD*/	
// LG_FW : 2011.07.06 moon.yongho -----------------------------------------------------]]

static unsigned vfat_count = 0;
static char *vfat_partitions[] = {"modem", "NONE"};

struct MmcPartition {
    char *device_index;
    char *filesystem;
    char *name;
    unsigned dstatus;
    unsigned dtype ;
    unsigned dfirstsec;
    unsigned dsize;
};

typedef struct {
    MmcPartition *partitions;
    int partitions_allocd;
    int partition_count;
} MmcState;

static MmcState g_mmc_state = {
    NULL,   // partitions
    0,      // partitions_allocd
    -1      // partition_count
};

typedef struct {
	char ret[32];
} testmode_rsp_from_diag_type;

#define FACTORY_RESET_STR_SIZE 11
#define FACTORY_RESET_STR "FACT_RESET_"
#define MMC_DEVICENAME "/dev/block/mmcblk0"
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
#define MMC_DEVICENAME_MISC "/dev/block/mmcblk0p25"
#endif
#define LCD_K_CAL_SIZE 6//kcal for 325
static unsigned char lcd_buf[LCD_K_CAL_SIZE]={255,};//kcal for 325
static unsigned char global_buf[FACTORY_RESET_STR_SIZE+2];

#ifdef CONFIG_LGE_ERI_DOWNLOAD
#define ERI_FILE_PATH "/data/eri/eri.bin"
extern void remote_eri_rpc(void);

static struct workqueue_struct *eri_dload_wq;
struct __eri_data {
    unsigned long flag;
    struct work_struct work;
};
static struct __eri_data eri_dload_data;

static void eri_dload_func(struct work_struct *work);
#endif

#ifdef CONFIG_LGE_DID_BACKUP
static struct workqueue_struct *did_dload_wq;
struct __did_data {
    unsigned long flag;
    struct work_struct work;
};
static struct __did_data did_dload_data;

static void did_dload_func(struct work_struct *work);
#endif

#ifdef CONFIG_LGE_VOLD_SUPPORT_CRYPT
static struct workqueue_struct *cryptfs_cmd_wq;
struct __cryptfs_cmd_data {
    unsigned long cmd;
    struct work_struct work;
};
static struct __cryptfs_cmd_data cryptfs_cmd_data;

static void cryptfs_cmd_func(struct work_struct *work);
#endif

int lge_erase_block(int secnum, size_t size);
int lge_write_block(unsigned int secnum, unsigned char *buf, size_t size);
int lge_read_block(unsigned int secnum, unsigned char *buf, size_t size);

static int dummy_arg;

int boot_info = 0;

//2012.02.24 kabjoo.choi add it for testing  kernel panic  
static DEFINE_MUTEX(emmc_dir_lock);


static int boot_info_write(const char *val, struct kernel_param *kp)
{
    unsigned long flag=0;

    if(val == NULL)
    {
        printk(KERN_ERR "%s, NULL buf\n", __func__);
        return -1;
    }

    flag = simple_strtoul(val,NULL,10);
    boot_info = (int)flag;
    printk(KERN_INFO "%s, flag : %d\n", __func__, boot_info);

    queue_work(did_dload_wq, &did_dload_data.work); //DID BACKUP support to DLOD Mode

    return 0;
}

static int boot_info_read(char *buffer, struct kernel_param *kp)
{
    int ret;
    printk(KERN_INFO "%s, flag : %d\n", __func__, boot_info);
    ret = sprintf(buffer, "%d", boot_info);
    return ret;
}

module_param_call(boot_info, boot_info_write, boot_info_read, &boot_info, S_IWUSR | S_IRUGO);

#ifdef CONFIG_LGE_MANUAL_TEST_MODE
int lg_manual_test_mode = 0;
extern int msm_get_manual_test_mode(void);

static int android_get_manual_test_mode(char *buffer, struct kernel_param *kp)
{
    int ret;
    lg_manual_test_mode = msm_get_manual_test_mode();
    printk(KERN_ERR "[%s] get manual test mode - %d\n", __func__, lg_manual_test_mode);
    ret = sprintf(buffer, "%d", lg_manual_test_mode);
    return ret;
}

module_param_call(manual_test_mode, NULL, android_get_manual_test_mode, &dummy_arg, 0444);
#endif

#if defined (CONFIG_MACH_LGE_I_BOARD_DCM) || defined(CONFIG_MACH_LGE_325_BOARD_DCM)
extern void msm_rw_felica(uint32_t event, uint8_t is_write, char *buf, uint32_t len);

static int felica_key_write(const char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);
    msm_rw_felica(LG_FW_FELICA_KEY, 1, (char *)buf, 8);
    return 8;
}

static int felica_key_read(char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);
    msm_rw_felica(LG_FW_FELICA_KEY, 0, buf, 8);
    return 8;
}

module_param_call(felica_key, felica_key_write, felica_key_read, &dummy_arg, 0644);

static int felica_sign_write(const char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);
    msm_rw_felica(LG_FW_FELICA_SIGN, 1, (char *)buf, 16);
    return 16;
}

static int felica_sign_read(char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);
    msm_rw_felica(LG_FW_FELICA_SIGN, 0, buf, 16);
    return 16;
}

module_param_call(felica_sign, felica_sign_write, felica_sign_read, &dummy_arg, 0644);

#define ANNOYING_FLC_1ST_DATA_SIZE (16)
#define ANNOYING_FLC_2ND_DATA_SIZE (1)

extern int msm_rw_annoying_flc(uint32_t event, uint8_t is_write, char *buf, uint32_t len);

static int annoying_flc_1st_write(const char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);

    return msm_rw_annoying_flc(LG_FW_ANNOYING_FLC_1ST, 1, (char *)buf, ANNOYING_FLC_1ST_DATA_SIZE);
}

static int annoying_flc_1st_read(char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);

    return msm_rw_annoying_flc(LG_FW_ANNOYING_FLC_1ST, 0, (char *)buf, ANNOYING_FLC_1ST_DATA_SIZE);
}

module_param_call(annoying_flc_1st, annoying_flc_1st_write, annoying_flc_1st_read, &dummy_arg, 0644);

static int annoying_flc_2nd_write(const char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);

    return msm_rw_annoying_flc(LG_FW_ANNOYING_FLC_2ND, 1, (char *)buf, ANNOYING_FLC_2ND_DATA_SIZE);
}

static int annoying_flc_2nd_read(char *buf, struct kernel_param *kp)
{
    pr_info("%s\n", __func__);

    return msm_rw_annoying_flc(LG_FW_ANNOYING_FLC_2ND, 0, buf, ANNOYING_FLC_2ND_DATA_SIZE);
}

module_param_call(annoying_flc_2nd, annoying_flc_2nd_write, annoying_flc_2nd_read, &dummy_arg, 0644);
#endif

int db_integrity_ready = 0;
module_param(db_integrity_ready, int, S_IWUSR | S_IRUGO);

int fpri_crc_ready = 0;
module_param(fpri_crc_ready, int, S_IWUSR | S_IRUGO);

int file_crc_ready = 0;
module_param(file_crc_ready, int, S_IWUSR | S_IRUGO);

int db_dump_ready = 0;
module_param(db_dump_ready, int, S_IWUSR | S_IRUGO);

int db_copy_ready = 0;
module_param(db_copy_ready, int, S_IWUSR | S_IRUGO);

int external_memory_test = 0;
module_param(external_memory_test, int, S_IWUSR | S_IRUGO);

unsigned char fota_id_read[20] = "0";
module_param_string(fota_id_read, fota_id_read, 20, S_IWUSR | S_IRUGO);

unsigned char testmode_result[20] = {0,};
module_param_string(testmode_result, testmode_result, 20, S_IWUSR | S_IRUGO);

#if defined(CONFIG_MACH_LGE_325_BOARD)
int deepsleep_result = 2;
module_param(deepsleep_result, int, S_IWUSR | S_IRUGO);
#endif

testmode_rsp_from_diag_type integrity_ret;
static int integrity_ret_write(const char *val, struct kernel_param *kp)
{
	memcpy(integrity_ret.ret, val, 32);
	return 0;
}
static int integrity_ret_read(char *buf, struct kernel_param *kp)
{
	memcpy(buf, integrity_ret.ret, 32);
	return 0;
}

module_param_call(integrity_ret, integrity_ret_write, integrity_ret_read, &dummy_arg, S_IWUSR | S_IRUGO);

#ifdef CONFIG_LGE_ERI_DOWNLOAD
static int eri_write(const char *val, struct kernel_param *kp)
{
	mm_segment_t oldfs;
	//int read;
	unsigned long flag = 5;
	struct file *phMscd_Filp = NULL;
	unsigned int file_position = 0;
	
	oldfs = get_fs();
	set_fs(KERNEL_DS);

	//read = sys_open((const char __user *)ERI_FILE_PATH, O_RDONLY , 0);
	phMscd_Filp = filp_open(ERI_FILE_PATH, O_RDONLY, 0);
	if(IS_ERR(phMscd_Filp)){
		printk(KERN_INFO "%s not found\n", ERI_FILE_PATH);
		file_position = 0;
	}
	else{
		phMscd_Filp->f_pos = 0;
		file_position = (unsigned int)phMscd_Filp->f_op->llseek(phMscd_Filp, phMscd_Filp->f_pos, SEEK_END);
		printk(KERN_INFO "%s size : %d\n", ERI_FILE_PATH, file_position);
		filp_close(phMscd_Filp,NULL);
	}	

	set_fs(oldfs);

	// prevent 0 byte eri file generation, check its size as well
	if(file_position <= 4){
		printk(KERN_INFO "%s, received flag : %ld, activate work queue\n", __func__, flag);
		eri_dload_data.flag = flag;
		queue_work(eri_dload_wq, &eri_dload_data.work);
	}
	else {
		printk(KERN_INFO "%s, already saved eri.bin\n",__func__);
	}

	//set_fs(oldfs);
	//sys_close(read);
	return 0;
}
module_param_call(eri_info, eri_write, param_get_int, &dummy_arg, S_IWUSR | S_IRUGO);
#endif

#ifdef CONFIG_LGE_KERNEL_ROOTING_NV_INTERFACE   //kabjoo.choi
extern void remote_rpc_rooting_nv_cmmand( char nv_data) ;

static int rooting_nv_write(const char *val, struct kernel_param *kp)
{
	unsigned long flag=0;
	
	printk(KERN_ERR "%s, rooting_nv_write=%d\n", __func__, (int)*val);
	
	if(val == NULL)
	{
		printk(KERN_ERR "%s, NULL buf\n", __func__);
		return -1;
	}

	if(*val)
	{
		printk(KERN_ERR "%s, rooting_nv_write1\n", __func__);	
		remote_rpc_rooting_nv_cmmand(1) ;
	}
	else
	{
		printk(KERN_ERR "%s, rooting_nv_write0\n", __func__);		
		remote_rpc_rooting_nv_cmmand(0) ;
	}
	
	return (int)flag;	 
}

static int rooting_nv_read(char *buf, struct kernel_param *kp)
{
	//memcpy(buf, integrity_ret.ret, 32);
	return 0;
}

module_param_call(rooting_nv, rooting_nv_write, rooting_nv_read, &dummy_arg, S_IWUSR | S_IRUGO);
#endif

#ifdef CONFIG_LGE_VOLD_SUPPORT_CRYPT
static int send_cryptfs_cmd(int cmd)
{
	int ret;
	char cmdstr[100];
	int fd;
	char *envp[] = {
		"HOME=/",
		"TERM=linux",
		NULL,
	};

	char *argv[] = {
		"sh",
		"-c",
		cmdstr,
		NULL,
	};	

	//                                      
	// 0001794: [ARM9] ATS AT CMD added 
	if ( (fd = sys_open((const char __user *) "/system/bin/vdc", O_RDONLY ,0) ) < 0 )
	{
		printk("\n can not open /system/bin/vdc - execute /system/bin/vdc cryptfs crypt_setup %d\n", cmd);
		sprintf(cmdstr, "/system/bin/vdc cryptfs crypt_setup %d\n", cmd);
	}
	else
	{
		printk("\n execute /system/bin/vdc cryptfs crypt_setup %d\n", cmd);
		sprintf(cmdstr, "/system/bin/vdc cryptfs crypt_setup %d\n", cmd);
		sys_close(fd);
	}
	//                                    

	printk(KERN_INFO "execute - %s", cmdstr);
	if ((ret = call_usermodehelper("/system/bin/sh", argv, envp, UMH_WAIT_PROC)) != 0) {
		printk(KERN_ERR "%s failed to run \": %i\n",__func__, ret);
	}
	else
		printk(KERN_INFO "%s execute ok\n", __func__);
	return ret;
}

static void cryptfs_cmd_func(struct work_struct *work)
{
	printk(KERN_INFO "%s, cmd : %ld\n", __func__, cryptfs_cmd_data.cmd);	
	send_cryptfs_cmd((int)cryptfs_cmd_data.cmd);
	return;
}

static int cryptfs_cmd_write(const char *val, struct kernel_param *kp)
{
	unsigned long cmd=0;
	
	if(val == NULL)
	{
		printk(KERN_ERR "%s, NULL buf\n", __func__);
		return -1;
	}
	
	cmd = simple_strtoul(val,NULL,10);

	// send the command to the workqueue and return this write command response asap
	// this will prevent ANR in the userspace call
	printk(KERN_INFO "%s, received cmd : %ld, activate work queue\n", __func__, cmd);
	cryptfs_cmd_data.cmd = cmd;
	queue_work(cryptfs_cmd_wq, &cryptfs_cmd_data.work);
	
	return 0;
}

module_param_call(cryptfs_cmd, cryptfs_cmd_write, NULL, NULL, S_IWUSR | S_IRUGO);
#endif

static char *lge_strdup(const char *str)
{
	size_t len;
	char *copy;
	
	len = strlen(str) + 1;
	copy = kmalloc(len, GFP_KERNEL);
	if (copy == NULL)
		return NULL;
	memcpy(copy, str, len);
	return copy;
}

int lge_erase_block(int bytes_pos, size_t erase_size)
{
	unsigned char *erasebuf;
	unsigned written = 0;
	erasebuf = kmalloc(erase_size, GFP_KERNEL);
	// allocation exception handling
	if(!erasebuf)
	{
		printk("%s, allocation failed, expected size : %d\n", __func__, erase_size);
		return 0;
	}
	memset(erasebuf, 0xff, erase_size);
	written += lge_write_block(bytes_pos, erasebuf, erase_size);

	kfree(erasebuf);

	return written;
}
EXPORT_SYMBOL(lge_erase_block);

/*                                            */
/* MOD 0014570: [FACTORY RESET] change system call to filp function for handling the flag */
int lge_write_block(unsigned int bytes_pos, unsigned char *buf, size_t size)
{
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;
	unsigned int write_bytes = 0;

	// exception handling
	if((buf == NULL) || size <= 0)
	{
		printk(KERN_ERR "%s, NULL buffer or NULL size : %d\n", __func__, size);
		return 0;
	}
		
	old_fs=get_fs();
	set_fs(get_ds());

	// change from sys operation to flip operation, do not use system call since this routine is also system call service.
	// set O_SYNC for synchronous file io
	phMscd_Filp = filp_open(MMC_DEVICENAME, O_RDWR | O_SYNC, 0);
	
	if (IS_ERR(phMscd_Filp)) {
		printk(KERN_ERR "%s, Can't open %s\n", __func__, MMC_DEVICENAME);
		goto open_fail;
	}
	
	if( !phMscd_Filp )
	{
		printk(KERN_ERR "%s, Can not access 0x%x bytes postition\n", __func__, bytes_pos );
		goto write_fail;
	}

	phMscd_Filp->f_pos = (loff_t)bytes_pos;
	write_bytes = phMscd_Filp->f_op->write(phMscd_Filp, buf, size, &phMscd_Filp->f_pos);

	if(write_bytes <= 0)
	{
		printk(KERN_ERR "%s, Can not write 0x%x bytes postition %d size \n", __func__, bytes_pos, size);
		goto write_fail;
	}

write_fail:
	if(phMscd_Filp != NULL)
		filp_close(phMscd_Filp,NULL);
open_fail:
	set_fs(old_fs); 
	return write_bytes;
	
}
/*                                         */

EXPORT_SYMBOL(lge_write_block);

/*                                            */
/* MOD 0014570: [FACTORY RESET] change system call to filp function for handling the flag */
int lge_read_block(unsigned int bytes_pos, unsigned char *buf, size_t size)
{
	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;
	unsigned int read_bytes = 0;

	// exception handling
	if((buf == NULL) || size <= 0)
	{
		printk(KERN_ERR "%s, NULL buffer or NULL size : %d\n", __func__, size);
		return 0;
	}
		
	old_fs=get_fs();
	set_fs(get_ds());

	// change from sys operation to flip operation, do not use system call since this routine is also system call service.
	phMscd_Filp = filp_open(MMC_DEVICENAME, O_RDONLY, 0);
	if (IS_ERR(phMscd_Filp)) {
		printk(KERN_ERR "%s, Can't open %s\n", __func__, MMC_DEVICENAME);
		goto open_fail;
	}
	
	if( !phMscd_Filp )
	{
		printk(KERN_ERR "%s, Can not access 0x%x bytes postition\n", __func__, bytes_pos );
		goto read_fail;
	}

	phMscd_Filp->f_pos = (loff_t)bytes_pos;
	read_bytes = phMscd_Filp->f_op->read(phMscd_Filp, buf, size, &phMscd_Filp->f_pos);

	if(read_bytes <= 0)
	{
		printk(KERN_ERR "%s, Can not read 0x%x bytes postition %d size \n", __func__, bytes_pos, size);
		goto read_fail;
	}

read_fail:
	if(phMscd_Filp != NULL)
		filp_close(phMscd_Filp,NULL);
open_fail:
	set_fs(old_fs); 
	return read_bytes;
}
/*                                         */
EXPORT_SYMBOL(lge_read_block);

const MmcPartition *lge_mmc_find_partition_by_name(const char *name)
{
    if (g_mmc_state.partitions != NULL) {
        int i;
        for (i = 0; i < g_mmc_state.partitions_allocd; i++) {
            MmcPartition *p = &g_mmc_state.partitions[i];
            if (p->device_index !=NULL && p->name != NULL) {
                if (strcmp(p->name, name) == 0) {
                    return p;
                }
            }
        }
    }
    return NULL;
}
EXPORT_SYMBOL(lge_mmc_find_partition_by_name);

void lge_mmc_print_partition_status(void)
{
    if (g_mmc_state.partitions != NULL) 
    {
        int i;
        for (i = 0; i < g_mmc_state.partitions_allocd; i++) 
        {
            MmcPartition *p = &g_mmc_state.partitions[i];
            if (p->device_index !=NULL && p->name != NULL) {
                printk(KERN_INFO"Partition Name: %s\n",p->name);
                printk(KERN_INFO"Partition Name: %s\n",p->device_index);
            }
        }
    }
    return;
}
EXPORT_SYMBOL(lge_mmc_print_partition_status);

static void lge_mmc_partition_name (MmcPartition *mbr, unsigned int type) {
	char *name;
	name = kmalloc(64, GFP_KERNEL);
    switch(type)
    {
		case MMC_MISC_TYPE:
            sprintf(name,"misc");
            mbr->name = lge_strdup(name);
			break;
		case MMC_RECOVERY_TYPE:
            sprintf(name,"recovery");
            mbr->name = lge_strdup(name);
			break;
		case MMC_XCALBACKUP_TYPE:
            sprintf(name,"xcalbackup");
            mbr->name = lge_strdup(name);
			break;
        case MMC_BOOT_TYPE:
            sprintf(name,"boot");
            mbr->name = lge_strdup(name);
            break;
        case MMC_EXT3_TYPE:
            if (strcmp("NONE", ext3_partitions[ext3_count])) {
                strcpy((char *)name,(const char *)ext3_partitions[ext3_count]);
                mbr->name = lge_strdup(name);
                ext3_count++;
            }
            mbr->filesystem = lge_strdup("ext3");
            break;
        case MMC_VFAT_TYPE:
            if (strcmp("NONE", vfat_partitions[vfat_count])) {
                strcpy((char *)name,(const char *)vfat_partitions[vfat_count]);
                mbr->name = lge_strdup(name);
                vfat_count++;
            }
            mbr->filesystem = lge_strdup("vfat");
            break;
    };
	kfree(name);
}

//static int lge_mmc_read_mbr (MmcPartition *mbr) {
/*                                            */
/* MOD 0014570: [FACTORY RESET] change system call to filp function for handling the flag */
int lge_mmc_read_mbr (MmcPartition *mbr) {
	//int fd;
	unsigned char *buffer = NULL;
	char *device_index = NULL;
	int idx, i;
	unsigned mmc_partition_count = 0;
	unsigned int dtype;
	unsigned int dfirstsec;
	unsigned int EBR_first_sec;
	unsigned int EBR_current_sec;
	int ret = -1;

	struct file *phMscd_Filp = NULL;
	mm_segment_t old_fs;

	old_fs=get_fs();
	set_fs(get_ds());

	buffer = kmalloc(512, GFP_KERNEL);
	device_index = kmalloc(128, GFP_KERNEL);
	if((buffer == NULL) || (device_index == NULL))
	{
		printk("%s, allocation failed\n", __func__);
		goto ERROR2;
	}

	// change from sys operation to flip operation, do not use system call since this routine is also system call service.
	phMscd_Filp = filp_open(MMC_DEVICENAME, O_RDONLY, 0);
	if (IS_ERR(phMscd_Filp)) {
		printk(KERN_ERR "%s, Can't open %s\n", __func__, MMC_DEVICENAME);
		goto ERROR2;
	}
	
	if( !phMscd_Filp )
	{
		printk(KERN_ERR "%s, Can't open device\n", __func__ );
		goto ERROR2;
	}

	phMscd_Filp->f_pos = (loff_t)0;
	if (phMscd_Filp->f_op->read(phMscd_Filp, buffer, 512, &phMscd_Filp->f_pos) != 512)
	{
		printk(KERN_ERR "%s, Can't read device: \"%s\"\n", __func__, MMC_DEVICENAME);
		goto ERROR1;
	}

	/* Check to see if signature exists */
	if ((buffer[TABLE_SIGNATURE] != 0x55) || \
		(buffer[TABLE_SIGNATURE + 1] != 0xAA))
	{
		printk(KERN_ERR "Incorrect mbr signatures!\n");
		goto ERROR1;
	}
	idx = TABLE_ENTRY_0;
	for (i = 0; i < 4; i++)
	{
		//char device_index[128];

		mbr[mmc_partition_count].dstatus = \
		            buffer[idx + i * TABLE_ENTRY_SIZE + OFFSET_STATUS];
		mbr[mmc_partition_count].dtype   = \
		            buffer[idx + i * TABLE_ENTRY_SIZE + OFFSET_TYPE];
		mbr[mmc_partition_count].dfirstsec = \
		            GET_LWORD_FROM_BYTE(&buffer[idx + \
		                                i * TABLE_ENTRY_SIZE + \
		                                OFFSET_FIRST_SEC]);
		mbr[mmc_partition_count].dsize  = \
		            GET_LWORD_FROM_BYTE(&buffer[idx + \
		                                i * TABLE_ENTRY_SIZE + \
		                                OFFSET_SIZE]);
		dtype  = mbr[mmc_partition_count].dtype;
		dfirstsec = mbr[mmc_partition_count].dfirstsec;
		lge_mmc_partition_name(&mbr[mmc_partition_count], \
		                mbr[mmc_partition_count].dtype);

		sprintf(device_index, "%sp%d", MMC_DEVICENAME, (mmc_partition_count+1));
		mbr[mmc_partition_count].device_index = lge_strdup(device_index);

		mmc_partition_count++;
		if (mmc_partition_count == MAX_PARTITIONS)
			goto SUCCESS;
	}

	/* See if the last partition is EBR, if not, parsing is done */
	if (dtype != 0x05)
	{
		goto SUCCESS;
	}

	EBR_first_sec = dfirstsec;
	EBR_current_sec = dfirstsec;

	phMscd_Filp->f_pos = (loff_t)(EBR_first_sec * 512);
	if (phMscd_Filp->f_op->read(phMscd_Filp, buffer, 512, &phMscd_Filp->f_pos) != 512)
	{
		printk(KERN_ERR "%s, Can't read device: \"%s\"\n", __func__, MMC_DEVICENAME);
		goto ERROR1;
	}

	/* Loop to parse the EBR */
	for (i = 0;; i++)
	{

		if ((buffer[TABLE_SIGNATURE] != 0x55) || (buffer[TABLE_SIGNATURE + 1] != 0xAA))
		{
		break;
		}
		mbr[mmc_partition_count].dstatus = \
                    buffer[TABLE_ENTRY_0 + OFFSET_STATUS];
		mbr[mmc_partition_count].dtype   = \
                    buffer[TABLE_ENTRY_0 + OFFSET_TYPE];
		mbr[mmc_partition_count].dfirstsec = \
                    GET_LWORD_FROM_BYTE(&buffer[TABLE_ENTRY_0 + \
                                        OFFSET_FIRST_SEC])    + \
                                        EBR_current_sec;
		mbr[mmc_partition_count].dsize = \
                    GET_LWORD_FROM_BYTE(&buffer[TABLE_ENTRY_0 + \
                                        OFFSET_SIZE]);
		lge_mmc_partition_name(&mbr[mmc_partition_count], \
                        mbr[mmc_partition_count].dtype);

		sprintf(device_index, "%sp%d", MMC_DEVICENAME, (mmc_partition_count+1));
		mbr[mmc_partition_count].device_index = lge_strdup(device_index);

		mmc_partition_count++;
		if (mmc_partition_count == MAX_PARTITIONS)
		goto SUCCESS;

		dfirstsec = GET_LWORD_FROM_BYTE(&buffer[TABLE_ENTRY_1 + OFFSET_FIRST_SEC]);
		if(dfirstsec == 0)
		{
			/* Getting to the end of the EBR tables */
			break;
		}
		
		 /* More EBR to follow - read in the next EBR sector */
		 phMscd_Filp->f_pos = (loff_t)((EBR_first_sec + dfirstsec) * 512);
		 if (phMscd_Filp->f_op->read(phMscd_Filp, buffer, 512, &phMscd_Filp->f_pos) != 512)
		 {
			 printk(KERN_ERR "%s, Can't read device: \"%s\"\n", __func__, MMC_DEVICENAME);
			 goto ERROR1;
		 }

		EBR_current_sec = EBR_first_sec + dfirstsec;
	}

SUCCESS:
    ret = mmc_partition_count;
ERROR1:
    if(phMscd_Filp != NULL)
		filp_close(phMscd_Filp,NULL);
ERROR2:
	set_fs(old_fs);
	if(buffer != NULL)
		kfree(buffer);
	if(device_index != NULL)
		kfree(device_index);
    return ret;
}
/*                                         */

static int lge_mmc_partition_initialied = 0;
int lge_mmc_scan_partitions(void) {
    int i;
    //ssize_t nbytes;

mutex_lock(&emmc_dir_lock);  //kabjoo.choi
	
	if ( lge_mmc_partition_initialied )
	{
		mutex_unlock(&emmc_dir_lock);	//kabjoo.choi
		return g_mmc_state.partition_count;
	}
	
    if (g_mmc_state.partitions == NULL) {
        const int nump = MAX_PARTITIONS;
        MmcPartition *partitions = kmalloc(nump * sizeof(*partitions), GFP_KERNEL);
        if (partitions == NULL) {
	      mutex_unlock(&emmc_dir_lock);	//kabjoo.choi
            return -1;
        }
        g_mmc_state.partitions = partitions;
        g_mmc_state.partitions_allocd = nump;
        memset(partitions, 0, nump * sizeof(*partitions));
    }
    g_mmc_state.partition_count = 0;
    ext3_count = 0;
    vfat_count = 0;

    /* Initialize all of the entries to make things easier later.
     * (Lets us handle sparsely-numbered partitions, which
     * may not even be possible.)
     */
    for (i = 0; i < g_mmc_state.partitions_allocd; i++) {
        MmcPartition *p = &g_mmc_state.partitions[i];
        if (p->device_index != NULL) {
            kfree(p->device_index);
            p->device_index = NULL;
        }
        if (p->name != NULL) {
            kfree(p->name);
            p->name = NULL;
        }
        if (p->filesystem != NULL) {
            kfree(p->filesystem);
            p->filesystem = NULL;
        }
    }

    g_mmc_state.partition_count = lge_mmc_read_mbr(g_mmc_state.partitions);
    if(g_mmc_state.partition_count == -1)
    {
        printk(KERN_ERR"Error in reading mbr!\n");
        // keep "partitions" around so we can free the names on a rescan.
        g_mmc_state.partition_count = -1;
    }
	if ( g_mmc_state.partition_count != -1 )
		lge_mmc_partition_initialied = 1;

    mutex_unlock(&emmc_dir_lock);	//kabjoo.choi
    return g_mmc_state.partition_count;
}

EXPORT_SYMBOL(lge_mmc_scan_partitions);

#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
//                                                                    
static int write_status_power(const char *val)
{
	int h_file = 0;
	int ret = 0;

	int offset = (int)PTN_POWER_POSITION_IN_MISC_PARTITION;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	h_file = sys_open(MMC_DEVICENAME_MISC, O_RDWR | O_SYNC,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_write( h_file, val, 1);

		if( ret != 1 )
		{
			printk("Can't write in MISC partition.\n");
			return ret;
		}

		sys_close(h_file);
	}
	else
	{
		printk("Can't open MISC partition handle = %d.\n",h_file);
		return 0;
	}
	set_fs(old_fs);

	return 1;

	
}

int read_status_power(void)
{
	int h_file = 0;
	int ret = 0;
	char buf;

	int offset = (int)PTN_POWER_POSITION_IN_MISC_PARTITION;

	mm_segment_t old_fs = get_fs();
	set_fs(KERNEL_DS);
	h_file = sys_open(MMC_DEVICENAME_MISC, O_RDWR | O_SYNC,0);

	if(h_file >= 0)
	{
		sys_lseek( h_file, offset, 0 );

		ret = sys_read( h_file, &buf, 1);

		if( ret != 1 )
		{
			printk("Can't read MISC partition.\n");
			return ret;
		}

		sys_close(h_file);
	}
	else
	{
		printk("Can't open MISC partition handle = %d.\n",h_file);
		return 0;
	}
	set_fs(old_fs);

	return (int)buf;
}

int set_status_power(char val)
{	
	int ret;
	char *stats;

	stats = kmalloc(1, GFP_KERNEL);
	*stats = val;

	printk("set_status_power :%d\n",*stats);

	ret = write_status_power(stats);
	if (ret != 1) {
		printk("%s : write val has failed!!\n", __func__);
		}
	kfree(stats);
	return 0;
		
}
EXPORT_SYMBOL(set_status_power);

int get_status_power(void)
{	
	char ret;
	char *stats;

	stats = kmalloc(1, GFP_KERNEL);

	ret = read_status_power();
	printk("get_status_power : %d \n",ret);

	kfree(stats);

	return ret;
		
}
EXPORT_SYMBOL(get_status_power);
//                                                                  
#endif

//[START]kcal for 325
static int write_lcd_k_cal(const char *val, struct kernel_param *kp)
{

	int i = 0;
	int err = 0; //                                            
	int mtd_op_result = 0;
	int mmc_scan_partion_result = 0;
	const MmcPartition *pMisc_part;
	unsigned long lcdkcal_bytes_pos_in_emmc = 0;
	static unsigned char lcd_buf2[6];

	memcpy(lcd_buf,val,LCD_K_CAL_SIZE);

#if 1
	for(i=0;i<LCD_K_CAL_SIZE;i++)
	{
		printk("write_lcd_k_cal :%x:\n",lcd_buf[i]);
	}
#endif

	mmc_scan_partion_result = lge_mmc_scan_partitions();
	if (mmc_scan_partion_result < 0)
	{
		printk(KERN_INFO"mmc_scan_fail\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_INFO"NO MISC\n");
		return 0;
	}

	lcdkcal_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_LCD_K_CAL_PARTITION;

	printk("write_lcd_k_cal %ld block\n", lcdkcal_bytes_pos_in_emmc);

	mtd_op_result = lge_write_block(lcdkcal_bytes_pos_in_emmc, lcd_buf, LCD_K_CAL_SIZE);

	if ( mtd_op_result != LCD_K_CAL_SIZE ) {
		printk("%s: write %u block fail\n", __func__, i);
		return err;
	}
	mtd_op_result = lge_read_block(lcdkcal_bytes_pos_in_emmc, &lcd_buf2[0], LCD_K_CAL_SIZE);
	if ( mtd_op_result != LCD_K_CAL_SIZE ) {
		printk("%s: write %u block fail\n", __func__, i);
		return err;
	}
#if 1
	for(i=0;i<LCD_K_CAL_SIZE;i++)
	{
		printk("read_lcd_k_cal :%x:\n",lcd_buf2[i]);
	}
#endif
	printk("write %d block\n", i);
	return 0;
}


int read_lcd_k_cal( char *buf)
{
	int err=0;
	int mtd_op_result = 0;
	int mmc_scan_partion_result = 0;
	//int i;

	const MmcPartition *pMisc_part;
	unsigned long lcdkcal_bytes_pos_in_emmc = 0;

	printk(KERN_INFO"read read_lcd_k_cal\n");

	mmc_scan_partion_result = lge_mmc_scan_partitions();
	if (mmc_scan_partion_result < 0)
	{
		printk(KERN_INFO"mmc_scan_fail\n");
		return 0;
	}
	pMisc_part = lge_mmc_find_partition_by_name("misc");

	if ( pMisc_part == NULL )
	{
		printk(KERN_INFO"NO MISC\n");
		return 0;
	}

	lcdkcal_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_LCD_K_CAL_PARTITION;

	memset(lcd_buf, 0 ,LCD_K_CAL_SIZE);
	mtd_op_result = lge_read_block(lcdkcal_bytes_pos_in_emmc, &lcd_buf[0], LCD_K_CAL_SIZE);

	if (mtd_op_result != LCD_K_CAL_SIZE ) {
		printk(KERN_INFO" read %ld block fail\n", lcdkcal_bytes_pos_in_emmc);
		return err;
	}

	printk(KERN_INFO"read %ld block\n", lcdkcal_bytes_pos_in_emmc);
	memcpy(&buf[0],&lcd_buf[0],LCD_K_CAL_SIZE);

	return LCD_K_CAL_SIZE;
}
EXPORT_SYMBOL(read_lcd_k_cal);
//[END]kcal for 325

/*                                            */
/* MOD 0013861: [FACTORY RESET] emmc_direct_access factory reset flag access */
/* add carriage return and change flag size in each functions for the platform access */
/*                                          */
static int test_write_block(const char *val, struct kernel_param *kp)
{
	int i = 0;
	//                               
	int err = 0;
	//int normal_block_seq = 0;
	int mtd_op_result = 0;
	const MmcPartition *pMisc_part;
	unsigned long factoryreset_bytes_pos_in_emmc = 0;
	unsigned long flag=0;

	unsigned char *test_string;

	test_string = kmalloc(FACTORY_RESET_STR_SIZE+2, GFP_KERNEL);
	// allocation exception handling
	if(!test_string)
	{
		printk(KERN_ERR "allocation failed, return\n");
		return 0;
	}

	printk(KERN_INFO"write block1\n");

	flag = simple_strtoul(val,NULL,10);
	sprintf(test_string,"FACT_RESET_%d\n", (char)flag);

	lge_mmc_scan_partitions();
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_INFO"NO MISC\n");
		return 0;
	}

	factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;

	printk(KERN_INFO"writing block\n");

	mtd_op_result = lge_write_block(factoryreset_bytes_pos_in_emmc, test_string, FACTORY_RESET_STR_SIZE+2);
	if ( mtd_op_result != (FACTORY_RESET_STR_SIZE+2) ) {
		printk(KERN_INFO"%s: write %u block fail\n", __func__, i);
		kfree(test_string);
		return err;
	}

/*                                            */
/* ADD 0013860: [FACTORY RESET] ERI file save */
/* request rpc for eri file when the factory reset completes */
/* migrate eri codes to qem daemon */
#if 0 //                              
	if (flag == 5)
	{
		printk(KERN_INFO "%s, received flag : %ld, activate work queue\n", __func__, flag);
		eri_dload_data.flag = flag;
		queue_work(eri_dload_wq, &eri_dload_data.work);
	}
#endif
/*                                          */

	printk(KERN_INFO"write %d block\n", i);
	kfree(test_string);
	return 0;
}

static int test_read_block( char *buf, struct kernel_param *kp)
{
	int err=0;
	int mtd_op_result = 0;

	const MmcPartition *pMisc_part;
	unsigned long factoryreset_bytes_pos_in_emmc = 0;

	printk(KERN_INFO"read block1\n");

	lge_mmc_scan_partitions();

	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_INFO"NO MISC\n");
		return 0;
	}
	factoryreset_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+PTN_FRST_PERSIST_POSITION_IN_MISC_PARTITION;

	printk(KERN_INFO"read block\n");
	memset(global_buf, 0 , FACTORY_RESET_STR_SIZE+2);

	mtd_op_result = lge_read_block(factoryreset_bytes_pos_in_emmc, global_buf, FACTORY_RESET_STR_SIZE+2);

	if (mtd_op_result != (FACTORY_RESET_STR_SIZE+2) ) {
		printk(KERN_INFO" read %ld block fail\n", factoryreset_bytes_pos_in_emmc);
		return err;
	}

	printk(KERN_INFO"read %ld block\n", factoryreset_bytes_pos_in_emmc);

//                                              
// MOD 0009484: [FactoryReset] Enable FactoryReset
	if(memcmp(global_buf, FACTORY_RESET_STR, FACTORY_RESET_STR_SIZE)==0){
		err = sprintf(buf,"%s",global_buf+FACTORY_RESET_STR_SIZE);
		return err;
	}
	else{
		err = sprintf(buf,"1");
		return err;
	}
//                                            
}

module_param_call(frst_flag, test_write_block, test_read_block, &dummy_arg, S_IWUSR | S_IRUGO);
#if defined(CONFIG_MACH_LGE_120_BOARD) || defined(CONFIG_MACH_LGE_IJB_BOARD_LGU) || defined(CONFIG_MACH_LGE_IJB_BOARD_SKT)
module_param_call(lcd_k_cal, write_lcd_k_cal, NULL, NULL,S_IWUSR|S_IRUSR|S_IRGRP|S_IWGRP);	//daheui.kim for kcal
#else
module_param_call(lcd_k_cal, write_lcd_k_cal, NULL, NULL,S_IWUSR | S_IRUSR);	//kcal for 325
#endif
#ifdef CONFIG_LGE_DID_BACKUP
extern void remote_did_rpc(void);

static void did_dload_func(struct work_struct *work)
{
	printk(KERN_INFO "%s, flag : %ld\n", __func__, did_dload_data.flag);	
#ifdef CONFIG_LGE_SUPPORT_RAPI 
	remote_did_rpc();
#endif
	return;
}
#endif

#ifdef CONFIG_LGE_ERI_DOWNLOAD
static void eri_dload_func(struct work_struct *work)
{
	printk(KERN_INFO "%s, flag : %ld\n", __func__, eri_dload_data.flag);	
#ifdef CONFIG_LGE_SUPPORT_RAPI
	remote_eri_rpc();
#endif
	return;
}
#endif

#if defined(CONFIG_MACH_LGE_I_BOARD_VZW) || defined(CONFIG_MACH_LGE_325_BOARD_VZW)
/*                                  */
/* ADD 0013860: [FOTA] bootcmd save */
#define BOOTCMD_FLAG_OFFSET_IN_BYTES 0x780000  //7.5MB 
struct bootloader_message {
	char command[32];
	char status[32];
	char recovery[128]; //ys.seong reduce size for avoid stackoverflow from original size
};

static int test_bootcmd_write_block(const char *val, struct kernel_param *kp)
{

	int i = 0;
	int err = 0; /* mbhyun.kim 2013.01.22 : WBT ID 100709 */
	int mtd_op_result = 0;
	const MmcPartition *pMisc_part; 
	unsigned long bootcmd_bytes_pos_in_emmc = 0;
	struct bootloader_message boot;

	memset(&boot, 0, sizeof(boot));

	strlcpy(boot.command, "boot-recovery", sizeof(boot.command));
	strlcpy(boot.recovery, "recovery\n", sizeof(boot.recovery));
	
	printk(KERN_INFO"bootcmd write block\n");
	
	lge_mmc_scan_partitions();
	pMisc_part = lge_mmc_find_partition_by_name("misc");
	if ( pMisc_part == NULL )
	{
		printk(KERN_INFO"NO MISC\n");
		return 0;
	}

	bootcmd_bytes_pos_in_emmc = (pMisc_part->dfirstsec*512)+BOOTCMD_FLAG_OFFSET_IN_BYTES;

	printk(KERN_INFO"bootcmd writing block\n");

	mtd_op_result = lge_write_block(bootcmd_bytes_pos_in_emmc, (void *)&boot, sizeof(boot));
	if ( mtd_op_result != sizeof(boot) ) {
		printk(KERN_INFO"bootcmd %s: write %u block fail\n", __func__, i);
		return err;
	}

	printk(KERN_INFO"bootcmd write %d block\n", i);
	return 0;
}

module_param_call(bootcmd_write_block, test_bootcmd_write_block, param_get_bool, &dummy_arg, S_IWUSR | S_IRUGO);
/*                                 */
#endif

static int __init lge_emmc_direct_access_init(void)
{
	printk(KERN_INFO"%s: finished\n", __func__);

/*                                            */
/* ADD 0013860: [FACTORY RESET] ERI file save */
#ifdef CONFIG_LGE_ERI_DOWNLOAD
	eri_dload_wq = create_singlethread_workqueue("eri_dload_wq");
	INIT_WORK(&eri_dload_data.work, eri_dload_func);
#endif
/*                                          */

#ifdef CONFIG_LGE_DID_BACKUP
	did_dload_wq = create_singlethread_workqueue("did_dload_wq");
	INIT_WORK(&did_dload_data.work, did_dload_func);
#endif

#ifdef CONFIG_LGE_VOLD_SUPPORT_CRYPT
	cryptfs_cmd_wq = create_singlethread_workqueue("cryptfs_cmd_wq");
	INIT_WORK(&cryptfs_cmd_data.work, cryptfs_cmd_func);
#endif

	return 0;
}

static void __exit lge_emmc_direct_access_exit(void)
{
	return;
}

module_init(lge_emmc_direct_access_init);
module_exit(lge_emmc_direct_access_exit);

MODULE_DESCRIPTION("LGE emmc direct access apis");
MODULE_AUTHOR("SeHyun Kim <sehyuny.kim@lge.com>");
MODULE_LICENSE("GPL");
