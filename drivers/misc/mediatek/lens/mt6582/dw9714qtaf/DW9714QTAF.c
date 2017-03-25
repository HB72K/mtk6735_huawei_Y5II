/*
 * MD218A voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include "DW9714QTAF.h"
#include "../camera/kd_camera_hw.h"

#define LENS_I2C_BUSNUM 1
static struct i2c_board_info kd_lens_dev __initdata = { I2C_BOARD_INFO("DW9714QTAF", 0x20) };


#define DW9714QTAF_DRVNAME "DW9714QTAF"
#define DW9714QTAF_VCM_WRITE_ID           0x18

#define DW9714QTAF_DEBUG
#ifdef DW9714QTAF_DEBUG
#define DW9714QTAFDB pr_debug
#else
#define DW9714QTAFDB(x, ...)
#endif

static spinlock_t g_DW9714QTAF_SpinLock;

static struct i2c_client *g_pstDW9714QTAF_I2Cclient;

static dev_t g_DW9714QTAF_devno;
static struct cdev *g_pDW9714QTAF_CharDrv;
static struct class *actuator_class;

static int g_s4DW9714QTAF_Opened;
static long g_i4MotorStatus;
static long g_i4Dir;
static unsigned long g_u4DW9714QTAF_INF;
static unsigned long g_u4DW9714QTAF_MACRO = 1023;
static unsigned long g_u4TargetPosition;
static unsigned long g_u4CurrPosition;

static int g_sr = 3;

#if 0
extern s32 mt_set_gpio_mode(u32 u4Pin, u32 u4Mode);
extern s32 mt_set_gpio_out(u32 u4Pin, u32 u4PinOut);
extern s32 mt_set_gpio_dir(u32 u4Pin, u32 u4Dir);
#endif

static int s4DW9714QTAF_ReadReg(unsigned short *a_pu2Result)
{
	int i4RetValue = 0;
	char pBuff[2];

	i4RetValue = i2c_master_recv(g_pstDW9714QTAF_I2Cclient, pBuff, 2);

	if (i4RetValue < 0) {
		DW9714QTAFDB("[DW9714QTAF] I2C read failed!!\n");
		return -1;
	}

	*a_pu2Result = (((u16) pBuff[0]) << 4) + (pBuff[1] >> 4);

	return 0;
}

static int s4DW9714QTAF_WriteReg(u16 a_u2Data)
{
	int i4RetValue = 0;

	char puSendCmd[2] = { (char)(a_u2Data >> 4), (char)(((a_u2Data & 0xF) << 4) + g_sr) };

	/* DW9714QTAFDB("[DW9714QTAF] g_sr %d, write %d\n", g_sr, a_u2Data); */
	g_pstDW9714QTAF_I2Cclient->ext_flag |= I2C_A_FILTER_MSG;
	i4RetValue = i2c_master_send(g_pstDW9714QTAF_I2Cclient, puSendCmd, 2);

	if (i4RetValue < 0) {
		DW9714QTAFDB("[DW9714QTAF] I2C send failed!!\n");
		return -1;
	}

	return 0;
}

inline static int getDW9714QTAFInfo(__user stDW9714QTAF_MotorInfo * pstMotorInfo)
{
	stDW9714QTAF_MotorInfo stMotorInfo;
	stMotorInfo.u4MacroPosition = g_u4DW9714QTAF_MACRO;
	stMotorInfo.u4InfPosition = g_u4DW9714QTAF_INF;
	stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
	stMotorInfo.bIsSupportSR = TRUE;

	if (g_i4MotorStatus == 1) {
		stMotorInfo.bIsMotorMoving = 1;
	} else {
		stMotorInfo.bIsMotorMoving = 0;
	}

	if (g_s4DW9714QTAF_Opened >= 1) {
		stMotorInfo.bIsMotorOpen = 1;
	} else {
		stMotorInfo.bIsMotorOpen = 0;
	}

	if (copy_to_user(pstMotorInfo, &stMotorInfo, sizeof(stDW9714QTAF_MotorInfo))) {
		DW9714QTAFDB("[DW9714QTAF] copy to user failed when getting motor information\n");
	}

	return 0;
}

#ifdef LensdrvCM3
inline static int getDW9714QTAFMETA(__user stDW9714QTAF_MotorMETAInfo * pstMotorMETAInfo)
{
	stDW9714QTAF_MotorMETAInfo stMotorMETAInfo;
	stMotorMETAInfo.Aperture = 2.8;	/* fn */
	stMotorMETAInfo.Facing = 1;
	stMotorMETAInfo.FilterDensity = 1;	/* X */
	stMotorMETAInfo.FocalDistance = 1.0;	/* diopters */
	stMotorMETAInfo.FocalLength = 34.0;	/* mm */
	stMotorMETAInfo.FocusRange = 1.0;	/* diopters */
	stMotorMETAInfo.InfoAvalibleApertures = 2.8;
	stMotorMETAInfo.InfoAvalibleFilterDensity = 1;
	stMotorMETAInfo.InfoAvalibleFocalLength = 34.0;
	stMotorMETAInfo.InfoAvalibleHypeDistance = 1.0;
	stMotorMETAInfo.InfoAvalibleMinFocusDistance = 1.0;
	stMotorMETAInfo.InfoAvalibleOptStabilization = 0;
	stMotorMETAInfo.OpticalAxisAng[0] = 0.0;
	stMotorMETAInfo.OpticalAxisAng[1] = 0.0;
	stMotorMETAInfo.Position[0] = 0.0;
	stMotorMETAInfo.Position[1] = 0.0;
	stMotorMETAInfo.Position[2] = 0.0;
	stMotorMETAInfo.State = 0;
	stMotorMETAInfo.u4OIS_Mode = 0;

	if (copy_to_user(pstMotorMETAInfo, &stMotorMETAInfo, sizeof(stDW9714QTAF_MotorMETAInfo))) {
		DW9714QTAFDB("[DW9714QTAF] copy to user failed when getting motor information\n");
	}

	return 0;
}
#endif

inline static int moveDW9714QTAF(unsigned long a_u4Position)
{
	int ret = 0;

	if ((a_u4Position > g_u4DW9714QTAF_MACRO) || (a_u4Position < g_u4DW9714QTAF_INF)) {
		DW9714QTAFDB("[DW9714QTAF] out of range\n");
		return -EINVAL;
	}

	if (g_s4DW9714QTAF_Opened == 1) {
		unsigned short InitPos;
		ret = s4DW9714QTAF_ReadReg(&InitPos);

		if (ret == 0) {
			DW9714QTAFDB("[DW9714QTAF] Init Pos %6d\n", InitPos);

			spin_lock(&g_DW9714QTAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(&g_DW9714QTAF_SpinLock);

		} else {
			spin_lock(&g_DW9714QTAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(&g_DW9714QTAF_SpinLock);
		}

		spin_lock(&g_DW9714QTAF_SpinLock);
		g_s4DW9714QTAF_Opened = 2;
		spin_unlock(&g_DW9714QTAF_SpinLock);
	}

	if (g_u4CurrPosition < a_u4Position) {
		spin_lock(&g_DW9714QTAF_SpinLock);
		g_i4Dir = 1;
		spin_unlock(&g_DW9714QTAF_SpinLock);
	} else if (g_u4CurrPosition > a_u4Position) {
		spin_lock(&g_DW9714QTAF_SpinLock);
		g_i4Dir = -1;
		spin_unlock(&g_DW9714QTAF_SpinLock);
	} else {
		return 0;
	}

	spin_lock(&g_DW9714QTAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(&g_DW9714QTAF_SpinLock);

	/* DW9714QTAFDB("[DW9714QTAF] move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition); */

	spin_lock(&g_DW9714QTAF_SpinLock);
	g_sr = 3;
	g_i4MotorStatus = 0;
	spin_unlock(&g_DW9714QTAF_SpinLock);

	if (s4DW9714QTAF_WriteReg((unsigned short)g_u4TargetPosition) == 0) {
		spin_lock(&g_DW9714QTAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(&g_DW9714QTAF_SpinLock);
	} else {
		DW9714QTAFDB("[DW9714QTAF] set I2C failed when moving the motor\n");
		spin_lock(&g_DW9714QTAF_SpinLock);
		g_i4MotorStatus = -1;
		spin_unlock(&g_DW9714QTAF_SpinLock);
	}

	return 0;
}

inline static int setDW9714QTAFInf(unsigned long a_u4Position)
{
	spin_lock(&g_DW9714QTAF_SpinLock);
	g_u4DW9714QTAF_INF = a_u4Position;
	spin_unlock(&g_DW9714QTAF_SpinLock);
	return 0;
}

inline static int setDW9714QTAFMacro(unsigned long a_u4Position)
{
	spin_lock(&g_DW9714QTAF_SpinLock);
	g_u4DW9714QTAF_MACRO = a_u4Position;
	spin_unlock(&g_DW9714QTAF_SpinLock);
	return 0;
}

/* ////////////////////////////////////////////////////////////// */
static long DW9714QTAF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;

	switch (a_u4Command) {
	case DW9714QTAFIOC_G_MOTORINFO:
		i4RetValue = getDW9714QTAFInfo((__user stDW9714QTAF_MotorInfo *) (a_u4Param));
		break;
#ifdef LensdrvCM3
	case DW9714QTAFIOC_G_MOTORMETAINFO:
		i4RetValue = getDW9714QTAFMETA((__user stDW9714QTAF_MotorMETAInfo *) (a_u4Param));
		break;
#endif
	case DW9714QTAFIOC_T_MOVETO:
		i4RetValue = moveDW9714QTAF(a_u4Param);
		break;

	case DW9714QTAFIOC_T_SETINFPOS:
		i4RetValue = setDW9714QTAFInf(a_u4Param);
		break;

	case DW9714QTAFIOC_T_SETMACROPOS:
		i4RetValue = setDW9714QTAFMacro(a_u4Param);
		break;

	default:
		DW9714QTAFDB("[DW9714QTAF] No CMD\n");
		i4RetValue = -EPERM;
		break;
	}

	return i4RetValue;
}

/* Main jobs: */
/* 1.check for device-specified errors, device not ready. */
/* 2.Initialize the device if it is opened for the first time. */
/* 3.Update f_op pointer. */
/* 4.Fill data structures into private_data */
/* CAM_RESET */
static int DW9714QTAF_Open(struct inode *a_pstInode, struct file *a_pstFile)
{
	DW9714QTAFDB("[DW9714QTAF] DW9714QTAF_Open - Start\n");
 long i4RetValue = 0;
     char puSendCmd1[2] = {0xEC , 0xA3};
     char puSendCmd2[2] = {0xA1 , 0x0D};
     char puSendCmd3[2] = {0xF2 , 0x30};
     char puSendCmd4[2] = {0xDC , 0x51};

	if (g_s4DW9714QTAF_Opened) {
		DW9714QTAFDB("[DW9714QTAF] the device is opened\n");
		return -EBUSY;
	}

	spin_lock(&g_DW9714QTAF_SpinLock);
	g_s4DW9714QTAF_Opened = 1;
	spin_unlock(&g_DW9714QTAF_SpinLock);
 #if 1
     i4RetValue = i2c_master_send(g_pstDW9714QTAF_I2Cclient, puSendCmd1, 2);
     i4RetValue = i2c_master_send(g_pstDW9714QTAF_I2Cclient, puSendCmd2, 2);
     i4RetValue = i2c_master_send(g_pstDW9714QTAF_I2Cclient, puSendCmd3, 2);
     i4RetValue = i2c_master_send(g_pstDW9714QTAF_I2Cclient, puSendCmd4, 2);
 #endif
	DW9714QTAFDB("[DW9714QTAF] DW9714QTAF_Open - End\n");

	return 0;
}

/* Main jobs: */
/* 1.Deallocate anything that "open" allocated in private_data. */
/* 2.Shut down the device on last close. */
/* 3.Only called once on last time. */
/* Q1 : Try release multiple times. */
static int DW9714QTAF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
	DW9714QTAFDB("[DW9714QTAF] DW9714QTAF_Release - Start\n");

	if (g_s4DW9714QTAF_Opened == 2) {
		g_sr = 5;
		s4DW9714QTAF_WriteReg(300);
		msleep(10);
		s4DW9714QTAF_WriteReg(200);
		msleep(10);
		s4DW9714QTAF_WriteReg(100);
		msleep(10);
	}

	if (g_s4DW9714QTAF_Opened) {
		DW9714QTAFDB("[DW9714QTAF] feee\n");

		spin_lock(&g_DW9714QTAF_SpinLock);
		g_s4DW9714QTAF_Opened = 0;
		spin_unlock(&g_DW9714QTAF_SpinLock);
	}    

	DW9714QTAFDB("[DW9714QTAF] DW9714QTAF_Release - End\n");

	return 0;
}

static const struct file_operations g_stDW9714QTAF_fops = {
	.owner = THIS_MODULE,
	.open = DW9714QTAF_Open,
	.release = DW9714QTAF_Release,
	.unlocked_ioctl = DW9714QTAF_Ioctl
};

inline static int Register_DW9714QTAF_CharDrv(void)
{
	struct device *vcm_device = NULL;

	DW9714QTAFDB("[DW9714QTAF] Register_DW9714QTAF_CharDrv - Start\n");

	/* Allocate char driver no. */
	if (alloc_chrdev_region(&g_DW9714QTAF_devno, 0, 1, DW9714QTAF_DRVNAME)) {
		DW9714QTAFDB("[DW9714QTAF] Allocate device no failed\n");

		return -EAGAIN;
	}
	/* Allocate driver */
	g_pDW9714QTAF_CharDrv = cdev_alloc();

	if (NULL == g_pDW9714QTAF_CharDrv) {
		unregister_chrdev_region(g_DW9714QTAF_devno, 1);

		DW9714QTAFDB("[DW9714QTAF] Allocate mem for kobject failed\n");

		return -ENOMEM;
	}
	/* Attatch file operation. */
	cdev_init(g_pDW9714QTAF_CharDrv, &g_stDW9714QTAF_fops);

	g_pDW9714QTAF_CharDrv->owner = THIS_MODULE;

	/* Add to system */
	if (cdev_add(g_pDW9714QTAF_CharDrv, g_DW9714QTAF_devno, 1)) {
		DW9714QTAFDB("[DW9714QTAF] Attatch file operation failed\n");

		unregister_chrdev_region(g_DW9714QTAF_devno, 1);

		return -EAGAIN;
	}

	actuator_class = class_create(THIS_MODULE, "actuatordrv");
	if (IS_ERR(actuator_class)) {
		int ret = PTR_ERR(actuator_class);
		DW9714QTAFDB("Unable to create class, err = %d\n", ret);
		return ret;
	}

	vcm_device = device_create(actuator_class, NULL, g_DW9714QTAF_devno, NULL, DW9714QTAF_DRVNAME);

	if (NULL == vcm_device) {
		return -EIO;
	}

	DW9714QTAFDB("[DW9714QTAF] Register_DW9714QTAF_CharDrv - End\n");
	return 0;
}

inline static void Unregister_DW9714QTAF_CharDrv(void)
{
	DW9714QTAFDB("[DW9714QTAF] Unregister_DW9714QTAF_CharDrv - Start\n");

	/* Release char driver */
	cdev_del(g_pDW9714QTAF_CharDrv);

	unregister_chrdev_region(g_DW9714QTAF_devno, 1);

	device_destroy(actuator_class, g_DW9714QTAF_devno);

	class_destroy(actuator_class);

	DW9714QTAFDB("[DW9714QTAF] Unregister_DW9714QTAF_CharDrv - End\n");
}

/* //////////////////////////////////////////////////////////////////// */

static int DW9714QTAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int DW9714QTAF_i2c_remove(struct i2c_client *client);
static const struct i2c_device_id DW9714QTAF_i2c_id[] = { {DW9714QTAF_DRVNAME, 0}, {} };

struct i2c_driver DW9714QTAF_i2c_driver = {
	.probe = DW9714QTAF_i2c_probe,
	.remove = DW9714QTAF_i2c_remove,
	.driver.name = DW9714QTAF_DRVNAME,
	.id_table = DW9714QTAF_i2c_id,
};

#if 0
static int DW9714QTAF_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info)
{
	strcpy(info->type, DW9714QTAF_DRVNAME);
	return 0;
}
#endif
static int DW9714QTAF_i2c_remove(struct i2c_client *client)
{
	return 0;
}

/* Kirby: add new-style driver {*/
static int DW9714QTAF_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int i4RetValue = 0;

	DW9714QTAFDB("[DW9714QTAF] DW9714QTAF_i2c_probe\n");

	/* Kirby: add new-style driver { */
	g_pstDW9714QTAF_I2Cclient = client;
	g_pstDW9714QTAF_I2Cclient->addr = 0x18;
	g_pstDW9714QTAF_I2Cclient->addr = g_pstDW9714QTAF_I2Cclient->addr >> 1;

	/* Register char driver */
	i4RetValue = Register_DW9714QTAF_CharDrv();

	if (i4RetValue) {

		DW9714QTAFDB("[DW9714QTAF] register char device failed!\n");

		return i4RetValue;
	}

	spin_lock_init(&g_DW9714QTAF_SpinLock);

	DW9714QTAFDB("[DW9714QTAF] Attached!!\n");

	return 0;
}

static int DW9714QTAF_probe(struct platform_device *pdev)
{
	return i2c_add_driver(&DW9714QTAF_i2c_driver);
}

static int DW9714QTAF_remove(struct platform_device *pdev)
{
	i2c_del_driver(&DW9714QTAF_i2c_driver);
	return 0;
}

static int DW9714QTAF_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	return 0;
}

static int DW9714QTAF_resume(struct platform_device *pdev)
{
	return 0;
}

/* platform structure */
static struct platform_driver g_stDW9714QTAF_Driver = {
	.probe = DW9714QTAF_probe,
	.remove = DW9714QTAF_remove,
	.suspend = DW9714QTAF_suspend,
	.resume = DW9714QTAF_resume,
	.driver = {
		   .name = "lens_actuator",
		   .owner = THIS_MODULE,
		   }
};

static int __init DW9714QTAF_i2C_init(void)
{
	i2c_register_board_info(LENS_I2C_BUSNUM, &kd_lens_dev, 1);

	if (platform_driver_register(&g_stDW9714QTAF_Driver)) {
		DW9714QTAFDB("failed to register DW9714QTAF driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit DW9714QTAF_i2C_exit(void)
{
	platform_driver_unregister(&g_stDW9714QTAF_Driver);
}
module_init(DW9714QTAF_i2C_init);
module_exit(DW9714QTAF_i2C_exit);

MODULE_DESCRIPTION("DW9714QTAF lens module driver");
MODULE_AUTHOR("KY Chen <ky.chen@Mediatek.com>");
MODULE_LICENSE("GPL");
