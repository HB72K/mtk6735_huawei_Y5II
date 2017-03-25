#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_hw.h"
#include <cust_gpio_usage.h>
//#include <cust_i2c.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/xlog.h>
#include <linux/version.h>

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
#include <linux/mutex.h>
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
#include <linux/semaphore.h>
#else
#include <asm/semaphore.h>
#endif
#endif

#include <linux/i2c.h>
#include <linux/leds.h>

#include <mach/mt_gpio.h>		// For gpio control

/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10       11    12       13      14       15    16
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5  1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
// availible parameter
// ANDROID_LOG_ASSERT
// ANDROID_LOG_ERROR
// ANDROID_LOG_WARNING
// ANDROID_LOG_INFO
// ANDROID_LOG_DEBUG
// ANDROID_LOG_VERBOSE
#define TAG_NAME "leds_strobe.c"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_WARN(fmt, arg...)        xlog_printk(ANDROID_LOG_WARNING, TAG_NAME, KERN_WARNING  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_NOTICE(fmt, arg...)      xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME, KERN_NOTICE  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_INFO(fmt, arg...)        xlog_printk(ANDROID_LOG_INFO   , TAG_NAME, KERN_INFO  "%s: " fmt, __FUNCTION__ ,##arg)
#define PK_TRC_FUNC(f)              xlog_printk(ANDROID_LOG_DEBUG  , TAG_NAME,  "<%s>\n", __FUNCTION__);
#define PK_TRC_VERBOSE(fmt, arg...) xlog_printk(ANDROID_LOG_VERBOSE, TAG_NAME,  fmt, ##arg)
#define PK_ERROR(fmt, arg...)       xlog_printk(ANDROID_LOG_ERROR  , TAG_NAME, KERN_ERR "%s: " fmt, __FUNCTION__ ,##arg)


#define DEBUG_LEDS_STROBE
#ifdef  DEBUG_LEDS_STROBE
	#define PK_DBG PK_DBG_FUNC
	#define PK_VER PK_TRC_VERBOSE
	#define PK_ERR PK_ERROR
#else
	#define PK_DBG(a,...)
	#define PK_VER(a,...)
	#define PK_ERR(a,...)
#endif



/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock); /* cotta-- SMP proection */


static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int gDuty=0;
static int g_timeOutTimeMs=0;
static BOOL g_is_torch_mode = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif





static struct work_struct workTimeOut;
static void work_timeOutFunc(struct work_struct *data);


#define LEDS_TORCH_MODE 		1
#define LEDS_FLASH_MODE 		0
#define LEDS_CUSTOM_MODE_THRES 	20

#ifndef GPIO_CAMERA_FLASH_PWM_PIN 
#define GPIO_CAMERA_FLASH_PWM_PIN GPIO12
#endif

#ifndef GPIO_CAMERA_FLASH_PWM_PIN_M_GPIO 
#define GPIO_CAMERA_FLASH_PWM_PIN_M_GPIO GPIO_MODE_00
#endif

#ifndef GPIO_CAMERA_FLASH_MODE_PIN 
#define GPIO_CAMERA_FLASH_MODE_PIN GPIO13
#endif

#ifndef GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO 
#define GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO GPIO_MODE_00
#endif
static U16 g_duty = 0;

extern  S32 sgm3785_set_flash_mode(U16 duty);
extern  S32 sgm3785_set_torch_mode(U16 duty);
extern s32 sgm3785_shutdoff();
int FL_enable(void)
{
	PK_DBG("lili>>>>g_is_torch_mode=%d,g_duty=%d\n",g_is_torch_mode,g_duty);	
	if(!g_is_torch_mode)
	{
		if(g_duty>5)
			g_duty = 11;
		PK_DBG("shenwuyi go here.g_is_torch_mode=1\n");
		sgm3785_set_flash_mode(g_duty);
		g_is_torch_mode=0;
	}
	else
	{	
		if(g_duty==0)			
			sgm3785_set_torch_mode(11); //torch
		else 
			sgm3785_set_torch_mode(2);  //pf
	}
	
//	upmu_set_rg_bst_drv_1m_ck_pdn(0);
//	upmu_set_flash_en(1);
    return 0;
}

int FL_disable(void)
{
	PK_DBG("FL_disable");
    sgm3785_shutdoff();
//	upmu_set_flash_en(0);
	//upmu_set_rg_bst_drv_1m_ck_pdn(1);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	PK_DBG("FL_dim_duty %d, g_is_torch_mode %d", duty, g_is_torch_mode);
	if(duty <=1)	{
		g_is_torch_mode = 1;
		g_duty = duty;
	}
	else{
		g_is_torch_mode = 0;	
		g_duty = duty;
	}
	if((g_timeOutTimeMs == 0) && (duty >1))
	{
	    
		g_duty = duty;
		PK_ERR("FL_dim_duty %d > thres %d, FLASH mode but timeout %d", duty, LEDS_CUSTOM_MODE_THRES, g_timeOutTimeMs);	
		g_is_torch_mode = 0;	
	}	
//	upmu_set_flash_dim_duty(duty);
    return 0;
}

int FL_step(kal_uint32 step)
{
	int sTab[8]={0,2,4,6,9,11,13,15};
	PK_DBG("FL_step");
//	upmu_set_flash_sel(sTab[step]);	
    return 0;
}

S32 set_huawei_torch_duty(U16 torch)
{
	printk("wangyang the torch  torch = %d \n",torch);
	if(torch  == 1)
	{
			g_is_torch_mode = 1;
		        sgm3785_set_torch_mode(11);
		//return 1;
	}
        else
   	 {
   	 		g_is_torch_mode = 0;
			sgm3785_shutdoff();
			printk("[sgm3785] duty error!!!\n");
   	 }
}


int FL_init(void)
{
//	upmu_set_flash_dim_duty(0);
//	upmu_set_flash_sel(0);
	PK_DBG("FL_init");

	mt_set_gpio_mode(GPIO_CAMERA_FLASH_PWM_PIN, GPIO_CAMERA_FLASH_PWM_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_PWM_PIN, GPIO_DIR_OUT);  
	mt_set_gpio_mode(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_CAMERA_FLASH_MODE_PIN_M_GPIO);
	mt_set_gpio_dir(GPIO_CAMERA_FLASH_MODE_PIN, GPIO_DIR_OUT);

	FL_disable();
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_is_torch_mode = 0;
    return 0;
}

int FL_uninit(void)
{
	PK_DBG("FL_uninit");
	FL_disable();
	g_is_torch_mode = 0;
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_disable();
    PK_DBG("ledTimeOut_callback\n");
    //printk(KERN_ALERT "work handler function./n");
}
static enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
    schedule_work(&workTimeOut);
    return HRTIMER_NORESTART;
}
static struct hrtimer g_timeOutTimer;
static void timerInit(void)
{
       static int init_flag;	
       if (init_flag==0){	
	     init_flag=1;
  	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init( &g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL );
	g_timeOutTimer.function=ledTimeOutCallback;
}
}

static int constant_flashlight_ioctl(MUINT32 cmd, MUINT32 arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	int iFlashType = (int)FLASHLIGHT_NONE;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("RT4505 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
	switch(cmd)
	{
		case FLASH_IOC_SET_TIME_OUT_TIME_MS:
			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);
			g_timeOutTimeMs=arg;
		break;

		case FLASH_IOC_SET_DUTY :
			PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
			FL_dim_duty(arg);
			break;

		case FLASH_IOC_SET_STEP:
			PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);
			break;

		case FLASH_IOC_SET_ONOFF :
			PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
			if(arg==1)
			{
			if(g_timeOutTimeMs!=0)
		            {
	        		ktime_t ktime;
				ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
				hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
		            }
				FL_enable();
			}
			else
			{
				FL_disable();
				hrtimer_cancel( &g_timeOutTimer );
			}
			break;
	    //case FLASHLIGHTIOC_G_FLASHTYPE:
	    //    iFlashType = FLASHLIGHT_LED_CONSTANT;
	     //   if(copy_to_user((void __user *) arg , (void*)&iFlashType , _IOC_SIZE(cmd)))
	     //   {
	     //       PK_DBG("[strobe_ioctl] ioctl copy to user failed\n");
	     //       return -EFAULT;
	     //   }
	     //   break;			
		default :
			PK_DBG(" No such command \n");
			i4RetValue = -EPERM;
			break;
	}
    return i4RetValue;
}




static int constant_flashlight_open(void *pArg)
{
    int i4RetValue = 0;
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
	    FL_init();
		timerInit();
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);


    if(strobe_Res)
    {
        PK_ERR(" busy!\n");
        i4RetValue = -EBUSY;
    }
    else
    {
        strobe_Res += 1;
    }


    spin_unlock_irq(&g_strobeSMPLock);
    PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

    return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
    PK_DBG(" constant_flashlight_release\n");

    if (strobe_Res)
    {
        spin_lock_irq(&g_strobeSMPLock);

        strobe_Res = 0;
        strobe_Timeus = 0;

        /* LED On Status */
        g_strobe_On = FALSE;

        spin_unlock_irq(&g_strobeSMPLock);

    	FL_uninit();
    }

    PK_DBG(" Done\n");

    return 0;

}


FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc != NULL)
    {
        *pfFunc = &constantFlashlightFunc;
    }
    return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

    return 0;
}

EXPORT_SYMBOL(strobe_VDIrq);







