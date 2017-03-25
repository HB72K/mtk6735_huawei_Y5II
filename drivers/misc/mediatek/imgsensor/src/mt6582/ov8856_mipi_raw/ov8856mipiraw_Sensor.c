/*******************************************************************************************/
     

/*******************************************************************************************/

#include <linux/videodev2.h>    
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
#include <linux/xlog.h>
#include <asm/system.h>
 #include <linux/slab.h>
#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "ov8856mipiraw_Sensor.h"


//#include "ov8856mipiraw_Sensor_R2A.h"
#include "ov8856mipiraw_Camera_Sensor_para.h"
#include "ov8856mipiraw_CameraCustomized.h"

//WangChao 14_0806 add+
//#include "../../huaqin/hardwareinfo/hardwareinfo.h"
//WangChao 14_0806 add-

static DEFINE_SPINLOCK(ov8856mipiraw_drv_lock);
#define OV8856R1AOTP

#define OV8856_DEBUG
#ifdef OV8856_DEBUG
	#define OV8856DB  printk
#else
	#define OV8856DB(fmt, arg...)
#endif


kal_uint32 OV8856_FeatureControl_PERIOD_PixelNum=OV8856_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV8856_FeatureControl_PERIOD_LineNum=OV8856_PV_PERIOD_LINE_NUMS;

UINT16 OV8856_VIDEO_MODE_TARGET_FPS = 30;

MSDK_SCENARIO_ID_ENUM OV8856CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
MSDK_SENSOR_CONFIG_STRUCT OV8856SensorConfigData;
static OV8856_PARA_STRUCT ov8856;
kal_uint32 OV8856_FAC_SENSOR_REG;


SENSOR_REG_STRUCT OV8856SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV8856SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;

#define OV8856_TEST_PATTERN_CHECKSUM 0x899a55db  //0x47a75476
kal_bool OV8856_During_testpattern = KAL_FALSE;
void OV8856SetFlipMirror(kal_int32 imgMirror);

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

#define OV8856_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, ov8856.write_id) //yanggy modify for I2c read >>>Do not modify this part when code merging,or I2C will not available
 kal_uint16 OV8856_read_cmos_sensor(kal_uint32 addr)
 {
     kal_uint16 get_byte=0;
     iReadReg((u16) addr ,(u8*)&get_byte,ov8856.write_id);   //yanggy modify for I2c read >>>Do not modify this part when code merging,or I2C will not available
     return get_byte;
 }
 

 #ifdef OV8856R1AOTP
 /*****************************OV8856 OTP start***********************************/
 
 struct otp_struct 
 {
   int flag; // bit[7]: info, bit[6]:wb, bit[5]:vcm, bit[4]:lenc
   int module_integrator_id;
   int lens_id;
   int production_year;
   int production_month;
   int production_day;
   int rg_ratio;
   int bg_ratio;
   int lenc[240];
   int checksum;
   int VCM_start;
   int VCM_end;
   int VCM_dir;
 };
 // return value:
 // bit[7]: 0 no otp info, 1 valid otp info
 // bit[6]: 0 no otp wb, 1 valib otp wb
 // bit[5]: 0 no otp vcm, 1 valid otp vcm
 // bit[4]: 0 no otp lenc/invalid otp lenc, 1 valid otp lenc
static struct otp_struct ov8856_otp_ptr={0} ;
 static int read_otp(struct otp_struct *otp_ptr)
 {
	OV8856_write_cmos_sensor(0x0100,0x01);
	mdelay(10);
   int otp_flag, addr, temp, i;
   //set 0x5001[3] to ??0??
   int temp1;
   temp1 = OV8856_read_cmos_sensor(0x5001);    
   
   OV8856_write_cmos_sensor(0x5001, (0x00 & 0x08) | (temp1 & (~0x08)));
   // read OTP into buffer
   OV8856_write_cmos_sensor(0x3d84, 0xC0);
   OV8856_write_cmos_sensor(0x3d88, 0x70); // OTP start address
   OV8856_write_cmos_sensor(0x3d89, 0x10);
   OV8856_write_cmos_sensor(0x3d8A, 0x72); // OTP end address
   OV8856_write_cmos_sensor(0x3d8B, 0x0E);
   OV8856_write_cmos_sensor(0x3d81, 0x01); // load otp into buffer
   mdelay(10);
   
   // OTP base information and WB calibration data
   otp_flag = OV8856_read_cmos_sensor(0x7010);
   
   OV8856DB("lili>>>>ov8856_otp_read_test  otp_flag: 0x%x,\n", otp_flag); 
   addr = 0;
   
   if((otp_flag & 0xc0) == 0x40) 
   {
     addr = 0x7011; // base address of info group 1
   }
   else if((otp_flag & 0x30) == 0x10) 
   {
     addr = 0x701a; // base address of info group 2
   }
   if(addr != 0)
   {
     (*otp_ptr).flag = 0xC0; // valid info and AWB in OTP
     (*otp_ptr).module_integrator_id = OV8856_read_cmos_sensor(addr);
     (*otp_ptr).lens_id = OV8856_read_cmos_sensor( addr + 1);
     (*otp_ptr).production_year = OV8856_read_cmos_sensor( addr + 2);
     (*otp_ptr).production_month = OV8856_read_cmos_sensor( addr + 3);
     (*otp_ptr).production_day = OV8856_read_cmos_sensor(addr + 4);
     temp = OV8856_read_cmos_sensor(addr + 7);
     (*otp_ptr).rg_ratio = (OV8856_read_cmos_sensor(addr + 5)<<2) + ((temp>>6) & 0x03);
     (*otp_ptr).bg_ratio = (OV8856_read_cmos_sensor(addr + 6)<<2) + ((temp>>4) & 0x03);
   }
   else 
   {
     (*otp_ptr).flag = 0x00; // not info and AWB in OTP
     (*otp_ptr).module_integrator_id = 0;
     (*otp_ptr).lens_id = 0;
     (*otp_ptr).production_year = 0;
     (*otp_ptr).production_month = 0;
     (*otp_ptr).production_day = 0;
     (*otp_ptr).rg_ratio = 0;
     (*otp_ptr).bg_ratio = 0;
   }
   // OTP VCM Calibration
   otp_flag = OV8856_read_cmos_sensor(0x7023);
   addr = 0;
   if((otp_flag & 0xc0) == 0x40) 
   {
     addr = 0x7024; // base address of VCM Calibration group 1
   }
   else if((otp_flag & 0x30) == 0x10) 
   {
     addr = 0x7028; // base address of VCM Calibration group 2
   }
   if(addr != 0) 
   {
     (*otp_ptr).flag |= 0x20;
     temp = OV8856_read_cmos_sensor(addr + 2);
     (* otp_ptr).VCM_start = (OV8856_read_cmos_sensor(addr)<<2) | ((temp>>6) & 0x03);
     (* otp_ptr).VCM_end = (OV8856_read_cmos_sensor(addr + 1) << 2) | ((temp>>4) & 0x03);
     (* otp_ptr).VCM_dir = (temp>>2) & 0x03;
   }
   else 
   {
     (* otp_ptr).VCM_start = 0;
     (* otp_ptr).VCM_end = 0;
     (* otp_ptr).VCM_dir = 0;
    }
 // OTP Lenc Calibration
   otp_flag = OV8856_read_cmos_sensor(0x702c);
   addr = 0;
   int checksum2=0;
   
   if((otp_flag & 0xc0) == 0x40) 
   {
     addr = 0x702d; // base address of Lenc Calibration group 1
   }
   else if((otp_flag & 0x30) == 0x10) 
   {
     addr = 0x711e; // base address of Lenc Calibration group 2
   }
   if(addr != 0) 
   {
     for(i=0;i<240;i++) 
     {
       (* otp_ptr).lenc[i]=OV8856_read_cmos_sensor(addr + i);
       checksum2 += (* otp_ptr).lenc[i];
     }             
     
     checksum2 = (checksum2)%255 +1;
     (* otp_ptr).checksum = OV8856_read_cmos_sensor((addr + 240));
     
     if((* otp_ptr).checksum == checksum2)
     {
       (*otp_ptr).flag |= 0x10;
     }
   }
   else 
   {
     for(i=0;i<240;i++) 
     {
       (* otp_ptr).lenc[i]=0;
     }
   }
   for(i=0x7010;i<=0x720e;i++) 
   {
     OV8856_write_cmos_sensor(i,0); // clear OTP buffer, recommended use continuous write to accelarate
   }
   //set 0x5001[3] to ??1??
   temp1 = OV8856_read_cmos_sensor(0x5001);
   OV8856_write_cmos_sensor(0x5001, (0x08 & 0x08) | (temp1 & (~0x08)));
   
   return (*otp_ptr).flag;
 }
 
 // return value:
 // bit[7]: 0 no otp info, 1 valid otp info
 // bit[6]: 0 no otp wb, 1 valib otp wb
 // bit[5]: 0 no otp vcm, 1 valid otp vcm
 // bit[4]: 0 no otp lenc, 1 valid otp lenc
static int RG_Ratio_Typical= 0x133 ; //0x14C 
static int BG_Ratio_Typical= 0x142 ; //0x130
static int apply_otp(struct otp_struct *otp_ptr)
 {
	printk("lili>>>>>%s\n",__func__);
   int rg, bg, R_gain, G_gain, B_gain, Base_gain, temp, i;
   // apply OTP WB Calibration
   if ((*otp_ptr).flag & 0x40) 
   {
     rg = (*otp_ptr).rg_ratio;
     bg = (*otp_ptr).bg_ratio;
     //calculate G gain
     R_gain = (RG_Ratio_Typical*1000) / rg;
     B_gain = (BG_Ratio_Typical*1000) / bg;
     G_gain = 1000;
     
     if (R_gain < 1000 || B_gain < 1000)
     {
       if (R_gain < B_gain)
         Base_gain = R_gain;
       else
         Base_gain = B_gain;
     }
     else
     {
       Base_gain = G_gain;
     }
     R_gain = 0x400 * R_gain / (Base_gain);
     B_gain = 0x400 * B_gain / (Base_gain);
     G_gain = 0x400 * G_gain / (Base_gain);
     
     // update sensor WB gain
     if (R_gain>0x400) 
     {
       OV8856_write_cmos_sensor(0x5019, R_gain>>8);
       OV8856_write_cmos_sensor(0x501a, R_gain & 0x00ff);
     }
     if (G_gain>0x400) 
     {
       OV8856_write_cmos_sensor(0x501b, G_gain>>8);
       OV8856_write_cmos_sensor(0x501c, G_gain & 0x00ff);
     }
     if (B_gain>0x400) 
     {
       OV8856_write_cmos_sensor(0x501d, B_gain>>8);
       OV8856_write_cmos_sensor(0x501e, B_gain & 0x00ff);
     }
   }
	OV8856DB("lili>>>>0x%x>>>0x%x>>>0x%x\n",R_gain,B_gain,G_gain);
   // apply OTP Lenc Calibration
   if ((*otp_ptr).flag & 0x10) 
   {
     temp = OV8856_read_cmos_sensor(0x5000);
     temp = 0x20 | temp;
     OV8856_write_cmos_sensor(0x5000, temp); 
     
     for(i=0;i<240;i++) 
     {
       OV8856_write_cmos_sensor(0x5900 + i, (*otp_ptr).lenc[i]);
     }
	OV8856DB("lili>>>>>>lsc OTP success\n");
   }
   return (*otp_ptr).flag;
 }
 
 /*****************************  OTP Feature  End**********************************/
#endif



void OV8856_Init_Para(void)
{

	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.sensorMode = SENSOR_MODE_INIT;
	ov8856.OV8856AutoFlickerMode = KAL_FALSE;
	ov8856.OV8856VideoMode = KAL_FALSE;
	ov8856.DummyLines= 0;
	ov8856.DummyPixels= 0;
	ov8856.pvPclk =  (14400); 
	ov8856.videoPclk = (14400);
	ov8856.capPclk = (14400);

	ov8856.shutter = 0x4C00;
	ov8856.ispBaseGain = BASEGAIN;		//64
	ov8856.sensorGlobalGain = 0x0200;  //512
	spin_unlock(&ov8856mipiraw_drv_lock);
}

kal_uint32 GetOv8856LineLength(void)
{
	kal_uint32 OV8856_line_length = 0;
	if ( SENSOR_MODE_PREVIEW == ov8856.sensorMode )  
	{
		OV8856_line_length = OV8856_PV_PERIOD_PIXEL_NUMS + ov8856.DummyPixels;
	}
	else if( SENSOR_MODE_VIDEO == ov8856.sensorMode ) 
	{
		OV8856_line_length = OV8856_VIDEO_PERIOD_PIXEL_NUMS + ov8856.DummyPixels;
	}
	else
	{
		OV8856_line_length = OV8856_FULL_PERIOD_PIXEL_NUMS + ov8856.DummyPixels;
	}
	
#ifdef OV8856_DEBUG
	OV8856DB("[GetOv8856LineLength]: ov8856.sensorMode = %d, OV8856_line_length =%d, ov8856.DummyPixels = %d\n", ov8856.sensorMode,OV8856_line_length,ov8856.DummyPixels);
#endif


    return OV8856_line_length;

}


kal_uint32 GetOv8856FrameLength(void)
{
	kal_uint32 OV8856_frame_length = 0;

	if ( SENSOR_MODE_PREVIEW == ov8856.sensorMode )  
	{
		OV8856_frame_length = OV8856_PV_PERIOD_LINE_NUMS + ov8856.DummyLines ;
	}
	else if( SENSOR_MODE_VIDEO == ov8856.sensorMode ) 
	{
		OV8856_frame_length = OV8856_VIDEO_PERIOD_LINE_NUMS + ov8856.DummyLines ;
	}
	else
	{
		OV8856_frame_length = OV8856_FULL_PERIOD_LINE_NUMS + ov8856.DummyLines ;
	}

#ifdef OV8856_DEBUG
		OV8856DB("[GetOv8856FrameLength]: ov8856.sensorMode = %d, OV8856_frame_length =%d, ov8856.DummyLines = %d\n", ov8856.sensorMode,OV8856_frame_length,ov8856.DummyLines);
#endif


	return OV8856_frame_length;
}


kal_uint32 OV8856_CalcExtra_For_ShutterMargin(kal_uint32 shutter_value,kal_uint32 shutterLimitation)
{
    kal_uint32 extra_lines = 0;

	
	if (shutter_value <4 ){
		shutter_value = 4;
	}

	
	if (shutter_value > shutterLimitation)
	{
		extra_lines = shutter_value - shutterLimitation;
    }
	else
		extra_lines = 0;

#ifdef OV8856_DEBUG
			OV8856DB("[OV8856_CalcExtra_For_ShutterMargin]: shutter_value = %d, shutterLimitation =%d, extra_lines = %d\n", shutter_value,shutterLimitation,extra_lines);
#endif

    return extra_lines;

}

kal_uint32 OV8856_CalcFrameLength_For_AutoFlicker(void)
{

    kal_uint32 AutoFlicker_min_framelength = 0;

	switch(OV8856CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			AutoFlicker_min_framelength = (ov8856.capPclk*10000) /(OV8856_FULL_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/OV8856_AUTOFLICKER_OFFSET_30*10 ;
			break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			if(OV8856_VIDEO_MODE_TARGET_FPS==30)
			{
				AutoFlicker_min_framelength = (ov8856.videoPclk*10000) /(OV8856_VIDEO_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/OV8856_AUTOFLICKER_OFFSET_30*10 ;
			}
			else if(OV8856_VIDEO_MODE_TARGET_FPS==15)
			{
				AutoFlicker_min_framelength = (ov8856.videoPclk*10000) /(OV8856_VIDEO_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/OV8856_AUTOFLICKER_OFFSET_15*10 ;
			}
			else
			{
				AutoFlicker_min_framelength = OV8856_VIDEO_PERIOD_LINE_NUMS + ov8856.DummyLines;
			}
			break;
			
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		default:
			AutoFlicker_min_framelength = (ov8856.pvPclk*10000) /(OV8856_PV_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/OV8856_AUTOFLICKER_OFFSET_30*10 ;
			break;
	}

	#ifdef OV8856_DEBUG 
	OV8856DB("AutoFlicker_min_framelength =%d,OV8856CurrentScenarioId =%d\n", AutoFlicker_min_framelength,OV8856CurrentScenarioId);
	#endif

	return AutoFlicker_min_framelength;

}
#if 0
static void set_max_framerate(UINT16 framerate)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = 0;
	kal_uint32 line_length = 0;
	kal_uint32 min_frame_length = 0x9b2;
	kal_uint32 max_frame_length = 0x7530;
	kal_uint32 imgsensor_frame_length = 0;
	kal_uint32 imgsensor_dummy_line = 0;
	//unsigned long flags;
  frame_length = GetOv8856FrameLength();
  line_length  = GetOv8856LineLength();
	OV8856DB("framerate = %d \n", framerate);
   
	frame_length = ov8856.pvPclk / framerate * 10 / line_length;
	
	imgsensor_frame_length = (frame_length > min_frame_length) ? frame_length : min_frame_length; 
	imgsensor_dummy_line = imgsensor_frame_length - min_frame_length;
	
	if (imgsensor_frame_length > max_frame_length)
	{
		imgsensor_frame_length = max_frame_length;
		imgsensor_dummy_line = imgsensor_frame_length - min_frame_length;
	}
			
	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.DummyLines = imgsensor_dummy_line;
	OV8856_FeatureControl_PERIOD_LineNum = imgsensor_frame_length;
	spin_unlock(&ov8856mipiraw_drv_lock);
	
}	/*	set_max_framerate  */
#endif

void OV8856_write_shutter(kal_uint32 shutter)
{
  //kal_uint32 min_framelength = OV8856_PV_PERIOD_PIXEL_NUMS;
	//the init code write as up line;
	//modify it as follow
	kal_uint32 min_framelength = OV8856_PV_PERIOD_LINE_NUMS;
	kal_uint32 max_shutter=0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	kal_uint32 framelength = 0;
	unsigned long flags;

	//TODO~
	kal_uint32 read_shutter_1 = 0;
	kal_uint32 read_shutter_2 = 0;
	kal_uint32 read_shutter_3 = 0;

	//TODO~
    if(shutter > 0x90f7)//500ms for capture SaturationGain
    {
    	#ifdef OV8856_DEBUG
		OV8856DB("[OV8856_write_shutter] shutter > 0x90f7 [warn.] shutter=%x, \n", shutter);
		#endif
		shutter = 0x90f7;
    }
	
    line_length  = GetOv8856LineLength();
	frame_length = GetOv8856FrameLength();
	
	max_shutter  = frame_length-OV8856_SHUTTER_MARGIN;

    frame_length = frame_length + OV8856_CalcExtra_For_ShutterMargin(shutter,max_shutter);
	


	if(ov8856.OV8856AutoFlickerMode == KAL_TRUE)
	{
        min_framelength = OV8856_CalcFrameLength_For_AutoFlicker();

        if(frame_length < min_framelength)
			frame_length = min_framelength;
	}
	
	if((frame_length % 2) == 0)
		{
		  	framelength = frame_length;
		}
	else
		{
		    framelength = frame_length + 1;	
		}

	spin_lock_irqsave(&ov8856mipiraw_drv_lock,flags);
	OV8856_FeatureControl_PERIOD_PixelNum = line_length;
	OV8856_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock_irqrestore(&ov8856mipiraw_drv_lock,flags);

	//Set total frame length
	OV8856_write_cmos_sensor(0x380e, (framelength >> 8) & 0xFF);
	OV8856_write_cmos_sensor(0x380f, framelength & 0xFF);
	
	//Set shutter 
	OV8856_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
	OV8856_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
	OV8856_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	

	#ifdef OV8856_DEBUG
	OV8856DB("[OV8856_write_shutter]ov8856 write shutter=%x, line_length=%x, frame_length=%x\n", shutter, line_length, frame_length);
	#endif

}

void OV8856_SetShutter(kal_uint32 iShutter)
{

   spin_lock(&ov8856mipiraw_drv_lock);
   ov8856.shutter= iShutter;
   spin_unlock(&ov8856mipiraw_drv_lock);

   OV8856_write_shutter(iShutter);
   return;
}


UINT32 OV8856_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV8856_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV8856_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV8856_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

static kal_uint16 OV8856Reg2Gain(const kal_uint16 iReg)
{
    kal_uint16 iGain =0; 

	iGain = iReg*BASEGAIN/OV8856_GAIN_BASE;
	return iGain;
}

static kal_uint16 OV8856Gain2Reg(const kal_uint32 iGain)
{
    kal_uint32 iReg = 0x0000;

	iReg = iGain * 2; //(iGain/BASEGAIN)*OV8856_GAIN_BASE;

    return iReg;
}

void write_OV8856_gain(kal_uint16 gain)
{
	//kal_uint16 read_gain=0;

	OV8856_write_cmos_sensor(0x3508,(gain>>8));
	OV8856_write_cmos_sensor(0x3509,(gain&0xff));

	//read_gain=(((OV8856_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8856_read_cmos_sensor(0x3509));
	//OV8856DB("[OV8856_SetGain]0x3508|0x3509=0x%x \n",read_gain);

	return;
}

void OV8856_SetGain(UINT16 iGain)
{
	unsigned long flags;

	
	OV8856DB("OV8856_SetGain iGain = %d :\n ",iGain);

	spin_lock_irqsave(&ov8856mipiraw_drv_lock,flags);
	ov8856.realGain = iGain;
	ov8856.sensorGlobalGain = OV8856Gain2Reg(iGain);
	spin_unlock_irqrestore(&ov8856mipiraw_drv_lock,flags);
	write_OV8856_gain(ov8856.sensorGlobalGain);
	#ifdef OV8856_DEBUG
	OV8856DB(" [OV8856_SetGain]ov8856.sensorGlobalGain=0x%x,ov8856.realGain =0x%x",ov8856.sensorGlobalGain,ov8856.realGain); 
	#endif
	//temperature test
	//OV8856_write_cmos_sensor(0x4d12,0x01);
	//OV8856DB("Temperature read_reg  0x4d13  =%x \n",OV8856_read_cmos_sensor(0x4d13));
}   

kal_uint16 read_OV8856_gain(void)
{
	kal_uint16 read_gain=0;

	read_gain=(((OV8856_read_cmos_sensor(0x3508)&0x1F) << 8) | OV8856_read_cmos_sensor(0x3509));

	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.sensorGlobalGain = read_gain;
	ov8856.realGain = OV8856Reg2Gain(ov8856.sensorGlobalGain);
	spin_unlock(&ov8856mipiraw_drv_lock);

	OV8856DB("ov8856.sensorGlobalGain=0x%x,ov8856.realGain=%d\n",ov8856.sensorGlobalGain,ov8856.realGain);

	return ov8856.sensorGlobalGain;
}  



static void OV8856_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	kal_uint32 framelength = 0;

	if ( SENSOR_MODE_PREVIEW == ov8856.sensorMode )
	{
		line_length = OV8856_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8856_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== ov8856.sensorMode )
	{
		line_length = OV8856_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8856_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else
	{
		line_length = OV8856_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV8856_FULL_PERIOD_LINE_NUMS + iLines;
	}
	
	if((frame_length % 2) == 0)
		{
		  	framelength = frame_length;
		}
	else
		{
		    framelength = frame_length + 1;	
		}

	spin_lock(&ov8856mipiraw_drv_lock);
	OV8856_FeatureControl_PERIOD_PixelNum = line_length;
	OV8856_FeatureControl_PERIOD_LineNum = framelength;
	spin_unlock(&ov8856mipiraw_drv_lock);

	//Set total frame length
	OV8856_write_cmos_sensor(0x380e, (framelength >> 8) & 0xFF);
	OV8856_write_cmos_sensor(0x380f, framelength & 0xFF);
	//Set total line length
	OV8856_write_cmos_sensor(0x380c, (line_length >> 8) & 0xFF);
	OV8856_write_cmos_sensor(0x380d, line_length & 0xFF);

	#ifdef OV8856_DEBUG
	OV8856DB(" [OV8856_SetDummy]ov8856.sensorMode = %d, line_length = %d,iPixels = %d, frame_length =%d, iLines = %d\n",ov8856.sensorMode, line_length,iPixels, frame_length, iLines); 
	#endif

}   

void OV8856_camera_para_to_sensor(void)
{
    kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=OV8856SensorReg[i].Addr; i++)
    {
        OV8856_write_cmos_sensor(OV8856SensorReg[i].Addr, OV8856SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8856SensorReg[i].Addr; i++)
    {
        OV8856_write_cmos_sensor(OV8856SensorReg[i].Addr, OV8856SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        OV8856_write_cmos_sensor(OV8856SensorCCT[i].Addr, OV8856SensorCCT[i].Para);
    }
}

void OV8856_sensor_to_camera_para(void)
{
    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=OV8856SensorReg[i].Addr; i++)
    {
         temp_data = OV8856_read_cmos_sensor(OV8856SensorReg[i].Addr);
		 spin_lock(&ov8856mipiraw_drv_lock);
		 OV8856SensorReg[i].Para =temp_data;
		 spin_unlock(&ov8856mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=OV8856SensorReg[i].Addr; i++)
    {
        temp_data = OV8856_read_cmos_sensor(OV8856SensorReg[i].Addr);
		spin_lock(&ov8856mipiraw_drv_lock);
		OV8856SensorReg[i].Para = temp_data;
		spin_unlock(&ov8856mipiraw_drv_lock);
    }
}

kal_int32  OV8856_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV8856_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
   switch (group_idx)
   {
        case PRE_GAIN:
            sprintf((char *)group_name_ptr, "CCT");
            *item_count_ptr = 2;
            break;
        case CMMCLK_CURRENT:
            sprintf((char *)group_name_ptr, "CMMCLK Current");
            *item_count_ptr = 1;
            break;
        case FRAME_RATE_LIMITATION:
            sprintf((char *)group_name_ptr, "Frame Rate Limitation");
            *item_count_ptr = 2;
            break;
        case REGISTER_EDITOR:
            sprintf((char *)group_name_ptr, "Register Editor");
            *item_count_ptr = 2;
            break;
        default:
            ASSERT(0);
}
}

void OV8856_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
    kal_int16 temp_reg=0;
    kal_uint16 temp_gain=0, temp_addr=0, temp_para=0;

    switch (group_idx)
    {
        case PRE_GAIN:
           switch (item_idx)
          {
              case 0:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-R");
                  temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gr");
                  temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-Gb");
                  temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                sprintf((char *)info_ptr->ItemNamePtr,"Pregain-B");
                  temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                 sprintf((char *)info_ptr->ItemNamePtr,"SENSOR_BASEGAIN");
                 temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

            temp_para= OV8856SensorCCT[temp_addr].Para;
			//temp_gain= (temp_para/ov8856.sensorBaseGain) * 1000;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= OV8856_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= OV8856_MAX_ANALOG_GAIN * 1000;
            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Drv Cur[2,4,6,8]mA");

                    //temp_reg=MT9P017SensorReg[CMMCLK_CURRENT_INDEX].Para;
                    temp_reg = ISP_DRIVING_2MA;
                    if(temp_reg==ISP_DRIVING_2MA)
                    {
                        info_ptr->ItemValue=2;
                    }
                    else if(temp_reg==ISP_DRIVING_4MA)
                    {
                        info_ptr->ItemValue=4;
                    }
                    else if(temp_reg==ISP_DRIVING_6MA)
                    {
                        info_ptr->ItemValue=6;
                    }
                    else if(temp_reg==ISP_DRIVING_8MA)
                    {
                        info_ptr->ItemValue=8;
                    }

                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_TRUE;
                    info_ptr->Min=2;
                    info_ptr->Max=8;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"Max Exposure Lines");
                    info_ptr->ItemValue=    111;  
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"Min Frame Rate");
                    info_ptr->ItemValue=12;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_TRUE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0;
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Addr.");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                case 1:
                    sprintf((char *)info_ptr->ItemNamePtr,"REG Value");
                    info_ptr->ItemValue=0;
                    info_ptr->IsTrueFalse=KAL_FALSE;
                    info_ptr->IsReadOnly=KAL_FALSE;
                    info_ptr->IsNeedRestart=KAL_FALSE;
                    info_ptr->Min=0;
                    info_ptr->Max=0xFFFF;
                    break;
                default:
                ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
}



kal_bool OV8856_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;

   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
              case 0:
                temp_addr = PRE_GAIN_R_INDEX;
              break;
              case 1:
                temp_addr = PRE_GAIN_Gr_INDEX;
              break;
              case 2:
                temp_addr = PRE_GAIN_Gb_INDEX;
              break;
              case 3:
                temp_addr = PRE_GAIN_B_INDEX;
              break;
              case 4:
                temp_addr = SENSOR_BASEGAIN;
              break;
              default:
                 ASSERT(0);
          }

		 temp_gain=((ItemValue*BASEGAIN+500)/1000);			//+500:get closed integer value

		  if(temp_gain>=1*BASEGAIN && temp_gain<=16*BASEGAIN)
          {
//             temp_para=(temp_gain * ov8856.sensorBaseGain + BASEGAIN/2)/BASEGAIN;
          }
          else
			  ASSERT(0);

		  spin_lock(&ov8856mipiraw_drv_lock);
          OV8856SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&ov8856mipiraw_drv_lock);
          OV8856_write_cmos_sensor(OV8856SensorCCT[temp_addr].Addr,temp_para);

            break;
        case CMMCLK_CURRENT:
            switch (item_idx)
            {
                case 0:
                    //no need to apply this item for driving current
                    break;
                default:
                    ASSERT(0);
            }
            break;
        case FRAME_RATE_LIMITATION:
            ASSERT(0);
            break;
        case REGISTER_EDITOR:
            switch (item_idx)
            {
                case 0:
					spin_lock(&ov8856mipiraw_drv_lock);
                    OV8856_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&ov8856mipiraw_drv_lock);
                    break;
                case 1:
                    OV8856_write_cmos_sensor(OV8856_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }
    return KAL_TRUE;
}


void OV8856_1224pSetting(void)
{
OV8856PreviewSetting();
}

void OV8856PreviewSetting(void)
{
	// ++++++++  ;Pclk 144MHz, MIPI 720Mbps/lane, 1632x1224
	// ++++++++  ;pixels per line=1932) 
	// ++++++++  ;lines per frame=2482
	// ++++++++  ;
	
	OV8856_write_cmos_sensor(0x0100, 0x00);
	OV8856_write_cmos_sensor(0x3501, 0x4c);
	OV8856_write_cmos_sensor(0x3502, 0xe0);
	OV8856_write_cmos_sensor(0x366e, 0x08);
	OV8856_write_cmos_sensor(0x37c2, 0x14);
  	OV8856_write_cmos_sensor(0x3714, 0x27);
  	OV8856_write_cmos_sensor(0x3800, 0x00);        
	OV8856_write_cmos_sensor(0x3801, 0x00);        
	OV8856_write_cmos_sensor(0x3802, 0x00);        
	OV8856_write_cmos_sensor(0x3803, 0x0c);        
	OV8856_write_cmos_sensor(0x3804, 0x0c);        
	OV8856_write_cmos_sensor(0x3805, 0xdf);        
	OV8856_write_cmos_sensor(0x3806, 0x09);        
	OV8856_write_cmos_sensor(0x3807, 0xa3); 
	OV8856_write_cmos_sensor(0x3808, 0x06);
	OV8856_write_cmos_sensor(0x3809, 0x60);
	OV8856_write_cmos_sensor(0x380a, 0x04);
	OV8856_write_cmos_sensor(0x380b, 0xc8);
	OV8856_write_cmos_sensor(0x380c, 0x07);
	OV8856_write_cmos_sensor(0x380d, 0x8c);
  OV8856_write_cmos_sensor(0x380e, 0x09);//0x04);
  OV8856_write_cmos_sensor(0x380f, 0xb2);//0xde);
  OV8856_write_cmos_sensor(0x3810, 0x00);
	OV8856_write_cmos_sensor(0x3811, 0x0a);  //FOV, please modify:Preview: R3811  08-->02
  OV8856_write_cmos_sensor(0x3813, 0x02);
	OV8856_write_cmos_sensor(0x3814, 0x03);
	OV8856_write_cmos_sensor(0x3820, 0xd6);//0x90
	OV8856_write_cmos_sensor(0x3821, 0x67);
	OV8856_write_cmos_sensor(0x382a, 0x03);
	OV8856_write_cmos_sensor(0x4009, 0x05);
	OV8856_write_cmos_sensor(0x5795, 0x00);
	OV8856_write_cmos_sensor(0x5796, 0x10);
	OV8856_write_cmos_sensor(0x5797, 0x10);
	OV8856_write_cmos_sensor(0x5798, 0x73);
	OV8856_write_cmos_sensor(0x5799, 0x73);
	OV8856_write_cmos_sensor(0x579a, 0x00);
	OV8856_write_cmos_sensor(0x579b, 0x28);
	OV8856_write_cmos_sensor(0x579c, 0x00);
	OV8856_write_cmos_sensor(0x579d, 0x16);
	OV8856_write_cmos_sensor(0x579e, 0x06);
	OV8856_write_cmos_sensor(0x579f, 0x20);
	OV8856_write_cmos_sensor(0x57a0, 0x04);
	OV8856_write_cmos_sensor(0x57a1, 0xa0);
	OV8856_write_cmos_sensor(0x0100, 0x01);
}

void OV8856CaptureSetting(void)
{
	OV8856_write_cmos_sensor(0x0100, 0x00);
	OV8856_write_cmos_sensor(0x3501, 0x9a);
	OV8856_write_cmos_sensor(0x3502, 0x20);
	OV8856_write_cmos_sensor(0x366e, 0x10);
	OV8856_write_cmos_sensor(0x3714, 0x23);
	OV8856_write_cmos_sensor(0x37c2, 0x04);
	OV8856_write_cmos_sensor(0x3800, 0x00);        
	OV8856_write_cmos_sensor(0x3801, 0x00);        
	OV8856_write_cmos_sensor(0x3802, 0x00);        
	OV8856_write_cmos_sensor(0x3803, 0x0c);        
	OV8856_write_cmos_sensor(0x3804, 0x0c);        
	OV8856_write_cmos_sensor(0x3805, 0xdf);        
	OV8856_write_cmos_sensor(0x3806, 0x09);        
	OV8856_write_cmos_sensor(0x3807, 0xa3);
	OV8856_write_cmos_sensor(0x3808, 0x0c);
	OV8856_write_cmos_sensor(0x3809, 0xc0);
	OV8856_write_cmos_sensor(0x380a, 0x09);
	OV8856_write_cmos_sensor(0x380b, 0x90);
	OV8856_write_cmos_sensor(0x380c, 0x07);
	OV8856_write_cmos_sensor(0x380d, 0x8c);
	OV8856_write_cmos_sensor(0x380e, 0x09);
	OV8856_write_cmos_sensor(0x380f, 0xb2);
	OV8856_write_cmos_sensor(0x3810, 0x00);
	OV8856_write_cmos_sensor(0x3811, 0x10);
	OV8856_write_cmos_sensor(0x3813, 0x04);
	OV8856_write_cmos_sensor(0x3814, 0x01);
	OV8856_write_cmos_sensor(0x3820, 0xc6);//0x80
	OV8856_write_cmos_sensor(0x3821, 0x46);
	OV8856_write_cmos_sensor(0x382a, 0x01);
	OV8856_write_cmos_sensor(0x4009, 0x0b);
	OV8856_write_cmos_sensor(0x5795, 0x02);
	OV8856_write_cmos_sensor(0x5796, 0x20);
	OV8856_write_cmos_sensor(0x5797, 0x20);
	OV8856_write_cmos_sensor(0x5798, 0xd5);
	OV8856_write_cmos_sensor(0x5799, 0xd5);
	OV8856_write_cmos_sensor(0x579a, 0x00);
	OV8856_write_cmos_sensor(0x579b, 0x50);
	OV8856_write_cmos_sensor(0x579c, 0x00);
	OV8856_write_cmos_sensor(0x579d, 0x2c);
	OV8856_write_cmos_sensor(0x579e, 0x0c);
	OV8856_write_cmos_sensor(0x579f, 0x40);
	OV8856_write_cmos_sensor(0x57a0, 0x09);
	OV8856_write_cmos_sensor(0x57a1, 0x40);
	OV8856_write_cmos_sensor(0x0100, 0x01);

}

void OV8856VideoSetting(void)
{
	OV8856_write_cmos_sensor(0x0100, 0x00);
	OV8856_write_cmos_sensor(0x3501, 0x9a);
	OV8856_write_cmos_sensor(0x3502, 0x20);
	OV8856_write_cmos_sensor(0x366e, 0x10);
	OV8856_write_cmos_sensor(0x3714, 0x23);
	OV8856_write_cmos_sensor(0x37c2, 0x04);
OV8856_write_cmos_sensor(0x3800, 0x00);
OV8856_write_cmos_sensor(0x3801, 0x00);
OV8856_write_cmos_sensor(0x3802, 0x01);
OV8856_write_cmos_sensor(0x3803, 0x38);
OV8856_write_cmos_sensor(0x3804, 0x0c);
OV8856_write_cmos_sensor(0x3805, 0xdf);
OV8856_write_cmos_sensor(0x3806, 0x08);
OV8856_write_cmos_sensor(0x3807, 0x77);
	OV8856_write_cmos_sensor(0x3808, 0x0c);
	OV8856_write_cmos_sensor(0x3809, 0xc0);
	OV8856_write_cmos_sensor(0x380a, 0x07);//0x09
	OV8856_write_cmos_sensor(0x380b, 0x2c);//0x90
	OV8856_write_cmos_sensor(0x380c, 0x07);
	OV8856_write_cmos_sensor(0x380d, 0x8c);
	OV8856_write_cmos_sensor(0x380e, 0x09);
	OV8856_write_cmos_sensor(0x380f, 0xb2);
	OV8856_write_cmos_sensor(0x3810, 0x00);
	OV8856_write_cmos_sensor(0x3811, 0x04);//0x10
	OV8856_write_cmos_sensor(0x3812, 0x00);//add
	OV8856_write_cmos_sensor(0x3813, 0x02);//0x04
	OV8856_write_cmos_sensor(0x3814, 0x01);
	OV8856_write_cmos_sensor(0x3820, 0xc6);//0x80
	OV8856_write_cmos_sensor(0x3821, 0x46);
	OV8856_write_cmos_sensor(0x382a, 0x01);
	OV8856_write_cmos_sensor(0x4009, 0x0b);
	OV8856_write_cmos_sensor(0x4601, 0x80);//add
	OV8856_write_cmos_sensor(0x5795, 0x02);
	OV8856_write_cmos_sensor(0x5796, 0x20);
	OV8856_write_cmos_sensor(0x5797, 0x20);
	OV8856_write_cmos_sensor(0x5798, 0xd5);
	OV8856_write_cmos_sensor(0x5799, 0xd5);
	OV8856_write_cmos_sensor(0x579a, 0x00);
	OV8856_write_cmos_sensor(0x579b, 0x50);
	OV8856_write_cmos_sensor(0x579c, 0x00);
	OV8856_write_cmos_sensor(0x579d, 0x00);//0x2c
	OV8856_write_cmos_sensor(0x579e, 0x0c);
	OV8856_write_cmos_sensor(0x579f, 0x40);
	OV8856_write_cmos_sensor(0x57a0, 0x07);//0x09
	OV8856_write_cmos_sensor(0x57a1, 0x40);
	OV8856_write_cmos_sensor(0x0100, 0x01);
}
static void OV8856_Sensor_Init(void)
{
	/* R1A_AM05, Ken Cui@2015/7/20
	;Xclk 24Mhz
	;pclk 144Mhz
	linelength = 1932(0x78C)
	framelength = 2482(0x9b2)
	grabwindow_width  = 1632
	grabwindow_height = 1224
	max_framerate = 300,	
	mipi_data_lp2hs_settle_dc =23?
	mipi_datarate = 720; Mbps
    */
	OV8856_write_cmos_sensor(0x0103, 0x01);        
	OV8856_write_cmos_sensor(0x0302, 0x3c);        
	OV8856_write_cmos_sensor(0x0303, 0x01);        
	OV8856_write_cmos_sensor(0x031e, 0x0c);        
	OV8856_write_cmos_sensor(0x3000, 0x00);        
	OV8856_write_cmos_sensor(0x300e, 0x00);        
	OV8856_write_cmos_sensor(0x3010, 0x00);        
	OV8856_write_cmos_sensor(0x3015, 0x84);        
	OV8856_write_cmos_sensor(0x3018, 0x72);        
	OV8856_write_cmos_sensor(0x3033, 0x24);        
	OV8856_write_cmos_sensor(0x3500, 0x00);        
	OV8856_write_cmos_sensor(0x3501, 0x4c);        
	OV8856_write_cmos_sensor(0x3502, 0xe0);        
	OV8856_write_cmos_sensor(0x3503, 0x08);        
	OV8856_write_cmos_sensor(0x3505, 0x83);        
	OV8856_write_cmos_sensor(0x3508, 0x01);        
	OV8856_write_cmos_sensor(0x3509, 0x80);        
	OV8856_write_cmos_sensor(0x350c, 0x00);        
	OV8856_write_cmos_sensor(0x350d, 0x80);        
	OV8856_write_cmos_sensor(0x350e, 0x04);        
	OV8856_write_cmos_sensor(0x350f, 0x00);        
	OV8856_write_cmos_sensor(0x3510, 0x00);        
	OV8856_write_cmos_sensor(0x3511, 0x02);        
	OV8856_write_cmos_sensor(0x3512, 0x00);        
	OV8856_write_cmos_sensor(0x3600, 0x72);        
	OV8856_write_cmos_sensor(0x3601, 0x40);        
	OV8856_write_cmos_sensor(0x3602, 0x30);        
	OV8856_write_cmos_sensor(0x3610, 0xc5);        
	OV8856_write_cmos_sensor(0x3611, 0x58);        
	OV8856_write_cmos_sensor(0x3612, 0x5c);        
	OV8856_write_cmos_sensor(0x3613, 0x5a);        
	OV8856_write_cmos_sensor(0x3614, 0x60);        
	OV8856_write_cmos_sensor(0x3628, 0xff);        
	OV8856_write_cmos_sensor(0x3629, 0xff);        
	OV8856_write_cmos_sensor(0x362a, 0xff);        
	OV8856_write_cmos_sensor(0x3633, 0x10);        
	OV8856_write_cmos_sensor(0x3634, 0x10);        
	OV8856_write_cmos_sensor(0x3635, 0x10);        
	OV8856_write_cmos_sensor(0x3636, 0x10); 
	OV8856_write_cmos_sensor(0x3645, 0x13);//for mipi  13 is ok
	OV8856_write_cmos_sensor(0x3663, 0x08);        
	OV8856_write_cmos_sensor(0x3669, 0x34);        
	OV8856_write_cmos_sensor(0x366e, 0x08);        
	OV8856_write_cmos_sensor(0x3706, 0x86);//0x87 Add by Yajun 20151109        
	OV8856_write_cmos_sensor(0x370b, 0x7e);//0x80 Add by Yajun 20151109       
	OV8856_write_cmos_sensor(0x3714, 0x27);        
	OV8856_write_cmos_sensor(0x3730, 0x12);        
	OV8856_write_cmos_sensor(0x3733, 0x10);        
	OV8856_write_cmos_sensor(0x3764, 0x00);        
	OV8856_write_cmos_sensor(0x3765, 0x00);        
	OV8856_write_cmos_sensor(0x3769, 0x62);        
	OV8856_write_cmos_sensor(0x376a, 0x2a);        
	OV8856_write_cmos_sensor(0x376b, 0x36);//0x30  Add by Yajun 20151109      
	OV8856_write_cmos_sensor(0x3780, 0x00);        
	OV8856_write_cmos_sensor(0x3781, 0x24);        
	OV8856_write_cmos_sensor(0x3782, 0x00);        
	OV8856_write_cmos_sensor(0x3783, 0x23);        
	OV8856_write_cmos_sensor(0x3798, 0x2f);        
	OV8856_write_cmos_sensor(0x37a1, 0x60);        
	OV8856_write_cmos_sensor(0x37a8, 0x6a);        
	OV8856_write_cmos_sensor(0x37ab, 0x3f);        
	OV8856_write_cmos_sensor(0x37c2, 0x14);        
	OV8856_write_cmos_sensor(0x37c3, 0xf1);        
	OV8856_write_cmos_sensor(0x37c9, 0x80);        
	OV8856_write_cmos_sensor(0x37cb, 0x03);        
	OV8856_write_cmos_sensor(0x37cc, 0x0a);        
	OV8856_write_cmos_sensor(0x37cd, 0x16);        
	OV8856_write_cmos_sensor(0x37ce, 0x1f);        
	OV8856_write_cmos_sensor(0x3800, 0x00);        
	OV8856_write_cmos_sensor(0x3801, 0x00);        
	OV8856_write_cmos_sensor(0x3802, 0x00);        
	OV8856_write_cmos_sensor(0x3803, 0x0c);        
	OV8856_write_cmos_sensor(0x3804, 0x0c);        
	OV8856_write_cmos_sensor(0x3805, 0xdf);        
	OV8856_write_cmos_sensor(0x3806, 0x09);        
	OV8856_write_cmos_sensor(0x3807, 0xa3);        
	OV8856_write_cmos_sensor(0x3808, 0x06);        
	OV8856_write_cmos_sensor(0x3809, 0x60);        
	OV8856_write_cmos_sensor(0x380a, 0x04);        
	OV8856_write_cmos_sensor(0x380b, 0xc8);        
	OV8856_write_cmos_sensor(0x380c, 0x07);        
	OV8856_write_cmos_sensor(0x380d, 0x8c);        
	OV8856_write_cmos_sensor(0x380e, 0x09);//04    
	OV8856_write_cmos_sensor(0x380f, 0xb2);//de    
	OV8856_write_cmos_sensor(0x3810, 0x00);        
	OV8856_write_cmos_sensor(0x3811, 0x08);        
	OV8856_write_cmos_sensor(0x3812, 0x00);        
	OV8856_write_cmos_sensor(0x3813, 0x02);        
	OV8856_write_cmos_sensor(0x3814, 0x03);        
	OV8856_write_cmos_sensor(0x3815, 0x01);        
	OV8856_write_cmos_sensor(0x3816, 0x00);        
	OV8856_write_cmos_sensor(0x3817, 0x00);        
	OV8856_write_cmos_sensor(0x3818, 0x00);        
	OV8856_write_cmos_sensor(0x3819, 0x00);        
	OV8856_write_cmos_sensor(0x3820, 0xd6);//0x90        
	OV8856_write_cmos_sensor(0x3821, 0x67);        
	OV8856_write_cmos_sensor(0x382a, 0x03);        
	OV8856_write_cmos_sensor(0x382b, 0x01);        
	OV8856_write_cmos_sensor(0x3830, 0x06);        
	OV8856_write_cmos_sensor(0x3836, 0x02);        
	OV8856_write_cmos_sensor(0x3862, 0x04);        
	OV8856_write_cmos_sensor(0x3863, 0x08);        
	OV8856_write_cmos_sensor(0x3cc0, 0x33);        
	OV8856_write_cmos_sensor(0x4001, 0xe0);        
	OV8856_write_cmos_sensor(0x4003, 0x40);        
	OV8856_write_cmos_sensor(0x3d85, 0x17);        
	OV8856_write_cmos_sensor(0x3d8c, 0x73);        
	OV8856_write_cmos_sensor(0x3d8d, 0xde);        
	OV8856_write_cmos_sensor(0x4008, 0x00);        
	OV8856_write_cmos_sensor(0x4009, 0x05);        
	OV8856_write_cmos_sensor(0x400f, 0x80);        
	OV8856_write_cmos_sensor(0x4010, 0xf0);        
	OV8856_write_cmos_sensor(0x4011, 0xff);        
	OV8856_write_cmos_sensor(0x4012, 0x02);        
	OV8856_write_cmos_sensor(0x4013, 0x01);        
	OV8856_write_cmos_sensor(0x4014, 0x01);        
	OV8856_write_cmos_sensor(0x4015, 0x01);        
	OV8856_write_cmos_sensor(0x4042, 0x00);        
	OV8856_write_cmos_sensor(0x4043, 0x80);        
	OV8856_write_cmos_sensor(0x4044, 0x00);        
	OV8856_write_cmos_sensor(0x4045, 0x80);        
	OV8856_write_cmos_sensor(0x4046, 0x00);        
	OV8856_write_cmos_sensor(0x4047, 0x80);        
	OV8856_write_cmos_sensor(0x4048, 0x00);        
	OV8856_write_cmos_sensor(0x4049, 0x80);        
	OV8856_write_cmos_sensor(0x4041, 0x03);        
	OV8856_write_cmos_sensor(0x404c, 0x20);        
	OV8856_write_cmos_sensor(0x404d, 0x00);        
	OV8856_write_cmos_sensor(0x404e, 0x20);        
	OV8856_write_cmos_sensor(0x4203, 0x80);        
	OV8856_write_cmos_sensor(0x4307, 0x30);        
	OV8856_write_cmos_sensor(0x4317, 0x00);        
	OV8856_write_cmos_sensor(0x4503, 0x08);        
	OV8856_write_cmos_sensor(0x4601, 0x80);
	OV8856_write_cmos_sensor(0x4816, 0x53);        
	OV8856_write_cmos_sensor(0x4837, 0x16);        
	OV8856_write_cmos_sensor(0x481b, 0x58);        
	OV8856_write_cmos_sensor(0x481f, 0x27);        
	OV8856_write_cmos_sensor(0x5000, 0x77);        
	OV8856_write_cmos_sensor(0x5001, 0x0e);//0x0a       
	OV8856_write_cmos_sensor(0x5004, 0x04);        
	OV8856_write_cmos_sensor(0x502e, 0x00);//0x03        
	OV8856_write_cmos_sensor(0x5030, 0x41);        
	OV8856_write_cmos_sensor(0x5795, 0x00);        
	OV8856_write_cmos_sensor(0x5796, 0x10);        
	OV8856_write_cmos_sensor(0x5797, 0x10);        
	OV8856_write_cmos_sensor(0x5798, 0x73);        
	OV8856_write_cmos_sensor(0x5799, 0x73);        
	OV8856_write_cmos_sensor(0x579a, 0x00);        
	OV8856_write_cmos_sensor(0x579b, 0x28);        
	OV8856_write_cmos_sensor(0x579c, 0x00);        
	OV8856_write_cmos_sensor(0x579d, 0x16);        
	OV8856_write_cmos_sensor(0x579e, 0x06);        
	OV8856_write_cmos_sensor(0x579f, 0x20);        
	OV8856_write_cmos_sensor(0x57a0, 0x04);        
	OV8856_write_cmos_sensor(0x57a1, 0xa0);        
	OV8856_write_cmos_sensor(0x5780, 0x14);        
	OV8856_write_cmos_sensor(0x5781, 0x0f);        
	OV8856_write_cmos_sensor(0x5782, 0x44);        
	OV8856_write_cmos_sensor(0x5783, 0x02);        
	OV8856_write_cmos_sensor(0x5784, 0x01);        
	OV8856_write_cmos_sensor(0x5785, 0x01);        
	OV8856_write_cmos_sensor(0x5786, 0x00);        
	OV8856_write_cmos_sensor(0x5787, 0x04);        
	OV8856_write_cmos_sensor(0x5788, 0x02);        
	OV8856_write_cmos_sensor(0x5789, 0x0f);        
	OV8856_write_cmos_sensor(0x578a, 0xfd);        
	OV8856_write_cmos_sensor(0x578b, 0xf5);        
	OV8856_write_cmos_sensor(0x578c, 0xf5);        
	OV8856_write_cmos_sensor(0x578d, 0x03);        
	OV8856_write_cmos_sensor(0x578e, 0x08);        
	OV8856_write_cmos_sensor(0x578f, 0x0c);        
	OV8856_write_cmos_sensor(0x5790, 0x08);        
	OV8856_write_cmos_sensor(0x5791, 0x04);        
	OV8856_write_cmos_sensor(0x5792, 0x00);        
	OV8856_write_cmos_sensor(0x5793, 0x52);        
	OV8856_write_cmos_sensor(0x5794, 0xa3);        
	OV8856_write_cmos_sensor(0x5a08, 0x02);        
	OV8856_write_cmos_sensor(0x5b00, 0x02);        
	OV8856_write_cmos_sensor(0x5b01, 0x10);        
	OV8856_write_cmos_sensor(0x5b02, 0x03);        
	OV8856_write_cmos_sensor(0x5b03, 0xcf);        
	OV8856_write_cmos_sensor(0x5b05, 0x6c);        
	OV8856_write_cmos_sensor(0x5e00, 0x00);   
	OV8856_write_cmos_sensor(0x0100, 0x01);        


}


UINT32 OV8856Open(void)
{

	volatile signed int i;
	kal_uint16 sensor_id = 0;

	OV8856DB("OV8856 Open enter :\n ");
	OV8856_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mdelay(2);

	for(i=0;i<2;i++)
	{
		sensor_id = (OV8856_read_cmos_sensor(0x300B)<<8)|OV8856_read_cmos_sensor(0x300C);
		OV8856DB("OV8856 READ ID :%x",sensor_id);
		if(sensor_id != OV8856_SENSOR_ID)
		{
			return ERROR_SENSOR_CONNECT_FAIL;
		}else
			break;
	}
	
	OV8856_Sensor_Init();	
	mdelay(10);
    OV8856_Init_Para();
	mdelay(5);
	
#ifdef OV8856R1AOTP                 
            apply_otp(&ov8856_otp_ptr);
#endif

    OV8856SetFlipMirror(IMAGE_HV_MIRROR);
	#ifdef OV8856_DEBUG
		OV8856DB("[OV8856Open] enter and exit."); 
	#endif

    return ERROR_NONE;
}

UINT32 OV8856GetSensorID(UINT32 *sensorID)
{
    int  retry = 2;

	OV8856DB("OV8856GetSensorID enter :\n ");

    int i;
    const kal_uint16 sccb_writeid[] = {OV8856MIPI_WRITE_ID1};
    for (i=0;i<(sizeof(sccb_writeid)/sizeof(sccb_writeid[0]));i++){
    do {
        ov8856.write_id = sccb_writeid[i];
        ov8856.read_id = (sccb_writeid[i]|0x01);
        *sensorID = (OV8856_read_cmos_sensor(0x300B)<<8)|OV8856_read_cmos_sensor(0x300C);
        if (*sensorID == OV8856_SENSOR_ID)
        	{
        		OV8856DB("Sensor ID = 0x%04x\n", *sensorID);
			#ifdef OV8856R1AOTP                  
            		read_otp(&ov8856_otp_ptr);
			#endif

            	break;
        	}
        OV8856DB("[OV8856GetSensorID] Read Sensor ID Fail = 0x%04x ,i2c addr is:0x%02x====\n ", *sensorID,ov8856.write_id);
        retry--;
    } while (retry > 0);
    }
    if (*sensorID != OV8856_SENSOR_ID) {
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

    return ERROR_NONE;
}

UINT32 OV8856Close(void)
{
	#ifdef OV8856_DEBUG
		OV8856DB("[OV8856Close]enter and exit.\n");
	#endif

    return ERROR_NONE;
}


void OV8856SetFlipMirror(kal_int32 image_mirror)
{
         OV8856DB("image_mirror = %d\n", image_mirror);

 

         /********************************************************

            *

            *   0x3820[2] ISP Vertical flip

            *   0x3820[1] Sensor Vertical flip

            *

            *   0x3821[2] ISP Horizontal mirror

            *   0x3821[1] Sensor Horizontal mirror

            *

            *   ISP and Sensor flip or mirror register bit should be the same!!

            *

            ********************************************************/

         

         switch (image_mirror) {

                   case IMAGE_NORMAL:

                            OV8856_write_cmos_sensor(0x3820,((OV8856_read_cmos_sensor(0x3820) & 0xB9) | 0x00));

                            OV8856_write_cmos_sensor(0x3821,((OV8856_read_cmos_sensor(0x3821) & 0xF9) | 0x06));

                            OV8856_write_cmos_sensor(0x502e,((OV8856_read_cmos_sensor(0x502e) & 0xFC) | 0x03));

                            OV8856_write_cmos_sensor(0x5001,((OV8856_read_cmos_sensor(0x5001) & 0xFB) | 0x00));         

                            OV8856_write_cmos_sensor(0x5004,((OV8856_read_cmos_sensor(0x5004) & 0xFB) | 0x04));

                            OV8856_write_cmos_sensor(0x376b,0x30);         

                            break;

                   case IMAGE_H_MIRROR:

                            OV8856_write_cmos_sensor(0x3820,((OV8856_read_cmos_sensor(0x3820) & 0xB9) | 0x00));

                            OV8856_write_cmos_sensor(0x3821,((OV8856_read_cmos_sensor(0x3821) & 0xF9) | 0x00));

                            OV8856_write_cmos_sensor(0x502e,((OV8856_read_cmos_sensor(0x502e) & 0xFC) | 0x03));

                            OV8856_write_cmos_sensor(0x5001,((OV8856_read_cmos_sensor(0x5001) & 0xFB) | 0x00));         

                            OV8856_write_cmos_sensor(0x5004,((OV8856_read_cmos_sensor(0x5004) & 0xFB) | 0x00));

                            OV8856_write_cmos_sensor(0x376b,0x30);         

                            break;

                   case IMAGE_V_MIRROR:

                            OV8856_write_cmos_sensor(0x3820,((OV8856_read_cmos_sensor(0x3820) & 0xB9) | 0x46));

                            OV8856_write_cmos_sensor(0x3821,((OV8856_read_cmos_sensor(0x3821) & 0xF9) | 0x06));

                            OV8856_write_cmos_sensor(0x502e,((OV8856_read_cmos_sensor(0x502e) & 0xFC) | 0x00));

                            OV8856_write_cmos_sensor(0x5001,((OV8856_read_cmos_sensor(0x5001) & 0xFB) | 0x04));         

                            OV8856_write_cmos_sensor(0x5004,((OV8856_read_cmos_sensor(0x5004) & 0xFB) | 0x04));

                            OV8856_write_cmos_sensor(0x376b,0x36);

                            break;

                   case IMAGE_HV_MIRROR:

                            OV8856_write_cmos_sensor(0x3820,((OV8856_read_cmos_sensor(0x3820) & 0xB9) | 0x46));

                            OV8856_write_cmos_sensor(0x3821,((OV8856_read_cmos_sensor(0x3821) & 0xF9) | 0x00));

                            OV8856_write_cmos_sensor(0x502e, 0x00);

                            OV8856_write_cmos_sensor(0x5001, 0x0e);         

                            OV8856_write_cmos_sensor(0x5004, 0x00);

                            OV8856_write_cmos_sensor(0x376b, 0x36);

                            break;

                   default:

                            OV8856DB("Error image_mirror setting\n");

         }


}


kal_uint32 OV8856_SET_FrameLength_ByVideoMode(UINT16 Video_TargetFps)
{

    UINT32 frameRate = 0;
	kal_uint32 MIN_FrameLength=0;
	
	if(ov8856.OV8856AutoFlickerMode == KAL_TRUE)
	{
		if (Video_TargetFps==30)
			frameRate= OV8856_AUTOFLICKER_OFFSET_30;
		else if(Video_TargetFps==15)
			frameRate= OV8856_AUTOFLICKER_OFFSET_15;
		else
			frameRate=Video_TargetFps*10;
	
		MIN_FrameLength = (ov8856.videoPclk*10000)/(OV8856_VIDEO_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/frameRate*10;
	}
	else
		MIN_FrameLength = (ov8856.videoPclk*10000) /(OV8856_VIDEO_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/Video_TargetFps;

     return MIN_FrameLength;


}



UINT32 OV8856SetVideoMode(UINT16 u2FrameRate)
{
    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV8856DB("[OV8856SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov8856mipiraw_drv_lock);
	OV8856_VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov8856mipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV8856DB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV8856DB("abmornal frame rate seting,pay attention~\n");

    if(ov8856.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {

        MIN_Frame_length = OV8856_SET_FrameLength_ByVideoMode(u2FrameRate);

		if((MIN_Frame_length <=OV8856_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV8856_VIDEO_PERIOD_LINE_NUMS;
			OV8856DB("[OV8856SetVideoMode]current fps = %d\n", (ov8856.videoPclk*10000)  /(OV8856_VIDEO_PERIOD_PIXEL_NUMS)/OV8856_VIDEO_PERIOD_LINE_NUMS);
		}
		OV8856DB("[OV8856SetVideoMode]current fps (10 base)= %d\n", (ov8856.videoPclk*10000)*10/(OV8856_VIDEO_PERIOD_PIXEL_NUMS + ov8856.DummyPixels)/MIN_Frame_length);
		extralines = MIN_Frame_length - OV8856_VIDEO_PERIOD_LINE_NUMS;
		
		spin_lock(&ov8856mipiraw_drv_lock);
		ov8856.DummyPixels = 0;//define dummy pixels and lines
		ov8856.DummyLines = extralines ;
		spin_unlock(&ov8856mipiraw_drv_lock);
		
		OV8856_SetDummy(0, extralines);
    }
	
	OV8856DB("[OV8856SetVideoMode]MIN_Frame_length=%d,ov8856.DummyLines=%d\n",MIN_Frame_length,ov8856.DummyLines);

    return KAL_TRUE;
}


UINT32 OV8856SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	if(bEnable) {   
		spin_lock(&ov8856mipiraw_drv_lock);
		ov8856.OV8856AutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov8856mipiraw_drv_lock);
        OV8856DB("OV8856 Enable Auto flicker\n");
    } else {
    	spin_lock(&ov8856mipiraw_drv_lock);
        ov8856.OV8856AutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov8856mipiraw_drv_lock);
        OV8856DB("OV8856 Disable Auto flicker\n");
    }

    return ERROR_NONE;
}


UINT32 OV8856SetTestPatternMode(kal_bool bEnable)
{
    OV8856DB("[OV8856SetTestPatternMode] Test pattern enable:%d\n", bEnable);
    if(bEnable == KAL_TRUE)
    {
        OV8856_During_testpattern = KAL_TRUE;
		OV8856_write_cmos_sensor(0x5E00,0x80);
    }
	else
	{
        OV8856_During_testpattern = KAL_FALSE;
		OV8856_write_cmos_sensor(0x5E00,0x00);
	}

    return ERROR_NONE;
}


/*************************************************************************
*
* DESCRIPTION:
* INTERFACE FUNCTION, FOR USER TO SET MAX  FRAMERATE;
* 
*************************************************************************/
UINT32 OV8856MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV8856DB("OV8856MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV8856_PREVIEW_PCLK;
			lineLength = OV8856_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8856_PV_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8856mipiraw_drv_lock);
			ov8856.sensorMode = SENSOR_MODE_PREVIEW;
			spin_unlock(&ov8856mipiraw_drv_lock);
			OV8856_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV8856_VIDEO_PCLK;
			lineLength = OV8856_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8856_VIDEO_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8856mipiraw_drv_lock);
			ov8856.sensorMode = SENSOR_MODE_VIDEO;
			spin_unlock(&ov8856mipiraw_drv_lock);
			OV8856_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV8856_CAPTURE_PCLK;
			lineLength = OV8856_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV8856_FULL_PERIOD_LINE_NUMS;
			if(dummyLine<0)
				dummyLine = 0;
			spin_lock(&ov8856mipiraw_drv_lock);
			ov8856.sensorMode = SENSOR_MODE_CAPTURE;
			spin_unlock(&ov8856mipiraw_drv_lock);
			OV8856_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW:
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE:   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;

}


UINT32 OV8856MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = OV8856_MAX_FPS_PREVIEW;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			 *pframeRate = OV8856_MAX_FPS_CAPTURE;
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = OV8856_MAX_FPS_CAPTURE;
			break;		
		default:
			break;
	}

	return ERROR_NONE;

}


void OV8856_NightMode(kal_bool bEnable)
{
	
	#ifdef OV8856_DEBUG
	OV8856DB("[OV8856_NightMode]enter and exit.\n");
	#endif
}



UINT32 OV8856Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV8856DB("OV8856Preview enter:");

	OV8856PreviewSetting();

	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.sensorMode = SENSOR_MODE_PREVIEW; 
	ov8856.DummyPixels = 0;
	ov8856.DummyLines = 0 ;
	OV8856_FeatureControl_PERIOD_PixelNum=OV8856_PV_PERIOD_PIXEL_NUMS+ ov8856.DummyPixels;
	OV8856_FeatureControl_PERIOD_LineNum=OV8856_PV_PERIOD_LINE_NUMS+ov8856.DummyLines;
	//TODO~
	//ov8856.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov8856mipiraw_drv_lock);
	
	//OV8856SetFlipMirror(sensor_config_data->SensorImageMirror);
	//TODO~
    mdelay(10);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
    
    OV8856SetFlipMirror(IMAGE_HV_MIRROR);
	OV8856DB("OV8856Preview exit:\n");

	  
    return ERROR_NONE;
}


UINT32 OV8856Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV8856DB("OV8856Video enter:");

	OV8856VideoSetting();

	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.sensorMode = SENSOR_MODE_VIDEO;
	OV8856_FeatureControl_PERIOD_PixelNum=OV8856_VIDEO_PERIOD_PIXEL_NUMS+ ov8856.DummyPixels;
	OV8856_FeatureControl_PERIOD_LineNum=OV8856_VIDEO_PERIOD_LINE_NUMS+ov8856.DummyLines;
	ov8856.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov8856mipiraw_drv_lock);
	
	//OV8856SetFlipMirror(sensor_config_data->SensorImageMirror);

    mdelay(10);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY
    OV8856SetFlipMirror(IMAGE_HV_MIRROR);
	OV8856DB("OV8856Video exit:\n");
    return ERROR_NONE;
}


UINT32 OV8856Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                            MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

 	kal_uint32 shutter = ov8856.shutter;
	kal_uint32 temp_data;


	OV8856DB("OV8856Capture enter:\n");

	OV8856CaptureSetting();

	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.sensorMode = SENSOR_MODE_CAPTURE;
	//TODO~
	//ov8856.imgMirror = sensor_config_data->SensorImageMirror;
	ov8856.DummyPixels = 0;
	ov8856.DummyLines = 0 ;
	OV8856_FeatureControl_PERIOD_PixelNum = OV8856_FULL_PERIOD_PIXEL_NUMS + ov8856.DummyPixels;
	OV8856_FeatureControl_PERIOD_LineNum = OV8856_FULL_PERIOD_LINE_NUMS + ov8856.DummyLines;
	spin_unlock(&ov8856mipiraw_drv_lock);

	//OV8856SetFlipMirror(sensor_config_data->SensorImageMirror);
    mdelay(10);//THIS DELAY SHOULD BE NEED BY CTS OR MONKEY  
    OV8856SetFlipMirror(IMAGE_HV_MIRROR);
	#if 0
	if(OV8856_During_testpattern == KAL_TRUE)
	{
		//TODO~
		//Test pattern
		OV8856_write_cmos_sensor(0x5E00,0x80);
	}
	#endif
	OV8856DB("OV8856Capture exit:\n");
    return ERROR_NONE;

}	


UINT32 OV8856GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV8856DB("OV8856GetResolution!!\n");

	pSensorResolution->SensorPreviewWidth	= OV8856_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV8856_IMAGE_SENSOR_PV_HEIGHT;
	
    pSensorResolution->SensorFullWidth		= OV8856_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV8856_IMAGE_SENSOR_FULL_HEIGHT;
	
    pSensorResolution->SensorVideoWidth		= OV8856_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV8856_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   

UINT32 OV8856GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
    OV8856DB("OV8856GetInfo enter!!\n");
	spin_lock(&ov8856mipiraw_drv_lock);
	ov8856.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov8856mipiraw_drv_lock);

    pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
   
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;	    
    pSensorInfo->AESensorGainDelayFrame = 0;
    pSensorInfo->AEISPGainDelayFrame = 2;
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
	pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;


    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8856_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8856_PV_Y_START;
			
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8856_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV8856_VIDEO_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;//0,4,14,32,40
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8856_FULL_X_START;	
            pSensorInfo->SensorGrabStartY = OV8856_FULL_Y_START;	

            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV8856_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV8856_PV_Y_START;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 30;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }
	
    memcpy(pSensorConfigData, &OV8856SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
    OV8856DB("OV8856GetInfo exit!!\n");

    return ERROR_NONE;
}   /* OV8856GetInfo() */



UINT32 OV8856Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov8856mipiraw_drv_lock);
		OV8856CurrentScenarioId = ScenarioId;
		spin_unlock(&ov8856mipiraw_drv_lock);
		
		OV8856DB("[OV8856Control]OV8856CurrentScenarioId=%d\n",OV8856CurrentScenarioId);

	switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV8856Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV8856Video(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV8856Capture(pImageWindow, pSensorConfigData);
            break;

        default:
            return ERROR_INVALID_SCENARIO_ID;

    }
    return ERROR_NONE;
} /* OV8856Control() */


UINT32 OV8856FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
                                                                UINT8 *pFeaturePara,UINT32 *pFeatureParaLen)
{
    UINT16 *pFeatureReturnPara16=(UINT16 *) pFeaturePara;
    UINT16 *pFeatureData16=(UINT16 *) pFeaturePara;
    UINT32 *pFeatureReturnPara32=(UINT32 *) pFeaturePara;
    UINT32 *pFeatureData32=(UINT32 *) pFeaturePara;
    UINT32 SensorRegNumber;
    UINT32 i;
    PNVRAM_SENSOR_DATA_STRUCT pSensorDefaultData=(PNVRAM_SENSOR_DATA_STRUCT) pFeaturePara;
    MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData=(MSDK_SENSOR_CONFIG_STRUCT *) pFeaturePara;
    MSDK_SENSOR_REG_INFO_STRUCT *pSensorRegData=(MSDK_SENSOR_REG_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_GROUP_INFO_STRUCT *pSensorGroupInfo=(MSDK_SENSOR_GROUP_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ITEM_INFO_STRUCT *pSensorItemInfo=(MSDK_SENSOR_ITEM_INFO_STRUCT *) pFeaturePara;
    MSDK_SENSOR_ENG_INFO_STRUCT	*pSensorEngInfo=(MSDK_SENSOR_ENG_INFO_STRUCT *) pFeaturePara;

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV8856_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV8856_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV8856_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV8856_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV8856CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV8856_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV8856_VIDEO_PCLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV8856_CAPTURE_PCLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV8856_PREVIEW_PCLK;
					*pFeatureParaLen=4;
					break;
			}
		    break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV8856_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV8856_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:  
           	OV8856_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV8856_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV8856_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV8856_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov8856mipiraw_drv_lock);
                OV8856SensorCCT[i].Addr=*pFeatureData32++;
                OV8856SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov8856mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8856SensorCCT[i].Addr;
                *pFeatureData32++=OV8856SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov8856mipiraw_drv_lock);
                OV8856SensorReg[i].Addr=*pFeatureData32++;
                OV8856SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov8856mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV8856SensorReg[i].Addr;
                *pFeatureData32++=OV8856SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV8856_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV8856SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV8856SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV8856SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV8856_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV8856_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV8856_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV8856_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV8856_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV8856_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
			//TODO~
            pSensorEngInfo->SensorId = 129;
            pSensorEngInfo->SensorType = CMOS_SENSOR;
            pSensorEngInfo->SensorOutputDataFormat=SENSOR_OUTPUT_FORMAT_RAW_B;
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ENG_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *pFeatureReturnPara32=LENS_DRIVER_ID_DO_NOT_CARE;
            *pFeatureParaLen=4;
            break;

        case SENSOR_FEATURE_INITIALIZE_AF:
            break;
        case SENSOR_FEATURE_CONSTANT_AF:
            break;
        case SENSOR_FEATURE_MOVE_FOCUS_LENS:
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
			OV8856SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV8856GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
			//TODO~
			OV8856SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV8856MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV8856MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_SET_TEST_PATTERN:
			//TODO~
			OV8856SetTestPatternMode((BOOL)*pFeatureData16);
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
			*pFeatureReturnPara32=OV8856_TEST_PATTERN_CHECKSUM; 		  
			*pFeatureParaLen=4; 							
		     break;
        default:
            break;
    }
    return ERROR_NONE;
}	


SENSOR_FUNCTION_STRUCT	SensorFuncOV8856=
{
    OV8856Open,
    OV8856GetInfo,
    OV8856GetResolution,
    OV8856FeatureControl,
    OV8856Control,
    OV8856Close
};

UINT32 OV8856_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV8856;

    return ERROR_NONE;
}  

