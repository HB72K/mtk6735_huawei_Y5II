/*******************************************************************************************/


/*******************************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/xlog.h>
#include <asm/atomic.h>
#include <asm/system.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "s5k4h8mipiraw_Sensor.h"
#include "s5k4h8mipiraw_Camera_Sensor_para.h"
#include "s5k4h8mipiraw_CameraCustomized.h"

#define mDELAY(ms)  mdelay(ms)
#define Sleep(ms) mdelay(ms)

#define S5K4H8_DEBUG
#ifdef S5K4H8_DEBUG
#define LOG_TAG (__FUNCTION__)
#define SENSORDB(fmt,arg...) xlog_printk(ANDROID_LOG_DEBUG , LOG_TAG, fmt, ##arg)  	//printk(LOG_TAG "%s: " fmt "\n", __FUNCTION__ ,##arg)
#else
#define SENSORDB(fmt,arg...)  
#endif

#define SENSOR_PCLK_PREVIEW  	28000*10000 //26000*10000  //27600*10000
#define SENSOR_PCLK_VIDEO  		SENSOR_PCLK_PREVIEW //26000*10000
#define SENSOR_PCLK_CAPTURE  	SENSOR_PCLK_PREVIEW //26000*10000
#define SENSOR_PCLK_ZSD  		SENSOR_PCLK_CAPTURE

#define S5K4H8_TEST_PATTERN_CHECKSUM (0xc0c10284)

MSDK_SENSOR_CONFIG_STRUCT S5K4H8SensorConfigData;

kal_uint32 S5K4H8_FAC_SENSOR_REG;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT S5K4H8SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT S5K4H8SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static DEFINE_SPINLOCK(S5K4H8mipiraw_drv_lock);
static MSDK_SCENARIO_ID_ENUM s_S5K4H8CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
static S5K4H8_PARA_STRUCT S5K4H8;
static kal_uint16 S5K4H8_slave_addr = S5K4H8MIPI_WRITE_ID;

extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
UINT32 S5K4H8MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate);

kal_uint32 S5K4H8_FRAME_LENGTH_COUNT = S5K4H8_PV_PERIOD_LINE_NUMS;
inline kal_uint16 S5K4H8_read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;
	char puSendCmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
	iReadRegI2C(puSendCmd , 2, (u8*)&get_byte,2,S5K4H8_slave_addr);
	return ((get_byte<<8)&0xff00)|((get_byte>>8)&0x00ff);
}
static kal_uint16 read_cmos_sensor_8(kal_uint16 addr)
{
    kal_uint16 get_byte=0;
    char pusendcmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    iReadRegI2C(pusendcmd , 2, (u8*)&get_byte,1,S5K4H8_slave_addr);
    return get_byte;
}

inline void S5K4H8_wordwrite_cmos_sensor(u16 addr, u32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF) ,  (char)(para >> 8),	(char)(para & 0xFF) };
	iWriteRegI2C(puSendCmd , 4,S5K4H8_slave_addr);
}

inline void S5K4H8_bytewrite_cmos_sensor(u16 addr, u32 para)
{
	char puSendCmd[4] = {(char)(addr >> 8) , (char)(addr & 0xFF)  ,	(char)(para & 0xFF) };
	iWriteRegI2C(puSendCmd , 3,S5K4H8_slave_addr);
}

static inline kal_uint32 GetScenarioLinelength(void)
{
	kal_uint32 u4Linelength=S5K4H8_PV_PERIOD_PIXEL_NUMS; //+S5K4H8.DummyPixels;
	switch(s_S5K4H8CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			u4Linelength=S5K4H8_PV_PERIOD_PIXEL_NUMS; //+S5K4H8.DummyPixels;
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			u4Linelength=S5K4H8_VIDEO_PERIOD_PIXEL_NUMS; //+S5K4H8.DummyPixels;
		break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			u4Linelength=S5K4H8_ZSD_PERIOD_PIXEL_NUMS; //+S5K4H8.DummyPixels;
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			u4Linelength=S5K4H8_FULL_PERIOD_PIXEL_NUMS; //+S5K4H8.DummyPixels;
		break;
		default:
		break;
	}
	//SENSORDB("u4Linelength=%d\n",u4Linelength);
	return u4Linelength;		
}

static inline kal_uint32 GetScenarioPixelClock(void)
{
	kal_uint32 Pixelcloclk = S5K4H8.pvPclk;
	switch(s_S5K4H8CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			Pixelcloclk = S5K4H8.pvPclk;
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			Pixelcloclk = S5K4H8.m_vidPclk;
		break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			Pixelcloclk = S5K4H8.capPclk;
		break;
		default:
		break;
	}
	//SENSORDB("u4Linelength=%d\n",u4Linelength);
	return Pixelcloclk;		
}


static inline kal_uint32 GetScenarioFramelength(void)
{
	kal_uint32 u4Framelength=S5K4H8_PV_PERIOD_LINE_NUMS; //+S5K4H8.DummyLines ;
	switch(s_S5K4H8CurrentScenarioId)
	{
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			u4Framelength=S5K4H8_PV_PERIOD_LINE_NUMS; //+S5K4H8.DummyLines ;
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			u4Framelength=S5K4H8_VIDEO_PERIOD_LINE_NUMS; //+S5K4H8.DummyLines ;
		break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			u4Framelength=S5K4H8_ZSD_PERIOD_LINE_NUMS; //+S5K4H8.DummyLines ;
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			u4Framelength=S5K4H8_FULL_PERIOD_LINE_NUMS; //+S5K4H8.DummyLines ;
		break;
		default:
		break;
	}
	//SENSORDB("u4Framelength=%d\n",u4Framelength);
	return u4Framelength;		
}


/*************************************************************************
* FUNCTION
*	Huawei_otp_read
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
//HuangJiaLiang add for OTP read
#define otp_ofilm
#ifdef otp_ofilm
struct S5K4H8_MIPI_otp_struct{
 	int flag;
	int Module_integrator_id;
	int lens_id;
	int production_year;
	int production_month;
	int production_day;
	int rg_ratio;
	int bg_ratio;
	int lenc[240];
	int checksum;
	int vcm_start;
	int vcm_end;
	int vcm_dir;
	int module_NO;
};
static struct S5K4H8_MIPI_otp_struct S5K4H8_otp_data;
//flag value
int S5K4H8_read_otp(struct S5K4H8_MIPI_otp_struct *otp_ptr){
	int addr,temp,i,stram_flag;
	kal_uint16 otp_flag;
        stram_flag = 0;
	S5K4H8_bytewrite_cmos_sensor(0X0100,0X01); //stream on
        mdelay(10);
	S5K4H8_bytewrite_cmos_sensor(0x0A02,0x0F);//set 15page 
	S5K4H8_wordwrite_cmos_sensor(0x0A00,0X0100);//set start read addr

        otp_flag = read_cmos_sensor_8(0x0A04);
	printk("huang----1-----%X\n",otp_flag);
        i = 0;
     	addr = 0;
        if(otp_flag == 0x10 ){
	   addr = 0x0A05;  //set base_addr
           i = 1;
	}
	else if(otp_flag  == 0xf1){
	   addr = 0x0A25;
           i = 2;
	} 
        if(addr != 0){
           (*otp_ptr).flag = 0xc0;
           (*otp_ptr).Module_integrator_id = read_cmos_sensor_8(addr);
           (*otp_ptr).production_year = read_cmos_sensor_8(addr + 1);
           (*otp_ptr).production_month = read_cmos_sensor_8(addr + 2);
           (*otp_ptr).production_day = read_cmos_sensor_8(addr + 3);
	    temp=addr+4;
	      (*otp_ptr).rg_ratio = ((read_cmos_sensor_8(temp) << 8) | (read_cmos_sensor_8(temp+1)));
           (*otp_ptr).bg_ratio = ((read_cmos_sensor_8(temp+2) << 8) | (read_cmos_sensor_8(temp+3)));
           
	}
        else{
           (*otp_ptr).flag = 0;
           (*otp_ptr).Module_integrator_id = 0;
           (*otp_ptr).production_year = 0;
           (*otp_ptr).production_month = 0;
           (*otp_ptr).production_day = 0;
           (*otp_ptr).rg_ratio = 0;
           (*otp_ptr).bg_ratio = 0;
	}      
        
        S5K4H8_wordwrite_cmos_sensor(0x0A00,0X0000);//set read end 
	printk("lili>>>>>>>>rg_ratio = 0x%x,bg_ratio = 0x%x\n",(*otp_ptr).rg_ratio,(*otp_ptr).bg_ratio);
        return (*otp_ptr).flag;
        
}
#define GAIN_DEFAULT       0x0100
int S5K4H8_write_otp(struct S5K4H8_MIPI_otp_struct *otp_ptr){
      
	int RG_Ratio_Typical = 0x28c,BG_Ratio_Typical = 0X2BA;
        int rg,bg,r_ratio,b_ratio,g_ratio,Base_gain,temp,i;
	if((*otp_ptr).flag == 0xc0)
	{
		rg = (*otp_ptr).rg_ratio;
		bg = (*otp_ptr).bg_ratio;
		r_ratio = 512 * (RG_Ratio_Typical) /(rg);
		b_ratio = 512 * (BG_Ratio_Typical) /(bg);
		int R_GAIN;
		int B_GAIN;
		int Gr_GAIN;
		int Gb_GAIN;
		int G_GAIN;
		if(r_ratio >= 512 )
		{
			if(b_ratio>=512) 
			{
				R_GAIN = (int)(GAIN_DEFAULT * r_ratio / 512);
				G_GAIN = GAIN_DEFAULT;	
				B_GAIN = (int)(GAIN_DEFAULT * b_ratio / 512);
			}
			else
			{
				R_GAIN =  (int)(GAIN_DEFAULT * r_ratio / b_ratio);
				G_GAIN = (int)(GAIN_DEFAULT * 512 / b_ratio);
				B_GAIN = GAIN_DEFAULT;	
			}
		}
		else 			
		{
			if(b_ratio >= 512)
			{
				R_GAIN = GAIN_DEFAULT;	
				G_GAIN =(int)(GAIN_DEFAULT * 512 / r_ratio);
				B_GAIN =(int)(GAIN_DEFAULT *  b_ratio / r_ratio);
			} 
			else 
			{
				Gr_GAIN = (int)(GAIN_DEFAULT * 512 / r_ratio );
				Gb_GAIN = (int)(GAIN_DEFAULT * 512 / b_ratio );
			
				if(Gr_GAIN >= Gb_GAIN)
				{
					R_GAIN = GAIN_DEFAULT;
					G_GAIN = (int)(GAIN_DEFAULT * 512 / r_ratio );
					B_GAIN = (int)(GAIN_DEFAULT * b_ratio / r_ratio);
				} 
				else
				{
					R_GAIN =  (int)(GAIN_DEFAULT * r_ratio / b_ratio );
					G_GAIN = (int)(GAIN_DEFAULT * 512 / b_ratio );
					B_GAIN = GAIN_DEFAULT;
				}
			}	
		}
		printk("lili->>>>>>R_GAIN=%x,G_GAIN=%x,B_GAIN=%x\n",R_GAIN,G_GAIN,B_GAIN);
		S5K4H8_wordwrite_cmos_sensor(0x6028,0x4000);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x3058);
		S5K4H8_wordwrite_cmos_sensor(0x6F12, 0x0100);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x020E);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,(G_GAIN >> 8));
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x020F);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,G_GAIN & 0x00ff);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x0210);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,R_GAIN >> 8);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x0211);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,R_GAIN & 0x00ff);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x0212);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,B_GAIN >> 8);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x0213);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,B_GAIN & 0x00ff);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x0214);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,G_GAIN >> 8);
		S5K4H8_wordwrite_cmos_sensor(0x602A,0x0215);
		S5K4H8_bytewrite_cmos_sensor(0x6F12,G_GAIN & 0x00ff);
		//apply_wb_otp
		mdelay(10);        	
	}
	return 0;	
}

#endif




static inline void SetLinelength(kal_uint16 u2Linelength)
{
	SENSORDB("u4Linelength=%d\n",u2Linelength);
	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x01);	 //Grouped parameter hold	 
	S5K4H8_wordwrite_cmos_sensor(0x342,u2Linelength);		
	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x00);	 //Grouped parameter release	
}

static inline void SetFramelength(kal_uint16 u2Framelength)
{
	SENSORDB("u2Framelength=%d\n",u2Framelength);
	
	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.maxExposureLines = u2Framelength;
	spin_unlock(&S5K4H8mipiraw_drv_lock);
	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x01);	 //Grouped parameter hold	 
	S5K4H8_wordwrite_cmos_sensor(0x0340,u2Framelength);		
	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x00);	 //Grouped parameter release	
	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8_FRAME_LENGTH_COUNT = u2Framelength;
	spin_unlock(&S5K4H8mipiraw_drv_lock);
}



void S5K4H8_write_shutter(kal_uint32 shutter)
{
	kal_uint16 frame_length = 0, line_length = 0, framerate = 0 , frame_length_min = 0;	
	kal_uint32 pixelclock = 0;
	unsigned long flags;

	#define SHUTTER_FRAMELENGTH_MARGIN 16

	//frame_length = GetScenarioFramelength();

	//frame_length = (S5K4H8.FixedFrameLength>frame_length)?S5K4H8.FixedFrameLength:frame_length;

	if (shutter < 3)
		shutter = 3;

	//if (shutter+SHUTTER_FRAMELENGTH_MARGIN > frame_length)
		frame_length = shutter + SHUTTER_FRAMELENGTH_MARGIN; //extend framelength

	//frame_length_min = GetScenarioFramelength();

	frame_length_min = S5K4H8_FRAME_LENGTH_COUNT;
	SENSORDB("0x0340=%d\n",frame_length_min);
	if(frame_length < frame_length_min)
		frame_length = frame_length_min;
	

	if(S5K4H8.S5K4H8AutoFlickerMode == KAL_TRUE)
	{
		line_length = GetScenarioLinelength();
		pixelclock = GetScenarioPixelClock();
		framerate = (10 * pixelclock) / (frame_length * line_length);
		  
		if(framerate > 290)
		{
		  	framerate = 290;
		  	frame_length = (10 * pixelclock) / (framerate * line_length);
		}
		else if(framerate > 147 && framerate < 152)
		{
		  	framerate = 147;
			frame_length = (10 * pixelclock) / (framerate * line_length);
		}
	}

	spin_lock_irqsave(&S5K4H8mipiraw_drv_lock,flags);
	S5K4H8.maxExposureLines = frame_length;
	S5K4H8.shutter = shutter;
	spin_unlock_irqrestore(&S5K4H8mipiraw_drv_lock,flags);

	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x01);    //Grouped parameter hold    
	S5K4H8_wordwrite_cmos_sensor(0x0340, frame_length); 
 	S5K4H8_wordwrite_cmos_sensor(0x0202, shutter);
 	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x00);    //Grouped parameter release
 	
	SENSORDB("shutter=%d,frame_length=%d,framerate=%d\n",shutter,frame_length, framerate);
}   /* write_S5K4H8_shutter */


void write_S5K4H8_gain(kal_uint16 gain)
{
	unsigned long flags;
	SENSORDB("gain=%d\n",gain);
	
	spin_lock_irqsave(&S5K4H8mipiraw_drv_lock,flags);
	S5K4H8.sensorGain = gain/2;
	spin_unlock_irqrestore(&S5K4H8mipiraw_drv_lock,flags);
	
	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x01);    //Grouped parameter hold    
	S5K4H8_wordwrite_cmos_sensor(0x0204,S5K4H8.sensorGain);   //sensor base gain is 32
	S5K4H8_bytewrite_cmos_sensor(0x0104, 0x00);    //Grouped parameter release 
	
}

/*************************************************************************
* FUNCTION
*    S5K4H8_SetGain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    gain : sensor global gain(base: 0x40)
*
* RETURNS
*    the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H8_SetGain(UINT16 iGain)
{
	write_S5K4H8_gain(iGain);
}


/*************************************************************************
* FUNCTION
*    read_S5K4H8_gain
*
* DESCRIPTION
*    This function is to set global gain to sensor.
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_uint16 read_S5K4H8_gain(void)
{
	kal_uint16 read_gain=S5K4H8_read_cmos_sensor(0x0204);
	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.sensorGain = read_gain;
	spin_unlock(&S5K4H8mipiraw_drv_lock);
	return S5K4H8.sensorGain;
}  


void S5K4H8_camera_para_to_sensor(void)
{
  /*  kal_uint32    i;
    for(i=0; 0xFFFFFFFF!=S5K4H8SensorReg[i].Addr; i++)
    {
        S5K4H8_wordwrite_cmos_sensor(S5K4H8SensorReg[i].Addr, S5K4H8SensorReg[i].Para);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K4H8SensorReg[i].Addr; i++)
    {
        S5K4H8_wordwrite_cmos_sensor(S5K4H8SensorReg[i].Addr, S5K4H8SensorReg[i].Para);
    }
    for(i=FACTORY_START_ADDR; i<FACTORY_END_ADDR; i++)
    {
        S5K4H8_wordwrite_cmos_sensor(S5K4H8SensorCCT[i].Addr, S5K4H8SensorCCT[i].Para);
    }*/
}


/*************************************************************************
* FUNCTION
*    S5K4H8_sensor_to_camera_para
*
* DESCRIPTION
*    // update camera_para from sensor register
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H8_sensor_to_camera_para(void)
{
/*    kal_uint32    i, temp_data;
    for(i=0; 0xFFFFFFFF!=S5K4H8SensorReg[i].Addr; i++)
    {
         temp_data = S5K4H8_read_cmos_sensor(S5K4H8SensorReg[i].Addr);
		 spin_lock(&S5K4H8mipiraw_drv_lock);
		 S5K4H8SensorReg[i].Para =temp_data;
		 spin_unlock(&S5K4H8mipiraw_drv_lock);
    }
    for(i=ENGINEER_START_ADDR; 0xFFFFFFFF!=S5K4H8SensorReg[i].Addr; i++)
    {
        temp_data = S5K4H8_read_cmos_sensor(S5K4H8SensorReg[i].Addr);
		spin_lock(&S5K4H8mipiraw_drv_lock);
		S5K4H8SensorReg[i].Para = temp_data;
		spin_unlock(&S5K4H8mipiraw_drv_lock);
    }*/
}

/*************************************************************************
* FUNCTION
*    S5K4H8_get_sensor_group_count
*
* DESCRIPTION
*    //
*
* PARAMETERS
*    None
*
* RETURNS
*    gain : sensor global gain(base: 0x40)
*
* GLOBALS AFFECTED
*
*************************************************************************/
kal_int32  S5K4H8_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void S5K4H8_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{
 /*  switch (group_idx)
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
	}*/
}

void S5K4H8_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{
/*    kal_int16 temp_reg=0;
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

            temp_para= S5K4H8SensorCCT[temp_addr].Para;
			temp_gain= (temp_para*1000/S5K4H8.sensorBaseGain) ;

            info_ptr->ItemValue=temp_gain;
            info_ptr->IsTrueFalse=KAL_FALSE;
            info_ptr->IsReadOnly=KAL_FALSE;
            info_ptr->IsNeedRestart=KAL_FALSE;
            info_ptr->Min= S5K4H8_MIN_ANALOG_GAIN * 1000;
            info_ptr->Max= S5K4H8_MAX_ANALOG_GAIN * 1000;
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
                    info_ptr->ItemValue=    111;  //MT9P017_MAX_EXPOSURE_LINES;
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
    }*/
}



kal_bool S5K4H8_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{
/*
   kal_uint16  temp_gain=0,temp_addr=0, temp_para=0;
   switch (group_idx)
    {
        case PRE_GAIN:
            switch (item_idx)
            {
				case 0:	temp_addr = PRE_GAIN_R_INDEX;		break;	
				case 1:	temp_addr = PRE_GAIN_Gr_INDEX;		break;
				case 2: temp_addr = PRE_GAIN_Gb_INDEX;		break;
				case 3: temp_addr = PRE_GAIN_B_INDEX;		break;
				case 4:	temp_addr = SENSOR_BASEGAIN;		break;
				default: ASSERT(0);
          }

			temp_gain=((ItemValue*S5K4H8.sensorBaseGain+500)/1000);			//+500:get closed integer value

		  spin_lock(&S5K4H8mipiraw_drv_lock);
          S5K4H8SensorCCT[temp_addr].Para = temp_para;
		  spin_unlock(&S5K4H8mipiraw_drv_lock);
          S5K4H8_wordwrite_cmos_sensor(S5K4H8SensorCCT[temp_addr].Addr,temp_para);
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
					spin_lock(&S5K4H8mipiraw_drv_lock);
                    S5K4H8_FAC_SENSOR_REG=ItemValue;
					spin_unlock(&S5K4H8mipiraw_drv_lock);
                    break;
                case 1:
                    S5K4H8_wordwrite_cmos_sensor(S5K4H8_FAC_SENSOR_REG,ItemValue);
                    break;
                default:
                    ASSERT(0);
            }
            break;
        default:
            ASSERT(0);
    }*/
    return KAL_TRUE; 
}



static void S5K4H8InitSetting(void)
{
	kal_uint16 chip_id = 0;
 	chip_id = S5K4H8_read_cmos_sensor(0x0002);
	SENSORDB("enter\n");
    S5K4H8_wordwrite_cmos_sensor(0x6010, 0x0001);  //reset
    mDELAY(3);
  // ***********************************************************************************
	// ATTENTION: DO NOT DELETE or CHANGE without prior notice to CHQ(or HQ) or local FAE.
	//            WE make NO WARRANTY on any use without prior consultation.
	// ***********************************************************************************
	//$MIPI[width:3264,height:2448,lane:4,format:raw10,datarate:700]
	////$MV1[MCLK:24,Width:3264,Height:2448,Format:MIPI_RAW10,mipi_lane:4,mipi_datarate:700,pvi_pclk_inverse:0]

	// Clock Gen
    SENSORDB("chip_id = 0x%x,enter sensor init EVT2\n",chip_id);
    //evt2 sensor init empty, all init codes are put in the specified mode
    S5K4H8_wordwrite_cmos_sensor(0x6028,0x4000);
	S5K4H8_wordwrite_cmos_sensor(0x602A,0x6214);
	S5K4H8_wordwrite_cmos_sensor(0x6F12,0x7971);
	S5K4H8_wordwrite_cmos_sensor(0x602A,0x6218);
	S5K4H8_wordwrite_cmos_sensor(0x6F12,0x7150);
	S5K4H8_wordwrite_cmos_sensor(0x6028,0x2000);
	S5K4H8_wordwrite_cmos_sensor(0x602A,0x0EC6);
	S5K4H8_wordwrite_cmos_sensor(0x6F12,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x6028,0x4000); //0xFCFC, 0x4000 lili modify
	S5K4H8_wordwrite_cmos_sensor(0xF490,0x0030);
	S5K4H8_wordwrite_cmos_sensor(0xF47A,0x0012);
	S5K4H8_wordwrite_cmos_sensor(0xF428,0x0200);
	S5K4H8_wordwrite_cmos_sensor(0xF48E,0x0010);
	S5K4H8_wordwrite_cmos_sensor(0xF45C,0x0004);
	S5K4H8_wordwrite_cmos_sensor(0x0B04,0x0101);
	S5K4H8_wordwrite_cmos_sensor(0x0B00,0x0180);    //lsc on 0x0180
	S5K4H8_wordwrite_cmos_sensor(0x6028,0x2000);
	S5K4H8_wordwrite_cmos_sensor(0x602A,0x0C40);
	S5K4H8_wordwrite_cmos_sensor(0x6F12,0x0140);
	//S5K4H8_wordwrite_cmos_sensor(0x6028, 0x4000); //lili modify
}

void S5K4H8PreviewSetting(void)
{
	//1600x1200
	kal_uint16 chip_id = 0;
	chip_id = S5K4H8_read_cmos_sensor(0x0002);	
	SENSORDB("chip_id = %d,enter preview EVT2\n",chip_id);
	S5K4H8_wordwrite_cmos_sensor(0x0100,0x0000);
 //	mDELAY(80); 	
	S5K4H8_wordwrite_cmos_sensor(0x6028,0x4000);
	S5K4H8_wordwrite_cmos_sensor(0x0200,0x0618);
	S5K4H8_wordwrite_cmos_sensor(0x0202,0x0904);
	S5K4H8_wordwrite_cmos_sensor(0x31AA,0x0004);
	S5K4H8_wordwrite_cmos_sensor(0x1006,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x31FA,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x0204,0x0020);
	//S5K4H8_wordwrite_cmos_sensor(0x020E,0x0100);
	S5K4H8_wordwrite_cmos_sensor(0x0344,0x0008);
	S5K4H8_wordwrite_cmos_sensor(0x0348,0x0CC7);
	S5K4H8_wordwrite_cmos_sensor(0x0346,0x0008);
	S5K4H8_wordwrite_cmos_sensor(0x034A,0x0997);
	S5K4H8_wordwrite_cmos_sensor(0x034C,0x0660);
	S5K4H8_wordwrite_cmos_sensor(0x034E,0x04C8);
	S5K4H8_wordwrite_cmos_sensor(0x0342,0X0EA0);
	S5K4H8_wordwrite_cmos_sensor(0x0340,0X09C2);
	S5K4H8_wordwrite_cmos_sensor(0x0900,0x0212);
	S5K4H8_wordwrite_cmos_sensor(0x0380,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0382,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0384,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0386,0x0003);
	S5K4H8_wordwrite_cmos_sensor(0x0400,0x0002);
	S5K4H8_wordwrite_cmos_sensor(0x0404,0x0020);
	S5K4H8_wordwrite_cmos_sensor(0x0114,0x0330);
	S5K4H8_wordwrite_cmos_sensor(0x0136,0x1800);//open lsc
	S5K4H8_wordwrite_cmos_sensor(0x0300,0x0005);
	S5K4H8_wordwrite_cmos_sensor(0x0302,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0304,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x0306,0x00AF);
	S5K4H8_wordwrite_cmos_sensor(0x030C,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x030E,0x00AF);
	S5K4H8_wordwrite_cmos_sensor(0x3008,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x317A,0x0101);
	S5K4H8_wordwrite_cmos_sensor(0x0100,0x0100);

}

void S5K4H8CaptureSetting(void)
{
	kal_uint16 chip_id = 0;
	chip_id = S5K4H8_read_cmos_sensor(0x0002);
	SENSORDB("chip_id = %d,enter capture EVT2\n",chip_id);
	S5K4H8_wordwrite_cmos_sensor(0x0100,0x0000);	
//	mDELAY(80);
	S5K4H8_wordwrite_cmos_sensor(0x6028,0x4000);
	S5K4H8_wordwrite_cmos_sensor(0x0200,0x0618);
	S5K4H8_wordwrite_cmos_sensor(0x0202,0x0904);
	S5K4H8_wordwrite_cmos_sensor(0x31AA,0x0004);
	S5K4H8_wordwrite_cmos_sensor(0x1006,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x31FA,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x0204,0x0020);
	//S5K4H8_wordwrite_cmos_sensor(0x020E,0x0100);
	S5K4H8_wordwrite_cmos_sensor(0x0344,0x0008);
	S5K4H8_wordwrite_cmos_sensor(0x0348,0x0CC7);
	S5K4H8_wordwrite_cmos_sensor(0x0346,0x0008);
	S5K4H8_wordwrite_cmos_sensor(0x034A,0x0997);
	S5K4H8_wordwrite_cmos_sensor(0x034C,0x0CC0);
	S5K4H8_wordwrite_cmos_sensor(0x034E,0x0990);
	S5K4H8_wordwrite_cmos_sensor(0x0342,0X0EA0);
	S5K4H8_wordwrite_cmos_sensor(0x0340,0X09C2);
	S5K4H8_wordwrite_cmos_sensor(0x0900,0x0111);
	S5K4H8_wordwrite_cmos_sensor(0x0380,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0382,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0384,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0386,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0400,0x0002);
	S5K4H8_wordwrite_cmos_sensor(0x0404,0x0010);
	S5K4H8_wordwrite_cmos_sensor(0x0114,0x0330);
	S5K4H8_wordwrite_cmos_sensor(0x0136,0x1800);
	S5K4H8_wordwrite_cmos_sensor(0x0300,0x0005);
	S5K4H8_wordwrite_cmos_sensor(0x0302,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0304,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x0306,0x00AF);
	S5K4H8_wordwrite_cmos_sensor(0x030C,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x030E,0x00AF);
	S5K4H8_wordwrite_cmos_sensor(0x3008,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x317A,0x0101);
	S5K4H8_wordwrite_cmos_sensor(0x0100,0x0100);
}

void S5K4H8VideoSetting(void)
{
	SENSORDB("enter\n");
	kal_uint16 chip_id = 0;
 	chip_id = S5K4H8_read_cmos_sensor(0x0002);    
    SENSORDB("chip_id = 0x%x,enter video EVT2\n",chip_id);
#if 1	//same preview setting 1632 x 1224
	S5K4H8_wordwrite_cmos_sensor(0x0100,0x0000);
 //	mDELAY(80); 
	
	S5K4H8_wordwrite_cmos_sensor(0x6028,0x4000);
	S5K4H8_wordwrite_cmos_sensor(0x0200,0x0618);
	S5K4H8_wordwrite_cmos_sensor(0x0202,0x0904);
	S5K4H8_wordwrite_cmos_sensor(0x31AA,0x0004);
	S5K4H8_wordwrite_cmos_sensor(0x1006,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x31FA,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x0204,0x0020);
	//S5K4H8_wordwrite_cmos_sensor(0x020E,0x0100);
	S5K4H8_wordwrite_cmos_sensor(0x0344,0x0008);
	S5K4H8_wordwrite_cmos_sensor(0x0348,0x0CC7);
	S5K4H8_wordwrite_cmos_sensor(0x0346,0x0008);
	S5K4H8_wordwrite_cmos_sensor(0x034A,0x0997);
	S5K4H8_wordwrite_cmos_sensor(0x034C,0x0660);
	S5K4H8_wordwrite_cmos_sensor(0x034E,0x04C8);
	S5K4H8_wordwrite_cmos_sensor(0x0342,0X0EA0);
	S5K4H8_wordwrite_cmos_sensor(0x0340,0X09C2);
	S5K4H8_wordwrite_cmos_sensor(0x0900,0x0212);
	S5K4H8_wordwrite_cmos_sensor(0x0380,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0382,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0384,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0386,0x0003);
	S5K4H8_wordwrite_cmos_sensor(0x0400,0x0002);
	S5K4H8_wordwrite_cmos_sensor(0x0404,0x0020);
	S5K4H8_wordwrite_cmos_sensor(0x0114,0x0330);
	S5K4H8_wordwrite_cmos_sensor(0x0136,0x1800);
	S5K4H8_wordwrite_cmos_sensor(0x0300,0x0005);
	S5K4H8_wordwrite_cmos_sensor(0x0302,0x0001);
	S5K4H8_wordwrite_cmos_sensor(0x0304,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x0306,0x00AF);
	S5K4H8_wordwrite_cmos_sensor(0x030C,0x0006);
	S5K4H8_wordwrite_cmos_sensor(0x030E,0x00AF);
	S5K4H8_wordwrite_cmos_sensor(0x3008,0x0000);
	S5K4H8_wordwrite_cmos_sensor(0x317A,0x0101);
	S5K4H8_wordwrite_cmos_sensor(0x0100,0x0100);


#endif
}

   /*  S5K4H8InitSetting  */

/*************************************************************************
* FUNCTION
*   S5K4H8Open
*
* DESCRIPTION
*   This function initialize the registers of CMOS sensor
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/

UINT32 S5K4H8Open(void)
{

	volatile signed int i,j;
	kal_uint16 sensor_id = 0;

	SENSORDB("enter\n");

	//  Read sensor ID to adjust I2C is OK?
	for(j=0;j<2;j++)
	{
		SENSORDB("Read sensor ID=0x%x\n",sensor_id);
		if(S5K4H8_SENSOR_ID == sensor_id)
		{
			break;
		}	
		
		switch(j) {
			case 0:
				S5K4H8_slave_addr = S5K4H8MIPI_WRITE_ID2;
				break;
			case 1:
				S5K4H8_slave_addr = S5K4H8MIPI_WRITE_ID;
				break;
			default:
				break;
		}
				SENSORDB("S5K4H8_slave_addr =0x%x\n",S5K4H8_slave_addr);
		for(i=3;i>0;i--)
		{
			sensor_id = S5K4H8_read_cmos_sensor(0x0000);
			SENSORDB("Read sensor ID=0x%x\n",sensor_id);
			if(S5K4H8_SENSOR_ID == sensor_id)
			{
				break;
			}		
		}
	
		
	}
	if(S5K4H8_SENSOR_ID != sensor_id)
	{
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	
	S5K4H8InitSetting();
	mdelay(10);
#ifdef otp_ofilm
     
	if(S5K4H8_otp_data.flag==0){
		S5K4H8_read_otp(&S5K4H8_otp_data);
		SENSORDB("lanjun OTP load\n");
	}
       	S5K4H8_write_otp(&S5K4H8_otp_data); 
		
		
		      
#endif
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*   S5K4H8GetSensorID
*
* DESCRIPTION
*   This function get the sensor ID
*
* PARAMETERS
*   *sensorID : return the sensor ID
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H8GetSensorID(UINT32 *sensorID)
{
    //int  retry = 2;
	int i=0,j =0;
	

	SENSORDB("enter\n");
	
	for(j=0;j<2;j++)
	{
		SENSORDB("Read sensor ID=0x%x\n",*sensorID);
		if(S5K4H8_SENSOR_ID == *sensorID)
		{
			break;
		}

		switch(j) {
			case 0:
				S5K4H8_slave_addr = S5K4H8MIPI_WRITE_ID2;
				break;
			case 1:
				S5K4H8_slave_addr = S5K4H8MIPI_WRITE_ID;
				break;
			default:
				break;
		}
				SENSORDB("S5K4H8_slave_addr =0x%x\n",S5K4H8_slave_addr);
		for(i=3;i>0;i--)
		{
			S5K4H8_wordwrite_cmos_sensor(0x6010,0x0001);	// Reset		
	    	mDELAY(1);
			*sensorID = S5K4H8_read_cmos_sensor(0x0000);
			SENSORDB("Read sensor ID=0x%x\n",*sensorID);
			if(S5K4H8_SENSOR_ID == *sensorID)
			{
				break;
			}		
		}
		
		
	}


	if (*sensorID != S5K4H8_SENSOR_ID)
	{
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
    }

	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.sensorMode = SENSOR_MODE_INIT;
	S5K4H8.S5K4H8AutoFlickerMode = KAL_FALSE;
	S5K4H8.S5K4H8VideoMode = KAL_FALSE;	
	S5K4H8.DummyLines= 0;
	S5K4H8.DummyPixels= 0;
	S5K4H8.pvPclk = SENSOR_PCLK_PREVIEW; //260MHz 
	S5K4H8.m_vidPclk= SENSOR_PCLK_VIDEO;
	S5K4H8.capPclk= SENSOR_PCLK_CAPTURE;
	S5K4H8.shutter = 0x4EA;
	S5K4H8.pvShutter = 0x4EA;
	S5K4H8.maxExposureLines = S5K4H8_PV_PERIOD_LINE_NUMS;
	S5K4H8.FixedFrameLength = S5K4H8_PV_PERIOD_LINE_NUMS;
	S5K4H8.sensorGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	S5K4H8.pvGain = 0x1f*3; //SL for brighter to SMT load
	s_S5K4H8CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;
	spin_unlock(&S5K4H8mipiraw_drv_lock);
    return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*   S5K4H8_SetShutter
*
* DESCRIPTION
*   This function set e-shutter of S5K4H8 to change exposure time.
*
* PARAMETERS
*   shutter : exposured lines
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H8_SetShutter(kal_uint32 iShutter)
{
	SENSORDB("lanjun iShutter=0x%x\n",iShutter);   
	spin_lock(&S5K4H8mipiraw_drv_lock);
   S5K4H8.shutter= iShutter;
   spin_unlock(&S5K4H8mipiraw_drv_lock);
   S5K4H8_write_shutter(iShutter);

}   /*  S5K4H8_SetShutter   */



/*************************************************************************
* FUNCTION
*   S5K4H8_read_shutter
*
* DESCRIPTION
*   This function to  Get exposure time.
*
* PARAMETERS
*   None
*
* RETURNS
*   shutter : exposured lines
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H8_read_shutter(void)
{
	kal_uint16 read_shtter=0;	
	read_shtter= S5K4H8_read_cmos_sensor(0x0202);   // smiaRegs_rw_integration_time_coarse_integration_time
	SENSORDB("lanjun iShutter=0x%x\n",read_shtter);
	return  read_shtter;
	
}

/*************************************************************************
* FUNCTION
*   S5K4H8_night_mode
*
* DESCRIPTION
*   This function night mode of S5K4H8.
*
* PARAMETERS
*   none
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
void S5K4H8_NightMode(kal_bool bEnable)
{
}/*	S5K4H8_NightMode */



/*************************************************************************
* FUNCTION
*   S5K4H8Close
*
* DESCRIPTION
*   This function is to turn off sensor module power.
*
* PARAMETERS
*   None
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H8Close(void)
{
    //  CISModulePowerOn(FALSE);
    //s_porting
    //  DRV_I2CClose(S5K4H8hDrvI2C);
    //e_porting
    return ERROR_NONE;
}	/* S5K4H8Close() */

void S5K4H8SetFlipMirror(kal_int32 imgMirror)
{
	SENSORDB("imgMirror=%d\n",imgMirror);
	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.imgMirror = imgMirror; //(imgMirror+IMAGE_HV_MIRROR)%(IMAGE_HV_MIRROR+1);
	spin_unlock(&S5K4H8mipiraw_drv_lock);
	
    switch (imgMirror)
    {
        case IMAGE_H_MIRROR://IMAGE_NORMAL:  bit0 mirror,   bit1 flip.
			S5K4H8_bytewrite_cmos_sensor(0x0101,0x01  ); //morror
            break;
        case IMAGE_NORMAL://IMAGE_V_MIRROR:
			S5K4H8_bytewrite_cmos_sensor(0x0101,0x03  );
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
			S5K4H8_bytewrite_cmos_sensor(0x0101,0x00 );   //morror +flip
            break;
        case IMAGE_V_MIRROR://IMAGE_HV_MIRROR:
			S5K4H8_bytewrite_cmos_sensor(0x0101,0x02  ); //flip
            break;
    }
}


/*************************************************************************
* FUNCTION
*   S5K4H8Preview
*
* DESCRIPTION
*   This function start the sensor preview.
*
* PARAMETERS
*   *image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*   None
*
* GLOBALS AFFECTED
*
*************************************************************************/
UINT32 S5K4H8Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	SENSORDB("enter\n");

	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	//S5K4H8_FeatureControl_PERIOD_PixelNum=S5K4H8_PV_PERIOD_PIXEL_NUMS+ S5K4H8.DummyPixels;
	//S5K4H8_FeatureControl_PERIOD_LineNum=S5K4H8_PV_PERIOD_LINE_NUMS+S5K4H8.DummyLines;
	spin_unlock(&S5K4H8mipiraw_drv_lock);

	S5K4H8PreviewSetting();
	S5K4H8SetFlipMirror(sensor_config_data->SensorImageMirror);
	
    return ERROR_NONE;
}	/* S5K4H8Preview() */

UINT32 S5K4H8Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	SENSORDB("enter\n");

	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.sensorMode = SENSOR_MODE_VIDEO; // Need set preview setting after capture mode
	//S5K4H8_FeatureControl_PERIOD_PixelNum=S5K4H8_PV_PERIOD_PIXEL_NUMS+ S5K4H8.DummyPixels;
	//S5K4H8_FeatureControl_PERIOD_LineNum=S5K4H8_PV_PERIOD_LINE_NUMS+S5K4H8.DummyLines;
	spin_unlock(&S5K4H8mipiraw_drv_lock);

	S5K4H8VideoSetting();
	S5K4H8SetFlipMirror(sensor_config_data->SensorImageMirror);
	
    return ERROR_NONE;
}	/* S5K4H8Preview() */


UINT32 S5K4H8ZSDPreview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("enter\n");	
	// Full size setting
	S5K4H8CaptureSetting();

	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.sensorMode = SENSOR_MODE_ZSD_PREVIEW;	
	//S5K4H8_FeatureControl_PERIOD_PixelNum = S5K4H8_FULL_PERIOD_PIXEL_NUMS + S5K4H8.DummyPixels;
	//S5K4H8_FeatureControl_PERIOD_LineNum = S5K4H8_FULL_PERIOD_LINE_NUMS + S5K4H8.DummyLines;
	spin_unlock(&S5K4H8mipiraw_drv_lock);
	
	S5K4H8SetFlipMirror(sensor_config_data->SensorImageMirror);
	
    return ERROR_NONE;
}

UINT32 S5K4H8Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	SENSORDB("sensorMode=%d\n",S5K4H8.sensorMode);
	if((S5K4H8.sensorMode = SENSOR_MODE_INIT) ||(S5K4H8.sensorMode = SENSOR_MODE_PREVIEW) || (S5K4H8.sensorMode = SENSOR_MODE_VIDEO))	

	S5K4H8CaptureSetting();


	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.sensorMode = SENSOR_MODE_CAPTURE;	
	spin_unlock(&S5K4H8mipiraw_drv_lock);
	
	S5K4H8SetFlipMirror(sensor_config_data->SensorImageMirror);
	
    return ERROR_NONE;
}	/* S5K4H8Capture() */

UINT32 S5K4H8GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{
    SENSORDB("enter\n");
	pSensorResolution->SensorPreviewWidth	= 	S5K4H8_IMAGE_SENSOR_PV_WIDTH;
	pSensorResolution->SensorPreviewHeight	= 	S5K4H8_IMAGE_SENSOR_PV_HEIGHT;
	pSensorResolution->SensorVideoWidth		=	S5K4H8_IMAGE_SENSOR_VIDEO_WIDTH;
	pSensorResolution->SensorVideoHeight 	=	S5K4H8_IMAGE_SENSOR_VIDEO_HEIGHT;
	pSensorResolution->SensorFullWidth		= 	S5K4H8_IMAGE_SENSOR_FULL_WIDTH;
	pSensorResolution->SensorFullHeight		= 	S5K4H8_IMAGE_SENSOR_FULL_HEIGHT;
	//SENSORDB("Video width/height: %d/%d",pSensorResolution->SensorVideoWidth,pSensorResolution->SensorVideoHeight);
    return ERROR_NONE;
}   /* S5K4H8GetResolution() */

UINT32 S5K4H8GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	switch(s_S5K4H8CurrentScenarioId)
	{
    	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			pSensorInfo->SensorPreviewResolutionX= S5K4H8_IMAGE_SENSOR_FULL_WIDTH;
			pSensorInfo->SensorPreviewResolutionY= S5K4H8_IMAGE_SENSOR_FULL_HEIGHT;
			break;
		default:
			pSensorInfo->SensorPreviewResolutionX= S5K4H8_IMAGE_SENSOR_PV_WIDTH;
			pSensorInfo->SensorPreviewResolutionY= S5K4H8_IMAGE_SENSOR_PV_HEIGHT;
			break;
	}

	pSensorInfo->SensorFullResolutionX= S5K4H8_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= S5K4H8_IMAGE_SENSOR_FULL_HEIGHT;

	SENSORDB("SensorImageMirror=%d\n", pSensorConfigData->SensorImageMirror);

	switch(S5K4H8.imgMirror)
	{
		case IMAGE_NORMAL:
   			pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gb;
		break;
		case IMAGE_H_MIRROR:
   			pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gr;
		break;
		case IMAGE_V_MIRROR:
   			pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_Gb;
		break;
		case IMAGE_HV_MIRROR:
   			pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_R;
		break;
		default:
			pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
	}
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;

    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0 ;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

	pSensorInfo->SensorClockFreq=24;  //26
	pSensorInfo->SensorClockRisingCount= 0;
	#ifdef USE_MIPI_2_LANES
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_2_LANE;
	#else
	pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
	#endif
	pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 14;
	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
	pSensorInfo->SensorPacketECCOrder = 1;
	
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorGrabStartX = S5K4H8_PV_X_START;
            pSensorInfo->SensorGrabStartY = S5K4H8_PV_Y_START;
		break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pSensorInfo->SensorGrabStartX = S5K4H8_VIDEO_X_START;
			pSensorInfo->SensorGrabStartY = S5K4H8_VIDEO_Y_START;
        break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorGrabStartX = S5K4H8_FULL_X_START;	//2*S5K4H8_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = S5K4H8_FULL_Y_START;	//2*S5K4H8_IMAGE_SENSOR_PV_STARTY;
        break;
        default:
            pSensorInfo->SensorGrabStartX = S5K4H8_PV_X_START;
            pSensorInfo->SensorGrabStartY = S5K4H8_PV_Y_START;
            break;
    }

    memcpy(pSensorConfigData, &S5K4H8SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* S5K4H8GetInfo() */


UINT32 S5K4H8Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
	spin_lock(&S5K4H8mipiraw_drv_lock);
	s_S5K4H8CurrentScenarioId = ScenarioId;
	S5K4H8.FixedFrameLength = GetScenarioFramelength();
	spin_unlock(&S5K4H8mipiraw_drv_lock);

	SENSORDB("s_S5K4H8CurrentScenarioId=%d\n",s_S5K4H8CurrentScenarioId);
	
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            S5K4H8Preview(pImageWindow, pSensorConfigData);
        break;
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			S5K4H8Video(pImageWindow, pSensorConfigData);
		break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			S5K4H8Capture(pImageWindow, pSensorConfigData);
        break;
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			S5K4H8ZSDPreview(pImageWindow, pSensorConfigData);
		break;
        default:
            return ERROR_INVALID_SCENARIO_ID;

    }	
    return ERROR_NONE;
} /* S5K4H8Control() */

UINT32 S5K4H8SetVideoMode(UINT16 u2FrameRate)
{
	S5K4H8.sensorMode=MSDK_SCENARIO_ID_VIDEO_PREVIEW;
    SENSORDB("u2FrameRate=%d,sensorMode=%d\n", u2FrameRate,S5K4H8.sensorMode);
	
	
	if(0==u2FrameRate || u2FrameRate >30 || u2FrameRate <5) //do not fix frame rate 
	{
		spin_lock(&S5K4H8mipiraw_drv_lock);
		S5K4H8.FixedFrameLength = GetScenarioFramelength();
		spin_unlock(&S5K4H8mipiraw_drv_lock);
		SENSORDB("S5K4H8.FixedFrameLength=%d\n",S5K4H8.FixedFrameLength);
		return ERROR_NONE;
	}
	
	S5K4H8MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_VIDEO_PREVIEW,u2FrameRate*10);
	return ERROR_NONE;
}

static void S5K4H8SetMaxFrameRate(UINT16 u2FrameRate)
{
	kal_uint16 FrameHeight;
		
	SENSORDB("[S5K4H8] [S5K4H8MIPISetMaxFrameRate] u2FrameRate=%d\n",u2FrameRate);

	if(SENSOR_MODE_PREVIEW == S5K4H8.sensorMode)
	{
		FrameHeight= (10 * S5K4H8.pvPclk) / u2FrameRate / S5K4H8_PV_PERIOD_PIXEL_NUMS;
		FrameHeight = (FrameHeight > S5K4H8_PV_PERIOD_LINE_NUMS) ? FrameHeight : S5K4H8_PV_PERIOD_LINE_NUMS;
	}
	else if(SENSOR_MODE_CAPTURE== S5K4H8.sensorMode || SENSOR_MODE_ZSD_PREVIEW == S5K4H8.sensorMode)
	{
		FrameHeight= (10 * S5K4H8.capPclk) / u2FrameRate / S5K4H8_FULL_PERIOD_PIXEL_NUMS;
		FrameHeight = (FrameHeight > S5K4H8_FULL_PERIOD_LINE_NUMS) ? FrameHeight : S5K4H8_FULL_PERIOD_LINE_NUMS;
	}
	else
	{
		FrameHeight = (10 * S5K4H8.m_vidPclk) / u2FrameRate / S5K4H8_VIDEO_PERIOD_PIXEL_NUMS;
		FrameHeight = (FrameHeight > S5K4H8_VIDEO_PERIOD_LINE_NUMS) ? FrameHeight : S5K4H8_VIDEO_PERIOD_LINE_NUMS;
	}
	SENSORDB("[S5K4H8] [S5K4H8MIPISetMaxFrameRate] FrameHeight=%d",FrameHeight);
	SetFramelength(FrameHeight); /* modify dummy_pixel must gen AE table again */	
}


UINT32 S5K4H8SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	if(bEnable) 
	{
		SENSORDB("[S5K4H8] [S5K4H8SetAutoFlickerMode] enable\n");
		spin_lock(&S5K4H8mipiraw_drv_lock);
		S5K4H8.S5K4H8AutoFlickerMode = KAL_TRUE;
		spin_unlock(&S5K4H8mipiraw_drv_lock);

		if(u2FrameRate == 300)
			S5K4H8SetMaxFrameRate(296);
		else if(u2FrameRate == 150)
			S5K4H8SetMaxFrameRate(148);
    } 
	else 
	{
    	SENSORDB("[S5K4H8] [S5K4H8SetAutoFlickerMode] disable\n");
    	spin_lock(&S5K4H8mipiraw_drv_lock);
        S5K4H8.S5K4H8AutoFlickerMode = KAL_FALSE;
		spin_unlock(&S5K4H8mipiraw_drv_lock);
    }
    return ERROR_NONE;
}


UINT32 S5K4H8SetTestPatternMode(kal_bool bEnable)
{
    SENSORDB("bEnable=%d\n", bEnable);
	if(bEnable) 
	{   
	    S5K4H8_wordwrite_cmos_sensor(0x36E8,0x0001);
	    S5K4H8_wordwrite_cmos_sensor(0x6214,0x7971);
	    S5K4H8_wordwrite_cmos_sensor(0x9B00,0x0001);
	    S5K4H8_wordwrite_cmos_sensor(0x0B00,0x0000);
        S5K4H8_wordwrite_cmos_sensor(0x0600,0x0002);
	}	
	else        
	{
	    S5K4H8_wordwrite_cmos_sensor(0x36E8,0x00C0);
	    S5K4H8_wordwrite_cmos_sensor(0x6214,0x7971);
	    S5K4H8_wordwrite_cmos_sensor(0x9B00,0x0000);
	    S5K4H8_wordwrite_cmos_sensor(0x0B00,0x0180);
		S5K4H8_wordwrite_cmos_sensor(0x0600,0x0000);
		
	}
    return ERROR_NONE;
}


UINT32 S5K4H8MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) 
{
	kal_uint16 frameLength = 0;
		
	SENSORDB("scenarioId=%d,frameRate=%d\n",scenarioId,frameRate);
	switch (scenarioId) 
	{
		//SetDummy() has to switch scenarioId again, so we do not use it here
		//when SetDummy() is ok, we'll switch to using SetDummy()
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			frameLength = (S5K4H8.pvPclk)/frameRate*10/S5K4H8_PV_PERIOD_PIXEL_NUMS;
			frameLength = (frameLength>S5K4H8_PV_PERIOD_LINE_NUMS)?(frameLength):(S5K4H8_PV_PERIOD_LINE_NUMS);				
		break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			frameLength = (S5K4H8.m_vidPclk)/frameRate*10/S5K4H8_VIDEO_PERIOD_PIXEL_NUMS;
			frameLength = (frameLength>S5K4H8_VIDEO_PERIOD_LINE_NUMS)?(frameLength):(S5K4H8_VIDEO_PERIOD_LINE_NUMS);	
		break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:	
			frameLength = (S5K4H8.m_vidPclk)/frameRate*10/S5K4H8_FULL_PERIOD_PIXEL_NUMS;
			frameLength = (frameLength>S5K4H8_FULL_PERIOD_LINE_NUMS)?(frameLength):(S5K4H8_FULL_PERIOD_LINE_NUMS);	
		break;	
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			frameLength = (S5K4H8.m_vidPclk)/frameRate*10/S5K4H8_ZSD_PERIOD_PIXEL_NUMS;
			frameLength = (frameLength>S5K4H8_ZSD_PERIOD_LINE_NUMS)?(frameLength):(S5K4H8_ZSD_PERIOD_LINE_NUMS);
		break;
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
		break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
		break;		
		default:
			frameLength = S5K4H8_PV_PERIOD_LINE_NUMS;
		break;
	}
	spin_lock(&S5K4H8mipiraw_drv_lock);
	S5K4H8.FixedFrameLength = frameLength;
	spin_unlock(&S5K4H8mipiraw_drv_lock);
	
	SetFramelength(frameLength); //direct set frameLength
	return ERROR_NONE;
}


UINT32 S5K4H8MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
		#ifdef FULL_SIZE_30_FPS
			 *pframeRate = 300;
		#else
			*pframeRate = 240;
		#endif
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			 *pframeRate = 300;
			break;		
		default:
			break;
	}

	return ERROR_NONE;
}

UINT32 S5K4H8MIPIGetTemperature(UINT32 *temperature)
{

	*temperature = 0;//read register
    return ERROR_NONE;
}



UINT32 S5K4H8FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
	
	SENSORDB("FeatureId=%d\n",FeatureId);
    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= S5K4H8_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= S5K4H8_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= GetScenarioLinelength();
				*pFeatureReturnPara16= GetScenarioFramelength();
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			//same pclk for preview/capture
    	 	*pFeatureReturnPara32 = S5K4H8.pvPclk;
			SENSORDB("sensor clock=%d\n",*pFeatureReturnPara32);
    	 	*pFeatureParaLen=4;
 			 break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            S5K4H8_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            S5K4H8_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            S5K4H8_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //S5K4H8_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            S5K4H8_wordwrite_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = S5K4H8_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&S5K4H8mipiraw_drv_lock);
                S5K4H8SensorCCT[i].Addr=*pFeatureData32++;
                S5K4H8SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&S5K4H8mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return ERROR_INVALID_PARA;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K4H8SensorCCT[i].Addr;
                *pFeatureData32++=S5K4H8SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&S5K4H8mipiraw_drv_lock);
                S5K4H8SensorReg[i].Addr=*pFeatureData32++;
                S5K4H8SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&S5K4H8mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return ERROR_INVALID_PARA;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=S5K4H8SensorReg[i].Addr;
                *pFeatureData32++=S5K4H8SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=S5K4H8_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, S5K4H8SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, S5K4H8SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return ERROR_INVALID_PARA;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &S5K4H8SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            S5K4H8_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            S5K4H8_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=S5K4H8_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            S5K4H8_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            S5K4H8_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            S5K4H8_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_GET_ENG_INFO:
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
            S5K4H8SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            S5K4H8GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            S5K4H8SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            S5K4H8SetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			S5K4H8MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			S5K4H8MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE://for factory mode auto testing 			
            *pFeatureReturnPara32= S5K4H8_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4; 							
			break;	
		case SENSOR_FEATURE_GET_SENSOR_CURRENT_TEMPERATURE:
			S5K4H8MIPIGetTemperature(pFeatureReturnPara32);
			*pFeatureParaLen=4; 
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* S5K4H8FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncS5K4H8=
{
    S5K4H8Open,
    S5K4H8GetInfo,
    S5K4H8GetResolution,
    S5K4H8FeatureControl,
    S5K4H8Control,
    S5K4H8Close
};

UINT32 S5K4H8_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncS5K4H8;

    return ERROR_NONE;
}   /* SensorInit() */


