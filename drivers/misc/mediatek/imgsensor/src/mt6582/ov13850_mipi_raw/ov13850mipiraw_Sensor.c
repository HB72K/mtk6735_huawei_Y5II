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

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include <linux/slab.h>

#include "ov13850mipiraw_Sensor.h"
//#include "ov13850mipiraw_Camera_Sensor_para.h"
//#include "ov13850mipiraw_CameraCustomized.h"
static DEFINE_SPINLOCK(ov13850mipiraw_drv_lock);

#define OV13850_DEBUG
//#define OV13850_DEBUG_SOFIA
#define OV13850_TEST_PATTERN_CHECKSUM (0xa6096115) //V_MIRROR
#define OV13850_DEBUG_SOFIA

#ifdef OV13850_DEBUG
	#define OV13850DB(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV13850Raw] ",  fmt, ##arg)
#else
	#define OV13850DB(fmt, arg...)
#endif

#ifdef OV13850_DEBUG_SOFIA
	#define OV13850DBSOFIA(fmt, arg...) xlog_printk(ANDROID_LOG_DEBUG, "[OV13850Raw] ",  fmt, ##arg)
#else
	#define OV13850DBSOFIA(fmt, arg...)
#endif

//xb.pang for test
kal_uint8 prv_flag = 0;

#define mDELAY(ms)  mdelay(ms)
kal_uint8 OV13850_WRITE_ID = OV13850MIPI_WRITE_ID;
kal_uint32 OV13850_FeatureControl_PERIOD_PixelNum=OV13850_PV_PERIOD_PIXEL_NUMS;
kal_uint32 OV13850_FeatureControl_PERIOD_LineNum=OV13850_PV_PERIOD_LINE_NUMS;
UINT16  ov13850VIDEO_MODE_TARGET_FPS = 30;
MSDK_SENSOR_CONFIG_STRUCT OV13850SensorConfigData;
MSDK_SCENARIO_ID_ENUM OV13850CurrentScenarioId = MSDK_SCENARIO_ID_CAMERA_PREVIEW;

/* FIXME: old factors and DIDNOT use now. s*/
SENSOR_REG_STRUCT OV13850SensorCCT[]=CAMERA_SENSOR_CCT_DEFAULT_VALUE;
SENSOR_REG_STRUCT OV13850SensorReg[ENGINEER_END]=CAMERA_SENSOR_REG_DEFAULT_VALUE;
/* FIXME: old factors and DIDNOT use now. e*/

static OV13850_PARA_STRUCT ov13850;

extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
extern int iMultiWriteReg(u8 *pData, u16 lens, u16 i2cId);

kal_uint16 OV13850_read_cmos_sensor(kal_uint32 addr);
#define OV13850_write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV13850_WRITE_ID)

#define OV13850_multi_write_cmos_sensor(pData, lens) iMultiWriteReg((u8*) pData, (u16) lens, OV13850_WRITE_ID)

#define OV13850_ORIENTATION IMAGE_V_MIRROR


#define OTP_CALIBRATION
#ifdef OTP_CALIBRATION
#define RG_Ratio_Typical 0x12b
#define BG_Ratio_Typical 0x126

#define OTP_DRV_START_ADDR 0x7220
#define OTP_DRV_INFO_GROUP_COUNT 3
#define OTP_DRV_INFO_SIZE 5
#define OTP_DRV_AWB_GROUP_COUNT 3
#define OTP_DRV_AWB_SIZE 5
#define OTP_DRV_VCM_GROUP_COUNT 3
#define OTP_DRV_VCM_SIZE 3
#define OTP_DRV_LSC_GROUP_COUNT 3
#define OTP_DRV_LSC_SIZE 62
#define OTP_DRV_LSC_REG_ADDR 0x5200

struct otp_struct {
int module_integrator_id;
int lens_id;
int production_year;
int production_month;
int production_day;
int rg_ratio;
int bg_ratio;
int light_rg;
int light_bg;
int lenc[OTP_DRV_LSC_SIZE ];
int VCM_start;
int VCM_end;
int VCM_dir;
};

#ifndef ORIGINAL_VERSION	
static struct otp_struct s_otp_wb;
static struct otp_struct s_otp_lenc;
static bool isNeedReadOtp = false;
#endif
// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
static int check_otp_info(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff);
	OV13850_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff );
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	flag = OV13850_read_cmos_sensor(nFlagAddress);
	//select group
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index ==3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

static int check_otp_wb(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//select group
	flag = OV13850_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}
static int check_otp_lenc(int index)
{
	int flag;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE
	+1+OTP_DRV_VCM_GROUP_COUNT*OTP_DRV_VCM_SIZE ;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	flag = OV13850_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>> 2)& 0x03;
	}
	// clear otp buffer
	OV13850_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}

static int check_otp_VCM(int index, int code)
{
	int flag;
	int nFlagAddress= OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d89, nFlagAddress & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (nFlagAddress>>8) & 0xff );
	OV13850_write_cmos_sensor(0x3d8B, nFlagAddress & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//select group
	flag = OV13850_read_cmos_sensor(nFlagAddress);
	if (index == 1)
	{
	flag = (flag>>6) & 0x03;
	}
	else if (index == 2)
	{
	flag = (flag>>4) & 0x03;
	}
	else if (index == 3)
	{
	flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	OV13850_write_cmos_sensor(nFlagAddress, 0x00);
	if (flag == 0x00) {
	return 0;
	}
	else if (flag & 0x02) {
	return 1;
	}
	else {
	return 2;
	}
}
static int read_otp_info(int index, struct otp_struct *otp_ptr)
{
	int i;
	int nFlagAddress = OTP_DRV_START_ADDR;
	int start_addr, end_addr;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_INFO_SIZE;
	end_addr = start_addr+OTP_DRV_INFO_SIZE-1;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	(*otp_ptr).module_integrator_id = OV13850_read_cmos_sensor(start_addr);
	(*otp_ptr).lens_id = OV13850_read_cmos_sensor(start_addr + 1);
	(*otp_ptr).production_year = OV13850_read_cmos_sensor(start_addr + 2);
	(*otp_ptr).production_month = OV13850_read_cmos_sensor(start_addr + 3);
	(*otp_ptr).production_day = OV13850_read_cmos_sensor(start_addr + 4);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850_write_cmos_sensor(i, 0x00);
}
return 0;
}
static int read_otp_wb(int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE;
	start_addr = nFlagAddress+1+(index-1)* OTP_DRV_AWB_SIZE;
	end_addr = start_addr+OTP_DRV_AWB_SIZE;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	temp = OV13850_read_cmos_sensor(start_addr + 4);
	(*otp_ptr).rg_ratio = (OV13850_read_cmos_sensor(start_addr)<<2) + ((temp>>6) & 0x03);
	(*otp_ptr).bg_ratio = (OV13850_read_cmos_sensor(start_addr + 1)<<2) + ((temp>>4) & 0x03);
	(*otp_ptr).light_rg = (OV13850_read_cmos_sensor(start_addr + 2) <<2) + ((temp>>2) & 0x03);
	(*otp_ptr).light_bg = (OV13850_read_cmos_sensor(start_addr + 3)<<2) + (temp & 0x03);
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850_write_cmos_sensor(i, 0x00);
	}

return 0;
}

static int read_otp_VCM(int index, struct otp_struct * otp_ptr)
{
	int i;
	int temp;
	int start_addr, end_addr;
	int nFlagAddress = OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT*OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT*OTP_DRV_AWB_SIZE;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_VCM_SIZE;
	end_addr = start_addr+OTP_DRV_VCM_SIZE-1;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(5);
	//flag and lsb of VCM start code
	temp = OV13850_read_cmos_sensor(start_addr+2);
	(* otp_ptr).VCM_start = (OV13850_read_cmos_sensor(start_addr)<<2) | ((temp>>6) & 0x03);
	(* otp_ptr).VCM_end = (OV13850_read_cmos_sensor(start_addr + 1) << 2) | ((temp>>4) & 0x03);
	(* otp_ptr).VCM_dir = (temp>>2) & 0x03;
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850_write_cmos_sensor(i, 0x00);
	}
return 0;
}
static int read_otp_lenc(int index, struct otp_struct * otp_ptr)
{
	int i;
	int start_addr, end_addr;
	int nFlagAddress= OTP_DRV_START_ADDR+1+OTP_DRV_INFO_GROUP_COUNT* OTP_DRV_INFO_SIZE
	+1+OTP_DRV_AWB_GROUP_COUNT* OTP_DRV_AWB_SIZE
	+1+OTP_DRV_VCM_GROUP_COUNT* OTP_DRV_VCM_SIZE ;
	start_addr = nFlagAddress+1+(index-1)*OTP_DRV_LSC_SIZE ;
	end_addr = start_addr+OTP_DRV_LSC_SIZE-1;
	OV13850_write_cmos_sensor(0x3d84, 0xC0);
	//partial mode OTP write start address
	OV13850_write_cmos_sensor(0x3d88, (start_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d89, start_addr & 0xff);
	// partial mode OTP write end address
	OV13850_write_cmos_sensor(0x3d8A, (end_addr >> 8) & 0xff);
	OV13850_write_cmos_sensor(0x3d8B, end_addr & 0xff);
	// read otp into buffer
	OV13850_write_cmos_sensor(0x3d81, 0x01);
	mDELAY(10);
	for(i=0; i<OTP_DRV_LSC_SIZE; i++) {
	(* otp_ptr).lenc[i] = OV13850_read_cmos_sensor(start_addr + i);
	
	OV13850DB("i is : %d, lenc is : 0x%x \n ",i,(* otp_ptr).lenc[i]);

	}
	// clear otp buffer
	for (i=start_addr; i<=end_addr; i++) {
	OV13850_write_cmos_sensor(i, 0x00);
	}
return 0;
}
// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
static int update_awb_gain(int R_gain, int G_gain, int B_gain)
{
	if (R_gain>0x400) {
	OV13850_write_cmos_sensor(0x5056, R_gain>>8);
	OV13850_write_cmos_sensor(0x5057, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
	OV13850_write_cmos_sensor(0x5058, G_gain>>8);
	OV13850_write_cmos_sensor(0x5059, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
	OV13850_write_cmos_sensor(0x505A, B_gain>>8);
	OV13850_write_cmos_sensor(0x505B, B_gain & 0x00ff);
	}
	return 0;
	// otp_ptr: pointer of otp_struct
}

static int update_lenc(struct otp_struct * otp_ptr)
{
	int i, temp;
	temp = OV13850_read_cmos_sensor(0x5000);
	temp = 0x01 | temp;
	OV13850_write_cmos_sensor(0x5000, temp);
	for(i=0;i<OTP_DRV_LSC_SIZE ;i++) {
	OV13850_write_cmos_sensor(OTP_DRV_LSC_REG_ADDR + i, (*otp_ptr).lenc[i]);
	}
	return 0;
}

// call this function after OV13850 initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_wb()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	int rg,bg;
	int R_gain, G_gain, B_gain;
	// R/G and B/G of current camera module is read out from sensor OTP
	// check first OTP with valid data
	for(i=1;i<=OTP_DRV_AWB_GROUP_COUNT;i++) {
	temp = check_otp_wb(i);
	if (temp == 2) {
	otp_index = i;
	break;
	}
	}
	if (i>OTP_DRV_AWB_GROUP_COUNT) {
	// no valid wb OTP data
	return 1;
	}
	read_otp_wb(otp_index, &current_otp);
	if(current_otp.light_rg==0) {
	// no light source information in OTP, light factor = 1
	rg = current_otp.rg_ratio;
	}
	else {
	rg = current_otp.rg_ratio * (current_otp.light_rg +512) / 1024;
	}
	if(current_otp.light_bg==0) {
	// not light source information in OTP, light factor = 1
	bg = current_otp.bg_ratio;
	}
	else {
	bg = current_otp.bg_ratio * (current_otp.light_bg +512) / 1024;
	}

	OV13850DB("rg=0x%x\n ",rg);
	OV13850DB("bg=0x%x\n ",bg);

	//calculate G gain
	int nR_G_gain, nB_G_gain, nG_G_gain;
	int nBase_gain;
	nR_G_gain = (RG_Ratio_Typical*1000) / rg;
	nB_G_gain = (BG_Ratio_Typical*1000) / bg;
	nG_G_gain = 1000;
	if (nR_G_gain < 1000 || nB_G_gain < 1000)
	{
	if (nR_G_gain < nB_G_gain)
	nBase_gain = nR_G_gain;
	else
	nBase_gain = nB_G_gain;
	}
	else
	{
	nBase_gain = nG_G_gain;
	}
	R_gain = 0x400 * nR_G_gain / (nBase_gain);
	B_gain = 0x400 * nB_G_gain / (nBase_gain);
	G_gain = 0x400 * nG_G_gain / (nBase_gain);

	OV13850DB("R_gain=0x%x\n ",R_gain);
	OV13850DB("B_gain=0x%x\n ",B_gain);
	OV13850DB("G_gain=0x%x\n ",G_gain);

	update_awb_gain(R_gain, G_gain, B_gain);
	return 0;

}

// call this function after OV13850 initialization
// return value: 0 update success
// 1, no OTP
static int update_otp_lenc()
{
	struct otp_struct current_otp;
	int i;
	int otp_index;
	int temp;
	// check first lens correction OTP with valid data
	for(i=1;i<=OTP_DRV_LSC_GROUP_COUNT;i++) {
	temp = check_otp_lenc(i);	
	OV13850DB("temp is : %d, i is :%d \n ",temp, i);
	if (temp == 2) {
	otp_index = i;
	break;}
	}
	if (i>OTP_DRV_LSC_GROUP_COUNT) {
	// no valid WB OTP data
	return 1;
	}
	read_otp_lenc(otp_index, &current_otp);
	update_lenc(&current_otp);
	// success
	return 0;
}

#endif



kal_uint16 OV13850_read_cmos_sensor(kal_uint32 addr)
{
kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV13850_WRITE_ID);
    return get_byte;
}

#define Sleep(ms) mdelay(ms)

void OV13850_write_shutter(kal_uint32 shutter)
{
//xb.pang
#if 1
	kal_uint32 min_framelength = OV13850_PV_PERIOD_PIXEL_NUMS, max_shutter=0;
	kal_uint32 extra_lines = 0;
	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
	unsigned long flags;

	OV13850DBSOFIA("!!shutter=%d!!!!!\n", shutter);


	if(ov13850.OV13850AutoFlickerMode == KAL_TRUE)
	{
		if ( SENSOR_MODE_PREVIEW == ov13850.sensorMode )  //(g_iOV13850_Mode == OV13850_MODE_PREVIEW)	//SXGA size output
		{
			line_length = OV13850_PV_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
					max_shutter = OV13850_PV_PERIOD_LINE_NUMS + ov13850.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == ov13850.sensorMode ) //add for video_6M setting
		{
			line_length = OV13850_VIDEO_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
			max_shutter = OV13850_VIDEO_PERIOD_LINE_NUMS + ov13850.DummyLines ;

		}
		else
		{
			line_length = OV13850_FULL_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
			max_shutter = OV13850_FULL_PERIOD_LINE_NUMS + ov13850.DummyLines ;

		}
		OV13850DBSOFIA("linelength %d, max_shutter %d!!\n",line_length,max_shutter);

		switch(OV13850CurrentScenarioId)
		{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				OV13850DBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_CAMERA_ZSD  0!!\n");
				min_framelength = max_shutter;

				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				
				OV13850DBSOFIA("AutoFlickerMode!!! MSDK_SCENARIO_ID_VIDEO_PREVIEW  0!!\n");
				if( ov13850VIDEO_MODE_TARGET_FPS==30)
				{
					min_framelength = (OV13850MIPI_VIDEO_CLK) /(OV13850_VIDEO_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/306*10 ;
				}
				else if( ov13850VIDEO_MODE_TARGET_FPS==15)
				{
					min_framelength = (OV13850MIPI_VIDEO_CLK) /(OV13850_VIDEO_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/148*10 ;
				}
				else
				{
					min_framelength = max_shutter;

				}
				break;
			default:
				min_framelength = (OV13850MIPI_PREVIEW_CLK) /(OV13850_PV_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/294*10 ;
    			break;
		}

		OV13850DBSOFIA("AutoFlickerMode!!! min_framelength for AutoFlickerMode = %d (0x%x)\n",min_framelength,min_framelength);
		OV13850DBSOFIA("max framerate(10 base) autofilker = %d\n",(OV13850MIPI_PREVIEW_CLK)*10 /line_length/min_framelength);

		if (shutter < 3)
			shutter = 3;

		//if (shutter > (max_shutter -4 ))
		//	extra_lines = shutter -( max_shutter -4 );
			if (shutter > (max_shutter-16))
			extra_lines = shutter -( max_shutter - 16);
		else
			extra_lines = 0;
		OV13850DBSOFIA("extra_lines 0=%d!!\n",extra_lines);

		if ( SENSOR_MODE_PREVIEW == ov13850.sensorMode )	//SXGA size output
		{
			frame_length = OV13850_PV_PERIOD_LINE_NUMS+ ov13850.DummyLines + extra_lines ;
		}
		else if(SENSOR_MODE_VIDEO == ov13850.sensorMode)
		{
			frame_length = OV13850_VIDEO_PERIOD_LINE_NUMS+ ov13850.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			frame_length = OV13850_FULL_PERIOD_LINE_NUMS + ov13850.DummyLines + extra_lines ;
		}
		OV13850DBSOFIA("frame_length 0= %d\n",frame_length);

		if (frame_length < min_framelength)
		{
			//shutter = min_framelength - 4;
 			
			switch(OV13850CurrentScenarioId)
			{
        	case MSDK_SCENARIO_ID_CAMERA_ZSD:
			case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				extra_lines = min_framelength- (OV13850_FULL_PERIOD_LINE_NUMS+ ov13850.DummyLines);
				break;
			case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
				extra_lines = min_framelength- (OV13850_VIDEO_PERIOD_LINE_NUMS+ ov13850.DummyLines);
			default:
				extra_lines = min_framelength- (OV13850_PV_PERIOD_LINE_NUMS+ ov13850.DummyLines);
    			break;
			}
			frame_length = min_framelength;
		}
		//Set total frame length
		if (frame_length >= 0x8000)
			frame_length = 0x7fff;
		OV13850_write_cmos_sensor(0x380e, (frame_length >> 8) & 0x7F);
		OV13850_write_cmos_sensor(0x380f, frame_length & 0xFF);
		spin_lock_irqsave(&ov13850mipiraw_drv_lock,flags);
		//ov13850.maxExposureLines = frame_length ;
				ov13850.maxExposureLines = frame_length -16 ;
		OV13850_FeatureControl_PERIOD_PixelNum = line_length;
		OV13850_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&ov13850mipiraw_drv_lock,flags);

		//Set shutter (Coarse integration time, uint: lines.)
		if (shutter > 0x7ffb)
			shutter = 0x7ffb;
		OV13850_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		OV13850_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
		OV13850_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	/* Don't use the fraction part. */
	OV13850DBSOFIA("frame_length %d,shutter %d!!\n",frame_length,shutter);
		//OV13850DB("framerate(10 base) = %d\n",(OV13850MIPI_PREVIEW_CLK)*10 /line_length/frame_length);
		//OV13850DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);

	}
	else
	{
		if ( SENSOR_MODE_PREVIEW == ov13850.sensorMode )  //(g_iOV13850_Mode == OV13850_MODE_PREVIEW)	//SXGA size output
		{
			max_shutter = OV13850_PV_PERIOD_LINE_NUMS + ov13850.DummyLines ;
		}
		else if( SENSOR_MODE_VIDEO == ov13850.sensorMode ) //add for video_6M setting
		{
			max_shutter = OV13850_VIDEO_PERIOD_LINE_NUMS + ov13850.DummyLines ;
					}
		else
		{
			max_shutter = OV13850_FULL_PERIOD_LINE_NUMS + ov13850.DummyLines ;
			
			}
		OV13850DBSOFIA(" max_shutter %d!!\n",max_shutter);

		if (shutter < 3)
			shutter = 3;

		//if (shutter > (max_shutter-4))
		//	extra_lines = shutter - (max_shutter -4);
					if (shutter > (max_shutter-16))
			extra_lines = shutter - (max_shutter -16);
		else
			extra_lines = 0;
		OV13850DBSOFIA("extra_lines 0=%d!!\n",extra_lines);

		if ( SENSOR_MODE_PREVIEW == ov13850.sensorMode )	//SXGA size output
		{
			line_length = OV13850_PV_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
			frame_length = OV13850_PV_PERIOD_LINE_NUMS+ ov13850.DummyLines + extra_lines ;
		}
		else if( SENSOR_MODE_VIDEO == ov13850.sensorMode )
		{
			line_length = OV13850_VIDEO_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
			frame_length = OV13850_VIDEO_PERIOD_LINE_NUMS + ov13850.DummyLines + extra_lines ;
		}
		else				//QSXGA size output
		{
			line_length = OV13850_FULL_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
			frame_length = OV13850_FULL_PERIOD_LINE_NUMS + ov13850.DummyLines + extra_lines ;
		}

		ASSERT(line_length < OV13850_MAX_LINE_LENGTH);		//0xCCCC
		ASSERT(frame_length < OV13850_MAX_FRAME_LENGTH); 	//0xFFFF
		
		//Set total frame length
		if (frame_length >= 0x8000)
			frame_length = 0x7fff;
		OV13850_write_cmos_sensor(0x380e, (frame_length >> 8) & 0x7F);
		OV13850_write_cmos_sensor(0x380f, frame_length & 0xFF);
		spin_lock_irqsave(&ov13850mipiraw_drv_lock,flags);
		//ov13850.maxExposureLines = frame_length -4;
		ov13850.maxExposureLines = frame_length -16;
		OV13850_FeatureControl_PERIOD_PixelNum = line_length;
		OV13850_FeatureControl_PERIOD_LineNum = frame_length;
		spin_unlock_irqrestore(&ov13850mipiraw_drv_lock,flags);

		//Set shutter (Coarse integration time, uint: lines.)
		if (shutter > 0x7ffb)
			shutter = 0x7ffb;
		OV13850_write_cmos_sensor(0x3500, (shutter>>12) & 0x0F);
		OV13850_write_cmos_sensor(0x3501, (shutter>>4) & 0xFF);
		OV13850_write_cmos_sensor(0x3502, (shutter<<4) & 0xF0);	/* Don't use the fraction part. */
		OV13850DBSOFIA("frame_length %d,shutter %d!!\n",frame_length,shutter);

		OV13850DB("framerate(10 base) = %d\n",(OV13850MIPI_PREVIEW_CLK)*10 /line_length/frame_length);

		OV13850DB("shutter=%d, extra_lines=%d, line_length=%d, frame_length=%d\n", shutter, extra_lines, line_length, frame_length);
	}
#endif
}   /* write_OV13850_shutter */

static kal_uint16 OV13850Reg2Gain(const kal_uint16 iReg)
{
    kal_uint8 iI;
    kal_uint16 iGain = ov13850.ispBaseGain;    // 1x-gain base

    // Range: 1x to 32x
    // Gain = (GAIN[9] + 1) *(GAIN[8] + 1) *(GAIN[7] + 1) * (GAIN[6] + 1) * (GAIN[5] + 1) * (GAIN[4] + 1) * (1 + GAIN[3:0] / 16)
    //for (iI = 8; iI >= 4; iI--) {
    //    iGain *= (((iReg >> iI) & 0x01) + 1);
    //}
    iGain = iReg * ov13850.ispBaseGain / 32;
    return iGain; //ov13850.realGain
}

static kal_uint16 OV13850Gain2Reg(const kal_uint16 Gain)
{
    kal_uint16 iReg = 0x0000;
	kal_uint16 iGain=Gain;
	//if(iGain <  ov13850.ispBaseGain) 
	//{
		iReg = Gain*32/ov13850.ispBaseGain;
		if(iReg < 0x20)
		{
			iReg = 0x20;
		}
		if(iReg > 0xfc)
		{
			iReg = 0xfc;
		}
	//}
	//else
	//{
	//	OV13850DB("out of range!\n");
	//}
	OV13850DBSOFIA("[OV13850Gain2Reg]: isp gain:%d,sensor gain:0x%x\n",iGain,iReg);

    return iReg;//ov13850. sensorGlobalGain

}

void write_OV13850_gain(kal_uint16 gain)
{
//xb.pang
	OV13850_write_cmos_sensor(0x350a,(gain>>8));
	OV13850_write_cmos_sensor(0x350b,(gain&0xff));
	return;
}
void OV13850_SetGain(UINT16 iGain)
{
	unsigned long flags;
	spin_lock_irqsave(&ov13850mipiraw_drv_lock,flags);
	ov13850.realGain = iGain;
	ov13850.sensorGlobalGain = OV13850Gain2Reg(iGain);
	spin_unlock_irqrestore(&ov13850mipiraw_drv_lock,flags);
	write_OV13850_gain(ov13850.sensorGlobalGain);
	OV13850DB("[OV13850_SetGain]ov13850.sensorGlobalGain=0x%x,ov13850.realGain=%d\n",ov13850.sensorGlobalGain,ov13850.realGain);

}   /*  OV13850_SetGain_SetGain  */

kal_uint16 read_OV13850_gain(void)
{
	kal_uint16 read_gain=0;
	read_gain=(((OV13850_read_cmos_sensor(0x350a)&0x01) << 8) | OV13850_read_cmos_sensor(0x350b));
	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.sensorGlobalGain = read_gain;
	ov13850.realGain = OV13850Reg2Gain(ov13850.sensorGlobalGain);
	spin_unlock(&ov13850mipiraw_drv_lock);
	OV13850DB("ov13850.sensorGlobalGain=0x%x,ov13850.realGain=%d\n",ov13850.sensorGlobalGain,ov13850.realGain);
	return ov13850.sensorGlobalGain;
}  /* read_OV13850_gain */


void OV13850_camera_para_to_sensor(void)
{}

void OV13850_sensor_to_camera_para(void)
{}

kal_int32  OV13850_get_sensor_group_count(void)
{
    return GROUP_TOTAL_NUMS;
}

void OV13850_get_sensor_group_info(kal_uint16 group_idx, kal_int8* group_name_ptr, kal_int32* item_count_ptr)
{}
void OV13850_get_sensor_item_info(kal_uint16 group_idx,kal_uint16 item_idx, MSDK_SENSOR_ITEM_INFO_STRUCT* info_ptr)
{}
kal_bool OV13850_set_sensor_item_info(kal_uint16 group_idx, kal_uint16 item_idx, kal_int32 ItemValue)
{    return KAL_TRUE;}

static void OV13850_SetDummy( const kal_uint32 iPixels, const kal_uint32 iLines )
{
 	kal_uint32 line_length = 0;
	kal_uint32 frame_length = 0;
//xb.pang
#if 1
	if ( SENSOR_MODE_PREVIEW == ov13850.sensorMode )	//SXGA size output
	{
		line_length = OV13850_PV_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV13850_PV_PERIOD_LINE_NUMS + iLines;
	}
	else if( SENSOR_MODE_VIDEO== ov13850.sensorMode )
	{
		line_length = OV13850_VIDEO_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV13850_VIDEO_PERIOD_LINE_NUMS + iLines;
	}
	else//QSXGA size output
	{
		line_length = OV13850_FULL_PERIOD_PIXEL_NUMS + iPixels;
		frame_length = OV13850_FULL_PERIOD_LINE_NUMS + iLines;
	}

	//if(ov13850.maxExposureLines > frame_length -4 )
	//	return;

	//ASSERT(line_length < OV13850_MAX_LINE_LENGTH);		//0xCCCC
	//ASSERT(frame_length < OV13850_MAX_FRAME_LENGTH);	//0xFFFF

	//Set total frame length
	if (frame_length >= 0x8000)
			frame_length = 0x7fff;
	OV13850_write_cmos_sensor(0x380e, (frame_length >> 8) & 0x7F);
	OV13850_write_cmos_sensor(0x380f, frame_length & 0xFF);

	spin_lock(&ov13850mipiraw_drv_lock);
	//ov13850.maxExposureLines = frame_length -4;
	ov13850.maxExposureLines = frame_length -16;
	OV13850_FeatureControl_PERIOD_PixelNum = line_length;
	OV13850_FeatureControl_PERIOD_LineNum = frame_length;
	spin_unlock(&ov13850mipiraw_drv_lock);

	//Set total line length
	OV13850_write_cmos_sensor(0x380c, (line_length >> 8) & 0x7F);
	OV13850_write_cmos_sensor(0x380d, line_length & 0xFF);
	#endif
	
	OV13850DB("OV13850_SetDummy linelength %d,  frame_length= %d \n",line_length,frame_length);

}   /*  OV13850_SetDummy */


static kal_uint8 ov13850_init[] = {
		//for PCLK :384M or fast:pclk:240M
	/*
	@@ RES_4208x3120 24fps
	;24Mhz Xclk
	;SCLK 96Mhz, Pclk 384MHz
	;4Lane, MIPI datarate 960Mbps/Lane
	;24fps
	;pixels per line=4800(0x12c0) 
	;lines per frame=3328(0xD00)
	
	;102 2630 960
	;88 e7 3f
	
	;100 99 4208 3120
	;100 98 1 0
	;102 81 0
	;102 3601 964
	
	;102 910 31
	;102 84 1
	
	;c8 0300 62
	*/
	//0x01, 0x03, 0x01,//reset , need delay
	
     0x03, 0x00, 0x01,
     0x03, 0x01, 0x00,
     0x03, 0x02, 0x28,
     0x03, 0x03, 0x00,
     0x03, 0x0a, 0x00,
     0x30, 0x0f, 0x11,
     0x30, 0x10, 0x01,
     0x30, 0x11, 0x76,
     0x30, 0x12, 0x41,
     0x30, 0x13, 0x12,
     0x30, 0x14, 0x11,
     0x30, 0x1f, 0x03,
     0x31, 0x06, 0x00,
     0x32, 0x10, 0x47,
     0x35, 0x00, 0x00,
     0x35, 0x01, 0x60,
     0x35, 0x02, 0x00,
     0x35, 0x06, 0x00,
     0x35, 0x07, 0x02,
     0x35, 0x08, 0x00,
     0x35, 0x0a, 0x00,
     0x35, 0x0b, 0x80,
     0x35, 0x0e, 0x00,
     0x35, 0x0f, 0x10,
     0x35, 0x1a, 0x00,
     0x35, 0x1b, 0x10,
     0x35, 0x1c, 0x00,
     0x35, 0x1d, 0x20,
     0x35, 0x1e, 0x00,
     0x35, 0x1f, 0x40,
     0x35, 0x20, 0x00,
     0x35, 0x21, 0x80,
     0x36, 0x00, 0xc0,
     0x36, 0x01, 0xfc,
     0x36, 0x02, 0x02,
     0x36, 0x03, 0x78,
     0x36, 0x04, 0xb1,
     0x36, 0x05, 0xb5,
     0x36, 0x06, 0x73,
     0x36, 0x07, 0x07,
     0x36, 0x09, 0x40,
     0x36, 0x0a, 0x30,
     0x36, 0x0b, 0x91,
     0x36, 0x0c, 0x09,
     0x36, 0x0f, 0x02,
     0x36, 0x11, 0x10,
     0x36, 0x12, 0x27,
     0x36, 0x13, 0x33,
     0x36, 0x15, 0x0c,
     0x36, 0x16, 0x0e,
     0x36, 0x41, 0x02,
     0x36, 0x60, 0x82,
     0x36, 0x68, 0x54,
     0x36, 0x69, 0x00,
     0x36, 0x6a, 0x3f,
     0x36, 0x67, 0xa0,
     0x37, 0x02, 0x40,
     0x37, 0x03, 0x44,
     0x37, 0x04, 0x2c,
     0x37, 0x05, 0x01,
     0x37, 0x06, 0x15,
     0x37, 0x07, 0x44,
     0x37, 0x08, 0x3c,
     0x37, 0x09, 0x1f,
     0x37, 0x0a, 0x27,
     0x37, 0x0b, 0x3c,
     0x37, 0x20, 0x55,
     0x37, 0x22, 0x84,
     0x37, 0x28, 0x40,
     0x37, 0x2a, 0x00,
     0x37, 0x2b, 0x02,
     0x37, 0x2e, 0x22,
     0x37, 0x2f, 0x90,
     0x37, 0x30, 0x00,
     0x37, 0x31, 0x00,
     0x37, 0x32, 0x00,
     0x37, 0x33, 0x00,
     0x37, 0x10, 0x28,
     0x37, 0x16, 0x03,
     0x37, 0x18, 0x10,
     0x37, 0x19, 0x0c,
     0x37, 0x1a, 0x08,
     0x37, 0x1c, 0xfc,
     0x37, 0x48, 0x00,
     0x37, 0x60, 0x13,
     0x37, 0x61, 0x33,
     0x37, 0x62, 0x86,
     0x37, 0x63, 0x16,
     0x37, 0x67, 0x24,
     0x37, 0x68, 0x06,
     0x37, 0x69, 0x45,
     0x37, 0x6c, 0x23,
     0x37, 0x6f, 0x80,
     0x37, 0x73, 0x06,
     0x3d, 0x84, 0x00,
     0x3d, 0x85, 0x17,
     0x3d, 0x8c, 0x73,
     0x3d, 0x8d, 0xbf,
     0x38, 0x00, 0x00,
     0x38, 0x01, 0x08,
     0x38, 0x02, 0x00,
     0x38, 0x03, 0x04,
     0x38, 0x04, 0x10,
     0x38, 0x05, 0x97,
     0x38, 0x06, 0x0c,
     0x38, 0x07, 0x4b,
     0x38, 0x08, 0x08,
     0x38, 0x09, 0x40,
     0x38, 0x0a, 0x06,
     0x38, 0x0b, 0x20,
     0x38, 0x0c, 0x09,
     0x38, 0x0d, 0x60,
     0x38, 0x0e, 0x06,
     0x38, 0x0f, 0x80,
     0x38, 0x10, 0x00,
     0x38, 0x11, 0x04,
     0x38, 0x12, 0x00,
     0x38, 0x13, 0x02,
     0x38, 0x14, 0x31,
     0x38, 0x15, 0x31,
     0x38, 0x20, 0x02,
     0x38, 0x21, 0x06,
     0x38, 0x23, 0x00,
     0x38, 0x26, 0x00,
     0x38, 0x27, 0x02,
     0x38, 0x34, 0x00,
     0x38, 0x35, 0x1c,
     0x38, 0x36, 0x08,
     0x38, 0x37, 0x02,
     0x40, 0x00, 0xf1,
     0x40, 0x01, 0x00,
     0x40, 0x06, 0x04,
     0x40, 0x07, 0x04,
     0x40, 0x0b, 0x0c,
     0x40, 0x11, 0x00,
     0x40, 0x1a, 0x00,
     0x40, 0x1b, 0x00,
     0x40, 0x1c, 0x00,
     0x40, 0x1d, 0x00,
     0x40, 0x20, 0x00,
     0x40, 0x21, 0xe4,
     0x40, 0x22, 0x04,
     0x40, 0x23, 0xd7,
     0x40, 0x24, 0x05,
     0x40, 0x25, 0xbc,
     0x40, 0x26, 0x05,
     0x40, 0x27, 0xbf,
     0x40, 0x28, 0x00,
     0x40, 0x29, 0x02,
     0x40, 0x2a, 0x04,
     0x40, 0x2b, 0x08,
     0x40, 0x2c, 0x02,
     0x40, 0x2d, 0x02,
     0x40, 0x2e, 0x0c,
     0x40, 0x2f, 0x08,
     0x40, 0x3d, 0x2c,
     0x40, 0x3f, 0x7f,
     0x40, 0x41, 0x07,
     0x45, 0x00, 0x82,
     0x45, 0x01, 0x3c,
     0x45, 0x8b, 0x00,
     0x45, 0x9c, 0x00,
     0x45, 0x9d, 0x00,
     0x45, 0x9e, 0x00,
     0x46, 0x01, 0x83,
     0x46, 0x02, 0x22,
     0x46, 0x03, 0x01,
     0x48, 0x37, 0x19,
     0x4d, 0x00, 0x04,
     0x4d, 0x01, 0x42,
     0x4d, 0x02, 0xd1,
     0x4d, 0x03, 0x90,
     0x4d, 0x04, 0x66,
     0x4d, 0x05, 0x65,
     0x4d, 0x0b, 0x00,
     0x50, 0x00, 0x0e,
     0x50, 0x01, 0x01,
     0x50, 0x02, 0x07,
     0x50, 0x13, 0x40,
     0x50, 0x1c, 0x00,
     0x50, 0x1d, 0x10,
     0x51, 0x0f, 0xfc,
     0x51, 0x10, 0xf0,
     0x51, 0x11, 0x10,
     0x53, 0x6d, 0x02,
     0x53, 0x6e, 0x67,
     0x53, 0x6f, 0x01,
     0x53, 0x70, 0x4c,
     0x54, 0x00, 0x00,
     0x54, 0x00, 0x00,
     0x54, 0x01, 0x61,
     0x54, 0x02, 0x00,
     0x54, 0x03, 0x00,
     0x54, 0x04, 0x00,
     0x54, 0x05, 0x40,
     0x54, 0x0c, 0x05,
     0x55, 0x01, 0x00,
     0x5b, 0x00, 0x00,
     0x5b, 0x01, 0x00,
     0x5b, 0x02, 0x01,
     0x5b, 0x03, 0xff,
     0x5b, 0x04, 0x02,
     0x5b, 0x05, 0x6c,
     0x5b, 0x09, 0x02,
     0x5e, 0x00, 0x00,
     0x5e, 0x10, 0x1c,
     0x01, 0x00, 0x01,

};

void OV13850PreviewSetting(void)
{
OV13850DB("ofilm OV13850PreviewSetting \n");
    /*
	@@ 0 20 RES_2112x1568 30fps_key setting
	;24Mhz Xclk
	;SCLK 60Mhz, Pclk 240MHz
	;4Lane, MIPI datarate 640Mbps/Lane
	;30fps
	;pixels per line=4800(0x12c0) 
	;lines per frame=1664(0x680)

	;100 99 2112 1568
	;102 3601 BBD
	*/
	OV13850DB("cheysh, OV13850 %s\n",__func__);
	OV13850_write_cmos_sensor(0x0100, 0x00);
	
	OV13850_write_cmos_sensor(0x0102, 0x01);  /*tydrv qupw add for avoid -3 AE huaping*/

    OV13850_write_cmos_sensor(0x0300, 0x01);
    OV13850_write_cmos_sensor(0x0302, 0x28);
    OV13850_write_cmos_sensor(0x0303, 0x00);
    OV13850_write_cmos_sensor(0x3612, 0x27);
    OV13850_write_cmos_sensor(0x3702, 0x40);
    OV13850_write_cmos_sensor(0x370a, 0x27);
    OV13850_write_cmos_sensor(0x3718, 0x1c);
    OV13850_write_cmos_sensor(0x372a, 0x00);
    OV13850_write_cmos_sensor(0x372f, 0xa0);
    OV13850_write_cmos_sensor(0x3801, 0x00);
    OV13850_write_cmos_sensor(0x3802, 0x00);
    OV13850_write_cmos_sensor(0x3803, 0x04);
    OV13850_write_cmos_sensor(0x3805, 0x9f);
    OV13850_write_cmos_sensor(0x3806, 0x0c);
    OV13850_write_cmos_sensor(0x3807, 0x4b);
    OV13850_write_cmos_sensor(0x3808, 0x08);
    OV13850_write_cmos_sensor(0x3809, 0x40);
    OV13850_write_cmos_sensor(0x380a, 0x06);
    OV13850_write_cmos_sensor(0x380b, 0x20);
    OV13850_write_cmos_sensor(0x380c, 0x09);
    OV13850_write_cmos_sensor(0x380d, 0x60);
    OV13850_write_cmos_sensor(0x380e, 0x0d);
    OV13850_write_cmos_sensor(0x380f, 0x00);
    OV13850_write_cmos_sensor(0x3811, 0x08);
    OV13850_write_cmos_sensor(0x3813, 0x02);
    OV13850_write_cmos_sensor(0x3814, 0x31);
    OV13850_write_cmos_sensor(0x3815, 0x31);
    OV13850_write_cmos_sensor(0x3820, 0x01);
    OV13850_write_cmos_sensor(0x3821, 0x06);
    OV13850_write_cmos_sensor(0x3834, 0x00);
    OV13850_write_cmos_sensor(0x3836, 0x08);
    OV13850_write_cmos_sensor(0x3837, 0x02);
    OV13850_write_cmos_sensor(0x4020, 0x00);
    OV13850_write_cmos_sensor(0x4021, 0xe4);
    OV13850_write_cmos_sensor(0x4022, 0x04);
    OV13850_write_cmos_sensor(0x4023, 0xd7);
    OV13850_write_cmos_sensor(0x4024, 0x05);
    OV13850_write_cmos_sensor(0x4025, 0xbc);
    OV13850_write_cmos_sensor(0x4026, 0x05);
    OV13850_write_cmos_sensor(0x4027, 0xbf);
    OV13850_write_cmos_sensor(0x402a, 0x04);
    OV13850_write_cmos_sensor(0x402b, 0x08);
    OV13850_write_cmos_sensor(0x402c, 0x02);
    OV13850_write_cmos_sensor(0x402e, 0x0c);
    OV13850_write_cmos_sensor(0x402f, 0x08);
    OV13850_write_cmos_sensor(0x4501, 0x3c);
    OV13850_write_cmos_sensor(0x4601, 0x83);
    OV13850_write_cmos_sensor(0x4603, 0x01);
    OV13850_write_cmos_sensor(0x4837, 0x19);
    OV13850_write_cmos_sensor(0x5401, 0x61);
    OV13850_write_cmos_sensor(0x5405, 0x40); 
	OV13850_write_cmos_sensor(0x350b, 0x20);
	OV13850_write_cmos_sensor(0x3500, 0x00);
	OV13850_write_cmos_sensor(0x3501, 0xbb);
	OV13850_write_cmos_sensor(0x3502, 0x20);
	OV13850_write_cmos_sensor(0x0100, 0x01);
	mdelay(10);	

}


void OV13850VideoSetting(void)
{
	OV13850DB("OV13850VideoSetting \n");
	 /*
	@@ 0 20 RES_2112x1568 30fps_key setting
	;24Mhz Xclk
	;SCLK 60Mhz, Pclk 240MHz
	;4Lane, MIPI datarate 640Mbps/Lane
	;30fps
	;pixels per line=4800(0x12c0) 
	;lines per frame=1664(0x680)

	;100 99 2112 1568
	;102 3601 BBD
	*/
	OV13850DB("cheysh, OV13850 %s\n",__func__);   
	OV13850_write_cmos_sensor(0x0100, 0x00);
    OV13850_write_cmos_sensor(0x0300, 0x01);
    OV13850_write_cmos_sensor(0x0302, 0x28);
    OV13850_write_cmos_sensor(0x0303, 0x00);
    OV13850_write_cmos_sensor(0x3612, 0x27);
    OV13850_write_cmos_sensor(0x3702, 0x40);
    OV13850_write_cmos_sensor(0x370a, 0x27);
    OV13850_write_cmos_sensor(0x3718, 0x1c);
    OV13850_write_cmos_sensor(0x372a, 0x00);
    OV13850_write_cmos_sensor(0x372f, 0xa0);
    OV13850_write_cmos_sensor(0x3801, 0x00);
    OV13850_write_cmos_sensor(0x3802, 0x00);
    OV13850_write_cmos_sensor(0x3803, 0x04);
    OV13850_write_cmos_sensor(0x3805, 0x9f);
    OV13850_write_cmos_sensor(0x3806, 0x0c);
    OV13850_write_cmos_sensor(0x3807, 0x4b);
    OV13850_write_cmos_sensor(0x3808, 0x08);
    OV13850_write_cmos_sensor(0x3809, 0x40);
    OV13850_write_cmos_sensor(0x380a, 0x06);
    OV13850_write_cmos_sensor(0x380b, 0x20);
    OV13850_write_cmos_sensor(0x380c, 0x09);
    OV13850_write_cmos_sensor(0x380d, 0x60);
    OV13850_write_cmos_sensor(0x380e, 0x0d);
    OV13850_write_cmos_sensor(0x380f, 0x00);
    OV13850_write_cmos_sensor(0x3811, 0x08);
    OV13850_write_cmos_sensor(0x3813, 0x02);
    OV13850_write_cmos_sensor(0x3814, 0x31);
    OV13850_write_cmos_sensor(0x3815, 0x31);
    OV13850_write_cmos_sensor(0x3820, 0x01);
    OV13850_write_cmos_sensor(0x3821, 0x06);
    OV13850_write_cmos_sensor(0x3834, 0x00);
    OV13850_write_cmos_sensor(0x3836, 0x08);
    OV13850_write_cmos_sensor(0x3837, 0x02);
    OV13850_write_cmos_sensor(0x4020, 0x00);
    OV13850_write_cmos_sensor(0x4021, 0xe4);
    OV13850_write_cmos_sensor(0x4022, 0x04);
    OV13850_write_cmos_sensor(0x4023, 0xd7);
    OV13850_write_cmos_sensor(0x4024, 0x05);
    OV13850_write_cmos_sensor(0x4025, 0xbc);
    OV13850_write_cmos_sensor(0x4026, 0x05);
    OV13850_write_cmos_sensor(0x4027, 0xbf);
    OV13850_write_cmos_sensor(0x402a, 0x04);
    OV13850_write_cmos_sensor(0x402b, 0x08);
    OV13850_write_cmos_sensor(0x402c, 0x02);
    OV13850_write_cmos_sensor(0x402e, 0x0c);
    OV13850_write_cmos_sensor(0x402f, 0x08);
    OV13850_write_cmos_sensor(0x4501, 0x3c);
    OV13850_write_cmos_sensor(0x4601, 0x83);
    OV13850_write_cmos_sensor(0x4603, 0x01);
    OV13850_write_cmos_sensor(0x4837, 0x19);
    OV13850_write_cmos_sensor(0x5401, 0x61);
    OV13850_write_cmos_sensor(0x5405, 0x40); 
	OV13850_write_cmos_sensor(0x350b, 0x20);
	OV13850_write_cmos_sensor(0x3500, 0x00);
	OV13850_write_cmos_sensor(0x3501, 0xbb);
	OV13850_write_cmos_sensor(0x3502, 0x20);
	OV13850_write_cmos_sensor(0x0100, 0x01);
	mdelay(10);
}




void OV13850CaptureSetting(void)
{
	/*
	@@ RES_4224x3136 24fps-Key setting
	;24Mhz Xclk
	;SCLK 96Mhz, Pclk 384MHz
	;4Lane, MIPI datarate 600Mbps/Lane
	;15fps
	;pixels per line=4800(0x12c0) 
	;lines per frame=3328(0xD00)

	;100 99 4208 3120
	;102 3601 964
	*/
	OV13850DB("cheysh, OV13850 %s\n",__func__);
	#if 1
	OV13850_write_cmos_sensor(0x0100, 0x00);  
  OV13850_write_cmos_sensor(0x0300, 0x00);
  OV13850_write_cmos_sensor(0x0302, 0x32);
  OV13850_write_cmos_sensor(0x0303, 0x01);
  OV13850_write_cmos_sensor(0x3612, 0x27);
  OV13850_write_cmos_sensor(0x3702, 0x40);
  OV13850_write_cmos_sensor(0x370a, 0x24);
  OV13850_write_cmos_sensor(0x3718, 0x10);
  OV13850_write_cmos_sensor(0x372a, 0x04);
  OV13850_write_cmos_sensor(0x372f, 0xa0);
  OV13850_write_cmos_sensor(0x3801, 0x0c);
  OV13850_write_cmos_sensor(0x3802, 0x00);
  OV13850_write_cmos_sensor(0x3803, 0x04);
  OV13850_write_cmos_sensor(0x3805, 0x93);
  OV13850_write_cmos_sensor(0x3806, 0x0c);
  OV13850_write_cmos_sensor(0x3807, 0x4b);
  OV13850_write_cmos_sensor(0x3808, 0x10);
  OV13850_write_cmos_sensor(0x3809, 0x80);
  OV13850_write_cmos_sensor(0x380a, 0x0c);
  OV13850_write_cmos_sensor(0x380b, 0x40);
  OV13850_write_cmos_sensor(0x380c, 0x12);
  OV13850_write_cmos_sensor(0x380d, 0xc0);
  OV13850_write_cmos_sensor(0x380e, 0x0d);
  OV13850_write_cmos_sensor(0x380f, 0x00);
  OV13850_write_cmos_sensor(0x3811, 0x04);
  OV13850_write_cmos_sensor(0x3813, 0x04);
  OV13850_write_cmos_sensor(0x3814, 0x11);
  OV13850_write_cmos_sensor(0x3815, 0x11);
  OV13850_write_cmos_sensor(0x3820, 0x00);
  OV13850_write_cmos_sensor(0x3821, 0x04);
  OV13850_write_cmos_sensor(0x3834, 0x00);
  OV13850_write_cmos_sensor(0x3836, 0x04);
  OV13850_write_cmos_sensor(0x3837, 0x01);
  OV13850_write_cmos_sensor(0x4020, 0x02);
  OV13850_write_cmos_sensor(0x4021, 0x4c);
  OV13850_write_cmos_sensor(0x4022, 0x0E);
  OV13850_write_cmos_sensor(0x4023, 0x37);
  OV13850_write_cmos_sensor(0x4024, 0x0F);
  OV13850_write_cmos_sensor(0x4025, 0x1C);
  OV13850_write_cmos_sensor(0x4026, 0x0F);
  OV13850_write_cmos_sensor(0x4027, 0x1F);
  OV13850_write_cmos_sensor(0x402a, 0x04);
  OV13850_write_cmos_sensor(0x402b, 0x08);
  OV13850_write_cmos_sensor(0x402c, 0x02);
  OV13850_write_cmos_sensor(0x402e, 0x0c);
  OV13850_write_cmos_sensor(0x402f, 0x08);
  OV13850_write_cmos_sensor(0x4501, 0x38);
  OV13850_write_cmos_sensor(0x4601, 0x04);
  OV13850_write_cmos_sensor(0x4603, 0x00);
  OV13850_write_cmos_sensor(0x4837, 0x1b);
  OV13850_write_cmos_sensor(0x5401, 0x71);
  OV13850_write_cmos_sensor(0x5405, 0x80);  
	OV13850_write_cmos_sensor(0x350b, 0x80);  
	OV13850_write_cmos_sensor(0x3500, 0x00);  
	OV13850_write_cmos_sensor(0x3501, 0xCF);  
	OV13850_write_cmos_sensor(0x3502, 0x80);  
	OV13850_write_cmos_sensor(0x0100, 0x01);  
    #else
	//OV13850_write_cmos_sensor(0x0100, 0x00);
	OV13850_write_cmos_sensor(0x3208 , 0xa2);
	//OV13850_write_cmos_sensor(0x0100, 0x01);
	#endif

}



static void OV13850_Sensor_Init(void)
{

#if 0

	OV13850_write_cmos_sensor(0x0103,0x01);
	OV13850_write_cmos_sensor(0x3001,0x06);
	OV13850_write_cmos_sensor(0x3002,0x80);
	OV13850_write_cmos_sensor(0x3011,0x41);
	OV13850_write_cmos_sensor(0x3014,0x16);
	OV13850_write_cmos_sensor(0x3015,0x0b);
	OV13850_write_cmos_sensor(0x3022,0x03);
	OV13850_write_cmos_sensor(0x3090,0x02);
	OV13850_write_cmos_sensor(0x3091,0x1b);
	OV13850_write_cmos_sensor(0x3092,0x00);
	OV13850_write_cmos_sensor(0x3093,0x00);
	OV13850_write_cmos_sensor(0x3098,0x03);
	OV13850_write_cmos_sensor(0x3099,0x11);
	OV13850_write_cmos_sensor(0x309c,0x01);
	OV13850_write_cmos_sensor(0x30b4,0x03);
	OV13850_write_cmos_sensor(0x30b5,0x04);
	OV13850_write_cmos_sensor(0x3304,0x28);
	OV13850_write_cmos_sensor(0x3305,0x41);
	OV13850_write_cmos_sensor(0x3306,0x30);
	OV13850_write_cmos_sensor(0x3308,0x00);
	OV13850_write_cmos_sensor(0x3309,0xc8);
	OV13850_write_cmos_sensor(0x330a,0x01);
	OV13850_write_cmos_sensor(0x330b,0x90);
	OV13850_write_cmos_sensor(0x330c,0x02);
	OV13850_write_cmos_sensor(0x330d,0x58);
	OV13850_write_cmos_sensor(0x330e,0x03);
	OV13850_write_cmos_sensor(0x330f,0x20);
	OV13850_write_cmos_sensor(0x3300,0x00);
	OV13850_write_cmos_sensor(0x3500,0x00);
	OV13850_write_cmos_sensor(0x3503,0x07);
	OV13850_write_cmos_sensor(0x3509,0x08);
	OV13850_write_cmos_sensor(0x350a,0x00);
	OV13850_write_cmos_sensor(0x350b,0x80);
	OV13850_write_cmos_sensor(0x3602,0x28);
	OV13850_write_cmos_sensor(0x3612,0x80);
	OV13850_write_cmos_sensor(0x3622,0x0f);
	OV13850_write_cmos_sensor(0x3631,0xb3);
	OV13850_write_cmos_sensor(0x3634,0x04);
	OV13850_write_cmos_sensor(0x3660,0x80);
	OV13850_write_cmos_sensor(0x3662,0x10);
	OV13850_write_cmos_sensor(0x3663,0xf0);
	OV13850_write_cmos_sensor(0x3667,0x00);
	OV13850_write_cmos_sensor(0x366f,0x20);
	OV13850_write_cmos_sensor(0x3680,0xb5);
	OV13850_write_cmos_sensor(0x3682,0x00);
	OV13850_write_cmos_sensor(0x370b,0xa8);
	OV13850_write_cmos_sensor(0x370d,0x11);
	OV13850_write_cmos_sensor(0x370e,0x00);
	OV13850_write_cmos_sensor(0x371c,0x01);
	OV13850_write_cmos_sensor(0x3726,0x00);
	OV13850_write_cmos_sensor(0x372a,0x09);
	OV13850_write_cmos_sensor(0x3739,0x7c);
	OV13850_write_cmos_sensor(0x373c,0x44);
	OV13850_write_cmos_sensor(0x376b,0x44);
	OV13850_write_cmos_sensor(0x377b,0x44);
	OV13850_write_cmos_sensor(0x3780,0x22);
	OV13850_write_cmos_sensor(0x3781,0x0c);
	OV13850_write_cmos_sensor(0x3783,0x31);
	OV13850_write_cmos_sensor(0x379c,0x0c);
	OV13850_write_cmos_sensor(0x37c5,0x00);
	OV13850_write_cmos_sensor(0x37c6,0x00);
	OV13850_write_cmos_sensor(0x37c7,0x00);
	OV13850_write_cmos_sensor(0x37c9,0x00);
	OV13850_write_cmos_sensor(0x37ca,0x00);
	OV13850_write_cmos_sensor(0x37cb,0x00);
	OV13850_write_cmos_sensor(0x37cc,0x00);
	OV13850_write_cmos_sensor(0x37cd,0x00);
	OV13850_write_cmos_sensor(0x37ce,0x10);
	OV13850_write_cmos_sensor(0x37cf,0x00);
	OV13850_write_cmos_sensor(0x37d0,0x00);
	OV13850_write_cmos_sensor(0x37d1,0x00);
	OV13850_write_cmos_sensor(0x37d2,0x00);
	OV13850_write_cmos_sensor(0x37de,0x00);
	OV13850_write_cmos_sensor(0x37df,0x00);
	OV13850_write_cmos_sensor(0x3800,0x00);
	OV13850_write_cmos_sensor(0x3804,0x10);
	OV13850_write_cmos_sensor(0x3810,0x00);
	OV13850_write_cmos_sensor(0x3812,0x00);
	OV13850_write_cmos_sensor(0x3829,0x0b);
	OV13850_write_cmos_sensor(0x382b,0x6a);
	OV13850_write_cmos_sensor(0x4000,0x10);
	OV13850_write_cmos_sensor(0x4001,0x06);
	OV13850_write_cmos_sensor(0x4002,0x45);
	OV13850_write_cmos_sensor(0x4005,0x18);
	OV13850_write_cmos_sensor(0x4008,0x24);
	OV13850_write_cmos_sensor(0x4100,0x50);
	OV13850_write_cmos_sensor(0x4101,0xb2);
	OV13850_write_cmos_sensor(0x4102,0x34);
	OV13850_write_cmos_sensor(0x4104,0xdc);
	OV13850_write_cmos_sensor(0x4109,0x62);
	OV13850_write_cmos_sensor(0x4300,0xff);
	OV13850_write_cmos_sensor(0x4303,0x00);
	OV13850_write_cmos_sensor(0x4304,0x08);
	OV13850_write_cmos_sensor(0x4307,0x30);
	OV13850_write_cmos_sensor(0x4311,0x04);
	OV13850_write_cmos_sensor(0x4511,0x05);
	OV13850_write_cmos_sensor(0x4816,0x52);
	OV13850_write_cmos_sensor(0x481f,0x30);
	OV13850_write_cmos_sensor(0x4826,0x2c);
	OV13850_write_cmos_sensor(0x4a00,0xaa);
	OV13850_write_cmos_sensor(0x4a03,0x01);
	OV13850_write_cmos_sensor(0x4a05,0x08);
	OV13850_write_cmos_sensor(0x4d01,0x71);
	OV13850_write_cmos_sensor(0x4d02,0xfd);
	OV13850_write_cmos_sensor(0x4d03,0xf5);
	OV13850_write_cmos_sensor(0x4d04,0x0c);
	OV13850_write_cmos_sensor(0x4d05,0xcc);
	OV13850_write_cmos_sensor(0x5000,0x06);
	OV13850_write_cmos_sensor(0x5001,0x01);
	OV13850_write_cmos_sensor(0x5003,0x21);
	OV13850_write_cmos_sensor(0x5043,0x48);
	OV13850_write_cmos_sensor(0x5013,0x80);
	OV13850_write_cmos_sensor(0x501f,0x00);
	OV13850_write_cmos_sensor(0x5e00,0x00);
	OV13850_write_cmos_sensor(0x5a01,0x00);
	OV13850_write_cmos_sensor(0x5a02,0x00);
	OV13850_write_cmos_sensor(0x5a03,0x00);
	OV13850_write_cmos_sensor(0x5a04,0x10);
	OV13850_write_cmos_sensor(0x5a05,0xa0);
	OV13850_write_cmos_sensor(0x5a06,0x0c);
	OV13850_write_cmos_sensor(0x5a07,0x78);
	OV13850_write_cmos_sensor(0x5a08,0x00);
	OV13850_write_cmos_sensor(0x5e00,0x00);
	OV13850_write_cmos_sensor(0x5e01,0x41);
	OV13850_write_cmos_sensor(0x5e11,0x30);
	OV13850_write_cmos_sensor(0x5000,0x06);
	OV13850_write_cmos_sensor(0x5001,0x01);
	OV13850_write_cmos_sensor(0x3400,0x04);
	OV13850_write_cmos_sensor(0x3401,0x00);
	OV13850_write_cmos_sensor(0x3402,0x04);
	OV13850_write_cmos_sensor(0x3403,0x00);
	OV13850_write_cmos_sensor(0x3404,0x04);
	OV13850_write_cmos_sensor(0x3405,0x00);
	OV13850_write_cmos_sensor(0x3406,0x01);
	OV13850_write_cmos_sensor(0x4005,0x18);
	OV13850_write_cmos_sensor(0x4009,0x10);
	OV13850_write_cmos_sensor(0x3503,0x07);
	OV13850_write_cmos_sensor(0x3500,0x00);
	OV13850_write_cmos_sensor(0x3501,0x10);
	OV13850_write_cmos_sensor(0x3502,0x40);
	OV13850_write_cmos_sensor(0x350b,0x80);
	OV13850_write_cmos_sensor(0x4800,0x14);
	OV13850_write_cmos_sensor(0x30b4,0x03);
	OV13850_write_cmos_sensor(0x30b3,0x54);
	OV13850_write_cmos_sensor(0x3106,0x21);
	OV13850_write_cmos_sensor(0x3090,0x02);
	OV13850_write_cmos_sensor(0x3091,0x1C);
	OV13850_write_cmos_sensor(0x3708,0xe6);
	OV13850_write_cmos_sensor(0x3709,0xc7);
	OV13850_write_cmos_sensor(0x3801,0x00);
	OV13850_write_cmos_sensor(0x3802,0x00);
	OV13850_write_cmos_sensor(0x3803,0x00);
	OV13850_write_cmos_sensor(0x3805,0x9f);
	OV13850_write_cmos_sensor(0x3806,0x0b);
	OV13850_write_cmos_sensor(0x3807,0xc7);
	OV13850_write_cmos_sensor(0x3808,0x08);
	OV13850_write_cmos_sensor(0x3809,0x40);
	OV13850_write_cmos_sensor(0x380a,0x05);
	OV13850_write_cmos_sensor(0x380b,0xdc);
	OV13850_write_cmos_sensor(0x380c,0x0b);
	OV13850_write_cmos_sensor(0x380d,0x00);
	OV13850_write_cmos_sensor(0x380e,0x07);
	OV13850_write_cmos_sensor(0x380f,0x9D);
	OV13850_write_cmos_sensor(0x3811,0x08);
	OV13850_write_cmos_sensor(0x3813,0x02);
	OV13850_write_cmos_sensor(0x3814,0x31);
	OV13850_write_cmos_sensor(0x3815,0x31);
	OV13850_write_cmos_sensor(0x3820,0x14);
	OV13850_write_cmos_sensor(0x3821,0x0f);
	OV13850_write_cmos_sensor(0x4004,0x02);
	OV13850_write_cmos_sensor(0x4837,0x0b);
	OV13850_write_cmos_sensor(0x5002,0x00);
	OV13850_write_cmos_sensor(0x0100,0x01);

#else
    int totalCnt = 0, len = 0;
	int transfer_len, transac_len=3;
	kal_uint8* pBuf=NULL;
	dma_addr_t dmaHandle;
	pBuf = (kal_uint8*)kmalloc(1024, GFP_KERNEL);
	

    totalCnt = ARRAY_SIZE(ov13850_init);
	transfer_len = totalCnt / transac_len;
	len = (transfer_len<<8)|transac_len;    
	OV13850DB("Total Count = %d, Len = 0x%x\n", totalCnt,len);    
	memcpy(pBuf, &ov13850_init, totalCnt );   
	dmaHandle = dma_map_single(NULL, pBuf, 1024, DMA_TO_DEVICE);	
	OV13850_multi_write_cmos_sensor(dmaHandle, len); 

	dma_unmap_single(NULL, dmaHandle, 1024, DMA_TO_DEVICE);

#ifdef OTP_CALIBRATION
	update_otp_wb();
	update_otp_lenc();
#endif
	kfree(pBuf);

	
#endif	

	
}   /*  OV13850_Sensor_Init  */

UINT32 OV13850Open(void)
{
	volatile signed int i;
	int  retry = 1;
	kal_uint16 sensor_id = 0;
	OV13850DB("OV13850Open enter :\n ");
	OV13850_WRITE_ID = OV13850MIPI_WRITE_ID_1;
	OV13850_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mDELAY(10);

    // check if sensor ID correct
    do {
        sensor_id = (OV13850_read_cmos_sensor(0x300A)<<8)|OV13850_read_cmos_sensor(0x300B);
        if (sensor_id == OV13850_SENSOR_ID)
        	{
        		OV13850DB("write id=%x, Sensor ID = 0x%04x\n", OV13850_WRITE_ID,sensor_id);
            	break;
        	}
        OV13850DB("Read Sensor ID Fail = 0x%04x\n", sensor_id);
        retry--;
    } while (retry > 0);

    if (sensor_id != OV13850_SENSOR_ID) {
		OV13850_WRITE_ID=OV13850MIPI_WRITE_ID;
		OV13850_write_cmos_sensor(0x0103,0x01);// Reset sensor
	    mDELAY(10);
        retry = 1;
	    // check if sensor ID correct
	    do {
	        sensor_id = (OV13850_read_cmos_sensor(0x300A)<<8)|OV13850_read_cmos_sensor(0x300B);
	        if (sensor_id == OV13850_SENSOR_ID)
	        	{
	        		OV13850DB("write id=%x,Sensor ID = 0x%04x\n",OV13850_WRITE_ID, sensor_id);
	            	break;
	        	}
	        OV13850DB("Read Sensor ID Fail = 0x%04x\n", sensor_id);
	        retry--;
	    } while (retry > 0);
		 if (sensor_id != OV13850_SENSOR_ID) 
		 {
           return ERROR_SENSOR_CONNECT_FAIL;
		 	}
    }
	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.sensorMode = SENSOR_MODE_INIT;
	ov13850.OV13850AutoFlickerMode = KAL_FALSE;
	ov13850.OV13850VideoMode = KAL_FALSE;
	spin_unlock(&ov13850mipiraw_drv_lock);
	OV13850_Sensor_Init();

	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.DummyLines= 0;
	ov13850.DummyPixels= 0;
	ov13850.shutter = 0x4EA;
	ov13850.pvShutter = 0x4EA;
	//ov13850.maxExposureLines =OV13850_PV_PERIOD_LINE_NUMS -4;
	ov13850.maxExposureLines =OV13850_PV_PERIOD_LINE_NUMS -16;
	ov13850.ispBaseGain = BASEGAIN;//0x40
	ov13850.sensorGlobalGain = 0x1f;//sensor gain read from 0x350a 0x350b; 0x1f as 3.875x
	ov13850.pvGain = 0x1f;
	ov13850.realGain = OV13850Reg2Gain(0x1f);//ispBaseGain as 1x
	spin_unlock(&ov13850mipiraw_drv_lock);

	OV13850DB("OV13850Open exit :\n ");

    return ERROR_NONE;
}

UINT32 OV13850GetSensorID(UINT32 *sensorID)
{
    int  retry = 1;

	printk("OV13850GetSensorID enter :\n ");
	OV13850_WRITE_ID = OV13850MIPI_WRITE_ID;
	OV13850_write_cmos_sensor(0x0103,0x01);// Reset sensor
    mDELAY(10);

    // check if sensor ID correct
    do {
        *sensorID = (OV13850_read_cmos_sensor(0x300A)<<8)|OV13850_read_cmos_sensor(0x300B);
        if (*sensorID == OV13850_SENSOR_ID)
        	{
        		printk("write id=%x, Sensor ID = 0x%04x\n", OV13850_WRITE_ID,*sensorID);
            	break;
        	}
        printk("Read Sensor ID Fail = 0x%04x\n", *sensorID);
        retry--;
    } while (retry > 0);

    if (*sensorID != OV13850_SENSOR_ID) {
		OV13850_WRITE_ID=OV13850MIPI_WRITE_ID_1;
		OV13850_write_cmos_sensor(0x0103,0x01);// Reset sensor
	    mDELAY(10);
        retry = 1;
	    // check if sensor ID correct
	    do {
	        *sensorID = (OV13850_read_cmos_sensor(0x300A)<<8)|OV13850_read_cmos_sensor(0x300B);
	        if (*sensorID == OV13850_SENSOR_ID)
	        	{
	        		printk("write id=%x,Sensor ID = 0x%04x\n",OV13850_WRITE_ID, *sensorID);
	            	break;
	        	}
	        printk("Read Sensor ID Fail = 0x%04x\n", *sensorID);
	        retry--;
	    } while (retry > 0);
		 if (*sensorID != OV13850_SENSOR_ID) 
		 {
		 
        *sensorID = 0xFFFFFFFF;
        return ERROR_SENSOR_CONNECT_FAIL;
		 	}
    }
    
    #ifndef ORIGINAL_VERSION
	   isNeedReadOtp = true;
    #endif
    
    return ERROR_NONE;
}


void OV13850_SetShutter(kal_uint32 iShutter)
{
	if(MSDK_SCENARIO_ID_CAMERA_ZSD == OV13850CurrentScenarioId )
	{
		//OV13850DB("always UPDATE SHUTTER when ov13850.sensorMode == SENSOR_MODE_CAPTURE\n");
	}
	else{
		if(ov13850.sensorMode == SENSOR_MODE_CAPTURE)
		{
			//OV13850DB("capture!!DONT UPDATE SHUTTER!!\n");
			//return;
		}
	}
	//if(ov13850.shutter == iShutter)
		//return;
   spin_lock(&ov13850mipiraw_drv_lock);
   ov13850.shutter= iShutter;
   spin_unlock(&ov13850mipiraw_drv_lock);
   OV13850_write_shutter(iShutter);
   return;
}   /*  OV13850_SetShutter   */

UINT32 OV13850_read_shutter(void)
{

	kal_uint16 temp_reg1, temp_reg2 ,temp_reg3;
	UINT32 shutter =0;
	temp_reg1 = OV13850_read_cmos_sensor(0x3500);    // AEC[b19~b16]
	temp_reg2 = OV13850_read_cmos_sensor(0x3501);    // AEC[b15~b8]
	temp_reg3 = OV13850_read_cmos_sensor(0x3502);    // AEC[b7~b0]
	shutter  = (temp_reg1 <<12)| (temp_reg2<<4)|(temp_reg3>>4);

	return shutter;
}

void OV13850_NightMode(kal_bool bEnable)
{}


UINT32 OV13850Close(void)
{    return ERROR_NONE;}

void OV13850SetFlipMirror(kal_int32 imgMirror)
{
	kal_int16 mirror=0,flip=0;
    flip = OV13850_read_cmos_sensor(0x3820);
	mirror   = OV13850_read_cmos_sensor(0x3821);
    switch (imgMirror)
    {
        case IMAGE_NORMAL://IMAGE_NORMAL:
            OV13850_write_cmos_sensor(0x3820, (flip & (0xFB)));//Set normal 0xBD--->0xbc xb.pang for capture size
            OV13850_write_cmos_sensor(0x3821, (mirror  & (0xFB)));	//Set normal  0xf9-->0xf8 xb.pang for capture size
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            OV13850_write_cmos_sensor(0x3820, (flip & (0xFB)));//Set normal  0xbd--->0xbc xb.pang for capture size
            OV13850_write_cmos_sensor(0x3821, (mirror  | (0x04)));	//Set mirror
            break;
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            OV13850_write_cmos_sensor(0x3820, (flip |(0x04)));	//Set flip
            OV13850_write_cmos_sensor(0x3821, (mirror  & (0xFB)));	//Set normal //0xf9-->0xf8 xb.pang for capture size
            break;
        case IMAGE_HV_MIRROR://IMAGE_HV_MIRROR:
            OV13850_write_cmos_sensor(0x3820, (flip |(0x4)));	//Set flip
            OV13850_write_cmos_sensor(0x3821, (mirror  |(0x04)));	//Set mirror
            break;
    }
}

UINT32 OV13850Preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{

	OV13850DB("OV13850Preview enter:");

	// preview size
	if(ov13850.sensorMode == SENSOR_MODE_PREVIEW)
	{
		// do nothing
		// FOR CCT PREVIEW
	}
	else
	{
		//OV13850DB("OV13850Preview setting!!\n");
		#ifdef ORIGINAL_VERSION		
		OV13850_Sensor_Init();
		#endif
		OV13850PreviewSetting();
		//mdelay(30);
	}
	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.sensorMode = SENSOR_MODE_PREVIEW; // Need set preview setting after capture mode
	ov13850.DummyPixels = 0;//define dummy pixels and lines
	ov13850.DummyLines = 0 ;
	OV13850_FeatureControl_PERIOD_PixelNum=OV13850_PV_PERIOD_PIXEL_NUMS+ ov13850.DummyPixels;
	OV13850_FeatureControl_PERIOD_LineNum=OV13850_PV_PERIOD_LINE_NUMS+ov13850.DummyLines;
	spin_unlock(&ov13850mipiraw_drv_lock);

	//OV13850_write_shutter(ov13850.shutter);
	//write_OV13850_gain(ov13850.pvGain);
	//set mirror & flip
	//OV13850DB("[OV13850Preview] mirror&flip: %d \n",sensor_config_data->SensorImageMirror);
	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov13850mipiraw_drv_lock);
	OV13850SetFlipMirror(OV13850_ORIENTATION);
	OV13850DB("OV13850Preview exit: \n");
    return ERROR_NONE;
}	/* OV13850Preview() */


UINT32 OV13850Video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	OV13850DB("OV13850Video enter:");
	if(ov13850.sensorMode == SENSOR_MODE_VIDEO)
	{
		// do nothing
	}
	else
		OV13850VideoSetting();
	
	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.sensorMode = SENSOR_MODE_VIDEO;
	OV13850_FeatureControl_PERIOD_PixelNum=OV13850_VIDEO_PERIOD_PIXEL_NUMS+ ov13850.DummyPixels;
	OV13850_FeatureControl_PERIOD_LineNum=OV13850_VIDEO_PERIOD_LINE_NUMS+ov13850.DummyLines;
	spin_unlock(&ov13850mipiraw_drv_lock);

	//OV13850_write_shutter(ov13850.shutter);
	//write_OV13850_gain(ov13850.pvGain);

	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.imgMirror = sensor_config_data->SensorImageMirror;
	spin_unlock(&ov13850mipiraw_drv_lock);
	OV13850SetFlipMirror(OV13850_ORIENTATION);

	OV13850DBSOFIA("[OV13850Video]frame_len=%x\n", ((OV13850_read_cmos_sensor(0x380e)<<8)+OV13850_read_cmos_sensor(0x380f)));

	OV13850DB("OV13850Video exit:\n");
    return ERROR_NONE;
}


UINT32 OV13850Capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                                                MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
 	kal_uint32 shutter = ov13850.shutter;
	kal_uint32 temp_data;
	if( SENSOR_MODE_CAPTURE== ov13850.sensorMode)
	{
		OV13850DB("OV13850Capture BusrtShot!!!\n");
	}else{
		OV13850DB("OV13850Capture enter:\n");
		//Record Preview shutter & gain
		shutter=OV13850_read_shutter();
		temp_data =  read_OV13850_gain();
		spin_lock(&ov13850mipiraw_drv_lock);
		ov13850.pvShutter =shutter;
		ov13850.sensorGlobalGain = temp_data;
		ov13850.pvGain =ov13850.sensorGlobalGain;
		spin_unlock(&ov13850mipiraw_drv_lock);

		OV13850DB("[OV13850Capture]ov13850.shutter=%d, read_pv_shutter=%d, read_pv_gain = 0x%x\n",ov13850.shutter, shutter,ov13850.sensorGlobalGain);

		// Full size setting
		OV13850CaptureSetting();
		spin_lock(&ov13850mipiraw_drv_lock);
		ov13850.sensorMode = SENSOR_MODE_CAPTURE;
		ov13850.imgMirror = sensor_config_data->SensorImageMirror;
		ov13850.DummyPixels = 0;//define dummy pixels and lines                                                                                                         
		ov13850.DummyLines = 0 ;    
		OV13850_FeatureControl_PERIOD_PixelNum = OV13850_FULL_PERIOD_PIXEL_NUMS + ov13850.DummyPixels;
		OV13850_FeatureControl_PERIOD_LineNum = OV13850_FULL_PERIOD_LINE_NUMS + ov13850.DummyLines;
		spin_unlock(&ov13850mipiraw_drv_lock);

		//OV13850DB("[OV13850Capture] mirror&flip: %d\n",sensor_config_data->SensorImageMirror);
		OV13850SetFlipMirror(OV13850_ORIENTATION);

	    if(OV13850CurrentScenarioId==MSDK_SCENARIO_ID_CAMERA_ZSD)
	    {
			OV13850DB("OV13850Capture exit ZSD!!\n");
			return ERROR_NONE;
	    }
		OV13850DB("OV13850Capture exit:\n");
	}

    return ERROR_NONE;
}	/* OV13850Capture() */

UINT32 OV13850GetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution)
{

    OV13850DB("OV13850GetResolution!!\n");
	pSensorResolution->SensorPreviewWidth	= OV13850_IMAGE_SENSOR_PV_WIDTH;
    pSensorResolution->SensorPreviewHeight	= OV13850_IMAGE_SENSOR_PV_HEIGHT;
    pSensorResolution->SensorFullWidth		= OV13850_IMAGE_SENSOR_FULL_WIDTH;
    pSensorResolution->SensorFullHeight		= OV13850_IMAGE_SENSOR_FULL_HEIGHT;
    pSensorResolution->SensorVideoWidth		= OV13850_IMAGE_SENSOR_VIDEO_WIDTH;
    pSensorResolution->SensorVideoHeight    = OV13850_IMAGE_SENSOR_VIDEO_HEIGHT;
    return ERROR_NONE;
}   /* OV13850GetResolution() */

UINT32 OV13850GetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId,
                                                MSDK_SENSOR_INFO_STRUCT *pSensorInfo,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{

	pSensorInfo->SensorPreviewResolutionX= OV13850_IMAGE_SENSOR_PV_WIDTH;
	pSensorInfo->SensorPreviewResolutionY= OV13850_IMAGE_SENSOR_PV_HEIGHT;
	pSensorInfo->SensorFullResolutionX= OV13850_IMAGE_SENSOR_FULL_WIDTH;
    pSensorInfo->SensorFullResolutionY= OV13850_IMAGE_SENSOR_FULL_HEIGHT;

	spin_lock(&ov13850mipiraw_drv_lock);
	ov13850.imgMirror = pSensorConfigData->SensorImageMirror ;
	spin_unlock(&ov13850mipiraw_drv_lock);

   	pSensorInfo->SensorOutputDataFormat= SENSOR_OUTPUT_FORMAT_RAW_B;
    pSensorInfo->SensorClockPolarity =SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorClockFallingPolarity=SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    pSensorInfo->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;

    pSensorInfo->SensroInterfaceType=SENSOR_INTERFACE_TYPE_MIPI;
    //pSensorInfo->MIPIsensorType = MIPI_OPHY_CSI2;
    pSensorInfo->CaptureDelayFrame = 2;
    pSensorInfo->PreviewDelayFrame = 2;
    pSensorInfo->VideoDelayFrame = 2;

    pSensorInfo->SensorDrivingCurrent = ISP_DRIVING_8MA;
    pSensorInfo->AEShutDelayFrame = 0;//0;		    /* The frame of setting shutter default 0 for TG int */
    pSensorInfo->AESensorGainDelayFrame = 0 ;//0;     /* The frame of setting sensor gain */
    pSensorInfo->AEISPGainDelayFrame = 2;

    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV13850_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850_VIDEO_X_START;
            pSensorInfo->SensorGrabStartY = OV13850_VIDEO_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850_FULL_X_START;	//2*OV13850_IMAGE_SENSOR_PV_STARTX;
            pSensorInfo->SensorGrabStartY = OV13850_FULL_Y_START;	//2*OV13850_IMAGE_SENSOR_PV_STARTY;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
            pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
        default:
			pSensorInfo->SensorClockFreq=24;
            pSensorInfo->SensorClockRisingCount= 0;

            pSensorInfo->SensorGrabStartX = OV13850_PV_X_START;
            pSensorInfo->SensorGrabStartY = OV13850_PV_Y_START;

            pSensorInfo->SensorMIPILaneNumber = SENSOR_MIPI_4_LANE;
            pSensorInfo->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
	     	pSensorInfo->MIPIDataLowPwr2HighSpeedSettleDelayCount = 23;
	    	pSensorInfo->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
            pSensorInfo->SensorPacketECCOrder = 1;
            break;
    }

    memcpy(pSensorConfigData, &OV13850SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));

    return ERROR_NONE;
}   /* OV13850GetInfo() */


UINT32 OV13850Control(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow,
                                                MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData)
{
		spin_lock(&ov13850mipiraw_drv_lock);
		OV13850CurrentScenarioId = ScenarioId;
		spin_unlock(&ov13850mipiraw_drv_lock);
		OV13850DB("OV13850CurrentScenarioId=%d\n",OV13850CurrentScenarioId);
    switch (ScenarioId)
    {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            OV13850Preview(pImageWindow, pSensorConfigData);
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			OV13850Video(pImageWindow, pSensorConfigData);
			break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
            OV13850Capture(pImageWindow, pSensorConfigData);
            break;
        default:
            return ERROR_INVALID_SCENARIO_ID;
    }
    return ERROR_NONE;
} /* OV13850Control() */


UINT32 OV13850SetVideoMode(UINT16 u2FrameRate)
{

    kal_uint32 MIN_Frame_length =0,frameRate=0,extralines=0;
    OV13850DB("[OV13850SetVideoMode] frame rate = %d\n", u2FrameRate);

	spin_lock(&ov13850mipiraw_drv_lock);
	 ov13850VIDEO_MODE_TARGET_FPS=u2FrameRate;
	spin_unlock(&ov13850mipiraw_drv_lock);

	if(u2FrameRate==0)
	{
		OV13850DB("Disable Video Mode or dynimac fps\n");
		return KAL_TRUE;
	}
	if(u2FrameRate >30 || u2FrameRate <5)
	    OV13850DB("error frame rate seting\n");

    if(ov13850.sensorMode == SENSOR_MODE_VIDEO)//video ScenarioId recording
    {
    	if(ov13850.OV13850AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==30)
				frameRate= 306;
			else if(u2FrameRate==15)
				frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (OV13850MIPI_VIDEO_CLK)/(OV13850_VIDEO_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (OV13850MIPI_VIDEO_CLK) /(OV13850_VIDEO_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=OV13850_VIDEO_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV13850_VIDEO_PERIOD_LINE_NUMS;
			OV13850DB("[OV13850SetVideoMode]current fps = %d\n", (OV13850MIPI_PREVIEW_CLK)  /(OV13850_PV_PERIOD_PIXEL_NUMS)/OV13850_PV_PERIOD_LINE_NUMS);
		}
		//OV13850DB("[OV13850SetVideoMode]current fps (10 base)= %d\n", (OV13850MIPI_PREVIEW_CLK)*10/(OV13850_PV_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/MIN_Frame_length);
	//	if(ov13850.shutter+4 > MIN_Frame_length) 
		//	MIN_Frame_length = ov13850.shutter + 4;
		
				if(ov13850.shutter+16 > MIN_Frame_length) 
			MIN_Frame_length = ov13850.shutter + 16;
		extralines = MIN_Frame_length - OV13850_VIDEO_PERIOD_LINE_NUMS;

		spin_lock(&ov13850mipiraw_drv_lock);
		ov13850.DummyPixels = 0;//define dummy pixels and lines
		ov13850.DummyLines = extralines ;
		spin_unlock(&ov13850mipiraw_drv_lock);
		
		OV13850_SetDummy(ov13850.DummyPixels,extralines);
    }
	else if(ov13850.sensorMode == SENSOR_MODE_CAPTURE)
	{
		OV13850DB("-------[OV13850SetVideoMode]ZSD???---------\n");
		if(ov13850.OV13850AutoFlickerMode == KAL_TRUE)
    	{
    		if (u2FrameRate==15)
			    frameRate= 148;
			else
				frameRate=u2FrameRate*10;

			MIN_Frame_length = (OV13850MIPI_CAPTURE_CLK) /(OV13850_FULL_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/frameRate*10;
    	}
		else
			MIN_Frame_length = (OV13850MIPI_CAPTURE_CLK) /(OV13850_FULL_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/u2FrameRate;

		if((MIN_Frame_length <=OV13850_FULL_PERIOD_LINE_NUMS))
		{
			MIN_Frame_length = OV13850_FULL_PERIOD_LINE_NUMS;
			//OV13850DB("[OV13850SetVideoMode]current fps = %d\n", (OV13850MIPI_CAPTURE_CLK) /(OV13850_FULL_PERIOD_PIXEL_NUMS)/OV13850_FULL_PERIOD_LINE_NUMS);

		}
		//OV13850DB("[OV13850SetVideoMode]current fps (10 base)= %d\n", (OV13850MIPI_CAPTURE_CLK)*10/(OV13850_FULL_PERIOD_PIXEL_NUMS + ov13850.DummyPixels)/MIN_Frame_length);
		if(ov13850.shutter+16 > MIN_Frame_length) 
			MIN_Frame_length = ov13850.shutter + 16;
			
			//		if(ov13850.shutter+4 > MIN_Frame_length) 
		//	MIN_Frame_length = ov13850.shutter + 4;
			
		extralines = MIN_Frame_length - OV13850_FULL_PERIOD_LINE_NUMS;

		spin_lock(&ov13850mipiraw_drv_lock);
		ov13850.DummyPixels = 0;//define dummy pixels and lines
		ov13850.DummyLines = extralines ;
		spin_unlock(&ov13850mipiraw_drv_lock);

		OV13850_SetDummy(ov13850.DummyPixels,extralines);
	}
	OV13850DB("[OV13850SetVideoMode]MIN_Frame_length=%d,ov13850.DummyLines=%d\n",MIN_Frame_length,ov13850.DummyLines);

    return KAL_TRUE;
}

UINT32 OV13850SetAutoFlickerMode(kal_bool bEnable, UINT16 u2FrameRate)
{
	//return ERROR_NONE;
    //OV13850DB("[OV13850SetAutoFlickerMode] frame rate(10base) = %d %d\n", bEnable, u2FrameRate);
	if(bEnable) {   // enable auto flicker
		spin_lock(&ov13850mipiraw_drv_lock);
		ov13850.OV13850AutoFlickerMode = KAL_TRUE;
		spin_unlock(&ov13850mipiraw_drv_lock);
    } else {
    	spin_lock(&ov13850mipiraw_drv_lock);
        ov13850.OV13850AutoFlickerMode = KAL_FALSE;
		spin_unlock(&ov13850mipiraw_drv_lock);
        OV13850DB("Disable Auto flicker\n");
    }

    return ERROR_NONE;
}

UINT32 OV13850SetTestPatternMode(kal_bool bEnable)
{
    OV13850DB("[OV13850SetTestPatternMode] Test pattern enable:%d\n", bEnable);
	if(bEnable)
		{
		   OV13850_write_cmos_sensor(0x5E00, 0x80);
		}
		else
		{
		
			OV13850_write_cmos_sensor(0x5E00, 0x00);
		}

    return ERROR_NONE;
}


UINT32 OV13850MIPISetMaxFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 frameRate) {
	kal_uint32 pclk;
	kal_int16 dummyLine;
	kal_uint16 lineLength,frameHeight;
		
	OV13850DB("OV13850MIPISetMaxFramerateByScenario: scenarioId = %d, frame rate = %d\n",scenarioId,frameRate);
	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
			pclk = OV13850MIPI_PREVIEW_CLK;
			lineLength = OV13850_PV_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV13850_PV_PERIOD_LINE_NUMS;
			ov13850.sensorMode = SENSOR_MODE_PREVIEW;
			OV13850_SetDummy(0, dummyLine);			
			break;			
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			pclk = OV13850MIPI_VIDEO_CLK; 
			lineLength = OV13850_VIDEO_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV13850_VIDEO_PERIOD_LINE_NUMS;
			ov13850.sensorMode = SENSOR_MODE_VIDEO;
			OV13850_SetDummy(0, dummyLine);			
			break;			
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:			
			pclk = OV13850MIPI_CAPTURE_CLK;
			lineLength = OV13850_FULL_PERIOD_PIXEL_NUMS;
			frameHeight = (10 * pclk)/frameRate/lineLength;
			dummyLine = frameHeight - OV13850_FULL_PERIOD_LINE_NUMS;
			ov13850.sensorMode = SENSOR_MODE_CAPTURE;
			OV13850_SetDummy(0, dummyLine);			
			break;		
        case MSDK_SCENARIO_ID_CAMERA_3D_PREVIEW: //added
            break;
        case MSDK_SCENARIO_ID_CAMERA_3D_VIDEO:
			break;
        case MSDK_SCENARIO_ID_CAMERA_3D_CAPTURE: //added   
			break;		
		default:
			break;
	}	
	return ERROR_NONE;
}


UINT32 OV13850MIPIGetDefaultFramerateByScenario(MSDK_SCENARIO_ID_ENUM scenarioId, MUINT32 *pframeRate) 
{

	switch (scenarioId) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			 *pframeRate = 300;
			 break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
		case MSDK_SCENARIO_ID_CAMERA_ZSD:
			// *pframeRate = 240;
			*pframeRate = 150;
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


UINT32 OV13850FeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId,
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
	OV13850DB(" OV13850FeatureControl is %d \n", FeatureId);

    switch (FeatureId)
    {
        case SENSOR_FEATURE_GET_RESOLUTION:
            *pFeatureReturnPara16++= OV13850_IMAGE_SENSOR_FULL_WIDTH;
            *pFeatureReturnPara16= OV13850_IMAGE_SENSOR_FULL_HEIGHT;
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_PERIOD:
				*pFeatureReturnPara16++= OV13850_FeatureControl_PERIOD_PixelNum;
				*pFeatureReturnPara16= OV13850_FeatureControl_PERIOD_LineNum;
				*pFeatureParaLen=4;
				break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
			switch(OV13850CurrentScenarioId)
			{
				case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
					*pFeatureReturnPara32 = OV13850MIPI_PREVIEW_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
					*pFeatureReturnPara32 = OV13850MIPI_VIDEO_CLK;
					*pFeatureParaLen=4;
					break;
				case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
				case MSDK_SCENARIO_ID_CAMERA_ZSD:
					*pFeatureReturnPara32 = OV13850MIPI_CAPTURE_CLK;
					*pFeatureParaLen=4;
					break;
				default:
					*pFeatureReturnPara32 = OV13850MIPI_CAPTURE_CLK;
					*pFeatureParaLen=4;
					break;
			}
		      break;

        case SENSOR_FEATURE_SET_ESHUTTER:
            OV13850_SetShutter(*pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            OV13850_NightMode((BOOL) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            OV13850_SetGain((UINT16) *pFeatureData16);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            //OV13850_isp_master_clock=*pFeatureData32;
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            OV13850_write_cmos_sensor(pSensorRegData->RegAddr, pSensorRegData->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            pSensorRegData->RegData = OV13850_read_cmos_sensor(pSensorRegData->RegAddr);
            break;
        case SENSOR_FEATURE_SET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov13850mipiraw_drv_lock);
                OV13850SensorCCT[i].Addr=*pFeatureData32++;
                OV13850SensorCCT[i].Para=*pFeatureData32++;
				spin_unlock(&ov13850mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_CCT_REGISTER:
            SensorRegNumber=FACTORY_END_ADDR;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV13850SensorCCT[i].Addr;
                *pFeatureData32++=OV13850SensorCCT[i].Para;
            }
            break;
        case SENSOR_FEATURE_SET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            for (i=0;i<SensorRegNumber;i++)
            {
            	spin_lock(&ov13850mipiraw_drv_lock);
                OV13850SensorReg[i].Addr=*pFeatureData32++;
                OV13850SensorReg[i].Para=*pFeatureData32++;
				spin_unlock(&ov13850mipiraw_drv_lock);
            }
            break;
        case SENSOR_FEATURE_GET_ENG_REGISTER:
            SensorRegNumber=ENGINEER_END;
            if (*pFeatureParaLen<(SensorRegNumber*sizeof(SENSOR_REG_STRUCT)+4))
                return FALSE;
            *pFeatureData32++=SensorRegNumber;
            for (i=0;i<SensorRegNumber;i++)
            {
                *pFeatureData32++=OV13850SensorReg[i].Addr;
                *pFeatureData32++=OV13850SensorReg[i].Para;
            }
            break;
        case SENSOR_FEATURE_GET_REGISTER_DEFAULT:
            if (*pFeatureParaLen>=sizeof(NVRAM_SENSOR_DATA_STRUCT))
            {
                pSensorDefaultData->Version=NVRAM_CAMERA_SENSOR_FILE_VERSION;
                pSensorDefaultData->SensorId=OV13850_SENSOR_ID;
                memcpy(pSensorDefaultData->SensorEngReg, OV13850SensorReg, sizeof(SENSOR_REG_STRUCT)*ENGINEER_END);
                memcpy(pSensorDefaultData->SensorCCTReg, OV13850SensorCCT, sizeof(SENSOR_REG_STRUCT)*FACTORY_END_ADDR);
            }
            else
                return FALSE;
            *pFeatureParaLen=sizeof(NVRAM_SENSOR_DATA_STRUCT);
            break;
        case SENSOR_FEATURE_GET_CONFIG_PARA:
            memcpy(pSensorConfigData, &OV13850SensorConfigData, sizeof(MSDK_SENSOR_CONFIG_STRUCT));
            *pFeatureParaLen=sizeof(MSDK_SENSOR_CONFIG_STRUCT);
            break;
        case SENSOR_FEATURE_CAMERA_PARA_TO_SENSOR:
            OV13850_camera_para_to_sensor();
            break;

        case SENSOR_FEATURE_SENSOR_TO_CAMERA_PARA:
            OV13850_sensor_to_camera_para();
            break;
        case SENSOR_FEATURE_GET_GROUP_COUNT:
            *pFeatureReturnPara32++=OV13850_get_sensor_group_count();
            *pFeatureParaLen=4;
            break;
        case SENSOR_FEATURE_GET_GROUP_INFO:
            OV13850_get_sensor_group_info(pSensorGroupInfo->GroupIdx, pSensorGroupInfo->GroupNamePtr, &pSensorGroupInfo->ItemCount);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_GROUP_INFO_STRUCT);
            break;
        case SENSOR_FEATURE_GET_ITEM_INFO:
            OV13850_get_sensor_item_info(pSensorItemInfo->GroupIdx,pSensorItemInfo->ItemIdx, pSensorItemInfo);
            *pFeatureParaLen=sizeof(MSDK_SENSOR_ITEM_INFO_STRUCT);
            break;

        case SENSOR_FEATURE_SET_ITEM_INFO:
            OV13850_set_sensor_item_info(pSensorItemInfo->GroupIdx, pSensorItemInfo->ItemIdx, pSensorItemInfo->ItemValue);
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
            OV13850SetVideoMode(*pFeatureData16);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            OV13850GetSensorID(pFeatureReturnPara32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            OV13850SetAutoFlickerMode((BOOL)*pFeatureData16, *(pFeatureData16+1));
	        break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            OV13850SetTestPatternMode((BOOL)*pFeatureData16);
            break;
		case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE:
			*pFeatureReturnPara32=OV13850_TEST_PATTERN_CHECKSUM;
			*pFeatureParaLen=4;
			break;
		case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
			OV13850MIPISetMaxFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, *(pFeatureData32+1));
			break;
		case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
			OV13850MIPIGetDefaultFramerateByScenario((MSDK_SCENARIO_ID_ENUM)*pFeatureData32, (MUINT32 *)(*(pFeatureData32+1)));
			break;
        default:
            break;
    }
    return ERROR_NONE;
}	/* OV13850FeatureControl() */


SENSOR_FUNCTION_STRUCT	SensorFuncOV13850=
{
    OV13850Open,
    OV13850GetInfo,
    OV13850GetResolution,
    OV13850FeatureControl,
    OV13850Control,
    OV13850Close
};

UINT32 OV13850_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
    /* To Do : Check Sensor status here */
    if (pfFunc!=NULL)
        *pfFunc=&SensorFuncOV13850;

    return ERROR_NONE;
}   /* SensorInit() */

