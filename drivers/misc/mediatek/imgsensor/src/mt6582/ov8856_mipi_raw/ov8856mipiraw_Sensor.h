/*******************************************************************************************/
  
  
/*******************************************************************************************/

/* SENSOR FULL SIZE */
#ifndef __OV8856_SENSOR_H
#define __OV8856_SENSOR_H   

typedef enum group_enum {
    PRE_GAIN=0,
    CMMCLK_CURRENT,
    FRAME_RATE_LIMITATION,
    REGISTER_EDITOR,
    GROUP_TOTAL_NUMS
} FACTORY_GROUP_ENUM;


#define ENGINEER_START_ADDR 10
#define FACTORY_START_ADDR 0

typedef enum engineer_index
{
    CMMCLK_CURRENT_INDEX=ENGINEER_START_ADDR,
    ENGINEER_END
} FACTORY_ENGINEER_INDEX;

typedef enum register_index
{
	SENSOR_BASEGAIN=FACTORY_START_ADDR,
	PRE_GAIN_R_INDEX,
	PRE_GAIN_Gr_INDEX,
	PRE_GAIN_Gb_INDEX,
	PRE_GAIN_B_INDEX,
	FACTORY_END_ADDR
} FACTORY_REGISTER_INDEX;

typedef struct
{
    SENSOR_REG_STRUCT	Reg[ENGINEER_END];
    SENSOR_REG_STRUCT	CCT[FACTORY_END_ADDR];
} SENSOR_DATA_STRUCT, *PSENSOR_DATA_STRUCT;

typedef enum {
    SENSOR_MODE_INIT = 0,
    SENSOR_MODE_PREVIEW,
    SENSOR_MODE_VIDEO,
    SENSOR_MODE_CAPTURE
} OV8856_SENSOR_MODE;


typedef struct
{
	kal_uint32 DummyPixels;
	kal_uint32 DummyLines;
	
	kal_uint32 pvShutter;
	kal_uint32 pvGain;
	
	kal_uint32 pvPclk;  
	kal_uint32 videoPclk;
	kal_uint32 capPclk;
	
	kal_uint32 shutter;

	kal_uint16 sensorGlobalGain;
	kal_uint16 ispBaseGain;
	kal_uint16 realGain;

	kal_int16 imgMirror;

	OV8856_SENSOR_MODE sensorMode;

	kal_bool OV8856AutoFlickerMode;
	kal_bool OV8856VideoMode;
    kal_uint16 write_id;   //yanggy add for I2C write     >>>Do not del this part when code merging,or I2C will not available
    kal_uint16 read_id;   //yanggy add for I2c read
	
}OV8856_PARA_STRUCT,*POV8856_PARA_STRUCT;


    #define OV8856_SHUTTER_MARGIN 			(4)
	#define OV8856_GAIN_BASE				(128)
	#define OV8856_AUTOFLICKER_OFFSET_30 	(296)
	#define OV8856_AUTOFLICKER_OFFSET_15 	(146)
	#define OV8856_PREVIEW_PCLK 			(144000000)
	#define OV8856_VIDEO_PCLK 				(144000000)
	#define OV8856_CAPTURE_PCLK 			(144000000)
	
	#define OV8856_MAX_FPS_PREVIEW			(300)
	#define OV8856_MAX_FPS_VIDEO			(300)
	#define OV8856_MAX_FPS_CAPTURE			(300)
	//#define OV8865_MAX_FPS_N3D				(300)


	//grab window
	#define OV8856_IMAGE_SENSOR_PV_WIDTH					(1632)    
	#define OV8856_IMAGE_SENSOR_PV_HEIGHT					(1224)	  
	#define OV8856_IMAGE_SENSOR_VIDEO_WIDTH 			(3264)  
	#define OV8856_IMAGE_SENSOR_VIDEO_HEIGHT			(1836)  
	#define OV8856_IMAGE_SENSOR_FULL_WIDTH				(3264)	
	#define OV8856_IMAGE_SENSOR_FULL_HEIGHT 			(2448-2)

	#define OV8856_FULL_X_START						    		(0)
	#define OV8856_FULL_Y_START						    		(0)
	#define OV8856_PV_X_START						    		(0)
	#define OV8856_PV_Y_START						    		(0)
	#define OV8856_VIDEO_X_START								(0)
	#define OV8856_VIDEO_Y_START								(0)

//TODO~
	#define OV8856_MAX_ANALOG_GAIN					(15)
	#define OV8856_MIN_ANALOG_GAIN					(1)


	/* SENSOR PIXEL/LINE NUMBERS IN ONE PERIOD */
	#define OV8856_PV_PERIOD_PIXEL_NUMS 				(1932)//(1928)   
	#define OV8856_PV_PERIOD_LINE_NUMS					(2482)  
                                                      
	#define OV8856_VIDEO_PERIOD_PIXEL_NUMS			(1932)//(1928)
	#define OV8856_VIDEO_PERIOD_LINE_NUMS				(2482)  
                                                      
	#define OV8856_FULL_PERIOD_PIXEL_NUMS				(1932)  
	#define OV8856_FULL_PERIOD_LINE_NUMS				(2482)//(2474)  
	
	#define OV8856MIPI_WRITE_ID 	(0x20)
	#define OV8856MIPI_READ_ID	(0x21)
	
    #define OV8856MIPI_WRITE_ID1 	(0x6C)
	#define OV8856MIPI_READ_ID1	(0x6D)

    #define OV8856MIPI_WRITE_ID2 	(0x42)
	#define OV8856MIPI_READ_ID2	(0x43)
	#define OV8856MIPI_SENSOR_ID            OV8856_SENSOR_ID


	UINT32 OV8856MIPIOpen(void);
	UINT32 OV8856MIPIGetResolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *pSensorResolution);
	UINT32 OV8856MIPIGetInfo(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_INFO_STRUCT *pSensorInfo, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	UINT32 OV8856MIPIControl(MSDK_SCENARIO_ID_ENUM ScenarioId, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *pImageWindow, MSDK_SENSOR_CONFIG_STRUCT *pSensorConfigData);
	UINT32 OV8856MIPIFeatureControl(MSDK_SENSOR_FEATURE_ENUM FeatureId, UINT8 *pFeaturePara,UINT32 *pFeatureParaLen);
	UINT32 OV8856MIPIClose(void);
    void OV8856PreviewSetting(void);

#endif 

