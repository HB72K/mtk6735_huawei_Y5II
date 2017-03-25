
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif
#include "cust_gpio_usage.h"

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE; ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  				(720)
#define FRAME_HEIGHT 				(1280)

#define REGFLAG_DELAY             		0xFE
#define REGFLAG_END_OF_TABLE      		0xFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE			0

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					        lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)							lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)  


/*  -------------------- start ic 65132 ------------------------ */
#ifdef BUILD_LK
//i2c hardware type -start
#define tps6132_I2C_ID	I2C1
static struct mt_i2c_t tps6132_i2c;
#define tps6132_I2C_ADDR       0x3E 

 U32 tps6132_reg_i2c_read (U8 addr, U8 *dataBuffer)
 {
    kal_uint32 ret_code = I2C_OK;
    kal_uint16 len;
    *dataBuffer = addr;

    tps6132_i2c.id = tps6132_I2C_ID;
    tps6132_i2c.addr = (tps6132_I2C_ADDR);
    tps6132_i2c.mode = ST_MODE;
    tps6132_i2c.speed = 100;
    len = 1;

    ret_code = i2c_write_read(&tps6132_i2c, dataBuffer, len,len);
   
	 return ret_code;
 }
 

 U32 tps6132_reg_i2c_write(U8 addr, U8 value)
 {
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    tps6132_i2c.id = tps6132_I2C_ID;
    tps6132_i2c.addr = (tps6132_I2C_ADDR);
    tps6132_i2c.mode = ST_MODE;
    tps6132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&tps6132_i2c, write_data, len);
     
	 return ret_code;
 }
#endif
/*  -------------------- end ic 65132 ------------------------ */

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_init(void)
{
	    unsigned int data_array[35];



     SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
      MDELAY(40);

   #ifndef BUILD_LK
    data_array[0] = 0x00043902;                          
    data_array[1] = 0x9483FFB9;                 
    dsi_set_cmdq(&data_array, 2, 1); 
    //MDELAY(10);

    		
     data_array[0] = 0x00113902;//             
     data_array[1] = 0x04007CB1;   // b0
     data_array[2] = 0xEE0E030F; 
     data_array[3] = 0x19192921; 
     data_array[4] = 0xE6001257; 
     data_array[5] = 0x000000E2; 
     dsi_set_cmdq(&data_array, 6, 1);
    MDELAY(10);	


    lcm_util.set_gpio_out(GPIO_LCD_ENN, GPIO_OUT_ONE);
    MDELAY(7);
    lcm_util.set_gpio_out(GPIO_LCD_ENP, GPIO_OUT_ONE);
    MDELAY(10);

#endif





/*
{0x39, 0, 0, 0, 0, 4,{0xB9,0xFF,0x83,0x94}},//SET PASSWORD

{0x39, 0, 0, 0, 0, 2,{0xBA,0x13}},//SET MIPI 4 LANE
	
{0x39, 0, 0, 0, 0, 17,{0xB1,0x01,0x00,0x07,0x87,0x01,0x11,0x11,0x2A,0x30,0x3F,0x3F,0x47,0x12,0x01,0xE6,0xE2}},//SET POWER
//{0x39, 0, 0, 0, 0, 17,{0xB1,0x01,0x00,0x04,0x86,0x01,0x11,0x11,0x2F,0x37,0x3F,0x3F,0x47,0x12,0x01,0xE6,0xE2}},

{0x39, 0, 0, 0, 0, 23,{0xB4,0x80,0x06,0x32,0x10,0x03,0x32,0x15,0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,0x37,0x04,0x3F,0x06,0x61,0x61,0x06}},//SET CYC
//{0x39, 0, 0, 0, 0, 23,{0xB4,0x80,0x06,0x32,0x10,0x03,0x32,0x15,0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,0x37,0x04,0x43,0x06,0x61,0x61,0x06}},//SET CYC

{0x39, 0, 0, 0, 0, 7,{0xB2,0x00,0xC8,0x08,0x04,0x00,0x22}},//SET DISPLAY RELATED REGISTER

{0x39, 0, 0, 0, 0, 33,{0xD5,0x00,0x00,0x00,0x00,0x0A,0x00,0x01,0x00,0xCC,0x00,0x00,0x00,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,0x67,0x45,0x23,0x01,0x23,0x88,0x88,0x88,0x88}},//SET GIP

{0x39, 0, 0, 0, 0, 5,{0xC7,0x00,0x10,0x00,0x10}},//SET TCON
{0x39, 0, 0, 0, 0, 5,{0xBF,0x06,0x00,0x10,0x04}},// Row line issue(reduce charge pumb ripple)

{0x39, 0, 0, 0, 0, 2,{0xCC,0x09}},// SET PANEL

{0x39, 0, 0, 0, 0, 43,{0xE0,0x00,0x04,0x06,0x2B,0x33,0x3F,0x11,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,0x10,0x17,0x00,0x04,0x06,0x2B,0x33,0x3F,0x11,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,0x10,0x17,0x0B,0x17,0x07,0x11,0x0B,0x17,0x07,0x11}},//SET GAMMA
       
{0x39, 0, 0, 0, 0, 128,{0xC1,0x01,0x00,0x07,0x0E,0x15,0x1D,0x25,0x2D,0x34,0x3C,0x42,0x49,0x51,0x58,0x5F,0x67,0x6F,0x77,0x80,0x87,0x8F,0x98,0x9F,0xA7,0xAF,0xB7,0xC1,0xCB,0xD3,0xDD,0xE6,0xEF,0xF6,0xFF,0x16,0x25,0x7C,0x62,0xCA,0x3A,0xC2,0x1F,0xC0,
		               0x00,0x07,0x0E,0x15,0x1D,0x25,0x2D,0x34,0x3C,0x42,0x49,0x51,0x58,0x5F,0x67,0x6F,0x77,0x80,0x87,0x8F,0x98,0x9F,0xA7,0xAF,0xB7,0xC1,0xCB,0xD3,0xDD,0xE6,0xEF,0xF6,0xFF,0x16,0x25,0x7C,0x62,0xCA,0x3A,0xC2,0x1F,0xC0,
		               0x00,0x07,0x0E,0x15,0x1D,0x25,0x2D,0x34,0x3C,0x42,0x49,0x51,0x58,0x5F,0x67,0x6F,0x77,0x80,0x87,0x8F,0x98,0x9F,0xA7,0xAF,0xB7,0xC1,0xCB,0xD3,0xDD,0xE6,0xEF,0xF6,0xFF,0x16,0x25,0x7C,0x62,0xCA,0x3A,0xC2,0x1F,0xC0}},//SET GAMMA 

{0x39, 0, 0, 0, 0, 2,{0xB6,0x0C}},// VCOM
{0x39, 0, 0, 0, 0, 2,{0xD4,0x32}},
	


{0x05, 0, 0, 1, 150,1 , {0x11}},
{0x05, 0, 0, 1, 40, 1 , {0x29}},
*/  
	
	//{0x11,	0,	{}},     //sleep out
	//{REGFLAG_DELAY, 150, {}},

    data_array[0] = 0x00043902;                          
    data_array[1] = 0x9483FFB9;                 
    dsi_set_cmdq(&data_array, 2, 1); 
    //MDELAY(10);

    data_array[0] = 0x00023902;                          
    data_array[1] = 0x000013BA;     
	//data_array[2] = 0x1010C516;   
	//data_array[3] = 0x03240FFF; 
	//data_array[4] = 0x20252421; 
	//data_array[5] = 0x00000008; 
    dsi_set_cmdq(&data_array, 2, 1); 
    //MDELAY(10);	

//{0xB1,0x01,0x00,0x07,0x87,0x01,0x11,0x11,
//0x2A,0x30,0x3F,0x3F,0x47,0x12,0x01,0xE6,0xE2}
      data_array[0] = 0x00113902;//             
     data_array[1] = 0x070001B1;   // b0
     data_array[2] = 0x11110186; 
     data_array[3] = 0x3F3F332B; 
     data_array[4] = 0xE6011247; 
     data_array[5] = 0x000000E2; 
     dsi_set_cmdq(&data_array, 6, 1);
    //MDELAY(10);	
//{0xB4,0x80,0x06,0x32,0x10,0x03,0x32,0x15,
//0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,
//0x37,0x04,0x3F,0x06,0x61,0x61,0x06} 

     


//{0xB2,0x00,0xC8,0x08,0x04,0x00,0x22}    
    data_array[0] = 0x000D3902;//                          
    data_array[1] = 0x08C800B2; //f0  
    data_array[2] = 0x00110004;    //0x00220004    
    data_array[3] = 0x050000ff;    //0x00220004  	
	data_array[4] = 0x00000011;    //0x00220004  
    dsi_set_cmdq(&data_array, 5, 1);

     data_array[0] = 0x00173902;//             
     data_array[1] = 0x320680B4;   // b0
     data_array[2] = 0x15320410; 
     data_array[3] = 0x08103208; 
     data_array[4] = 0x054F0133; 
     data_array[5] = 0x064F0137; 
     data_array[6] = 0x00066161; 
     dsi_set_cmdq(&data_array, 7, 1);

//{0x39, 0, 0, 0, 0, 33,{0xD5,0x00,0x00,0x00,0x00,0x0A,0x00,0x01,
//0x00,0xCC,0x00,0x00,0x00,0x88,0x88,0x88,
//0x88,0x88,0x88,0x88,0x88,0x88,0x88,0x01,
//0x67,0x45,0x23,0x01,0x23,0x88,0x88,0x88,0x88}},//////SET GIP

     data_array[0] = 0x00213902;//             
     data_array[1] = 0x000000D5;   // d5
     data_array[2] = 0x01000A00; 
     data_array[3] = 0x0000CC00; 
     data_array[4] = 0x88888800; 
     data_array[5] = 0x88888888; 
     data_array[6] = 0x01888888; 
     data_array[7] = 0x01234567; 
     data_array[8] = 0x88888823; 
     data_array[9] = 0x00000088;
     dsi_set_cmdq(&data_array,10, 1);

//{0xC7,0x00,0x10,0x00,0x10}
    data_array[0] = 0x00053902;//                          
    data_array[1] = 0x001000C7; //f0
    data_array[2] = 0x00000010;                 
            dsi_set_cmdq(&data_array, 3, 1);
        

//{0xBF,0x06,0x00,0x10,0x04}}
    data_array[0] = 0x00053902;//                          
    data_array[1] = 0x100206BF; //f0
    data_array[2] = 0x00000004;                 
    dsi_set_cmdq(&data_array, 3, 1);



     data_array[0] = 0x09CC1500;//              
     dsi_set_cmdq(&data_array, 1, 1);

//{0xE0,0x00,0x04,0x06,0x2B,0x33,0x3F,0x11,
//0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,0x13,
//0x10,0x17,0x00,0x04,0x06,0x2B,0x33,0x3F,
//0x11,0x34,0x0A,0x0E,0x0D,0x11,0x13,0x11,
//0x13,0x10,0x17,0x0B,0x17,0x07,0x11,0x0B,
//0x17,0x07,0x11}},//SET GAMMA

     data_array[0] = 0x002b3902;//             
     data_array[1] = 0x020000E0;   // E0
     data_array[2] = 0x0C3F332B; 
     data_array[3] = 0x0C0C0733; 
     data_array[4] = 0x12101210; 
     data_array[5] = 0x00001710; 
     data_array[6] = 0x3F332B02; 
     data_array[7] = 0x0C07330C; 
     data_array[8] = 0x1012100C; 
     data_array[9] = 0x0A171012;
     data_array[10] = 0x0A110716; 
     data_array[11] = 0x00110716;
     dsi_set_cmdq(&data_array,12, 1);

#if 1
     data_array[0] = 0x00213902;//             
     data_array[1] = 0x070001C1;   // E0
     data_array[2] = 0x251D150E; 
     data_array[3] = 0x423C342D; 
     data_array[4] = 0x5F585149; 
     data_array[5] = 0x80776F67; 
     data_array[6] = 0x9F988F87; 
     data_array[7] = 0xC1B7AFA7; 
     data_array[8] = 0xE6DDD3CB;//
     data_array[9] = 0x000000EF;//
    dsi_set_cmdq(&data_array,10, 1);
 
     data_array[0] = 0x00212902;//             
     data_array[1] = 0x16FFF6C1;
     data_array[2] = 0xCA627C25; 
     data_array[3] = 0xC01FC23A;
     data_array[4] = 0x150E0700;    
     data_array[5] = 0x342D251D; 
     data_array[6] = 0x5149423C; 
     data_array[7] = 0x6F675F58; 
     data_array[8] = 0x8F878077; //
     data_array[9] = 0x00000098;//
    dsi_set_cmdq(&data_array,10, 1);

     data_array[0] = 0x00212902;//             
     data_array[1] = 0xAFA79FC1; 
     data_array[2] = 0xD3CBC1B7; 
     data_array[3] = 0xF6EFE6DD;
     data_array[4] = 0x7C2516FF; 
     data_array[5] = 0xC23ACA62;
     data_array[6] = 0x0700C01F; 
     data_array[7] = 0x251D150E; 
     data_array[8] = 0x423C342D; //
     data_array[9] = 0x00000049;//
    dsi_set_cmdq(&data_array,10, 1);

     data_array[0] = 0x00202902;//             
     data_array[1] = 0x5F5851C1; 
     data_array[2] = 0x80776F67; 
     data_array[3] = 0x9F988F87; 
     data_array[4] = 0xC1B7AFA7; 
     data_array[5] = 0xE6DDD3CB;
     data_array[6] = 0x16FFF6EF; 
     data_array[7] = 0xCA627C25;
     data_array[8] = 0xC01FC23A;
    dsi_set_cmdq(&data_array,9, 1);
#endif
     data_array[0] = 0x00B61500;//              
            dsi_set_cmdq(&data_array, 1, 1);

     data_array[0] = 0x32D41500;//             
            dsi_set_cmdq(&data_array, 1, 1);
    	

     data_array[0] = 0x00351500;//             
     dsi_set_cmdq(&data_array, 1, 1);		
	//MDELAY(25);	

     data_array[0] = 0x00110500; 			   
     dsi_set_cmdq(&data_array, 1, 1); 
     MDELAY(200);
	
     data_array[0] = 0x00290500; 			   
      dsi_set_cmdq(&data_array, 1, 1);
        MDELAY(10);
 
	// Note
	// Strongly recommend not to set Sleep out / Display On here. That will cause messed frame to be shown as later the backlight is on.


};



static void lcm_get_params(LCM_PARAMS *params)
{
   #ifdef BUILD_LK
	printf("###lcm_get_params \n");
   #else
	
	printk("###lcm_get_params \n");
   #endif
		memset(params, 0, sizeof(LCM_PARAMS));
	
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		// enable tearing-free
		//params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	        params->dbi.te_mode 			= LCM_DBI_TE_MODE_DISABLED;
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

            
#if (LCM_DSI_CMD_MODE)
		params->dsi.mode   = CMD_MODE;
#else
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;
                // params->dsi.mode   = BURST_VDO_MODE;
		//params->dsi.mode   = SYNC_EVENT_VDO_MODE; 
#endif
	
		// DSI
		/* Command mode setting */
		params->dsi.LANE_NUM		    = LCM_FOUR_LANE;
		//The following defined the fomat for data coming from LCD engine.
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
		params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
		params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
		params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;


		// Highly depends on LCD driver capability.
		params->dsi.packet_size=256;

		// Video mode setting		
		params->dsi.intermediat_buffer_num = 0;

		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;	
		params->dsi.word_count=720*3;	
	        params->dsi.vertical_sync_active				= 2;  //---3
		params->dsi.vertical_backporch					= 8; //---14
		params->dsi.vertical_frontporch					= 6;  //----8
		params->dsi.vertical_active_line				= FRAME_HEIGHT; 

		params->dsi.horizontal_sync_active				= 40;  //----2
		params->dsi.horizontal_backporch				= 86; //----28
		params->dsi.horizontal_frontporch				= 86; //----50
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;
	
	        //params->dsi.HS_PRPR=3;
		//params->dsi.CLK_TRAIL = 5;
		params->dsi.CLK_HS_POST=22;
		params->dsi.DA_HS_EXIT=20;
    
     		params->dsi.PLL_CLOCK=217;
                params->dsi.ssc_range=8;
                params->dsi.ssc_disable=0;


		// Bit rate calculation
		//1 Every lane speed
		//params->dsi.pll_div1=0;		// div1=0,1,2,3;div1_real=1,2,4,4 ----0: 546Mbps  1:273Mbps
		//params->dsi.pll_div2=1;		// div2=0,1,2,3;div1_real=1,2,4,4	
		//params->dsi.fbk_div =17;    // fref=26MHz, fvco=fref*(fbk_div+1)*2/(div1_real*div2_real)	
}

static void lcm_suspend(void)
{

    #ifdef BUILD_LK
	  printf("[LK]--lcm_suspend---%s------\n",__func__);
    #else
	  printk("[KERNEL]---lcm_suspend---%s------\n",__func__);
    #endif
	
	unsigned int data_array[35];

	
     //   data_array[0]=0x00280500; // Display Off
	//dsi_set_cmdq(data_array, 1, 1);
	//MDELAY(30);
	data_array[0] = 0x00100500; // Sleep In
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);

	    SET_RESET_PIN(1);
	   MDELAY(20);
	    SET_RESET_PIN(0);
	    MDELAY(20);
	    SET_RESET_PIN(1);
	   MDELAY(120);

	//    SET_RESET_PIN(0);
    //disable VSP & VSN
	lcm_util.set_gpio_out(GPIO_LCD_ENN, GPIO_OUT_ZERO);
	MDELAY(10);

	lcm_util.set_gpio_out(GPIO_LCD_ENP, GPIO_OUT_ZERO);
    MDELAY(10);	
   
}


static void lcm_resume(void)
{

    #ifdef BUILD_LK
	  printf("[LK]--lcm_resume---%s------\n",__func__);
    #else
	  printk("[KERNEL]---lcm_resume--%s------\n",__func__);
    #endif
	  lcm_init();
  
}






static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	data_array[3]= 0x00053902;
	data_array[4]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[5]= (y1_LSB);
	data_array[6]= 0x002c3909;

	dsi_set_cmdq(&data_array, 7, 0);

}

static unsigned int lcm_compare_id(void)
{


		int   array[4];
		char  buffer[3];
		char  id0=0;
		char  id1=0;
		char  id2=0;
                int  id=0;

		char read_data=0;
		
	    unsigned int data_array[35];

     SET_RESET_PIN(1);
    MDELAY(20);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(50);

#if 1
    data_array[0] = 0x00043902;                          
    data_array[1] = 0x9483FFB9;                 
    dsi_set_cmdq(&data_array, 2, 1); 
    MDELAY(10);

     data_array[0] = 0x00113902;//             
     data_array[1] = 0x04007CB1;   // b0
     data_array[2] = 0xEE0E030F; 
     data_array[3] = 0x19192921; 
     data_array[4] = 0xE6001257; 
     data_array[5] = 0x000000E2; 
     dsi_set_cmdq(&data_array, 6, 1);
    MDELAY(10);	
#endif
  

    lcm_util.set_gpio_out(GPIO_LCD_ENN, GPIO_OUT_ONE); 
    MDELAY(20);//2ms
    lcm_util.set_gpio_out(GPIO_LCD_ENP, GPIO_OUT_ONE);
    MDELAY(50);


	#ifdef BUILD_LK

	//tps6132_reg_i2c_write(0x02, 0x00);
	tps6132_reg_i2c_read(0x03, &read_data);
	printf("read_data = %04x\n",read_data);

	if(read_data & 0x30) //NOVATEK
	{
		tps6132_reg_i2c_write(0x00, 0x0A);
		tps6132_reg_i2c_write(0x01, 0x0A);
	   	tps6132_reg_i2c_write(0x03, 0x30);//VSP/VSN FLOATING
	   	tps6132_reg_i2c_write(0xFF, 0x80);
		
		printf("read_data,NOVATEK power IC \n");
	}
	else  //TI
	{
		tps6132_reg_i2c_write(0x00, 0x0A);
		tps6132_reg_i2c_write(0x01, 0x0A);
   		tps6132_reg_i2c_write(0x03, 0x40);//VSP/VSN FLOATING //0x03--- 0x40 ---tablet
	   	tps6132_reg_i2c_write(0xFF, 0x80);	
		
		printf("read_data,TI power IC \n");
	}
	
	#endif


        array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xda, buffer, 1);
        id0 = buffer[0]; //should be 0x83

   
        array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xdb, buffer, 1);
        id1 = buffer[0]; //should be 0x94

        array[0] = 0x00013700;// read id return two byte,version and id
	dsi_set_cmdq(array, 1, 1);
	
	read_reg_v2(0xdc, buffer, 1);
        id2 = buffer[0]; //should be 0x1A

        id = (id0<<16)| (id1<<8) | id2; //we only need ID

	#ifdef BUILD_LK
	printf("%s, LK hx8394a id0 = 0x%08x\n", __func__, id0);
	printf("%s, LK hx8394a id1 = 0x%08x\n", __func__, id1);
	printf("%s, LK hx8394a id2 = 0x%08x\n", __func__, id2);
	printf("%s, lk hx8394a id = 0x%08x\n", __func__, id);
	#else
	printk("%s, Kernel hx8394a id0 = 0x%08x\n", __func__, id0);
	printk("%s, Kernel hx8394a id1 = 0x%08x\n", __func__, id1);
	printk("%s, Kernel hx8394a id2 = 0x%08x\n", __func__, id2);
	printk("%s, Kernel hx8394a id = 0x%08x\n", __func__, id);
	#endif

   	if (id == 0X83941A)//8394
		return 1;
  	else
   		return 0;

}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK 
        unsigned int id = 0; 
        unsigned int id1 = 0; 
        unsigned char buffer[6]; 
        unsigned int data_array[16]; 

        if(lcm_esd_test) 
        { 
                lcm_esd_test = FALSE; 
                return TRUE; 
        } 

        data_array[0] = 0x00023700; 
        dsi_set_cmdq(data_array, 1, 1); 
        read_reg_v2(0x0a, buffer, 2); 
        id = buffer[0]; 
        printk("lx[%s] esd check: id = %x .\n", __FUNCTION__, id); 

        if(id == 0x1c) 
        { 
                return FALSE; 
        } 
        else 
        {             
                return TRUE; 
        } 
#endif
}

static unsigned int lcm_esd_recover(void)
{
    lcm_init();

#ifdef BUILD_LK 
    printf("[lx-lk] %s, \n", __func__); 
#else 
    printk("[lx- k] %s, \n", __func__); 
#endif
    return TRUE;
}

LCM_DRIVER hx8394a_hd720_dsi_vdo_xinli_lcm_drv = 
{
        .name		= "hx8394a_hd720_dsi_vdo_xinli",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
        .compare_id     = lcm_compare_id,
        .esd_check      = lcm_esd_check,
        .esd_recover    = lcm_esd_recover,
#if (LCM_DSI_CMD_MODE)
        // .set_backlight = lcm_setbacklight,
	//.set_pwm        = lcm_setpwm,
	//.get_pwm        = lcm_getpwm,
        .update         = lcm_update
#endif
    };
