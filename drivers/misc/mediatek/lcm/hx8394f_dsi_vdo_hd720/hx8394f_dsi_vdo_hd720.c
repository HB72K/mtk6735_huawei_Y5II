#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
//#include <platform/mt_gpio.h>
//#include <platform/mt_pmic.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include <mach/mt_gpio.h>
#endif

#include <cust_gpio_usage.h>

#ifdef BUILD_LK
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h> 
#include <platform/mt_pmic.h>

#define TPS65132_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t TPS65132_i2c;
//#define GPIO_LCD_ENN GPIO_LCD_ENN //sophiarui
//#define GPIO_LCD_ENP GPIO_LCD_ENP //sophiarui


static int TPS65132_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    TPS65132_i2c.id = 1;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}
#endif
#if 0
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
//#include <cust_i2c.h>


/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  1//I2C_I2C_LCD_BIAS_CHANNEL//for I2C channel 0//sophia
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

struct tps65132_dev	{	
    struct i2c_client	*client;
};

static const struct i2c_device_id tps65132_id[] = {
    { I2C_ID_NAME, 1 },
    { }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver tps65132_iic_driver = {
    .id_table	= tps65132_id,
    .probe		= tps65132_probe,
    .remove		= tps65132_remove,
    //.detect		= mt6605_detect,
    .driver		= {
        .owner	= THIS_MODULE,
        .name	= "tps65132",
    },
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 



/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
    printk( "tps65132_iic_probe\n");
    printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
    tps65132_i2c_client  = client;		
    return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
    printk( "tps65132_remove\n");
    tps65132_i2c_client = NULL;
    i2c_unregister_device(client);
    return 0;
}


static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
    int ret = 0;
    struct i2c_client *client = tps65132_i2c_client;
    char write_data[2]={0};	
    write_data[0]= addr;
    write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
    if(ret<0)
        printk("tps65132 write data fail !!\n");	
    return ret ;
}
EXPORT_SYMBOL_GPL(tps65132_write_bytes);



/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{
    printk( "tps65132_iic_init\n");
    i2c_register_board_info(1, &tps65132_board_info, 1);
    printk( "tps65132_iic_init2\n");
    i2c_add_driver(&tps65132_iic_driver);
    printk( "tps65132_iic_init success\n");	
    return 0;
}

static void __exit tps65132_iic_exit(void)
{
    printk( "tps65132_iic_exit\n");
    i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif



// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define GPIO_65132_ENP GPIO_LCD_ENP //sophiarui
#define GPIO_65132_ENN GPIO_LCD_ENN //sophiarui


#define REGFLAG_DELAY             							0XFEFF
#define REGFLAG_END_OF_TABLE      							0xFFFF   // END OF REGISTERS MARKER

#define LCM_DSI_CMD_MODE									0

#define LCM_ID1 	0x94
#define LCM_ID2 	0x94

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test


// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define SET_GPIO_OUT(gpio_num,val)    						(lcm_util.set_gpio_out((gpio_num),(val)))


#define UDELAY(n) 											(lcm_util.udelay(n))
#define MDELAY(n) 											(lcm_util.mdelay(n))


#define _LCM_DEBUG_

#ifdef BUILD_LK
#define printk printf
#endif

#ifdef _LCM_DEBUG_
#define lcm_debug(fmt, args...) printk(fmt, ##args)
#else
#define lcm_debug(fmt, args...) do { } while (0)
#endif

#ifdef _LCM_INFO_
#define lcm_info(fmt, args...) printk(fmt, ##args)
#else
#define lcm_info(fmt, args...) do { } while (0)
#endif
#define lcm_err(fmt, args...) printk(fmt, ##args)

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)    

static struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static void init_lcm_registers(void)
{
	unsigned int data_array[16];

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);

//#ifndef BUILD_LK
	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(&data_array, 2, 1);
	//MDELAY(10);

	data_array[0] = 0x00073902;//             
	data_array[1] = 0x680363BA;   // b0
	data_array[2] = 0x00C0B26B; 
	dsi_set_cmdq(&data_array, 3, 1);
	//MDELAY(10); 

	data_array[0] = 0x000B3902;//             
	data_array[1] = 0x721250B1;   // b1
	data_array[2] = 0x71543309; 
	data_array[3] = 0x002F4D31; 
	dsi_set_cmdq(&data_array, 4, 1);

/*
	lcm_util.set_gpio_out(GPIO_LCM_ENN, GPIO_OUT_ONE);//   -5.0V
	MDELAY(7);
	lcm_util.set_gpio_out(GPIO_LCM_ENP, GPIO_OUT_ONE);//    5.0V
	MDELAY(10);
*/

	//tps6132_reg_i2c_write(0x03, 0x40);//VSP/VSN FLOATING //0x03--- 0x40 ---tablet
//#endif

	//MDELAY(10); 
	//{0xB4,0x80,0x06,0x32,0x10,0x03,0x32,0x15,
	//0x08,0x32,0x10,0x08,0x33,0x04,0x43,0x05,
	//0x37,0x04,0x3F,0x06,0x61,0x61,0x06} 
	
	data_array[0] = 0x00073902;//             
	data_array[1] = 0x648000B2;   // b2
	data_array[2] = 0x002F0D0E; 
	dsi_set_cmdq(&data_array, 3, 1);

      data_array[0] = 0x001F3902;//
	data_array[1] = 0x6C6D6CB4;   // b4
	data_array[2] = 0x016D6C6D; 
	data_array[3] = 0x00757F01; 
	data_array[4] = 0x6C6D6C3F; 
	data_array[5] = 0x016D6C6D; 
      data_array[6] = 0x003f7F01;
       data_array[7] = 0x818181FF;
       data_array[8] = 0x00010881;
      dsi_set_cmdq(&data_array, 9, 1);

	data_array[0] = 0x00033902;//                          
	data_array[1] = 0x006767B6; //B6 
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00223902;//             
	data_array[1] = 0x070000D3;   // d5
	data_array[2] = 0x10074007; 
	data_array[3] = 0x08100800; 
	data_array[4] = 0x15540800; 
	data_array[5] = 0x020E050E; 
	data_array[6] = 0x06050615; 
	data_array[7] = 0x0A0A4447; 
	data_array[8] = 0x0707104B; 
	data_array[9] = 0x0000400E;
	dsi_set_cmdq(&data_array,10, 1);
	
	data_array[0] = 0x002D3902;//             
	data_array[1] = 0x1B1A1AD5;   // d5
	data_array[2] = 0x0201001B; 
	data_array[3] = 0x06050403; 
	data_array[4] = 0x0A090807; 
	data_array[5] = 0x1825240B; 
	data_array[6] = 0x18272618; 
	data_array[7] = 0x18181818; 
	data_array[8] = 0x18181818; 
	data_array[9] = 0x18181818;
	data_array[10] = 0x20181818;
	data_array[11] = 0x18181821;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(&data_array,13, 1);
	
	data_array[0] = 0x002D3902;//             
	data_array[1] = 0x1B1A1AD6;   // d6
	data_array[2] = 0x090A0B1B; 
	data_array[3] = 0x05060708; 
	data_array[4] = 0x01020304; 
	data_array[5] = 0x18202100; 
	data_array[6] = 0x18262718; 
	data_array[7] = 0x18181818; 
	data_array[8] = 0x18181818; 
	data_array[9] = 0x18181818;
	data_array[10] = 0x25181818;
	data_array[11] = 0x18181824;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(&data_array,13, 1);

	/*data_array[0] = 0x003b3902;//                     
	data_array[1] = 0x190D00E0;   // E0         
	data_array[2] = 0x2B272320;         
	data_array[3] = 0x75655829;         
	data_array[4] = 0x8F897A73;         
	data_array[5] = 0x969A9A8F;         
	data_array[6] = 0x5557B4A4;         
	data_array[7] = 0x655E5C58;         
	data_array[8] = 0x0D006769;         
	data_array[9] = 0x27232019;         
	data_array[10] = 0x6558292B;         
	data_array[11] = 0x897A7375;  
	data_array[12] = 0x9A9A8F8F;         
	data_array[13] = 0x57B4A496;   
	data_array[14] = 0x5E5C5855;         
	data_array[15] = 0x00676965;         
	dsi_set_cmdq(&data_array, 16, 1);     */
       data_array[0] = 0x003b3902;//
      data_array[1] = 0x261800E0;   // E0
      data_array[2] = 0x34312e2b;
      data_array[3] = 0x82736431;
      data_array[4] = 0x9693847f;
      data_array[5] = 0x9aa0a197;
      data_array[6] = 0x5659b5A6;
      data_array[7] = 0x645f5C59;
      data_array[8] = 0x18006769;
      data_array[9] = 0x312e2b26;
      data_array[10] = 0x73643134;
      data_array[11] = 0x93847f82;
      data_array[12] = 0xa0a19796;
      data_array[13] = 0x59B5A69a;
      data_array[14] = 0x5f5c5956;
      data_array[15] = 0x00676964;
      dsi_set_cmdq(&data_array, 16, 1);

	data_array[0] = 0x00033902;//             
	data_array[1] = 0x00731FC0;   // c0
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902;//             
	data_array[1] = 0x00000BCC; 
	dsi_set_cmdq(&data_array, 2, 1);

	data_array[0] = 0x00023902;
	data_array[1] = 0x000002D4; 
	dsi_set_cmdq(&data_array, 2, 1);   
	
	data_array[0] = 0x00023902;
	data_array[1] = 0x000001BD; 
	dsi_set_cmdq(&data_array, 2, 1); 
	data_array[0] = 0x00023902;
	data_array[1] = 0x000060B1; 
	dsi_set_cmdq(&data_array, 2, 1); 
	data_array[0] = 0x00023902;
	data_array[1] = 0x000000BD; 
	dsi_set_cmdq(&data_array, 2, 1); 
	data_array[0] = 0x00083902;
	data_array[1] = 0x508140BF; 
	data_array[2] = 0x01FC1A00;
	dsi_set_cmdq(&data_array, 3, 1); 
#if 0
	data_array[0] = 0x00351500;//             
	dsi_set_cmdq(&data_array, 1, 1); 

	//MDELAY(25); 
#endif
	data_array[0] = 0x00110500;     
	dsi_set_cmdq(&data_array, 1, 1); 
	MDELAY(200);
	data_array[0] = 0x00290500;     
	dsi_set_cmdq(&data_array, 1, 1);
	MDELAY(10);
}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    lcm_debug("%s %d\n", __func__,__LINE__);
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	lcm_debug("%s %d\n", __func__,__LINE__);
	memset(params, 0, sizeof(LCM_PARAMS));
	
	params->type   = LCM_TYPE_DSI;

	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 				= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity		= LCM_POLARITY_RISING;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode   = CMD_MODE;
#else
    params->dsi.mode   = BURST_VDO_MODE;
#endif

#if 0
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x1c;
#endif

	/* Command mode setting */
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active                                = 4;//4;//2;
	params->dsi.vertical_backporch                                  = 12;//16;//14;
	params->dsi.vertical_frontporch                                 = 15;//16;
	params->dsi.vertical_active_line                                = FRAME_HEIGHT; 

	params->dsi.horizontal_sync_active                              = 20;//6;//2;
	params->dsi.horizontal_backporch                                = 40;//44;//44;//42;
	params->dsi.horizontal_frontporch                               = 60;//44;
	params->dsi.horizontal_active_pixel                             = FRAME_WIDTH;

	params->dsi.PLL_CLOCK = 200;//dsi clock customization: should config clock value directly
}

#ifdef BUILD_LK
extern int IMM_GetOneChannelValue(int dwChannel, int data[4], int* rawdata);
#else
extern int tps65132_write_bytes(unsigned char addr, unsigned char value);
#endif
static unsigned int lcm_compare_id(void)
{
    unsigned int hd_id1=0;
    unsigned int hd_id2=0;
    unsigned int id1 = 0xFF;
    unsigned int id2 = 0xFF;
    unsigned int id3 = 0xFF;
    unsigned char buffer[3];
    unsigned int data_array[16];
    int lcm_adc = 0, data[4] = {0,0,0,0}, ret=0;

    lcm_debug("hx8394f_hd720_dsi_vdo_tcl %s %d\n", __func__,__LINE__);
#ifdef BUILD_LK
    //IMM_GetOneChannelValue(12, &data, &lcm_adc); //read lcd _id
   /* IMM_GetOneChannelValue_Cali(0,&lcm_adc);
	printf("w5   %d\n\n",lcm_adc);
   if ((lcm_adc >0) && (lcm_adc <50000))
		printf("w5   HX8394F TXD\n");	
		else
			return 0;*/
#endif
	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	MDELAY(10);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
    MDELAY(5);//w5
#ifdef BUILD_LK
	ret=TPS65132_write_byte(0x0 ,0xC); //5.2V
	if(ret) 	
		printf("[LK]hx8394f_tcl ----tps65132----ret=%0x--i2c write error----\n",ret);		
	else
		printf("[LK]hx8394f_tcl ----tps6132----ret=%0x--i2c write success----\n",ret);			
#else
	ret=tps65132_write_bytes(0x0, 0xC);
	if(ret<0)
		printk("[KERNEL]hx8394f_tcl ----tps65132---ret=%0x-- i2c write error-----\n",ret);
	else
		printk("[KERNEL]hx8394f_tcl ----tps65132---ret=%0x-- i2c write success-----\n",ret);
#endif
MDELAY(10);
#ifdef BUILD_LK
	ret=TPS65132_write_byte(0x1, 0xC); //-5.2v
	if(ret) 	
		dprintf(0, "[LK]hx8394f_tcl ----tps65132----ret=%0x--i2c write error----\n",ret);		
	else
		dprintf(0, "[LK]hx8394f_tcl ----tps65132----ret=%0x--i2c write success----\n",ret);  
#else
	ret=tps65132_write_bytes(0x1, 0xC);
	if(ret<0)
		printk("[KERNEL]hx8394f_tcl ----tps6132---ret=%0x-- i2c write error-----\n",ret);
	else
		printk("[KERNEL]hx8394f_tcl ----tps6132---ret=%0x-- i2c write success-----\n",ret);
#endif

///////////////////////////////
    ret=tps65132_write_bytes(0x03, 0x33);
    ret=tps65132_write_bytes(0xff, 0x80);
///////////////////////////////
	MDELAY(10);

	SET_RESET_PIN(1);
	MDELAY(5);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(40);

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

/*
	lcm_util.set_gpio_out(GPIO_LCM_ENN, GPIO_OUT_ONE);//   -5.0V
	MDELAY(7);
	lcm_util.set_gpio_out(GPIO_LCM_ENP, GPIO_OUT_ONE);//    5.0V
	MDELAY(10);
*/
   	//tps6132_reg_i2c_write(0x03, 0x40);//VSP/VSN FLOATING //0x03--- 0x40 ---tablet

    //array[0] = 0x00033700;// read id return three byte,manufacturer,version and id
    //dsi_set_cmdq(array, 1, 1);
    //read_reg_v2(0xDC, buffer, 3);

    //set EXTC
    data_array[0]=0x00043902;
    data_array[1]=0x9483ffb9;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    //set MIPI
    data_array[0]=0x00023902;
    data_array[1]=0x000073ba;
    dsi_set_cmdq(data_array, 2, 1);
    MDELAY(1);

    //set maximum return size
    data_array[0] = 0x00033700;
    dsi_set_cmdq(data_array, 1, 1);

    //read id R04h
   	//data_array[0] = 0x00040600;
   	//dsi_set_cmdq(data_array, 1, 1);

    read_reg_v2(0xdb, buffer, 3);

    id1 = buffer[0]; //we only need ID
    id2 = buffer[1];
    id3 = buffer[2];
    //IMM_GetOneChannelValue(12, &data, &lcm_adc); //read lcd _id


/*	mt_set_gpio_mode(GPIO_LCD_ID1,0);  // gpio mode   high
	mt_set_gpio_pull_enable(GPIO_LCD_ID1,0);
	mt_set_gpio_dir(GPIO_LCD_ID1,0);  //input
	hd_id1= mt_get_gpio_in(GPIO_LCD_ID1);//should be 

	mt_set_gpio_mode(GPIO_LCD_ID2,0);  // gpio mode   high
	mt_set_gpio_pull_enable(GPIO_LCD_ID2,0);
	mt_set_gpio_dir(GPIO_LCD_ID2,0);  //input
	hd_id2 = mt_get_gpio_in(GPIO_LCD_ID2);//should be */

#if defined(BUILD_LK)
    printf("%s,[LK] hx8394a_hd720_dsi_vdo id1 = 0x%08x\n", __func__, id1);
    printf("%s,[LK] hx8394a_hd720_dsi_vdo id2 = 0x%08x\n", __func__, id2);
    printf("%s,[LK] hx8394a_hd720_dsi_vdo: hd_id1 = 0x%08x\n", __func__, hd_id1);
    printf("%s,[LK] hx8394a_hd720_dsi_vdo: hd_id2 = 0x%08x\n", __func__, hd_id2);

    printf("\n [LK] *******lcm_adc=%d*******\n",lcm_adc);
#else
    printk("%s, hx8394a_hd720_dsi_vdo id1 = 0x%08x\n", __func__, id1);
    printk("%s, hx8394a_hd720_dsi_vdo id2 = 0x%08x\n", __func__, id2);
    printk("\n*******lcm_adc=%d *****\n",lcm_adc);
#endif

	if ((LCM_ID1==id1) && (LCM_ID2==id2)) {

			return 1;
	}
	else

		return 0;
}

static void lcm_init(void)
{
	int ret=0;
	lcm_debug("%s %d\n", __func__,__LINE__);
	mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
	mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
/*#ifdef BUILD_LK
		ret=TPS65132_write_byte(0x0 ,0xC); //5.2V
		if(ret) 	
			printf("[LK]hx8394f_tcl ----tps65132----ret=%0x--i2c write error----\n",ret);		
		else
			printf("[LK]hx8394f_tcl ----tps6132----ret=%0x--i2c write success----\n",ret);			
#else
		ret=tps65132_write_bytes(0x0, 0xC);
		if(ret<0)
			printk("[KERNEL]hx8394f_tcl ----tps65132---ret=%0x-- i2c write error-----\n",ret);
		else
			printk("[KERNEL]hx8394f_tcl ----tps65132---ret=%0x-- i2c write success-----\n",ret);
#endif
#ifdef BUILD_LK
		ret=TPS65132_write_byte(0x1, 0xC); //-5.2v
		if(ret) 	
			dprintf(0, "[LK]hx8394f_tcl ----tps65132----ret=%0x--i2c write error----\n",ret);		
		else
			dprintf(0, "[LK]hx8394f_tcl ----tps65132----ret=%0x--i2c write success----\n",ret);	 
#else
		ret=tps65132_write_bytes(0x1, 0xC);
		if(ret<0)
			printk("[KERNEL]hx8394f_tcl ----tps6132---ret=%0x-- i2c write error-----\n",ret);
		else
			printk("[KERNEL]hx8394f_tcl ----tps6132---ret=%0x-- i2c write success-----\n",ret);
#endif*/
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ONE);
	MDELAY(5);
	mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ONE);
	MDELAY(5);
	//ret=tps65132_write_bytes(0x3, 0x33);//w5
    init_lcm_registers();
}

static void lcm_suspend(void)
{
    lcm_debug("%s %d\n", __func__,__LINE__);

	unsigned int data_array[16];

	//data_array[0] = 0x00100500;
	//dsi_set_cmdq(&data_array, 1, 1);
	//MDELAY(150);

	SET_RESET_PIN(1);
	MDELAY(10);
    SET_RESET_PIN(0);	
    MDELAY(20);	
    SET_RESET_PIN(1);
	MDELAY(120);

    mt_set_gpio_mode(GPIO_65132_ENN, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENN, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENN, GPIO_OUT_ZERO);
    MDELAY(10);

    mt_set_gpio_mode(GPIO_65132_ENP, GPIO_MODE_00);
    mt_set_gpio_dir(GPIO_65132_ENP, GPIO_DIR_OUT);
    mt_set_gpio_out(GPIO_65132_ENP, GPIO_OUT_ZERO);
	MDELAY(10);

}

static void lcm_resume(void)
{
	
	lcm_debug("%s %d\n", __func__,__LINE__);
 
    lcm_init();
	
	
}


static void lcm_update(unsigned int x, unsigned int y,
        unsigned int width, unsigned int height)
{
    lcm_debug("%s %d\n", __func__,__LINE__);
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
    data_array[3]= 0x00000000;
    data_array[4]= 0x00053902;
    data_array[5]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
    data_array[6]= (y1_LSB);
    data_array[7]= 0x00000000;
    data_array[8]= 0x002c3909;

    dsi_set_cmdq(data_array, 9, 0);
}

static unsigned int lcm_esd_check(void)
{
#ifndef BUILD_LK
    char buffer[3];
    int array[4];

#if 1
    if(lcm_esd_test)
    {
        lcm_esd_test = FALSE;
        return TRUE;
    }
#endif
	//set EXTC
	/* array[0]=0x00043902;
	array[1]=0x9483ffb9;
	dsi_set_cmdq(array, 2, 1);
	MDELAY(1);

	//set MIPI
	array[0]=0x00023902;
	array[1]=0x000073ba;
	dsi_set_cmdq(array, 2, 1);
	MDELAY(1);*/

	//printk("\n************hx8394f esd_check********\n");
	/*
	 * read 0x45, return 0x05, 0x1d || 0x0f, 0xf0
	 */
	array[0] = 0x00023700; 
    dsi_set_cmdq(array, 1, 1);
	read_reg_v2(0x45, buffer, 2);
	printk("read reg[0x45], return 0x%x, 0x%x\n", buffer[0], buffer[1]);
	if (! (((buffer[0]==0x05)&&(buffer[1]==0x1d)) || ((buffer[0]==0x0f)&&(buffer[1]==0xf0))))
		goto esd_check_fail;

	/*
	 * read 0x09, return 0x80, 0x73
	 */
    array[0] = 0x00023700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0x09, buffer, 2);
	printk("read reg[0x09], return 0x%x, 0x%x\n", buffer[0], buffer[1]);
    if (! ((buffer[0]==0x80) && (buffer[1]==0x73)))
		goto esd_check_fail;

	/*
	 * read 0xCC, return 0x0B
	 */
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xCC, buffer, 1);
	printk("read reg[0xCC], return 0x%x\n", buffer[0]);
    if (! (buffer[0] == 0x0B))
		goto esd_check_fail;

	/*
	 * read 0xD9, return 0x80
	 */
    array[0] = 0x00013700;
    dsi_set_cmdq(array, 1, 1);
    read_reg_v2(0xD9, buffer, 1);
	printk("read reg[0xD9], return 0x%x\n", buffer[0]);
    if (! (buffer[0] == 0x80))
		goto esd_check_fail;

	// both ok
	return FALSE;

esd_check_fail:
    return TRUE;
#endif
}

static unsigned int lcm_esd_recover(void)
{
	// FIXME
	//lcm_suspend();
	lcm_resume();

	//lcm_init();

#ifndef BUILD_LK
    printk("lcm_esd_recover hx8394f\n");
#endif
    return TRUE;
}

LCM_DRIVER hx8394f_dsi_vdo_hd720_drv = 
{
    .name			= "hx8394f_dsi_vdo_hd720",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
    .set_backlight	= lcm_setbacklight,
    .update   		= lcm_update,
#endif
    .esd_check   	= lcm_esd_check,
    .esd_recover   = lcm_esd_recover,
};



