/* Huaqin  Inc. (C) 2011. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("HUAQIN SOFTWARE")
 * RECEIVED FROM HUAQIN AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. HUAQIN EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES HUAQIN PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE HUAQIN SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN HUAQIN SOFTWARE. HUAQIN SHALL ALSO NOT BE RESPONSIBLE FOR ANY HUAQIN
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND HUAQIN'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE HUAQIN SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT HUAQIN'S OPTION, TO REVISE OR REPLACE THE HUAQIN SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * HUAQIN FOR SUCH HUAQIN SOFTWARE AT ISSUE.
 *

 */


/*******************************************************************************
* Dependency
*******************************************************************************/

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
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <mach/mtk_nand.h>

#include <linux/proc_fs.h>

#include <mach/hardwareinfo.h>


#define HARDWARE_INFO_VERSION   0x7503

hardware_info_struct hardware_info;

/****************************************************************************** 
 * Function Configuration
******************************************************************************/
//extern LCM_PARAMS *_get_lcm_driver_by_handle(disp_lcm_handle *plcm);

/****************************************************************************** 
 * Debug configuration
******************************************************************************/
#define EMMC_INFO "/sys/devices/platform/mtk-msdc.0/mmc_host/mmc0/mmc0:0001/manfid"
static unsigned int get_emmc_info(void)
{
        
    struct file *fp = NULL;
    unsigned int manfid;
    char buff[30] = {0};
    mm_segment_t fs;  
    loff_t pos;  
    fp = filp_open(EMMC_INFO, O_RDONLY, 0);

    if (IS_ERR(fp) || !fp->f_op)
    {
            printk("lpz emmc filp_open return NULL\n");
            return NULL;
    }
    printk("lpz emmc read\n");
    fs = get_fs();  
    set_fs(KERNEL_DS);  
    pos = 0;  
    vfs_read(fp, buff, sizeof(buff), &pos);  
    filp_close(fp,NULL);
    printk("lpz read: %s\n", buff);
    sscanf(buff, "0x%06x\n", &manfid);
    return manfid;
}

//hardware info driver
static ssize_t show_hardware_info(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;
    unsigned int manfid = 0;

    //show lcm driver 
    printk("show_lcm_info return lcm name\n");
    count += sprintf(buf, "LCM Name:       ");//12 bytes
    if(hardware_info.hq_lcm_name){      
        if (!strcmp(hardware_info.hq_lcm_name, "ili9881c_dsi_vdo_txd")) {
                count += sprintf(buf+count, "Txd "); 
        }  else if (!strcmp(hardware_info.hq_lcm_name, "hx8394f_dsi_vdo_hd720")) {
                count += sprintf(buf+count, "Tcl "); 
        } else {
                count += sprintf(buf+count, "DJ "); 
        }
            count += sprintf(buf+count, "%s %s\n", hardware_info.hq_lcm_name, "5.0\"_HD");
    }else {
            count += sprintf(buf+count,"no lcd name found\n");
    }

        count += sprintf(buf + count, "ctp name:       ");//12bytes
        if(hardware_info.hq_tpd_name != NULL)
        {
                if (!strcmp(hardware_info.hq_tpd_name, "GT915L")) {
                        count += sprintf(buf+count, "Top-Touch "); 
						count += sprintf(buf + count, "cfg_version:%x\n", hardware_info.tpd_cfg_version);
                } else {
                        count += sprintf(buf+count, "O-Film "); 
                }
                count += sprintf(buf + count, "%s ", hardware_info.hq_tpd_name);
                count += sprintf(buf + count, "firmware:%x\n", hardware_info.tpd_fw_version);
        }
        else
        {
                count += sprintf(buf + count, "ctp driver NULL !\n");
        }
/*
    //show nand drvier name 
    printk("show nand device name\n");
    count += sprintf(buf+count, "NAND Name:  ");//12 bytes
    if(hardware_info.nand_device_name) 
        count += sprintf(buf+count, "%s\n", hardware_info.nand_device_name);
    else
        count += sprintf(buf+count,"no nand name found\n");    
*/
        manfid = get_emmc_info();
        if (manfid == 0) {
                count += sprintf(buf + count, "%s", "emmc :invild\n");
        } else if (manfid == 0x90){
                count += sprintf(buf + count, "%s", "EMMC info:      Hynix H9TQ64A8GTMCUR_KUM ROM:8G RAM:1G\n");
        } else if (manfid == 0x15){
                count += sprintf(buf + count, "%s", "EMMC info:      Samsung KMFN10012M_B214 ROM:8G RAM:1G\n");
                printk("manfid=0x%x\n", manfid);
        } else {
                count += sprintf(buf + count, "%s", "EMMC info:      Toshiba TYD0GH121661RA ROM:8G RAM:1G\n");
                printk("manfid=0x%x\n", manfid);
        }
    //show Main Camera drvier name
    printk("show Main Camera device name\n");
    count += sprintf(buf+count, "Main Camera:    ");//12 bytes
    if(hardware_info.mainCameraName) { 
            if (!strcmp(hardware_info.mainCameraName, "imx219mipiraw")) {
                    count += sprintf(buf+count, "O-Film "); 
            } else if(!strcmp(hardware_info.mainCameraName, "ov8856mipiraw"))
 	    {
                    count += sprintf(buf+count, "Q-Tech "); 
            } else
	   {
		count += sprintf(buf+count, "give up "); 
	   }
            count += sprintf(buf+count, "%s %s\n", hardware_info.mainCameraName, "AF 8M F2.0");
    }

    //show Sub Camera drvier name
    printk("show Sub camera name\n");
    count += sprintf(buf+count, "Sub Camera:     ");//12 bytes
    if(hardware_info.subCameraName){
            if (!strcmp(hardware_info.subCameraName, "gc2355mipiraw")) {
                    count += sprintf(buf+count, "HuaQuan "); 
            } else if (!strcmp(hardware_info.subCameraName, "ov2680mipiraw")) {
                    count += sprintf(buf+count, "SunWin "); 
            }else {
                    count += sprintf(buf+count, "give up "); 
            }
            count += sprintf(buf+count, "%s %s\n", hardware_info.subCameraName, "FF 2M F2.2");
        }

        
    //A20 using 6620
    //show WIFI
    printk("show WIFI name\n");
    count += sprintf(buf+count, "WIFI&BT&GPS&FM: ");//16 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no WIFI name found\n");
#if 0
    printk("show Bluetooth name\n");
    count += sprintf(buf+count, "Bluetooth:  ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no Bluetooth name found\n");

    printk("show GPS name\n");
    count += sprintf(buf+count, "GPS:        ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no GPS name found\n");

    printk("show FM name\n");
    count += sprintf(buf+count, "FM:         ");//12 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"no FM name found\n");
#endif
    
    count += sprintf(buf+count, "AlSPS name :    "); 	
    if(hardware_info.alsps_name) {
            count += sprintf(buf+count, "%s %s\n","dyna-image",hardware_info.alsps_name); 	
    }else{
            count += sprintf(buf+count, "AlSPS name  :Not found\n"); 
    }
    count += sprintf(buf+count, "GSensor name:   ");   
    if(hardware_info.gsensor_name){
            if (!strcmp(hardware_info.gsensor_name, "kxtj2_1009")) {
                      
                    count += sprintf(buf+count, "Kionix "); 
            } else {
                    count += sprintf(buf+count, "Bosch "); 
            }
            count += sprintf(buf+count, "%s\n",hardware_info.gsensor_name);
    }else{
            count += sprintf(buf+count, "GSensor name:Not found\n"); 
    }
    count += sprintf(buf+count, "Compass name:   ");
    count += sprintf(buf+count, "AK09911C\n");
    count += sprintf(buf+count, "Flashlight name:");
    count += sprintf(buf+count, "SGM3785\n");
    ret_value = count;

    return ret_value;
}
static ssize_t store_hardware_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


extern char* fgauge_get_battery_manufacturer(void);
extern int battery_not_match;
static ssize_t show_manufacturer_info(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;
    unsigned int manfid = 0;


    count += sprintf(buf, "board ID:       MT6582,Y5IICUN\n");//16 bytes
    /****************************************/
    count += sprintf(buf + count, "ctp name:       ");//16bytes
    if(hardware_info.hq_tpd_name != NULL)
    {
            if (!strcmp(hardware_info.hq_tpd_name, "GT915L")) {
                    count += sprintf(buf+count, "Top-Touch "); 
            } else {
                    count += sprintf(buf+count, "O-Film "); 
            }
            count += sprintf(buf + count, "%s \n", hardware_info.hq_tpd_name);
    }
    else
    {
            count += sprintf(buf + count, "No ctp found!\n");
    }

    /****************************************/
    //show Main Camera drvier name
    printk("show Main Camera device name\n");
    count += sprintf(buf+count, "Main Camera:    ");//16 bytes
    if(hardware_info.mainCameraName) { 
            if (!strcmp(hardware_info.mainCameraName, "s5k4h8mipiraw")) {
                    count += sprintf(buf+count, "O-Film "); 
            } else {
                    count += sprintf(buf+count, "Q-Tech "); 
            }
            count += sprintf(buf+count, "%s\n", hardware_info.mainCameraName);
    }

    //show Sub Camera drvier name
    printk("show Sub camera name\n");
    count += sprintf(buf+count, "Sub Camera:     ");//12 bytes
    if(hardware_info.subCameraName){
            if (!strcmp(hardware_info.subCameraName, "gc2355mipiraw")) {
                    count += sprintf(buf+count, "HuaQuan "); 
            } else {
                    count += sprintf(buf+count, "SunWin "); 
            }
            count += sprintf(buf+count, "%s\n", hardware_info.subCameraName);
    }
    /****************************************/
    manfid = get_emmc_info();
    if (manfid == 0) {
            count += sprintf(buf + count, "%s", "emmc :invild\n");
            count += sprintf(buf + count, "%s", "ddr :invild\n");
    } else if (manfid == 0x90){
            count += sprintf(buf + count, "%s", "EMMC info:      Hynix H9TQ64A8GTMCUR_KUM ROM:8G\n");
            count += sprintf(buf + count, "%s", "DDR  info:      Hynix H9TQ64A8GTMCUR_KUM RAM:1G\n");
    } else if (manfid == 0x15){
            count += sprintf(buf + count, "%s", "EMMC info:      Samsung KMFN10012M_B214 ROM:8G\n");
            count += sprintf(buf + count, "%s", "DDR  info:      Samsung KMFN10012M_B214 RAM:1G\n");
            printk("manfid=0x%x\n", manfid);
    } else {
            count += sprintf(buf + count, "%s", "EMMC info:      Toshiba TYD0GH121661RA ROM:8G\n");
            count += sprintf(buf + count, "%s", "DDR  info:      Toshiba TYD0GH121661RA RAM:1G\n");
            printk("manfid=0x%x\n", manfid);
    }

    /****************************************/
    count += sprintf(buf+count, "LCM Name:       ");//16 bytes
    if(hardware_info.hq_lcm_name){      
        if (!strcmp(hardware_info.hq_lcm_name, "ili9881c_dsi_vdo_txd")) {
                count += sprintf(buf+count, "Txd "); 
        } else {
                count += sprintf(buf+count, "Dj "); 
        }
            count += sprintf(buf+count, "%s \n", hardware_info.hq_lcm_name);
    }else {
            count += sprintf(buf+count,"No lcd name found!\n");
    }
    /****************************************/
    char *battery = NULL;
     battery = fgauge_get_battery_manufacturer();
     printk("battery:%s\n", battery);
     if ((!strcmp(battery,"SUNWODA"))&&(!battery_not_match))        
             count += sprintf(buf+count, "battery;        %s %s\n", battery, "PGF364197HT");//16 bytes
     else if ((!strcmp(battery,"SCUD"))&&(!battery_not_match))        
             count += sprintf(buf+count, "battery;        %s %s\n", battery, "ATL364197H");//16 bytes
     else
             count += sprintf(buf+count,"        Battery is not match!\n");     
    /****************************************/

    printk("show GPS name\n");
    count += sprintf(buf+count, "GPS:            ");//16 bytes
    if(1) 
        count += sprintf(buf+count, "MT6625L\n");
    else
        count += sprintf(buf+count,"No GPS found!\n");
    /****************************************/
    count += sprintf(buf+count, "compass:        ");//16 bytes
    count += sprintf(buf+count, "AK09911C\n");
    /****************************************/
    count += sprintf(buf+count, "GSensor name:   ");   
    if(hardware_info.gsensor_name){
            if (!strcmp(hardware_info.gsensor_name, "kxtj2_1009")) {
                      
                    count += sprintf(buf+count, "Kionix "); 
            } else {
                    count += sprintf(buf+count, "Bosch "); 
            }
            count += sprintf(buf+count, "%s\n",hardware_info.gsensor_name);
    }else{
            count += sprintf(buf+count, "No GSensor found!\n"); 
    }
    /****************************************/
    if(hardware_info.alsps_name) {
            count += sprintf(buf+count, "AlS   name:     "); 	
            count += sprintf(buf+count, "%s %s\n","dyna-image",hardware_info.alsps_name); 	
    }else{
            count += sprintf(buf+count, "No AlSPS found!\n"); 
    }
    count += sprintf(buf+count, "gyroscope:      NULL\n"); 	
    if(hardware_info.alsps_name) {
            count += sprintf(buf+count, "PS name:        "); 	
            count += sprintf(buf+count, "%s %s\n","dyna-image",hardware_info.alsps_name); 	
    }else{
            count += sprintf(buf+count, "No AlSPS found!\n"); 
    }
    /****************************************/
    count += sprintf(buf+count, "WIFI:           ");//16 bytes
    count += sprintf(buf+count, "MT6625L\n");
    count += sprintf(buf+count, "BT:             ");//16 bytes
    count += sprintf(buf+count, "MT6625L\n");
    count += sprintf(buf+count, "NFC:            NULL\n");//16 bytes
    ret_value = count;

    return ret_value;
}
static ssize_t store_manufacturer_info(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

static ssize_t show_version(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
        
    ret_value = sprintf(buf, "Version:    :0x%x\n", HARDWARE_INFO_VERSION);     

    return ret_value;
}
static ssize_t store_version(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}




static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

        //lcm_drv = _get_lcm_driver_by_handle(plcm);
        
    ret_value = sprintf(buf, "lcd name    :%s\n", hardware_info.hq_lcm_name);    

    return ret_value;
}
static ssize_t store_lcm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;
        
        if(hardware_info.hq_tpd_name != NULL)
        {
                count += sprintf(buf + count, "ctp driver  :%s , ctp fw ver  :%x\n", hardware_info.hq_tpd_name, hardware_info.tpd_fw_version);
        }
        else
        {
                count += sprintf(buf + count, "ctp driver NULL !\n");
        }

        ret_value = count;
        
    return ret_value;
}
static ssize_t store_ctp(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;

   
    if(hardware_info.mainCameraName)
        ret_value = sprintf(buf , "main camera :%s\n", hardware_info.mainCameraName);
    else
        ret_value = sprintf(buf , "main camera :PLS TURN ON CAMERA FIRST\n");
        
        
    return ret_value;
}
static ssize_t store__main_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    if(hardware_info.subCameraName)
        ret_value = sprintf(buf , "sub camera  :%s\n", hardware_info.subCameraName);
    else
        ret_value = sprintf(buf , "sub camera  :PLS TURN ON CAMERA FIRST\n");

        
    return ret_value;
}
static ssize_t store_sub_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

/*
static ssize_t show_flash(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int count = 0;

    ret_value = sprintf(buf , "NAND Name  :%s\n",  devinfo.devciename);

    return ret_value;
}

static ssize_t store_flash(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);	
    return size;
}

*/

//MT6625L names
static ssize_t show_wifi(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "wifi name   :MT6625L\n"); 	
    return ret_value;
}
static ssize_t show_bt(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "bt name     :MT6625L\n"); 	
    return ret_value;
}
static ssize_t show_gps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "GPS name    :MT6625L\n"); 	
    return ret_value;
}
static ssize_t show_fm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    ret_value = sprintf(buf, "FM name     :MT6625L\n"); 	
    return ret_value;
}




static ssize_t store_wifi(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);	
    return size;
}
static ssize_t store_bt(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);	
    return size;
}
static ssize_t store_gps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);	
    return size;
}
static ssize_t store_fm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);	
    return size;
}

static ssize_t show_alsps(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    if(hardware_info.alsps_name)
        ret_value = sprintf(buf, "AlSPS name  :%s\n",hardware_info.alsps_name); 	
    else
        ret_value = sprintf(buf, "AlSPS name  :Not found\n"); 

    return ret_value;
}

static ssize_t store_alsps(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_gsensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    if(hardware_info.gsensor_name)
        ret_value = sprintf(buf, "GSensor name:%s\n",hardware_info.gsensor_name);   
    else
        ret_value = sprintf(buf, "GSensor name:Not found\n"); 
    
    return ret_value;
}

static ssize_t store_gsensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}


static ssize_t show_msensor(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if 1
    if(hardware_info.msensor_name)
        ret_value = sprintf(buf, "MSensor name:%s\n",hardware_info.msensor_name);   
    else
        ret_value = sprintf(buf, "MSensor name:Not found\n"); 
    #else
    ret_value = sprintf(buf, "MSensor name:Not support MSensor\n");     
    #endif
    return ret_value;
}

static ssize_t store_msensor(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

static ssize_t show_gyro(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    #if 0
    if(hardware_info.gyro_name)
        ret_value = sprintf(buf, "Gyro  name  :%s\n",hardware_info.gyro_name);      
    else
        ret_value = sprintf(buf, "Gyro  name  :Not found\n"); 
    #else
    ret_value = sprintf(buf, "Gyro  name  :Not support Gyro\n");        
    #endif
    return ret_value;
}

static ssize_t store_gyro(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);      
    return size;
}

static ssize_t show_hwnff(struct device *dev,struct device_attribute *attr, char *buf)
{
        int ret_value = 1;

        int count = 0;
#if 1
        printk("lpz enter show_hwnff\n");
        count += sprintf(buf + count, "Touch screen type: %s\n", hardware_info.hq_tpd_name);

        printk("lpz 1 buf:%s",buf);
        if(hardware_info.mainCameraName)
                count += sprintf(buf + count, "Main Camera type:  %s\n", hardware_info.mainCameraName);
        else
                count += sprintf(buf + count, "Main Camera not found\n"); 	
            
        printk("lpz 2buf:%s",buf);
        if(hardware_info.subCameraName)
                count += sprintf(buf + count, "Sub Camera type:   %s\n", hardware_info.subCameraName);
        else
                count += sprintf(buf + count, "Sub Camera not found\n");
              
        printk("lpz 3buf:%s",buf);

        count += sprintf(buf + count, "Lcd type:          %s\n", hardware_info.hq_lcm_name);	

        printk("lpz 4buf:%s\n",buf);

        count += sprintf(buf + count, "Gps type:          MTK-MT6625L\n");

        count += sprintf(buf + count, "Wifi type:         MTK-MT6625L\n");

        count += sprintf(buf + count, "BlueTooth type:    MTK-MT6625L\n");					 

        printk("lpz 5buf:%s\n",buf);
        if(hardware_info.gsensor_name)
                count += sprintf(buf + count, "Gsensor type:    %s\n", hardware_info.gsensor_name);
        else
               count += sprintf(buf + count, "Gsensor type:    Not found\n"); 
 #if 1
        printk("lpz 6buf:%s\n",buf);
        if(hardware_info.alsps_name)
                count = sprintf(buf+count, "AlSPS type:    %s\n",hardware_info.alsps_name); 	
        else
                count = sprintf(buf+count, "AlSPS type:    Not found\n");
#endif
#endif
        printk("lpz 7buf:%s",buf);
        ret_value = count;

        return ret_value;
}

static ssize_t store_hwnff(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: kk Not Support Write Function\n",__func__);
    return size;
}

//add by sunsongjian at 20140509 begin
extern char* saved_command_line;

static ssize_t show_bootflag(struct device *dev,struct device_attribute *attr, char *buf)
{    
	int ret_value = 0;
	int count = 0;
	char *p;   
	unsigned int sdcard_up = 0;
	p = strstr(saved_command_line, "sdcard_up="); 
	
	if(p == NULL)    
	{	
		sprintf(buf, "0");    	
		return ret_value;    
	}       
	p += 10;   

	if((p - saved_command_line) > strlen(saved_command_line+1))    
	{	
		sprintf(buf, "0");    	
		return ret_value;   
	}    

	sdcard_up = simple_strtol(p, NULL, 0);
	   printk("lpz sdcard_up=%d\n", sdcard_up);
	count += sprintf(buf + count, "%d", sdcard_up);

	ret_value = count;
	
	return ret_value;
}

static ssize_t store_bootflag(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);	
    return size;
}



static DEVICE_ATTR(99_version, 0644, show_version, store_version);
static DEVICE_ATTR(00_lcm, 0644, show_lcm, store_lcm);
static DEVICE_ATTR(01_ctp, 0644, show_ctp, store_ctp);
static DEVICE_ATTR(02_main_camera, 0644, show_main_camera, store__main_camera);
static DEVICE_ATTR(03_sub_camera, 0644, show_sub_camera, store_sub_camera);

//static DEVICE_ATTR(04_flash, 0644, show_flash, store_flash);
static DEVICE_ATTR(05_gsensor, 0644, show_gsensor, store_gsensor);
static DEVICE_ATTR(06_msensor, 0644, show_msensor, store_msensor);
static DEVICE_ATTR(08_gyro, 0644, show_gyro, store_gyro);


static DEVICE_ATTR(07_alsps, 0644, show_alsps, store_alsps);


static DEVICE_ATTR(09_wifi, 0644, show_wifi, store_wifi);
static DEVICE_ATTR(10_bt, 0644, show_bt, store_bt);
static DEVICE_ATTR(11_gps, 0644, show_gps, store_gps);
static DEVICE_ATTR(12_fm, 0644, show_fm, store_fm);
static DEVICE_ATTR(hq_hardwareinfo, 0644, show_hardware_info, store_hardware_info);
static DEVICE_ATTR(hq_manfinfo, 0644, show_manufacturer_info, store_manufacturer_info);
static DEVICE_ATTR(20_hwnff, 0644, show_hwnff, store_hwnff);

//add by sunsongjian at 20140509 begin
static DEVICE_ATTR(30_bootflag, 0644, show_bootflag, store_bootflag);
//add end






///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int HardwareInfo_driver_probe(struct platform_device *dev)       
{       
        int ret_device_file = 0;

    printk("** HardwareInfo_driver_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_00_lcm)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_01_ctp)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_02_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_03_sub_camera)) != 0) goto exit_error;
  
//    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_04_flash)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_05_gsensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_06_msensor)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_08_gyro)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_07_alsps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_09_wifi)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_10_bt)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_11_gps)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_12_fm)) != 0) goto exit_error;

    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_20_hwnff)) != 0) goto exit_error;

    //add by sunsongjian at 20140509 begin
	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_30_bootflag)) != 0) goto exit_error;
    //add end

	if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_99_version)) != 0) goto exit_error;   
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_hq_hardwareinfo)) != 0) goto exit_error;  
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_hq_manfinfo)) != 0) goto exit_error; 

exit_error:     
    return ret_device_file;
}

static int HardwareInfo_driver_remove(struct platform_device *dev)
{
        printk("** HardwareInfo_drvier_remove!! **");

        device_remove_file(&(dev->dev), &dev_attr_00_lcm);
    device_remove_file(&(dev->dev), &dev_attr_01_ctp);
    device_remove_file(&(dev->dev), &dev_attr_02_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_03_sub_camera);
    
    device_remove_file(&(dev->dev), &dev_attr_05_gsensor);
    device_remove_file(&(dev->dev), &dev_attr_06_msensor);
    device_remove_file(&(dev->dev), &dev_attr_08_gyro);
    device_remove_file(&(dev->dev), &dev_attr_07_alsps);
    device_remove_file(&(dev->dev), &dev_attr_09_wifi);
    device_remove_file(&(dev->dev), &dev_attr_10_bt);
    device_remove_file(&(dev->dev), &dev_attr_11_gps);
    device_remove_file(&(dev->dev), &dev_attr_12_fm);

    device_remove_file(&(dev->dev), &dev_attr_20_hwnff);

	//add by sunsongjian at 20140509 begin
    device_remove_file(&(dev->dev), &dev_attr_30_bootflag);
    //add end

    device_remove_file(&(dev->dev), &dev_attr_99_version);
    return 0;
}





static struct platform_driver HardwareInfo_driver = {
    .probe              = HardwareInfo_driver_probe,
    .remove     = HardwareInfo_driver_remove,
    .driver     = {
        .name = "HardwareInfo",
    },
};

static struct platform_device HardwareInfo_device = {
    .name   = "HardwareInfo",
    .id     = -1,
};




static int __init HardwareInfo_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&HardwareInfo_device);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }
    

    ret = platform_driver_register(&HardwareInfo_driver);
    if (ret) {
        printk("**HardwareInfo_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

    goto ok_result;

    
fail_1:
        platform_driver_unregister(&HardwareInfo_driver);
fail_2:
        platform_device_unregister(&HardwareInfo_device);
ok_result:

    return ret;
}


/*****************************************************************************/
static void __exit HardwareInfo_mod_exit(void)
{
    platform_driver_unregister(&HardwareInfo_driver);
        platform_device_unregister(&HardwareInfo_device);
}
/*****************************************************************************/
module_init(HardwareInfo_mod_init);
module_exit(HardwareInfo_mod_exit);
/*****************************************************************************/
MODULE_AUTHOR("Kaka Ni <nigang@huaqin.com>");
MODULE_DESCRIPTION("MT6575 Hareware Info driver");
MODULE_LICENSE("GPL");

