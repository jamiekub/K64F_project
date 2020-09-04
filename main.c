#include "stdio.h"
#include "stdlib.h"
#include "MK64F12.h"
#include "uart_drvr.h"
#include "gpio_drvr.h"
#include "mag_accel_drvr.h"
#include "i2c_drvr.h"
#include <string.h>
#include <math.h>

// Default System clock value
// period = 1/20485760  = 4.8814395e-8
#ifndef DEFAULT_SYSTEM_CLOCK
#define DEFAULT_SYSTEM_CLOCK 20485760u
#endif

#define DEBUG_RAW 0
#define DEBUG 0

void initialize(void);
void en_interrupts(void);

int main(void)
{  
  char string[255];
  uint8_t status;
  SRAWDATA accel_data_raw;
  SRAWDATA mag_data_raw;
  SDATA accel_data;
  SDATA mag_data;

  initialize();
  
  for(;;)
  {
    //i2c_read(FXOS8700CQ_INT_SOURCE, &status, 1);
    if(DataReady == 1 )//|| status == 0x01)
    {
      DataReady = 0;
      
      status = ReadAccelMagnData(&accel_data_raw, &mag_data_raw);

#if DEBUG_RAW    
      sprintf(string, "Status: 0x%02X\n\r", status);
      put(string);
      sprintf(string, "RAW Accelerometer Data: x:%d, y:%d, z:%d\n\r", accel_data_raw.x, accel_data_raw.y, accel_data_raw.z);
      put(string);
      sprintf(string, "RAW Magnetometer Data: x:%d, y:%d, z:%d\n\r", mag_data_raw.x, mag_data_raw.y, mag_data_raw.z);
      put(string);
#endif
      
      if(status == 0xFF)
      {
        Red_LED(LED_ON);
        Blue_LED(LED_OFF);
      }
      else
      {
        Red_LED(LED_OFF);
        Blue_LED(LED_ON);
      }

#if DEBUG
      ConvertAccelMagnData(&accel_data_raw, &mag_data_raw, &accel_data, &mag_data);
      
      sprintf(string, "Accelerometer Data (mg): x:%f, y:%f, z:%f", accel_data.x, accel_data.y, accel_data.z);
      put(string);
      sprintf(string, "\tMagnetometer Data (uT): x:%f, y:%f, z:%f\n\r", mag_data.x, mag_data.y, mag_data.z);
      put(string);
#else
      ConvertAccelMagnData(&accel_data_raw, &mag_data_raw, &accel_data, &mag_data);
      
      sprintf(string, "%f\n", accel_data.x);
      put(string);
      sprintf(string, "%f\n", accel_data.y);
      put(string);
      sprintf(string, "%f\n", accel_data.z);
      put(string);
      sprintf(string, "%f\n", mag_data.x);
      put(string);
      sprintf(string, "%f\n", mag_data.y);
      put(string);
      sprintf(string, "%f\n", mag_data.z);
      put(string);
#endif  
    }
    
    if(SW3_pressed())
    {
      Blue_LED(LED_TOGGLE);
      Red_LED(LED_OFF);
      Green_LED(LED_OFF);
      if(whoami() == 1)
      {
        Red_LED(LED_ON);
      }
      else
      {
        Green_LED(LED_ON);
        i2c_read(FXOS8700CQ_M_CTRL_REG1, &status, 1);
        sprintf(string, "M CTRL REG1: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_M_CTRL_REG2, &status, 1);
        sprintf(string, "M CTRL REG2: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_XYZ_DATA_CFG, &status, 1);
        sprintf(string, "XYZ DATA CFG: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_CTRL_REG1, &status, 1);
        sprintf(string, "CTRL REG1: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_CTRL_REG2, &status, 1);
        sprintf(string, "CTRL REG2: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_CTRL_REG3, &status, 1);
        sprintf(string, "CTRL REG3: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_CTRL_REG4, &status, 1);
        sprintf(string, "CTRL REG4: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_CTRL_REG5, &status, 1);
        sprintf(string, "CTRL REG5: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_SYSMOD, &status, 1);
        sprintf(string, "SYSMOD: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_INT_SOURCE, &status, 1);
        sprintf(string, "INT SOURCE: 0x%02X\n\r", status);
        put(string);
        
        i2c_read(FXOS8700CQ_STATUS, &status, 1);
        sprintf(string, "STATUS: 0x%02X\n\r", status);
        put(string);
        
        status = ReadAccelMagnData(&accel_data_raw, &mag_data_raw);
        sprintf(string, "Status: 0x%02X\n\r", status);
        put(string);
        sprintf(string, "RAW Accelerometer Data: x:%d, y:%d, z:%d\n\r", accel_data_raw.x, accel_data_raw.y, accel_data_raw.z);
        put(string);
        sprintf(string, "RAW Magnetometer Data: x:%d, y:%d, z:%d\n\r", mag_data_raw.x, mag_data_raw.y, mag_data_raw.z);
        put(string);
      }
      Blue_LED(LED_TOGGLE);
    }/*
    else if(SW2_pressed())
    {
      Blue_LED(LED_TOGGLE);
      Green_LED(LED_ON);
      put("SW2 Pressed!\n\r");
      delay(50);
      Green_LED(LED_OFF);
      Blue_LED(LED_TOGGLE);
    }*/
  }
  return 0;
}

void initialize()
{
	uart_init();
  LED_init();
  //SW2_init();
  SW3_init();
  //Initialize I2C0 for communication with FXOS8700CQ
  I2C0_init(FXOS8700CQ_ICR, 0x21);
  FXOS8700CQ_INT_init();
  if(FXOS8700CQ_init() == 1)
  {
    //put("I2C initialization failure! :(\n\r");
    Red_LED(LED_ON);
  }
  else
  {
	  //put("I2C initialization success! :)\n\r");
    Green_LED(LED_ON);
    delay(500);
   // put("Press a key to begin accelerometer calibration.\n\r");
    //uart_getchar();
    CalibrateAccel();
  }
}
