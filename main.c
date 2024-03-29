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
void CalibrateMagn(void);

int main(void)
{  
  char string[255];
  uint8_t status;
  SRAWDATA accel_data_raw;
  SRAWDATA mag_data_raw;
  SDATA accel_data;
  SDATA mag_data;
  SDATA rpy_data;

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
      ComputeRpy(&accel_data, &mag_data, &rpy_data);
      
      sprintf(string, "Accel (mg): x:%f, y:%f, z:%f", accel_data.x, accel_data.y, accel_data.z);
      put(string);
      sprintf(string, "\tMagn (uT): x:%f, y:%f, z:%f", mag_data.x, mag_data.y, mag_data.z);
      put(string);
      sprintf(string, "\tRpy (deg): x:%f, y:%f, z:%f\n\r", rpy_data.x, rpy_data.y, rpy_data.z);
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
      Blue_LED(LED_OFF);
      Red_LED(LED_OFF);
      Green_LED(LED_OFF);
      if(whoami() == 1)
      {
        Red_LED(LED_ON);
      }
      else
      {
        Green_LED(LED_ON);
#if DEBUG_RAW
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
#endif

        CalibrateAccel();
        Green_LED(LED_OFF);
      }
    }
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
#if DEBUG
    put("I2C initialization failure! :(\n\r");
#endif
    Red_LED(LED_ON);
  }
  else
  {
#if DEBUG
	  put("I2C initialization success! :)\n\r");
#endif
    Green_LED(LED_ON);
    /* Calibrate Magnetometer at this time */
    CalibrateMagn();
    Red_LED(LED_ON);
    Blue_LED(LED_ON);
#if DEBUG
    put("Press a key to end magnetometer calibration.\n\r");
#endif
    uart_getchar();
    CalibrateAccel();
    Green_LED(LED_OFF);
    Red_LED(LED_OFF);
    Blue_LED(LED_OFF);
  }
}

/**
 * Calibrates by finding min/max values for each axis
 * Green LED toggles for each min/max value obtained until all six are found
 */
void CalibrateMagn()
{
  uint8_t flag = 0x00;
  uint8_t status;
  SRAWDATA accel_data_raw;
  SRAWDATA mag_data_raw;
  SDATA accel_data;
  SDATA mag_data;
  
  while(flag != 0x3F)
  {
    if(DataReady == 1)
    {
      DataReady = 0;
      
      status = ReadAccelMagnData(&accel_data_raw, &mag_data_raw);
      ConvertAccelMagnData(&accel_data_raw, &mag_data_raw, &accel_data, &mag_data);
      
      if(mag_data.y < 1 && mag_data.y > -1 &&
         mag_data.z < 1 && mag_data.z > -1 &&
         mag_data.x > 0 && !(flag & 0x01))
      {
        Green_LED(LED_TOGGLE);
        flag |= (1 << 0);
      }
      
      if(mag_data.y < 1 && mag_data.y > -1 &&
         mag_data.z < 1 && mag_data.z > -1 &&
         mag_data.x < 0 && !(flag & 0x02))
      {
        Green_LED(LED_TOGGLE);
        flag |= (1 << 1);
      }
      
      if(mag_data.x < 1 && mag_data.x > -1 &&
         mag_data.z < 1 && mag_data.z > -1 &&
         mag_data.y > 0 && !(flag & 0x04))
      {
        Green_LED(LED_TOGGLE);
        flag |= (1 << 2);
      }
      
      if(mag_data.x < 1 && mag_data.x > -1 &&
         mag_data.z < 1 && mag_data.z > -1 &&
         mag_data.y < 0 && !(flag & 0x08))
      {
        Green_LED(LED_TOGGLE);
        flag |= (1 << 3);
      }
      
      if(mag_data.y < 1 && mag_data.y > -1 &&
         mag_data.x < 1 && mag_data.x > -1 &&
         mag_data.z > 0 && !(flag & 0x10))
      {
        Green_LED(LED_TOGGLE);
        flag |= (1 << 4);
      }
      
      if(mag_data.y < 1 && mag_data.y > -1 &&
         mag_data.x < 1 && mag_data.x > -1 &&
         mag_data.z < 0 && !(flag & 0x20))
      {
        Green_LED(LED_TOGGLE);
        flag |= (1 << 5);
      }
    }
  }
}