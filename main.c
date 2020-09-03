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
      Green_LED(LED_OFF);
      Blue_LED(LED_TOGGLE);
      DataReady = 0;
      
      status = ReadAccelMagnData(&accel_data_raw, &mag_data_raw);

#if DEBUG      
      sprintf(string, "Status: 0x%02X\n\r", status);
      put(string);
      sprintf(string, "RAW Accelerometer Data: x:%d, y:%d, z:%d\n\r", accel_data_raw.x, accel_data_raw.y, accel_data_raw.z);
      put(string);
      sprintf(string, "RAW Magnetometer Data: x:%d, y:%d, z:%d\n\r", mag_data_raw.x, mag_data_raw.y, mag_data_raw.z);
      put(string);
#endif

      ConvertAccelMagnData(&accel_data_raw, &mag_data_raw, &accel_data, &mag_data);
      
      sprintf(string, "Accelerometer Data (mg): x:%f, y:%f, z:%f\n\r", accel_data.x, accel_data.y, accel_data.z);
      put(string);
      sprintf(string, "Magnetometer Data (uT): x:%f, y:%f, z:%f\n\r", mag_data.x, mag_data.y, mag_data.z);
      put(string);
      
      Blue_LED(LED_TOGGLE);
      Green_LED(LED_ON);
      
      //FXOS8700CQ_init(); // This is wrong!
      /* Re-initializing the sensor "fixed" the issue with not receiving interrupts consistently.
      However, I now believe that the issue is caused by my i2c driver.
      The interrupt works correctly after initialization and I get valid data.
      Then a second interrupt is received, but the data is incorrect.
      After this, no more interrupts are received.
      
      I think the issue is that the auto-increment register address mechanism in the sensor 
      is not resetting after the initial data read. Then when we get a second interrupt,
      we read the wrong registers and so the data ready flag is not cleared.
      This means no more interrupts are sent, and further attempts to read data fail.
      Also, the data might be all 0xFF because it's just reading the status register 13 times.*/
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
    
  }
}
