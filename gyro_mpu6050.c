//****************************************
// GY-521 MPU6050 IIC測試程序
// MPU6050 function 
// MCU: PIC24F
// 2013.06.19
// 功能: 顯示加速度計和陀螺儀的10位原始數據
// Regis Hsu
//****************************************
/* Standard includes. */
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "croutine.h"
#include "led.h"

#include "i2c.h"
#include "i2c_func.h"
#include "lcd_i2c_1602.h"
#include "gyro_mpu6050.h"


UINT8 Gyro_i2c_read(UINT8 ControlByte, UINT8 Address)
{
    UINT8 Data;

	IdleI2C();				//wait for bus Idle
	StartI2C();				//Generate Start Condition
	WriteI2C(ControlByte);                  //Write Control Byte
        IdleI2C();				//wait for bus Idle
	WriteI2C(Address);			//Write start address
        IdleI2C();				//wait for bus Idle

	RestartI2C();				//Generate restart condition
	WriteI2C(ControlByte | 0x01);           //Write control byte for read
        IdleI2C();				//wait for bus Idle

	Data = getI2C();                  //read Length number of bytes
	NotAckI2C();				//Send Not Ack
	StopI2C();				//Generate Stop

        return Data;
}

void Gyro_i2c_write(UINT8 ControlByte, UINT8 LowAdd, UINT8 data)
{
	//UINT8 ErrorCode;

	IdleI2C();				//Ensure Module is Idle
	StartI2C();				//Generate Start COndition
	WriteI2C(ControlByte);			//Write Control byte
	IdleI2C();

	//ErrorCode = ACKStatus();		//Return ACK Status

	WriteI2C(LowAdd);			//Write Low Address
	IdleI2C();

	//ErrorCode = ACKStatus();		//Return ACK Status

	WriteI2C(data);				//Write Data
	IdleI2C();
	StopI2C();				//Initiate Stop Condition
}

//**************************************
//初始化MPU6050
//**************************************
UINT8 Gyro_MPU6050_Init(void)
{
    UINT8 id;

    Gyro_i2c_write(Gyro_MPU6050_Address, PWR_MGMT_1, 0x80);
    delay_ms(100);
    id = Gyro_i2c_read(Gyro_MPU6050_Address, WHO_AM_I);

    if (id == Gyro_MPU6050_ID) {
        Gyro_i2c_write(Gyro_MPU6050_Address, PWR_MGMT_1, 0x00);
        Gyro_i2c_write(Gyro_MPU6050_Address, SMPLRT_DIV, 0x07);
        Gyro_i2c_write(Gyro_MPU6050_Address, CONFIG, 0x06);
        Gyro_i2c_write(Gyro_MPU6050_Address, GYRO_CONFIG, 0x18);
        Gyro_i2c_write(Gyro_MPU6050_Address, ACCEL_CONFIG, 0x01);
    }
    else
        id = 0x00;

    return id;
}

//**************************************
//合成數據
//**************************************
// UINT8 aaa;
UINT16 Gyro_MPU6050_GetData(UINT8 REG_Addr)
{
    	UINT8 data_h,data_l,id;
        UINT16 data;

        data_h = Gyro_i2c_read(Gyro_MPU6050_Address, REG_Addr);
        data_l = Gyro_i2c_read(Gyro_MPU6050_Address, REG_Addr + 1);
#if 0
        id = Gyro_i2c_read(Gyro_MPU6050_Address, WHO_AM_I);
   if (id == Gyro_MPU6050_ID) {
       data_h = data_l = aaa++;
   }
#endif
        //合成數據
        data = (UINT16)data_h;
        data = data << 8;
        data = data + (UINT16)data_l;
	return data;

}

void vTask_Gyro_MPU6050(void *pvParameters )
{
    UINT16 acc_x, acc_y, acc_z, temp;
    UINT16 gyro_x, gyro_y, gyro_z;
    portTickType xLastWakeTime;
    t_LCD_data lcd_data;
    portBASE_TYPE xStatus;

    Gyro_MPU6050_Init();
    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, 500);

        gyro_x = Gyro_MPU6050_GetData(GYRO_XOUT_H);
        LCD_Show(0, 0, gyro_x, 1);

        gyro_y = Gyro_MPU6050_GetData(GYRO_YOUT_H);
        LCD_Show(6, 0, gyro_y, 1);

        gyro_z = Gyro_MPU6050_GetData(GYRO_ZOUT_H);
        LCD_Show(12, 0, gyro_z, 1);

    }
}
