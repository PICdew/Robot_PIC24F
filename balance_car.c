//****************************************
// balance_car.c
// Test the car balance
// MCU: PIC24F
// 2014.02.08
// Regis Hsu
//****************************************
/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>

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
#include "kalman.h"
#include "math.h"
#include "balance_car.h"
#include "moter_l298n.h"
#include "uart1.h"

int cmd_sw_debug;
int cmd_sw_motor;

//******角度參數************
float Gyro_y;           //Y軸陀螺儀數據暫存
float Angle_gy;         //由角速度計算的傾斜角度
float Accel_x;          //X軸加速度值暫存
float Angle_ax;         //由加速度計算的傾斜角度
float Angle;            //小車最終傾斜角度
UINT8 value;            //角度正負極性標記
float Gyro_y;           //Y軸陀螺儀數據暫存

float Accel_x_offset;    //Regis, X軸加速度零點偏移
float Gyro_y_offset;       //Regis, 角速度軸Y零點偏移

//******PWM參數*************
int   speed_mr;         //右電機轉速
int   speed_ml;         //左電機轉速
int   PWM_R;            //右輪PWM值計算
int   PWM_L;            //左輪PWM值計算
float PWM;              //綜合PWM計算
float PWMI;             //PWM積分值

//******電機參數*************
float speed_r_l;        //電機轉速
float speed;            //電機轉速濾波
float position;         //位移
char  turn_need;
char  speed_need;

//******卡爾曼參數************
static float Q_angle=0.001;
static float Q_gyro=0.003;
static float R_angle=0.5;
static char  C_0 = 1;
static float dt=0.01;                 //0.01=10ms, dt為kalman濾波器採樣時間;
static portTickType task_dt = 10;     //task_dt為主循環執行時間

static float Q_bias = 0, Angle_err;
static float PCt_0, PCt_1, E;
static float K_0, K_1, t_0, t_1;
static float Pdot[4] ={0,0,0,0};
static float PP[2][2] = { { 1, 0 },{ 0, 1 } };

//******PID 參數************
static float Kp  = 15.0;   //PID參數
static float Kd  = 3.2;   //PID參數
static float Kpn = 0.01;  //PID參數
static float Ksp = 2.0;   //PID參數

#define CMD_ACCEL_X_OFFSET_U    41
#define CMD_ACCEL_X_OFFSET_D    42
#define CMD_GYRO_Y_OFFSET_U     43
#define CMD_GYRO_Y_OFFSET_D     44

void Balance_Car_Init(void)
{
    //******PID參數************
    Kp  = 15.0;   //PID參數
    Kd  = 3.2;   //PID參數
    Kpn = 0.01;  //PID參數
    Ksp = 2.0;   //PID參數

    //******卡爾曼參數************
    Q_angle = 0.001;
    Q_gyro = 0.003;
    R_angle = 0.5;
    C_0 = 1;

    //*****設定delta, kalman濾波器採樣時間**********
    //dt=0.006;              //0.006=6ms, dt為kalman濾波器採樣時間;
    dt=0.01;                 //0.01=10ms, dt為kalman濾波器採樣時間;
    //dt=0.05;               //0.05=50ms, dt為kalman濾波器採樣時間;
    task_dt = 10;           //task_dt為主循環執行時間

    //******校正Gyro零點偏移量************
    Accel_x_offset = -1100;    //Regis, X軸加速度零點偏移
    Gyro_y_offset = -17;       //Regis, 角速度軸Y零點偏移

    //******Reset moter counter**********
    speed_mr = 0;	//右電機轉速
    speed_ml = 0;       //左電機轉速

    //******Set the BT command **********
    cmd_sw_debug = 0;
    cmd_sw_motor = 1;

}

void BT_Cmd(void)
{
    int n,m;
    char cmd[50];

    n = Check_Rx_Buffer(UART_BT);
    if (n >= 2)
    {
        // parser the command
        cmd[0] = UART_PopFIFO(UART_BT);
        cmd[1] = UART_PopFIFO(UART_BT);
        cmd[2] = 0;
        UART_PutString(UART_BT, cmd);
        m = atoi(cmd);
        switch (m)
        {
            case CMD_DEBUG_1:
                cmd_sw_debug = 1 - cmd_sw_debug;
                sprintf(cmd, "Debug=%d\r\n", cmd_sw_debug);
                break;
            case CMD_MOTOR:
                cmd_sw_motor = 1 - cmd_sw_motor;
                sprintf(cmd, "Motor=%d\r\n", cmd_sw_motor);
                break;
            case CMD_ACCEL_X_OFFSET_U:
                Accel_x_offset += 100;
                sprintf(cmd, "Accel_x_offset=%f\r\n", Accel_x_offset);
                break;
            case CMD_ACCEL_X_OFFSET_D:
                Accel_x_offset -= 100;
                sprintf(cmd, "Accel_x_offset=%f\r\n", Accel_x_offset);
                break;
            case CMD_GYRO_Y_OFFSET_U:
                Gyro_y_offset += 2;
                sprintf(cmd, "Gyro_y_offset=%f\r\n", Gyro_y_offset);
                break;
            case CMD_GYRO_Y_OFFSET_D:
                Gyro_y_offset -= 2;
                sprintf(cmd, "Gyro_y_offset=%f\r\n", Gyro_y_offset);
                break;

            default:
                sprintf(cmd, "No Cmd=%d\r\n", m);
                break;
        }
        UART_PutString(UART_BT, cmd);

    }
}

//PPS Outputs for PIC24FJ256GA110 and PIC24FJ256GB110
#define OC1_IO		18
#define OC2_IO		19

void Motor_PWM_Init(void)
{
    // set the port as output
    TRISBbits.TRISB0 = 0;      //output, RP0/RB0
    TRISBbits.TRISB1 = 0;      //output, RP1/RB1
    TRISBbits.TRISB2 = 0;      //output, RP13/RB2
    TRISBbits.TRISB4 = 0;      //output, RP28/RB4

    // Configure Output Functions *********************
    RPOR0bits.RP0R      = OC1_IO;	// RR0	pin#25, J5#01,AN0,
    RPOR0bits.RP1R 	= OC1_IO;	// RP1	pin#24, J5#02,AN1
    RPOR6bits.RP13R	= OC2_IO;	// RP13	pin#23, J5#03,AN2
    RPOR14bits.RP28R 	= OC2_IO;	// RP28	pin#21, J5#05,AN4

    // reset OCxCON
    OC1CON1			= 0; 		// It is a good practice to clear off the control bits initially
    OC1CON2			= 0;
   // set the duty range
    OC1R			= 0;
    OC1RS			= 0xFF; 	// set PWM as 256 step
    // config OCxCON
    //	OC1CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
    OC1CON2bits.SYNCSEL         = 0x1;		// synchronized by the OC1 module
    //OC1CON1bits.OCTSEL          = 0x7;		// System Clock (Tcy) is the clock source for output Compare
    OC1CON1bits.OCTSEL          = 0x4;		// Timer1 as the clock source for output Compare
    OC1CON1bits.OCFLT           = 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC1CON2bits.OCINV           = 0;		// OCx output is not inverted
    OC1CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC

    OC2CON1			= 0; 		// It is a good practice to clear off the control bits initially
    OC2CON2			= 0;
    //OC2R			= 0x1f3f;	// Initialize Compare Register1 with 50% duty cycle
    OC2R			= 0;
    OC2RS			= 0xFF; 	//
    //OC2CON2bits.SYNCSEL	= 0x1;		// synchronized by the OC1 module
    OC2CON2bits.SYNCSEL         = 0x2;		// synchronized by the OC2 module
//    OC2CON1bits.OCTSEL          = 0x7;		// System Clock (Tcy) is the clock source for output Compare
    OC2CON1bits.OCTSEL          = 0x4;		// Timer1 as the clock source for output Compare
    OC2CON1bits.OCFLT           = 0;		// No PWM Fault condition has occurred (this bit is only used when OCM<2:0> = 111)
    OC2CON2bits.OCINV           = 0;		// OCx output is not inverted
    OC2CON1bits.OCM		= 0x6;		// Edge-aligned PWM mode on OC
}


/*
 *   Gyro i2c interface, Read/Write
 */
UINT8 Gyro_I2C_R(UINT8 Address)
{
    UINT8 Data;
    IdleI2C();                              //wait for bus Idle
    StartI2C();                             //Generate Start Condition
    WriteI2C(Gyro_MPU6050_Address);         //Write Control Byte
    IdleI2C();                              //wait for bus Idle
    WriteI2C(Address);                      //Write start address
    IdleI2C();                              //wait for bus Idle

    RestartI2C();                           //Generate restart condition
    WriteI2C(Gyro_MPU6050_Address | 0x01);  //Write control byte for read
    IdleI2C();                              //wait for bus Idle

    Data = getI2C();                        //read Length number of bytes
    NotAckI2C();                            //Send Not Ack
    StopI2C();                              //Generate Stop

    return Data;
}

void Gyro_I2C_W(UINT8 LowAdd, UINT8 data)
{
	//UINT8 ErrorCode;
	IdleI2C();				//Ensure Module is Idle
	StartI2C();				//Generate Start COndition
	WriteI2C(Gyro_MPU6050_Address);		//Write Control byte
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
void MPU6050_Init(void)
{
    Gyro_I2C_W(PWR_MGMT_1, 0x00);	//解除休眠狀態
    Gyro_I2C_W(SMPLRT_DIV, 0x07);
    Gyro_I2C_W(CONFIG, 0x06);
    Gyro_I2C_W(GYRO_CONFIG, 0x18);
    Gyro_I2C_W(ACCEL_CONFIG, 0x01);
}

//**************************************
//合成數據 Hi_byte+Lo_byte as 16bit data
//**************************************
int MPU6050_GetData(UINT8 REG_Address)
{
	UINT H;
	UINT8 L;
	H = Gyro_I2C_R(REG_Address);
	L = Gyro_I2C_R(REG_Address+1);
	return (H<<8) + L;      //合成數據
}

//*********************************************************
// 卡爾曼濾波
//*********************************************************

//Kalman濾波，處理時間約0.6ms；

void Kalman_Filter(float Accel,float Gyro)
{
	Angle += (Gyro - Q_bias) * dt; //先驗估計

	Pdot[0] = Q_angle - PP[0][1] - PP[1][0]; // Pk-先驗估計誤差協方差的微分

	Pdot[1] = -PP[1][1];
	Pdot[2] = -PP[1][1];
	Pdot[3] = Q_gyro;

	PP[0][0] += Pdot[0] * dt;   // Pk-先驗估計誤差協方差微分的積分
	PP[0][1] += Pdot[1] * dt;   // =先驗估計誤差協方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;

	Angle_err = Accel - Angle;	//zk-先驗估計

	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];

	E = R_angle + C_0 * PCt_0;

	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;

	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //後驗估計誤差協方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;

	Angle	+= K_0 * Angle_err;	 //後驗估計
	Q_bias	+= K_1 * Angle_err;	 //後驗估計
	Gyro_y   = Gyro - Q_bias;	 //輸出值(後驗估計)的微分=角速度
}



//*********************************************************
// 傾角計算（卡爾曼融合）
//*********************************************************
//角度計算，處理時間約低於1ms；
#if 0
void Angle_Calculate(void)
{
	//------加速度--------------------------
	//範圍為2g時，換算關係：16384 LSB/g
	//角度較小時，x=sinx得到角度（弧度）, deg = rad*180/3.14
	//因為x>=sinx,故乘以1.3適當放大
	Accel_x  = MPU6050_GetData(ACCEL_XOUT_H);	  //讀取X軸加速度
	Angle_ax = (Accel_x - 1100) /16384;         //去除零點偏移,計算得到角度（弧度）
	Angle_ax = Angle_ax * 1.2 * 180/3.14;           //弧度轉換為度,

        //-------角速度-------------------------
	//範圍為2000deg/s時，換算關係：16.4 LSB/(deg/s)
	Gyro_y = MPU6050_GetData(GYRO_YOUT_H);	      //靜止時角速度Y軸輸出為-30左右
	Gyro_y = -(Gyro_y + 30)/16.4;         //去除零點偏移，計算角速度值,負號為方向處理
	//Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度積分得到傾斜角度.

	//-------卡爾曼濾波融合-----------------------
	Kalman_Filter(Angle_ax, Gyro_y);       //卡爾曼濾波計算傾角

	/*//-------互補濾波-----------------------
	//補償原理是取當前傾角和加速度獲得傾角差值進行放大，然後與
        //陀螺儀角速度疊加後再積分，從而使傾角最跟蹤為加速度獲得的角度
	//0.5為放大倍數，可調節補償度；0.01為系統週期10ms
	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
}
#endif

void Angle_Calculate(void)
{
    char m[100];

	//------加速度--------------------------
	//範圍為2g時，換算關係：16384 LSB/g
	//角度較小時，x=sinx得到角度（弧度）, deg = rad*180/3.14
	//因為x>=sinx,故乘以1.3適當放大
	Accel_x  = MPU6050_GetData(ACCEL_XOUT_H);	  //讀取X軸加速度
        //sprintf(m, "%f ", Accel_x);
        //UART_PutString(UART_BT, m);

        Angle_ax = (Accel_x + Accel_x_offset) /16384;         //去除零點偏移,計算得到角度（弧度）
	Angle_ax = Angle_ax * 1.2 * RAD_TO_DEG;           //弧度轉換為度,(180/3.14)

        //-------角速度-------------------------
	//範圍為2000deg/s時，換算關係：16.4 LSB/(deg/s)
	Gyro_y = MPU6050_GetData(GYRO_YOUT_H);	      //靜止時角速度Y軸輸出為+17左右
        //sprintf(m, "%f\r\n", Gyro_y);
        //UART_PutString(UART_BT, m);

        Gyro_y = -(Gyro_y - Gyro_y_offset)/16.4;         //去除零點偏移，計算角速度值,負號為方向處理
	//Angle_gy = Angle_gy + Gyro_y*0.01;  //角速度積分得到傾斜角度.

	//-------卡爾曼濾波融合-----------------------
	Kalman_Filter(Angle_ax, Gyro_y);       //卡爾曼濾波計算傾角

	/*//-------互補濾波-----------------------
	//補償原理是取當前傾角和加速度獲得傾角差值進行放大，然後與
        //陀螺儀角速度疊加後再積分，從而使傾角最跟蹤為加速度獲得的角度
	//0.5為放大倍數，可調節補償度；0.01為系統週期10ms
	Angle = Angle + (((Angle_ax-Angle)*0.5 + Gyro_y)*0.01);*/
}


//*********************************************************
//電機轉速和位移值計算
//*********************************************************
void Position_Calculate(void)
{
	speed_r_l =(speed_mr + speed_ml)*0.5;
	speed *= 0.7;		                  //車輪速度濾波
	speed += speed_r_l*0.3;
	position += speed;	                  //積分得到位移
	position += speed_need;
	if(position<-6000) position = -6000;
	if(position> 6000) position =  6000;
}

//*********************************************************
//電機PWM值計算
//*********************************************************
void PWM_Calculate(void)
{
    int PWM_R_temp, PWM_L_temp;
    int direction;

    if (Angle < -40 || Angle > 40)               //角度過大，關閉電機
    {
        Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
        Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
    }
    else
    {
	PWM  = Kp * Angle + Kd * Gyro_y;          //PID：角速度和角度
	PWM += Kpn * position + Ksp * speed;      //PID：速度和位置
	PWM_R = PWM + turn_need;
	PWM_L = PWM - turn_need;

        PWM_R_temp = PWM_R * 200;
        PWM_L_temp = PWM_L * 200;

        direction = MOTOR_CW;
        if (PWM_R < 0)
        {
            direction = MOTOR_CCW;
            PWM_R_temp = -PWM_R_temp;
        }

        if (cmd_sw_motor)
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_R, direction, PWM_R_temp);
            Motor_PWM_Dir_Set_Raw(MOTOR_L, direction, PWM_R_temp);
        }
        else
        {
            Motor_PWM_Dir_Set_Raw(MOTOR_L, MOTOR_STOP, 0);
            Motor_PWM_Dir_Set_Raw(MOTOR_R, MOTOR_STOP, 0);
        }

        //PWM_Motor(PWM_L,PWM_R);
     }
}

/*
 * JGA25-371全金属电机(12V)、超耐磨65MM橡胶轮胎、3MM铝合金车身
 * 2014.02.08 Regis
 */
void vTask_Balance_Car_1(void *pvParameters )
{
    (void)pvParameters;     //prevent compiler worning/error
    char m[100];
    portTickType xLastWakeTime;
    //portTickType xStartTime, xEndTime;

    Balance_Car_Init();
    Motor_L298N_Init();
    MPU6050_Init();
    Motor_L298N_PWM_Init(1000); //1kHz

    xLastWakeTime = xTaskGetTickCount();
    for( ;; )
    {
        vTaskDelayUntil(&xLastWakeTime, task_dt);    // set the timer as 10ms
        BT_Cmd();
        //xStartTime = xTaskGetTickCount();
        Angle_Calculate();
        Position_Calculate();
        PWM_Calculate();
        speed_mr = speed_ml = 0;
        //xEndTime = xTaskGetTickCount();
        //xEndTime -= xStartTime;
        if (cmd_sw_debug)
        {
            //sprintf(m, "%u %f %f S:%f %f P:%d %d\r\n", xEndTime, Angle, Gyro_y, speed, position, PWM_R, PWM_L);
            sprintf(m, "%f %f %f %d\r\n", Angle, Gyro_y, speed, PWM_L);
            UART_PutString(UART_BT, m);
        }
    }
}

