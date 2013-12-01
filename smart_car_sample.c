//MCU：Mega16；晶振：8MHz；
//PWM:4KHz;滤波器频率:100Hz;系统频率：100Hz；10ms;
//赵国霖：10.05.10；
#include <iom16v.h>
#include <macros.h>
#include <math.h>

//#define checkbit(var,bit)     (var&(0x01<<(bit)))     /*定义查询位函数*/
//#define setbit(var,bit)     (var|=(0x01<<(bit)))     /*定义置位函数*/
//#define clrbit(var,bit)     (var&=(~(0x01<<(bit)))) /*定义清零位函数*/


//-------------------------------------------------------
//输出端口初始化
void PORT_initial(void)
{
    DDRA=0B00000000;
    PINA=0X00;
    PORTA=0X00;

    DDRB=0B00000000;
    PINB=0X00;
    PORTB=0X00;

    DDRC=0B00010000;
    PINC=0X00;
    PORTC=0X00;

    DDRD=0B11110010;
    PIND=0X00;
    PORTD=0X00;
}


//-------------------------------------------------------
//定时器1初始化
void T1_initial(void)
{
    TCCR1A|=(1<<COM1A1)|(1<<WGM10)|(1<<COM1B1);               //T1：8位快速PWM模式、匹配时清0，TOP时置位
    TCCR1B|=(1<<WGM12)|(1<<CS11);           //PWM：8分频：8M/8/256=4KHz;
}


//-------------------------------------------------------
//定时器2初始化
void T2_initial(void)           //T2：计数至OCR2时产生中断
{
    OCR2=0X4E;          //T2：计数20ms(0X9C)10ms(0X4E)时产生中断;
    TIMSK|=(1<<OCIE2);
    TCCR2|=(1<<WGM21)|(1<<CS22)|(1<<CS21)|(1<<CS20);            //CTC模式，1024分频
}


//-------------------------------------------------------
//外部中断初始化
void INT_initial(void)
{
    MCUCR|=(1<<ISC01)|(1<<ISC00)|(1<<ISC11)|(1<<ISC10);         //INT0、INT1上升沿有效
    GICR|=(1<<INT0)|(1<<INT1);          //外部中断使能
}


//-------------------------------------------------------
//串口初始化；
void USART_initial( void )
{
    UBRRH = 0X00;
    UBRRL = 51;     //f=8MHz;设置波特率:9600:51;19200:25;
    UCSRB = (1<<RXEN)|(1<<TXEN);        //接收器与发送器使能；
    UCSRC = (1<<URSEL)|(1<<UCSZ0)|(1<<UCSZ1);     //设置帧格式: 8 个数据位, 1 个停止位；

    UCSRB|=(1<<RXCIE);        //USART接收中断使能
}


//-------------------------------------------------------
//串口发送数据；
void USART_Transmit( unsigned char data )
{
    while ( !( UCSRA & (1<<UDRE)));       //等待发送缓冲器为空；
    UDR = data;     //将数据放入缓冲器，发送数据；
}


//-------------------------------------------------------
//串口接收数据中断，确定数据输出的状态；
#pragma interrupt_handler USART_Receive_Int:12
static char USART_State;
void USART_Receive_Int(void)
{
    USART_State=UDR;//USART_Receive();
}


//-------------------------------------------------------
//计算LH侧轮速：INT0中断；
//-------------------------------------------------------
static int speed_real_LH;
//-------------------------------------------------------
#pragma interrupt_handler SPEEDLHINT_fun:2
void SPEEDLHINT_fun(void)
{
    if (0==(PINB&BIT(0)))
    {
        speed_real_LH-=1;
    }
    else
    {
        speed_real_LH+=1;
    }
}


//-------------------------------------------------------
//计算RH侧轮速，：INT1中断；
//同时将轮速信号统一成前进方向了；
//-------------------------------------------------------
static int speed_real_RH;
//-------------------------------------------------------
#pragma interrupt_handler SPEEDRHINT_fun:3
void SPEEDRHINT_fun(void)
{
    if (0==(PINB&BIT(1)))
    {
        speed_real_RH+=1;
    }
    else
    {
        speed_real_RH-=1;
    }
}


//-------------------------------------------------------
//ADport采样：10位，采样基准电压Aref
//-------------------------------------------------------
static int AD_data;
//-------------------------------------------------------
int ADport(unsigned char port)
{
    ADMUX=port;
    ADCSRA|=(1<<ADEN)|(1<<ADSC)|(1<<ADPS1)|(1<<ADPS0);          //采样频率为8分频；
    while(!(ADCSRA&(BIT(ADIF))));
    AD_data=ADCL;
    AD_data+=ADCH*256;
    AD_data-=512;
    return (AD_data);
}


//*
//-------------------------------------------------------
//Kalman滤波，8MHz的处理时间约1.8ms；
//-------------------------------------------------------
static float angle, angle_dot;      //外部需要引用的变量
//-------------------------------------------------------
static const float Q_angle=0.001, Q_gyro=0.003, R_angle=0.5, dt=0.01;
            //注意：dt的取值为kalman滤波器采样时间;
static float P[2][2] = {
                            { 1, 0 },
                            { 0, 1 }
                        };

static float Pdot[4] ={0,0,0,0};

static const char C_0 = 1;

static float q_bias, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;
//-------------------------------------------------------
void Kalman_Filter(float angle_m,float gyro_m)          //gyro_m:gyro_measure
{
    angle+=(gyro_m-q_bias) * dt;

    Pdot[0]=Q_angle - P[0][1] - P[1][0];
    Pdot[1]=- P[1][1];
    Pdot[2]=- P[1][1];
    Pdot[3]=Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;


    angle_err = angle_m - angle;


    PCt_0 = C_0 * P[0][0];
    PCt_1 = C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;

    K_0 = PCt_0 / E;
    K_1 = PCt_1 / E;

    t_0 = PCt_0;
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;


    angle   += K_0 * angle_err;
    q_bias  += K_1 * angle_err;
    angle_dot = gyro_m-q_bias;
}
//*/

/*
//-------------------------------------------------------
//互补滤波
//-------------------------------------------------------
static float angle,angle_dot;       //外部需要引用的变量
//-------------------------------------------------------
static float bias_cf;
static const float dt=0.01;
//-------------------------------------------------------
void complement_filter(float angle_m_cf,float gyro_m_cf)
{
    bias_cf*=0.998;         //陀螺仪零飘低通滤波；500次均值；
    bias_cf+=gyro_m_cf*0.002;
    angle_dot=gyro_m_cf-bias_cf;
    angle=(angle+angle_dot*dt)*0.90+angle_m_cf*0.05;
    //加速度低通滤波；20次均值；按100次每秒计算，低通5Hz；
}
*/




//-------------------------------------------------------
//AD采样；
//以角度表示；
//加速度计：1.2V=1g=90°；满量程：1.3V~3.7V；
//陀螺仪：0.5V~4.5V=-80°~+80°；满量程5V=200°=256=200°；
//-------------------------------------------------------
static float gyro,acceler;
//-------------------------------------------------------
void AD_calculate(void)
{

    acceler=ADport(2)+28;           //角度校正
    gyro=ADport(3);

    acceler*=0.004069;      //系数换算：2.5/(1.2*512);
    acceler=asin(acceler);
    gyro*=0.00341;          //角速度系数：(3.14/180)* 100/512=0.01364；

    Kalman_Filter(acceler,gyro);
    //complement_filter(acceler,gyro);
}



//-------------------------------------------------------
//PWM输出
//-------------------------------------------------------
void PWM_output (int PWM_LH,int PWM_RH)
{
    if (PWM_LH<0)
    {
        PORTD|=BIT(6);
        PWM_LH*=-1;
    }
    else
    {
        PORTD&=~BIT(6);
    }

    if (PWM_LH>252)
    {
        PWM_LH=252;
    }


    if (PWM_RH<0)
    {
        PORTD|=BIT(7);
        PWM_RH*=-1;
    }
    else
    {
        PORTD&=~BIT(7);
    }

    if (PWM_RH>252)
    {
        PWM_RH=252;
    }


    OCR1AH=0;
    OCR1AL=PWM_LH;          //OC1A输出；

    OCR1BH=0;
    OCR1BL=PWM_RH;          //OC1B输出；

}




//-------------------------------------------------------
//计算PWM输出值
//车辆直径：76mm; 12*64pulse/rev; 1m=3216pulses;
//-------------------------------------------------------
//static int speed_diff,speed_diff_all,speed_diff_adjust;
//static float K_speed_P,K_speed_I;
static float K_voltage,K_angle,K_angle_dot,K_position,K_position_dot;
static float K_angle_AD,K_angle_dot_AD,K_position_AD,K_position_dot_AD;
static float position,position_dot;
static float position_dot_filter;
static float PWM;
static int speed_output_LH,speed_output_RH;
static int Turn_Need,Speed_Need;
//-------------------------------------------------------
void PWM_calculate(void)
{
    if ( 0==(~PINA&BIT(1)) )    //左转
    {
        Turn_Need=-40;
    }
    else if ( 0==(~PINB&BIT(2)) )   //右转
    {
        Turn_Need=40;
    }
    else    //不转
    {
        Turn_Need=0;
    }

    if ( 0==(~PINC&BIT(0)) )    //前进
    {
        Speed_Need=-2;
    }
    else if ( 0==(~PINC&BIT(1)) )   //后退
    {
        Speed_Need=2;
    }
    else    //不动
    {
        Speed_Need=0;
    }

    K_angle_AD=ADport(4)*0.007;
    K_angle_dot_AD=ADport(5)*0.007;
    K_position_AD=ADport(6)*0.007;
    K_position_dot_AD=ADport(7)*0.007;

    position_dot=(speed_real_LH+speed_real_RH)*0.5;

    position_dot_filter*=0.85;      //车轮速度滤波
    position_dot_filter+=position_dot*0.15;

    position+=position_dot_filter;
    //position+=position_dot;
    position+=Speed_Need;

    if (position<-768)       //防止位置误差过大导致的不稳定
    {
        position=-768;
    }
    else if  (position>768)
    {
        position=768;
    }

    PWM = K_angle*angle *K_angle_AD + K_angle_dot*angle_dot *K_angle_dot_AD +
                    K_position*position *K_position_AD + K_position_dot*position_dot_filter *K_position_dot_AD;


    speed_output_RH = PWM - Turn_Need;
    speed_output_LH = - PWM - Turn_Need ;

    /*
    speed_diff=speed_real_RH-speed_real_LH;         //左右轮速差PI控制；
    speed_diff_all+=speed_diff;
    speed_diff_adjust=(K_speed_P*speed_diff+K_speed_I*speed_diff_all)/2;
    */

    PWM_output (speed_output_LH,speed_output_RH);
}

//-------------------------------------------------------
//定时器2中断处理
//-------------------------------------------------------
static unsigned char temp;
//-------------------------------------------------------
#pragma interrupt_handler T2INT_fun:4
void T2INT_fun(void)
{
    AD_calculate();

    PWM_calculate();

    if(temp>=4)          //10ms即中断；每秒计算：100/4=25次；
    {
        if (USART_State==0X30)      //ASCII码：0X30代表字符'0'
        {
            USART_Transmit(angle*57.3+128);
            USART_Transmit(angle_dot*57.3+128);
            USART_Transmit(128);
        }
        else if(USART_State==0X31)      //ASCII码：0X30代表字符'1'
        {
            USART_Transmit(speed_output_LH+128);
            USART_Transmit(speed_output_RH+128);
            USART_Transmit(128);
        }
        else if(USART_State==0X32)      //ASCII码：0X30代表字符'2'
        {
            USART_Transmit(speed_real_LH+128);
            USART_Transmit(speed_real_RH+128);
            USART_Transmit(128);
        }
        else if(USART_State==0X33)      //ASCII码：0X30代表字符'3'
        {
            USART_Transmit(K_angle+128);
            USART_Transmit(K_angle_dot+128);
            USART_Transmit(K_position_dot+128);
        }
        temp=0;
    }
    speed_real_LH=0;
    speed_real_RH=0;
    temp+=1;
}



//-------------------------------------------------------
int i,j;
//-------------------------------------------------------
void main(void)
{
    PORT_initial();
    T2_initial();
    INT_initial();
    USART_initial ();

    SEI();

    K_position=0.8 * 0.209;     //换算系数：(256/10) * (2*pi/(64*12))=0.20944；//256/10:电压换算至PWM，256对应10V；
    K_angle=34 * 25.6;      //换算系数：256/10 =25.6；
    K_position_dot=1.09 * 20.9;     //换算系数：(256/10) * (25*2*pi/(64*12))=20.944；
    K_angle_dot=2 * 25.6;       //换算系数：256/10 =25.6；

    for (i=1;i<=500;i++)     //延时启动PWM，等待卡尔曼滤波器稳定
    {
        for (j=1;j<=300;j++);;

    }
    T1_initial();

    while(1)
    {
        ;
    }
}


/*
建模、控制算法及PD调节的取值参考：
http://www.mathworks.com/matlabcentral/fileexchange/19147-nxtway-gs-self-balancing-two-wheeled-robot-controller-design

硬件及软件参考如下网站资料；
http://www.circuitcellar.com/avr2006/winners/de/at3329.htm
http://tlb.org/scooter.html
http://www.geology.smu.edu/~dpa-www/robo/nbot/


matlab（matlab需要2008版，旧的6.5版打不开）里打开NXT.mdl，输入NXT.m.txt内容回车，K_f的四个值就是所需的PD调节比例；详细参考NXT的文档；
主控制板说明：
1：新布置的，因为修改了一些不合理的地方，和我使用的不一样，主控制板上RS232硬件部分还未调试过，所以使用时需要注意；
2：可调电位器用来调试修改PD参数；
3：另外几个接口可以用来改成遥控输入，比例遥控可以用AD采样口实现，但需要增加滤波；
4：车轮速度我进行了低通滤波，否则可能会发抖；

马达控制板说明：
1：同样重新修订，PCB最好自己优化并检查；
2：马达控制板最好是使用IR2184，因为直接兼容5V电平，IR2111需要使用12V输入，我直接在单片机输出口接12V上拉电阻，也可以使用，但不推荐；
上位机只是简单的查看滤波效果和车速是否正确取得，VB编写的，大家可做完善；

zl
2010.06.16

 *
 */