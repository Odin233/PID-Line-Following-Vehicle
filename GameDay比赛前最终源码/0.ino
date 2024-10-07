#include<Servo.h>

#define Ba A5
#define LArm 9
#define RArm 11
#define Gp 10

#define BaseIn 0
#define L_HeightIn 30
#define R_FrontIn 100
#define GripIn 0

#define Angle1 0//第一次底座旋转角度
#define Angle2 0//第二次底座旋转角度

#define HeightAdjust1 66//抓取高度的舵机角度
#define HeightAdjust2 62//释放高度的舵机角度

#define FrontAdjust1 150//抓取前后距离的舵机角度
#define FrontAdjust2 150//释放前后距离的舵机角度

#define Grip_Grasp_Angle 0//机械手抓紧的角度
#define Grip_Release_Angle 30//机械手释放的角度

Servo Base;
Servo L_high;
Servo R_front;
Servo Grip;

#include<stdio.h>
//左电机端口定义
#define MotorRpin1   8 //控制位3
#define MotorRpin2   7 //控制位4
#define MotorRpwm    6  //使能调速 ENB
#define MotorRcountA 2 //编码器A 中断号：5

 
//右电机端口定义
#define MotorLpin1   13 //控制位1
#define MotorLpin2   A4 //控制位2
#define MotorLpwm    5  //使能调速 ENA
#define MotorLcountA 3 //编码器A 中断号：3

const int er = analogRead(A1)-analogRead(A0);

void clockwise(Servo myservo1,int stop_angle1)
{
  int start_angle1=myservo1.read();
  
  for(int pos=start_angle1;pos<stop_angle1;pos=pos+1)
  {
    myservo1.write(pos);
    delay(5);
  }
}


void anticlockwise(Servo myservo2,int stop_angle2)
{
  int start_angle2=myservo2.read();
  
  for(int pos=start_angle2;pos>stop_angle2;pos=pos-1)
  {
    myservo2.write(pos);
    delay(7);
  }
}

void InPos(Servo S,int POS)
{
  S.write(POS);
}

void BaseIn233()
{
  digitalWrite(Ba,LOW);
  
  unsigned char i = 50;
  
  while (i--)
    {
    digitalWrite(Ba, HIGH);
    delayMicroseconds(500); 
    digitalWrite(Ba, LOW);
    delayMicroseconds(19500);
    }
}

void ServoIn()
{
  Base.attach(Ba);
  
  L_high.attach(LArm);
  
  R_front.attach(RArm);
  
  Grip.attach(Gp);

  InPos(Base,BaseIn);

  InPos(L_high,L_HeightIn);

  InPos(R_front,R_FrontIn);

  InPos(Grip,GripIn);
}

void BaseR1(Servo Base,int angle)
{
  clockwise(Base,angle);
}
void BaseR2(Servo Base,int angle)
{
  anticlockwise(Base,angle);
}
void HeightAD1(Servo L,int angle)
{
  clockwise(L,angle);
}
void HeightAD2(Servo L,int angle)
{
  anticlockwise(L,angle);
}
void HeightAD3(Servo L,int angle)
{
  clockwise(L,angle);
}
void frontAD1(Servo R,int angle)
{
  clockwise(R,angle);
}
void frontAD2(Servo R,int angle)
{
  anticlockwise(R,angle);
}
void frontAD3(Servo R,int angle)
{
  clockwise(R,angle);
}

void firmly_grasp(Servo G,int angle1,int angle2)
{
  Gripper_release(G,angle1);
  
  Gripper_grasp(G,angle2);
}

void Gripper_grasp(Servo G,int angle)
{
  anticlockwise(G,angle);
}

void Gripper_release(Servo G,int angle)
{
  clockwise(G,angle);
}

void Grip_and_return()

{
  BaseIn233();
  
  InPos(Grip,GripIn);

  BaseIn233();

  Gripper_release(Grip,Grip_Release_Angle);

  HeightAD1(L_high,HeightAdjust1);

  frontAD1(R_front,FrontAdjust1);
  
  firmly_grasp(Grip,Grip_Release_Angle,Grip_Grasp_Angle);

  frontAD2(R_front,R_FrontIn);

  HeightAD2(L_high,L_HeightIn);

  BaseIn233();
}

void release_and_return()

{
  BaseIn233();

  HeightAD1(L_high,HeightAdjust2);

  frontAD1(R_front,FrontAdjust2);

  Gripper_release(Grip,Grip_Release_Angle);

  delay(1000);

  Gripper_grasp(Grip,Grip_Grasp_Angle);

  frontAD2(R_front,R_FrontIn);

  HeightAD2(L_high,L_HeightIn);

  InPos(Grip,Grip_Grasp_Angle);

  BaseIn233();
}

volatile float motorL=0;//中断变量，左轮子脉冲计数
volatile float motorR=0;//中断变量，右轮子脉冲计数
float V_L=0; //左轮速度 单位cm/s
float V_R=0; //右边轮速 单位cm/s
int v1=0;  //单位cm/s
int v2=0;  //单位cm/s
float Target_V_L=20,Target_V_R=20;   //单位cm/s
int Pwm_L=0,Pwm_R=0;  //左右轮PWM
int delta;
unsigned long lastTime;
double Input, Output, Setpoint;
double errSum, lastErr;
double kp1=0.18, ki1=0.08, kd1=0.12;
double error;
unsigned long nt;
 int allError,lastError;
 
 void PID(int left_val,int right_val){
  int Error=left_val-right_val;
  allError+=Error;
  float p=kp1*Error;
  float i=ki1*allError;
  float d=kd1*(Error-lastError);
  lastError=Error;
  if(allError>=400)
  allError=400;
  if(allError<=-400)
  allError=-400;
  delta=int(p+i+d);
  if(delta>150)
  delta=150;
   if(delta<-150)
   delta=-150;
  }
    
 //PID变量
float kp=1.4,ki=0.36,kd=0.08;  //PID参数
 
 
/**************************************
 * Arduino初始化函数
 * 
 *************************************/
void setup() {
   delay(200);
   Motor_Init();//电机端口初始化
   Serial.begin(9600);//开启串口
    ServoIn();
   
}
 
 int Incremental_Pi_L(int current_speed,int target_speed){
  
  static float pwm,bias,last_bias,prev_bias;  //静态变量存在程序全周期：pwm:增量输出,bias:本次偏差,last_bias:上次偏差,prev_bais_:上上次偏差
  bias=current_speed-target_speed;    //计算本次偏差e(k)
  pwm-=(kp*(bias-last_bias)+ki*bias+kd*(bias-2*last_bias+prev_bias));   //增量式PID控制器
  prev_bias=last_bias;  //保存上上次偏差
  last_bias=bias;     //保存上一次偏差
 
 
 
  //PWM 限幅度  Arduino的PWM 最高为255  限制在250
  if(pwm<0){
    pwm=0;     
  }
  if(pwm>250){
    pwm=250;  
  }
  //Serial.println(pwm);
  return pwm;         //增量输出
 }
  
//右轮速度增量式PID控制器
int Incremental_Pi_R(float current_speed,float target_speed){

  static float pwm,bias,last_bias,prev_bias;  //静态变量存在程序全周期：pwm:增量输出,bias:本次偏差,last_bias:上次偏差,prev_bais_:上上次偏差
  bias=current_speed-target_speed;    //计算本次偏差e(k)
  pwm-=(kp*(bias-last_bias)+ki*bias+kd*(bias-2*last_bias+prev_bias));   //增量式PID控制器
  prev_bias=last_bias;  //保存上上次偏差
  last_bias=bias;     //保存上一次偏差
 
  //PWM 限幅度  Arduino的PWM 最高为255限制在250
  if(pwm<0){
    pwm=0;     
  }
  if(pwm>250){
    pwm=250;  
  }
  //Serial.println(pwm);
  return pwm;         //增量输出
 }
 

void Set_Pwm(int mode,int speed_L,int speed_R){
 
  if(mode==1){
  //前进模式
  //左电机
  digitalWrite(MotorLpin1,LOW);
  digitalWrite(MotorLpin2,HIGH);
  analogWrite(MotorLpwm,speed_L);
  
  //右电机
  digitalWrite(MotorRpin1,HIGH);
  digitalWrite(MotorRpin2,LOW);
  analogWrite(MotorRpwm,speed_R);
  
  }else if(mode==2){
  //后退模式
  //左电机
  digitalWrite(MotorLpin1,HIGH);
  digitalWrite(MotorLpin2,LOW);
  analogWrite(MotorLpwm,speed_L);
  
  //右电机
  digitalWrite(MotorRpin1,LOW);
  digitalWrite(MotorRpin2,HIGH);
  analogWrite(MotorRpwm,speed_R);
  }else if(mode==3){
  //右转模式
  //左电机
  digitalWrite(MotorLpin1,HIGH);
  digitalWrite(MotorLpin2,LOW);
  analogWrite(MotorLpwm,speed_L);
  
  //右电机
  digitalWrite(MotorRpin1,HIGH);
  digitalWrite(MotorRpin2,LOW);
  analogWrite(MotorRpwm,speed_R);
  
  }else if(mode==4){
  //左转模式
  //左电机
  digitalWrite(MotorLpin1,LOW);
  digitalWrite(MotorLpin2,HIGH);
  analogWrite(MotorLpwm,speed_L);
  
  //右电机
  digitalWrite(MotorRpin1,LOW);
  digitalWrite(MotorRpin2,HIGH);
  analogWrite(MotorRpwm,speed_R);
  
  }
}

void Motor_Init(){
  //左电机
  pinMode(MotorLpin1,OUTPUT);  //驱动芯片控制引脚
  pinMode(MotorLpin2,OUTPUT);  //驱动芯片控制引脚
  pinMode(MotorLpwm,OUTPUT);   //驱动芯片控制引脚，PWM调速
  pinMode(MotorLcountA,INPUT); //左轮编码器A引脚

  
  //右电机
  pinMode(MotorRpin1,OUTPUT);  //驱动芯片控制引脚
  pinMode(MotorRpin2,OUTPUT);  //驱动芯片控制引脚
  pinMode(MotorRpwm,OUTPUT);   //驱动芯片控制引脚，PWM调速
  pinMode(MotorRcountA,INPUT); //右轮编码器A引脚

 
  //驱动芯片控制引脚全部拉低
  digitalWrite(MotorLpin1,LOW); //左电机
  digitalWrite(MotorLpin2,LOW);
  digitalWrite(MotorLpwm,LOW);
  digitalWrite(MotorRpin1,LOW); //右电机
  digitalWrite(MotorRpin2,LOW);
  digitalWrite(MotorRpwm,LOW);
}
 
  void Read_Moto_V(){
  unsigned long nowtime=0;
  motorL=0;
  motorR=0;
  nowtime=millis()+60;//读50毫秒
  attachInterrupt(digitalPinToInterrupt(MotorLcountA),Read_Moto_L,RISING);//左轮脉冲开中断计数
  attachInterrupt(digitalPinToInterrupt(MotorRcountA),Read_Moto_R,RISING);//右轮脉冲开中断计数
  while(millis()<nowtime); //达到50毫秒关闭中断
  detachInterrupt(digitalPinToInterrupt(MotorLcountA));//左轮脉冲关中断计数
  detachInterrupt(digitalPinToInterrupt(MotorRcountA));//右轮脉冲关中断计数
  V_L=1200*motorL/455;   //单位cm/s
  V_R=1200*motorR/455;   //单位cm/s
  v1=V_L;
  v2=V_R;
}
 void Read_Moto_L(){
  motorL++;
}
  unsigned long et;
 void Read_Moto_R(){
  motorR++;
}
void Turn(int a,double b){
  if(a==1)
  {Set_Pwm(3,82 ,150);//右转
  delay(520*b/90.0);}
  if(a==2)
 {Set_Pwm(4,150 ,82);
 delay(520*b/90.0);
 }
  }
  /*int timeup(){
     nt=millis();
     delay(10);
  et=millis()-nt;
 
  return et;
    }*/
static int state=0; 

void loop() {
  Read_Moto_V();
  int r,ri;
  int m;

int l,li;
li=0;
ri=0;
li=digitalRead(12);
ri=digitalRead(4);
  l= analogRead(A1);//读l
  r=analogRead(A0);//读r
  m=analogRead(A2);
PID(l,r);
int vr=0;
int vl=0;
if(state>=0)
{vr=Incremental_Pi_R(V_R,170-delta);
vl=Incremental_Pi_L(V_L,170+delta);
Set_Pwm(1,vr ,vl);}
/*if((li==1)||(ri==1))
{if(li==1)
   vl=0;
 if(ri==1)
   vr=0;
 Set_Pwm(1,vr ,vl);}*/  
 if(state==0)
 {if((li==1)&&(ri==1))
   {Set_Pwm(1,0 ,0);
   
   state=-2;
   }}
   //if((li==0)&&(ri==0))
   //state=0;
  if(state==-2)//抓取
  {
    Grip_and_return();
    state=1;
    }
 if(state==2)
 {
if((li==1)&&(ri==1))
   {Set_Pwm(2,0 ,0);
  
state=-1;

   }

 }
  if(state==1)
  {Turn(1,93);//右转为1，左转为2
 Set_Pwm(1,100 ,100);
 delay(200);
state=2;
 }
if(state==-1)//放置
{   state=-3;
    release_and_return();
    }
    Serial.println(li);  //直接用串口绘图画出速度曲线
   //设置左右轮速度
}
