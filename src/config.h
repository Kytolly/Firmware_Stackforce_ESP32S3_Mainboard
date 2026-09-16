#ifndef _CONFIG_H_
#define _CONFIG_H_

#define CAN_ID  __int_reg[0]

extern float __float_reg[];
extern int __int_reg[];

typedef struct 
{
    float pos;
    float vel;
    float kp;
    float kd;
    float tor;
    float posMin = -12.5f;
    float posMax = 12.5f;
    float velMin = -65.0f;
    float velMax = 65.0f;
    float kpMin = 0.0f;
    float kpMax = 500.0f;
    float kdMin = 0.0f;
    float kdMax = 5.0f;
    float torMin = -18.0f;
    float torMax = 18.0f;
}MIT;

typedef struct{
  float motor1taget;
  float motor2taget;
  float motortvel;
  float motorpos;
}motor_data;

typedef struct{
  MIT MITData;
}controller;

typedef struct{

}motorconfig;

#endif