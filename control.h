#ifndef CONTROL_H_
#define CONTROL_H_


typedef struct
{
    float Kp;
    float Kr;
    float wo;
    float wc;
    float Ts;
    float A0, A1, A2, B0, B1, B2;
    float vo, vo_1, vo_2;
    float vi, vi_1, vi_2;
    float target,actual;
} PR;

typedef struct
{
    float Kp;
    float Ki;
    float vo;
    float target,actual;
} PI;

typedef struct
{
    float Ui1,Ui2;
    float Ualpha, Ubeta;
    float Ud, Uq;
    float sinwt, coswt;
    float ref, fed, err, errpre, tempvalue, out;
    float Kp,Ki;
    float TS;
} PLL;

typedef struct 
{
    
} coordinate_transformation;


void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo,float input);
float PR_calc(PR *pr,float target,float actual);
void Phase_PLL(float U1,float U2,float Kp,float Ki,float Ts);
//void PI_init(PI *pi,float Kp,float Ki);
//float PI_calc(PI *pi,float target,float actual);

#endif /* CONTROL_H_ */
