#ifndef svpwm_h
#define svpwm_h


typedef struct 
{
    float Uα; //α轴的电压
    float Uβ; //β轴电压
    float U1; //用于扇区判断
    float U2;
    float U3;
    float T0; //八个矢量的作用时间
    float T1;
    float T2;
    float T3;
    float T4;
    float T5;
    float T6;
    float T7;
    float Ts; //开关周期
    float Udc; //直流母线电压
    float ARR;
    float ccr1;
    float ccr2;
    float ccr3;
    int sector; //扇区

} SV;

void SvpwmControl(SV *sv,float Uα,float Uβ,float Ts,float ARR,float Udc);
void SVPWM(void);







#endif /*svpwm_h*/

