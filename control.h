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
    float k;
    float w;
    float Ts;
    float A1,A2,B0,B2,QB0,QB1,QB2;
    float vo,vo_1,vo_2;
    float vi,vi_1,vi_2;
    float qvo,qvo_1,qvo_2;
    
    

} SOGI;






void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo,float input);
float PR_calc(PR *pr,float target,float actual);
void SOGI_init(SOGI *sg,float k,float w,float Ts);


#endif /* CONTROL_H_ */
