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
    

} SOGI;






void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo,float input);
float PR_calc(PR *pr,float target,float actual);


#endif /* CONTROL_H_ */
