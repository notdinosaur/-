#include "control.h"
#include "math.h"
/**/


void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo)
{
    float temp = 0;

    pr->Ts=Ts;
    pr->Kp=Kp;
    pr->Kr=Kr;
    pr->wc=wc;
    pr->wo=wo;

    temp = 4 / pr->Ts / pr->Ts + 4 * pr->wc / pr->Ts + pr->wo * pr->wo;

    pr->B0 = (4 * pr->Kp / pr->Ts / pr->Ts + 4 * pr->wc * (pr->Kp + pr->Kr) / pr->Ts+ pr->Kp * pr->wo * pr->wo) / temp;
    pr->B1 = (-8 * pr->Kp / pr->Ts / pr->Ts >Ts / pr->Ts + 2 * pr->Kp * pr->wo * pr->wo) / temp;
    pr->B2 = (4 * pr->Kp / - 4 * pr->wc / pr->Ts * (pr->Kp + pr->Kr)+ pr->Kp * pr->wo * pr->wo) / temp;
    pr->A1 = (-8 / pr->Ts / pr->Ts + 2 * pr->wo * pr->wo) / temp;
    pr->A2 = (4 / pr->Ts / pr->Ts - 4 * pr->wc / pr->Ts + pr->wo * pr->wo) / temp;/*Bo,B1，B2，A1，A2为Z域下的相关系数*/


}

float PR_calc(PR *pr,float target,float actual)
{
    float error=0;
    pr->target=target;
    pr->actual=actual;
    
    error = pr->target - pr->actual;
    pr->vi = error;

    /*y[n]+A1[n-1]+A2[n-2]=B0x[n]+B1x[n-1]+B2[n-2]由Z域传函离散化得到差分方程*/
    pr->vo = -pr->A1 * pr->vo_1 - pr->A2 * pr->vo_2 + pr->B0 * pr->vi + pr->B1 * pr->vi_1+ pr->B2 * pr->vi_2;

    /*误差传递*/
    pr->vo_2 = pr->vo_1;
    pr->vo_1 = pr->vo;
    pr->vi_2 = pr->vi_1;
    pr->vi_1 = pr->vi;

    return pr->vo;/*返回控制器的输出值*/
}



void SOGI(void) // 广义二重积分正交变换
{

	diff = (U - integral_2) * k_sogi;
	integral_2 = integral_2 + w * (diff - integral_3) * T_sogi;

	U1 = integral_2;
	integral_3 = integral_3 + w * integral_2 * T_sogi;

	U2 = integral_3;

}


