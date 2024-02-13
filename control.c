#include "control.h"
#include "math.h"
/**/


void PR_init(PR *pr,float Kp,float Kr,float Ts,float wc, float wo)//Wc调节带宽，一般为0.628f(2*pi*0.1)
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

    /*y[n]+A1y[n-1]+A2y[n-2]=B0x[n]+B1x[n-1]+B2x[n-2]由Z域传函离散化得到差分方程*/
    pr->vo = -pr->A1 * pr->vo_1 - pr->A2 * pr->vo_2 + pr->B0 * pr->vi + pr->B1 * pr->vi_1+ pr->B2 * pr->vi_2;

    /*误差传递*/
    pr->vo_2 = pr->vo_1;
    pr->vo_1 = pr->vo;
    pr->vi_2 = pr->vi_1;
    pr->vi_1 = pr->vi;

    return pr->vo;/*返回控制器的输出值*/
}



void SOGI_init(SOGI *sg,float k,float w,float Ts) // 基于广义二重积分的正交变换
{
     float x = 0;
     float y = 0;

     x = 2 * sg->k * sg->w *sg->Ts;
     y = sg->w * sg->w * sg->Ts * sg->Ts;


     sg->B0 = x/(x+y+4);
     sg->B1 = -1*sg->B0;
     sg->A1 = 2*(4-y);
     sg->A2 = x-y-4/(x+y+4);
     sg->QB0 = k*y/(x+y+4);
     sg->QB1 = 2*sg->QB0;
     sg->QB2 = sg->QB0;
	


}

float SOGI_D(SOGI *sg，float v)
{

     sg->vi = v;
	
     /*离散化所得差分方程*/
     sg->vo = sg->A1 * sg->vo_1 + sg->A2 * sg->vo_2 + sg->B0 * sg->vi + sg->B2 * sg->vi_2;

     /*误差更迭*/
     sg->vo_2 = sg->vo_1;
     sg->vo_1 = sg->vo;
     sg->vi_2 = sg->vi;


     return sg->vo; /*返回与输入相同的D轴值*/



}
float SOGI_Q(SOGI *sg,float qv)
{

     sg->vi = qv;
	
     /*离散化所得差分方程*/
     sg->qvo = sg->A1 * sg->qvo_1 + sg->A2 * sg->qvo_2 + sg->QB0 * sg->vi + sg->QB1 * sg->vi_1 + sg->QB2 * sg->vi_2;

     /*误差更迭*/
     sg->qvo_2 = sg->qvo_1;
     sg->qvo_1 = sg->qvo;
     sg->qvi_2 = sg->qvi_1;
     sg->qvi_1 = sg->qvi;


     return sg->qvo; /*返回滞后90°的Q轴值*/




}


