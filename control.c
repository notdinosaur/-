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


void Phase_PLL(PLL *pll,float U1,float U2,float Kp,float Ki,float Ts)
{
    //参数传递
	pll.TS = Ts;
    pll.Ui1 = U1;
    pll.Ui2 = U2;
    pll.Kp = Kp;
    pll.Ki = Ki;

	// Clark 坐标转换
	pll.Ualpha = pll.Ui1;
	pll.Ubeta = pll.Ui2;

	// Park 坐标转换
	pll.Ud = pll.Ualpha * pll.coswt + pll.Ubeta * pll.sinwt;
	pll.Uq = -pll.Ualpha * pll.sinwt + pll.Ubeta * pll.coswt;

	// 锁相环环路计算
	pll.ref = pll.Uq;
	pll.fed = 0;
	pll.err = pll.ref - pll.fed;

	pll.tempvalue = pll.Ki * pll.err; // 积分计算
	pll.tempvalue += pll.Kp * (pll.err - pll.errpre);
	pll.out += pll.tempvalue; // 增量运算

	pll.errpre = pll.err;

	pll.theta += pll.out * pll.TS; // 环路输出即为w，需要对t积分

	if (pll.theta >= 2*PI)
	{
		pll.theta -= 2*PI;
	}
	if (pll.theta <= 0)
	{
		pll.theta += 2*PI;
	}

	pll.sinwt = sinf(pll.theta);
	pll.coswt = cosf(pll.theta);
}

void SOGI(void) // 广义二重积分正交变换
{

	diff = (U - integral_2) * k_sogi;
	integral_2 = integral_2 + w * (diff - integral_3) * T_sogi;

	U1 = integral_2;
	integral_3 = integral_3 + w * integral_2 * T_sogi;

	U2 = integral_3;

}
//void PI_init(PI *pi,float Kp,float Ki)
//{
//    pi->Kp=Kp;
//    pi->Ki=Ki;
//}
//float PI_calc(PI *pi,float target,float actual)
//{
//    float error=0,error_1=0,integral=0;
//    float vo=0;

//    pi->target=target;
//    pi->actual=actual;

//	error = target - actual;
//	integral += error;
//	vo = (pi->Kp*error) + (pi->Ki*integral);
//	error_1 = error;
//	return vo;//输出实际值
//}

