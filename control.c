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
    pr->A2 = (4 / pr->Ts / pr->Ts - 4 * pr->wc / pr->Ts + pr->wo * pr->wo) / temp;/*Bo,B1��B2��A1��A2ΪZ���µ����ϵ��*/


}

float PR_calc(PR *pr,float target,float actual)
{
    float error=0;
    pr->target=target;
    pr->actual=actual;
    
    error = pr->target - pr->actual;
    pr->vi = error;

    /*y[n]+A1[n-1]+A2[n-2]=B0x[n]+B1x[n-1]+B2[n-2]��Z�򴫺���ɢ���õ���ַ���*/
    pr->vo = -pr->A1 * pr->vo_1 - pr->A2 * pr->vo_2 + pr->B0 * pr->vi + pr->B1 * pr->vi_1+ pr->B2 * pr->vi_2;

    /*����*/
    pr->vo_2 = pr->vo_1;
    pr->vo_1 = pr->vo;
    pr->vi_2 = pr->vi_1;
    pr->vi_1 = pr->vi;

    return pr->vo;/*���ؿ����������ֵ*/
}


void Phase_PLL(PLL *pll,float U1,float U2,float Kp,float Ki,float Ts)
{
    //��������
	pll.TS = Ts;
    pll.Ui1 = U1;
    pll.Ui2 = U2;
    pll.Kp = Kp;
    pll.Ki = Ki;

	// Clark ����ת��
	pll.Ualpha = pll.Ui1;
	pll.Ubeta = pll.Ui2;

	// Park ����ת��
	pll.Ud = pll.Ualpha * pll.coswt + pll.Ubeta * pll.sinwt;
	pll.Uq = -pll.Ualpha * pll.sinwt + pll.Ubeta * pll.coswt;

	// ���໷��·����
	pll.ref = pll.Uq;
	pll.fed = 0;
	pll.err = pll.ref - pll.fed;

	pll.tempvalue = pll.Ki * pll.err; // ���ּ���
	pll.tempvalue += pll.Kp * (pll.err - pll.errpre);
	pll.out += pll.tempvalue; // ��������

	pll.errpre = pll.err;

	pll.theta += pll.out * pll.TS; // ��·�����Ϊw����Ҫ��t����

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

void SOGI(void) // ������ػ��������任
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
//	return vo;//���ʵ��ֵ
//}

