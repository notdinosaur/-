#include "svpwm.h"
#include "math.h"

#define SQRT3_2 0.866f
#define SQRT3 1.732f


void SvpwmControl(SV *sv,float U��,float U��,float Ts,float ARR,float Udc)
{
	int a=0,b=0,c=0,N=0;
	float Ta,Tb,Tc;	
    float sin,cos;
	float k = SQRT3/sv->Udc;
    float Length = sqrt(U��*U��+U��*U��);
	
    sv->U�� = U��;
    sv->U�� = U��;
    sv->ARR = ARR;
    sv->Ts = Ts;
    sv->Udc = Udc;

    //��ֹ������
    if(Length >= 0.5774f*sv->Udc)
	{
		sin=sv->U��/Length;
		cos=sv->U��/Length;
		sv->U��=0.5774f*Udc*cos;
		sv->U��=0.5774f*Udc*sin;
	}
	
	//�����ж�
	sv->U1 = sv->U��;
    sv->U2 = sv->U�� * SQRT3_2 - sv->U�� / 2.0f;
	sv->U3 = -sv->U�� * SQRT3_2 - sv->U�� / 2.0f;

    if(sv->U1 > 0)  a = 1; else a = 0;
    if(sv->U2 > 0)  b = 1; else b = 0;
    if(sv->U3 > 0)  c = 1; else c = 0;

    N = 4*c + 2*b + a;

    switch(N)
    {
        case 1: sv->sector = 2; break;
        case 2: sv->sector = 6; break;
        case 3: sv->sector = 1; break;
        case 4: sv->sector = 4; break;
        case 5: sv->sector = 3; break;
        case 6: sv->sector = 5; break;

    }
		
	//ʸ������ʱ�����
	switch(sv->sector)
    {
        case 1:
            sv->T4 = k*sv->U2;
            sv->T6 = k*sv->U1;
            sv->T0 = sv->T7 = (1 - sv->T4 - sv->T6)/2.0f;
            break;
        case 2:
            sv->T2 = -k*sv->U2;
            sv->T6 = -k*sv->U3;
            sv->T0 = sv->T7 = (1 - sv->T2 - sv->T6)/2.0f;
            break;
        case 3:
            sv->T2 = k*sv->U1;
            sv->T3 = k*sv->U3;
            sv->T0 = sv->T7 = (1 - sv->T2 - sv->T3)/2.0f;
            break;
        case 4:
            sv->T1 = -k*sv->U1;
            sv->T3 = -k*sv->U2;
            sv->T0 = sv->T7 = (sv->Ts - sv->T1 - sv->T3)/2.0f;
            break;
        case 5:
            sv->T1 = k*sv->U3;
            sv->T5 = k*sv->U2;
            sv->T0 = sv->T7 = (sv->Ts - sv->T1 - sv->T5)/2.0f;
            break;
        case 6:
            sv->T4 = -k*sv->U3;
            sv->T5 = -k*sv->U1;
            sv->T0 = sv->T7 = (sv->Ts - sv->T4 - sv->T5)/2.0f;
            break;
				default:
						break;

    }
		
		//�߶�ʽ����
	switch(sv->sector)
    {
        case 1:
            Ta = sv->T4 + sv->T6 + sv->T7;
            Tb = sv->T6 + sv->T7;
            Tc = sv->T7;
            break;
        case 2:
            Ta = sv->T6 + sv->T7;
            Tb = sv->T2 + sv->T6 + sv->T7;
            Tc = sv->T7;
            break;
        case 3:
            Ta = sv->T7;
            Tb = sv->T2 + sv->T3 + sv->T7;
            Tc = sv->T3 + sv->T7;
            break;
        case 4:
            Ta = sv->T7;
            Tb = sv->T3 + sv->T7;
            Tc = sv->T1 + sv->T3 +sv->T7;
            break;
        case 5:
            Ta = sv->T5 + sv->T7;
            Tb = sv->T7;
            Tc = sv->T1 + sv->T5 +sv->T7;      
            break;
        case 6:
            Ta = sv->T4 + sv->T5 + sv->T7;
            Tb = sv->T7;
            Tc = sv->T5 +sv->T7;    
            break;                  
    }



    sv->ccr1 = Ta * sv->ARR;
    sv->ccr2 = Tb * sv->ARR;
    sv->ccr3 = Tc * sv->ARR;
		



}
/*****************************************************
 * ��ȡ����
 * 
 * 
******************************************************/
int GetSVPWMSector(SV *sv)
{
    return sv->sector;
}



