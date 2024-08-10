#ifndef svpwm_h
#define svpwm_h


typedef struct 
{
    float U��; //����ĵ�ѹ
    float U��; //�����ѹ
    float U1; //���������ж�
    float U2;
    float U3;
    float T0; //�˸�ʸ��������ʱ��
    float T1;
    float T2;
    float T3;
    float T4;
    float T5;
    float T6;
    float T7;
    float Ts; //��������
    float Udc; //ֱ��ĸ�ߵ�ѹ
    float ARR;
    float ccr1;
    float ccr2;
    float ccr3;
    int sector; //����

} SV;

void SvpwmControl(SV *sv,float U��,float U��,float Ts,float ARR,float Udc);
void SVPWM(void);







#endif /*svpwm_h*/

