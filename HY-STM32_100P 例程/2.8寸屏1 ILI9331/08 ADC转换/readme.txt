1���Ҫ��
����ADC�ĵ�11ͨ���Կ���������ĵ�ѹֵ��ADת������������ת��ģʽ��ת�����ͨ��DMAͨ
��1��ȡ��ADCת���Ľ����ÿ���1�����򴮿ڷ���һ�Ρ�

2 Ӳ����·���													
�ڿ�������ͨ��I/O��PC.01��XS10��������PC.01ӳ�䵽ADC��11ͨ��������ʵ������ADC_IN11
�������ѹ��ADת����

3����������
	�����������Ҫ�����������Ҫ������
	(1)	����GPIO�ڣ���PC.01����ΪADC�ĵ�11����ͨ����������GPIO��PA.09��PA.10������Ϊ�������������
	(2)	����ADC����ADC_IN11����Ϊ����ת��ģʽ��
	(3)	����DMAͨ��1����ADC_IN14����ת���Ľ����
	(4)	���ô��ڼ���ط��͹��ܣ�
	(5)	ÿ��1S�򴮿����ADת�������

4 ���й���
(1)	ʹ��Keil uVision3 ͨ��JLINK���������ӿ����壬ʹ�ô����ߣ�����ʵ���
�ϵ�UART1��XS5)��PC���Ĵ��ڣ���ʵ������Ŀ¼�µ�STM32-FD-ADC.Uv2���̣��������ӹ��̣�
(2)	��PC��������windows�Դ��ĳ����ն˴���ͨ�ų��򣨲�����115200��1λֹͣλ����У��λ����Ӳ����
���ƣ�������ʹ����������ͨ�ų���
(3)	���MDK ��Debug�˵������Start/Stop Debug Session��
(4)	��ת��λ��RV1�����Կ������������ֵ���ϱ仯��������ʾ���������ʾ��

usart1 print AD_value --------------------------
The current AD value = 0x0425
The current AD value = 0x0423
The current AD value = 0x0421
The current AD value = 0x0422
The current AD value = 0x0420
The current AD value = 0x0416
The current AD value = 0x03B6
The current AD value = 0x0841
The current AD value = 0x08C3
The current AD value = 0x08C0
The current AD value = 0x08BE
The current AD value = 0x09E9
The current AD value = 0x0A12
The current AD value = 0x0ACA
The current AD value = 0x0B0D
The current AD value = 0x0B10
The current AD value = 0x0B0E
....
....

