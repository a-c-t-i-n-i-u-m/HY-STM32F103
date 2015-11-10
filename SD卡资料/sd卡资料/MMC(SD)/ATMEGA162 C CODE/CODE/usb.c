#include<iom162v.h>

// �� �� ԭ �� ˵ ��
void uart1_init(void);
void uart1_send(unsigned char);
unsigned char uart1_rev(void);

//����ͨ�ų�ʼ������
//1. �������趨
//2. ֡�ṹ�趨 
//3. ���պͷ����������

//******************************************
//*����: uart1_init
//*����: ����1��ʼ��
//******************************************
void uart1_init(void)
{
 // FoscƵ��Ϊ16.0000Mhz
  // �������趨Ϊ19200bps +/-0.2%
    UBRR1H=0x00;                                 
    UBRR1L=0x33;   
 // 8λ����+����żУ��+1λSTOP                                                             
    UCSR1C=(1<<URSEL1)|(1<<UCSZ11)|(1<<UCSZ10);
 // �����շ�,������ɺ��ж�
    UCSR1B|=(1<<RXCIE1)|(1<<TXEN1)|(1<<RXEN1);
}

//******************************************
//*����: uart1_send()
//*����: ����1��������
//******************************************
void uart1_send(unsigned char data)
{
 // ����Ƿ���Է���,UDRE=1�Ĵ���Ϊ��
    while ( !( UCSR1A & (1<<UDRE1)) )
           ;
    UDR1=data;
}

//******************************************
//*����: uart1_rev()
//*����: ����0��������
//******************************************
unsigned char uart1_rev(void)
{
 // ����Ƿ�������
    while ( !(UCSR1A & (1<<RXC1)) )
          ;
 // ���ؽ�������
    return UDR1;                      
}

//******************************************
//*����: uart1_intrev()
//*����: ����1�жϷ�ʽ��������
//******************************************
#pragma interrupt_handler uart1_intrev:iv_USART1_RXC
void uart1_intrev(void)
{
    asm("cli");
	uart1_send(UDR1);
	asm("sei");
}