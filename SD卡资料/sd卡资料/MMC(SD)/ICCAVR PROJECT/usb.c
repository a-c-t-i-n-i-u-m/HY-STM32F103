#include<iom162v.h>

// 函 数 原 形 说 明
void uart1_init(void);
void uart1_send(unsigned char);
unsigned char uart1_rev(void);

//串口通信初始化设置
//1. 波特率设定
//2. 帧结构设定 
//3. 接收和发送允许控制

//******************************************
//*名称: uart1_init
//*功能: 串口1初始化
//******************************************
void uart1_init(void)
{
 // Fosc频率为16.0000Mhz
  // 波特率设定为19200bps +/-0.2%
    UBRR1H=0x00;                                 
    UBRR1L=0x33;   
 // 8位数据+无奇偶校验+1位STOP                                                             
    UCSR1C=(1<<URSEL1)|(1<<UCSZ11)|(1<<UCSZ10);
 // 允许收发,接收完成后中断
    UCSR1B|=(1<<RXCIE1)|(1<<TXEN1)|(1<<RXEN1);
}

//******************************************
//*名称: uart1_send()
//*功能: 串口1发送数据
//******************************************
void uart1_send(unsigned char data)
{
 // 检测是否可以发送,UDRE=1寄存器为空
    while ( !( UCSR1A & (1<<UDRE1)) )
           ;
    UDR1=data;
}

//******************************************
//*名称: uart1_rev()
//*功能: 串口0接收数据
//******************************************
unsigned char uart1_rev(void)
{
 // 检测是否接收完成
    while ( !(UCSR1A & (1<<RXC1)) )
          ;
 // 返回接收数据
    return UDR1;                      
}

//******************************************
//*名称: uart1_intrev()
//*功能: 串口1中断方式接收数据
//******************************************
#pragma interrupt_handler uart1_intrev:iv_USART1_RXC
void uart1_intrev(void)
{
    asm("cli");
	uart1_send(UDR1);
	asm("sei");
}