			   /******************** (C) COPYRIGHT 2009 HY嵌入式开发工作室 ********************
* Description        : 演示将一段字符串写入M25P16的1页中，然后读出并通过USART1传送出去。
                       字符串：SPI M25P16 Example: This is SPI DEMO, 终端上出现这一行字，说明M25P16的读写正常 
                       
    定义：	
	USART1
	TXD1----- PA9-US1-TX
	RXD1----- PA10-US1-RX	  速率：115200,n,8,1 

	SPI2 
	NSS---PB12-SPI2-NSS
    MISO--PB14-SPI2-MISO
	MOSI--PB15-SPI2-MOSI
	SCK---PB13-SPI2-SCK

	
*********************************************************************/