1设计要求
利用ADC的第11通道对开发板输入的电压值作AD转换，采用连续转换模式，转换结果通过DMA通
道1读取。ADC转换的结果，每间隔1秒钟向串口发送一次。

2 硬件电路设计													
在开发板上通用I/O口PC.01与XS10相连，将PC.01映射到ADC第11通道，即可实现利用ADC_IN11
对输入电压作AD转换。

3软件程序设计
	根据设计任务要求，软件程序主要包括：
	(1)	配置GPIO口，将PC.01配置为ADC的第11采样通道；将配置GPIO中PA.09和PA.10根引脚为串口输入输出。
	(2)	设置ADC，将ADC_IN11设置为连续转换模式；
	(3)	配置DMA通道1用于ADC_IN14传输转换的结果；
	(4)	配置串口及相关发送功能；
	(5)	每隔1S向串口输出AD转换结果。

4 运行过程
(1)	使用Keil uVision3 通过JLINK仿真器连接开发板，使用串口线，连接实验板
上的UART1（XS5)和PC机的串口，打开实验例程目录下的STM32-FD-ADC.Uv2例程，编译链接工程；
(2)	在PC机上运行windows自带的超级终端串口通信程序（波特率115200、1位停止位、无校验位、无硬件流
控制）；或者使用其它串口通信程序；
(3)	点击MDK 的Debug菜单，点击Start/Stop Debug Session；
(4)	旋转电位器RV1，可以看到串口输出数值不断变化，正常显示结果如下所示。

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

