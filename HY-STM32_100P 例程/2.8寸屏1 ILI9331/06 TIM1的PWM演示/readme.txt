1设计要求
	对TIM1定时器进行控制，通道1输出的占空比为25%.
	
2 硬件电路设计
	TIM1定时器的通道1对应PA.08引脚。
	
3软件程序设计
	由于TIM1计数器的时钟频率为72MHz，希望各通道输出频率fTIM1为17.57KHz，根据：
	fTIM1=TIM1CLK/(TIM1_Period + 1)，可得到TIM1预分频器的值TIM1_Period为0xFFFF。
根据公式：通道输出占空比 = TIM1_CCRx/(TIM1_Period + 1)，可以得到各通道比较/捕获
寄存器的计数值。其中：TIM1_CCR1寄存器的值为0x7FFF、TIM1_CCR2寄存器的值为0x3FFF、
TIM1_CCR3寄存器的值为0x1FFF。

	该应用实例软件设计较为简单，只要配置TIM1工作模式为PWM模式，并分别设置上述值，
	TIM1即可按要求工作。
	
4 运行过程
(1)	使用Keil uVision3 通过JLINK 仿真器连接开发板，打开实验例程目录
下的STM32-FD-TIM1.Uv2例程，编译链接工程；
(2)	点击MDK 的Debug菜单，点击Start/Stop Debug Session；
(3)	通过示波器察看PA.08的输出波形。
(4) 如果无示波器，则可以采用软件仿真模式来观测输出波形。

