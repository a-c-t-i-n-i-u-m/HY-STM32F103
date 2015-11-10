1设计要求
对开发板上STM32处理器的RTC模块进行操作。RTC模块的当前时间通过串口传送给PC机的超级终端显示，
若RTC模块还未设置时间则通过超级终端进行设置。RTC秒中断每发生一次，发光二极管LED1闪烁一次。
2 硬件电路设计

在开发板上STM32F103VET6处理器的VBAT引脚接+3V钮扣电池，PC6引脚接LED1，晶振、USART等均已连接。
该应用实例不需要额外电路设计，只需将用一根RS232串行通讯线将开发板的RS232口（XS5连接器）
与PC机的串口相连即可。

3 软件程序设计
根据设计要求，软件需实现以下任务：
(1)	系统启动后检查RTC是否已设置。由于RTC在BKP区域，当Vdd掉电之后可由后备电源提供电源，
当后备电源连接到针脚VBAT上时，RTC的设置不会由于外部电源的断开而丢失。在本例中写一个值
到BKP_DR1中以标示RTC是否已配置，在启动之后程序检查BKP_DR1的值。
(2)	 若BKP_DR1的值不正确：（BKP_DR1的值有误或者由于是第一次运行值还未写进去），则需要
配置时间并且询问用户调整时间。
(3)	若BKP_DR1的值正确，则意味着RTC已配置，此时将在超级终端上显示时间。
(4)	在RTC秒中断发生时，连接到PC6 的LED1灯每秒闪烁一次。
整个工程包含3个源文件：startup_stm32f10x_hd.s、stm32f10x_it.c和main.c，其中startup_stm32f10x_hd.s
为启动代码，所有中断服务子程序均在stm32f10x_it.c中，其它函数则在main.c中。下面分别介绍相关的函数，
具体程序清单见参考程序。
函数RTC_IRQHandler用于处理秒中断事件，每次秒中断令LED1闪烁一次，在每次遇到23:59:59时将
时钟回零。
函数RTC_Configuration用于配置RTC模块。
函数USART_Scanf用于从PC超级终端中获取数字值，Time_Regulate利用函数USART_Scanf从超级终端
获取新的RTC时间值，函数Time_Adjust则利用函数USART_Scanf设置新的RTC时间。
函数Time_Display和Time_Show用于将RTC时间转换了字符串送往USART1。
源文件其他函数，例如GPIO、RCC、NVIC、USART的配置，在此不作冗述。

4 运行过程
(1)	使用Keil uVision3 通过JLINK仿真器连接开发板，打开实验例程目录下的
STM32-FD-RTC.Uv2例程，编译链接工程；
(2)	使用串口线，连接开发板上的COM1和PC机的串口；
(3)	在PC机上运行Windows自带的超级终端串口通信程序（波特率115200、1位停止位、无校验位、
无硬件流控制）；或者使用其它串口通信程序；
(4)	选择硬件调试模式，点击MDK的Debug菜单，选择Start/Stop Debug Session项或Ctrl+F5键，
远程连接目标板并下载调试代码到目标系统中；
(5)	例程正常运行之后会在超级终端显示以下信息：
RTC not yet configured....
 RTC configured....
============TimeSettings===================
  Please Set Hours:
在PC机上依次输入时钟、分钟、秒钟之后每隔1秒在超级终端上显示一次时间：
  Please Set Hours:  12
  Please Set Minutes:  0
  Please Set Seconds:  0
Time: 12:00:00
同时开发板的LED1灯也会每隔1S闪烁一次。
(6)	程序正常运行并在开发外部电源保持的情况下，按下Reset按钮，PC超级终端上将继续显示正常时间：
External Reset occurred....
 No need to configure RTC....
Time: 12:03:09
(7)	程序正常运行时断开开发板外部电源，然后重新接上外部电源，PC超级终端上也将继续显示正常时间：
Power On Reset occurred....
 No need to configure RTC....
Time: 12:05:57

(8)	取下处理器板上的纽扣电池，并断开外部电源，然后重新接上外部电源，PC超级终端上将无法继续
正常显示时间，PC超级终端将出现第（5）步所出现内容。
(9)	也可采用软件调试模式，利用USART窗口来模拟实现COM的输入和输出。
