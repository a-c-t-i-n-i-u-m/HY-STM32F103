1设计要求
	利用一块HY-STM32开发板的SD卡及USB接口实现板上的U盘功能。	

 
3 软件程序设计
	
 	main.c
		该函数中主要包含2个函数，其中main函数初始化系统以及USB接口，之后不断查询游戏杆是
		否有动作，如果有动作则根据动作向USB缓冲器发出相关数据；Delay函数用于延迟。
	usb_desc.c
		该文件中没有任何函数，只是包含一些定义USB设备的描述符常数。
		
 	stm32f10x_it.c
		该文件中包含USB中断服务程序，由于USB中断有很多情况，这里的中断服务程序只是调
		用usb_Istr.c文件中的USB_Istr函数，由USB_Istr函数再做轮询处理。参考程序如下：
 	usb_Istr.c
		该文件中只有一个函数，即USB中断的USB_Istr函数，该函数对各类引起USB中断的事件作轮询
		处理。参考程序如下: 
 	usb_prop.c
		该文件用于实现相关设备的USB协议，例如初始化、SETUP包、IN包、OUT包等等。
 	usb_pwr.c
		该文件中包含处理上电、调电、挂起和恢复事件的函数，
 	hw_config.c
		该文件中包含系统配置的函数。
    memory.c
		该文件中包含USB读写SD卡的函数

4 运行过程
	(1)	使用Keil uVision3，通过USB电缆分别连接开发板的USB接口；
	(2)	打开STM32-FD-USB-SD.Uv2例程，编译链接工程；
	(3)	点击MDK 的Debug菜单，点击Start/Stop Debug Session；或者将程序烧写到开发板上，
	重启开发板；
	(4)	通过PC机的我的电脑选项查看SD卡。



