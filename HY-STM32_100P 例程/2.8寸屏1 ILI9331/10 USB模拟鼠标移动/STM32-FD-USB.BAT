SET PATH=d:\Keil\ARM\BIN40\;C:\Program Files\NVIDIA Corporation\PhysX\Common;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\Program Files\Common Files\Ulead Systems\MPEG;C:\Program Files\Common Files\Thunder Network\KanKan\Codecs
SET ARMCC41INC=d:\Keil\ARM\RV31\INC
SET ARMCC41LIB=d:\Keil\ARM\RV31\LIB
SET CPU_TYPE=STM32F103VE
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=STM32-FD-USB
SET CPU_CLOCK=0x00000000
"d:\Keil\ARM\BIN40\ArmAsm" --Via ".\obj\startup_stm32f10x_hd._ia"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\hw_config.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\main.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_desc.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_istr.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_prop.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_pwr.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_it.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\misc.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_gpio.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_rcc.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_flash.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_usart.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\stm32f10x_exti.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_core.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_init.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_int.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_mem.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\usb_regs.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\core_cm3.__i"
"d:\Keil\ARM\BIN40\ArmCC" --Via ".\obj\system_stm32f10x.__i"
"d:\Keil\ARM\BIN40\ArmLink" --Via ".\Obj\STM32-FD-USB.lnp"
d:\Keil\ARM\BIN40\fromelf.exe ".\Obj\STM32-FD-USB.axf" --i32combined --output ".\Obj\STM32-FD-USB.hex"
