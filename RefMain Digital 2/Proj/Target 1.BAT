SET PATH=C:\Keil_v4\ARM\BIN40;C:\Program Files\Intel\iCLS Client\;C:\Windows\system32;C:\Windows;C:\Windows\System32\Wbem;C:\Windows\System32\WindowsPowerShell\v1.0\;C:\Program Files\Intel\Intel(R) Management Engine Components\DAL;C:\Program Files\Intel\Intel(R) Management Engine Components\IPT;C:\Program Files\ATMEL Corporation\SAM-BA v2.10
SET ARMCC41INC=C:\Keil_v4\ARM\RV31\Inc
SET ARMCC41LIB=C:\Keil_v4\ARM\RV31\Lib
SET CPU_TYPE=STM32F051K6
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=Target 1
SET CPU_CLOCK=0x007A1200
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "main.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_gpio.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_rcc.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_tim.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_adc.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_usart.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_misc.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_i2c.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "stm32f0xx_iwdg.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "led.__i"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "delay.__i"
"C:\Keil_v4\ARM\BIN40\ArmAsm" --Via ".\startup_stm32f0xx_ld._ia"
"C:\Keil_v4\ARM\BIN40\ArmCC" --Via "system_stm32f0xx.__i"
"C:\Keil_v4\ARM\BIN40\ArmLink" --Via "RefMain.lnp"
C:\Keil_v4\ARM\BIN40\fromelf.exe "RefMain.axf" --i32combined --output "RefMain.hex"
