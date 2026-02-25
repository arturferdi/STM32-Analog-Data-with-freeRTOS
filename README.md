# STM32-Analog-Data-with-freeRTOS
Here I created a program using STM32Cube IDE Framework to read analog data and combine it with freeRTOS. For this, I use STM32F103 Series, or commonly it is called "Blue Pill". Well, we all know that STM32 GPIO's are around 3.3V tolerant. And only use single channel ADC.

![GPIO Usage](https://github.com/arturferdi/STM32-Analog-Data-with-freeRTOS/blob/064ef2992bb5c842979725d97a0436f64ea0c7a0/Documentation/Screenshot%20from%202026-02-25%2022-38-36.png)
To set configurations, I use STM32CubeMX. And I generate some configurations, I also utilize USB CDC for serial monitor, so I shouldn't use USB To Serial to monitoring the result data through Serial Monitor. Here are complete configurations.

 1. System Core
 
		- **RCC**	: Set HSE to Crystal/Ceramic Resonator.
		- **SYS**	: Set "Timebase Source to TIM4" because freeRTOS strongly recommends not using SysTick for the HAL timebase.
2. Analog

		- **ADC1**	: Check IN1. Leave the settings as default.
3. Connectivity

		- **USB**	: Check Device (FS).
4. Middleware & Software Packages

		- **USB_DEVICE**	: Set Class For FS IP to Communication Device Class (Virtual Port COM)
		- **FREERTOS**	: Set Interface to CMSIS_V1 or CMSIS_V2
5. GPIO

		- Use PC13 and set it to GPIO_Output (It is Blue Pill's built-in LED)
