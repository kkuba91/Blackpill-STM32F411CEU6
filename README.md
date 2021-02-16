# Blackpill-STM32F411CEU6
Blackpill STM32F411CEU6 board example start project (FREERTOS + USB VCOM)

This is example start project created with STM32CubeIDE 1.3.0.

In the project used settings works with ST-Link V2 (mini) - SWD debugging.

The project includes FREERTOS and OTG_USB (device only as Virtual COM)
For USB driver check out: [STM32 Virtual COM Port Driver](https://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-stm32102.html)

## Blackpill STM32F411 general settings in configurator
- RCC (required for USB to be operational)
- usbd_cdc.c must have some memory init in `static uint8_t USBD_CDC_Init(USBD_HandleTypeDef *pdev, uint8_t cfgidx)` (added `memset(hcdc,0,sizeof(USBD_CDC_HandleTypeDef));    // memory set for hcdc / pdev->pClassData`)

## FREERTOS
For start example created only 3 start tasks with Normal Priority:
- `ButtonTask` for button state monitoring
- `LedTask` for Led Blinking (PC13) control
- `USBTask` for USB messaging by Virual COM

## Virtual COM tests
After installing STM Virtual COM Port Driver, is is able to connect to the device by USB-C connector (check Device Manager in Control Panel if COM port installed properly).
Ex. use RealTerm tool for data send/receive as ansi stream:

![alt text](https://github.com/kkuba91/Blackpill-STM32F411CEU6/blob/main/realterm.png?raw=true)

Used commands in example are: `info`, `help`, `blink slow`, `blink fast`, `blink normal`

## Copywritghts
STM32, STM32Cube, STM32CubeIDE - ST Microelectronics company (C)

## References
[1] USB CDC Port init problem: https://community.st.com/s/question/0D50X00009XkfnjSAB/problem-solved-with-usb-cdc
[2] USB VCOM Tutorial (polish): https://forbot.pl/blog/kurs-stm32-f4-11-komunikacja-przez-usb-id13477

