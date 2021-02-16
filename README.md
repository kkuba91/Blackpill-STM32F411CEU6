# Blackpill-STM32F411CEU6
Blackpill STM32F411CEU6 board example start project (FREERTOS + USB VCOM)

This is example start project created with STM32CubeIDE 1.3.0.

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

