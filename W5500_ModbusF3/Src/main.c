/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"

#include "socket.h"
#include "Internet/DHCP/dhcp.h"
#include "Internet/DNS/dns.h"
#include "uMB.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MSG_INFO_SIZE 93
#define MSG_HELP_SIZE 97

#define TCP1_SOCKET   1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t* msgINFO;
uint8_t* msgHELP;

uint32_t t_blinking, t_blinkingMem;

uint8_t bState, bStateMem;
uint8_t usbBlinkFast, usbBlinkNorm, usbBlinkSlow;

struct uMB_request request = {
    .header = HEADER_START,
    .unitID = 10,
    .fcode = READ_HOLDING_REGISTER,
    .address = 3,
    .quantity = 9,
  };

struct uMB_response response = {
    .header = HEADER_START,
    .unitID = 10
  };

uint16_t hregisters[20] = {
    0x1234, 0x1567, 0x8910, 0x0011, 0x0022,
    0x0033, 0x0000, 0x0009, 0x0008, 0x0007,
    0x0006, 0x0005, 0x0004, 0x0003, 0x0002,
    0x0001, 0x0000, 0x0099, 0x0088, 0x0077
    };

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for ledTask */
osThreadId_t ledTaskHandle;
const osThreadAttr_t ledTask_attributes = {
  .name = "ledTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for usbTask */
osThreadId_t usbTaskHandle;
const osThreadAttr_t usbTask_attributes = {
  .name = "usbTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 128 * 4
};
/* Definitions for spiTask */
osThreadId_t spiTaskHandle;
const osThreadAttr_t spiTask_attributes = {
  .name = "spiTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
uint8_t DataToSend[512];    // Data to send
uint8_t RcvData[512];    // Data received
uint8_t MessageCounter = 0;    // Sent messages counter
int32_t MessageLength = 0;    // Length of the message

uint8_t ReceivedDataBuffer[512];    // Received data buffer
uint8_t ReceivedDataFlag = 0;    // Data received - flag
uint8_t tcp_sequence = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
void ButtonTask(void *argument);
void LedTask(void *argument);
void USBTask(void *argument);
void SpiStartTask(void *argument);

/* USER CODE BEGIN PFP */
void W5500_Select(void);
void W5500_Unselect(void);
void W5500_ReadBuff(uint8_t* buff, uint16_t len);
void W5500_WriteBuff(uint8_t* buff, uint16_t len);
uint8_t W5500_ReadByte(void);
void W5500_WriteByte(uint8_t byte);
void initW5500(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void W5500_Select(void) {
    HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
}

void W5500_Unselect(void) {
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
}

void W5500_ReadBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Receive(&hspi1, buff, len, HAL_MAX_DELAY);
}

void W5500_WriteBuff(uint8_t* buff, uint16_t len) {
    HAL_SPI_Transmit(&hspi1, buff, len, HAL_MAX_DELAY);
}

uint8_t W5500_ReadByte(void) {
    uint8_t byte;
    W5500_ReadBuff(&byte, sizeof(byte));
    return byte;
}

void W5500_WriteByte(uint8_t byte) {
    W5500_WriteBuff(&byte, sizeof(byte));
}

volatile _Bool ip_assigned = 1;


void initW5500(void) {

		uint8_t rx_tx_buff_sizes[] = {2, 2, 2, 2, 2, 2, 2, 2};


	    wiz_NetInfo net_info = {
	        .mac  = { 0xEA, 0x11, 0x22, 0x33, 0x44, 0xEA },
			.ip = { 192, 168, 0, 10 },
			.gw = { 192, 168, 0, 1 },
			.sn = { 255, 255, 255, 0},
	        .dhcp = NETINFO_STATIC
	    };

	    wiz_NetTimeout s_nettime =
	    {
	       	.time_100us = 200,
	    	.retry_cnt = 3
	    };

	    wizchip_init(rx_tx_buff_sizes, rx_tx_buff_sizes);

	    reg_wizchip_cs_cbfunc(W5500_Select, W5500_Unselect);
	    reg_wizchip_spi_cbfunc(W5500_ReadByte, W5500_WriteByte);
	    reg_wizchip_spiburst_cbfunc(W5500_ReadBuff, W5500_WriteBuff);

	    wizchip_setnetinfo(&net_info);

	    /* Set the timeout and the maximum number of re-transmit */
	    wizchip_settimeout(&s_nettime);

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	msgINFO = (uint8_t*)"#############################\n\r# STM32F411CEU6 Blackpill   #\n\r#############################\n\r";
	msgHELP = (uint8_t*)"For led blinking control type:\n\r blink fast,\n\r blink normal,\n\r blink slow\n\nFor MCU type: info\n\r\n\r";

	t_blinkingMem = 500;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(ButtonTask, NULL, &buttonTask_attributes);

  /* creation of ledTask */
  ledTaskHandle = osThreadNew(LedTask, NULL, &ledTask_attributes);

  /* creation of usbTask */
  usbTaskHandle = osThreadNew(USBTask, NULL, &usbTask_attributes);

  /* creation of spiTask */
  spiTaskHandle = osThreadNew(SpiStartTask, NULL, &spiTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bState = 0;
  bStateMem = 0;
  usbBlinkFast = 0;
  usbBlinkNorm = 0;
  usbBlinkSlow = 0;
  t_blinking = 500;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_ButtonTask */
/**
  * @brief  Function implementing the buttonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_ButtonTask */
void ButtonTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  if(GPIO_PIN_SET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	  {
		  bState = 1;
		  osDelay(30);    // Simple debounce for rising edge
	  }
	  else
	  {
		  bState = 0;
	  }
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_LedTask */
/**
* @brief Function implementing the ledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LedTask */
void LedTask(void *argument)
{
  /* USER CODE BEGIN LedTask */
  /* Infinite loop */
  for(;;)
  {
	  // Change from Button PA0
	  if(!bState && bStateMem)
	  {
		  bStateMem = bState;
		  t_blinking = 20;
		  osDelay(30);
	  }
	  if(bState && !bStateMem)
	  {
		  bStateMem = bState;
		  t_blinking = t_blinkingMem;
		  osDelay(30);
	  }

	  // Change from USB requests
	  if(usbBlinkFast)
	  {
		  usbBlinkFast = 0;
		  t_blinking = 50;
		  t_blinkingMem = t_blinking;
	  }
	  if(usbBlinkNorm)
	  {
		  usbBlinkNorm = 0;
		  t_blinking = 500;
		  t_blinkingMem = t_blinking;
	  }
	  if(usbBlinkSlow)
	  {
		  usbBlinkSlow = 0;
		  t_blinking = 2000;
		  t_blinkingMem = t_blinking;
	  }

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	  osDelay(t_blinking);
	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	  osDelay(t_blinking);

  }
  /* USER CODE END LedTask */
}

/* USER CODE BEGIN Header_USBTask */
/**
* @brief Function implementing the usbTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_USBTask */
void USBTask(void *argument)
{
  /* USER CODE BEGIN USBTask */
  /* Infinite loop */
  for(;;)
  {
	  // Receive data by USB
	  if(ReceivedDataFlag == 1){
		uint8_t rcv = 0;
	  	ReceivedDataFlag = 0;
	  	if(strncmp((char*)ReceivedDataBuffer, "help", 4) == 0)
	  	{
	  		rcv = 1;
	  		CDC_Transmit_FS(msgHELP, MSG_HELP_SIZE);
	  	}
	  	if(strncmp((char*)ReceivedDataBuffer, "info", 4) == 0)
	  	{
	  		rcv = 1;
	  		CDC_Transmit_FS(msgINFO, MSG_INFO_SIZE);
	    }
	  	if(strncmp((char*)ReceivedDataBuffer, "blink fast", 10) == 0)
	    {
	  		rcv = 1;
	  		usbBlinkFast = 1;
	  		CDC_Transmit_FS((uint8_t*)"Blinking set to fast speed..\n\r", (uint16_t)30);
	    }
	  	if(strncmp((char*)ReceivedDataBuffer, "blink normal", 12) == 0)
	  	{
	  		rcv = 1;
	  		usbBlinkNorm = 1;
	  		CDC_Transmit_FS((uint8_t*)"Blinking set to normal speed..\n\r", 32);
	    }
	  	if(strncmp((char*)ReceivedDataBuffer, "blink slow", 10) == 0)
	  	{
	  		rcv = 1;
	  		usbBlinkSlow = 1;
	  		CDC_Transmit_FS((uint8_t*)"Blinking set to slow speed..\n\r", 30);
	    }

	  	if(!rcv)
	  	{
	  		rcv = 1;
	  		MessageLength = sprintf((char*)DataToSend, "Got other message (type: help): %s\n\r", ReceivedDataBuffer);
	  		CDC_Transmit_FS(DataToSend, MessageLength);
	  	}
	  }
    osDelay(1);
  }
  /* USER CODE END USBTask */
}

/* USER CODE BEGIN Header_SpiStartTask */
/**
* @brief Function implementing the spiTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SpiStartTask */
void SpiStartTask(void *argument)
{
  /* USER CODE BEGIN SpiStartTask */
	initW5500();
	tcp_sequence = 1;
	// create a socket: TCP
	uint8_t SockNr = TCP1_SOCKET;
	uint8_t sock_buf[64];
	uint16_t sock_len = 64;
	uint8_t sock_status = 0;

	//Modbus TCP
	uMB_resp_msg_create(&response);

	sock_status = socket(SockNr, Sn_MR_TCP, 502, SF_TCP_NODELAY); //SF_TCP_NODELAY
	while(sock_status != SockNr) {
			osDelay(10);
			sock_status = socket(SockNr, Sn_MR_TCP, 502, SF_TCP_NODELAY);
		}
	tcp_sequence = 2;
	sock_status = 0;

	sock_status = listen(SockNr);
	while(sock_status != SOCK_OK) osDelay(10);
	tcp_sequence = 3;
	sock_status = 0;

	int32_t rec_len;
	uint16_t getSn, ir;
	int8_t phylink;

  /* Infinite loop */
  for(;;)
  {
	  getSn = getSn_SR(SockNr);
	  ir = Sn_IR(SockNr);
	  if(getSn == SOCK_CLOSED || getSn != SOCK_ESTABLISHED || getSn == SOCK_CLOSE_WAIT || ir == Sn_IR_TIMEOUT || sock_status == SOCKERR_TIMEOUT || phylink == PHY_LINK_OFF)
	  {
		  if(getSn == SOCK_CLOSE_WAIT || ir == Sn_IR_TIMEOUT || sock_status == SOCKERR_TIMEOUT) disconnect(SockNr);
		  close(SockNr);
		  socket(SockNr, Sn_MR_TCP, 502, SF_TCP_NODELAY);
		  tcp_sequence = 6;
	  }

	  getSn = getSn_SR(SockNr);
	  if(getSn == SOCK_INIT){
		  listen(SockNr);
		  tcp_sequence = 7;
	  }

	  rec_len = 0;
	  if ( (rec_len = getSn_RX_RSR(SockNr) ) > 0){
		  rec_len = recv(SockNr, sock_buf, sock_len);
		  tcp_sequence = 4;
	  }
	  if(rec_len > 0)
	  {
		  request.msg = sock_buf;
		  if(uMB_check_id(request.msg, &response)){
		  	    uMB_resp_parse(request.msg, &hregisters[0], &response);
		  	    //uMB_resp_msg_print(&response);
		  	  }
		  tcp_sequence = 5;
		  sock_status = send(SockNr, response.msg, response.msg_len);
		  osDelay(10);
	  }

	  phylink = wizphy_getphylink();

	  osDelay(10);
  }
  /* USER CODE END SpiStartTask */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
