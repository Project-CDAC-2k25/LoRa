/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "LoRa.h"
#include "string.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
	uint8_t Node_ID;
	uint8_t RSSI;
	uint8_t lastseen;

} NodeList;


typedef struct  __attribute__((packed)){
	uint8_t sourId;
	uint8_t destId;
	// 7th bit - to indicate fframe fragmented
	// 6,5,4 - Number of Frames
	// 3,2,1,0 - 0 - Data, 1 - ACK, 2 - NACK, 3 - CONN , F- Broadcast
	uint8_t control;
	uint8_t frameNo;
	uint8_t pLen; // Length of actual data
	uint8_t data[100];
	uint16_t crc16;  // CRC checksum added at the end
} Packet;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define THIS_NODE 'A'
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
LoRa myLoRa;
struct NodeManager {
	uint8_t NoofNodesPresent;
	NodeList Nodes[10];	//Can handle only ten peers

}nm={0};
uint8_t rx_buf[150] = {0};
uint8_t packetReceivedFlag,sendACK,sendBcast,rlen;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
int __io_putchar(uint8_t ch)
#else
int fputc(int ch,FILE *fp)
#endif
{

	HAL_UART_Transmit(&huart2,&ch, 1, HAL_MAX_DELAY);
	return ch;

}

Packet newPacket(){
	Packet test = {THIS_NODE,0 };
	return test;
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Timer Callbacks;
	if(htim==&htim4){
		// TIM4 for ack and general purpose

	}else if(&htim3==htim){
		sendBcast=1;
	}
}

uint8_t validate_packet(Packet *recv){
	if((recv->destId==THIS_NODE)||(recv->destId==0xFF))	// FF mean its a Broadcast
		return 0;
	return 1;
}

uint16_t crc16_ccitt(const uint8_t* data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++) {
        crc ^= (uint16_t)(data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

uint8_t serializePacket(const Packet* pkt, uint8_t* buf, size_t bufsize)
{
    if (!pkt || !buf)
    	return 1;

    uint16_t total_length = 5 + pkt->pLen + 2; // header (5) + data + crc (2)

    if (bufsize < total_length)
    	return 1;

    buf[0] = pkt->sourId;
    buf[1] = pkt->destId;
    buf[2] = pkt->control;
    buf[3] = pkt->frameNo;
    buf[4] = pkt->pLen;
    memcpy(&buf[5], pkt->data, pkt->pLen);

    // Calculate CRC over all fields except the CRC itself
    uint16_t crc = crc16_ccitt(buf, 5 + pkt->pLen);
    buf[5 + pkt->pLen] = (crc >> 8) & 0xFF;    // CRC high byte
    buf[6 + pkt->pLen] = crc & 0xFF;           // CRC low byte

    return total_length; // total bytes to send
}

uint8_t deserializePacket(const uint8_t* buf, size_t len, Packet* pkt)
{
    if (!buf || !pkt || len < 7) return 1;  // minimum packet size chcking

    uint8_t pLen = buf[4];
    if (pLen > 100 || (len != (size_t)(pLen + 7)))
    	return 1; // size mismatch

    // Verify CRC
    uint16_t received_crc = ((uint16_t)buf[5 + pLen] << 8) | buf[6 + pLen];
    uint16_t calc_crc = crc16_ccitt(buf, 5 + pLen);
    if (received_crc != calc_crc)
        return 1; // CRC check failed

    pkt->sourId = buf[0];
    pkt->destId = buf[1];
    pkt->control = buf[2];
    pkt->frameNo = buf[3];
    pkt->pLen = pLen;
    memcpy(pkt->data, &buf[5], pLen);
    pkt->crc16 = received_crc;

    return 0; // success
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin != GPIO_PIN_1)
		return;
	// Receive the Data;;;; - here our rx_buf as max as possible , because receive func will do take
	rlen = LoRa_receive(&myLoRa, rx_buf, sizeof(rx_buf));

	packetReceivedFlag = 1;


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  myLoRa = newLoRa();
// Setup LoRa pins & SPI handler specific to your board!
  myLoRa.CS_port = GPIOB;
  myLoRa.CS_pin = GPIO_PIN_6;
  myLoRa.reset_port = GPIOA;
  myLoRa.reset_pin = GPIO_PIN_0;
  myLoRa.DIO0_port = GPIOA;
  myLoRa.DIO0_pin = GPIO_PIN_1;
  myLoRa.hSPIx = &hspi1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  LoRa_reset(&myLoRa);
  printf("--> 0x%2X \r\n",LoRa_read(&myLoRa, RegVersion));
  if( LoRa_init(&myLoRa) == LORA_OK )
	  printf("LoRa Started - Status OK\r\n");


  htim3.Instance->ARR=15000;	// Hard coded it to 15 Sec
  	  	  	  	  	  	  	  	// But, Should Implement a Rand Func for selectinga time from 10-25 sec
  	  	  	  	  	  	  	  	// such every node will transmit broadcast msg in rand time
  HAL_TIM_OC_Start_IT(&htim3,TIM_CHANNEL_1);


//---------Just for testing--
  uint8_t msg[]="Hello B !";
  Packet sp={0};
  sp.sourId=THIS_NODE;
  sp.destId='B';
  sp.control=0;
  memcpy(sp.data,msg,sizeof(msg));
  sp.pLen=sizeof(msg);
  uint8_t tx_buf[150]={0};
  uint8_t len = serializePacket(&sp, tx_buf, 150);
  LoRa_transmit(&myLoRa, tx_buf, len, 400);

  Packet rxpckt;
  while (1)
  {
	  if(packetReceivedFlag)
	  {
	      packetReceivedFlag = 0;  // clear flag
	      if(deserializePacket(rx_buf, rlen, &rxpckt) || validate_packet(&rxpckt)){
	    	  printf("Error occured in Deserializing");
	    	  break;
	      }

	      if(rxpckt.destId==0xFF){

	    	  register uint8_t temp=nm.NoofNodesPresent;
	    	  register uint8_t peeralreadyexist=0;
	    	  for(  uint8_t i=0; i < temp ;i++){
	    		  if(nm.Nodes[i].Node_ID==rxpckt.sourId){
	    			  peeralreadyexist=1;
	    		  }
	    	  }

	    	  if(peeralreadyexist){ //If Peer Exist, Update the time by Hal_ticks, which is in milli's ;
	    		  nm.Nodes[temp].lastseen=HAL_GetTick();
	    	  }
	    	  else{
	    		  nm.Nodes[temp].lastseen=HAL_GetTick();
	    		  nm.Nodes[temp].Node_ID=rxpckt.sourId;
	    		  nm.Nodes[temp].RSSI=LoRa_getRSSI(&myLoRa);
	    		  nm.NoofNodesPresent++;
	    	  }
	      }

	      switch(rxpckt.control){

	      case 0:		// data received
	    	  printf("Received - %d bytes %s , RSSI : %d\r\n",rlen, rxpckt.data,LoRa_getRSSI(&myLoRa));
	    	  sendACK=1;
	    	  break;
	      case 1:		//Its a ACK
	    	  // remove from Queue;
	    	  break;
	      case 2:		//Its a NACK, so should re-transmitt the packet.
	    	  break;
	      case 3:		//its CONN req
	    	  break;
	      case 0xF: //its a broad cast mesg ( thinking to imlement in Mesh type )
	    	  //sHOULD SAVE IT TO PEERLIST
	      	  {

	    	  break;
	      	  }
	      }

	  }
	  if(sendACK){
		  sp= newPacket();
		  sp.destId= rxpckt.sourId;
		  sp.control=1;
		  memset(tx_buf,0,len);
		  len = serializePacket(&sp, tx_buf, 150);
		  LoRa_transmit(&myLoRa, tx_buf, len, 400);
		  printf("ACK sent to %c !\r\n",(char)(rxpckt.sourId));
		  sendACK=0;
	  }


	  if(sendBcast){
		  sp=newPacket();
		  sp.destId=0xFF;
		  sp.control=0;
		  sp.pLen=0;
		  memset(tx_buf,0,len);
		  len=serializePacket(&sp, tx_buf, 150);
		  if(LoRa_transmit(&myLoRa, tx_buf, len, 400)){
			  printf("Broadcast msg sent\r\n");
			  sendBcast=0;
		  }


	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 64000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Reset_pin_GPIO_Port, Reset_pin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Reset_pin_Pin */
  GPIO_InitStruct.Pin = Reset_pin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Reset_pin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
