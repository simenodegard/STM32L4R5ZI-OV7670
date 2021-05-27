/*
 * new_main.c
 *
 *  Created on: May 20, 2021
 *      Author: Simen
 */


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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "new_camera.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Buffer used for transmission */

//uint8_t aTxBuffer[] = {0x12,0x80};
/* Buffer used for reception */

#define  OV7670_I2C_ADDRESS	0x21

uint8_t aRxBuf[];
//#define useVga
//#define useQvga                 //****************************** Resolution selection ***************************//
#define useQqvga

//STM32L031K6T6
//#define ImAdd	0x80009000
//#define Im_Size	0x12c00
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim22;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
uint8_t FrameHeader[]="*Frame*";
//uint8_t ImAdd1[80*40];
static uint8_t ImAdd1[80*40]={0x53,0x46,0xCD,0xCD};
static uint8_t ImAdd2[80*40]={0x53,0x46,0xCD,0xCD};

uint8_t *ImAdd11=ImAdd1;
uint8_t *ImAdd22=ImAdd2;

volatile uint8_t Mem_SW =0x00; //for switching between memory two locations ImAdd1 & ImAdd2
uint8_t aTxBuf[] ;
uint8_t aRxBuf[];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM22_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM22_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* ************************************XCLK for camera be started from TIM22 PWM channel2 Output********** */
  /* this commence to start must be tookplace before i2c and camera initialization since camera is not active still */

      if(HAL_TIM_PWM_Start(&htim22, TIM_CHANNEL_2) != HAL_OK)
        {
          Error_Handler();
         }
       /* ***************************************USER CODE END 2************************************************** */

  HAL_Delay(200);
  MX_I2C1_Init();
  HAL_Delay(20);


 // _disable_irq();//disable interrupts
  camInit();
  #ifdef useVga  //    bayerRGB ,yuv422,rgb565
	//wrReg(0x12, 0x80);
       HAL_Delay(100);
        //wrReg(REG_COM10, 32);
        HAL_Delay(100);
        setRes(VGA);
        HAL_Delay(100);
	setColor(BAYER_RGB);
        HAL_Delay(400);

#elif defined(useQvga)
	setRes(QVGA);
          HAL_Delay(100);
	setColor(RGB565);
          HAL_Delay(100);
	//wrReg(0x11, 12);
#else
	setRes(QQVGA);
	setColor(YUV422);
	wrReg(0x11, 3);
#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    #ifdef useVga
	captureImg(640, 480);
    #elif defined(useQvga)
        //wrReg(0x71, 0x80);
        //wrReg(0x70, 0x80);
        //wrReg(REG_COM17,COM17_CBAR);          //  COM17_CBAR          0x08  /* DSP Color bar */
	captureImg(320 * 2, 240);
    #else
	captureImg(160 * 2, 120);
     #endif

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_8;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */
//SPI2->CR1|=(1<<2);
  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */
  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */
 // SPI2->CR1&=0xdf;
 // SPI2->CR2|=4;
  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM22 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM22_Init(void)
{

  /* USER CODE BEGIN TIM22_Init 0 */

  /* USER CODE END TIM22_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM22_Init 1 */

  /* USER CODE END TIM22_Init 1 */
  htim22.Instance = TIM22;
  htim22.Init.Prescaler = 0;
  htim22.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim22.Init.Period = 1;
  htim22.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim22.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim22) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim22, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim22, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM22_Init 2 */

  /* USER CODE END TIM22_Init 2 */
  HAL_TIM_MspPostInit(&htim22);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */
  //USART1->CR3 = USART_CR3_DMAT;

  //USART1->ICR = USART_ICR_TCCF;/* Clear TC flag */
  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
    USART1->CR3 |= 1<<7;  //USART_CR3_DMAT;
  /* USER CODE END USART1_Init 2 */

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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
//static void MX_DMA_Init(void)
//{
//
//  /* DMA controller clock enable */
//  __HAL_RCC_DMA1_CLK_ENABLE();
//
//  /* DMA interrupt init */
//  /* DMA1_Channel2_3_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 1, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
//
//}

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SERIAL_OUT_GPIO_Port, SERIAL_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DD0_Pin DD1_Pin DD2_Pin DD3_Pin
                           DD4_Pin DD5_Pin DD6_Pin DD7_Pin */
  GPIO_InitStruct.Pin = DD0_Pin|DD1_Pin|DD2_Pin|DD3_Pin
                          |DD4_Pin|DD5_Pin|DD6_Pin|DD7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SERIAL_OUT_Pin */
  GPIO_InitStruct.Pin = SERIAL_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SERIAL_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : HREF_Pin PCLK_Pin */
  GPIO_InitStruct.Pin = HREF_Pin|PCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : YSYNC_Pin */
  GPIO_InitStruct.Pin = YSYNC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(YSYNC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//static void MX_DMA_Init(void)
//{
//
//  /* DMA controller clock enable */
//  __HAL_RCC_DMA1_CLK_ENABLE();
//
//
//
//  /* DMA1 Channel2 USART1_TX config */
//  /* (6)  Map USART1_TX DMA channel */
//  /* (7) Peripheral address */
//  /* (8) Memory address */
//  /* (9) Memory increment */
//  /*     Memory to peripheral*/
//  /*     8-bit transfer */
//  DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~DMA_CSELR_C2S) | (3 << (1 * 4)); /* (6) */
//  DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR); /* (7) */
//  DMA1_Channel2->CMAR = (uint32_t)ImAdd; /* (8) */
//  DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_DIR| DMA_CCR_TCIE; /* (9) */
//  DMA1_Channel2->CNDTR = 160*20; /* Data size */
// /* DMA interrupt init */
//  /* DMA1_Channel2_3_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
//
//}
//void DMA1_Channel2_3_IRQHandler(void)
//{
//  if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
//  {
//    DMA1->IFCR |= DMA_IFCR_CTCIF2; /* Clear TC flag */
//
//    USART1_ImageReceived = 1;
//
//    DMA1_Channel2->CCR &=~ DMA_CCR_EN;
//    DMA1_Channel2->CNDTR = 160*20; /* Data size */
//    DMA1_Channel2->CCR |= DMA_CCR_EN;
//  }
//  else
//  {
//     /* Report an error */
//    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn); /* Disable DMA1_Channel2_3_IRQn */
//  }
//}

static void captureImg(uint16_t wg, uint16_t hg){

  uint16_t lg2;

  //SPI2->CR1|=1<<6; // spi2 transmission mode enabled SPI2->CR1 |= SPI_CR1_SPE;

#ifdef useQvga
  //uint8_t buf[640];

#elif defined(useQqvga)
  //uint8_t buf[320];
#endif

  // HAL_UART_Transmit(&huart1, (uint8_t*)FrameHeader, 7, 2);
  //hg=3;
  // wg=5;
#ifdef useVga
  while(1){
    //SPI2->CR1 &= 0xDF;// spi2 transmission mode enabled SPI2->CR1 |= SPI_CR1_SPE;
    //Wait for vsync it is on pin GPIOB_PIN_3 (counting from 0) portD
    //HAL_UART_Transmit(&huart1, (uint8_t*)FrameHeader, 7, 2);
    while (!(GPIOB->IDR & GPIO_PIN_3));//wait for high
    SPI2->CR1|=1<<6;// spi2 transmission mode enabled SPI2->CR1 |= SPI_CR1_SPE;

    uint8_t temp=SPI2->DR; // to clear OVR flag by dummy reading of DR and SR
    temp=SPI2->SR;
    while(!((SPI2->SR)&(1<<1))){};
    SPI2->DR=0x53;
    while(!((SPI2->SR)&(1<<1))){};
    SPI2->DR=0x46;
    while ((GPIOB->IDR & 8 /*GPIO_PIN_3*/));//wait for low
    while (hg--){
      lg2 = wg;

      while (lg2--){
        SPI2->CR1|=1<<6;
        //while(!((SPI2->SR)&(1<<1))){};
        while ((GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for low

        if(lg2 == wg-1 | lg2 == wg-2){ SPI2->DR = 0xCD;  /* Line header is 0xCDCD in hex  */

        }

        //else{SPI2->DR=0x3;}
        //SPI2->CR1 &= 0xDF;
        else{SPI2->DR=GPIOC->IDR;}
        //else{ huart1.Instance->TDR = GPIOC->IDR;}

        //*(uint8_t *)&(SPI2->DR) = GPIOC->IDR;//SPI2->DR = GPIOC->IDR;
        // *Pdata=GPIOC->IDR;
        // *(uint8_t *)&(SPI2->DR) =*Pdata; //*(uint8_t *)&(Pdata);//GPIOC->IDR;

        //SPI2->DR=GPIOC->IDR;
        // SPI2->DR=0xCD;




        //while (!(GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for high
        //SPI2->DR=0;
        // while(!((SPI2->SR)&(1<<1))){};
        // while(((SPI2->SR)&(1<<7))){};

      }
    }
  }     //SPI2->CR1 &= 0xDF;// spi2 transmission mode enabled SPI2->CR1 |= SPI_CR1_SPE;
#elif defined(useQvga)
  while(1){
    HAL_Delay(8000);
    //SPI2->CR1 &= 0xDF;// spi2 transmission mode enabled SPI2->CR1 |= SPI_CR1_SPE;
    //Wait for vsync it is on pin GPIOB_PIN_3 (counting from 0) portD
    //HAL_UART_Transmit(&huart1, (uint8_t*)FrameHeader, 7, 2);
    while (!(GPIOB->IDR & GPIO_PIN_3));//wait for high
    SPI2->CR1|=1<<6;// spi2 transmission mode enabled SPI2->CR1 |= SPI_CR1_SPE;

    uint8_t temp=SPI2->DR; // to clear OVR flag by dummy reading of DR and SR
    temp=SPI2->SR;
    // while(!((SPI2->SR)&(1<<1))){};
    // SPI2->DR=0x53;
    // while(!((SPI2->SR)&(1<<1))){};
    //SPI2->DR=0x46;
    while ((GPIOB->IDR & 8 /*GPIO_PIN_3*/));//wait for low
    while (hg--){
      lg2 = wg;

      while (lg2--){
        SPI2->CR1|=1<<6;
        //while(!((SPI2->SR)&(1<<1))){};
        while ((GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for low

        // if(lg2 == wg-1 | lg2 == wg-2){huart1.Instance->TDR = 0xCD;  /* Line header is 0xCDCD in hex  */

        // }

        // else if(320<lg2 & lg2<501){SPI2->DR = 0x1f;}
        //else  if(500<lg2 & lg2<638){SPI2->DR = 0x00;}
        // else {SPI2->DR = 0x23;}

        //else if(320<lg2 & lg2<501){huart1.Instance->TDR = 0x1f;}
        // else  if(500<lg2 & lg2<638){huart1.Instance->TDR = 0x00;}
        // else {huart1.Instance->TDR = 0x23;}

        // else if(320<=lg2<400){SPI2->DR = 0x53;}
        //else if(240<=lg2<320){SPI2->DR = 0x7f;}
        //else if(160<=lg2<240){SPI2->DR = 0x05;}
        //else if(80<=lg2<160){SPI2->DR = 0x25;}
        // else if(0<=lg2<80){SPI2->DR = 0xff;}
        // else {SPI2->DR = 0x0f;}
        //SPI2->CR1 &= 0xDF;
        //else{SPI2->DR=GPIOC->IDR;}
        // else{ huart1.Instance->TDR = GPIOC->IDR;}
        huart1.Instance->TDR = GPIOC->IDR;
        //*(uint8_t *)&(SPI2->DR) = GPIOC->IDR;//SPI2->DR = GPIOC->IDR;
        // *Pdata=GPIOC->IDR;
        // *(uint8_t *)&(SPI2->DR) =*Pdata; //*(uint8_t *)&(Pdata);//GPIOC->IDR;

        //SPI2->DR=GPIOC->IDR;
        // SPI2->DR=0xCD;




        while (!(GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for high
        //SPI2->DR=0;
        // while(!((SPI2->SR)&(1<<1))){};
        // while(((SPI2->SR)&(1<<7))){};

      }
    }
  }
#else


  while(1){
  uint8_t hg2=hg;
 // USART1->CR3 |= ~(1<<7);
 // DMA1_Channel2->CCR &=~ DMA_CCR_EN;
  if(Mem_SW==0){
    ImAdd11=&ImAdd1[0];
  ImAdd22=&ImAdd2[4];

  }
  else {
     ImAdd11=&ImAdd2[0];
  ImAdd22=&ImAdd1[4];
  }

  DMA1_Channel2->CMAR = (uint32_t)ImAdd11;
  //DMA1_Channel2->CCR &= DMA_CCR_EN;
  //USART1->CR3 |= 1<<7;


  while (hg2--){

    lg2 = wg/2;
    while (lg2--){

      while ((GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for low

      if((lg2>49 & lg2<111) &(hg2>39 & hg2<81)){*ImAdd22++= GPIOC->IDR;}


      // else{huart1.Instance->TDR = GPIOC->IDR;}
      // huart1.Instance->TDR = GPIOC->IDR;
      while (!(GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for high
       while ((GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for low
      while (!(GPIOA->IDR & 1024 /*GPIO_PIN_10*/));//wait for high
    }
   // DMA1_Channel2->CCR &=~ DMA_CCR_EN;
    //DMA1_Channel2->CNDTR = 60*40; /* Data size */


  }
   DMA1_Channel2->CCR |= DMA_CCR_EN;
  Mem_SW =Mem_SW ^ 0xFF;
  }
#endif
}

 void wrReg(uint8_t reg,uint8_t dat){
        uint8_t addr[]=reg;
        uint8_t adat[]=dat;
        char buffer[100];

  	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)addr, 1, 100);
  	HAL_Delay(20);
  	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)adat, 1, 100);
  	HAL_Delay(1);
        sprintf(buffer,"This is REG: 0x%X --> 0x%X\r\n",addr[0],adat[0]);
                // printf(buffer);
                 HAL_UART_Transmit(&huart1, (uint8_t*)buffer,30, 2);
  }
  uint8_t rdReg(uint8_t reg){
  	uint8_t addr[]=reg;
  	if(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)addr, 1, 100)==!HAL_OK)
        {
         // printf("I2C1 M-2-S error: not AKN received\r\n");
        }
  	HAL_Delay(100);
  	HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)aRxBuf, 1, 100);
  	return aRxBuf[0];


  }
  static void wrSensorRegs8_8(const struct regval_list reglist[]){
  	uint8_t  i=0;
        const struct regval_list *next = reglist;
        char buffer[30];
  	for(;;){

  		if(((*next).reg_num==255)&&((*next).value==255))
  			break;
               HAL_Delay(100);

                 HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)next,2, 10);

                 HAL_Delay(10);
                 HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)next, 1, 10);
                 HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(OV7670_I2C_ADDRESS<<1), (uint8_t*)aRxBuf, 1, 10);
                 *buffer=0;
                 sprintf(buffer,"This is REG: %d 0x%X --> 0x%X\r\n",i,(*next).reg_num,aRxBuf[0]);
                // printf(buffer);
                 HAL_UART_Transmit(&huart1, (uint8_t*)buffer,30, 2);
                 next++;
                 i++;
  	}
  }
  void setColor(enum COLORSPACE color){
  	switch(color){
  		case YUV422:
  			wrSensorRegs8_8(yuv422_ov7670);
  		break;
  		case RGB565:
  			wrSensorRegs8_8(rgb565_ov7670);
  			//{uint8_t temp=rdReg(0x11);
  			//HAL_Delay(1);
  			//wrReg(0x11,temp);}//according to the Linux kernel driver rgb565 PCLK needs rewriting
  		break;
  		case BAYER_RGB:
  			wrSensorRegs8_8(bayerRGB_ov7670);
  		break;
  	}
  }
  void setRes(enum RESOLUTION res){
  	switch(res){
  		case VGA:
  			wrReg(REG_COM3,0);	// REG_COM3
  			wrSensorRegs8_8(vga_ov7670);
  		break;
  		case QVGA:
  			wrReg(REG_COM3,4);	// REG_COM3 enable scaling
  			wrSensorRegs8_8(qvga_ov7670);
  		break;
  		case QQVGA:
  			wrReg(REG_COM3,4);	// REG_COM3 enable scaling
  			wrSensorRegs8_8(qqvga_ov7670);
  		break;
  	}
  }
  void camInit(void){
  	//wrReg(0x12, 0x80);//Reset the camera.
  	HAL_Delay(100);
  	wrSensorRegs8_8(ov7670_default_regs);
  	//wrReg(REG_COM10,0x20);//PCLK does not toggle on HBLANK.{0x15 32]
  }

void DMA1_Channel2_3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 0 */
if((DMA1->ISR & DMA_ISR_TCIF2) == DMA_ISR_TCIF2)
  {
    DMA1->IFCR |= DMA_IFCR_CTCIF2; /* Clear TC flag */

    //USART1_ImageReceived = 1;

    DMA1_Channel2->CCR &=~ DMA_CCR_EN;
    DMA1_Channel2->CNDTR = (60*40+4); /* Data size */
    DMA1_Channel2->CCR |= DMA_CCR_EN;
  }
  else
  {
     /* Report an error */
    NVIC_DisableIRQ(DMA1_Channel2_3_IRQn); /* Disable DMA1_Channel2_3_IRQn */
  }
  /* USER CODE END DMA1_Channel2_3_IRQn 0 */
  //HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA1_Channel2_3_IRQn 1 */

  /* USER CODE END DMA1_Channel2_3_IRQn 1 */
}

static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();



  /* DMA1 Channel2 USART1_TX config */
  /* (6)  Map USART1_TX DMA channel */
  /* (7) Peripheral address */
  /* (8) Memory address */
  /* (9) Memory increment */
  /*     Memory to peripheral*/
  /*     8-bit transfer */
  DMA1_CSELR->CSELR = (DMA1_CSELR->CSELR & ~DMA_CSELR_C2S) | (3 << (1 * 4)); /* (6) */
  DMA1_Channel2->CPAR = (uint32_t)&(USART1->TDR); /* (7) */
  DMA1_Channel2->CMAR = (uint32_t)ImAdd11; /* (8) */
  DMA1_Channel2->CCR |= DMA_CCR_MINC | DMA_CCR_DIR| DMA_CCR_TCIE; /* (9) */
  DMA1_Channel2->CNDTR = (40*60+4); /* Data size */
 /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
