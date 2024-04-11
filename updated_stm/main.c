/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
#include<math.h>
#define ARM_MATH_CM7
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SIZE 2048
#define beamAngles 37
#define beamAngleInc 5
#define lenPhases 256
#define DAC_Start 2458
#define MAX_SAMPLES 4096-DAC_Start
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac;
DMA_HandleTypeDef hdma_dac2;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart3_rx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */
float phases[] = {0, 1.4, 2.8, 5.6, 11.2, 22.5, 45, 90, 180, 4.199999999999999, 7.0, 12.6, 23.9, 46.4, 91.4, 181.4, 8.399999999999999, 14.0, 25.3, 47.8, 92.8, 182.8, 16.799999999999997, 28.1, 50.6, 95.6, 185.6, 33.7, 56.2, 101.2, 191.2, 67.5, 112.5, 202.5, 135, 225, 270, 9.799999999999999, 15.399999999999999, 26.7, 49.2, 94.2, 184.2, 18.2, 29.5, 52.0, 97.0, 187.0, 35.1, 57.6, 102.6, 192.6, 68.9, 113.9, 203.9, 136.4, 226.4, 271.4, 19.599999999999998, 30.9, 53.4, 98.4, 188.4, 36.5, 59.0, 104.0, 194.0, 70.3, 115.3, 205.3, 137.8, 227.8, 272.8, 39.3, 61.8, 106.8, 196.8, 73.1, 118.1, 208.1, 140.6, 230.6, 275.6, 78.7, 123.7, 213.7, 146.2, 236.2, 281.2, 157.5, 247.5, 292.5, 315, 21.0, 32.3, 54.8, 99.8, 189.8, 37.9, 60.4, 105.4, 195.4, 71.7, 116.7, 206.7, 139.2, 229.2, 274.2, 40.7, 63.2, 108.2, 198.2, 74.5, 119.5, 209.5, 142.0, 232.0, 277.0, 80.1, 125.1, 215.1, 147.6, 237.6, 282.6, 158.9, 248.9, 293.9, 316.4, 42.099999999999994, 64.6, 109.6, 199.6, 75.9, 120.9, 210.9, 143.4, 233.4, 278.4, 81.5, 126.5, 216.5, 149.0, 239.0, 284.0, 160.3, 250.3, 295.3, 317.8, 84.3, 129.3, 219.3, 151.8, 241.8, 286.8, 163.1, 253.1, 298.1, 320.6, 168.7, 258.7, 303.7, 326.2, 337.5, 43.5, 66.0, 111.0, 201.0, 77.3, 122.3, 212.3, 144.8, 234.8, 279.8, 82.9, 127.9, 217.9, 150.4, 240.4, 285.4, 161.7, 251.7, 296.7, 319.2, 85.7, 130.7, 220.7, 153.2, 243.2, 288.2, 164.5, 254.5, 299.5, 322.0, 170.1, 260.1, 305.1, 327.6, 338.9, 87.1, 132.1, 222.1, 154.6, 244.6, 289.6, 165.9, 255.9, 300.9, 323.4, 171.5, 261.5, 306.5, 329.0, 340.3, 174.3, 264.3, 309.3, 331.8, 343.1, 348.7, 88.5, 133.5, 223.5, 156.0, 246.0, 291.0, 167.3, 257.3, 302.3, 324.8, 172.9, 262.9, 307.9, 330.4, 341.7, 175.7, 265.7, 310.7, 333.2, 344.5, 350.1, 177.1, 267.1, 312.1, 334.6, 345.9, 351.5, 354.3, 178.5, 268.5, 313.5, 336.0, 347.3, 352.9, 355.7, 357.1, 358.5};
uint8_t binCode[] = {0, 1, 2, 4, 8, 16, 32, 64, 128, 3, 5, 9, 17, 33, 65, 129, 6, 10, 18, 34, 66, 130, 12, 20, 36, 68, 132, 24, 40, 72, 136, 48, 80, 144, 96, 160, 192, 7, 11, 19, 35, 67, 131, 13, 21, 37, 69, 133, 25, 41, 73, 137, 49, 81, 145, 97, 161, 193, 14, 22, 38, 70, 134, 26, 42, 74, 138, 50, 82, 146, 98, 162, 194, 28, 44, 76, 140, 52, 84, 148, 100, 164, 196, 56, 88, 152, 104, 168, 200, 112, 176, 208, 224, 15, 23, 39, 71, 135, 27, 43, 75, 139, 51, 83, 147, 99, 163, 195, 29, 45, 77, 141, 53, 85, 149, 101, 165, 197, 57, 89, 153, 105, 169, 201, 113, 177, 209, 225, 30, 46, 78, 142, 54, 86, 150, 102, 166, 198, 58, 90, 154, 106, 170, 202, 114, 178, 210, 226, 60, 92, 156, 108, 172, 204, 116, 180, 212, 228, 120, 184, 216, 232, 240, 31, 47, 79, 143, 55, 87, 151, 103, 167, 199, 59, 91, 155, 107, 171, 203, 115, 179, 211, 227, 61, 93, 157, 109, 173, 205, 117, 181, 213, 229, 121, 185, 217, 233, 241, 62, 94, 158, 110, 174, 206, 118, 182, 214, 230, 122, 186, 218, 234, 242, 124, 188, 220, 236, 244, 248, 63, 95, 159, 111, 175, 207, 119, 183, 215, 231, 123, 187, 219, 235, 243, 125, 189, 221, 237, 245, 249, 126, 190, 222, 238, 246, 250, 252, 127, 191, 223, 239, 247, 251, 253, 254, 255};
float sepPhases[] = {0, 180, 91.4, 271.4, 5.6, 182.8, 1.4, 175.7, 11.2, 180, 11.2, 174.3, 0, 156.0, 182.8, 330.4, 201.0, 338.9, 180, 307.9, 19.599999999999998, 135, 35.1, 137.8, 56.2, 146.2, 199.6, 275.6, 91.4, 153.2, 21.0, 67.5, 244.6, 275.6, 244.6, 260.1, 0, 0, 260.1, 244.6, 275.6, 244.6, 90, 43.5, 198.2, 136.4, 275.6, 199.6, 191.2, 101.2, 182.8, 80.1, 180, 64.6, 285.4, 157.5, 271.4, 133.5, 285.4, 137.8, 201.0, 45, 196.8, 33.7, 180, 11.2, 198.2, 23.9, 181.4, 4.199999999999999, 271.4, 91.4, 180, 0};
float hamming[2048];
uint16_t raw[2];
uint8_t rawRX[1];
uint16_t values[FFT_SIZE/2];
char msg[5000];
uint8_t transmitting = 0;
float fftin[FFT_SIZE];
float fftout[FFT_SIZE];
float fftin1[FFT_SIZE];
float fftout1[FFT_SIZE];
uint16_t increment = 0;
arm_rfft_fast_instance_f32 fftHandler;
uint32_t missedTransmit = 0;
uint32_t timeUART = 0;
uint32_t timeFFT = 0;
uint16_t len = 0;
float antenna1Phase = 0;
float antenna2Phase = 0;
int curAngle = 0;
uint8_t indexPoo = 18;
uint8_t indexPhase1 = 0;
uint8_t indexPhase2 = 0;
uint8_t newBeam = 1;
uint8_t binary1 = 0;
uint8_t binary2 = 0;
uint8_t serial_transmit = 0;
uint8_t ps = 0;
uint8_t current_serial_bit = 0;
uint32_t dactest[MAX_SAMPLES];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ETH_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */
void parallel(uint8_t, uint8_t);
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
	for(int i = 0; i < MAX_SAMPLES; i++){
		dactest[i] = i + DAC_Start;
	}
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_ETH_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_DAC_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)raw, 2);
  arm_rfft_fast_init_f32(&fftHandler, FFT_SIZE);
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_OC_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim8);
  HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_2, &dactest, MAX_SAMPLES, DAC_ALIGN_12B_R);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  float peakVal = 0.0;
    uint32_t peakFreq = 0.0;
    float peakVal1 = 0.0;
    uint32_t peakFreq1 = 0.0;
    float phase = 0.0;
    float phase1 = 0.0;
    float phasedif = 0.0;
    HAL_UART_Receive_DMA(&huart3, rawRX, 1);
    for(uint16_t i = 0; i < FFT_SIZE; i++){
  	  hamming[i] = (.5-(.5*cos((2.0*M_PI*i)/(FFT_SIZE - 1))));
    }
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, ps);
    //Parallel is 0 Serial is 1
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, ps);

  while (1)
  {

	  if(newBeam && !serial_transmit){
	  		  antenna1Phase = sepPhases[indexPoo*2];
	  		  antenna2Phase = sepPhases[indexPoo*2 + 1];
	  		  for(int i = 0; i < lenPhases; i++){
	  			  if(phases[i] == antenna1Phase){
	  				  indexPhase1 = i;
	  			  }
	  			  if(phases[i] == antenna2Phase){
	  				  indexPhase2 = i;
	  			  }
	  		  }
	  		  binary1 = binCode[indexPhase1];
	  		  binary2 = binCode[indexPhase2];
	  		  if(ps){
	  			serial_transmit = 1;
	  		  } else{
	  			parallel(binary1, binary2);
	  		  }
	  		  newBeam = 0;
	  	  }
	  	  if(increment == 2048){
	  		  timeFFT = HAL_GetTick();
	  		  arm_rfft_fast_f32(&fftHandler, &fftin, &fftout, 0);
	  		  arm_rfft_fast_f32(&fftHandler, &fftin1, &fftout1, 0);
	  		  increment = 0;
	  		  uint16_t freqincrement = 0;
	  		  for(uint16_t increment = 0; increment < FFT_SIZE; increment+=2){
	  			  float curVal = sqrtf((fftout[increment] * fftout[increment]) + (fftout[increment+1]*fftout[increment+1]));
	  			  if(freqincrement < 1024){
	  				  values[freqincrement] = (uint16_t)curVal;
	  			  }
	  			  float curVal1 = sqrtf((fftout1[increment] * fftout1[increment]) + (fftout1[increment+1]*fftout1[increment+1]));
	  			  if(curVal > peakVal){
	  				  peakVal = curVal;
	  				  peakFreq = (uint32_t) (freqincrement * 56179)/((float) FFT_SIZE);
	  				  phase = atan2(fftout[increment+1], fftout[increment]);
	  			  }
	  			  if(curVal1 > peakVal1){
	  				  peakVal1 = curVal1;
	  				  peakFreq1 = (uint32_t) (freqincrement * 56179)/((float) FFT_SIZE);
	  				  phase1 = atan2(fftout1[increment+1], fftout1[increment]);
	  			  }
	  			  freqincrement++;
	  		  }
	  		  phasedif = phase1 - phase;
	  		  timeFFT = HAL_GetTick() - timeFFT;
	  		  if(!transmitting){
	  			  for(int i=0;i<1024;i++) {
	  				  	  	if(i < 1023){
	  				  	  		len+=snprintf(&msg[len],sizeof(msg)-len,"%d,",values[i]);
	  				  	  	}
	  				  	  	else{
	  			  		  		len+=snprintf(&msg[len],sizeof(msg)-len,"%d:",values[i]);
	  			  		  		len+=snprintf(&msg[len],sizeof(msg)-len,"%7.4f,%7.4f,%d\r\n", antenna1Phase, antenna2Phase, curAngle);
	  			  		  		//len+=snprintf(&msg[len],sizeof(msg)-len,"%d %d\r\n", timeUART, missedTransmit);
	  			  		  	}
	  			  		  }

	  			  //sprintf(msg, "FFT Time: %d UART Time: %d Missed: %d F1: %d F2: %d\r\n", timeFFT, timeUART, missedTransmit, peakFreq, peakFreq1);
	  			  //HAL_UART_Transmit_DMA(&huart3, (uint8_t*)msg, strlen(msg));
	  			  HAL_UART_Transmit_DMA(&huart3,(uint8_t*)msg,len);
	  			  transmitting = 1;
	  			  timeUART = HAL_GetTick();
	  		  } else {
	  			  missedTransmit++;
	  		  }
	  		  peakVal = 0.0;
	  		  peakFreq = 0.0;
	  		  peakVal1 = 0.0;
	  		  peakFreq1 = 0.0;
	  		  //memset(msg, 0, sizeof(msg));
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISINGFALLING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_CC2;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC_Init(void)
{

  /* USER CODE BEGIN DAC_Init 0 */

  /* USER CODE END DAC_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC_Init 1 */

  /* USER CODE END DAC_Init 1 */

  /** DAC Initialization
  */
  hdac.Instance = DAC;
  if (HAL_DAC_Init(&hdac) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_T8_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC_Init 2 */

  /* USER CODE END DAC_Init 2 */

}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 50;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 10000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 1080;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|GPIO_PIN_10|GPIO_PIN_11|LD3_Pin
                          |LD2_Pin|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE5
                           PE6 PE7 PE8 PE10
                           PE12 PE14 PE15 PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_12|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin PB10 PB11 LD3_Pin
                           LD2_Pin PB8 */
  GPIO_InitStruct.Pin = LD1_Pin|GPIO_PIN_10|GPIO_PIN_11|LD3_Pin
                          |LD2_Pin|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PG1 PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
	if(increment < 2048){
		float in1 = raw[0];
		in1 = (in1-2048)/2048;
		in1 = in1 * hamming[increment];
		float in2 = raw[1];
		in2 = (in2-2048)/2048;
		in2 = in2 * hamming[increment];
		fftin[increment] = in1;
		fftin1[increment] = in2;
		increment++;
	}
}
	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
		transmitting = 0;
		timeUART = HAL_GetTick() - timeUART;
		len = 0;
		memset(msg, 0, sizeof(msg));
	}
	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
		if(rawRX[0] == 97){
			if(indexPoo > 0 && indexPoo < beamAngles){
				indexPoo--;
				curAngle = -90 + (indexPoo * beamAngleInc);
				newBeam = 1;
			}
		} else if(rawRX[0] == 100){
			if(indexPoo >= 0 && indexPoo < beamAngles - 1){
				indexPoo++;
				curAngle = -90 + (indexPoo * beamAngleInc);
				newBeam = 1;
			}
		}
	}

	void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	  char msgError[30];
	  if (huart->ErrorCode & HAL_UART_ERROR_PE) {
		  sprintf(msgError, "Parity Error\r\n");
		  HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
	  }
	  if (huart->ErrorCode & HAL_UART_ERROR_FE) {
		  sprintf(msgError, "Frame Error\r\n");
		  HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
	  }
	  if (huart->ErrorCode & HAL_UART_ERROR_NE) {
	  	  sprintf(msgError, "Noise Error\r\n");
	  	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
	  }
	  if (huart->ErrorCode & HAL_UART_ERROR_ORE) {
	    	  sprintf(msgError, "Overrun Error\r\n");
	    	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
	  }
	  if (huart->ErrorCode & HAL_UART_ERROR_DMA) {
	    	  sprintf(msgError, "DMA Error\r\n");
	    	  HAL_UART_Transmit(&huart3, (uint8_t*)msg, sizeof(msg), HAL_MAX_DELAY);
	  }
	  // etc...
	}
	void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim){
		if(htim == &htim4){
			HAL_TIM_Base_Stop_IT(&htim4);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		}
		if(htim == &htim9 && serial_transmit && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3))){
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
			if(current_serial_bit < 8){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (binary1 >> current_serial_bit) & 1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (binary2 >> current_serial_bit) & 1);
				current_serial_bit++;
			} else if (current_serial_bit == 8){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, (binary1 >> 6) & 1);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (binary2 >> 6) & 1);
				current_serial_bit++;
			} else if (current_serial_bit > 8 && current_serial_bit < 13){
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, 0);
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, 0);
				current_serial_bit++;
			} else {
				current_serial_bit = 0;
				serial_transmit = 0;
				HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
			}
		}
	}
	void parallel(uint8_t binary1, uint8_t binary2){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, binary1 & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, (binary1 >> 1) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, (binary1 >> 2) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_6, (binary1 >> 3) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, (binary1 >> 4) & 1);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_8, (binary1 >> 5) & 1);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, (binary1 >> 6) & 1);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_9, (binary1 >> 7) & 1);
		HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, (binary1 >> 6) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, binary1 & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, (binary1 >> 1) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, (binary1 >> 2) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, (binary1 >> 3) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, (binary1 >> 4) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, (binary1 >> 5) & 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, (binary1 >> 6) & 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, (binary1 >> 7) & 1);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, (binary1 >> 6) & 1);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
		HAL_TIM_Base_Start_IT(&htim4);
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
