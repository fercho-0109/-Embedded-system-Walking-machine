/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
Lcd_HandleTypeDef lcd;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
uint16_t raw_in;
float Pot_mv;
float Pot;
int Potshow;
volatile int emergency = 0;
volatile int velocity = 0;
volatile int c=0,c_p=0;
int x=0;
int y=0;
int z=0;
int ref=0;
int pid=0;
float integral=0;
int u_max=200;
int u_min=0;
int pwm=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void EmergencyStop(void);
void start(void);
void velocities(void);

void EXTI9_5_IRQHandler(void){
  if(EXTI->PR & (0x01 << 5)){
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    if(velocity<3)
    velocity++;
    else if (velocity>=3)
    	velocity=3;
    EXTI->PR |= (0x01 << 5); // Clear the EXTI pending register
  }
  if(EXTI->PR & (0x01 << 6)){
      NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
      if(velocity>0)
         velocity--;
         else if (velocity<1)
         	velocity = 0;
      EXTI->PR |= (0x01 << 6); // Clear the EXTI pending register
    }
  if(EXTI->PR & (0x01 << 7)){
      NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
      emergency = 1;
      EXTI->PR |= (0x01 << 6); // Clear the EXTI pending register
    }
}
void EXTI2_IRQHandler(void){
			// GPIOC2
    NVIC_ClearPendingIRQ(EXTI2_IRQn);
    c_p++;

    EXTI->PR |= (0x01 << 2); 				// Clear the EXTI pending register
}
void EXTI15_10_IRQHandler(void){
  if(EXTI->PR & (0x01 << 15)){				// GPIOC13 - button
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    c_p++;
    EXTI->PR |= (0x01 << 15); 				// Clear the EXTI pending register
  }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM2_IRQHandler(void){					// Encoder check
	NVIC_ClearPendingIRQ(TIM2_IRQn);
	TIM2->SR= (uint16_t)(~(1 << 0));
	c=c_p;									// storing the number of pulses
	c_p = 0;								// pulses counter reset
	pid=1;
}
void ADC_IRQHandler(void) {
	HAL_ADC_IRQHandler(&hadc1);
	// Re-Start ADC in interrupt Mode
	HAL_ADC_Start_IT(&hadc1) ;
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	// Read the analog input value
	raw_in = HAL_ADC_GetValue(&hadc1);
	// Converts value in the 0V-3.3V range
	Pot_mv = (((float)raw_in) /255 ) * 2.5;
	// Convert mV to angle
	Potshow=(Pot_mv*45)/2.5;
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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  // Enable clock on GPIOA/GPIOB/GPIOC
      RCC->AHB1ENR |= 0x07;

      /* Booton  GPIOA5 and GPIOA6 GPIOA7 GPIOA10 */
      // GPIOA5 <-> PC_5 (USER_BUTTON)
      GPIOA->MODER &= ~(0x03 << 10); // Clear MODER GPIOA5
      GPIOA->MODER |= (0x00 << 10);  // Set Input
      GPIOA->PUPDR &= ~(0x03 << 10); // Clear PUPDR GPIOA5
      GPIOA->PUPDR |= (0x00 << 10);  // Set Pull-Up

      //GPIOA6 <-> PC_6
      GPIOC->MODER &= ~(0x03 << 12); // Clear MODER GPIOA6
      GPIOC->MODER |= (0x00 << 12);  // Set Input
      GPIOC->PUPDR &= ~(0x03 << 12); // Clear PUPDR GPIOA6
      GPIOC->PUPDR |= (0x00 << 12);  // Set Pull-Up
      /** GPIOA Configuration **/

       GPIOA->MODER  &= ~(0x03 << 14); // Clear GPIOA7
       GPIOA->MODER |= (0x00 << 14);   // Set Input
       GPIOA->PUPDR &= ~(0x03 << 14); // Clear PUPDR GPIOA10
       GPIOA->PUPDR |= (0x01 << 14);  // Pull-Up Mode su pin di input

       GPIOA->MODER &= ~(0x03 << 20); // Clear MODER GPIOA10
       GPIOA->MODER |= (0x00 << 20);  // Set Input
       GPIOA->PUPDR &= ~(0x03 << 20); // Clear PUPDR GPIOA10
       GPIOA->PUPDR |= (0x00 << 20);  // Set Pull-Up


       /** GPIOB Configuration **/
       GPIOB->MODER  &= ~(0x03 << 12); // Clear GPIOB6
       GPIOB->MODER |= (0x01 << 12);  // Set Output
       // Push-Pull output & Pull-up
       GPIOB->OTYPER &= ~(0x1 << 6);
       GPIOB->PUPDR &= ~(0x03 << 12); // Clear GPIOB6
       GPIOB->PUPDR |= (0x01 << 12);

       /** GPIOC Configuration **/
       GPIOC->MODER  &= ~(0x03 << 14); // Clear GPIOC7
       GPIOC->MODER |= (0x01 << 14);  // Set Output
       // Push-Pull output & Pull-up
       GPIOC->OTYPER &= ~(0x1 << 7);
       GPIOC->PUPDR &= ~(0x03 << 14); // Clear GPIOC7
       GPIOC->PUPDR |= (0x01 << 14);
       // Output LOW
       GPIOB->ODR &= ~(0x01 << 6);
       GPIOC->ODR &= ~(0x01 << 7);

   	/* Config PC2 - Encoder1 */
     	GPIOC->MODER &= ~(0x03 << 4); // Clear MODER GPIOC13
     	GPIOC->MODER |= (0x00 << 4);  // Set Input
     	GPIOC->PUPDR &= ~(0x03 << 4); // Clear PUPDR GPIOC13
     	GPIOC->PUPDR |= (0x01 << 4);  // Set Pull-Up

       /* GPIO configuration for PMW */
        // TIMER 3 CONFIGURATION
        RCC->APB1ENR |= (0x01 << 1);		// clock TIM3
        TIM3->PSC = 5999;
      	TIM3->ARR = 200;

       // MICRO SERVO
       	GPIOB->MODER &= ~(0x03 << 8);       // Clear GPIOB4
       	GPIOB->MODER |= (0x02 << 8);	    // Alternate Function (AF) Mode
       	GPIOB->PUPDR &= ~(0x03 << 8);
       	GPIOB->PUPDR |= (0x01 << 8);	    // Pull-up
       	GPIOB->AFR[0] |= (0x02 << 16);	    // [0]=AFRL, <<16 = pin4, 0x02 = AF2
      	// TIMER 3 CONFIGURATION
  	// ch1
  	TIM3->CCMR1 |= (0x6 << 4);			// PWM Mode in Capture/Compare register (ch1)
  	TIM3->CCER |= (1 << 0);				// Enable Capture/Compare output (ch1)
  	TIM3->CR1 |= (1 << 0);				// Counter Enable
  	TIM3->CCR1 &= 0;
  	TIM3->CCR1 = 10;				// Duty-cycle (ch1)

       	// GEARMOTOR
     	GPIOB->MODER &= ~(0x03 << 10);          // Clear GPIOB5
      	GPIOB->MODER |= (0x02 << 10);		// Alternate Function (AF) Mode
      	GPIOB->PUPDR &= ~(0x03 << 10);
      	GPIOB->PUPDR |= (0x01 << 10);		// Pull-up
      	GPIOB->AFR[0] |= (0x02 << 20);		// [0]=AFRL, <<20 = pin5, 0x02 = AF2

  	// ch2
  	TIM3->CCMR1 |= (0x6 << 12);			// PWM Mode in Capture/Compare register (ch2)
  	TIM3->CCER |= (1 << 4);				// Enable Capture/Compare output (ch2)
  	TIM3->CR1 |= (1 << 0);			        // Counter Enable
  	TIM3->CCR2 &= 0;
   	TIM3->CCR2 = 0;					// Duty-cycle (ch2)

       	 /* Timer 2 to check the encoder */
       	 RCC->APB1ENR |= (0x01 << 0);	// Clock TIM2
       	 TIM2->PSC = 5999;
       	 TIM2->ARR = 3999;
       	 TIM2->DIER &= ~0x01;
       	 TIM2->DIER|=0x01;					// Interrupt Enable
       	 NVIC_ClearPendingIRQ(TIM2_IRQn);
       	 NVIC_EnableIRQ(TIM2_IRQn);
       	 //__enable_irq();
       	 TIM2->CR1 |= (0x01 << 0);

      /* Manage the interrupts */
       // Enable the clock for SYSCFG (bit 14)
       RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

       SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI6_PA; // External interrupt on GPIOA6 
       SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI5_PA; // External interrupt on GPIOA5
       SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PA; // External interrupt on GPIOA7
       SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PC; // External interrupt on GPIOC2

       EXTI->IMR |= (0x01 << 6) | (0x01 << 5)| (0x01 << 7)| (0x01 << 2);     // Set not masked interrupt
       EXTI->RTSR |= (0x01 << 7);    // Rising Edge
       EXTI->FTSR |= (0x01 << 6) | (0x01 << 5)| (0x01 << 2);    // Falling Edge

       /* Gestione NVIC */
       // PA_5 Y 6 y 10
       NVIC_SetPriority(EXTI9_5_IRQn, 0);
       NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
       NVIC_EnableIRQ(EXTI9_5_IRQn);
       // Abilitazione Interrupt
       __asm volatile ("cpsie i" : : : "memory"); // Change Processor State, Enable Interrupts
       // INTERRUPTION OF ENCODER
     	NVIC_SetPriority(EXTI2_IRQn, 0);
     	NVIC_ClearPendingIRQ(EXTI2_IRQn);
     	NVIC_EnableIRQ(EXTI2_IRQn);

       // Interrupt configuration for ADC
       NVIC_SetPriority(ADC_IRQn, 0);
       NVIC_EnableIRQ(ADC_IRQn);
       // Start ADC in interrupt Mode
       HAL_ADC_Start_IT(&hadc1);

       //ASIGNATION OF PORTS FOR LCD

       // Lcd_PortType ports[] = { D4_GPIO_Port, D5_GPIO_Port, D6_GPIO_Port, D7_GPIO_Port };
         Lcd_PortType ports[] = { GPIOA, GPIOA, GPIOA, GPIOC };
         // Lcd_PinType pins[] = {D4_Pin, D5_Pin, D6_Pin, D7_Pin};
         Lcd_PinType pins[] = {GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15, GPIO_PIN_13};
         //Lcd_HandleTypeDef lcd;
         // Lcd_create(ports, pins, RS_GPIO_Port, RS_Pin, EN_GPIO_Port, EN_Pin, LCD_4_BIT_MODE);
         lcd = Lcd_create(ports, pins, GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_12, LCD_4_BIT_MODE);
         Lcd_clear(&lcd);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (!(GPIOA->IDR >> 10 & 0x1) & z==0){
	  	   GPIOB->ODR  |=  (0x01 << 6);
	  	   GPIOC->ODR  &= ~  (0x01 << 7);
	  	   x=0;
		   start();
          while (y==1){
	           if(emergency == 1){
	        	 Lcd_clear(&lcd);
	        	 Lcd_cursor(&lcd, 0,3);
	        	 Lcd_string(&lcd, "EMERGENCY");
	        	 Lcd_cursor(&lcd, 1,6);
	        	 Lcd_string(&lcd, "STOP");
	          	 EmergencyStop();
	             Lcd_clear(&lcd);
                }
	           velocities();
	  	     if(pid==1 && velocity>0){
	  	    	 pwm=calculate_PID();
	  	    	TIM3->CCR2 &= 0;
	  	    	TIM3->CCR2 = pwm;
	  	     }
	  	     pid=0;
	           Lcd_cursor(&lcd, 0,0);
	  	  	   Lcd_string(&lcd, "Velocity:");
	  	  	   Lcd_cursor(&lcd, 0,9);
	  	  	   Lcd_int(&lcd, velocity);
	  	       Lcd_cursor(&lcd, 0,13);
	  	       Lcd_int(&lcd, ref);
	  	  	   if (ref<10){
		  	  	   Lcd_cursor(&lcd, 0,14);
		  	  	   Lcd_string(&lcd, " ");
	  	  	   }
		  	  	if (ref<100){
		  	  	  Lcd_cursor(&lcd, 0,15);
		  	  	  Lcd_string(&lcd, " ");
		  	  	}

	  	       Lcd_cursor(&lcd, 1,0);
	  	  	   Lcd_string(&lcd, "Inclination:");
	  	  	   if (Pot<10){
		  	  	   Lcd_cursor(&lcd, 1,14);
		  	  	   Lcd_string(&lcd, " ");
	  	  	   }
	  	  	   Lcd_cursor(&lcd, 1,13);
	  	  	   Lcd_int(&lcd, Potshow);
	  	  	   //velocities();
	 		   TIM3->CCR1 &= 0;
	 		   TIM3->CCR1 = 50+Potshow;
	  	  	   if ((GPIOA->IDR >> 10 & 0x1) & Pot<5 & velocity==0){
	  	  		   z=1;
	  	  		   y=0;
	  	  	   }
	  	  }
	  }
	  else if((GPIOA->IDR >> 10 & 0x1) & z==1){
	   		GPIOB->ODR  &= ~  (0x01 << 6);
	   		GPIOC->ODR  &= ~  (0x01 << 7);
	   		Lcd_clear(&lcd);
	   		while  (x<=5){
	   			Lcd_cursor(&lcd, 0,4);
	   		    Lcd_string(&lcd, "GOODBYE");
	   			HAL_Delay (650);
	   			Lcd_clear(&lcd);
	   			GPIOC->ODR ^= (0x01 << 7);
	   			GPIOB->ODR ^= (0x1 << 6);
	   			HAL_Delay (650);
	   			x++;
	   		}
	   		GPIOB->ODR  &= ~  (0x01 << 6);
	   		GPIOC->ODR  &= ~  (0x01 << 7);
	   		z=0;
	   		x=0;
	   		Lcd_clear(&lcd);


	  }
	  if (!(GPIOA->IDR >> 10 & 0x1) & z==1){

		    GPIOC->ODR  &= ~  (0x01 << 7);
			Lcd_cursor(&lcd, 0,0);
		    Lcd_string(&lcd, "PUT SWICH TO OFF");
			Lcd_cursor(&lcd, 1,0);
		    Lcd_string(&lcd, "    TO RESET    ");
			 TIM3->CCR2 &= 0;
			 TIM3->CCR2 = 0;
			 TIM3->CCR1 &= 0;
			 TIM3->CCR1 = 50;
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
  RCC_OscInitStruct.PLL.PLLN = 60;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_10|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC10 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_10|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void EmergencyStop(void){
	GPIOB->ODR  &= ~ (0x01 << 6);
	GPIOC->ODR  |=  (0x01 << 7);
	//HAL_Delay(500);
	velocity=0;
	Pot=0;
	Potshow=0;
	Lcd_clear(&lcd);
	       Lcd_cursor(&lcd, 0,0);
	  	   Lcd_string(&lcd, "Velocity:");
	  	   Lcd_cursor(&lcd, 0,10);
	  	   Lcd_int(&lcd, velocity);
	       Lcd_cursor(&lcd, 1,0);
	  	   Lcd_string(&lcd, "Inclination:");
	  	   Lcd_cursor(&lcd, 1,13);
	  	   Lcd_int(&lcd, 0);
	HAL_Delay(1000);
	emergency = 0;
	y=0;
	z=1;
	//Lcd_clear(&lcd);

}
void start(void){
	   Lcd_cursor(&lcd, 0,4);
	   Lcd_string(&lcd, "WELCOME");
	   HAL_Delay(3000);
	   Lcd_clear(&lcd);
	   Lcd_cursor(&lcd, 0,6);
	   Lcd_string(&lcd, "-3-");
	   HAL_Delay(1000);
	   Lcd_cursor(&lcd, 0,6);
	   Lcd_string(&lcd, "-2-");
	   HAL_Delay(1000);
	   Lcd_cursor(&lcd, 0,6);
	   Lcd_string(&lcd, "-1-");
	   HAL_Delay(1000);
	   y=1;
}
void velocities(void){
	   if (velocity==0){
		   ref=0;
		TIM3->CCR2 &= 0;
		TIM3->CCR2 = 0;
	   }
	   if (velocity==1){
		   ref=54;
		/*TIM3->CCR2 &= 0;
		TIM3->CCR2 = 325;
		HAL_Delay(150);
		TIM3->CCR2 = 150;*/
	   }
	   if (velocity==2){
		   ref=72;
		//TIM3->CCR2 &= 0;
		//TIM3->CCR2 = 300;
	   }
	   if (velocity==3){
		   ref=108;
		//TIM3->CCR2 &= 0;
		//TIM3->CCR2 = 450;
	   }
	   if (velocity==4){
		   ref=135;
		//TIM3->CCR2 &= 0;
		//TIM3->CCR2 = 600;
	   }
	   if (velocity==5){
		   ref=153;
		//TIM3->CCR2 &= 0;
		//TIM3->CCR2 = 750;
	   }
}

int calculate_PID(){

	float error = ref-c*7.5;
	integral += (4*0.4*error);                      //ki=4, ts=0.4
	float u_input = (0.5*error)+integral;			// strategia PI  kp=0.5
	//u_input = anti_windup(u_input, err, sPID);
	if (u_input > u_max){
		u_input = u_max;
		integral-=(4*0.4*error);
	}
	else if (u_input < u_min){
		u_input = u_min;
		integral-=(4*0.4*error);
	}

return (int)u_input;


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
