/* USER CODE BEGIN Header */
/** Jotaro
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

osThreadId Task01Handle;
osThreadId Task02Handle;
osMessageQId Queue1Handle;
/* USER CODE BEGIN PV */
uint16_t moteurstart = 0; // =>1 moteur rentrer,  =>2 moteur sortir,  =>0 moteur arret,  =>3 moteur sequence finis mais pas maintenu


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
void StartTask01(void const * argument);
void StartTask02(void const * argument);

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
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
  TIM3->CCR1 = TIM3->ARR;


	int forcemot = 0, comptpas = 0, pasmax = 17, o = 375, i, pas = 0, ordre =0;
	forcemot = TIM1->ARR /2;

/*	while(1)
	{
		ordre = HAL_GPIO_ReadPin(GPIOB, Pilote_Pin);
		if(ordre == 1)
			  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_SET);
	}*/


/*
	  while(1)//(tempEvent.value.v == 20 && rentrer == 0)
	  {
		  	  //informe via led pene en mouvement
			  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_SET);
			  for(i=0;i<3;i++)
			  {
				  //tempEvent = osMessageGet(Queue1Handle, osDelay(1));
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = 0;
				  TIM1->CCR1 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = forcemot;
				  TIM1->CCR3 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = 0;
				  TIM1->CCR3 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = forcemot;
				  TIM1->CCR2 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = 0;
				  TIM1->CCR2 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = forcemot;
				  TIM1->CCR4 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = 0;
				  TIM1->CCR4 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = forcemot;
				  TIM1->CCR1 = forcemot;
				  o=o-20;
			  }
			  while(pas < pasmax)
			  {
				  //		tempEvent = osMessageGet(Queue1Handle, osDelay(1));
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = 0;
				  TIM1->CCR1 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = forcemot;
				  TIM1->CCR3 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = 0;
				  TIM1->CCR3 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = forcemot;
				  TIM1->CCR2 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = 0;
				  TIM1->CCR2 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = forcemot;
				  TIM1->CCR4 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = 0;
				  TIM1->CCR4 = forcemot;
				  pas=pas;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = forcemot;
				  TIM1->CCR1 = forcemot;
				  pas++;
			  }
			  for(i=0;i<=3;i++)
			  {
				  //  tempEvent = osMessageGet(Queue1Handle, osDelay(1));
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = 0;
				  TIM1->CCR1 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = forcemot;
				  TIM1->CCR3 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = 0;
				  TIM1->CCR3 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = forcemot;
				  TIM1->CCR2 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = 0;
				  TIM1->CCR2 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = forcemot;
				  TIM1->CCR4 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = 0;
				  TIM1->CCR4 = forcemot;
				  o=o;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = forcemot;
				  TIM1->CCR1 = forcemot;
				  o=o+20;
			  }
			  //controle frequence enable
			  TIM3->PSC = 1;
			  TIM3->ARR = 60;
			  // maintient en rentrer
			  // timer enable
			  TIM3->CCR1 = 40; //TIM3->ARR;

			  TIM1->PSC = 1;
			  TIM1->ARR = 60;
			  // timer sortie moteur
			  TIM1->CCR1 = 9;
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = 0;
			  TIM1->CCR4 = 0;
			  TIM3->PSC = 6;
			  TIM3->ARR = 10;

			  while(1)
			  {
				  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_SET);
				  TIM3->PSC = 4;
				  TIM3->ARR = 4;
			  }
	  }
*/

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of Queue1 */
  osMessageQDef(Queue1, 16, uint16_t);
  Queue1Handle = osMessageCreate(osMessageQ(Queue1), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of Task01 */
  osThreadDef(Task01, StartTask01, osPriorityBelowNormal, 0, 80);
  Task01Handle = osThreadCreate(osThread(Task01), NULL);

  /* definition and creation of Task02 */
  osThreadDef(Task02, StartTask02, osPriorityBelowNormal, 0, 80);
  Task02Handle = osThreadCreate(osThread(Task02), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 60;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  TIM1->CCR3 = 0;
  TIM1->CCR4 = 0;

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim3.Init.Prescaler = 8;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  //active les timer pour les fourches optiques
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  //active les timer pour les enables
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

    // timer pour les fourches optiques
    TIM3->CCR3 = TIM3->ARR/2;
    TIM3->CCR4 = TIM3->ARR/2;

    //timer pour les enables
    TIM3->CCR1 = TIM3->ARR/2; // ENB


  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOA, Led_defaut_code_Pin|Led_defaut_ressort_Pin|Led_pene_mouvement_Pin|Led_pene_IN_Pin
                          |Led_pene_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENA_Pin|Taquet_IN_Pin|Taquet_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : FO2_IN_Pin FO1_IN_Pin */
  GPIO_InitStruct.Pin = FO2_IN_Pin|FO1_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_defaut_code_Pin Led_defaut_ressort_Pin Led_pene_mouvement_Pin Led_pene_IN_Pin
                           Led_pene_OUT_Pin */
  GPIO_InitStruct.Pin = Led_defaut_code_Pin|Led_defaut_ressort_Pin|Led_pene_mouvement_Pin|Led_pene_IN_Pin
                          |Led_pene_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA_Pin Taquet_IN_Pin Taquet_OUT_Pin */
  GPIO_InitStruct.Pin = ENA_Pin|Taquet_IN_Pin|Taquet_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Pilote_Pin */
  GPIO_InitStruct.Pin = Pilote_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Pilote_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

int fourcheoptique(uint16_t choix)
{
	uint16_t x = 0, y = 0, compt = 0, nbrdetest = 100, etat = 0, etat1 = 0; //variabel de compteur
	// frequence du timer
	TIM3->PSC = 8;
	TIM3->ARR = 1000;
	// timer enable
	TIM3->CCR1 = TIM3->ARR;
	while(compt<nbrdetest) // on vÃ©rifie 50 fois lÃ©tat logique du signal
	{
		x = HAL_GPIO_ReadPin(GPIOA, FO1_IN_Pin); // on lit la fourche optique 1
		y = HAL_GPIO_ReadPin(GPIOA, FO2_IN_Pin); // on lit la fourche optique 2
		etat = etat + x;
		etat1 = etat1 + y;
		compt++;
	}
	if (etat == nbrdetest && choix == 1)
		return 1; // detection d'obstacle
	else if (etat < nbrdetest && choix == 1)
		return 2; // rien Ã  l'horizon
	else if (etat1 == nbrdetest && choix == 0)
		return 10; // detection d'obstacle
	else if (etat1 < nbrdetest && choix == 0)
		return 20; // rien Ã  l'horizon
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void const * argument)
{
  /* USER CODE BEGIN 5 */
	int fo1 = 0, fo2 = 0, ordre = 0, rentrer = 0;
	  /* Infinite loop */
	  for(;;)
	  {
		  //reception d'un ordre
		  HAL_GPIO_WritePin(GPIOB, Taquet_IN_Pin, SET);
		  ordre = HAL_GPIO_ReadPin(GPIOB, Pilote_Pin);
		  //Si ordre == 1 alors le pene doit rentrer
		  //SI ordre == 0 alors le pene doit sortir
		  if(ordre == 1 && rentrer == 0)
		  {
			  // on verifie si le pene est sortie
			  fo1 = fourcheoptique(1);
			  if(fo1 == 1)
			  {
				  // on demarre le moteur en envoyant un message vers la 2eme tache
				  moteurstart = 1; // moteur demarre la rentrer

				  while(fo1 != 10 && moteurstart == 1)
				  {
					  //reception message 63 pas fait avec la variable global start moteur
					  // on verifie si le pene est rentrer
					  fo1 = fourcheoptique(0);
					  osDelay(10);
				  }
				  // a parir de la le pen est bloque en position rentrer
				  // variable pour dire si le pene est rentrer ou non
				  rentrer = 1;
				  moteurstart = 0;
				  //envoyer au maitre que le taquet est rentrer
				  HAL_GPIO_WritePin(GPIOB, Taquet_IN_Pin, SET);
				  HAL_GPIO_WritePin(GPIOB, Taquet_OUT_Pin, RESET);
				  //informer pene rentrer
				  HAL_GPIO_WritePin(GPIOA, Led_pene_IN_Pin, SET);
				  HAL_GPIO_WritePin(GPIOA, Led_pene_OUT_Pin, RESET);
			  }
		  }// Fin ordre de rentrer
		  else if(ordre == 0 && rentrer == 1)
		  {
			  moteurstart = 5;
			  // on verifie si le pene est rentrer
			  fo1 = fourcheoptique(0);
			  if(fo1 == 10)//si rentrer
			  {
				  //informe pene en mouvement
				  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_SET);
				  //desactive les sorties pour laisser le ressort sortir
				  TIM1->CCR3 = 0;
				  TIM1->CCR2 = 0;
				  TIM1->CCR4 = 0;
				  TIM1->CCR1 = 0;
				  //desactivation des enables
				  HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_RESET);
				  TIM3->CCR1 = 0; // => ENB desactiver
				  HAL_Delay(1);//temps pour laisser le temps au pene de se faire voir par la fourche
				  //activation des enables
				  TIM3->CCR1 = TIM3->ARR; // => ENB activer
				  //active la sortie
				  TIM1->CCR4 = TIM1->ARR/2;
				  HAL_Delay(100);//temps pour laisser le temps au pene de se faire voir par la fourche
				  TIM1->CCR4 = 0;//relache la sorti totalement
				  TIM3->CCR1 = 0; // => ENB desactiver
				  HAL_Delay(50);//temps pour laisser le temps au pene de se faire voir par la fourche
				  //savoir si le pene est bien sorti
				  fo1 = fourcheoptique(1);
				  if(fo1 == 1)//le pen est bien sorti
				  {
					  //envoyer au maitre que le taquer est sorti
					  HAL_GPIO_WritePin(GPIOB, Taquet_OUT_Pin, SET);
					  HAL_GPIO_WritePin(GPIOB, Taquet_IN_Pin, RESET);
					  //informe pene s'arrete
					  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_RESET);
					  HAL_GPIO_WritePin(GPIOA, Led_defaut_ressort_Pin , GPIO_PIN_RESET);
					  //informer pene rentrer
					  HAL_GPIO_WritePin(GPIOA, Led_pene_IN_Pin, RESET);
					  HAL_GPIO_WritePin(GPIOA, Led_pene_OUT_Pin, SET);
				  }
				  else //probleme de ressort sort manuellement le pene
				  {
					  HAL_GPIO_WritePin(GPIOA, Led_defaut_ressort_Pin , GPIO_PIN_SET);
					  //enable pour faire fonctionner le moteur
					  HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
					  TIM3->CCR1 = TIM3->ARR; // => ENB activer
					  // on demarre le moteur en envoyant un message vers la 2eme tache
					  moteurstart = 2;
					  while(moteurstart == 2 && fo1 != 1)
					  {
						  //reception message 63 pas fait avec la variable global moteurstart
						  // on verifie si le pene est rentrer
						  fo1 = fourcheoptique(1);
						  osDelay(10);
					  }
					  //envoyer au maitre que le taquet est rentrer
					  HAL_GPIO_WritePin(GPIOB, Taquet_IN_Pin, RESET);
					  HAL_GPIO_WritePin(GPIOB, Taquet_OUT_Pin, SET);
					  //informer pene rentrer
					  HAL_GPIO_WritePin(GPIOA, Led_pene_IN_Pin, RESET);
					  HAL_GPIO_WritePin(GPIOA, Led_pene_OUT_Pin, SET);
				  }// fin resortir le pen manuellement
				  //active les enables
				  HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
				  TIM3->CCR1 = TIM3->ARR;
			  }
			  rentrer = 0;
		  }// fin ordre de sorti*/

		  fo1 = fourcheoptique(0);
		  fo2 = fourcheoptique(1);
		  if(fo2 == 2 && fo1 == 20 && rentrer == 1)
		  {
			  moteurstart = 1;
			  while(fo1 == 20 && fo2 == 2)
			  {
				  fo1 = fourcheoptique(0);
				  fo2 = fourcheoptique(1);
			  }
			  HAL_GPIO_WritePin(GPIOA, Led_defaut_ressort_Pin, SET);
			  moteurstart = 0;
			  TIM1->CCR1 = 9;
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = 0;
			  TIM1->CCR4 = 0;
		  }
		  else if(fo1 == 20 && fo2 == 2 && rentrer == 0)
		  {
			  moteurstart = 2;
			  while(fo1 == 20 && fo2 == 2)
			  {
				  fo1 = fourcheoptique(0);
				  fo2 = fourcheoptique(1);
			  }
			  HAL_GPIO_WritePin(GPIOA, Led_defaut_ressort_Pin, SET);
			  moteurstart = 0;
			  TIM1->CCR1 = 0;
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = 0;
			  TIM1->CCR4 = 0;
		  }
		  else
			  HAL_GPIO_WritePin(GPIOA, Led_defaut_code_Pin|Led_defaut_ressort_Pin, RESET);

	  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument)
{
  /* USER CODE BEGIN StartTask02 */
	int forcemot, comptpas = 0, pasmax = 6, o = 375, i, pas = 0, rentre = 10;
	forcemot = TIM1->ARR /2;
  /* Infinite loop */
  for(;;)
  {
	  comptpas = 0;
	  pas = 0;
	  if(moteurstart == 1) //moteur en mode rentrer le pene
	  {
		  //informe pene en mouvement
		  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_SET);
		  //active les enables
		  HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
		  TIM3->CCR1 = TIM3->ARR;
		  for(i=0;i<3 && moteurstart == 1;i++)
		  {
			  TIM1->CCR4 = 0;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = forcemot;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = 0;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = forcemot;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = 0;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = forcemot;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = 0;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = forcemot;
			  TIM1->CCR1 = forcemot;
			  o=o-20;
		  }
		  for(pas = 0; pas < pasmax  && moteurstart == 1; pas++)
		  {
			  TIM1->CCR4 = 0;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = forcemot;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = 0;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = forcemot;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = 0;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = forcemot;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = 0;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = forcemot;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
		  }
		  for(i=0;i<=3 && moteurstart == 1;i++)
		  {
				  TIM1->CCR4 = 0;
				  TIM1->CCR1 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = forcemot;
				  TIM1->CCR3 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR1 = 0;
				  TIM1->CCR3 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = forcemot;
				  TIM1->CCR2 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR3 = 0;
				  TIM1->CCR2 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = forcemot;
				  TIM1->CCR4 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR2 = 0;
				  TIM1->CCR4 = forcemot;
				  for(comptpas=0; comptpas < o; comptpas++);
				  TIM1->CCR4 = forcemot;
				  TIM1->CCR1 = forcemot;
				  o=o+20;
				  for(comptpas=0; comptpas < o; comptpas++);
		  }
		  	  moteurstart = 3; // fin séquence mais pas encore maintenue
			  //controle frequence enable
			  TIM3->PSC = 1;
			  TIM3->ARR = 60;
			  // maintient en rentrer
			  // timer enable
			  TIM3->CCR1 = 40; //TIM3->ARR;

			  TIM1->PSC = 1;
			  TIM1->ARR = 60;
			  // timer sortie moteur
			  TIM1->CCR1 = 9;
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = 0;
			  TIM1->CCR4 = 0;

			  moteurstart = 0 ; //dit que la séquence est terminé moteur maintenu

			  TIM3->PSC = 6;
			  TIM3->ARR = 10;
			  pas = 0;
			  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_RESET);
			  rentre = 1;
			  while(rentre == 1)
			  {
				  if(moteurstart == 5)
					  rentre = 0;
			  }//fin sortir pene
	  }//stop rentrer le moteur
	  else if (moteurstart == 2 && rentre == 0) // sortir le moteur
	  {
		  //informe via led pene en mouvement
		  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_SET);
		  //active les enables
		  HAL_GPIO_WritePin(GPIOB, ENA_Pin, GPIO_PIN_SET);
		  TIM3->CCR1 = TIM3->ARR;

		  for(i=0;i<3 && moteurstart == 2; i++)//envoyer au maitre que le taquet est rentrer
		  {
			  TIM1->CCR1 = 0;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = forcemot;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = 0;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = forcemot;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = forcemot;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = 0;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = forcemot;
			  TIM1->CCR4 = forcemot;
			  o=o+10;
			  for(comptpas=0; comptpas < o; comptpas++);
		  }
		  for(pas = 0; pas < pasmax && moteurstart == 2; pas++)
		  {
			  // blocage sans bruit nickel
			  // le moteur n'arrive pas a entrener le pene
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = 0;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = forcemot;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = 0;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = forcemot;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = forcemot;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = 0;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = forcemot;
			  TIM1->CCR4 = forcemot;
		  }
		  for(i=0;i<=3 && moteurstart == 2;i++)
		  {
			  TIM1->CCR1 = 0;
			  TIM1->CCR4 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = forcemot;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR4 = 0;
			  TIM1->CCR2 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = forcemot;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR2 = 0;
			  TIM1->CCR3 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = forcemot;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR3 = 0;
			  TIM1->CCR1 = forcemot;
			  for(comptpas=0; comptpas < o; comptpas++);
			  TIM1->CCR1 = forcemot;
			  TIM1->CCR4 = forcemot;
			  o=o+10;
			  for(comptpas=0; comptpas < o; comptpas++);
		  }
		  HAL_GPIO_WritePin(GPIOA, Led_pene_mouvement_Pin , GPIO_PIN_RESET);
		  moteurstart = 0;
		  rentre =1;
	  }//fin sortir le pene

  }// fin for ()
  /* USER CODE END StartTask02 */
}

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
