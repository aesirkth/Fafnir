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

FDCAN_HandleTypeDef hfdcan1;

TIM_HandleTypeDef htim3;


/* USER CODE BEGIN PV */

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t	TxData[8];
uint8_t	RxData[8];


typedef enum {
    STATE_IDLE,
    STATE_INIT,
    STATE_READY,
    STATE_ACTUATE
} State;

typedef enum {
    CMD_NONE,
    CMD_SERVO_ZERO,
    CMD_SERVO_ROTATE,
    CMD_PYRO_ACTUATE,
    CMD_PYRO_DISABLE
} CommandType;

typedef struct {
    CommandType type;
    uint8_t pyroIndex;
    uint16_t angle;
} Command;

State servoState = STATE_IDLE;
State pyroState[NUM_PYROS] = {STATE_IDLE, STATE_IDLE, STATE_IDLE};
uint8_t pyroMask = 0b000;
uint16_t targetAngle = 0;

GPIO_TypeDef* const pyroPort = GPIOA;
const uint16_t pyroPins[NUM_PYROS] = {GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10};

GPIO_TypeDef* const pyroDetectPort = GPIOB;
const uint16_t pyroDetectPins[NUM_PYROS] = {GPIO_PIN_15, GPIO_PIN_14, GPIO_PIN_13};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM3_Init(void);
static void MX_ICACHE_Init(void);
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
  MX_FDCAN1_Init();
  MX_TIM3_Init();
  MX_ICACHE_Init();
  /* USER CODE BEGIN 2 */

   // Start FDCAN1 peripheral
   if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Error_Handler();
   }

   // Enable RX FIFO 0 Notification for incoming messages
   if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
   }


   Command test = {CMD_SERVO_ZERO, 0, 0}; //This should be ignored since servo is still in STATE_IDLE (and actually all systems are)
   processCommand(test);


   forceInit();
   processCommand(test);



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {

	    handleServo();

	    for (int i = 0; i < NUM_PYROS; i++) {
	        handlePyro(i);
	    }

	    /*
	    static uint32_t lastStatusTime = 0;

	    uint32_t now = HAL_GetTick();
	    if (now - lastStatusTime > 500) { //every 0.5 seconds
	        lastStatusTime = now;

	        uint8_t ready = systemReady();
	        uint8_t idle  = systemIdle();

	        uint8_t data[2] = {ready, idle};
	        can_send_std(CAN_ID_STATUS, data, sizeof(data));
	    }

	    */

	    HAL_Delay(20);


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV2;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 9;
  hfdcan1.Init.NominalSyncJumpWidth = 2;
  hfdcan1.Init.NominalTimeSeg1 = 14;
  hfdcan1.Init.NominalTimeSeg2 = 5;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 2;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);

  FDCAN_FilterTypeDef sFilterConfig;

  // Filter for CAN_ID_SERVO_ZERO
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = CAN_ID_SERVO_ZERO ;
  sFilterConfig.FilterID2 = 0x7FF;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  // Filter for CAN_ID_SERVO_ROTATE
   sFilterConfig.IdType = FDCAN_STANDARD_ID;
   sFilterConfig.FilterIndex = 1;
   sFilterConfig.FilterType = FDCAN_FILTER_MASK;
   sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
   sFilterConfig.FilterID1 = CAN_ID_SERVO_ROTATE;
   sFilterConfig.FilterID2 = 0x7FF;
   HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig);

  TxHeader.IdType              = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
  TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;
  TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker       = 0;


  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 90-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA8
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */


void processCommand(Command cmd) {

	if (systemIdle()) return; //not so sure abt this


    switch (cmd.type) {
        case CMD_SERVO_ZERO:
            servoState = STATE_INIT;
            break;

        case CMD_SERVO_ROTATE:
            targetAngle = cmd.angle;
            if (servoState == STATE_READY) servoState = STATE_ACTUATE;
            break;

        case CMD_PYRO_ACTUATE:
            if (cmd.pyroIndex < NUM_PYROS && pyroState[cmd.pyroIndex] == STATE_READY)
                pyroState[cmd.pyroIndex] = STATE_ACTUATE;
            break;

        case CMD_PYRO_DISABLE:
            if (cmd.pyroIndex < NUM_PYROS)
                pyroState[cmd.pyroIndex] = STATE_INIT;
            break;

        default:
            break;
    }
}

uint8_t systemReady(void) {
    if (servoState != STATE_READY)
        return 0;

    for (int i = 0; i < NUM_PYROS; i++) {
        if (pyroState[i] != STATE_READY)
            return 0;
    }

    return 1;
} //essentially just check that everything is in the READY state for the first time, send this via CAN to Fjalar

uint8_t systemIdle(void) {
	  if (servoState == STATE_IDLE) return 1;

	    for (int i = 0; i < NUM_PYROS; i++) {
	        if (pyroState[i] == STATE_IDLE)
	            return 1;
	    }

	    return 0;
}

void handleServo(void) {
    switch (servoState) {
        case STATE_IDLE:
            //Do nothing until we receive some initial command
            break;

        case STATE_INIT:
            servoZero();
            servoState = STATE_READY;
            break;

        case STATE_READY:
            //Wait for rotation command
            break;

        case STATE_ACTUATE:
            servoRotate(targetAngle);
            printf("Servo Rotated\n");
            servoState = STATE_READY;
            break;
    }
}

void handlePyro(int i) {
    switch (pyroState[i]) {
        case STATE_IDLE:
            //Do nothing until we receive some initial command
            break;

        case STATE_INIT:
            pyroActuate(i, 0);
            if (pyroDetect(i) == 0) { //Low = continuity detected
                pyroState[i] = STATE_READY;
                printf("[Pyro %d] Continuity OK\n", i);
            }
            break;

        case STATE_READY:
            // WAit for actuation command
            break;

        case STATE_ACTUATE:
            pyroActuate(i, 1);
            //remain active until the CMD_PYRO_DISABLE is received
            break;
    }
}

void forceInit(void) {
	//USED ONLY FOR DEBUGGING
	servoState = STATE_INIT;
	for (int i = 0; i < NUM_PYROS; i++)
	    pyroState[i] = STATE_INIT;
}

void pyroActuate(uint8_t index, uint8_t state) {
	if (index >= NUM_PYROS) return;
	HAL_GPIO_WritePin(pyroPort, pyroPins[index], state ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (state) {
    	pyroMask |=  (1 << index); //set union
    } else {
    	pyroMask &= ~(1 << index); //set intersection
    }

}

uint8_t pyroDetect(uint8_t index) {
	if (index >= NUM_PYROS) return 69; //69 means error
    //LOW = continuity detected (return 0)
    //HIGH = open circuit (return 1)

	GPIO_PinState state = HAL_GPIO_ReadPin(pyroDetectPort, pyroDetectPins[index]);

    return (state == GPIO_PIN_RESET) ? 0 : 1;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t it)
{
    if ((it & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx, d) != HAL_OK) return;

    uint8_t n = getDataLength(&rx);

    switch (rx.Identifier) {

      case CAN_ID_SERVO_ZERO:
         if (n >= 1 && d[0] == 1) {
             //servoZeroRequested = true;
         }
        break;

      case CAN_ID_SERVO_ROTATE:
        if (n >= 2) {
          //int dir = d[0] ? +1 : -1;
          //uint8_t rotation = d[1];
          //servoAngle = (float)(dir * rotation);
          //servoRotateRequested = true;
        }
        break;

      default:
        // Optionally increment error counter here
        break;
    }
}


void can_send_std(uint16_t id, const uint8_t *data, uint8_t len)
{
    TxHeader.Identifier = id;
    switch (len) {
      case 0:  TxHeader.DataLength = FDCAN_DLC_BYTES_0; break;
      case 1:  TxHeader.DataLength = FDCAN_DLC_BYTES_1; break;
      case 2:  TxHeader.DataLength = FDCAN_DLC_BYTES_2; break;
      case 3:  TxHeader.DataLength = FDCAN_DLC_BYTES_3; break;
      case 4:  TxHeader.DataLength = FDCAN_DLC_BYTES_4; break;
      case 5:  TxHeader.DataLength = FDCAN_DLC_BYTES_5; break;
      case 6:  TxHeader.DataLength = FDCAN_DLC_BYTES_6; break;
      case 7:  TxHeader.DataLength = FDCAN_DLC_BYTES_7; break;
      default: TxHeader.DataLength = FDCAN_DLC_BYTES_8; break;
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader,
                                  (len && data) ? (uint8_t*)data : NULL);
}

uint8_t getDataLength(FDCAN_RxHeaderTypeDef *hdr) {
  switch (hdr->DataLength) {
    case FDCAN_DLC_BYTES_0: return 0;
    case FDCAN_DLC_BYTES_1: return 1;
    case FDCAN_DLC_BYTES_2: return 2;
    case FDCAN_DLC_BYTES_3: return 3;
    case FDCAN_DLC_BYTES_4: return 4;
    case FDCAN_DLC_BYTES_5: return 5;
    case FDCAN_DLC_BYTES_6: return 6;
    case FDCAN_DLC_BYTES_7: return 7;
    case FDCAN_DLC_BYTES_8: return 8;
    default: return 0;
  }
}


void setPWM(TIM_HandleTypeDef *timer_handle, uint32_t timer_channel, float duty) {

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(timer_handle);
    uint32_t ccr = (uint32_t)((duty * (float)(arr + 1U)) + 0.5f);
    if (ccr > arr) ccr = arr;
    __HAL_TIM_SET_COMPARE(timer_handle, timer_channel, ccr);
}

void servoZero(void) {
	HAL_TIM_PWM_Start(&MOTOR_TIMER_HANDLE, MOTOR_TIMER_CHANNEL);
	servoRotate(0.0f);
}

void servoRotate(float angle) {
	//The angle is mapped to -135 to 135 to properly represent CW and CCW rotations
	//Input of +90 == 90 deg rotation CW from the zero position.

	if (angle < -135 || angle > 135) angle = 0.0f;
	angle = 135.0f + angle; //135 degrees is the zero/middle position, since the servo motor can rotate 270 deg

	float degRatio = angle / 270.0f;

	TIM_HandleTypeDef *htim = &MOTOR_TIMER_HANDLE;
	float duty = degRatio * 0.10f + 0.025f;  //mapping to 2.5%–12.5% duty cycle
	setPWM(htim, MOTOR_TIMER_CHANNEL, duty);

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
