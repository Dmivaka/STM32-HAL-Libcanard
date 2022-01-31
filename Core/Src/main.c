/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include <stdlib.h>
#include "libcanard/canard.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/primitive/array/Real64_1_0.h"
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
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
CanardInstance 	canard;		// This is the core structure that keeps all of the states and allocated resources of the library instance
CanardTxQueue 	queue;		// Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface

static uint8_t my_message_transfer_id = 0;
CanardPortID const MSG_PORT_ID   = 1620U;
uint32_t test_uptimeSec = 0;

// buffer for serialization of a heartbeat message
size_t hbeat_ser_buf_size = uavcan_node_Heartbeat_1_0_EXTENT_BYTES_;
uint8_t hbeat_ser_buf[uavcan_node_Heartbeat_1_0_EXTENT_BYTES_];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

// Wrappers for using o1heap allocator with libcanard
static void* memAllocate(CanardInstance* const ins, const size_t amount);
static void memFree(CanardInstance* const ins, void* const pointer);

// Application-specific function prototypes
void process_canard_TX_queue(void);

// return useconds - not implemented yet
uint32_t micros(void)
{
  return 0;
}
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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  
  CAN_FilterTypeDef Filter;
  
  // accept all frames - filtration is managed by software
  Filter.FilterIdHigh = 0x0000;
  Filter.FilterIdLow = 0x0000;
  Filter.FilterMaskIdHigh = 0x0000;
  Filter.FilterMaskIdLow = 0x0000;
  Filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  Filter.FilterBank = 0;
  Filter.FilterMode = CAN_FILTERMODE_IDMASK;
  Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  Filter.FilterActivation = ENABLE;
  Filter.SlaveStartFilterBank = 0;
  
  HAL_CAN_ConfigFilter(&hcan1, &Filter);

  // Initialization of a canard instance with the previous allocator
  canard = canardInit(&memAllocate, &memFree);
  canard.node_id = 96;  
  
  queue = canardTxInit(	100,                 		// Limit the size of the queue at 100 frames.
                        CANARD_MTU_CAN_CLASSIC);
  
  CanardRxSubscription subscription; // Transfer subscription state.

  if( canardRxSubscribe((CanardInstance *const)&canard,
                        CanardTransferKindMessage,
                        MSG_PORT_ID,
                        uavcan_primitive_array_Real64_1_0_EXTENT_BYTES_,
                        CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                        &subscription) != 1 )
                        {
                          Error_Handler();
                        }	  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Create a heartbeat message
    uavcan_node_Heartbeat_1_0 test_heartbeat = {.uptime = test_uptimeSec,
                                                .health = {uavcan_node_Health_1_0_NOMINAL},
                                                .mode = {uavcan_node_Mode_1_0_OPERATIONAL}};

    // Serialize the heartbeat message
    if (uavcan_node_Heartbeat_1_0_serialize_(&test_heartbeat, hbeat_ser_buf, &hbeat_ser_buf_size) < 0)
    {
      Error_Handler();
    }

    // Create a transfer for the heartbeat message
    const CanardTransferMetadata transfer_metadata = {.priority = CanardPriorityNominal,
                                                      .transfer_kind = CanardTransferKindMessage,
                                                      .port_id = uavcan_node_Heartbeat_1_0_FIXED_PORT_ID_,
                                                      .remote_node_id = CANARD_NODE_ID_UNSET,
                                                      .transfer_id = my_message_transfer_id,};

    if(canardTxPush(&queue,               	// Call this once per redundant CAN interface (queue)
                    &canard,
                    0,     					// Zero if transmission deadline is not limited.
                    &transfer_metadata,
                    hbeat_ser_buf_size,		// Size of the message payload (see Nunavut transpiler)
                    hbeat_ser_buf) < 0 )
                    {
                      Error_Handler();
                    }
    
    // Block for a second before generating the next transfer
    uint32_t timestamp = HAL_GetTick();
    while( HAL_GetTick() < timestamp + 1000u )
    {
      process_canard_TX_queue();
      HAL_Delay(10);
    }

    // Increase uptime
    test_uptimeSec++;
    // Increment the transfer_id variable
    my_message_transfer_id++;

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void process_canard_TX_queue(void)
{
  // Look at top of the TX queue of individual CAN frames
  for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek(&queue)) != NULL;)
  {
    if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > micros()))  // Check the deadline.
    {
      /* Instantiate a frame for the media layer */
      CAN_TxHeaderTypeDef TxHeader;
      TxHeader.IDE = CAN_ID_EXT;
      TxHeader.RTR = CAN_RTR_DATA;
        
      TxHeader.DLC = ti->frame.payload_size;
      TxHeader.ExtId = ti->frame.extended_can_id; 
      
      uint8_t TxData[8];
      uint32_t TxMailbox;      
      
      memcpy( TxData, (uint8_t *)ti->frame.payload, ti->frame.payload_size );

      if ( HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
      {
        break;
      }
    }
    // After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
    canard.memory_free(&canard, canardTxPop(&queue, ti));
  }  
}

#pragma optimize=s none
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8] = {0};
  uint32_t RxFifo1 = 0;
  HAL_CAN_GetRxMessage(&hcan1, RxFifo1, &RxHeader, RxData);
  
  CanardFrame rxf;

  rxf.extended_can_id = RxHeader.ExtId;
  rxf.payload_size = (size_t)RxHeader.DLC;
  rxf.payload = (void*)RxData;

  CanardRxTransfer transfer;

  if( canardRxAccept(   (CanardInstance *const)&canard,
                        micros(),
                        &rxf,
                        0,
                        &transfer,
                        NULL) != 1 )
                        {
                          return ; // the frame received is not a valid transfer
                        }

  uavcan_primitive_array_Real64_1_0 array;
  size_t array_ser_buf_size = uavcan_primitive_array_Real64_1_0_EXTENT_BYTES_;

  if ( uavcan_primitive_array_Real64_1_0_deserialize_( &array, transfer.payload, &array_ser_buf_size) < 0)
  {
    Error_Handler();
  }

  canard.memory_free(&canard, transfer.payload);
  
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);  

  return ;
}

#pragma optimize=s none
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  return ;
}

static void* memAllocate(CanardInstance* const ins, const size_t amount)
{
  (void) ins;
  return malloc(amount);
}

static void memFree(CanardInstance* const ins, void* const pointer)
{
  (void) ins;
  free( pointer );
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
