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
#include "libcanard/canard.h"
#include "uavcan/node/Heartbeat_1_0.h"
#include "uavcan/node/ExecuteCommand_1_1.h"
#include "uavcan/_register/List_1_0.h"
#include "uavcan/_register/Access_1_0.h"
#include "uavcan/primitive/array/Real32_1_0.h"
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

TIM_HandleTypeDef htim7;

/* USER CODE BEGIN PV */
CanardInstance 	canard;		// This is the core structure that keeps all of the states and allocated resources of the library instance
CanardTxQueue 	queue;		// Prioritized transmission queue that keeps CAN frames destined for transmission via one CAN interface

// buffer for serialization of heartbeat message
size_t hbeat_ser_buf_size = uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_;
uint8_t hbeat_ser_buf[uavcan_node_Heartbeat_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_];

CanardPortID const MSG_PORT_ID   = 1620U;

uint8_t MCU_restart = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_TIM7_Init(void);
/* USER CODE BEGIN PFP */

// Wrappers for using memory allocator with libcanard
static void *memAllocate(CanardInstance *const canard, const size_t amount);
static void memFree(CanardInstance *const canard, void *const pointer);

// Application-specific function prototypes
void process_canard_TX_queue(void);
void processReceivedTransfer(int32_t redundant_interface_index, CanardRxTransfer *transfer);

extern uint64_t TIM7_ITs;
CanardMicrosecond micros()
{ 
  return (CanardMicrosecond)(__HAL_TIM_GET_COUNTER(&htim7) + 50000u * TIM7_ITs);
}

uint8_t LengthDecoder( uint32_t length );
uint32_t LengthCoder( uint8_t length );
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
  // UAVCAN initialization
  canard = canardInit(&memAllocate, &memFree);	// Initialization of a canard instance
  canard.node_id = 42;
  
  // Limit the size of the queue at 100 frames , set MTU CANARD_MTU_CAN_FD = 64 bytes
  queue = canardTxInit(	100, CANARD_MTU_CAN_FD);
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
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  
  FDCAN_FilterTypeDef sFilterConfig;  
  sFilterConfig.IdType = FDCAN_EXTENDED_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x0;
  sFilterConfig.FilterID2 = 0x1FFFFFFF;
  
  // Configure CAN frames filtering 
  if( HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK ){ Error_Handler(); }  

  // Configure global filter: Filter all remote frames with STD and EXT ID. Reject non matching frames with STD ID and EXT ID
  if( HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK ){ Error_Handler(); }
  
  // Activate Rx FIFO 0 new message notification
  if( HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK ){ Error_Handler(); }        
  
  // Configure and enable Tx Delay Compensation, required for BRS mode.
  if( HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, 5, 0) != HAL_OK){ Error_Handler(); }
  if( HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK){ Error_Handler(); }
  
    // Start FDCAN periphery
  if( HAL_FDCAN_Start(&hfdcan1) != HAL_OK ){ Error_Handler(); }
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  CanardRxSubscription command_subscription;
  if( canardRxSubscribe(        &canard,
                                CanardTransferKindRequest,
                                uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_,
                                uavcan_node_ExecuteCommand_Request_1_1_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &command_subscription) != 1 ){ Error_Handler(); }
  
  CanardRxSubscription reg_list_subscription;
  if( canardRxSubscribe(        &canard,
                                CanardTransferKindRequest,
                                uavcan_register_List_1_0_FIXED_PORT_ID_,
                                uavcan_register_List_Request_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &reg_list_subscription) != 1 ){ Error_Handler(); }
  
  CanardRxSubscription reg_access_subscription;
  if( canardRxSubscribe(        &canard,
                                CanardTransferKindRequest,
                                uavcan_register_Access_1_0_FIXED_PORT_ID_,
                                uavcan_register_Access_Request_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &reg_access_subscription) != 1 ){ Error_Handler(); }
  
  CanardRxSubscription rx_message_subscription;
  if( canardRxSubscribe(        &canard,
                                CanardTransferKindMessage,
                                MSG_PORT_ID,
                                uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_,
                                CANARD_DEFAULT_TRANSFER_ID_TIMEOUT_USEC,
                                &rx_message_subscription) != 1 ){ Error_Handler(); }	  
  
  static uint8_t my_message_transfer_id = 0;
  
  while (1)
  {
    // Create a heartbeat message
    uavcan_node_Heartbeat_1_0 test_heartbeat = {.uptime = micros()/1000000u,
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
    
    // unreliable impelementation 
    if( MCU_restart )
    {
      NVIC_SystemReset();
    }

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV2;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 24;
  hfdcan1.Init.NominalTimeSeg1 = 55;
  hfdcan1.Init.NominalTimeSeg2 = 24;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 5;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 160;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 50000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */
  HAL_TIM_Base_Start_IT(&htim7);
  /* USER CODE END TIM7_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_Pin */
  GPIO_InitStruct.Pin = BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void process_canard_TX_queue(void)
{
  // Look at top of the TX queue of individual CAN frames
  for (const CanardTxQueueItem* ti = NULL; (ti = canardTxPeek(&queue)) != NULL;)
  {
    if ((0U == ti->tx_deadline_usec) || (ti->tx_deadline_usec > micros()))  // Check the deadline.
    {
      FDCAN_TxHeaderTypeDef TxHeader;
      
      if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) != 0)
      {
        // Add message to Tx FIFO 
        TxHeader.Identifier = ti->frame.extended_can_id;
        TxHeader.IdType = FDCAN_EXTENDED_ID;
        TxHeader.TxFrameType = FDCAN_DATA_FRAME;
        TxHeader.DataLength = LengthCoder( ti->frame.payload_size );
        TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
        TxHeader.BitRateSwitch = FDCAN_BRS_ON;
        TxHeader.FDFormat = FDCAN_FD_CAN;
        TxHeader.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
        TxHeader.MessageMarker = 0x00;
        if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, (uint8_t *)ti->frame.payload) != HAL_OK)
        {
          Error_Handler();
        }
      }
    }
    // After the frame is transmitted or if it has timed out while waiting, pop it from the queue and deallocate:
    canard.memory_free(&canard, canardTxPop(&queue, ti));
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader = {0};
  uint8_t RxData[64];
  
  if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK){ Error_Handler(); }

  CanardFrame rxf;

  rxf.extended_can_id = (uint32_t)RxHeader.Identifier;
  rxf.payload_size = (size_t)LengthDecoder(RxHeader.DataLength);
  rxf.payload = (void*)RxData;

  CanardRxTransfer transfer;

  int8_t result = canardRxAccept(       &canard,
                                        micros(),
                                        &rxf,
                                        0,
                                        &transfer,
                                        NULL);

  if (result < 0)
  {
    // An error has occurred: either an argument is invalid or we've ran out of memory.
    // It is possible to statically prove that an out-of-memory will never occur for a given application if
    // the heap is sized correctly; for background, refer to the Robson's Proof and the documentation for O1Heap.
    // Reception of an invalid frame is NOT an error.
  }
  else if (result == 1)
  {
    processReceivedTransfer(0, &transfer);              // A transfer has been received, process it.
    canard.memory_free(&canard, transfer.payload);      // Deallocate the dynamic memory afterwards.
  }
  else
  {
    // Nothing to do.
    // The received frame is either invalid or it's a non-last frame of a multi-frame transfer.
    // Reception of an invalid frame is NOT reported as an error because it is not an error.
  }
  
  return ;
}

void rx_message_callback(CanardRxTransfer *transfer);
void execute_command_callback(CanardRxTransfer *transfer);
void register_list_callback(CanardRxTransfer *transfer);
void register_access_callback(CanardRxTransfer *transfer);

void processReceivedTransfer(int32_t redundant_interface_index, CanardRxTransfer *transfer)
{
  if( transfer->metadata.port_id == MSG_PORT_ID)
  {
    // process incoming real32 array
    rx_message_callback(transfer);
  }
  else if( transfer->metadata.port_id == uavcan_node_ExecuteCommand_1_1_FIXED_PORT_ID_)
  {
    // Got command request!
    execute_command_callback(transfer);
  }
  else if( transfer->metadata.port_id == uavcan_register_List_1_0_FIXED_PORT_ID_)
  {
    // Got register list request! 
    register_list_callback(transfer);
  }
  else if( transfer->metadata.port_id == uavcan_register_Access_1_0_FIXED_PORT_ID_)
  {
    // Got register list request! 
    register_access_callback(transfer);
  }
  else
  {
    // Received unknown completed transfer
  }
  
  return ;
}

void rx_message_callback(CanardRxTransfer *transfer)
{
  uavcan_primitive_array_Real32_1_0 array;
  size_t array_ser_buf_size = uavcan_primitive_array_Real32_1_0_EXTENT_BYTES_;

  if ( uavcan_primitive_array_Real32_1_0_deserialize_( &array, transfer->payload, &array_ser_buf_size) < 0 )
  {
    Error_Handler();
  }
}

void execute_command_callback(CanardRxTransfer *transfer)
{
  uavcan_node_ExecuteCommand_Request_1_1 request;
  uavcan_node_ExecuteCommand_Response_1_1 response = {0};
  
  size_t request_ser_buf_size = uavcan_node_ExecuteCommand_Request_1_1_EXTENT_BYTES_;

  if( uavcan_node_ExecuteCommand_Request_1_1_deserialize_(&request, transfer->payload, &request_ser_buf_size ) < 0)
  {
    Error_Handler();
  }
  
  if( request.command == uavcan_node_ExecuteCommand_Request_1_1_COMMAND_RESTART )
  {
    MCU_restart = 1;
    response.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_SUCCESS;
  }
  else
  {
    response.status = uavcan_node_ExecuteCommand_Response_1_1_STATUS_BAD_COMMAND;
  }

  uint8_t c_serialized[uavcan_node_ExecuteCommand_Response_1_1_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
  size_t c_serialized_size = sizeof(c_serialized);

  if ( uavcan_node_ExecuteCommand_Response_1_1_serialize_(&response, &c_serialized[0], &c_serialized_size) < 0)
  {
    Error_Handler();
  }

  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityNominal,
                                                        .transfer_kind  = CanardTransferKindResponse,
                                                        .port_id        = transfer->metadata.port_id,
                                                        .remote_node_id = transfer->metadata.remote_node_id,
                                                        .transfer_id    = transfer->metadata.transfer_id };

  int32_t debug =  canardTxPush(        &queue,               	// Call this once per redundant CAN interface (queue)
                                        &canard,
                                        0,     					// Zero if transmission deadline is not limited.
                                        &transfer_metadata,
                                        c_serialized_size,		// Size of the message payload (see Nunavut transpiler)
                                        c_serialized);
}

void register_list_callback(CanardRxTransfer *transfer)
{
  uavcan_register_List_Request_1_0 request;
  size_t request_ser_buf_size = uavcan_register_List_Request_1_0_EXTENT_BYTES_;

  if( uavcan_register_List_Request_1_0_deserialize_(&request, transfer->payload, &request_ser_buf_size ) < 0)
  {
    Error_Handler();
  }

  uavcan_register_List_Response_1_0 response = {0};

  char led1_register_name[] = "user.led1";
  char led2_register_name[] = "user.led2";

  if( request.index == 0 )
  {
    memcpy(&response.name.name.elements, led1_register_name, sizeof(led1_register_name));
    response.name.name.count = sizeof(led1_register_name);
  }
  else if( request.index == 1 )
  {
    memcpy(&response.name.name.elements, led2_register_name, sizeof(led2_register_name));
    response.name.name.count = sizeof(led2_register_name);
  }
  else
  {
    response.name.name.elements[0] = '\0';
    response.name.name.count = 0;
  }

  uint8_t c_serialized[uavcan_register_List_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
  size_t c_serialized_size = sizeof(c_serialized);

  if ( uavcan_register_List_Response_1_0_serialize_(&response, &c_serialized[0], &c_serialized_size) < 0)
  {
    Error_Handler();
  }

  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityNominal,
                                                        .transfer_kind  = CanardTransferKindResponse,
                                                        .port_id        = transfer->metadata.port_id,
                                                        .remote_node_id = transfer->metadata.remote_node_id,
                                                        .transfer_id    = transfer->metadata.transfer_id };
  
  int32_t debug =  canardTxPush(        &queue,               	// Call this once per redundant CAN interface (queue)
                                        &canard,
                                        0,     					// Zero if transmission deadline is not limited.
                                        &transfer_metadata,
                                        c_serialized_size,		// Size of the message payload (see Nunavut transpiler)
                                        c_serialized);
}

void LED1_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value );
void LED2_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value );

void register_access_callback(CanardRxTransfer *transfer)
{
  uavcan_register_Access_Request_1_0 request;
  uavcan_register_Access_Response_1_0 response = {0};
  
  size_t request_ser_buf_size = uavcan_register_Access_Request_1_0_EXTENT_BYTES_;

  if( uavcan_register_Access_Request_1_0_deserialize_(&request, transfer->payload, &request_ser_buf_size ) < 0)
  {
    Error_Handler();
  }
  
  GPIO_TypeDef * GPIO_port = NULL;
  uint16_t GPIO_pin = 0;
  
  if( !strncmp( (char const *)"user.led1", (char const *)request.name.name.elements, request.name.name.count) )
  {
    LED1_access_handler(&request.value, &response.value);
  }
  else if( !strncmp( (char const *)"user.led2", (char const *)request.name.name.elements, request.name.name.count) )
  {
    LED2_access_handler(&request.value, &response.value);
  }
  else
  {
    // access to non-existent register
    response.value._tag_= 0;
  }

  response.timestamp.microsecond = micros(); // taken along with register READING
  response._mutable = 1;
  response.persistent = 0;
  
  uint8_t c_serialized[uavcan_register_Access_Response_1_0_SERIALIZATION_BUFFER_SIZE_BYTES_] = {0};
  size_t c_serialized_size = sizeof(c_serialized);

  if ( uavcan_register_Access_Response_1_0_serialize_(&response, &c_serialized[0], &c_serialized_size) < 0)
  {
    Error_Handler();
  }

  const CanardTransferMetadata transfer_metadata = {    .priority       = CanardPriorityNominal,
                                                        .transfer_kind  = CanardTransferKindResponse,
                                                        .port_id        = transfer->metadata.port_id,
                                                        .remote_node_id = transfer->metadata.remote_node_id,
                                                        .transfer_id    = transfer->metadata.transfer_id };
  
  int32_t debug =  canardTxPush(        &queue,               	// Call this once per redundant CAN interface (queue)
                                        &canard,
                                        0,     					// Zero if transmission deadline is not limited.
                                        &transfer_metadata,
                                        c_serialized_size,		// Size of the message payload (see Nunavut transpiler)
                                        c_serialized);  
}

void LED1_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value )
{
  if( req_value->_tag_ != 0 ) // is it write access ?
  {
    if( req_value->integer8.value.elements[0] == 1 )
    {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
    }
    else if( req_value->integer8.value.elements[0] == 0 )
    {
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    }
  }

  // return register value
  res_value->_tag_= 7;
  res_value->integer8.value.count = 1;
  res_value->integer8.value.elements[0] = HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin);
}

void LED2_access_handler(uavcan_register_Value_1_0 * req_value, uavcan_register_Value_1_0 * res_value )
{
  if( req_value->_tag_ != 0 ) // is it write access ?
  {
    if( req_value->integer8.value.elements[0] == 1 )
    {
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
    else if( req_value->integer8.value.elements[0] == 0 )
    {
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
  }

  // return register value
  res_value->_tag_= 7;
  res_value->integer8.value.count = 1;
  res_value->integer8.value.elements[0] = HAL_GPIO_ReadPin(LED2_GPIO_Port, LED2_Pin);
}

// allocate dynamic memory of desired size in bytes
static void *memAllocate(CanardInstance *const canard, const size_t amount)
{
  (void)canard;
  return malloc(amount);
}

// free allocated memory
static void memFree(CanardInstance *const canard, void *const pointer)
{
  (void)canard;
  free(pointer);
}

/*
  * @brief Decodes FDCAN_data_length_code into the decimal length of FDCAN message
  * @param[in]          length           FDCAN_data_length_code
  * @retval             uint8_t         Decimal message length (bytes)
*/
uint8_t LengthDecoder( uint32_t length )
{
  switch( length )
  {
    case FDCAN_DLC_BYTES_0:     return 0;
    case FDCAN_DLC_BYTES_1:     return 1;
    case FDCAN_DLC_BYTES_2:     return 2;
    case FDCAN_DLC_BYTES_3:     return 3;
    case FDCAN_DLC_BYTES_4:     return 4;
    case FDCAN_DLC_BYTES_5:     return 5;
    case FDCAN_DLC_BYTES_6:     return 6;
    case FDCAN_DLC_BYTES_7:     return 7;
    case FDCAN_DLC_BYTES_8:     return 8;
    case FDCAN_DLC_BYTES_12:    return 12;
    case FDCAN_DLC_BYTES_16:    return 16;
    case FDCAN_DLC_BYTES_20:    return 20;
    case FDCAN_DLC_BYTES_24:    return 24;
    case FDCAN_DLC_BYTES_32:    return 32;
    case FDCAN_DLC_BYTES_48:    return 48; 
    case FDCAN_DLC_BYTES_64:    return 64;
      
    default:
      while(1); //error
  }
}

/*
  * @brief Codes decimal length of FDCAN message into the FDCAN_data_length_code
  * @param[in]          length              Decimal message length (bytes)
  * @retval             FDCAN_data_length_code        Code of required message length
*/
uint32_t LengthCoder( uint8_t length )
{
  switch( length )
  {
    case 0:     return FDCAN_DLC_BYTES_0;
    case 1:     return FDCAN_DLC_BYTES_1;
    case 2:     return FDCAN_DLC_BYTES_2;
    case 3:     return FDCAN_DLC_BYTES_3;
    case 4:     return FDCAN_DLC_BYTES_4;
    case 5:     return FDCAN_DLC_BYTES_5;
    case 6:     return FDCAN_DLC_BYTES_6;
    case 7:     return FDCAN_DLC_BYTES_7;
    case 8:     return FDCAN_DLC_BYTES_8;
    case 12:    return FDCAN_DLC_BYTES_12;
    case 16:    return FDCAN_DLC_BYTES_16;
    case 20:    return FDCAN_DLC_BYTES_20;
    case 24:    return FDCAN_DLC_BYTES_24;
    case 32:    return FDCAN_DLC_BYTES_32;
    case 48:    return FDCAN_DLC_BYTES_48;
    case 64:    return FDCAN_DLC_BYTES_64;
      
    default:
      while(1); //error
  }
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
