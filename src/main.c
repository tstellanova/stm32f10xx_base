

/* Includes ------------------------------------------------------------------*/
#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_conf.h>
#include <stm32f1xx_hal_tim.h>
#include <stm32f1xx_hal_tim_ex.h>
#include <stm32f1xx_hal_can.h>
#include <string.h>
#include "main.h"


/* Private typedef -----------------------------------------------------------*/

typedef volatile struct node_state_t {
    volatile uint8_t health;
    volatile uint8_t mode;
    volatile uint8_t sub_mode;
    volatile uint32_t uptime;
} node_state_t;


/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

UART_HandleTypeDef _DBGUART_Handle;
CAN_HandleTypeDef  _CAN1_Handle;
TIM_HandleTypeDef  _Heartbeat_Handle = {0};

node_state_t _node_state = {0};


/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void SystemClock_Config(void);
static void Error_Handler(HAL_StatusTypeDef status);

/* Private functions ---------------------------------------------------------*/


static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();



//  HAL_RCC_ClockConfig()

//  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC, ENABLE);
/**
 *  LED1 -> PA6 , LED2 -> PA7
 */
  GPIO_InitStructure.Pin  = GPIO_PIN_6 | GPIO_PIN_7 ;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);

}


static void CAN_Config(void)
{
//  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage; //TODO check use of static here
  static CanRxMsgTypeDef        RxMessage;

  /*##-1- Configure the CAN peripheral #######################################*/
  _CAN1_Handle.Instance = CAN1;
  _CAN1_Handle.pTxMsg = &TxMessage;
  _CAN1_Handle.pRxMsg = &RxMessage;

  _CAN1_Handle.Init.TTCM = DISABLE;
  _CAN1_Handle.Init.ABOM = DISABLE;
  _CAN1_Handle.Init.AWUM = DISABLE;
  _CAN1_Handle.Init.NART = DISABLE;
  _CAN1_Handle.Init.RFLM = DISABLE;
  _CAN1_Handle.Init.TXFP = DISABLE;
  _CAN1_Handle.Init.Mode = CAN_MODE_NORMAL;
  _CAN1_Handle.Init.SJW = CAN_SJW_1TQ;
  _CAN1_Handle.Init.BS1 = CAN_BS1_6TQ;
  _CAN1_Handle.Init.BS2 = CAN_BS2_8TQ;
  _CAN1_Handle.Init.Prescaler = 2;
  if (HAL_OK != HAL_CAN_Init(&_CAN1_Handle)) {
    Error_Handler(HAL_ERROR);
  }

//  // Configure CAN acceptance filter
//  sFilterConfig.FilterNumber = 0;
//  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
//  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
//  sFilterConfig.FilterIdHigh = 0x0000;
//  sFilterConfig.FilterIdLow = 0x0000;
//  sFilterConfig.FilterMaskIdHigh = 0x0000;
//  sFilterConfig.FilterMaskIdLow = 0x0000;
//  sFilterConfig.FilterFIFOAssignment = 0;
//  sFilterConfig.FilterActivation = ENABLE;
//  sFilterConfig.BankNumber = 14;
//  if (HAL_OK != HAL_CAN_ConfigFilter(&_CAN1_Handle, &sFilterConfig)) {
//    Error_Handler(HAL_ERROR);
//  }

  // Configure CAN transmission process
  _CAN1_Handle.pTxMsg->StdId = 0x321;
  _CAN1_Handle.pTxMsg->ExtId = 0x01;
  _CAN1_Handle.pTxMsg->RTR = CAN_RTR_DATA;
  _CAN1_Handle.pTxMsg->IDE = CAN_ID_EXT;
  _CAN1_Handle.pTxMsg->DLC = 2;
}


void UART_Config(void)
{
  /*##-1- Configure the UART peripheral ######################################*/
  /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
  /* UART configured as follows:
      - Word Length = 8 Bits (7 data bit + 1 parity bit) : BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
      - Stop Bit    = One Stop bit
      - Parity      = ODD parity
      - BaudRate    = 9600 baud
      - Hardware flow control disabled (RTS and CTS signals) */
  _DBGUART_Handle.Instance        = USARTx;

  _DBGUART_Handle.Init.BaudRate   = 9600;
  _DBGUART_Handle.Init.WordLength = UART_WORDLENGTH_8B;
  _DBGUART_Handle.Init.StopBits   = UART_STOPBITS_1;
  _DBGUART_Handle.Init.Parity     = UART_PARITY_ODD;
  _DBGUART_Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
  _DBGUART_Handle.Init.Mode       = UART_MODE_TX_RX;

  if (HAL_UART_Init(&_DBGUART_Handle) != HAL_OK) {
    Error_Handler(HAL_ERROR);
  }
}




static void heartbeat_config(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  __HAL_RCC_TIM3_CLK_ENABLE();

  _Heartbeat_Handle.Instance = TIM3;
  _Heartbeat_Handle.Init.Prescaler = 12500000;
  _Heartbeat_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  _Heartbeat_Handle.Init.Period = 1000;
  _Heartbeat_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  _Heartbeat_Handle.Init.RepetitionCounter = 0;
  //calls into HAL_TIM_Base_MspInit:
  if (HAL_OK != HAL_TIM_Base_Init(&_Heartbeat_Handle)) {
    Error_Handler(HAL_ERROR);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&_Heartbeat_Handle, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&_Heartbeat_Handle, &sMasterConfig);

  if (HAL_OK != HAL_TIM_Base_Start_IT(&_Heartbeat_Handle)) {
    Error_Handler(HAL_ERROR);
  }
}



static void sendNodeStatus()
{
//TODO implement

}
/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F107xC HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 72 MHz */
  SystemClock_Config();

  GPIO_Configuration();

  heartbeat_config();

//  UART_Config();

  CAN_Config();

  BSP_LED_Off(LED1);
  BSP_LED_Off(LED2);
  BSP_LED_On(LED1);

  printf("\n\r startup \r\n");
//  if (HAL_OK != HAL_CAN_Receive_IT(&_CAN1_Handle, CAN_FIFO0))  {
//    Error_Handler(HAL_ERROR);
//  }

  //send our NodeInfo over and over again
  while (1) {
    sendNodeStatus();
    HAL_Delay(10);
  }
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART2 and Loop until the end of transmission */
  HAL_UART_Transmit(&_DBGUART_Handle, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 72000000
  *            HCLK(Hz)                       = 72000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            HSE Frequency(Hz)              = 25000000
  *            HSE PREDIV1                    = 5
  *            HSE PREDIV2                    = 5
  *            PLL2MUL                        = 8
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLLs ------------------------------------------------------*/
  /* PLL2 configuration: PLL2CLK = (HSE / HSEPrediv2Value) * PLL2MUL = (25 / 5) * 8 = 40 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLL2CLK / HSEPredivValue = 40 / 5 = 8 MHz */
  /* PLL configuration: PLLCLK = PREDIV1CLK * PLLMUL = 8 * 9 = 72 MHz */ 

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  oscinitstruct.OscillatorType        = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState              = RCC_HSE_ON;
  oscinitstruct.HSEPredivValue        = RCC_HSE_PREDIV_DIV5;
  oscinitstruct.Prediv1Source         = RCC_PREDIV1_SOURCE_PLL2;
  oscinitstruct.PLL.PLLState          = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource         = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL            = RCC_PLL_MUL9;
  oscinitstruct.PLL2.PLL2State        = RCC_PLL2_ON;
  oscinitstruct.PLL2.PLL2MUL          = RCC_PLL2_MUL8;
  oscinitstruct.PLL2.HSEPrediv2Value  = RCC_HSE_PREDIV2_DIV5;
  if (HAL_OK !=  HAL_RCC_OscConfig(&oscinitstruct)) {
    Error_Handler(HAL_ERROR);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_OK != HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)) {
    Error_Handler(HAL_ERROR);
  }
}


// Called by HAL_TIM_IRQHandler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == &_Heartbeat_Handle) {
    BSP_LED_Toggle(LED2);
    _node_state.uptime++;
  }
}

void TIM3_IRQHandler(void)
{
  //foward to HAL_TIM_PeriodElapsedCallback
  HAL_TIM_IRQHandler(&_Heartbeat_Handle);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  //TODO handle received data
  if (CAN1 == hcan->Instance) {
    //rearm receive

    if (HAL_OK != HAL_CAN_Receive_IT(hcan, CAN_FIFO0) ) {
      /* Reception Error */
      Error_Handler(HAL_ERROR);
    }
  }
  //TODO handle CAN2
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(HAL_StatusTypeDef status)
{
  UNUSED(status);

  BSP_LED_Off(LED1);
  BSP_LED_On(LED2);
  while (1);

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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

