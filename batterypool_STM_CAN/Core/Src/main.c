/* USER CODE BEGIN Header */
/**
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
#include "can.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void CAN_Configuration(void);
void GPIO_Configuration(void);
void delay_ms(uint32_t ms);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t temperature = 0;

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
	DHT22_Start();
	/* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
				temperature = DHT22_ReadTemperature();

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// Send temperature over CAN bus
        CAN1->sTxMailBox[0].TDLR = temperature; // Assuming only 1 byte for temperature
        CAN1->sTxMailBox[0].TDTR = 1; // Set data length to 1 byte
        CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; // Set transmit request bit

        // Wait for transmission completion
        while ((CAN1->sTxMailBox[0].TIR & CAN_TI0R_TXRQ) != 0);

        delay_ms(1000); // Delay for 1 second before reading again
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 80;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void GPIO_Configuration(void) {
    // Configure GPIO pins for CAN peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock

    // Configure GPIO pins for CAN1: PA11 (RX) and PA12 (TX)
    GPIOA->MODER |= GPIO_MODER_MODER11_1 | GPIO_MODER_MODER12_1; // Set pins to alternate function mode
    GPIOA->AFR[1] |= 0x440; // AF9 (CAN1) for pins 11 and 12
}

void CAN_Configuration(void) {
    // Enable CAN1 and GPIO clocks
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // Reset CAN1 peripheral
    CAN1->MCR |= CAN_MCR_RESET;
    while ((CAN1->MSR & CAN_MCR_RESET) != 0);

    // Exit sleep mode
    CAN1->MCR &= ~CAN_MCR_SLEEP;

    // Initialize CAN1
    CAN1->MCR |= CAN_MCR_INRQ; // Enter initialization mode
    while ((CAN1->MSR & CAN_MSR_INAK) != CAN_MSR_INAK);

    // Set CAN1 bit timings (assuming 125 kbps)
    CAN1->BTR = (5 << 20) | (4 << 16) | (12 << 0); // SJW=5, TS2=4, TS1=12, BRP=1 (125 kbps)

    // Leave initialization mode
    CAN1->MCR &= ~CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK);

    // Enable transmit mailbox 0 empty interrupt
    CAN1->IER |= CAN_IER_TMEIE;

    // Enable CAN1
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    CAN1->MCR |= CAN_MCR_INRQ;
}

void delay_ms(uint32_t ms) {
    // Approximate delay in milliseconds
    // Adjust the following value based on your clock frequency and optimization level
    volatile uint32_t i;
    for (i = 0; i < (ms * 1000); i++) {}
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
