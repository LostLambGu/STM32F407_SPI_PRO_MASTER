
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

#define MAX_PRINTF_STR_SIZE (254)
int32_t SerialDbgPrintf(uint8_t type, char *fmt, ...)
{
	if (type == 1)
	{
		int32_t cnt;
		char string[MAX_PRINTF_STR_SIZE + 2] = {'\0'};
		va_list ap;
		va_start(ap, fmt);

		cnt = vsnprintf(string, MAX_PRINTF_STR_SIZE, fmt, ap);
		if (cnt > 0)
		{
			if (cnt < MAX_PRINTF_STR_SIZE)
			{
				HAL_UART_Transmit(&huart1, (uint8_t *)string, cnt, 50);
			}
			else
			{
        HAL_UART_Transmit(&huart1, (uint8_t *)string, MAX_PRINTF_STR_SIZE, 50);
			}
		}
		va_end(ap);
		return (cnt);
	}
	return -1;
}

#define DebugLog(format, ...) SerialDbgPrintf(1, "\r\n" format, ##__VA_ARGS__)
#define DebugPrint(format, ...) SerialDbgPrintf(1, format, ##__VA_ARGS__)

void BLESpiTick(void);
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  DebugLog("_Error_Handler: (%s) (%d)", file, line);
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  DebugLog("assert_failed: (%s) (%d)", file, line);
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

#define SPI_MAX_ITEM_SIZE (128 *3)
#define SPI_BUFFER_SIZE (SPI_MAX_ITEM_SIZE + 4)

#define WIFI_SPI_HEADER_LEN  (uint8_t)(6)
#define WSHC_LOW_B (4) // WIFI SPI HEADER CRC16 LOW BYTE
#define WSHC_HIGH_B (5) // WIFI SPI HEADER CRC16 HIGH BYTE
#define WIFI_SPI_CTRL_WRITE  (uint8_t)(0x02)
#define WIFI_SPI_CTRL_READ   (uint8_t)(0x03)

#define WIFI_SPI_IRQ_PIN_READ() HAL_GPIO_ReadPin(PF4_SPI_IRQ_GPIO_Port, PF4_SPI_IRQ_Pin)
#define WIFI_SPI_CS_SET_LOW() HAL_GPIO_WritePin(PF3_SPI_CS_GPIO_Port, PF3_SPI_CS_Pin, GPIO_PIN_RESET)
#define WIFI_SPI_CS_SET_HIGH() HAL_GPIO_WritePin(PF3_SPI_CS_GPIO_Port, PF3_SPI_CS_Pin, GPIO_PIN_SET)

#define WIFI_SPI_HANDLE hspi1

volatile uint8_t spi_irq_flag = 0;
volatile uint8_t spi_bus_halt = 0;
static uint8_t SpiRxBuffer[SPI_BUFFER_SIZE];            /**< RX buffer. */
static uint8_t SpiTxBuffer[SPI_BUFFER_SIZE];            /**< TX buffer. */
static uint16_t SpiTxSize = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_4)
  {
    spi_irq_flag = 1;
    spi_bus_halt = 0;
  }
}

int16_t SPI_Read_Bridge(void);
int16_t SPI_Write_Bridge(uint8_t* data, uint16_t Nb_bytes);
uint16_t Cal_CRC16(const uint8_t *data, uint32_t size);

void BLESpiTick(void)
{
  if (spi_bus_halt == 0)
  {
    if (spi_irq_flag == 1)
    {
      spi_irq_flag = 0;
      SPI_Read_Bridge();
    }

    if (SpiTxSize)
    {
      SPI_Write_Bridge((uint8_t *)SpiTxBuffer, SpiTxSize);
    }
  }
}

int16_t SPI_Read_Bridge(void)
{  
  int16_t result = -1;
  uint16_t byte_count = 0;
  uint16_t wait_count = 0;
  uint16_t crc16 = 0;
  
  uint8_t header_master[WIFI_SPI_HEADER_LEN] = {WIFI_SPI_CTRL_READ, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[WIFI_SPI_HEADER_LEN]  = {0x00};
  
  uint16_t i = 0;
  
  if(WIFI_SPI_IRQ_PIN_READ() == GPIO_PIN_RESET) {
    DebugLog("SPI_Read_Bridge IRQ pin state err");
    result = -2;
    goto failed;
  }

  DebugLog("SPI_Read_Bridge IRQPin(%d)", WIFI_SPI_IRQ_PIN_READ());

  memset(SpiRxBuffer, 0, sizeof(SpiRxBuffer));
  
  WIFI_SPI_CS_SET_LOW();
  if (HAL_SPI_TransmitReceive(&WIFI_SPI_HANDLE, header_master, header_slave, WIFI_SPI_HEADER_LEN, 150))
  {
    DebugLog("SPI_Read_Bridge Header transfer err");
    result = -3;
    goto failed;
  }

  crc16 = Cal_CRC16(header_slave + 2, 2);
  if (crc16 != (((uint16_t)header_slave[5])<<8 | (uint16_t)header_slave[4]))
  {
    DebugLog("SPI_Read_Bridge header_slave crc err");
    result = -4;
    goto failed;
  }

  byte_count = ((uint16_t)header_slave[3])<<8 | (uint16_t)header_slave[2];
  if (byte_count == 0)
  {
    DebugLog("SPI_Read_Bridge receive byte_count 0");
    result = -5;
    goto failed;
  }
  else if (byte_count > SPI_MAX_ITEM_SIZE)
  {
    DebugLog("SPI_Read_Bridge receive byte_count %d > max(%d)", byte_count, SPI_MAX_ITEM_SIZE);
    result = -6;
    goto failed;
  }
  else
  {
    DebugLog("SPI_Read_Bridge byte_count: %d", byte_count);
  }

  DebugLog("SPI_Read_Bridge Wait IRQPin Low");
  while (WIFI_SPI_IRQ_PIN_READ() != GPIO_PIN_RESET)
  {
    HAL_Delay(1); // Delay 10 ms
    wait_count++;
    if (wait_count == 50)
    {
      DebugLog("SPI_Read_Bridge Wait IRQPin Low Timeout");
      result = -8;
      goto failed;
    }
  }

  if (byte_count > 0) 
  {
    DebugLog("SPI_Read_Bridge receive data size(%d)", byte_count);
    if (HAL_SPI_Receive(&WIFI_SPI_HANDLE, SpiRxBuffer, byte_count, 1000))
    {
      DebugLog("SPI_Read_Bridge receive data err");
      result = -9;
      goto failed;
    }

    crc16 = Cal_CRC16(SpiRxBuffer, byte_count - 2);
    if (crc16 != (((uint16_t)SpiRxBuffer[byte_count - 1])<<8 | (uint16_t)SpiRxBuffer[byte_count - 2]))
    {
      DebugLog("SPI_Read_Bridge read data crc err");
      result = -10;
      goto failed;
    }
  }
  result = byte_count;

  DebugLog("SPI_Read_Bridge received data\r\n");
  for (i = 0; i < byte_count; i++)
  {
    DebugPrint("0x%x ", SpiRxBuffer[i]);
  }
  DebugPrint("\r\n");
  
failed:
  if(spi_irq_flag == 1)
  {
    DebugLog("SPI_Read_Bridge spi_irq_flag pending");
  }

  if (result < 0)
  {
      DebugLog("====>>>> SPI_Read_Bridge fail before");
      spi_irq_flag = 0;
      spi_bus_halt = 1;
  }

  WIFI_SPI_CS_SET_HIGH();
  
  return result;
}

int16_t SPI_Write_Bridge(uint8_t* data, uint16_t Nb_bytes)
{  
  int16_t result = -1;
  uint32_t wait_count = 0;
  uint16_t crc16 = 0;
  
  uint8_t header_master[WIFI_SPI_HEADER_LEN] = {WIFI_SPI_CTRL_WRITE, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t header_slave[WIFI_SPI_HEADER_LEN]  = {0x00};

  header_master[2] = (uint8_t)Nb_bytes;
  header_master[3] = (uint8_t)(Nb_bytes>>8);

  crc16 = Cal_CRC16(header_master, 5);
  header_master[4] = (uint8_t)crc16;
  header_master[5] = (uint8_t)(crc16>>8);
  
  WIFI_SPI_CS_SET_LOW();
  
  while ((WIFI_SPI_IRQ_PIN_READ() == GPIO_PIN_RESET) || (spi_irq_flag == 0)) {
    HAL_Delay(1);
    wait_count++;
    if (wait_count == 1000)
    {
      DebugLog("SPI_Write_Bridge Wait IRQPin High Timeout");
      result = -2;
      goto failed;
    }
  }

  DebugLog("SPI_Write_Bridge IRQPin(%d)", WIFI_SPI_IRQ_PIN_READ());

  if (HAL_SPI_TransmitReceive(&WIFI_SPI_HANDLE, header_master, header_slave, WIFI_SPI_HEADER_LEN, 150))
  {
    DebugLog("SPI_Write_Bridge Header transfer err");
    result = -3;
    goto failed;
  }

  spi_irq_flag = 0;

  DebugLog("SPI_Write_Bridge Wait IRQPin Low");
  wait_count = 0;
  while (WIFI_SPI_IRQ_PIN_READ() != GPIO_PIN_RESET)
  {
    HAL_Delay(1);
    wait_count++;
    if (wait_count == 50)
    {
      DebugLog("SPI_Write_Bridge Wait IRQPin Low Timeout");
      result = -7;
      goto failed;
    }
  }
  
  if (HAL_SPI_Transmit(&WIFI_SPI_HANDLE, data, Nb_bytes, 1000))
  {
    DebugLog("SPI_Write_Bridge data transfer err");
    result = -8;
    goto failed;
  }
  
  result = Nb_bytes;

failed:

  if(spi_irq_flag == 1)
  {
    DebugLog("SPI_Write_Bridge spi_irq_flag pending");
  }

  if (result < 0)
  {
    spi_irq_flag = 0;
    spi_bus_halt = 1;
  }
  
  WIFI_SPI_CS_SET_HIGH();

  return result;
}

/**
  * @brief  Update CRC 16 for input byte
  * @param  CRC input value 
  * @param  input byte
  * @retval Updated CRC value
  */
static uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
{
  uint32_t crc = crcIn;
  uint32_t in = byte | 0x100;

  do
  {
    crc <<= 1;
    in <<= 1;

    if (in & 0x100)
    {
      ++crc;
    }

    if (crc & 0x10000)
    {
      crc ^= 0x1021;
    }
  } while (!(in & 0x10000));

  return (crc & 0xffffu);
}

/**
  * @brief  Cal CRC 16 for YModem Packet
  * @param  data
  * @param  length
  * @retval CRC value
  */
uint16_t Cal_CRC16(const uint8_t *data, uint32_t size)
{
  uint32_t crc = 0;
  const uint8_t *dataEnd = data + size;

  while (data < dataEnd)
  {
    crc = UpdateCRC16(crc, *data++);
  }
  crc = UpdateCRC16(crc, 0);
  crc = UpdateCRC16(crc, 0);

  return (crc & 0xffffu);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
