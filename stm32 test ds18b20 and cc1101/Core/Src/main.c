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
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "cc1101.h"
#include "ds18b20.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define state 	1
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t GDO0_FLAG;
DS18B20 temperatureSensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UserLEDHide()
{
	 HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);
}


void UserLEDShow()
{
	 HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_SET);
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  int k;

   DS18B20_Init(&temperatureSensor, &huart1);

     DS18B20_InitializationCommand(&temperatureSensor);
     DS18B20_ReadRom(&temperatureSensor);
     DS18B20_ReadScratchpad(&temperatureSensor);

     uint8_t settings[3];
     settings[0] = temperatureSensor.temperatureLimitHigh;
     settings[1] = temperatureSensor.temperatureLimitLow;
     settings[2] = DS18B20_12_BITS_CONFIG;

     DS18B20_InitializationCommand(&temperatureSensor);
     DS18B20_SkipRom(&temperatureSensor);
     DS18B20_WriteScratchpad(&temperatureSensor, settings);
     //
    BYTE status;
    Power_up_reset();
    TI_init(&hspi1, CS_GPIO_Port, CS_Pin);


  #if state
    //for transiver
    BYTE veri = 7;
    uint8_t code1[5]="yi6 ";
    uint8_t code2[5]="on7\n";
    uint8_t board[10] = "stm32f4xx";
    //uint8_t l1[]="yy";
    //int l=0;
    while (1)
    {
    	      //l=l+1;
    	      //sprintf((char *)l1,"%d", l);
    	      DS18B20_InitializationCommand(&temperatureSensor);
    	      DS18B20_SkipRom(&temperatureSensor);
    	      DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);
    	      DS18B20_InitializationCommand(&temperatureSensor);
    	      DS18B20_SkipRom(&temperatureSensor);
    	      DS18B20_ReadScratchpad(&temperatureSensor);
    	      uint8_t str1[30]="yy";
    	           k=*&temperatureSensor.temperature;//k - temp
    	           //gcvt(k, 6, str1);
    	           sprintf((char *)str1,"%d ", k);//k - temp
    	           //if (l==99)
    	        		   //l=0;
    	              //float x = 123.4567;
    	              //char buf[1];
    	              //gcvt(k, 6, buf);
    	           //strcat(board,code2);
    	           strcat(code1,board);
    	           strcat(str1,code1);
    	           //strcat(str1,board);
    	           //strcat(str1,l1);//счётчик отправленных сообщений
    	           strcat(str1,code2);
    	           HAL_UART_Transmit(&huart2,str1,sizeof(str1),40);
  	  status = TI_read_status(CCxxx0_VERSION); // it is for checking only //sadece kontrol için
  	  //it must be 0x14 //0x14 değeri vermelidir

  	  status = TI_read_status(CCxxx0_TXBYTES); // it is too // bu da kontrol için

  	  TI_strobe(CCxxx0_SFTX); // flush the buffer //bufferi temizler

  	  UserLEDShow(); // turn on the led before send the data // veri göndermeden önce ledi yakar
  	  TI_send_packet(str1, sizeof(str1)); //the function is sending the data

  	  while(HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin));
  	  while(!HAL_GPIO_ReadPin(GDO0_GPIO_Port, GDO0_Pin));
  	  //if the pass to this function, the data was sent. // veri gönderme işlemi tamamlanması için bekletir

  	  status = TI_read_status(CCxxx0_TXBYTES); // it is checking to send the data //veri gönderildiğini kontrol etmek için

  	  UserLEDHide(); // turn off the led // veri gönderildiğinde led söner
  	  HAL_Delay(100);
    }


  #else
    //for receiver/modem

    MX_USART1_UART_Init();
    init_serial(&huart1);

    char buffer[64];
    char mesaj[7];
    char kontrol[7] = {'R', 'F', ' ', 'T', 'e', 's', 't' };
    uint8_t value = 0;
    BYTE length;

    TI_write_reg(CCxxx0_IOCFG0, 0x06);//0x06
    //interrupt setting on GDO0 for data receiving with FIFO
    // GDO0 pininde FIFO ile kullanmak için interrupt ayarı

    /* EXTI interrupt init*/
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    while(1)
    {
  	  uint8_t str[]="yy";
  	  int n=13 ;
  	  sprintf((char *)str,"%d\n", n);
  	  HAL_UART_Transmit(&huart1,str,sizeof(str)+2,15);
  	  BYTE status;
  	  BYTE LQI;

  	  __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

  	  GDO0_FLAG = 0;

  	  TI_strobe(CCxxx0_SFRX);// Flush the buffer //Bufferi temizle
  	  TI_strobe(CCxxx0_SRX); // Set RX Mode //RX moda ayarlar

  	  while(GDO0_FLAG == 0);//(GDO0_FLAG == 0)
  	  //when the data came, it passes this fucntion
  	  //veri geldiğinde bu adımı geçer

  	  status = TI_read_status(CCxxx0_RXBYTES);
  	  //it is for checking
  	  // gelen datanın olup olmadığını denetler

  	  if (!(status & 0x7f)) continue;
  	  // if it get the data, the code can contuniue
  	  // data gelirse, kod devam eder

  	  LQI = TI_read_status(CCxxx0_LQI);
  	  // LQI is quaility of the receiving data
  	  // LQI gelen veri kalitesi

  	  if (LQI & 0x80 /*CRC_OK*/)
  	  {
  		  status = TI_receive_packet(buffer, &length);//read function //okuma fonksiyonu

  		  for(uint8_t i = 0; i <7; i++)//to check for receiving data // gelen datayı kontrol etmek için
  		  {
  			  mesaj[i] = buffer[i];
  			  if(strcmp(mesaj[i], kontrol[i])!=0)
  			  {
  				  value = 1;
  				  break;
  			  }
  		  }

  		  //to send over uart on serial port with 9600baudrate
  		  // Gelen datayı uart üzerinden 9600 baudrate ile seri porta gönderir
  		  HAL_UART_Transmit(&huart1, (uint8_t*)mesaj, sizeof(mesaj), HAL_MAX_DELAY);
  		  HAL_UART_Transmit(&huart1, "\n\r", 2, HAL_MAX_DELAY);

  		  if (value == 0)
  			  // if receiving data is equal to checking data, the led is blink
  			  // eğer gelen data kontrol datasına eşitse, led yanıp söner
  		  {
  			  UserLEDShow();

  			  HAL_Delay(50);

  			  UserLEDHide();
  		  }
  	  }
  	  else
  	  {
  		  status = TI_read_status(CCxxx0_PKTSTATUS); // if it isnt, check pktstatus // değilse, paket durumunu kontrol eder

  		  GDO0_FLAG = 0;
  	  }
    }
  #endif
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
#if state == 0
//for receiver/modem
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//if it get the receiving data, the interrupt works
	// eğer gelen data gelirse, kesme çalışır.
	if(GPIO_Pin == GDO0_Pin){
			GDO0_FLAG = 1;
		}
}
#endif
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
