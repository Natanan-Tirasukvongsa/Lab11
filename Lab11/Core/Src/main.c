/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//address from datasheet
//control code = address code begin with 1010 A2 A1 A0
//A2 A1 A0 ต่อ GND 1010 + 0 0 0
//stm32 I2C 7 bits but require 8 bit with shift front bit
//ต้องครบ 8 bit โดยชิพไปข้างหน้า 1 เพื่อให้อยู่ด้านซ้านสุด 1010000 +0
#define EEPROM_ADDR 0b10100000
//I2C
//slave address 0100 A2 A1 A0
//A2 A1 A0 ต่อ GND 0100 + 0 0 0
#define IOEXPD_ADDR 0b01000000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//tricker
uint8_t eepromExampleWriteFlag = 0;
uint8_t eepromExampleReadFlag = 0;
uint8_t IOExpdrExampleWriteFlag = 0;
uint8_t IOExpdrExampleReadFlag = 0;

uint8_t eepromDataReadBack[4];
uint8_t IOExpdrDataReadBack;
uint8_t IOExpdrDataWrite = 0b1111; //ข้อมูลที่จะเขียนใน io expander ไฟจะติดสลับ

uint8_t led_read = 0b0;
uint8_t D_1 = 0;
uint8_t D_2 = 0;
uint8_t D_3 = 0;
uint8_t D_4 = 0;

GPIO_PinState button[2] = {0};
uint8_t press = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
//ไม่ได้เรีย�?ใช้บ่อย
//EEProm เป็นอุป�?รณ์เ�?็บข้อมูล เหมืนตารางอยู่ข้างใน
//เป็น�?ารเ�?็บข้อมูล�?บบไม่หายไป ถึง�?ม้ถอดปลั๊�? controller
//เ�?็บข้อมูลไม่ใช้ไฟได้ �?ต่มีจำ�?ัดจำนวนครั้งที่เขียนได้
//ถ้าเขียนเ�?ินจำนวนครั้งที่เขียนไว้ต่อ 1 cell , cell จะเสื่อม ทำให้ไม่เรีย�?ใช้บ่อย
//ใช้เขียนเฉพาะจำเป็น �?ต่อ่านได้เรื่อย ๆ
//มีประโยชน์ใน�?ารเ�?็บข้อมูล setting
void EEPROMWriteExample();
void EEPROMReadExample(uint8_t *Rdata, uint16_t len);
//เพิ่ม port gpio
//ทำงานคล้าย eeprom
void IOExpenderInit();
void IOExpenderReadPinA(uint8_t *Rdata);
void IOExpenderWritePinB(uint8_t Wdata);

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //delay ตอนเริ่มเนื่องจา�? ioexpender restart ไปพร้อมๆ�?ับเวลาที่�?ด reset
  //ใช้เวลาเล็�?น้อยใน�?าร ตั้งค่าตัดเอง
  HAL_Delay(100);
  //initial
  IOExpenderInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_I2C_Master_Transmit(&hi2c1, (0x23<<1),(uint8_t*)0x45 , 1, 200);

	  button[0]= HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);


	  if(button[1]==GPIO_PIN_SET
	  	  	&& button[0]==GPIO_PIN_RESET && press ==0) //if press button
	  {
		  IOExpdrExampleWriteFlag = 0;
		  IOExpdrExampleReadFlag = 1;
		  eepromExampleWriteFlag = 1;
		  eepromExampleReadFlag = 0;
		  press = 1;
	  }

	  if(button[1]==GPIO_PIN_RESET
	  	  	  	&& button[0]==GPIO_PIN_SET && press ==1) //if press button
	  	  {
	  		  IOExpdrExampleWriteFlag = 1;
	  		  IOExpdrExampleReadFlag = 0;
	  		  eepromExampleWriteFlag = 0;
	  		  eepromExampleReadFlag = 1;
	  		  press = 0;
	  	  }
	  IOExpenderReadPinA(&IOExpdrDataReadBack);
	 //	  led_read = IOExpdrDataReadBack<<4;
	 //	  IOExpdrDataWrite = IOExpdrDataReadBack&0b00001111;
	 	  D_4 = (IOExpdrDataReadBack&0b1000)>>3;
	 	  D_3 = (IOExpdrDataReadBack&0b0100)>>2;
	 	  D_2 = (IOExpdrDataReadBack&0b0010)>>1;
	 	  D_1 = (IOExpdrDataReadBack&0b0001);
	 	  EEPROMWriteExample();
	 	  IOExpenderWritePinB(IOExpdrDataReadBack&0b1111);
	 	  	  	   //เ�?็บค่าใน  eepromDataReadBack
	 	  EEPROMReadExample(eepromDataReadBack, 4);
	  button[1] = button[0];

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//�?ารเขียน eeprom
static uint8_t data[4];
void EEPROMWriteExample() {
	//flag = 1 && I2C ready
	if (eepromExampleWriteFlag && hi2c1.State == HAL_I2C_STATE_READY) {

		//ข้อมูลที่จเขียนใน eeprom
		//ข้อระวัง I2C ทำงาน �?บบ IT //ต้องมั่นใจว่า data ไม่เปลี่ยน�?ปลงไประหว่างที่เขียน
		//ใส่ static เผื่อเ�?็บค่า หลังจา�?จบฟัง�?์ชันยังคงรั�?ษา data

		data[0] = D_1;
		data[1] = D_2;
		data[2] = D_3;
		data[3] = D_4;
		//write memory
		//Memaddress = ตำ�?หน่งภายใน eeprom ที่ต้อง�?ารเขียน  0x20 //เลขที่ตัวเอง
		//high & low byte = 16bits ตำ�?หน่งใน eeprom
		HAL_I2C_Mem_Write_IT(&hi2c1, EEPROM_ADDR, 0x20, I2C_MEMADD_SIZE_16BIT,
				data, 4);

		HAL_Delay(10);
		//set flag = 0 //ทำงานครั้งเดียว
		eepromExampleWriteFlag = 0;

	}
}
void EEPROMReadExample(uint8_t *Rdata, uint16_t len) {
	if (eepromExampleReadFlag && hi2c1.State == HAL_I2C_STATE_READY) {

		//read
		HAL_I2C_Mem_Read_IT(&hi2c1, EEPROM_ADDR, 0x20, I2C_MEMADD_SIZE_16BIT,
				Rdata, len);
		HAL_Delay(10);
		eepromExampleReadFlag = 0;

	}
}
void IOExpenderInit() {
	//Init All
	//setting ตารางใน IOexpander เพื่อให้ใช้งานได้ sequencial write
	//ขนาด 16 bit
	//POR/RST = ค่า reset ตอนตัดไฟ ถ้าตัดไฟข้อมูลจะหายเลย
	//สามารถอ่านเขียนได้อย่างไม่จำ�?ัด
	//IODIRA = 1 input GPIOA 0xFF
	//IODIRB = 0 output GPIOB 0x00
	//ที่เหลือ default 0x00
	//GPPUA ช่อง 12 =1 pull up input �?�?้ปั�?หาปล่อยลยเป็น high 0xFF ค่าจะได้ไม่สุ่ม
	//olatb = 0x15 ตั้งไว้เป็น 0x00 ไฟจะติดทั้งหมด
	static uint8_t Setting[0x16] = { 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00 };
	//polling
	//อยู่ในรูป�?บบ initial write
	//ถู�?เรีย�?ครั้ง�?ร�?ตอนเปิด controller
	//เวลา�?ดปุ่ม reset = Nrst จะดึงขานี้ลง หลังจา�?นั้นจะ reset ค่าทั้งหมดในนี้
	//address 8 bit
	//ตำ�?หน่ง IODIRA = 0x00
	HAL_I2C_Mem_Write(&hi2c1, IOEXPD_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, Setting,
			0x16, 100);
}
//master recive & master transmit เป็น�?ารอ่านเขียนป�?ติ ใน project modlu3
//master seq recive, master seq rans,it เป็น�?ารตุมจังหวะรับส่ง

//ข้อมูลที่อ่านเขียนสามารถเ�?็บค่า �?ละ �?สดง�?ารทำงาน ควบคุม  Gpio �?ต่ละช่อง
//IOIDR �?ำหนด output �?ต่ละช่อง
//gpio = reflec logic level ของ�?ต่ลช่อง
//�?ารเขียนข้อมูลใน emory ของ IOexpender �?ต่ memory ไปควบคุม�?ารทำงานต่าง ๆ ใน IOExpender
//ทำงานเหมือน eeprom
void IOExpenderReadPinA(uint8_t *Rdata) {
	if (IOExpdrExampleReadFlag && hi2c1.State == HAL_I2C_STATE_READY) {
		//read gpio A at 0x12
		//เ�?็บใน Rdata
		//ตำ�?หน่ง GPIOA = 0x12
		HAL_I2C_Mem_Read_IT(&hi2c1, IOEXPD_ADDR, 0x12, I2C_MEMADD_SIZE_8BIT,
				Rdata, 1);
		//HAL_Delay(100);
		IOExpdrExampleReadFlag =0;
	}
}

//olat  = �?ำหนด output high low
//16 bits = Bank 0
//output high ไฟไม่ติด
void IOExpenderWritePinB(uint8_t Wdata) {
	if (IOExpdrExampleWriteFlag && hi2c1.State == HAL_I2C_STATE_READY) {
		//สร้าง data ตัวใหม่
		static uint8_t data;
		//ป้อง�?ัน�?าร�?�?้ไขข้อมูลใน wdata เมื่อ ข้อมูลใน i2c ยังส่งไม่เสร็จ ข้อมูลจะยังไม่ถู�?�?�?้ไข
		data = Wdata; //เปลี่ยน�?ปลงค่าของตัวนี้ได้เรื่อย ๆ
		//write memory
		//เขียนลงใน output GPIOB �?ต่ไม่ได้เขียนลง GPIOB โดยตรง โดยจะเขียนลงใน OLATB
		//olatb = 0x15 output gpiob
		HAL_I2C_Mem_Write_IT(&hi2c1, IOEXPD_ADDR, 0x15, I2C_MEMADD_SIZE_8BIT,
				&data, 1);
		//HAL_Delay(100);
		IOExpdrExampleWriteFlag=0;
	}
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) //Interrupt
//{
//	if (GPIO_Pin == GPIO_PIN_13 )
//	{
//		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET) //falling
//		{
//			State_button = 0;
//			IOExpdrExampleWriteFlag = 0;
//			IOExpdrExampleReadFlag = 1;
//			eepromExampleWriteFlag = 1;
//			eepromExampleReadFlag = 0;

//		}
//		else //rising
//		{
//			State_button = 1;
//			IOExpdrExampleReadFlag = 0;
//			IOExpdrExampleWriteFlag = 1;
//			eepromExampleReadFlag = 1;
//			eepromExampleWriteFlag = 0;
//
//		}
//	}
//}
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
