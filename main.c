/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_host.h"
#include <stdio.h>
#include "i2c_lcd.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#pragma pack(push, 1)
typedef struct {
    uint32_t package_no;

    uint8_t hour; // rtc
    uint8_t minute; //rtc
    uint8_t second; //rtc

    uint8_t angle; //us
    uint16_t distance; //us

    uint16_t water_level; //water
    uint16_t soil_humidity; //soil
    uint16_t light_intensity; //ldr

    uint8_t temperature; //dht11
    uint8_t air_humidity; //dht11

    uint8_t lock_flag; // if 1, screen lock
    uint8_t checksum;
} TelemetryPackage;
#pragma pack(pop)







/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */



#define RTC_I2C_ADDR (0x68 << 1) // rtc address ı2c standart



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */



TelemetryPackage mainPackage = {0};

uint16_t adc_buffer[3]; // ram array that dma writes

uint8_t cobs_buffer[sizeof(TelemetryPackage) + 2]; // for cobs encoded data
// and + 2 byte for cobs overhead load and for 0x00 at the end

uint8_t empty_counter = 0; // if someone is in front of the screen or not
uint8_t servo_angle = 80;
int8_t servo_direction = 1; // 1 for forward, -1 for backward

uint32_t last_dht_read = 0; // last read time when


typedef enum {
    STATE_LOCKED, //state 0
	STATE_WELCOME, // state 1
    STATE_READING, //state 2
    STATE_COUNTDOWN, //state 3
    STATE_ERROR //state 4
} SystemState;

SystemState currentState = STATE_LOCKED; 							//default safe start area // welcome is not first bc of security
SystemState lastState = STATE_ERROR; 								// just bc its different than the current state, dummy value

uint8_t error_code = 0;


uint32_t welcome_timer = 0;
uint32_t slide_timer = 0;
uint32_t countdown_timer = 0;

uint8_t seconds_left = 3;
uint8_t current_page = 0;
uint8_t last_page = 255;







/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


	uint8_t calculate_checksum(uint8_t data[], uint16_t length) {

		uint8_t checksum = 0;
		for(uint16_t i = 0; i < length; i++) {
			//checksum = (checksum + data[i]) % 256;
			checksum^= data[i]; // ^= == XOR  = checksum = checksum ^ data[i]
			//xor doesnt have overflow
		}
		return checksum;
}



	void delay_us(uint16_t us) { // hal delay freezes the system and hal(1) is too long

		__HAL_TIM_SET_COUNTER(&htim2, 0); // reset timer2
		while (__HAL_TIM_GET_COUNTER(&htim2) < us); // wait until wanted us microsn
	}




	size_t cobs_encode(const uint8_t original[], size_t length, uint8_t output[]) {

		size_t read_index = 0; // read index in original array
	    size_t write_index = 1; // bc 0. index is guide byte and shows where the next 0 is
	    size_t code_index = 0; // guide's new index num in output array, used for 0s
	    uint8_t code_step = 1; // step counter to 0x00, starts from 1 bc cant be 0

	    while (read_index < length) {

	    	if (original[read_index] != 0) {

	    		output[write_index] = original[read_index];
	    		read_index++;
	    		write_index++;
	    		code_step++;
	    	}
	    	else { // original[read_index] == 0
	    		output[code_index] = code_step;
	    		code_index = write_index; // went to next index
	    		write_index++;
	    		read_index++;
	    		code_step = 1; // reset steps to 1
	    	}

	    }

    	output[code_index] = code_step; // when the end is reached and while loop ended, for the last 0, write by hand //code index is given to last 0 before while loop ends
    	return write_index; // = length of output array

	}



	void update_servo(void) {

		servo_angle += (10 * servo_direction); // add 10 or subrtact 10 depending on directon

			  if (servo_angle >= 160) { //////////////180di
			            servo_direction = -1; // if reached 180, change direction
			            servo_angle = 160;/////////180
			        } else if (servo_angle <= 20) {//////////0
			            servo_direction = 1;  // if reached 0, change direction
			            servo_angle = 20;//////////0
			        }

			  //uint32_t pwm_value = 500 + (servo_angle * 2000) / 180; // servo with the new angle
			  uint32_t pwm_value = 700 + (servo_angle * 1600) / 180;
			  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pwm_value);

			  mainPackage.angle = servo_angle;//////////

	}


	void read_ultrasonic_sensor(void) {


		uint32_t timeout = 0;
		uint32_t duration = 0;
		uint16_t measured_distance = 999;

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); // open trig pin (pa1)
		delay_us(10); // small delay (not hal delay)
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // close trig




		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) { // echo high (pa15)

			timeout++;
		    if (timeout > 500000) {currentState = STATE_ERROR; error_code = 2; return;}
		}




		//if (timeout <= 500000) {
			__HAL_TIM_SET_COUNTER(&htim2, 0); // reset timer2
			timeout = 0;


			// echo low(pa15)
			while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET) {
				timeout++;
				if(timeout > 500000) {currentState = STATE_ERROR; error_code = 2; return;}
			}

		duration = __HAL_TIM_GET_COUNTER(&htim2);
		measured_distance = duration * 0.0343 / 2; // cm
		//}

		mainPackage.distance = measured_distance; // if no timeout //////////////

	}

	void check_security(void) {

		if (servo_angle >= 70 && servo_angle <= 110) { // 70- 110 angles reserved for me

		            if (mainPackage.distance > 50 || mainPackage.distance == 999) { // if signal is from 100+ or null = 999 (there isn't someone sitting)
		                empty_counter++;
		            } else {
		                empty_counter = 0; // if signal is normal (there is someone sitting)
		            }
		        }


		        if (empty_counter >= 3) { // if read empty 3+ than lock screen
		            mainPackage.lock_flag = 1; ////////////
		        } else {
		        	mainPackage.lock_flag = 0; ////////////
		        }
	}


	uint8_t bcd_to_decimal(uint8_t bcd) {

		return ((bcd >> 4) * 10) + (bcd & 0x0F); //first 4 bits tens digit, last 4 ones digit
		// slide 4 digits to right = tens digit number (*10)
		// bitwise and with 0x0f = 0000 1111 so tens digit will be deleted (bc and with 0 is 0) so the remaining is ones digit
	}


	void read_rtc(void) {
		uint8_t rtc_data[3];

		if (HAL_I2C_Mem_Read(&hi2c1, RTC_I2C_ADDR, 0x00, I2C_MEMADD_SIZE_8BIT, rtc_data, 3, 100) != HAL_OK) // ready func // 100 at the and = timeout
		{// 0x00 and 3 in parameters means = start from address 0x00 go 3 addresses: 0x00 = sec 0x01 = min 0x02 = hour

			currentState = STATE_ERROR;
			error_code = 3;
			return;

		}
		mainPackage.second = bcd_to_decimal(rtc_data[0]);
		mainPackage.minute = bcd_to_decimal(rtc_data[1]);
		mainPackage.hour   = bcd_to_decimal(rtc_data[2]);
}



	void dht_set_output(void) { // dht uses 1 data pin so pc5 will be both input and output, here output

	    GPIO_InitTypeDef GPIO_InitStruct = {0};
	    GPIO_InitStruct.Pin = GPIO_PIN_5;
	    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}

	void dht_set_input(void) {  // dht uses 1 data pin so pc5 will be both input and output, here input
	    GPIO_InitTypeDef GPIO_InitStruct = {0};
	    GPIO_InitStruct.Pin = GPIO_PIN_5;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_PULLUP;
	    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	}


	void read_dht11(void) {


		if (HAL_GetTick() - last_dht_read >= 2000) { // if 2 sec passed

			uint8_t data[5] = {0, 0, 0, 0, 0}; // 5 bytes of data = integral and decimals of temp and hum and a checksum
		    uint16_t timeout = 0;

		    // wakeup call signal from stm32
		    dht_set_output();

		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		    HAL_Delay(18); // 18ms
		    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
		    delay_us(20);  // 20 microsec

		    // stm32 listening
		    dht_set_input();

		    // wait for sensor to wakeup and answer and there is a timeout safety
		    // if timeout > 1000 then it means sensor is failed so dont wait here, pass
		    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
		    	timeout++;
		    	if(timeout>20000) {currentState = STATE_ERROR; error_code = 1; return;}
		    }

		    timeout = 0;

		    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
		    	timeout++;
		    	if(timeout>20000) {currentState = STATE_ERROR; error_code = 1; return;}
		    }

		    timeout = 0;

		    while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
		    	timeout++;
		    	if(timeout>20000) {currentState = STATE_ERROR; error_code = 1; return;}
		    }// set-reset-set = handshake = wakingup


		    // if 28 microsec high = 0, if 70 = 1 and 40 microsec safety

		    // 40 bits = 5 bytes of input comes (2 temp- 2 hum- 1 checksum)

		    for (int input_index = 0; input_index < 5; input_index++) {

		    	for( int bit = 7; bit >= 0; bit--) { // for each bit of the each byte

		    		timeout = 0;

		    		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_RESET) {
		    			timeout++;
		    			if(timeout>20000) {currentState = STATE_ERROR; error_code = 1; return;}
		    		}

		    		//delay_us(40); // 40 microsec safety limit (lower than 40 = 0, higher than 40 = 1)

		    		//uint8_t input_bit = 0; // default 0

		    		if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) {
		    		input_bit = 1; // if pin is still high then = 1


		    		timeout = 0;
		    		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) { // wait until pin is low
		    			timeout++;
		    			if(timeout>1000) {return;}
		    		}
				//////////////////


		    		uint8_t elapsed_time = 0;

		    		while(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_5) == GPIO_PIN_SET) { // wait until pin is low
		    			delay_us(1);
		    			elapsed_time++;

		    			if(elapsed_time > 100) {currentState = STATE_ERROR; error_code = 1; return;}
		    		}

		    		uint8_t input_bit = 0; // default 0

		    		if(elapsed_time > 40) {
		    			input_bit = 1;
		    		} else {
		    			input_bit = 0;
		    		}



		    		data[input_index] = data[input_index] << 1; // slide left (= *2) xxxxxxx0
		    		data[input_index] = data[input_index] + input_bit; // add input bit // 1 10 101 1011... msb first
		    	}
		    }




		    mainPackage.air_humidity = data[0];  // first byte is hum, second byte is the decimal of it so ignored
		    mainPackage.temperature = data[2];   // third  byte is temp, second byte is the decimal of it so ignored
		    // fifth byte is checksum, not written to the mainPackage


		    last_dht_read = HAL_GetTick(); // at the end reset the timer
		}


  }
	}


	void update_lcd_state_machine(void) {


		if(currentState == STATE_LOCKED){

			if (currentState != lastState){
				lcd_clear();
				lcd_put_cur(0,1);
				lcd_send_string("System Locked");
				lastState = currentState;
			}

			if(mainPackage.distance < 50) {
				currentState = STATE_WELCOME;
				welcome_timer = HAL_GetTick();

			}
		}



		else if(currentState == STATE_WELCOME){

			if (currentState != lastState){

				lcd_clear();
				lcd_put_cur(0,0);
				lcd_send_string("Welcome");
				lcd_put_cur(1,0);
				lcd_send_string("Receiving Data");
				lastState = currentState;
			}

			if (HAL_GetTick() - welcome_timer >= 2000 ) {
				currentState = STATE_READING;

			}

			if (mainPackage.distance >= 50) {
				currentState = STATE_COUNTDOWN;
				countdown_timer = HAL_GetTick();
				seconds_left = 3;
			}






		}else if(currentState == STATE_READING){


			if (currentState != lastState){
				lcd_clear();

				lcd_put_cur(0,0);
				lcd_send_string("t:    h:    ");
				lcd_put_cur(1,0);
				lcd_send_string("w:    l:    ");

				lastState= currentState;
			}

			// formatting
			char temp_str[6];
			char hum_str[6];
			char water_str[6];
			char light_str[6];

			sprintf(temp_str, "%dC  ", mainPackage.temperature);
			lcd_put_cur(0, 2); lcd_send_string(temp_str);

			sprintf(hum_str, "%%%d  ", mainPackage.air_humidity);
			lcd_put_cur(0, 8); lcd_send_string(hum_str);

			sprintf(water_str, "%d  ", mainPackage.water_level);
			lcd_put_cur(1, 2); lcd_send_string(water_str);

			sprintf(light_str, "%d  ", mainPackage.light_intensity);
			lcd_put_cur(1, 8); lcd_send_string(light_str);

			if (mainPackage.distance >= 50) {
				currentState = STATE_COUNTDOWN;
			    countdown_timer = HAL_GetTick();
			    seconds_left = 3;
			}



		}

		else if(currentState == STATE_COUNTDOWN){

		if (currentState != lastState){

			lcd_clear();
			lcd_put_cur(0,0);
			lcd_send_string("No User Seen");
			lastState = currentState;

		}


		char time_text[16];

		sprintf(time_text, "Locking: %d", seconds_left);
		lcd_put_cur(1, 0);
		lcd_send_string(time_text);

		if (HAL_GetTick() - countdown_timer >= 1000) {
			seconds_left--;
			countdown_timer = HAL_GetTick();
		}

		if (mainPackage.distance < 50) {
			currentState = STATE_READING;
		}
		else if (seconds_left == 0) {
			currentState = STATE_LOCKED;
		}







		}else if(currentState == STATE_ERROR){

			if (currentState != lastState){

				lcd_clear();
				lcd_put_cur(0, 0);
				lcd_send_string("! SYSTEM ERROR !");
				lcd_put_cur(1, 0);

				if (error_code == 1)      {lcd_send_string("DHT11 ERROR");}
				else if (error_code == 2) {lcd_send_string("ULTRASONIC ERROR");}
				else if (error_code == 3) {lcd_send_string("CLOCK CNNCTN");}
				else                      {lcd_send_string("UNKNOWN ERROR");}

				lastState = currentState;
			}


		}


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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USB_HOST_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 3); // start dma and write in 3 inedxed  adc_buffer array

  	  HAL_TIM_Base_Start(&htim2); // start timer 2


  	  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // start servo pwm signal

  	  lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  mainPackage.water_level = adc_buffer[0];
	  mainPackage.soil_humidity = adc_buffer[1];
	  mainPackage.light_intensity = adc_buffer[2];
	  mainPackage.package_no++;

	  update_servo(); // = angle
	  HAL_Delay(50);

	  read_ultrasonic_sensor(); // = distance

	  check_security(); // = lock_flag

	  read_rtc(); // = second + minute + hour

	  read_dht11();

	  update_lcd_state_machine();


	  mainPackage.checksum = 0;
	  mainPackage.checksum = calculate_checksum((uint8_t*)&mainPackage, sizeof(TelemetryPackage) - 1);
	  // convert struct to byte array, xor all except checksum

	  size_t encoded_length = cobs_encode((uint8_t*)&mainPackage, sizeof(TelemetryPackage), cobs_buffer);


	  cobs_buffer[encoded_length] = 0x00;
	  encoded_length++; // added 0x00 so increase length

	  HAL_UART_Transmit(&huart2, cobs_buffer, encoded_length, HAL_MAX_DELAY);

	  HAL_Delay(100);

    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

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
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim1.Init.Prescaler = 167;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 83;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD3_Pin|LD5_Pin|LD6_Pin|GPIO_PIN_0
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD5_Pin LD6_Pin PD0
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD5_Pin|LD6_Pin|GPIO_PIN_0
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
#ifdef USE_FULL_ASSERT
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
