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
#include "dma.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "cs43l22.h"
#include "math.h"

#include "stm32f4_discovery.h"
#include "stm32f4_discovery_accelerometer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define LINE_BUFFER_SIZE 256
// macros to define the sine signal
#define SAMPLING_RATE 48000
/*audio*/
#define AUDIO_BUF_SIZE 2048



/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
const uint8_t ISR_FLAG_RX    = 0x01;  // Received data
const uint8_t ISR_FLAG_TIM10 = 0x02;  // Timer 10 Period elapsed
const uint8_t ISR_FLAG_TIM11 = 0x04;  // Timer 10 Period elapsed
const uint8_t ISR_FLAG_BTN = 0x08;  // Button pressed
/*changereq*/
extern TIM_HandleTypeDef htim10;
extern volatile uint8_t isr_flags;
volatile uint32_t pwm_period = 5000; // 
volatile float duty_cycle = 0.5f;    // initial 50%
volatile uint8_t led_state = 0;      // 0:off, 1:on

/*PWM_manual_control*/
volatile uint8_t use_pwm_for_led = 1;
const uint8_t COMMAND_PWM_MAN[] = "pwmman"; // Enter manual mode command
volatile uint8_t pwm_mode_manual = 0;   // 0:auto set 3 levels , 1:press button to set period
volatile uint32_t btn_start_time = 0;  // Button press start time
/*acc*/
extern TIM_HandleTypeDef htim11;//acc
const uint8_t COMMAND_ACC_ON[] = "accon";
const uint8_t COMMAND_ACC_OFF[] = "accoff";
volatile uint8_t acc_enabled = 0;//acc enable flag

/*audio*/
const uint8_t COMMAND_MUTE[] = "mute";
const uint8_t COMMAND_UNMUTE[] = "unmute";

// This is the only variable that we will modify both in the ISR and in the main loop
// It has to be declared volatile to prevent the compiler from optimizing it out.
volatile uint8_t isr_flags = 0;
// Commands that we will receive from the PC

const uint8_t COMMAND_STOP[]        = "stop"; // Go to stop mode
const uint8_t COMMAND_STANDBY[]    = "standby"; // Go to standby mode
const uint8_t COMMAND_CHANGE_DUT[]  = "changedut";// change duty cycle
const uint8_t COMMAND_CHANGE_FREQ[] = "changefreq"; // Change the frequency


/*LED manual control*/
const uint8_t COMMAND_LED_PWM[] = "ledpwm";
const uint8_t COMMAND_LED_MAN[] = "ledman";
const uint8_t COMMAND_LED_ON[]  = "ledon";
const uint8_t COMMAND_LED_OFF[] = "ledoff";

// Buffer for command
static uint8_t line_ready_buffer[LINE_BUFFER_SIZE]; // Stable buffer for main

// Audio buffer
int16_t audio_buffer[AUDIO_BUF_SIZE];// the new buffer I set
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

// Interrupt handlers
void handle_new_line();

// Commands
void go_to_stop();

// Helper functions
void init_codec_and_play();

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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim10);//pwm
  HAL_TIM_Base_Start_IT(&htim11);//acc
  BSP_ACCELERO_Init();//acc init
  
  /*audio*/
  update_audio_buffer(); 
  cs43l22_init();
  cs43l22_play(audio_buffer, AUDIO_BUF_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // We check if that there are NO interrupts pending before going into sleep
    if (isr_flags == 0)
    {
      // Go to sleep, waiting for interrupt (WFI).
      HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON,PWR_SLEEPENTRY_WFI);
    }
    // This is needed for the UART transmission 
    if (isr_flags & ISR_FLAG_RX)
    {
      isr_flags &= ~ISR_FLAG_RX;
      handle_new_line();
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    //PWM_changefreq
    if (isr_flags & ISR_FLAG_TIM10)
    {
      isr_flags &= ~ISR_FLAG_TIM10;
      handle_timer10_pwm();
    }
    //PWM_manual_control
    if (isr_flags & ISR_FLAG_BTN)
    {
      isr_flags &= ~ISR_FLAG_BTN;
      handle_button();
    }
    //acc
    if (isr_flags & ISR_FLAG_TIM11)
    {
        isr_flags &= ~ISR_FLAG_TIM11; 
        handle_accel();              
    }
    
  }
  /* USER CODE END 3 */
}


/* USER CODE BEGIN 4 */
/*set interrupt flags*/


// All of this is used to manage commands from serial interface
static uint8_t line_buffer[LINE_BUFFER_SIZE];
static uint32_t line_len = 0;
void CDC_ReceiveCallBack(uint8_t *buf, uint32_t len)
{
  // Prevent overflow, does not handle the command
  if (line_len + len >= LINE_BUFFER_SIZE)
  {
    line_len = 0;
    return;
  }

  // Append received chunk
  memcpy(&line_buffer[line_len], buf, len);
  line_len += len;

  // Process all complete lines
  while (1)
  {
    // Look for '\n' or '\r' inside line_buffer
    uint8_t *line_feed = memchr(line_buffer, '\n', line_len);
    if (line_feed == NULL){
      line_feed = memchr(line_buffer, '\r', line_len);
      if (line_feed == NULL)
      break;
    }

    // Replace \n or \r by terminator
    *line_feed = '\0';

    // Remove optional '\r' in case input was \r\n
    if (line_feed > line_buffer && *(line_feed - 1) == '\r')
    *(line_feed - 1) = '\0';

    uint32_t processed = (line_feed + 1) - line_buffer;

    // Signal the main that there is a new line ready to be checked
    memcpy(line_ready_buffer, line_buffer, processed);
    isr_flags |= ISR_FLAG_RX;

    // Move leftover bytes to start
    line_len -= processed;
    memmove(line_buffer, line_buffer + processed, line_len);
  }
}
//set tim10 and tim11 interrupt flag
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /*PWM_changefreq*/
    if (htim->Instance == TIM10) {
        isr_flags |= ISR_FLAG_TIM10; 
    }
    /*acc*/
    if (htim->Instance == TIM11) {
    isr_flags |= ISR_FLAG_TIM11;
    }

}
//Set button interrupt flag
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_BUTTON_Pin) {
        isr_flags |= ISR_FLAG_BTN; 
    }
}



/**
* @brief  Handle possible new command
* @retval None
*/

//process new command
void handle_new_line()
{
  if (memcmp(line_ready_buffer, COMMAND_CHANGE_FREQ, sizeof(COMMAND_CHANGE_FREQ)) == 0)
  {
    CDC_Transmit_FS((uint8_t*)"Change Frequency\r\n", 18);
    change_freq();

  }
  else if (memcmp(line_ready_buffer, COMMAND_CHANGE_DUT, sizeof(COMMAND_CHANGE_DUT)) == 0)
  {
    if (duty_cycle == 0.5f) duty_cycle = 0.75f;
    else if (duty_cycle == 0.75f) duty_cycle = 0.25f;
    else duty_cycle = 0.5f;
    
    CDC_Transmit_FS((uint8_t*)"Duty Changed\r\n", 14);
  }
  else if (memcmp(line_ready_buffer, COMMAND_PWM_MAN, sizeof(COMMAND_PWM_MAN)) == 0)
  {
    pwm_mode_manual = 1;
    CDC_Transmit_FS((uint8_t*)"Manual Mode: Hold Button\r\n", 26);
  }
  
  else if (memcmp(line_ready_buffer, COMMAND_ACC_ON, sizeof(COMMAND_ACC_ON)) == 0)
  {
    acc_enabled = 1; 
    CDC_Transmit_FS((uint8_t*)"ACC ON\r\n", 8);
  }
  else if (memcmp(line_ready_buffer, COMMAND_ACC_OFF, sizeof(COMMAND_ACC_OFF)) == 0)
  {
    acc_enabled = 0; 
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    CDC_Transmit_FS((uint8_t*)"ACC OFF\r\n", 9);
  }

  else if (memcmp(line_ready_buffer, COMMAND_MUTE, sizeof(COMMAND_MUTE)) == 0)
  {
    cs43l22_mute();
    CDC_Transmit_FS((uint8_t*)"Muted\r\n", 7);
  }
  else if (memcmp(line_ready_buffer, COMMAND_UNMUTE, sizeof(COMMAND_UNMUTE)) == 0)
  {
    cs43l22_unmute();
    CDC_Transmit_FS((uint8_t*)"Unmuted\r\n", 9);
  }
 
  else if (memcmp(line_ready_buffer, COMMAND_LED_PWM, sizeof(COMMAND_LED_PWM)) == 0)
  {
    use_pwm_for_led = 1; // PWM 
    CDC_Transmit_FS((uint8_t*)"LED Mode: PWM\r\n", 15);
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_MAN, sizeof(COMMAND_LED_MAN)) == 0)
  {
    use_pwm_for_led = 0; // Manual
    CDC_Transmit_FS((uint8_t*)"LED Mode: Manual\r\n", 18);
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_ON, sizeof(COMMAND_LED_ON)) == 0)
  {
    // Only allow control in manual mode
    if (use_pwm_for_led == 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        CDC_Transmit_FS((uint8_t*)"LED ON\r\n", 8);
    }
  }
  else if (memcmp(line_ready_buffer, COMMAND_LED_OFF, sizeof(COMMAND_LED_OFF)) == 0)
  {
    // Only allow control in manual mode
    if (use_pwm_for_led == 0) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        CDC_Transmit_FS((uint8_t*)"LED OFF\r\n", 9);
    }
  }

  else if (memcmp(line_ready_buffer, COMMAND_STOP, sizeof(COMMAND_STOP)) == 0)
  {
    CDC_Transmit_FS((uint8_t*)"STOP mode\r\n", 11);
    go_to_stop();
  }
  else if (memcmp(line_ready_buffer, COMMAND_STANDBY, sizeof(COMMAND_STANDBY)) == 0) {
    CDC_Transmit_FS((uint8_t*)"Standby\r\n", 9);
    HAL_PWR_EnterSTANDBYMode(); 
  }
  else
  {
    // If we receive an unknown command, we send an error message back to the PC
    CDC_Transmit_FS((uint8_t*)"Unknown command\r\n", 17);
  }
}

//PWM handler
void handle_timer10_pwm(void)
{
    led_state = !led_state; // toggle state
  /* PWM control */
  if (use_pwm_for_led == 1) 
  {
    //base on current led_state to set the pin and ARR(off set 1-duty, on set duty)
    if (led_state) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);
        __HAL_TIM_SET_AUTORELOAD(&htim10, (uint32_t)(pwm_period * duty_cycle));//change ARR value
        //pwm_period: changefreq and button set
        //duty_cycle: changedut command set
    } else {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
        __HAL_TIM_SET_AUTORELOAD(&htim10, (uint32_t)(pwm_period * (1.0f - duty_cycle)));//change ARR value
    }
  }
}

//PWM_manual_control
void handle_button(void)
{
    // Read button state
    if (HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_Port, KEY_BUTTON_Pin) == GPIO_PIN_SET) 
    {
        // Pressed: record start time
        btn_start_time = HAL_GetTick();

        /*LED manual control*/
        if (use_pwm_for_led == 0) {
            HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
        }
   
    }
    else 
    {
        // Released: calculate duration
        uint32_t duration = HAL_GetTick() - btn_start_time;
        
        // pressed and debounce
        if (pwm_mode_manual && duration > 100) {

            //168Mhz/ (16799+1) = 10,000Hz (10 kHz)。
            // Convert milliseconds to timer counts (10kHz timer, 1ms = 10 ticks)
            pwm_period = duration * 10; 
            pwm_mode_manual = 0; // Switch back to automatic mode after change
            __HAL_TIM_SET_AUTORELOAD(&htim10, pwm_period);//ARR update
            char msg[32];
            sprintf(msg, "New Period: %lu ticks\r\n", pwm_period);
            CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
            use_pwm_for_led = 1;
        }
    }
    update_audio_buffer();
}

//accelometer handling
void handle_accel(void)
{
    if (acc_enabled == 0) return;

    int16_t buffer[3] = {0};
    BSP_ACCELERO_GetXYZ(buffer); // read X, Y, Z

    int16_t x = buffer[0];
    int16_t y = buffer[1];
    int16_t z = buffer[2];
    // Turn off all indicator LEDs (PD13,14,15), keep PWM LED (PD12) if not conflicting
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

    if (abs(x) > abs(y) && abs(x) > abs(z)) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
    } 
    else if (abs(y) > abs(x) && abs(y) > abs(z)) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
    } 
    else if (abs(z) > abs(x) && abs(z) > abs(y)) {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
    }    
    else {
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);
    }


}

//PWM_changefreq_fixed_3levels
void change_freq()
{
  static uint8_t freq_state = 0;
  freq_state = (freq_state + 1) % 3;
  switch (freq_state)
  {
    case 0:
      pwm_period = 20000; // Slow
      CDC_Transmit_FS((uint8_t*)"Frequency: Slow\r\n", 18);
      break;
    case 1:
      pwm_period = 10000; // Medium
      CDC_Transmit_FS((uint8_t*)"Frequency: Medium\r\n", 20);
      break;
    case 2:
      pwm_period = 5000; // Fast
      CDC_Transmit_FS((uint8_t*)"Frequency: Fast\r\n", 18);
      break;
    default:
      break;
  }
  update_audio_buffer();
}

// Generate audio buffer based on current pwm_period
void update_audio_buffer(void)
{
   
    float raw_freq = 10000000.0f / (float)pwm_period; 
    float time_samples = (float)(AUDIO_BUF_SIZE / 2); 
    float cycles_in_buffer = raw_freq * time_samples / 48000.0f;
    
    cycles_in_buffer = floorf(cycles_in_buffer + 0.5f); 

    if (cycles_in_buffer < 1.0f) cycles_in_buffer = 1.0f;

    float snapped_freq = cycles_in_buffer * 48000.0f / time_samples;
    
    for(int i = 0; i < AUDIO_BUF_SIZE / 2; i++) 
    {
        int16_t val = (int16_t)(5000.0f * sinf(2.0f * 3.1415926f * snapped_freq * (float)i / 48000.0f));
        
        audio_buffer[2*i]     = val; // left
        audio_buffer[2*i + 1] = val; // right
    }
}
// Initialize codec and start playback
void init_codec_and_play()
{
  update_audio_buffer(); 
  cs43l22_init();
  cs43l22_play(audio_buffer, AUDIO_BUF_SIZE);
}

/**
* @brief Go to stop mode
* @retval None
*/
void go_to_stop()
{
  // TODO: Make sure all user LEDS are off
  // To avoid noise during stop mode
  cs43l22_stop();

  // Required otherwise the audio wont work after wakeup
  HAL_I2S_DeInit(&hi2s3);

  // We disable the systick interrupt before going to stop (1ms tick)
  // Otherwise we would be woken up every 1ms
  HAL_SuspendTick();

  HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

  // We need to reconfigure the system clock after waking up.
  // After exiting from stop mode, the system clock is reset to the default
  // which is not the same we configure in cubeMx, so we do it as done
  // during the init, by calling SystemClock_Config().
  SystemClock_Config();
  HAL_ResumeTick();

  // Required otherwise the audio wont work after wakeup
  MX_I2S3_Init();
  MX_USB_DEVICE_Init();

  init_codec_and_play();
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
