#include "main.h"
#include "stm32f3xx_it.h"
#include <stdbool.h>
#include <stdio.h>


typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} pin_type;

typedef struct {
	pin_type digit_activators[4];
	pin_type BCD_input[4];
	uint32_t digits[4];
	uint32_t number;
} seven_segment_type;

extern char timestr[30];
extern int volume_degree;
extern int warnCount;
extern int critical_LDR_deg;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern RTC_TimeTypeDef myTime;
extern RTC_DateTypeDef myDate;
extern RTC_HandleTypeDef hrtc;


volatile int current_state = 0;

int dim_step = 0;
int LED_count = 0;
int LED_intensity = 0;
int p_state = 0;
int wave = 1;
int counter = 0;

bool isOn = false;

FILE *fptr;

volatile seven_segment_type seven_segment = { .digit_activators = { { .port =GPIOD, .pin = GPIO_PIN_0 },
																	{ .port = GPIOD, .pin = GPIO_PIN_1 },
																	{ .port = GPIOD, .pin = GPIO_PIN_2 },
																	{ .port = GPIOD, .pin = GPIO_PIN_3 } },
											.BCD_input = { { .port = GPIOD, .pin = GPIO_PIN_4 },
														   { .port = GPIOD, .pin = GPIO_PIN_5 },
														   { .port = GPIOD, .pin = GPIO_PIN_6 },
														   { .port = GPIOD, .pin = GPIO_PIN_7 } },
											.digits = { 0, 0, 0, 0 }, .number = 0 };


void seven_segment_deactivate_digits()
{
	for (int i = 0; i < 4; ++i)
	{
		HAL_GPIO_WritePin(seven_segment.digit_activators[i].port,
				seven_segment.digit_activators[i].pin, GPIO_PIN_SET);
	}

}

void seven_segment_activate_digit(uint32_t d)
{
	if ((d < 4))
	{
		HAL_GPIO_WritePin(seven_segment.digit_activators[d].port,
				seven_segment.digit_activators[d].pin, GPIO_PIN_RESET);
	}
}

void seven_segment_display_decimal(uint32_t n)
{
    if (n < 10)
    {
        HAL_GPIO_WritePin(seven_segment.BCD_input[0].port, seven_segment.BCD_input[0].pin,
                          (n & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(seven_segment.BCD_input[1].port, seven_segment.BCD_input[1].pin,
                          (n & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(seven_segment.BCD_input[2].port, seven_segment.BCD_input[2].pin,
                          (n & 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(seven_segment.BCD_input[3].port, seven_segment.BCD_input[3].pin,
                          (n & 8) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

void digit_blinking_seven_segment_refresh(void)
{

	static uint32_t state = 0;
	static uint32_t last_time = 0;
	static uint32_t blink_duration = 0;
	static bool blink = false;

		seven_segment_display_decimal(seven_segment.digits[state]);
		seven_segment_deactivate_digits();
		if (state == current_state)
		{
			HAL_GPIO_WritePin(
					seven_segment.digit_activators[current_state].port,
					seven_segment.digit_activators[current_state].pin, blink);
			if (HAL_GetTick() - blink_duration > 500)
			{

				blink = !blink;
				blink_duration = HAL_GetTick();
			}

			seven_segment_display_decimal(seven_segment.digits[state]);

		} else
		{
			seven_segment_activate_digit(state);
		}
		state = (state + 1) % 4;
		last_time = HAL_GetTick();

}

void seven_segment_refresh(void)
{

		static uint32_t state = 0;
		static uint32_t last_time = 0;

		if (HAL_GetTick() - last_time > 5)
		{
			seven_segment_deactivate_digits();
			seven_segment_activate_digit(state);
			seven_segment_display_decimal(seven_segment.digits[state]);
			state = (state + 1) % 4;
			last_time = HAL_GetTick();
		}
}

void blinking_seven_segment_refresh(void)
{

	static uint32_t last_time = 0;
	if(isOn && HAL_GetTick() - last_time > 500 )
	{
		seven_segment_deactivate_digits();
		last_time = HAL_GetTick();
		isOn = !isOn;
	}
	else if (!isOn && HAL_GetTick() - last_time > 500)
	{
		last_time = HAL_GetTick();
		isOn = !isOn;
	}

	if(isOn)
	{
		seven_segment_refresh();
	}

}

void programLoop()
{
	switch(p_state)
	{
	case 0:
		seven_segment_refresh();
		break;
	case 1:
		digit_blinking_seven_segment_refresh();
		break;
	case 2:
		blinking_seven_segment_refresh();
		break;
	}
}

void seven_segment_set_num(uint32_t n) {
    if (n < 10000) {
        seven_segment.number = n;
        for (uint32_t i = 0; i < 4; ++i) {
            seven_segment.digits[3 - i] = n % 10;
            n /= 10;
        }
    }
}

void LED_Handler() {

	 switch (seven_segment.digits[1])
	 {
	 case 0:
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	 break;

	 case 1:
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	 break;

	 case 2:
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	 break;

	 case 3:
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
	 break;

	 case 4:
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, LED_intensity);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, LED_intensity);
	 break;

	 }

}

void board_LED_handler(int status)
{

	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, status);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, status);

}

void critical()
{
	dim_step = seven_segment.digits[0];
	LED_count = seven_segment.digits[1];

	seven_segment_set_num(critical_LDR_deg);

	p_state = 2;

	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
	 __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);

}

void uncritical()
{
	seven_segment.digits[0] = dim_step;
	seven_segment.digits[1] = LED_count;
	seven_segment.digits[2] = wave;
	seven_segment.digits[3] = warnCount;
	p_state = 1;

	LED_Intensity();
	LED_Handler();


}

void LED_Intensity()
{

	LED_intensity = (seven_segment.digits[0] * 100) + volume_degree;


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	static uint32_t last_enter = 0;

	if (HAL_GetTick() - last_enter < 200)
		return;

	last_enter = HAL_GetTick();

	if(p_state == 0)
	{
		if(dim_step == 0 && LED_count == 0 && wave == 0)
		{
			seven_segment_set_num(0);

		}
		else
		{
			seven_segment.digits[0] = dim_step;
			seven_segment.digits[1] = LED_count;
			seven_segment.digits[2] = wave;
			seven_segment.digits[3] = warnCount;

		}
		board_LED_handler(0);
		p_state = 1;

	}

	else if(p_state == 1)
	{

		if (GPIO_Pin == GPIO_PIN_8)
		{
			int temp = seven_segment.digits[current_state];
			HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
			uint32_t length = sprintf(timestr, "[INFO][%2d:%2d:%2d]\n", myTime.Hours, myTime.Minutes, myTime.Seconds);
			HAL_UART_Transmit(&huart1, timestr, length, HAL_MAX_DELAY);

			char log[30];
			int len = sprintf(log,"[INFO] Digit %d Decreased\n", current_state + 1);
			HAL_UART_Transmit(&huart1, log, len, HAL_MAX_DELAY);

			if (current_state == 1) // Lights
			{
				seven_segment.digits[current_state] = (temp == 0) ? (4) : (--temp);

			}
			else if (current_state == 2) // warn count
			{
				if (temp == 0)
					seven_segment.digits[current_state] = 3;
				else
					seven_segment.digits[current_state] = (temp == 1) ? (3) : (--temp);

			}
			else
			{
				HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
				HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
				uint32_t length = sprintf(timestr, "[INFO][%2d:%2d:%2d]\n", myTime.Hours, myTime.Minutes, myTime.Seconds);
				HAL_UART_Transmit(&huart1, timestr, length, HAL_MAX_DELAY);

				HAL_UART_Transmit(&huart1, "[INFO] DimStep Decreased\n", 25, HAL_MAX_DELAY);
				seven_segment.digits[current_state] = (temp == 0) ? (9) : (--temp);
			}
		}
		else if (GPIO_Pin == GPIO_PIN_9)
		{
			if (++current_state >= 3)
			{
				current_state = 0;
			}

			HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
			uint32_t length = sprintf(timestr, "[INFO][%2d:%2d:%2d]\n", myTime.Hours, myTime.Minutes, myTime.Seconds);
			HAL_UART_Transmit(&huart1, timestr, length, HAL_MAX_DELAY);

			HAL_UART_Transmit(&huart1, "[INFO] Digit changed\n", 21, HAL_MAX_DELAY);

		}
		else if (GPIO_Pin == GPIO_PIN_10)
		{
			int temp = seven_segment.digits[current_state];
			HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
			HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
			uint32_t length = sprintf(timestr, "[INFO][%2d:%2d:%2d]\n", myTime.Hours, myTime.Minutes, myTime.Seconds);
			HAL_UART_Transmit(&huart1, timestr, length, HAL_MAX_DELAY);

			char log[30];
			int len = sprintf(log,"[INFO] Digit %d Increased\n", current_state + 1);
			HAL_UART_Transmit(&huart1, log, len, HAL_MAX_DELAY);

			if (current_state == 1) // Lights
			{
				seven_segment.digits[current_state] = (temp == 4) ? (0) : (++temp);
			}
			else if (current_state == 2) // warn count
			{

				seven_segment.digits[current_state] = (temp == 3) ? (1) : (++temp);
			}
			else
			{
				HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
				HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
				uint32_t length = sprintf(timestr, "[INFO][%2d:%2d:%2d]\n", myTime.Hours, myTime.Minutes, myTime.Seconds);
				HAL_UART_Transmit(&huart1, timestr, length, HAL_MAX_DELAY);

				HAL_UART_Transmit(&huart1, "[INFO] DimStep Increased\n", 25, HAL_MAX_DELAY);

				seven_segment.digits[current_state] = (temp == 9) ? (0) : (++temp);
			}
		}

		if (current_state == 0 || current_state == 1)
		{
			LED_Intensity();
			LED_Handler();
		}
		if (current_state == 2)
		{
			wave = seven_segment.digits[current_state];
			char wave_type[15];
			if(wave)
			{
				switch(wave)
				{
				case 1:
					sprintf(wave_type,"sin");
					break;
				case 2:
					sprintf(wave_type,"square");
					break;
				case 3:
					sprintf(wave_type,"triangular");
					break;
				}
				HAL_RTC_GetDate(&hrtc, &myDate, RTC_FORMAT_BIN);
				HAL_RTC_GetTime(&hrtc, &myTime, RTC_FORMAT_BIN);
				uint32_t length = sprintf(timestr, "[INFO][%2d:%2d:%2d]\n", myTime.Hours, myTime.Minutes, myTime.Seconds);
				HAL_UART_Transmit(&huart1, timestr, length, HAL_MAX_DELAY);

				char log[50];
				int len = sprintf(log,"[INFO] Wave changed to %s\n", wave_type);
				HAL_UART_Transmit(&huart1, log, len, HAL_MAX_DELAY);
			}

		}
	}

}
