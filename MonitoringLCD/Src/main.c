#include <string.h>
#include "monitoring_hal.h"

#include "FreeRTOS.h"
#include "task.h"

void SystemClock_config(void);

static void Green_LED_Task_Handler(void* parameters);
static void Blue_LED_Task_Handler(void* parameters);
static void Red_LED_Task_Handler(void* parameters);

int main(void)
{
	GPIO_Handle_t GreenLED;
	GPIO_Handle_t BlueLED;
	GPIO_Handle_t RedLED;

	TaskHandle_t GreenLEDTaskHandle;
	TaskHandle_t BlueLEDTaskHandle;
	TaskHandle_t RedLEDTaskHandle;

	BaseType_t status;

	memset(&GreenLED, 0, sizeof(GreenLED));
	memset(&BlueLED, 0, sizeof(BlueLED));
	memset(&RedLED, 0, sizeof(RedLED));

	Monitoring_Initialize();
	SystemClock_config();

	GreenLED.pGPIOx = GPIOB;
	GreenLED.PinConfig.PinNumber = GPIO_PIN_NUMBER_0;
	GreenLED.PinConfig.PinMode = GPIO_MODE_OUTPUT;
	GreenLED.PinConfig.OutputSpeed = GPIO_PIN_SPEED_LOW;
	GreenLED.PinConfig.OutputType = GPIO_OUTPUT_TYPE_PP;
	GreenLED.PinConfig.PullUpPullDown = GPIO_PUPD_NO_PUPD;

	BlueLED.pGPIOx = GPIOB;
	BlueLED.PinConfig.PinNumber = GPIO_PIN_NUMBER_7;
	BlueLED.PinConfig.PinMode = GPIO_MODE_OUTPUT;
	BlueLED.PinConfig.OutputSpeed = GPIO_PIN_SPEED_LOW;
	BlueLED.PinConfig.OutputType = GPIO_OUTPUT_TYPE_PP;
	BlueLED.PinConfig.PullUpPullDown = GPIO_PUPD_NO_PUPD;

	RedLED.pGPIOx = GPIOB;
	RedLED.PinConfig.PinNumber = GPIO_PIN_NUMBER_7;
	RedLED.PinConfig.PinMode = GPIO_MODE_OUTPUT;
	RedLED.PinConfig.OutputSpeed = GPIO_PIN_SPEED_LOW;
	RedLED.PinConfig.OutputType = GPIO_OUTPUT_TYPE_PP;
	RedLED.PinConfig.PullUpPullDown = GPIO_PUPD_NO_PUPD;

	status = xTaskCreate(Green_LED_Task_Handler, "GreenLEDTask", 200, NULL, 2, &GreenLEDTaskHandle);
	status = xTaskCreate(Blue_LED_Task_Handler, "BlueLEDTask", 200, NULL, 2, &BlueLEDTaskHandle);
	status = xTaskCreate(Red_LED_Task_Handler, "RedLEDTask", 200, NULL, 2, &RedLEDTaskHandle);

	GPIOB_PCLK_ENABLE();

	GPIO_Initialize(&GreenLED);
	GPIO_Initialize(&BlueLED);
	GPIO_Initialize(&RedLED);

	vTaskStartScheduler();

	while(1)
	{
	}

	return 0;
}

void SystemClock_config(void)
{
	RCC_OscInit_t RCC_OscInit = {0};
	RCC_CLKInit_t RCC_CLKInit = {0};

	RCC_OscInit.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInit.HSEState = RCC_HSE_BYPASS;
	RCC_OscInit.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInit.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInit.PLL.PLLM = 4;
	RCC_OscInit.PLL.PLLN = 96;
	RCC_OscInit.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInit.PLL.PLLQ = 4;
	RCC_OscInit.PLL.PLLR = 2;

	RCC_Osc_config(&RCC_OscInit);

	RCC_CLKInit.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_CLKInit.SYSCLKSource = RCC_SYSCLKSOURCE_STATUS_PLLCLK;
	RCC_CLKInit.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_CLKInit.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_CLKInit.APB2CLKDivider = RCC_HCLK_DIV1;

	RCC_Clock_config(&RCC_CLKInit, 0x00000003U);
}

static void Green_LED_Task_Handler(void* parameters)
{
	TickType_t last_wakeup_time;
	last_wakeup_time = xTaskGetTickCount();

	while(1)
	{
		GPIO_toggle_pin(GPIOB, GPIO_PIN_NUMBER_0);
		vTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(1000));
	}
}

static void Blue_LED_Task_Handler(void* parameters)
{
	TickType_t last_wakeup_time;
	last_wakeup_time = xTaskGetTickCount();

	while(1)
	{
		GPIO_toggle_pin(GPIOB, GPIO_PIN_NUMBER_7);
		vTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(800));
	}
}

static void Red_LED_Task_Handler(void* parameters)
{
	TickType_t last_wakeup_time;
	last_wakeup_time = xTaskGetTickCount();

	while(1)
	{
		GPIO_toggle_pin(GPIOB, GPIO_PIN_NUMBER_14);
		vTaskDelayUntil(&last_wakeup_time, pdMS_TO_TICKS(400));
	}
}
