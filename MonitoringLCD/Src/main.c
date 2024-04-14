#include <string.h>
#include "stm32f76xxx.h"

int main(void)
{
	GPIO_Handle_t LED_Handle;

	memset(&LED_Handle, 0, sizeof(LED_Handle));

	LED_Handle.pGPIOx = GPIOB;
	LED_Handle.PinConfig.PinNumber = GPIO_PIN_NUMBER_7;
	LED_Handle.PinConfig.PinMode = GPIO_MODE_OUTPUT;
	LED_Handle.PinConfig.OutputSpeed = GPIO_PIN_SPEED_VERY_HIGH;
	LED_Handle.PinConfig.OutputType = GPIO_OUTPUT_TYPE_PP;
	LED_Handle.PinConfig.PullUpPullDown = GPIO_PUPD_NO_PUPD;

	GPIOB_PCLK_ENABLE();

	GPIO_Initialize(&LED_Handle);

	while(1)
	{
		GPIO_write_to_pin(GPIOB, GPIO_PIN_NUMBER_7, GPIO_PIN_SET);
	}

	return 0;
}
