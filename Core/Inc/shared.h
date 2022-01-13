#ifndef __SHARED_H
#define __SHARED_H

#ifdef __cplusplus
extern "C" {
#endif

extern UART_HandleTypeDef huart1;

#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOB

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

