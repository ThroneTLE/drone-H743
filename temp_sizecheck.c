#include <stdio.h>
#include <stdint.h>
#include "App/Inc/app_messages.h"
int main(void){
    printf("APP_UART_TxMessage=%zu\n", sizeof(APP_UART_TxMessage));
    printf("APP_IMU_SampleMessage=%zu\n", sizeof(APP_IMU_SampleMessage));
    return 0;
}
