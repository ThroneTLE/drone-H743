#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>

typedef enum {
    LED_RED = 0,   /* PC13 */
    LED_1,         /* PC6, reused as Ai-WB2 EN on current debug wiring */
    LED_2,         /* PC7  */
    LED_3,         /* PC8  */
    LED_4,         /* PC9  */
    LED_COUNT
} BSP_LED_ID;

void BSP_LED_Init(void);
void BSP_LED_On(BSP_LED_ID id);
void BSP_LED_Off(BSP_LED_ID id);
void BSP_LED_Toggle(BSP_LED_ID id);

#endif
