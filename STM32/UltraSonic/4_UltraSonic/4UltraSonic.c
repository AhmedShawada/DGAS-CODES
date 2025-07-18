#include "main.h"

#define NUM_OF_ULTRASONIC 1

typedef enum {
    READY_TO_TRIGGER,
    WAITING_FOR_THE_PULSE,
    READY_TO_CALCULATE
} ULTRASONIC_State_t;

typedef enum {
    ULTRASONIC_0 = 0,
    ULTRASONIC_1,
    ULTRASONIC_2,
    ULTRASONIC_3
} ULTRASONIC_Num_t;

extern TIM_HandleTypeDef htim2;

uint32_t t1[NUM_OF_ULTRASONIC], t2[NUM_OF_ULTRASONIC];
uint16_t ultrasonicReadings[NUM_OF_ULTRASONIC];
uint8_t isFirstCapture[NUM_OF_ULTRASONIC] = {0};

ULTRASONIC_State_t ultrasonicState = READY_TO_TRIGGER;
ULTRASONIC_Num_t ultrasonicNum = ULTRASONIC_0;

GPIO_TypeDef* TRIG_PORTS[NUM_OF_ULTRASONIC] = {GPIOA/*, GPIOA, GPIOA, GPIOA*/};
uint16_t TRIG_PINS[NUM_OF_ULTRASONIC] = {GPIO_PIN_5/*, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3*/};

/* GPIO_TypeDef* TRIG_PORTS[NUM_OF_ULTRASONIC] = {GPIOA, GPIOA, GPIOA, GPIOB};
uint16_t TRIG_PINS[NUM_OF_ULTRASONIC] = {GPIO_PIN_5, GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_0}; */
void triggerUltrasonic(ULTRASONIC_Num_t num) {
    HAL_GPIO_WritePin(TRIG_PORTS[num], TRIG_PINS[num], GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(TRIG_PORTS[num], TRIG_PINS[num], GPIO_PIN_RESET);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    uint8_t current = ultrasonicNum;

    if (ultrasonicState != WAITING_FOR_THE_PULSE) return;

    uint32_t *t1_ptr = &t1[current], *t2_ptr = &t2[current];
    uint8_t *flag = &isFirstCapture[current];
    uint32_t value = 0;

    if (*flag == 0) {
        value = HAL_TIM_ReadCapturedValue(htim, htim->Channel);
        *t1_ptr = value;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, htim->Channel, TIM_INPUTCHANNELPOLARITY_FALLING);
        *flag = 1;
    } else {
        value = HAL_TIM_ReadCapturedValue(htim, htim->Channel);
        *t2_ptr = value;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, htim->Channel, TIM_INPUTCHANNELPOLARITY_RISING);
        HAL_TIM_IC_Stop_IT(htim, htim->Channel);
        ultrasonicState = READY_TO_CALCULATE;
        *flag = 0;
    }
}

void ULTRASONIC_Runnable(void) {
    switch (ultrasonicState) {
        case READY_TO_TRIGGER:
            triggerUltrasonic(ultrasonicNum);
            __HAL_TIM_SET_COUNTER(&htim2, 0);
            switch (ultrasonicNum) {
                case 0: HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); break;
                case 1: HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2); break;
                case 2: HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3); break;
                case 3: HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_4); break;
            }
            ultrasonicState = WAITING_FOR_THE_PULSE;
            break;

        case WAITING_FOR_THE_PULSE:
            // Waiting in interrupt
            break;

        case READY_TO_CALCULATE: {
            uint32_t delta = (t2[ultrasonicNum] >= t1[ultrasonicNum])
                ? (t2[ultrasonicNum] - t1[ultrasonicNum])
                : (0xFFFF - t1[ultrasonicNum] + t2[ultrasonicNum]);

            ultrasonicReadings[ultrasonicNum] = (delta < 23200) ? (delta / 58) : 0;
            ultrasonicNum++;
            if (ultrasonicNum == NUM_OF_ULTRASONIC) ultrasonicNum = ULTRASONIC_0;
            ultrasonicState = READY_TO_TRIGGER;
            break;
        }
    }
}

uint16_t ULTRASONIC_GetDistance(ULTRASONIC_Num_t num) {
    return ultrasonicReadings[num];
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM2_Init();
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);

    while (1) {
        ULTRASONIC_Runnable();
        HAL_Delay(50);
    }
}