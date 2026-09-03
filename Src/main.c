#include "stm32f411xe.h"
#include "stm32f4xx.h"
#include <stdint.h>
#include "BMI270_drivers.h"

// for cube monitor
volatile int32_t heartbeat = 0;
volatile double accel_pitch = 0;
volatile double accel_roll = 0;
volatile double gyro_pitch = 0;
volatile double gyro_roll = 0;
volatile double comp_pitch = 0;
volatile double comp_roll = 0;

// c version of asm code found in cortexm4 refrence manual
void enable_FPU(){
    SCB->CPACR |= (0xf<<20);
}

// basic complementary filter, tuning done in function
double complementary_filter(double accel_angle, double gyro_change, double old_filter_angle){

    // change for tuning (closer to 1 means trust gyro)
    double a = 0.99;

    double gyro_angle = gyro_change + old_filter_angle;

    double return_angle = (a*gyro_angle) + ((1-a)*accel_angle);

    return return_angle;
}

// enabled for rising edge
void enable_EXTI0(){
    EXTI->IMR |= EXTI_IMR_IM0;
    EXTI->RTSR |= EXTI_RTSR_TR0;

    // ! for testing
    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1; //pull down

    NVIC_EnableIRQ(EXTI0_IRQn);
}

// triggered when gyro data ready
void EXTI0_IRQHandler(){

    struct data_3D accel_data = get_accel_data();
    accel_pitch = get_pitch_from_accel(&accel_data);
    accel_roll = get_roll_from_accel(&accel_data);
    struct data_3D gyro_data = get_gyro_data();
    double gyro_pitch_change = integrate_gyro(gyro_data.x);
    double gyro_roll_change = integrate_gyro(gyro_data.y);
    gyro_pitch += gyro_pitch_change;
    gyro_roll += gyro_roll_change;
    comp_pitch = complementary_filter(accel_pitch, gyro_pitch_change, comp_pitch);
    comp_roll = complementary_filter(accel_roll, gyro_roll_change, comp_roll);

    EXTI->PR |= EXTI_PR_PR0;
}

int main(){

    enable_FPU();

    enum init_status status = BMI270_init();

    // keep trying until it works
    while(status != OK){
        status = BMI270_init();
    }

    enable_EXTI0();

    enable_data_ready_interrupt();

    while(1){

        heartbeat++;
    }
    return 1;
}