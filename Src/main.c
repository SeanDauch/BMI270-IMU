#include "stm32f4xx.h"
#include <stdint.h>
#include "BMI270_drivers.h"

// for cube monitor
BMI_settings BMI270_settings;
volatile int32_t heartbeat = 0;

struct data_3D accel_data;
volatile float accel_pitch = 0;
volatile float accel_roll = 0;

struct data_3D gyro_data;
volatile float gyro_pitch = 0;
volatile float gyro_roll = 0;
float gyro_pitch_change = 0;
float gyro_roll_change = 0;

volatile float comp_pitch = 0;
volatile float comp_roll = 0;

// c version of asm code found in cortexm4 refrence manual
void enable_FPU(){
    SCB->CPACR |= (0xf<<20);
}

// basic complementary filter, tuning done in function
float complementary_filter(float accel_angle, float gyro_change, float old_filter_angle){

    // change for tuning (closer to 1 means trust gyro)
    float a = 0.98;

    float gyro_angle = gyro_change + old_filter_angle;

    float return_angle = (a*gyro_angle) + ((1-a)*accel_angle);

    return return_angle;
}

// enabled for rising edge
void enable_EXTI0(){
    EXTI->IMR |= EXTI_IMR_IM0;
    EXTI->RTSR |= EXTI_RTSR_TR0;

    GPIOA->PUPDR |= GPIO_PUPDR_PUPD0_1; //pull down

    NVIC_EnableIRQ(EXTI0_IRQn);
}

uint64_t times_called = 0;
// triggered when gyro data ready
void EXTI0_IRQHandler(){

    // using to avoid reading status register
    times_called ++;

    gyro_data = get_gyro_data(&BMI270_settings);
    gyro_pitch_change = integrate_gyro(&BMI270_settings, gyro_data.x);
    gyro_roll_change = integrate_gyro(&BMI270_settings, gyro_data.y);
    gyro_pitch += gyro_pitch_change;
    gyro_roll += gyro_roll_change;

    
    if(times_called % (BMI270_settings.gyro_frequency/BMI270_settings.acc_frequency) == 1){
        accel_data = get_accel_data(&BMI270_settings);
        accel_pitch = get_pitch_from_accel(&accel_data);
        accel_roll = get_roll_from_accel(&accel_data);
    }

    comp_pitch = complementary_filter(accel_pitch, gyro_pitch_change, comp_pitch);
    comp_roll = complementary_filter(accel_roll, gyro_roll_change, comp_roll);

    EXTI->PR |= EXTI_PR_PR0;
}

int main(){

    enable_FPU();

    enum init_status status = BMI270_init(&BMI270_settings);
    // keep trying until it works
    while(status != OK){
        status = BMI270_init(&BMI270_settings);
    }

    enable_EXTI0();
    enable_data_ready_interrupt();

    while(1){
        heartbeat++;
    }
    return 1;
}