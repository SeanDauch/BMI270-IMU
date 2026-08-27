#include "stm32f4xx.h"
#include <stdint.h>
#include "BMI270_drivers.h"

int main(){

    enum init_status status = BMI270_init();

    // keep trying until it works
    while(status != OK){
        status = BMI270_init();
    }

    while(1){

        struct data_3D accel_data = get_accel_data();
        struct data_3D gyro_data = get_gyro_data();

    }
    return 1;
}