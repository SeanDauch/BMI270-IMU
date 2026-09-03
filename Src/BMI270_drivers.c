#include "stm32f4xx.h"
#include "BMI270_config_file.h"
#include "bmi2_defs.h"
#include "BMI270_drivers.h"
#include <math.h>

// all port A
#define CS_pin 4
#define SCK_pin 5
#define MISO_pin 6
#define MOSI_pin 7

// ---------------------------------- SPI --------------------------------------

void _cs_enable(){
    GPIOA->ODR &= ~(GPIO_ODR_ODR_4);
}

void _cs_disable(){
    GPIOA->ODR |=  (GPIO_ODR_ODR_4);
}

// Enables SPI1 for PA4-7
void _spi_init(){

    // --------- Clocks ---------
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // --------- GPIO ---------
    GPIOA->MODER &= ~(GPIO_MODER_MODER4|GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7);
    GPIOA->MODER |= (GPIO_MODER_MODER4_0|GPIO_MODER_MODER5_1|GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1);

    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR4|GPIO_OSPEEDER_OSPEEDR5|GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR4_1|GPIO_OSPEEDER_OSPEEDR5_1|GPIO_OSPEEDER_OSPEEDR6_1|GPIO_OSPEEDER_OSPEEDR7_1); // fast speed (datasheet)

    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL5|GPIO_AFRL_AFRL6|GPIO_AFRL_AFRL7);
    GPIOA->AFR[0] |= (GPIO_AFRL_AFRL5_0|GPIO_AFRL_AFRL5_2|GPIO_AFRL_AFRL6_0|GPIO_AFRL_AFRL6_2|GPIO_AFRL_AFRL7_0|GPIO_AFRL_AFRL7_2); // AF5 (spi)

    // --------- SPI1 ---------
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 &= ~(SPI_CR1_CPOL);
    SPI1->CR1 &= ~(SPI_CR1_CPHA);
    SPI1->CR1 &= ~(SPI_CR1_DFF);
    SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);

    SPI1->CR1 |= (SPI_CR1_SSM); // SSM enabled
    SPI1->CR1 |= (SPI_CR1_SSI);
    _cs_disable(); // active low so set high on start

    SPI1->CR2 &= ~(SPI_CR2_FRF);

    SPI1->CR1 |= (SPI_CR1_MSTR);
    SPI1->CR1 |= (SPI_CR1_SPE);
}

// send 0 if reading, doesnt include CS
uint8_t _spi_transmit(uint8_t send_data){

    while(!(SPI1->SR & SPI_SR_TXE)){}

    SPI1->DR = send_data;

    while(!(SPI1->SR & SPI_SR_RXNE)){}

    uint8_t read_data = SPI1->DR;

    return read_data;
}

// -------------------------------- Sensor -------------------------------------

#define accel_x_off 0
#define accel_y_off 0.02
#define accel_z_off 0.01

void _dummy_byte(){
    _spi_transmit(0);
}

// max address is 0x7E
void _write_data_to_address(uint8_t address, uint8_t data){

    // first bit must be 0 for write
    uint8_t adjusted_addr = address & ~(1<<7);

    _cs_enable();

    //! no dummy byte for writing?
    _spi_transmit(adjusted_addr);
    _spi_transmit(data);

    _cs_disable();
}

// writes series of commands starting at start_addr and incrementing after each
void _burst_write(uint8_t start_addr, uint8_t data_arr[], int arr_size){
    
    // first bit must be 0 for write
    uint8_t adjusted_addr = start_addr & ~(1<<7);

    _cs_enable();
    _spi_transmit(adjusted_addr);

    for(int i = 0; i<arr_size; i++){
        _spi_transmit(data_arr[i]);
    }

    _cs_disable();
}

// max address is 0x7E
uint8_t _read_data_from_address(uint8_t address){

    uint8_t adjusted_addr = address | (1<<7);

    _cs_enable();

    _spi_transmit(adjusted_addr);
    _dummy_byte();
    uint8_t data = _spi_transmit(0);

    _cs_disable();

    return data;
}

// fills array with data starting from start_addr
void _burst_read(uint8_t start_addr, uint8_t read_data[], int arr_size){

    uint8_t adjuster_addr = start_addr | (1<<7);

    _cs_enable();

    _spi_transmit(adjuster_addr);
    _dummy_byte();

    for(int i = 0; i<arr_size; i++){
        read_data[i] = _spi_transmit(0);
    }

    _cs_disable();
}

enum init_status BMI270_init(){

    _spi_init();

    // trigger soft reset for continuos debugging
    _write_data_to_address(BMI2_CMD_REG_ADDR, BMI2_SOFT_RESET_CMD);

// ------------- sensor quick start init found in data sheet -------------------
    // test communication
    uint8_t dummy_1 = _read_data_from_address(BMI2_CHIP_ID_ADDR);
    uint8_t chip_id = _read_data_from_address(BMI2_CHIP_ID_ADDR);
    if(chip_id != 0x24){
        return COMMUNICATION_ERROR;
    }

    // initialization squence
    _write_data_to_address(BMI2_PWR_CONF_ADDR, 0x00); // disable adv_pwr_save
    for(volatile int i = 0; i<1000000; i++){} // wait minimum 450us
    _write_data_to_address(BMI2_INIT_CTRL_ADDR, 0x00); // start initialization

    int config_size = sizeof(bmi270_config_file)/sizeof(bmi270_config_file[0]);
    _burst_write(BMI2_INIT_DATA_ADDR, bmi270_config_file, config_size);

    // make sure that the config file was written correctly
    uint8_t compare_arr[config_size];
    _burst_read(BMI2_INIT_DATA_ADDR, compare_arr, config_size);
    for(int i = 0; i<config_size; i++){
        if(compare_arr[i] != bmi270_config_file[i]){
            return INITIALIZATION_ERROR;
        }
    }

    _write_data_to_address(BMI2_INIT_CTRL_ADDR, 0x01); // complete initialization

    // confirm initialization
    for(volatile int i = 0; i<1000000; i++){} // wait minimum 20ms
    uint8_t init_status = _read_data_from_address(BMI2_INTERNAL_STATUS_ADDR);
    if(init_status != 1){
        return INITIALIZATION_ERROR;
    }

    // configure preformance mode
    _write_data_to_address(BMI2_PWR_CTRL_ADDR, BMI2_GYR_EN_MASK|BMI2_ACC_EN_MASK); // turn on gyro and acc
    _write_data_to_address(BMI2_ACC_CONF_ADDR, 0xA0 | BMI2_ACC_ODR_1600HZ); //1.6kHz no averaging
    _write_data_to_address(BMI2_ACC_RANGE_ADDR, BMI2_ACC_RANGE_4G); // +-4g
    _write_data_to_address(BMI2_GYR_CONF_ADDR, 0xE0 | BMI2_GYR_ODR_1600HZ); //1.6kHz 
    _write_data_to_address(BMI2_GYR_RANGE_ADDR, BMI2_GYR_RANGE_500); // +-500dps
    _write_data_to_address(BMI2_PWR_CONF_ADDR, 0x02);

    for(volatile int i = 0; i<1000000; i++){} // wait minimum 20ms

    uint8_t dummy_2[BMI2_ACC_NUM_BYTES + BMI2_GYR_NUM_BYTES];
    _burst_read(BMI2_ACC_X_LSB_ADDR, dummy_2, BMI2_ACC_NUM_BYTES + BMI2_GYR_NUM_BYTES); // not sure if required but part of sheet

    return OK;
}

// enable INT1 as active high for data ready
void enable_data_ready_interrupt(){

    _write_data_to_address(BMI2_INT_MAP_DATA_ADDR, BMI2_DRDY_INT);

    _write_data_to_address(BMI2_INT1_IO_CTRL_ADDR, BMI2_INT_LEVEL_MASK|BMI2_INT_OUTPUT_EN_MASK);

}

// returns data in g's
struct data_3D get_accel_data(){

    // get raw data
    uint8_t raw_data[BMI2_ACC_NUM_BYTES];

    _burst_read(BMI2_ACC_X_LSB_ADDR, raw_data, BMI2_ACC_NUM_BYTES);

    int16_t x_raw_data = raw_data[0] | (raw_data[1]<<8);
    int16_t y_raw_data = raw_data[2] | (raw_data[3]<<8);
    int16_t z_raw_data = raw_data[4] | (raw_data[5]<<8); 

    // format data
    uint8_t acc_range = pow(2, (1+_read_data_from_address(BMI2_ACC_RANGE_ADDR)));
    uint16_t acc_sensitivity = (INT16_MAX / acc_range)+1;

    double x_data = ((double)x_raw_data / acc_sensitivity) + accel_x_off;
    double y_data = ((double)y_raw_data / acc_sensitivity) + accel_y_off;
    double z_data = ((double)z_raw_data / acc_sensitivity) + accel_z_off;

    struct data_3D return_data = {x_data, y_data, z_data};

    return return_data;
}

// returns data in degrees per second
struct data_3D get_gyro_data(){

    uint8_t raw_data[BMI2_GYR_NUM_BYTES];

    _burst_read(BMI2_GYR_X_LSB_ADDR, raw_data, BMI2_GYR_NUM_BYTES);

    int16_t x_raw_data = raw_data[0] | (raw_data[1]<<8);
    int16_t y_raw_data = raw_data[2] | (raw_data[3]<<8);
    int16_t z_raw_data = raw_data[4] | (raw_data[5]<<8);
    
    // format data
    uint16_t gyr_range = 2000 / pow(2,_read_data_from_address(BMI2_GYR_RANGE_ADDR));
    double gyr_sensitivity = (double)INT16_MAX / gyr_range;

    double x_data = (double)x_raw_data / gyr_sensitivity;
    double y_data = (double)y_raw_data / gyr_sensitivity;
    double z_data = (double)z_raw_data / gyr_sensitivity;

    struct data_3D return_data = {x_data, y_data, z_data};

    return return_data;
}

// rolling around y_axis, follows RHR
double get_roll_from_accel(struct data_3D* accel_data){

    double numerator = accel_data->x;
    double denominator = sqrt(pow(accel_data->y, 2) + pow(accel_data->z, 2));

    double roll_rad = atan2(-1 * numerator, denominator);
    double roll_deg = roll_rad * 180 / M_PI;

    return roll_deg;
}

// pitching around x_axis, follows RHR
double get_pitch_from_accel(struct data_3D* accel_data){

    double numerator = accel_data->y;
    double denominator = sqrt(pow(accel_data->x, 2) + pow(accel_data->z, 2));

    double pitch_rad = atan2(numerator, denominator);
    double pitch_deg = pitch_rad * 180 / M_PI;

    return pitch_deg;

}

// returns degrees changed due to one ODR cycle
double integrate_gyro(double one_axis_gyro_data){

    uint8_t gyr_odr = _read_data_from_address(BMI2_GYR_CONF_ADDR) & 0xF;
    uint16_t gyr_freq = 25 * pow(2, gyr_odr-6);
    double  gyr_period = 1.0/gyr_freq;

    return one_axis_gyro_data * gyr_period;
}