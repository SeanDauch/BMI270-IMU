#include "stm32f4xx.h"

// all port A
#define CS_pin 4
#define SCK_pin 5
#define MISO_pin 6
#define MOSI_pin 7

// ---------------------------------- SPI --------------------------------------

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

void _cs_enable(){
    GPIOA->ODR &= ~(GPIO_ODR_ODR_4);
}

void _cs_disable(){
    GPIOA->ODR |=  (GPIO_ODR_ODR_4);
}

// send 0 if reading, doesnt include CS
uint8_t _spi_transmit(uint8_t send_data){

    while(!(SPI1->SR & SPI_SR_TXE)){}

    SPI1->DR = send_data;

    while(SPI1->SR & SPI_SR_BSY){}

    uint8_t read_data = SPI1->DR;
}

// -------------------------------- Sensor -------------------------------------
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

enum communication_status{
    OK,
    ERROR
};

enum communication_status BMI270_init(){

    _spi_init();

// sensor init found in data sheet

    // test communication
    data_from_address(0x00);
    uint8_t chip_id = _read_data_from_address(0x00);
    if(chip_id != 0x24){
        return ERROR;
    }

    // initialization squence
    
}