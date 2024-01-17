#include "../system/include/cmsis/stm32f4xx.h"

#define BME280_ADDRESS 0b01110111

void I2C1_init() {
    // GPIO init
    // PB6 I2C_SCL 
    // PB9 I2C_SDA
    // эти пины выбраны, так как подключены, к 4.7 кОм
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_6);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR6_0);
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2;
    GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_2;

    // I2C init
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // 1. Устанавливает тактовую частоту периферийного устройства 50 MHz
    I2C1->CR2 |= (I2C_CR2_FREQ_5 | I2C_CR2_FREQ_4 | I2C_CR2_FREQ_1);  // 0x2A;
    
    // 2. Задаем частоту работы
    // Есть два режима стандартная и повышенная:
    // - Sm, стандартный, скорость 100 кГц = 10  uS = 10 000 nS
    // - Fs, повышенный, скорость 400 кГц  = 2.5 uS =  2 500 nS
    // DUTY скважность
    // Если SM:
    // t_h = CCR * t_PCLK1
    // t_l = CCR * t_PCLK1
    // Если FM:
    //           DUTY = 0       |        DUTY = 1
    // t_h = 01 * CCR * t_PCLK1 | t_h = 09 * CCR * t_PCLK1
    // t_l = 02 * CCR * t_PCLK1 | t_l = 16 * CCR * t_PCLK1
    // Из этого складывается t_SCL = t_h + t_l
    // Пример расчета для 400 kHz, когда FM=1,DUTY=0:
    // CCR * t_PCLK1 + 2 CCR * t_PCLK1 = 2 500 nS
    // CCR = 2500 nS / (3 * t_PCLK1) => округлить до целого и перевести в hex.
    // t_PCLK1 = 1 / f_APB1, к APB1 подключены все I2C
    I2C1->CCR |= (I2C_CCR_FS | 0x00C);
    
    // 3. Определим время нарастания фронта, по-моему ни на что не влияет.
    // Tr = xM
    // - SM 1000 nS
    // - FM 300 nS
    // TRISE = (Tr / t_PCLK1) + 1
    I2C1->TRISE |= 0x0006;
    
    // 4. Включаем переферию
    I2C1->CR1 |= (I2C_CR1_PE);

    // https://hubstub.ru/stm32/184-stm32-i2c.html - помогло
}

void I2C1_write(uint8_t address) {
	// 0. Ждем не занят ли шина I2C
	while (I2C1->SR2 & I2C_SR2_BUSY);
	
    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
	
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));
	
    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = ((address << 1) | 0x01);

	// 4. Ждем пока предет подверждение получения адреса
    uint16_t count = 0;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if (count == 1000) break;
        ++count;
    }

    if (count != 1000) {
        GPIOD->ODR |= GPIO_ODR_OD12;
    }

    (void) I2C1->SR2;
	I2C1->CR1 |= I2C_CR1_STOP;
}

void GPIOD_init() {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER |= GPIO_MODER_MODER12_0;
}

int main(void) {
    I2C1_init();
    GPIOD_init();
    for (uint8_t i = 0; i < 128; i++) {
        I2C1_write(i);
    }
}
