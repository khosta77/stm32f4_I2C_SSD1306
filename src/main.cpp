#include "../system/include/cmsis/stm32f4xx.h"

#include "SH_GPIO.h"
#include "SH_I2C.h"

#define BME280_ADDRESS 0b01110110

#define BME280_ID 0xD0

#if 0
uint8_t _i2c1_data_rx[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t _i2c1_data_tx[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t _i2c1_mrk_rx = 0x00;
uint8_t _i2c1_mrk_rx_work = 0x00;
uint8_t _i2c1_mrk_tx = 0x00;

void DMA1_Stream6_IRQHandler(void) {  // TX
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {
        //GPIOD->ODR ^= GPIO_ODR_OD12;

        _i2c1_mrk_tx = 0x00;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        //while ((DMA1_Stream6->CR) & DMA_SxCR_EN){;}
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
        //GPIOD->ODR ^= GPIO_ODR_OD12;

    }
}

void DMA1_Stream5_IRQHandler(void) {  // RX
    if ((DMA1->HISR & DMA_HISR_TCIF5) == DMA_HISR_TCIF5) {
        _i2c1_mrk_rx = 0x00;
        _i2c1_mrk_rx_work = 0x01;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        //while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
        DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
    }
}



void I2C1_init() {
    // GPIO init
    // PB6 I2C_SCL 
    // PB9 I2C_SDA
    // эти пины выбраны, так как подключены, к 4.7 кОм
#if 0
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_6);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR6_0);
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2;
    GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_2;
#endif
    // I2C init
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    // 1. Устанавливает тактовую частоту периферийного устройства 50 MHz
    I2C1->CR2 |= (I2C_CR2_LAST | I2C_CR2_DMAEN | (I2C_CR2_FREQ_5 | I2C_CR2_FREQ_4 | I2C_CR2_FREQ_1));
    
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
    
    // Настройка DMA1 на чтение и запись по I2C
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    // Отключаем DMA, для чтения
    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
    // Настройка контрольных регистров
    // - (0x01 << 25)     - настройка DMA канала, для I2C1_RX
    // - DMA_SxCR_MINC    - Memory INCrement mode
    // - периферия-память
    // - DMA_SxCR_TCIE    - Прерывания по завершению передачи
    DMA1_Stream5->CR = ((0x01 << 25) | DMA_SxCR_MINC | DMA_SxCR_TCIE);

    DMA1->HIFCR |= DMA_HIFCR_CTCIF5;  // Включаем прерывание после успешной передачи передачи
    DMA1_Stream5->PAR = (uint32_t)&I2C1->DR;
    DMA1_Stream5->M0AR = (uint32_t)&_i2c1_data_rx[0];
    NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    NVIC_SetPriority(DMA1_Stream5_IRQn, 2);

    // Отключаем DMA, для записи
    DMA1_Stream6->CR &= ~DMA_SxCR_EN;
    while ((DMA1_Stream6->CR) & DMA_SxCR_EN){;}
    // Настройка контрольных регистров
    // - (0x01 << 25)     - настройка DMA канала, для I2C1_RX
    // - DMA_SxCR_MINC    - Memory INCrement mode
    // - DMA_SxCR_DIR_0   - память-перефирия
    // - DMA_SxCR_TCIE    - Прерывания по завершению передачи
    DMA1_Stream6->CR = ((0x01 << 25) | DMA_SxCR_MINC | DMA_SxCR_DIR_0 | DMA_SxCR_TCIE);

    DMA1->HIFCR |= DMA_HIFCR_CTCIF6;  // Включаем прерывание после успешной передачи передачи
    DMA1_Stream6->PAR = (uint32_t)&I2C1->DR;
    DMA1_Stream6->M0AR = (uint32_t)&_i2c1_data_tx[0];
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    NVIC_SetPriority(DMA1_Stream6_IRQn, 3);

}

static void DMA_Transmit(const uint8_t *pBuffer, uint32_t size) {
    if((NULL != pBuffer) && (size < 16)) {
        while (_i2c1_mrk_tx != 0x00) {;}  // Ждем пока предыдущая передача не закончится
        _i2c1_mrk_tx = 0x01;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        while ((DMA1_Stream6->CR) & DMA_SxCR_EN){;}
        for (uint8_t i = 0; i < size; ++i) {
            _i2c1_data_tx[i] = pBuffer[i];
        }

        DMA1_Stream6->NDTR = size;
        DMA1->HIFCR = DMA_HIFCR_CTCIF6;
        DMA1_Stream6->CR |= DMA_SxCR_EN;
        //while (!(I2C1->SR2 & I2C_SR2_BUSY)){;}

    } else {
        /* Null pointers, do nothing */
    }
}

static void DMA_Receive(const uint8_t size) {
    if(size < 16) {
        while (_i2c1_mrk_rx != 0x00) {;}  // Ждем пока предыдущая передача не закончится
        _i2c1_mrk_rx = 0x01;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	    while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
        DMA1_Stream5->NDTR = size;
        DMA1->HIFCR = DMA_HIFCR_CTCIF5;
        DMA1_Stream5->CR |= DMA_SxCR_EN;
    } else {
        //GPIOD->ODR ^= GPIO_ODR_OD12;
        /* Null pointers, do nothing */
    }
}

static void DMA_Receive_read(uint8_t *pBuffer, uint8_t size) {
    if((NULL != pBuffer) && (size < 16)) {
        while (_i2c1_mrk_rx_work == 0x00) {;}  // Ждем пока предыдущая передача не закончится
        _i2c1_mrk_rx_work = 0x00;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
	    while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
        for (uint8_t i = 0; i < size; ++i) {
            *(pBuffer + i) = _i2c1_data_rx[i];
        }
    } else {
        //GPIOD->ODR ^= GPIO_ODR_OD12;
        /* Null pointers, do nothing */
    }
}

// Такая DMA передача может крайне плохо работать
uint8_t I2C1_Read_with_DMA(const uint8_t _reg_addr) {
    uint8_t _data[1] = { _reg_addr};

    // 0.  Ждем не занят ли шина I2C
    while (I2C1->SR2 & I2C_SR2_BUSY);

    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
	
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));
	
    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (BME280_ADDRESS << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR1;
    (void) I2C1->SR2;

    // 4. Начинаем DMA отправку
    DMA_Transmit(&_data[0], 1);
	(void)I2C1->SR1;
	(void)I2C1->SR2;

    // 5. Ожидаем завершение передачи
	while(!(I2C_SR1_BTF & I2C1->SR1)){;} 

    // 6. Производим рестарт
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){};

	// 7. Передаем адрес устройства, но теперь для чтения
	I2C1->DR = ((BME280_ADDRESS << 1) | 0x01);
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};
	(void) I2C1->SR1;
	(void) I2C1->SR2;

	// 8. Считываем данные
    DMA_Receive(1);
    DMA_Receive_read(&_data[0], 1);

    // 9. Производим остановку
	I2C1->CR1 |= I2C_CR1_STOP;

	return _data[0];
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

uint8_t I2C1_Read(const uint8_t _reg_addr) {
    uint8_t _data = 0x00;

    // 0.  Ждем не занят ли шина I2C
    while (I2C1->SR2 & I2C_SR2_BUSY);

    // 1. Запускаем передачу
	I2C1->CR1 |= I2C_CR1_START;
	
    // 2. Ждем пока будет отправлен начальный бит
	while (!(I2C1->SR1 & I2C_SR1_SB));
	
    // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
    //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	I2C1->DR = (BME280_ADDRESS << 1);
    while (!(I2C1->SR1 & I2C_SR1_ADDR));
    (void) I2C1->SR1;
    (void) I2C1->SR2;

    // 4. Передаем адрес регистра
	I2C1->DR = _reg_addr;
	while(!(I2C1->SR1 & I2C_SR1_TXE)){};

    // 5. Останавливаем передачу
	I2C1->CR1 |= I2C_CR1_STOP;

    // 6. Производим рестарт
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){};

	// 7. Передаем адрес устройства, но теперь для чтения
	I2C1->DR = ((BME280_ADDRESS << 1) | 0x01);
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){};
	(void) I2C1->SR1;
	(void) I2C1->SR2;

	// 8. Считываем данные
	I2C1->CR1 &= ~I2C_CR1_ACK;
	while(!(I2C1->SR1 & I2C_SR1_RXNE)){};
	_data = I2C1->DR;

    // 9. Производим остановку
	I2C1->CR1 |= I2C_CR1_STOP;

	return _data;
}
#endif

SH_GPIO PD12_LED_GREEN(GPIOD, 12, 0x40);

SH_GPIO PB6_I2C1_SCL(GPIOB, 6, 0xA3, 0x04);
SH_GPIO PB9_I2C1_SDA(GPIOB, 9, 0xA3, 0x04);

SH_I2C I2C1_FS(I2C1, 0x01, 50);

void I2C1_init() {
    I2C1_FS._ClockInit();
    I2C1_FS._enable();
}

void GPIOD_init() {
    PD12_LED_GREEN._ON();
//    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
//    GPIOD->MODER |= GPIO_MODER_MODER12_0;
}

int main(void) {
    I2C1_init();
    GPIOD_init();
    uint8_t id = 0;
    while (1) {
        id = I2C1_FS._read_reg(BME280_ADDRESS, BME280_ID);
    }
}
