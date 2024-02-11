#ifndef STEPAN_HAL_I2C_H_
#define STEPAN_HAL_I2C_H_

#include "../system/include/cmsis/stm32f4xx.h"

#include "SH_DMA_Sx.h"

#if 0
void buffer() {
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
#endif


class SH_I2C {
    I2C_TypeDef *_I2Cx;

    SH_DMA_Sx<I2C_TypeDef, uint8_t> *_DMA_TX;
    SH_DMA_Sx<I2C_TypeDef, uint8_t> *_DMA_RX;

    /* 
     * 01234567
     * -+------------------------------------
     * 0|0 - DMA off
     *  |1 - DMA on, после передачи сработает 
     * -+------------------------------------
     * 1|
     * -+------------------------------------
     * 2|
     * -+------------------------------------
     * 3|
     * -+------------------------------------
     * 4|
     * -+------------------------------------
     * 5|
     * -+------------------------------------
     * 6|0 - DUTY off
     *  |1 - DUTY on
     * -+------------------------------------
     * 7|0 - Sm
     *  |1 - Fm
     * -+------------------------------------
     * */
    uint8_t _options;
    uint8_t _slave_freq;
    
    float t_pclk1;
    float I2C_mode_speed = 0.00001f;  // Это наносекунды
    float I2C_trice = 0.0000001f;

    
public:
    SH_I2C(I2C_TypeDef *I2Cx,
           const uint8_t &options,
           const uint8_t &slave_freq,
           SH_DMA_Sx<I2C_TypeDef, uint8_t> *DMA_TX,
           SH_DMA_Sx<I2C_TypeDef, uint8_t> *DMA_RX
           ) : _I2Cx(I2Cx), _options(options), _slave_freq(slave_freq), _DMA_TX(DMA_TX), _DMA_RX(DMA_RX) {
        
        if (_I2Cx == I2C1)
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        if (_I2Cx == I2C2)
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        if (_I2Cx == I2C3)
            RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

        // Найстройка DMA
        if ((_options & 0x80) == 0x80) {
            _I2Cx->CR2 |= (I2C_CR2_LAST | I2C_CR2_DMAEN);
            
#if 0
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
#endif
        }

        if ((_options & 0x01) == 0x01) {
            _I2Cx->CCR |= I2C_CCR_FS;
            I2C_mode_speed = 0.0000025f;
            I2C_trice = 0.0000003f;
        }

        if ((_options & 0x02) == 0x02)
            _I2Cx->CCR |= I2C_CCR_DUTY;

        _I2Cx->CR2 |= (_slave_freq & 0x3F);

    }

private:
    void t_pclk1_calculated() {
        uint32_t f_apb1 = 0;
        uint32_t divided = 0;
        uint32_t cfgr = RCC->CFGR;

        SystemCoreClockUpdate();
        f_apb1 = SystemCoreClock;

        if ((cfgr & 0x00000080) == 1) {
            divided = (((cfgr & 0x00000070) >> 4));
            f_apb1 /= (0x00000001 << divided);
        }

        if ((cfgr & 0x00001000) == 1) {
            divided = (((cfgr & 0x00000C00) >> 10) + 2);
            f_apb1 /= (0x00000001 << divided);
        }

        t_pclk1 = 1.0f / (f_apb1 * 1.0f);
    }

    inline float get_coefficient() {
        float coefficient = 2;
        if ((_options & 0x01) == 0x01)
            if ((_options & 0x02) != 0x02)
                coefficient = 3;
            else
                coefficient = 25;
        return coefficient;
    }

    void _clock_ccr_init() {
        const float coefficient = get_coefficient();
        const uint16_t ccr = (((uint16_t)(I2C_mode_speed / (coefficient * t_pclk1))) & 0x0FFF);
        _I2Cx->CCR |= ccr;
    }

    void _clock_trise_init() {
        const uint16_t trice = ((((uint16_t)(I2C_trice / t_pclk1)) + 1) & 0x003F);
        _I2Cx->TRISE |= trice;
    }

public:
    void _ClockInit() {
        t_pclk1_calculated();
        _clock_ccr_init();
        _clock_trise_init();
    }

    void _enable() {
        _I2Cx->CR1 |= I2C_CR1_PE;
    }

    void _write_reg(const uint8_t &_address, const uint8_t &_register, const uint8_t &_data) {
        // 0. Ждем не занят ли шина I2C
	    while (_I2Cx->SR2 & I2C_SR2_BUSY);

        // 1. Запускаем передачу
	    _I2Cx->CR1 |= I2C_CR1_START;
    
        // 2. Ждем пока будет отправлен начальный бит
	    while (!(_I2Cx->SR1 & I2C_SR1_SB));

        // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
        //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	    _I2Cx->DR = (_address << 1);
        while (!(_I2Cx->SR1 & I2C_SR1_ADDR));
	    (void) _I2Cx->SR1;
	    (void) _I2Cx->SR2;

	    // 4. Передаем регистр
	    _I2Cx->DR = _register;
	    while (!(_I2Cx->SR1 & I2C_SR1_TXE));

	    // 5. Передаем данные
	    _I2Cx->DR = _data;
	    while (!(_I2Cx->SR1 & I2C_SR1_TXE));

	    // 6. Останавливаем передачу
	    _I2Cx->CR1 |= I2C_CR1_STOP;
    }

    uint8_t _read_reg(uint8_t _address, const uint8_t &_register) {
        uint8_t _data = 0x00;

        // 0.  Ждем не занят ли шина I2C
        while (_I2Cx->SR2 & I2C_SR2_BUSY);

        // 1. Запускаем передачу
	    _I2Cx->CR1 |= I2C_CR1_START;
	
        // 2. Ждем пока будет отправлен начальный бит
	    while (!(_I2Cx->SR1 & I2C_SR1_SB));
	
        // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
        //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	    _I2Cx->DR = (_address << 1);
        while (!(_I2Cx->SR1 & I2C_SR1_ADDR));
        (void) _I2Cx->SR1;
        (void) _I2Cx->SR2;

        // 4. Передаем адрес регистра
	    _I2Cx->DR = _register;
	    while(!(_I2Cx->SR1 & I2C_SR1_TXE)){};

        // 5. Останавливаем передачу
    	_I2Cx->CR1 |= I2C_CR1_STOP;

        // 6. Производим рестарт
	    _I2Cx->CR1 |= I2C_CR1_START;
	    while(!(_I2Cx->SR1 & I2C_SR1_SB)){};

	    // 7. Передаем адрес устройства, но теперь для чтения
	    _I2Cx->DR = ((_address << 1) | 0x01);
	    while(!(_I2Cx->SR1 & I2C_SR1_ADDR)){};
	    (void) _I2Cx->SR1;
	    (void) _I2Cx->SR2;

	    // 8. Считываем данные
	    _I2Cx->CR1 &= ~I2C_CR1_ACK;
	    while(!(_I2Cx->SR1 & I2C_SR1_RXNE)){};
	    _data = _I2Cx->DR;

        // 9. Производим остановку
    	_I2Cx->CR1 |= I2C_CR1_STOP;

	    return _data;
    }

    void _dma_write_reg(const uint8_t &_address, const uint8_t *_data, const uint16_t &_size) {
        // 0. Проверяем не осуществляется ли сейчас чтение
        while (_DMA_RX->_is_work()) {};

        // 1. Ждем не занят ли шина I2C
	    while (_I2Cx->SR2 & I2C_SR2_BUSY);

        // 2. Запускаем передачу
	    _I2Cx->CR1 |= I2C_CR1_START;

        // 3. Ждем пока будет отправлен начальный бит
	    while (!(_I2Cx->SR1 & I2C_SR1_SB)){};

        // 4. Отправляем в канал адрес, для того чтобы происходила запись данных \
        //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	    _I2Cx->DR = (_address << 1);
        while (!(_I2Cx->SR1 & I2C_SR1_ADDR));

        _DMA_TX->dma_transmit(_data, _size);
	    (void) _I2Cx->SR1;
	    (void) _I2Cx->SR2;
	}

    void _dma_read_reg(const uint8_t &_address, const uint8_t &_register, const uint16_t &_size) {
        while (_DMA_TX->_is_work()) {};

        // 0.  Ждем не занят ли шина I2C
        while (_I2Cx->SR2 & I2C_SR2_BUSY);

        // 1. Запускаем передачу
	    _I2Cx->CR1 |= I2C_CR1_START;

        // 2. Ждем пока будет отправлен начальный бит
	    while (!(_I2Cx->SR1 & I2C_SR1_SB));

        // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
        //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	    _I2Cx->DR = (_address << 1);
        while (!(_I2Cx->SR1 & I2C_SR1_ADDR));
        (void) _I2Cx->SR1;
        (void) _I2Cx->SR2;

        // 4. Передаем адрес регистра
	    _I2Cx->DR = _register;
	    while(!(_I2Cx->SR1 & I2C_SR1_TXE)){};

        // 5. Останавливаем передачу
    	_I2Cx->CR1 |= I2C_CR1_STOP;

        // 6. Производим рестарт
	    _I2Cx->CR1 |= I2C_CR1_START;
	    while(!(_I2Cx->SR1 & I2C_SR1_SB)){};

	    // 7. Передаем адрес устройства, но теперь для чтения
	    _I2Cx->DR = ((_address << 1) | 0x01);
	    while(!(_I2Cx->SR1 & I2C_SR1_ADDR)){};
        
        if (_size < 2) {
            _I2Cx->CR1 &= ~I2C_CR1_ACK;
        } else {
            _I2Cx->CR1 |= I2C_CR1_ACK;
        }
        _DMA_RX->dma_receive(_size);
	    (void) _I2Cx->SR1;
	    (void) _I2Cx->SR2;
    }

    uint8_t *_dma_read_receive() {
        while (_DMA_RX->_is_work()) {};
        _I2Cx->CR1 |= I2C_CR1_STOP;

        return _DMA_RX->dma_receive_read();
    }
};

#endif  // STEPAN_HAL_I2C_H_
