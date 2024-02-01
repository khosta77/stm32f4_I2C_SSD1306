#ifndef STEPAN_HAL_I2C_H_
#define STEPAN_HAL_I2C_H_

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
    uint16_t I2C_mode_speed = 10000;
    uint16_t I2C_trice = 1000;

    
public:
    SH_I2C(I2C_TypeDef *I2Cx,
           const uint8_t &options,
           const uint8_t &slave_freq
           ) : _I2Cx(I2Cx), _options(options), _slave_freq(slave_freq) {
        
        if (_I2Cx == I2C1)
            RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        if (_I2Cx == I2C2)
            RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
        if (_I2Cx == I2C3)
            RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;

        // Найстройка DMA
        if ((_options & 0x80) == 1) {
            I2C1->CR2 |= (I2C_CR2_LAST | I2C_CR2_DMAEN);
            
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
            I2C1->CCR |= I2C_CCR_FS;
            I2C_mode_speed = 2500;
            I2C_trice = 300;
        }

        if ((_options & 0x02) == 0x02)
            I2C1->CCR |= I2C_CCR_DUTY;

        I2C1->CR2 |= (_slave_freq & 0x3F);

    }

private:
    void t_pclk1_calculated() {
        uint32_t f_apb1 = 0;
        uint32_t divided = 0;
        uint32_t cfgr = RCC->CFGR;

        SystemCoreClockUpdate();
        f_apb1 = SystemCoreClock;

        if ((cfgr & 0x00000080) == 1) {
            divided = (((cfgr & 0x00000070) >> 4) + 2);
            f_apb1 /= (0x00000001 << divided);
        }

        if ((cfgr & 0x00001000) == 1) {
            divided = (((cfgr & 0x00000C00) >> 10) + 2);
            f_apb1 /= (0x00000001 << divided);
        }

        t_pclk1 = 1 / f_apb1;
    }

    inline uint8_t get_coefficient() {
        uint8_t coefficient = 2;
        if ((_options & 0x01) == 0x01)
            if ((_options & 0x02) == 0x02)
                coefficient = 3;
            else
                coefficient = 25;
        return coefficient;
    }

    void _clock_ccr_init() {
        const uint8_t coefficient = get_coefficient();
        const uint16_t ccr = ((uint16_t)(I2C_mode_speed / (coefficient * t_pclk1)) & 0x0FFF);
        _I2Cx->CCR |= ccr;
    }

    void _clock_trise_init() {
        const uint16_t trice = (((uint16_t)(I2C_trice / t_pclk1) + 1) & 0x003F);
        _I2Cx->TRISE |= trice;
    }

public:
    void _ClockInit() {
        t_pclk1_calculated();
        _clock_ccr_init();
        _clock_trise_init();
    }

    void _enable() {
        I2C1->CR1 |= I2C_CR1_PE;
    }

    void _write_reg(uint8_t _address, uint8_t _register, uint8_t _data) {
        _address = 0;
        _register = 0;
        _data = 0;
        // TODO
    }

    uint8_t _read_reg(uint8_t _address, uint8_t _register) {
        uint8_t _data = 0x00;

        // 0.  Ждем не занят ли шина I2C
        while (I2C1->SR2 & I2C_SR2_BUSY);

        // 1. Запускаем передачу
	    I2C1->CR1 |= I2C_CR1_START;
	
        // 2. Ждем пока будет отправлен начальный бит
	    while (!(I2C1->SR1 & I2C_SR1_SB));
	
        // 3. Отправляем в канал адрес, для того чтобы происходила запись данных \
        //    его надо сместить в лево на 1 бит и оставить ноль первым битом
	    I2C1->DR = (_address << 1);
        while (!(I2C1->SR1 & I2C_SR1_ADDR));
        (void) I2C1->SR1;
        (void) I2C1->SR2;

        // 4. Передаем адрес регистра
	    I2C1->DR = _register;
	    while(!(I2C1->SR1 & I2C_SR1_TXE)){};

        // 5. Останавливаем передачу
    	I2C1->CR1 |= I2C_CR1_STOP;

        // 6. Производим рестарт
	    I2C1->CR1 |= I2C_CR1_START;
	    while(!(I2C1->SR1 & I2C_SR1_SB)){};

	    // 7. Передаем адрес устройства, но теперь для чтения
	    I2C1->DR = ((_address << 1) | 0x01);
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
};

#endif  // STEPAN_HAL_I2C_H_
