#ifndef STEPAN_HAL_I2C_H_
#define STEPAN_HAL_I2C_H_

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

class SH_I2C {
    I2C_TypeDef *_I2Cx;
    uint8_t _options;
    uint8_t _slave_freq;
    
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
    
        I2C1->CR2 |= (_slave_freq & 0x3F);

    }

};

#endif  // STEPAN_HAL_I2C_H_
