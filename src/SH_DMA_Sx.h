#ifndef STEPAN_HAL_DMA_STREAMX_H_
#define STEPAN_HAL_DMA_STREAMX_H_

#include "../system/include/cmsis/stm32f4xx.h"

#if 1
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

void DMA_init() {
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t _dmax_irq_status[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

void DMA1_Stream0_IRQHandler(void) {

}

void DMA1_Stream1_IRQHandler(void) {

}

void DMA1_Stream2_IRQHandler(void) {

}

void DMA1_Stream3_IRQHandler(void) {

}


void DMA1_Stream4_IRQHandler(void) {

}

void DMA1_Stream5_IRQHandler(void) {

}

void DMA1_Stream6_IRQHandler(void) {

}

void DMA1_Stream7_IRQHandler(void) {

}

void DMA2_Stream0_IRQHandler(void) {

}

void DMA2_Stream1_IRQHandler(void) {

}

void DMA2_Stream2_IRQHandler(void) {

}

void DMA2_Stream3_IRQHandler(void) {

}

void DMA2_Stream4_IRQHandler(void) {

}

void DMA2_Stream5_IRQHandler(void) {

}

void DMA2_Stream6_IRQHandler(void) {

}

void DMA2_Stream7_IRQHandler(void) {

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




template<typename P, typename A>
class SH_DMA_Sx {
    DMA_Stream_TypeDef *_DMA_Sx;
    DMA_TypeDef *_DMA;
    P *_periphery;
    uint32_t _options_cr;
    uint8_t _priority;


public:
    A *_array_0;
    A *_array_1;
    uint16_t _size;

    SH_DMA_Sx(DMA_Stream_TypeDef *DMA_Sx,
              P *periphery,
              const uint32_t &options_cr,
              A *array_0,
              A *array_1,
              const uint16_t &size,
              const uint8_t &priority
              ) : _DMA_Sx(DMA_Sx), _periphery(periphery), _options_cr(options_cr),
                  _array_0(array_0), _array_1(array_1), _size(size), _priority(priority)
    {
        uint8_t status = 0x00;

        if ((_DMA_Sx == DMA1_Stream0) || (_DMA_Sx == DMA1_Stream1) || (_DMA_Sx == DMA1_Stream2) || (_DMA_Sx == DMA1_Stream3) || (_DMA_Sx == DMA1_Stream4) || (_DMA_Sx == DMA1_Stream5) || (_DMA_Sx == DMA1_Stream6) || (_DMA_Sx == DMA1_Stream7))
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

        if ((_DMA_Sx == DMA2_Stream0) || (_DMA_Sx == DMA2_Stream1) || (_DMA_Sx == DMA2_Stream2) || (_DMA_Sx == DMA2_Stream3) || (_DMA_Sx == DMA2_Stream4) || (_DMA_Sx == DMA2_Stream5) || (_DMA_Sx == DMA2_Stream6) || (_DMA_Sx == DMA2_Stream7)) {
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
            _DMA = DMA2;

        // Отключаем DMA, для чтения
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        while ((DMA1_Stream5->CR) & DMA_SxCR_EN){;}
        DMA1_Stream5->CR = ((0x01 << 25) | DMA_SxCR_MINC | DMA_SxCR_TCIE);

        DMA1_Stream5->PAR = (uint32_t)&I2C1->DR;
        DMA1_Stream5->M0AR = (uint32_t)&_i2c1_data_rx[0];

        DMA1->HIFCR |= DMA_HIFCR_CTCIF5;  // Включаем прерывание после успешной передачи передачи
        NVIC_EnableIRQ(DMA1_Stream5_IRQn);
        NVIC_SetPriority(DMA1_Stream5_IRQn, 2);



    }
};

#endif  // STEPAN_HAL_DMA_STREAMX_H_
