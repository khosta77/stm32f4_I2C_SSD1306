#ifndef STEPAN_HAL_DMA_STREAMX_H_
#define STEPAN_HAL_DMA_STREAMX_H_

#include "../system/include/cmsis/stm32f4xx.h"

#if 0
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

#define DMAx_IRQ_STATUS_TCIF  0b00000001  // Полная передача
#define DMAx_IRQ_STATUS_HTIF  0b00000010  // Половина передачи
#define DMAx_IRQ_STATUS_TEIF  0b00000100  // Ошибка передачи
#define DMAx_IRQ_STATUS_DMEIF 0b00001000  // Ошибка direct-режима
#define DMAx_IRQ_STATUS_FEIF  0b00010000  // Ошибка FIFO
#define DMAx_IRQ_STATUS_CT    0b10000000  // Массив 1 или 0

void DMA1_Stream0_IRQHandler(void) {
    if ((DMA1->LISR & DMA_LISR_TCIF0) == DMA_LISR_TCIF0) {  // Полная передача
        _dmax_irq_status[0] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTCIF0;
    }

    if ((DMA1->LISR & DMA_LISR_HTIF0) == DMA_LISR_HTIF0) {  // Половина передачи
        _dmax_irq_status[0] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CHTIF0;
    }

    if ((DMA1->LISR & DMA_LISR_TEIF0) == DMA_LISR_TEIF0) {  // Ошибка передачи
        _dmax_irq_status[0] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTEIF0;
    }

    if ((DMA1->LISR & DMA_LISR_DMEIF0) == DMA_LISR_DMEIF0) {  // Ошибка direct-режима
        _dmax_irq_status[0] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF0;
    }

    if ((DMA1->LISR & DMA_LISR_FEIF0) == DMA_LISR_FEIF0) {  // Ошибка FIFO
        _dmax_irq_status[0] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream0->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CFEIF0;
    }
}

void DMA1_Stream1_IRQHandler(void) {
    if ((DMA1->LISR & DMA_LISR_TCIF1) == DMA_LISR_TCIF1) {  // Полная передача
        _dmax_irq_status[1] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTCIF1;
    }

    if ((DMA1->LISR & DMA_LISR_HTIF1) == DMA_LISR_HTIF1) {  // Половина передачи
        _dmax_irq_status[1] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CHTIF1;
    }

    if ((DMA1->LISR & DMA_LISR_TEIF1) == DMA_LISR_TEIF1) {  // Ошибка передачи
        _dmax_irq_status[1] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTEIF1;
    }

    if ((DMA1->LISR & DMA_LISR_DMEIF1) == DMA_LISR_DMEIF1) {  // Ошибка direct-режима
        _dmax_irq_status[1] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF0;
    }

    if ((DMA1->LISR & DMA_LISR_FEIF1) == DMA_LISR_FEIF1) {  // Ошибка FIFO
        _dmax_irq_status[1] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream1->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CFEIF1;
    }
}

void DMA1_Stream2_IRQHandler(void) {
    if ((DMA1->LISR & DMA_LISR_TCIF2) == DMA_LISR_TCIF2) {  // Полная передача
        _dmax_irq_status[2] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTCIF2;
    }

    if ((DMA1->LISR & DMA_LISR_HTIF2) == DMA_LISR_HTIF2) {  // Половина передачи
        _dmax_irq_status[2] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CHTIF2;
    }

    if ((DMA1->LISR & DMA_LISR_TEIF2) == DMA_LISR_TEIF2) {  // Ошибка передачи
        _dmax_irq_status[2] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTEIF2;
    }

    if ((DMA1->LISR & DMA_LISR_DMEIF2) == DMA_LISR_DMEIF2) {  // Ошибка direct-режима
        _dmax_irq_status[2] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF2;
    }

    if ((DMA1->LISR & DMA_LISR_FEIF2) == DMA_LISR_FEIF2) {  // Ошибка FIFO
        _dmax_irq_status[2] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream2->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CFEIF2;
    }
}

void DMA1_Stream3_IRQHandler(void) {
    if ((DMA1->LISR & DMA_LISR_TCIF3) == DMA_LISR_TCIF3) {  // Полная передача
        _dmax_irq_status[3] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTCIF3;
    }

    if ((DMA1->LISR & DMA_LISR_HTIF3) == DMA_LISR_HTIF3) {  // Половина передачи
        _dmax_irq_status[3] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CHTIF3;
    }

    if ((DMA1->LISR & DMA_LISR_TEIF3) == DMA_LISR_TEIF3) {  // Ошибка передачи
        _dmax_irq_status[3] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CTEIF3;
    }

    if ((DMA1->LISR & DMA_LISR_DMEIF3) == DMA_LISR_DMEIF3) {  // Ошибка direct-режима
        _dmax_irq_status[3] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CDMEIF3;
    }

    if ((DMA1->LISR & DMA_LISR_FEIF3) == DMA_LISR_FEIF3) {  // Ошибка FIFO
        _dmax_irq_status[3] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream3->CR &= ~DMA_SxCR_EN;
        DMA1->LIFCR |= DMA_LIFCR_CFEIF3;
    }
}


void DMA1_Stream4_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF4) == DMA_HISR_TCIF4) {  // Полная передача
        _dmax_irq_status[4] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF4;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF4) == DMA_HISR_HTIF4) {  // Половина передачи
        _dmax_irq_status[4] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA1->HISR & DMA_HISR_TEIF4) == DMA_HISR_TEIF4) {  // Ошибка передачи
        _dmax_irq_status[4] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTEIF4;
    }

    if ((DMA1->HISR & DMA_HISR_DMEIF4) == DMA_HISR_DMEIF4) {  // Ошибка direct-режима
        _dmax_irq_status[4] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CDMEIF4;
    }

    if ((DMA1->HISR & DMA_HISR_FEIF4) == DMA_HISR_FEIF4) {  // Ошибка FIFO
        _dmax_irq_status[4] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CFEIF4;
    }
}

void DMA1_Stream5_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF5) == DMA_HISR_TCIF5) {  // Полная передача
        _dmax_irq_status[5] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF5;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF5) == DMA_HISR_HTIF5) {  // Половина передачи
        _dmax_irq_status[5] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF5;
    }

    if ((DMA1->HISR & DMA_HISR_TEIF5) == DMA_HISR_TEIF5) {  // Ошибка передачи
        _dmax_irq_status[5] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTEIF5;
    }

    if ((DMA1->HISR & DMA_HISR_DMEIF5) == DMA_HISR_DMEIF5) {  // Ошибка direct-режима
        _dmax_irq_status[5] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CDMEIF5;
    }

    if ((DMA1->HISR & DMA_HISR_FEIF5) == DMA_HISR_FEIF5) {  // Ошибка FIFO
        _dmax_irq_status[5] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CFEIF5;
    }
}

void DMA1_Stream6_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {  // Полная передача
        _dmax_irq_status[6] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF6;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF6) == DMA_HISR_HTIF6) {  // Половина передачи
        _dmax_irq_status[6] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA1->HISR & DMA_HISR_TEIF6) == DMA_HISR_TEIF6) {  // Ошибка передачи
        _dmax_irq_status[6] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTEIF4;
    }

    if ((DMA1->HISR & DMA_HISR_DMEIF6) == DMA_HISR_DMEIF6) {  // Ошибка direct-режима
        _dmax_irq_status[6] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CDMEIF6;
    }

    if ((DMA1->HISR & DMA_HISR_FEIF4) == DMA_HISR_FEIF6) {  // Ошибка FIFO
        _dmax_irq_status[6] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CFEIF6;
    }
}

void DMA1_Stream7_IRQHandler(void) {
    if ((DMA1->HISR & DMA_HISR_TCIF7) == DMA_HISR_TCIF7) {  // Полная передача
        _dmax_irq_status[7] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTCIF7;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF7) == DMA_HISR_HTIF7) {  // Половина передачи
        _dmax_irq_status[7] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF7;
    }

    if ((DMA1->HISR & DMA_HISR_TEIF7) == DMA_HISR_TEIF7) {  // Ошибка передачи
        _dmax_irq_status[7] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CTEIF7;
    }

    if ((DMA1->HISR & DMA_HISR_DMEIF7) == DMA_HISR_DMEIF7) {  // Ошибка direct-режима
        _dmax_irq_status[7] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CDMEIF7;
    }

    if ((DMA1->HISR & DMA_HISR_FEIF7) == DMA_HISR_FEIF7) {  // Ошибка FIFO
        _dmax_irq_status[7] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CFEIF7;
    }
}

void DMA2_Stream0_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF0) == DMA_LISR_TCIF0) {  // Полная передача
        _dmax_irq_status[8] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF0;
    }

    if ((DMA2->LISR & DMA_LISR_HTIF0) == DMA_LISR_HTIF0) {  // Половина передачи
        _dmax_irq_status[8] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CHTIF0;
    }

    if ((DMA2->LISR & DMA_LISR_TEIF0) == DMA_LISR_TEIF0) {  // Ошибка передачи
        _dmax_irq_status[8] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTEIF0;
    }

    if ((DMA2->LISR & DMA_LISR_DMEIF0) == DMA_LISR_DMEIF0) {  // Ошибка direct-режима
        _dmax_irq_status[8] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF0;
    }

    if ((DMA2->LISR & DMA_LISR_FEIF0) == DMA_LISR_FEIF0) {  // Ошибка FIFO
        _dmax_irq_status[8] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream0->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CFEIF0;
    }
}

void DMA2_Stream1_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF1) == DMA_LISR_TCIF1) {  // Полная передача
        _dmax_irq_status[9] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;
    }

    if ((DMA2->LISR & DMA_LISR_HTIF1) == DMA_LISR_HTIF1) {  // Половина передачи
        _dmax_irq_status[9] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CHTIF1;
    }

    if ((DMA2->LISR & DMA_LISR_TEIF1) == DMA_LISR_TEIF1) {  // Ошибка передачи
        _dmax_irq_status[9] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTEIF1;
    }

    if ((DMA2->LISR & DMA_LISR_DMEIF1) == DMA_LISR_DMEIF1) {  // Ошибка direct-режима
        _dmax_irq_status[9] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF1;
    }

    if ((DMA2->LISR & DMA_LISR_FEIF1) == DMA_LISR_FEIF1) {  // Ошибка FIFO
        _dmax_irq_status[9] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream1->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CFEIF1;
    }
}

void DMA2_Stream2_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF2) == DMA_LISR_TCIF2) {  // Полная передача
        _dmax_irq_status[10] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF2;
    }

    if ((DMA2->LISR & DMA_LISR_HTIF2) == DMA_LISR_HTIF2) {  // Половина передачи
        _dmax_irq_status[10] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CHTIF2;
    }

    if ((DMA2->LISR & DMA_LISR_TEIF2) == DMA_LISR_TEIF2) {  // Ошибка передачи
        _dmax_irq_status[10] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTEIF2;
    }

    if ((DMA2->LISR & DMA_LISR_DMEIF2) == DMA_LISR_DMEIF2) {  // Ошибка direct-режима
        _dmax_irq_status[10] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF2;
    }

    if ((DMA2->LISR & DMA_LISR_FEIF2) == DMA_LISR_FEIF2) {  // Ошибка FIFO
        _dmax_irq_status[10] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream2->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CFEIF2;
    }
}

void DMA2_Stream3_IRQHandler(void) {
    if ((DMA2->LISR & DMA_LISR_TCIF3) == DMA_LISR_TCIF3) {  // Полная передача
        _dmax_irq_status[11] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF3;
    }

    if ((DMA2->LISR & DMA_LISR_HTIF3) == DMA_LISR_HTIF0) {  // Половина передачи
        _dmax_irq_status[11] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CHTIF3;
    }

    if ((DMA2->LISR & DMA_LISR_TEIF3) == DMA_LISR_TEIF3) {  // Ошибка передачи
        _dmax_irq_status[11] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CTEIF3;
    }

    if ((DMA2->LISR & DMA_LISR_DMEIF3) == DMA_LISR_DMEIF3) {  // Ошибка direct-режима
        _dmax_irq_status[11] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CDMEIF3;
    }

    if ((DMA2->LISR & DMA_LISR_FEIF3) == DMA_LISR_FEIF3) {  // Ошибка FIFO
        _dmax_irq_status[11] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream3->CR &= ~DMA_SxCR_EN;
        DMA2->LIFCR |= DMA_LIFCR_CFEIF3;
    }
}

void DMA2_Stream4_IRQHandler(void) {
    if ((DMA2->HISR & DMA_HISR_TCIF4) == DMA_HISR_TCIF4) {  // Полная передача
        _dmax_irq_status[12] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTCIF4;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF4) == DMA_HISR_HTIF4) {  // Половина передачи
        _dmax_irq_status[12] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA2->HISR & DMA_HISR_TEIF4) == DMA_HISR_TEIF4) {  // Ошибка передачи
        _dmax_irq_status[12] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTEIF4;
    }

    if ((DMA2->HISR & DMA_HISR_DMEIF4) == DMA_HISR_DMEIF4) {  // Ошибка direct-режима
        _dmax_irq_status[12] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF4;
    }

    if ((DMA2->HISR & DMA_HISR_FEIF4) == DMA_HISR_FEIF4) {  // Ошибка FIFO
        _dmax_irq_status[12] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CFEIF4;
    }
}

void DMA2_Stream5_IRQHandler(void) {
    if ((DMA2->HISR & DMA_HISR_TCIF5) == DMA_HISR_TCIF5) {  // Полная передача
        _dmax_irq_status[13] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTCIF5;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF5) == DMA_HISR_HTIF5) {  // Половина передачи
        _dmax_irq_status[13] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
    }

    if ((DMA2->HISR & DMA_HISR_TEIF5) == DMA_HISR_TEIF5) {  // Ошибка передачи
        _dmax_irq_status[13] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTEIF5;
    }

    if ((DMA2->HISR & DMA_HISR_DMEIF5) == DMA_HISR_DMEIF5) {  // Ошибка direct-режима
        _dmax_irq_status[13] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF5;
    }

    if ((DMA2->HISR & DMA_HISR_FEIF5) == DMA_HISR_FEIF5) {  // Ошибка FIFO
        _dmax_irq_status[13] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CFEIF5;
    }
}

void DMA2_Stream6_IRQHandler(void) {
    if ((DMA2->HISR & DMA_HISR_TCIF6) == DMA_HISR_TCIF6) {  // Полная передача
        _dmax_irq_status[14] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTCIF6;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF6) == DMA_HISR_HTIF6) {  // Половина передачи
        _dmax_irq_status[14] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA2->HISR & DMA_HISR_TEIF6) == DMA_HISR_TEIF6) {  // Ошибка передачи
        _dmax_irq_status[14] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTEIF4;
    }

    if ((DMA2->HISR & DMA_HISR_DMEIF6) == DMA_HISR_DMEIF6) {  // Ошибка direct-режима
        _dmax_irq_status[14] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF6;
    }

    if ((DMA2->HISR & DMA_HISR_FEIF4) == DMA_HISR_FEIF6) {  // Ошибка FIFO
        _dmax_irq_status[14] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CFEIF6;
    }
}

void DMA2_Stream7_IRQHandler(void) {
    if ((DMA2->HISR & DMA_HISR_TCIF7) == DMA_HISR_TCIF7) {  // Полная передача
        _dmax_irq_status[15] &= ~DMAx_IRQ_STATUS_TCIF;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTCIF7;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF7) == DMA_HISR_HTIF7) {  // Половина передачи
        _dmax_irq_status[15] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF7;
    }

    if ((DMA2->HISR & DMA_HISR_TEIF7) == DMA_HISR_TEIF7) {  // Ошибка передачи
        _dmax_irq_status[15] &= ~DMAx_IRQ_STATUS_TEIF;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CTEIF7;
    }

    if ((DMA2->HISR & DMA_HISR_DMEIF7) == DMA_HISR_DMEIF7) {  // Ошибка direct-режима
        _dmax_irq_status[15] &= ~DMAx_IRQ_STATUS_DMEIF;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CDMEIF7;
    }

    if ((DMA2->HISR & DMA_HISR_FEIF7) == DMA_HISR_FEIF7) {  // Ошибка FIFO
        _dmax_irq_status[15] &= ~DMAx_IRQ_STATUS_FEIF;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CFEIF7;
    }
}

/* P может быть: I2C_TypeDef, SPI_TypeDef, USART_TypeDef
 * A может быть: uint8_t, uint16_t, uint32_t
 * */
template<typename P, typename A>
class SH_DMA_Sx {
    DMA_Stream_TypeDef *_DMA_Sx;
    DMA_TypeDef *_DMA;
    P *_periphery;
    uint32_t _options_cr;
    uint8_t _dma_stream = 0x00;

    void _dmax_init() {
        if ((_DMA_Sx == DMA1_Stream0) || (_DMA_Sx == DMA1_Stream1) || (_DMA_Sx == DMA1_Stream2) ||
            (_DMA_Sx == DMA1_Stream3) || (_DMA_Sx == DMA1_Stream4) || (_DMA_Sx == DMA1_Stream5) ||
            (_DMA_Sx == DMA1_Stream6) || (_DMA_Sx == DMA1_Stream7)) {
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
            _DMA = DMA1;
        }

        if ((_DMA_Sx == DMA2_Stream0) || (_DMA_Sx == DMA2_Stream1) || (_DMA_Sx == DMA2_Stream2) ||
            (_DMA_Sx == DMA2_Stream3) || (_DMA_Sx == DMA2_Stream4) || (_DMA_Sx == DMA2_Stream5) ||
            (_DMA_Sx == DMA2_Stream6) || (_DMA_Sx == DMA2_Stream7)) {
            RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;
            _DMA = DMA2;
        }
    }

    void _iqr_dma_sx_init(const uint8_t &irq_priority) {
        //// DMA1
        if (_DMA_Sx == DMA1_Stream0) {
            _dma_stream = 0;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);
            NVIC_EnableIRQ(DMA1_Stream0_IRQn);
            NVIC_SetPriority(DMA1_Stream0_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA1_Stream1) {
            _dma_stream = 1;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1);
            NVIC_EnableIRQ(DMA1_Stream1_IRQn);
            NVIC_SetPriority(DMA1_Stream1_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA1_Stream2) {
            _dma_stream = 2;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2);
            NVIC_EnableIRQ(DMA1_Stream2_IRQn);
            NVIC_SetPriority(DMA1_Stream2_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA1_Stream3) {
            _dma_stream = 3;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
            NVIC_EnableIRQ(DMA1_Stream3_IRQn);
            NVIC_SetPriority(DMA1_Stream3_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA1_Stream4) {
            _dma_stream = 4;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4);
            NVIC_EnableIRQ(DMA1_Stream4_IRQn);
            NVIC_SetPriority(DMA1_Stream4_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA1_Stream5) {
            _dma_stream = 5;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
            NVIC_EnableIRQ(DMA1_Stream5_IRQn);
            NVIC_SetPriority(DMA1_Stream5_IRQn, irq_priority);
        }

        if (_DMA_Sx == DMA1_Stream6) {
            _dma_stream = 6;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6);
            NVIC_EnableIRQ(DMA1_Stream6_IRQn);
            NVIC_SetPriority(DMA1_Stream6_IRQn, irq_priority);
        }

        if (_DMA_Sx == DMA1_Stream7) {
            _dma_stream = 7;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7);
            NVIC_EnableIRQ(DMA1_Stream7_IRQn);
            NVIC_SetPriority(DMA1_Stream7_IRQn, irq_priority);
        }

        //// DMA2
        if (_DMA_Sx == DMA2_Stream0) {
            _dma_stream = 8;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF0 | DMA_LIFCR_CHTIF0 | DMA_LIFCR_CTEIF0 | DMA_LIFCR_CDMEIF0 | DMA_LIFCR_CFEIF0);
            NVIC_EnableIRQ(DMA2_Stream0_IRQn);
            NVIC_SetPriority(DMA2_Stream0_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA2_Stream1) {
            _dma_stream = 9;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1);
            NVIC_EnableIRQ(DMA2_Stream1_IRQn);
            NVIC_SetPriority(DMA2_Stream1_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA2_Stream2) {
            _dma_stream = 10;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF2 | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTEIF2 | DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CFEIF2);
            NVIC_EnableIRQ(DMA2_Stream2_IRQn);
            NVIC_SetPriority(DMA2_Stream2_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA2_Stream3) {
            _dma_stream = 11;
            _DMA->LIFCR |= (DMA_LIFCR_CTCIF3 | DMA_LIFCR_CHTIF3 | DMA_LIFCR_CTEIF3 | DMA_LIFCR_CDMEIF3 | DMA_LIFCR_CFEIF3);
            NVIC_EnableIRQ(DMA2_Stream3_IRQn);
            NVIC_SetPriority(DMA2_Stream3_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA2_Stream4) {
            _dma_stream = 12;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF4 | DMA_HIFCR_CHTIF4 | DMA_HIFCR_CTEIF4 | DMA_HIFCR_CDMEIF4 | DMA_HIFCR_CFEIF4);
            NVIC_EnableIRQ(DMA2_Stream4_IRQn);
            NVIC_SetPriority(DMA2_Stream4_IRQn, irq_priority);
        }
        
        if (_DMA_Sx == DMA2_Stream5) {
            _dma_stream = 13;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF5 | DMA_HIFCR_CHTIF5 | DMA_HIFCR_CTEIF5 | DMA_HIFCR_CDMEIF5 | DMA_HIFCR_CFEIF5);
            NVIC_EnableIRQ(DMA2_Stream5_IRQn);
            NVIC_SetPriority(DMA2_Stream5_IRQn, irq_priority);
        }

        if (_DMA_Sx == DMA2_Stream6) {
            _dma_stream = 14;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CFEIF6);
            NVIC_EnableIRQ(DMA2_Stream6_IRQn);
            NVIC_SetPriority(DMA2_Stream6_IRQn, irq_priority);
        }

        if (_DMA_Sx == DMA2_Stream7) {
            _dma_stream = 15;
            _DMA->HIFCR |= (DMA_HIFCR_CTCIF7 | DMA_HIFCR_CHTIF7 | DMA_HIFCR_CTEIF7 | DMA_HIFCR_CDMEIF7 | DMA_HIFCR_CFEIF7);
            NVIC_EnableIRQ(DMA2_Stream7_IRQn);
            NVIC_SetPriority(DMA2_Stream7_IRQn, irq_priority);
        }
    }
public:
    A *_array_0;
    A *_array_1;

    SH_DMA_Sx(DMA_Stream_TypeDef *DMA_Sx,
              P *periphery,
              const uint32_t &options_cr,
              A *array_0,
              A *array_1,
              const uint8_t &irq_priority
              ) : _DMA_Sx(DMA_Sx), _periphery(periphery), _options_cr(options_cr),
                  _array_0(array_0), _array_1(array_1)
    {
        uint8_t status = 0x00;

        // Включаем тактирование, назначаем внутреннеию переменную _DMA
        _dmax_init();

        // Отключаем DMA, для настройки
        _DMA_Sx->CR &= ~DMA_SxCR_EN;
        while ((_DMA_Sx->CR) & DMA_SxCR_EN){;}

        // Настройка DMA по документаци
        _DMA_Sx->CR = _options_cr;

        // Задаем потоки передачи/приема
        _DMA_Sx->PAR = (uint32_t)&_periphery->DR;
        _DMA_Sx->M0AR = (uint32_t)&_array_0;
        _DMA_Sx->M1AR = (uint32_t)&_array_1;
        //_DMA_Sx->NDTR = _size;

        // Инициализируем прирывания
        _iqr_dma_sx_init(irq_priority);
    }

private:
    void fuller_array(A *_array, const A *pBuffer, const uint16_t &_size) {
        for (uint32_t i = 0; i < _size; ++i) {
            _array[i] = pBuffer[i];
        }
    }

public:
    void dma_transmit(const A *pBuffer, const uint16_t &_size) {
        if(pBuffer != nullptr) {
            // Ждем/проверяем пока передача не закончится
            while ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_TCIF) == DMAx_IRQ_STATUS_TCIF) {;}

            // Отключаем DMA
            _DMA_Sx->CR &= ~DMA_SxCR_EN;
            while ((_DMA_Sx->CR) & DMA_SxCR_EN){;}

            // Выбираем и заполняем массив
            if ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_CT) == DMAx_IRQ_STATUS_CT) {
                _DMA_Sx->CR &= ~DMA_SxCR_CT;
                _dmax_irq_status[_dma_stream] &= ~DMAx_IRQ_STATUS_CT;
                fuller_array(_array_0, pBuffer, _size);
            } else {
                _DMA_Sx->CR |= DMA_SxCR_CT;
                _dmax_irq_status[_dma_stream] |= DMAx_IRQ_STATUS_CT;
                fuller_array(_array_1, pBuffer, _size);
            }

            // Выставляем маркер начала передачи.
            _dmax_irq_status[_dma_stream] |= DMAx_IRQ_STATUS_TCIF;
            _DMA_Sx->NDTR = _size;
            _DMA_Sx->CR |= DMA_SxCR_MINC;

            // Запускаем передачу
            _DMA_Sx->CR |= DMA_SxCR_EN;
        } else {
            /* Нулевой массив */
        }
    }

    void dma_receive(const uint16_t &_size) {
        // Ждем/проверяем пока передача не закончится
        while ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_TCIF) == DMAx_IRQ_STATUS_TCIF) {;}

        // Отключаем DMA
        _DMA_Sx->CR &= ~DMA_SxCR_EN;
        while ((_DMA_Sx->CR) & DMA_SxCR_EN){;}

        // Выбираем и заполняем массив
        if ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_CT) == DMAx_IRQ_STATUS_CT) {
            _DMA_Sx->CR &= ~DMA_SxCR_CT;
            _dmax_irq_status[_dma_stream] &= ~DMAx_IRQ_STATUS_CT;
        } else {
            _DMA_Sx->CR |= DMA_SxCR_CT;
            _dmax_irq_status[_dma_stream] |= DMAx_IRQ_STATUS_CT;
        }

        // Выставляем маркер начала передачи.
        _dmax_irq_status[_dma_stream] |= DMAx_IRQ_STATUS_TCIF;
        _DMA_Sx->NDTR = _size;
        //_DMA_Sx->CR |= DMA_SxCR_MINC;  // Не уверен надо ли это при приеме

        // Запускаем прием
        _DMA_Sx->CR |= DMA_SxCR_EN;
    }

    A *dma_receive_read() {
        // Ждем/проверяем пока передача не закончится
        while ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_TCIF) == DMAx_IRQ_STATUS_TCIF) {;}

        // Отключаем DMA
        _DMA_Sx->CR &= ~DMA_SxCR_EN;
        while ((_DMA_Sx->CR) & DMA_SxCR_EN){;}

        // Возвращаем заполненный массив
        if ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_CT) == DMAx_IRQ_STATUS_CT)
            return _array_1;
        else
            return _array_0;

        // Если происходит что-то изряда вон выходящее, возвращаем ноль.
        return nullptr;
    }

    inline const uint8_t _is_work() {
        return ((_dmax_irq_status[_dma_stream] & DMAx_IRQ_STATUS_TCIF) == DMAx_IRQ_STATUS_TCIF);
    }
};

#endif  // STEPAN_HAL_DMA_STREAMX_H_
