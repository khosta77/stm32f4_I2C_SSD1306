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

#define DMAx_IRQ_STATUS_TCIF  0b00000001  // Полная передача
#define DMAx_IRQ_STATUS_HTIF  0b00000010  // Половина передачи
#define DMAx_IRQ_STATUS_TEIF  0b00000100  // Ошибка передачи
#define DMAx_IRQ_STATUS_DMEIF 0b00001000  // Ошибка direct-режима
#define DMAx_IRQ_STATUS_FEIF  0b00010000  // Ошибка FIFO
#define DMAx_IRQ_STATUS_CT    0b10000000

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
        DMA1->HIFCR |= DMA_LIFCR_CTCIF4;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF4) == DMA_HISR_HTIF4) {  // Половина передачи
        _dmax_irq_status[4] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream4->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA1->HISR & DMA_LISR_TEIF4) == DMA_HISR_TEIF4) {  // Ошибка передачи
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
        DMA1->HIFCR |= DMA_LIFCR_CTCIF5;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF5) == DMA_HISR_HTIF5) {  // Половина передачи
        _dmax_irq_status[5] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream5->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF5;
    }

    if ((DMA1->HISR & DMA_LISR_TEIF5) == DMA_HISR_TEIF5) {  // Ошибка передачи
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
        DMA1->HIFCR |= DMA_LIFCR_CTCIF6;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF6) == DMA_HISR_HTIF6) {  // Половина передачи
        _dmax_irq_status[6] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream6->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA1->HISR & DMA_LISR_TEIF6) == DMA_HISR_TEIF6) {  // Ошибка передачи
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
        DMA1->HIFCR |= DMA_LIFCR_CTCIF7;
    }

    if ((DMA1->HISR & DMA_HISR_HTIF7) == DMA_HISR_HTIF7) {  // Половина передачи
        _dmax_irq_status[7] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA1_Stream7->CR &= ~DMA_SxCR_EN;
        DMA1->HIFCR |= DMA_HIFCR_CHTIF7;
    }

    if ((DMA1->HISR & DMA_LISR_TEIF7) == DMA_HISR_TEIF7) {  // Ошибка передачи
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
        DMA2->HIFCR |= DMA_LIFCR_CTCIF4;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF4) == DMA_HISR_HTIF4) {  // Половина передачи
        _dmax_irq_status[12] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream4->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA2->HISR & DMA_LISR_TEIF4) == DMA_HISR_TEIF4) {  // Ошибка передачи
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
        DMA2->HIFCR |= DMA_LIFCR_CTCIF5;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF5) == DMA_HISR_HTIF5) {  // Половина передачи
        _dmax_irq_status[13] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream5->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF5;
    }

    if ((DMA2->HISR & DMA_LISR_TEIF5) == DMA_HISR_TEIF5) {  // Ошибка передачи
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
        DMA2->HIFCR |= DMA_LIFCR_CTCIF6;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF6) == DMA_HISR_HTIF6) {  // Половина передачи
        _dmax_irq_status[14] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream6->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF4;
    }

    if ((DMA2->HISR & DMA_LISR_TEIF6) == DMA_HISR_TEIF6) {  // Ошибка передачи
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
        DMA2->HIFCR |= DMA_LIFCR_CTCIF7;
    }

    if ((DMA2->HISR & DMA_HISR_HTIF7) == DMA_HISR_HTIF7) {  // Половина передачи
        _dmax_irq_status[15] &= ~DMAx_IRQ_STATUS_HTIF;
        DMA2_Stream7->CR &= ~DMA_SxCR_EN;
        DMA2->HIFCR |= DMA_HIFCR_CHTIF7;
    }

    if ((DMA2->HISR & DMA_LISR_TEIF7) == DMA_HISR_TEIF7) {  // Ошибка передачи
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
