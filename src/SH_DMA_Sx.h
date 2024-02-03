#ifndef STEPAN_HAL_DMA_STREAMX_H_
#define STEPAN_HAL_DMA_STREAMX_H_

#include "../system/include/cmsis/stm32f4xx.h"

template<typename P, typename A>
class SH_DMA_Sx {
    DMA_Stream_TypeDef *_DMA_Sx;
    P *_periphery;
    uint32_t _options_cr;

public:
    A *_array_0;
    A *_array_1;

    SH_DMA_Sx(DMA_Stream_TypeDef *DMA_Sx, P *periphery, const uint32_t &options_cr, A *array_0, A *array_1) : {
        // TODO:

    }
};

#endif  // STEPAN_HAL_DMA_STREAMX_H_
