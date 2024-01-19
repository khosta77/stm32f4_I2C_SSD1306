#ifndef STEPAN_HAL_GPIO_H_
#define STEPAN_HAL_GPIO_H_

#include "../system/include/cmsis/stm32f4xx.h"

/*
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    GPIOB->MODER |= (GPIO_MODER_MODER6_1 | GPIO_MODER_MODER9_1);
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_6);
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPDR9_0 | GPIO_PUPDR_PUPDR6_0);
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL6_2;
    GPIOB->AFR[1] |= GPIO_AFRH_AFSEL9_2;
*/

class SH_GPIO {
    GPIO_TypeDef *_GPIOx;
    const uint8_t _pin_n;
    const uint8_t _options;
    const uint8_t _alternativ;

public:
    SH_GPIO(GPIO_TypeDef *GPIOx,        // Задать номер GPIOx
            const uint8_t &pin_number,        // Номер вывода P0-15
            const uint8_t &pin_options,       // Настройка: 
                                              // 0b   xx  |    x   | xx      |   xx  |   x
                                              //    MODER | OTYPER | OSPEEDR | PUPDR | ALTER
                                              //              По документации       | 0 - ничего; 1 - альтернативная фун.
            const uint8_t &alternativ = 0x00  // Альтернативная функция
            ) : _GPIOx(GPIOx), _pin_n(pin_number), _options(pin_options), _alternativ(alternativ) {
        // Включаем тактирование
        if (_GPIOx == GPIOA)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        if (_GPIOx == GPIOB)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        if (_GPIOx == GPIOC)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
        if (_GPIOx == GPIOD)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
        if (_GPIOx == GPIOE)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
        if (_GPIOx == GPIOF)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
        if (_GPIOx == GPIOG)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
        if (_GPIOx == GPIOH)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOHEN;
        if (_GPIOx == GPIOI)
            RCC->AHB1ENR |= RCC_AHB1ENR_GPIOIEN;

        // Настройка MODER
        _GPIOx->MODER |= (((_options & 0xC0) >> 6) << (2 * _pin_n));

        // Настройка OTYPER
        _GPIOx->OTYPER |= (((_options & 0x20) >> 5) << _pin_n);

        // Настройка OSPEED
        _GPIOx->OSPEEDR |= (((_options & 0x18) >> 3) << (2 * _pin_n));
        
        // Настройка PUPDR
        _GPIOx->PUPDR |= (((_options & 0x06) >> 1) << (2 * _pin_n));

        // Настройка Альтернативной функции
        if ((_options & 0x01) == 0x01)
            if (_pin_n <= 7)
                _GPIOx->AFR[0] |= ((_alternativ & 0x0F) << (4 * _pin_n));
            else
                _GPIOx->AFR[1] |= ((_alternativ & 0x0F) << (4 * (_pin_n - 8)));  // Тут вычитаем 8, так как расчет с нуля.
    }

    void _ON() {
        _GPIOx->ODR |= (0x01 << _pin_n);
    }

    void _OFF() {
        _GPIOx->ODR &= ~(0x01 << _pin_n);
    }

    void _XOR() {
        _GPIOx->ODR ^= (0x01 << _pin_n);
    }

    uint16_t _READ() {
        return ((_GPIOx->IDR & (0x01 << _pin_n)) == (0x01 << _pin_n));
    }
};

#endif  // STEPAN_HAL_GPIO_H_
