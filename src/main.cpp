#include "../system/include/cmsis/stm32f4xx.h"

#include "SH_GPIO.h"
#include "SH_DMA_Sx.h"
#include "SH_I2C.h"

#define BME280_ADDRESS 0b01110110

#define BME280_ID 0xD0

SH_GPIO PD12(GPIOD, 12, 0x40);

SH_GPIO PB6_I2C1_SCL(GPIOB, 6, 0xA3, 0x04);
SH_GPIO PB9_I2C1_SDA(GPIOB, 9, 0xA3, 0x04);

SH_I2C I2C1_FS(I2C1, 0x81, 50, nullptr, nullptr);

void I2C1_init() {
    I2C1_FS._ClockInit();
    I2C1_FS._enable();
}

void GPIOD_init() {
    PD12._ON();
}

int main(void) {
    I2C1_init();
    GPIOD_init();
    uint8_t id = 0x00;
    while (1) {
        id = I2C1_FS._read_reg(BME280_ADDRESS, BME280_ID);
    }
}
