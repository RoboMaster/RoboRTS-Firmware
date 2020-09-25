#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "struct_typedef.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

extern void BMI088_GPIO_Init(void);
extern void BMI088_Com_Init(void);
extern void BMI088_Delay_ms(uint16_t ms);
extern void BMI088_Delay_us(uint16_t us);

#if defined(BMI088_USE_SPI)
    extern void BMI088_ACCEL_NS_L(void);
    extern void BMI088_ACCEL_NS_H(void);

    extern void BMI088_GYRO_NS_L(void);
    extern void BMI088_GYRO_NS_H(void);

    extern uint8_t BMI088_Read_Write_Byte(uint8_t reg);

#elif defined(BMI088_USE_IIC)

    extern uint8_t I2C_WriteByte(uint8_t data, uint8_t addr, uint8_t reg);
    extern uint8_t I2C_WriteBuf(uint8_t *data, uint8_t len, uint8_t addr, uint8_t reg);
    extern uint8_t I2C_ReadByte(uint8_t *buf, uint8_t len, uint8_t addr, uint8_t reg);

    #define BMI088_IIC_Write_Single_Reg(data, addr, reg) I2C_WriteByte((data), (addr), (reg))

    #define BMI088_IIC_Read_Single_Reg(data, addr, reg) I2C_ReadByte((data), 1, (addr), (reg))

    #define BMI088_IIC_Write_Muli_Reg(data, len, addr, reg) I2C_WriteBuf((data), (len), (addr), (reg))

    #define BMI088_IIC_Read_Muli_Reg(data, len, addr, reg) I2C_ReadByte((data), (len), (addr), (reg))

#endif

#endif
