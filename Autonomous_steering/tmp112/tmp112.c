/**
 * @file tmp112.c
 * @brief Source file for TMP112 temperature sensor driver functions
 * Handles I2C communication and driver configuration.
 */
#include "tmp112.h"

void TMP_Init() {
    TMP_setConfig(TMP_CONFIG_FAULT_QUEUE_MASK(1));
    TMP_setConfig(TMP_CONFIG_POLARITY_MASK);
    TMP_setTLow(60);
    TMP_setTHigh(80);
}

int16_t TMP_getTemperature(void) {
    return TMP_data_to_celcius(TMP_read(TMP_TEMPERATURE));
}

uint8_t TMP_setTLow(float celcius) {
    return TMP_write(TMP_TLOW, TMP_data_to_celcius(celcius));
}

uint8_t TMP_setTHigh(float celcius) {
    return TMP_write(TMP_THIGH, TMP_data_to_celcius(celcius));
}

uint8_t TMP_setConfig(uint16_t data) {
    return TMP_write(TMP_CONFIG, data);
}

uint8_t TMP_write(uint8_t reg, uint16_t data) {
    uint8_t OK = 1;
    if (HAL_i2C_Mem_Write(TMP_I2C_INTERFACE, TMP_ADDR, reg, 1, &data, 1, HAL_DELAY) != HAL_OK) OK = 0;
    return OK;
}

uint16_t TMP_read(uint8_t reg) {
    uint8_t pData[2] = {0}, data = 0;
    if (HAL_I2C_Mem_Read(TMP_I2C_INTERFACE, TMP_ADDR, reg, 1, pData, 2, HAL_DELAY) != HAL_OK) return 0xFFFF;
    data = (pData[0] << 4) | (pData[1] >> 4);
    return data;
}

uint16_t TMP_celcius_to_data(float celcius) {
    return (uint16_t)(celcius / TMP_DEGREE_RESOLUTION);
}

float TMP_data_to_celcius(uint16_t data) {
    return (float)(data * TMP_DEGREE_RESOLUTION);
}
