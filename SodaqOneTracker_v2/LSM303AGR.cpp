#include <Arduino.h>
#include <math.h>
#include "LSM303AGR.h"

#define _BV(bit) (1 << (bit))

double mapDouble(double x, double in_min, double in_max, double out_min, double out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

LSM303AGR::LSM303AGR(TwoWire& wire, uint8_t accelAddress, uint8_t magAddress) :
    _wire(wire),
    _accelAddress(accelAddress),
    _magAddress(magAddress),
    _accelScale(Scale2g)
{

}

int8_t LSM303AGR::getTemperatureDelta()
{
    setAccelRegisterBits(CTRL_REG4_A, _BV(BDU));

    readAccelRegister(OUT_TEMP_L_A); // low byte can be thrown away, but must be read!
    int8_t value = readAccelRegister(OUT_TEMP_H_A);

    unsetAccelRegisterBits(CTRL_REG4_A, _BV(BDU));

    return value;
}

void LSM303AGR::setAccelScale(Scale scale)
{
    writeAccelRegister(CTRL_REG4_A, (scale << FS0));
    _accelScale = scale;
}

double LSM303AGR::getGsFromScaledValue(int16_t value)
{
    int8_t scaleMax = getAccelScaleMax(_accelScale);
    return mapDouble(value, INT16_MIN, INT16_MAX, -scaleMax, scaleMax);
}

int16_t LSM303AGR::getScaledValueFromGs(double gValue)
{
    int8_t scaleMax = getAccelScaleMax(_accelScale);
    return trunc(mapDouble(gValue, -scaleMax, scaleMax, INT16_MIN, INT16_MAX));
}

int8_t LSM303AGR::getAccelScaleMax(Scale scale)
{
    return (1 << (scale + 1));
}

void LSM303AGR::enableAccelerometer(AccelerometerMode mode, AccelerometerODR odr, Axes axes, Scale scale, bool isTemperatureOn)
{
    // set odr, mode, enabled axes
    // Note: the values of AccelerometerMode are 0b(LPen,HR)
    writeAccelRegister(CTRL_REG1_A, (odr << ODR0) | ((mode & 0b10) << LPen) | axes);
    if (mode & 0b01) {
        setAccelRegisterBits(CTRL_REG4_A, _BV(HR));
    }
    else {
        unsetAccelRegisterBits(CTRL_REG4_A, _BV(HR));
    }

    setAccelScale(scale);

    if (isTemperatureOn) {
        // enable aux ADC and temperature sensor
        writeAccelRegister(TEMP_CFG_REG_A, _BV(TEMP_EN1) | _BV(TEMP_EN0));
    }
    else {
        // disable aux ADC and temperature sensor
        writeAccelRegister(TEMP_CFG_REG_A, 0);
    }
}

void LSM303AGR::disableAccelerometer()
{
    enableAccelerometer(LowPowerMode, PowerDown, NoAxis, _accelScale, false);
}

void LSM303AGR::rebootAccelerometer()
{
    writeAccelRegister(CTRL_REG5_A, _BV(BOOT));
}

void LSM303AGR::disableMagnetometer()
{
    writeMagRegister(CFG_REG_A_M, 0b00010011); // idle
}

void LSM303AGR::setRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value |= byteValue;
    writeRegister(deviceAddress, reg, value);
}

void LSM303AGR::unsetRegisterBits(uint8_t deviceAddress, Register reg, uint8_t byteValue)
{
    uint8_t value = readRegister(deviceAddress, reg);
    value &= ~byteValue;
    writeRegister(deviceAddress, reg, value);
}

void LSM303AGR::enableInterrupt1(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(INT1_CFG_A, interruptMode | (axesEvents & 0b00111111));
    writeAccelRegister(INT1_THS_A, trunc(mapDouble(threshold, 0, getAccelScaleMax(_accelScale), 0, INT8_MAX)));
    writeAccelRegister(INT1_DURATION_A, duration); // time duration is INT1_DURATION_A/ODR

    // disable latching
    unsetAccelRegisterBits(CTRL_REG5_A, _BV(LIR_IG1));

    // enable interrupt generator 1 on INT1
    setAccelRegisterBits(CTRL_REG3_A, _BV(INT1_AOI1));
}

void LSM303AGR::disableInterrupt1()
{
    // disable interrupt generator 1
    unsetAccelRegisterBits(CTRL_REG3_A, _BV(INT1_AOI1));
}

void LSM303AGR::enableInterrupt2(uint8_t axesEvents, double threshold, uint8_t duration, InterruptMode interruptMode)
{
    // setup the interrupt
    writeAccelRegister(INT2_CFG_A, interruptMode | (axesEvents & 0b00111111));
    writeAccelRegister(INT2_THS_A, trunc(mapDouble(threshold, 0, getAccelScaleMax(_accelScale), 0, INT8_MAX)));
    writeAccelRegister(INT2_DURATION_A, duration);  // time duration is INT2_DURATION_A/ODR

    // disable latching
    unsetAccelRegisterBits(CTRL_REG5_A, _BV(LIR_IG2));

    // enable interrupt generator 2 on INT2
    setAccelRegisterBits(CTRL_REG6_A, _BV(I2_INT2));
}

void LSM303AGR::disableInterrupt2()
{
    // enable interrupt generator 2 on INT2
    unsetAccelRegisterBits(CTRL_REG6_A, _BV(I2_INT2));
}

uint8_t LSM303AGR::readRegister(uint8_t deviceAddress, uint8_t reg)
{
    _wire.beginTransmission(deviceAddress);
    _wire.write((uint8_t)reg);
    _wire.endTransmission();

    _wire.requestFrom(deviceAddress, 1);

    return _wire.read();
}

uint16_t LSM303AGR::readRegister16Bits(uint8_t deviceAddress, uint8_t reg)
{
    // TODO replace with request of 2 bytes?
    uint16_t result = readRegister(deviceAddress, reg);
    result |= readRegister(deviceAddress, reg + 1) << 8;

    return result;
}

void LSM303AGR::writeRegister(uint8_t deviceAddress, uint8_t reg, uint8_t value)
{
    _wire.beginTransmission(deviceAddress);

    _wire.write((uint8_t)reg);
    _wire.write((uint8_t)value);

    _wire.endTransmission();
}
