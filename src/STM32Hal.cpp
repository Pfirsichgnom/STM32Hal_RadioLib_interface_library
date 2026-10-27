/**
 * @file STM32Hal.cpp
 * @brief STM32 HAL driver for RadioLib (implementation).
 *
 * Implements STM32Hal, mapping the STM32 CubeMX/HAL GPIO and SPI driver APIs
 * to the RadioLib HAL. See STM32Hal.hpp for the interface documentation.
 *
 * Created on: Oct 18, 2026
 * Author:     pfirsichgnom
 */

#include <STM32Hal.hpp>
#include <cstring>

/**
 * @brief Extracts the GPIO port instance from an encoded pin value.
 *
 * Thin wrapper around the GET_PORT macro.
 *
 * @param[in] pin  An encoded pin (see ENCODE_PIN).
 * @return         The GPIO port instance (e.g. GPIOA).
 */
static GPIO_TypeDef* get_port(uint32_t pin)
{
    return GET_PORT(pin);
}

/**
 * @brief Extracts the HAL pin from an encoded pin value.
 *
 * Thin wrapper around the GET_PIN macro.
 *
 * @param[in] pin  An encoded pin (see ENCODE_PIN).
 * @return         The HAL pin value (e.g. GPIO_PIN_1).
 */
static uint16_t get_pin(uint32_t pin)
{
    return GET_PIN(pin);
}

/**
 * @brief Returns the position of the lowest set bit in a pin mask.
 *
 * @param[in] mask  A single-bit pin mask (e.g. GPIO_PIN_1).
 * @return          The zero-based position of the lowest set bit (0 for bit 0, ...).
 * @note Intended to be called with a mask that has a single bit set.
 */
static uint32_t pin_index(uint32_t mask)
{
    uint32_t i = 0;
    while (!(mask & 0x1U))
    {
        mask >>= 1; i++;
    }
    return i;
}

STM32Hal::STM32Hal(SPI_HandleTypeDef* hspi) : RadioLibHal(0, 1, 0, 1, 0, 1), _lastSpiStatus(HAL_OK)
{
    _hspi = hspi;
}

// Sets the pin mode (input or output) for the given encoded pin.
void STM32Hal::pinMode(uint32_t pin, uint32_t mode)
{
    if (pin == RADIOLIB_NC)
    {
        return;
    }
    GPIO_TypeDef* port = get_port(pin);
    uint32_t mask = get_pin(pin);

    uint32_t pull = (port->PUPDR >> (2 * pin_index(mask))) & 0x3U;

    GPIO_InitTypeDef cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.Pin = mask;
    cfg.Pull = pull;
    if (mode == GpioModeOutput) {
        cfg.Mode = GPIO_MODE_OUTPUT_PP;
        cfg.Speed = GPIO_SPEED_FREQ_LOW;
    } else {
        cfg.Mode = GPIO_MODE_INPUT;
    }
    HAL_GPIO_Init(port, &cfg);
}

// Writes the pin status (HIGH/LOW) using the encode/decode helpers.
void STM32Hal::digitalWrite(uint32_t pin, uint32_t value)
{
    if(!(pin == RADIOLIB_NC))
    {
        HAL_GPIO_WritePin(get_port(pin), get_pin(pin), (GPIO_PinState)value);
    }
}

// Reads the pin status (HIGH/LOW) using the encode/decode helpers.
uint32_t STM32Hal::digitalRead(uint32_t pin)
{
    if(!(pin == RADIOLIB_NC))
    {
        return (uint32_t)HAL_GPIO_ReadPin(get_port(pin), get_pin(pin));
    }
    return 0;
}

// Transmits, and (if inBuf is provided) also receives, the data over SPI.
// The resulting HAL status code is stored for getLastSpiTransferStatus().
void STM32Hal::spiTransfer(uint8_t* outBuf, size_t len, uint8_t* inBuf)
{
    HAL_StatusTypeDef status = HAL_OK;
    if (inBuf != nullptr) {
        status = HAL_SPI_TransmitReceive(_hspi, outBuf, inBuf, len, STM32HAL_SPI_TRANSFER_TIMEOUT_MS);
    } else {
        status = HAL_SPI_Transmit(_hspi, outBuf, len, STM32HAL_SPI_TRANSFER_TIMEOUT_MS);
    }
    _lastSpiStatus = (uint32_t)status;
}

// Delay (ms) implemented via HAL_Delay().
void STM32Hal::delay(RadioLibTime_t ms)
{
    HAL_Delay(ms);
}
// Delay (us) implemented as a busy-wait loop; assumes each iteration takes >=4 clock cycles.
void STM32Hal::delayMicroseconds(RadioLibTime_t us)
{
    uint32_t count = us * (SystemCoreClock / 1000000U) /4;
    for (volatile uint32_t i = 0; i < count; i++)
    {
        ;
    }
}
// Milliseconds since start, mapped to the system timer (HAL_GetTick()).
RadioLibTime_t STM32Hal::millis()
{
    return HAL_GetTick();
}

// Pseudo microseconds: HAL_GetTick() only has ms resolution, so this is just ms * 1000.
RadioLibTime_t STM32Hal::micros()
{
    return HAL_GetTick() * 1000;
}

// Reads the pulse width of a pin. Although returned in us, the resolution is
// limited to ms because micros()/millis() are derived from HAL_GetTick().
long STM32Hal::pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout)
{
    if (pin == RADIOLIB_NC)
    {
        return 0;
    }
    uint32_t start = micros();
    while (digitalRead(pin) != state)
    {
        if ((micros() - start) > timeout)
        {
            return 0;
        }
    }
    uint32_t pulseStart = micros();
    while (digitalRead(pin) == state)
    {
        if ((micros() - start) > timeout)
        {
            return 0;
        }
    }
    return (micros() - pulseStart);
}

// Attaching interrupts is not implemented; this is an empty no-op.
void STM32Hal::attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode)
{
    (void)interruptNum;
    (void)interruptCb;
    (void)mode;
}

// Detaching interrupts is not implemented; this is an empty no-op.
void STM32Hal::detachInterrupt(uint32_t interruptNum) {
    (void)interruptNum;
}
