/**
 * @file STM32Hal.hpp
 * @brief STM32 HAL driver and helpers for the RadioLib HAL interface.
 *
 * @details
 * Declares the STM32Hal class, a thin adapter that exposes the STM32
 * CubeMX/HAL GPIO and SPI driver APIs to RadioLib, along with inline
 * helpers and macros that pack and unpack a GPIO port and pin into a
 * single 32-bit value.
 *
 * Created on: Oct 18, 2026
 * Author:     pfirsichgnom
 */

#include "Hal.h"
#include "TypeDef.h"
#include "main.h"

#ifndef stm32hal
#define stm32hal 1


/**
 * @brief Maximum duration (in milliseconds) of a single SPI transfer.
 *
 * A transfer taking longer than this value is treated as an error.
 */
#define STM32HAL_SPI_TRANSFER_TIMEOUT_MS 50U

/*
 * Example - typical usage (pin definitions and module setup):
 *
 *   #define LORA_NSS1    ENCODE_PIN(NSS1_GPIO_Port, NSS1_Pin) //define other pins too
 *   STM32Hal hal(&hspi1);
 *   SX1280 radio1 = new Module(&hal, LORA_NSS1, LORA_DIO1, LORA_RST1, LORA_BUSY1);
 */

/**
 * @brief Encodes a GPIO port instance into a compact numeric index.
 *
 * @param[in] port  The GPIO port instance to encode (e.g. GPIOA, GPIOB, ...).
 * @return          The port index (0 for GPIOA, 1 for GPIOB, ...) or
 *                  UINT32_MAX if the port is not recognized.
 * @details
 * The index occupies the high 16 bits of the value produced by ENCODE_PIN.
 */
static inline uint32_t encode_gpio_port(GPIO_TypeDef* port) {
    if (port == GPIOA) return 0;
#ifdef GPIOB
    if (port == GPIOB) return 1;
#endif
#ifdef GPIOC
    if (port == GPIOC) return 2;
#endif
#ifdef GPIOD
    if (port == GPIOD) return 3;
#endif
#ifdef GPIOE
    if (port == GPIOE) return 4;
#endif
#ifdef GPIOF
    if (port == GPIOF) return 5;
#endif
#ifdef GPIOG
    if (port == GPIOG) return 6;
#endif
#ifdef GPIOH
    if (port == GPIOH) return 7;
#endif
    return UINT32_MAX;
}

/**
 * @brief Decodes a numeric port index into the corresponding GPIO port instance.
 *
 * @param[in] index  The port index as produced by encode_gpio_port.
 * @return           The matching GPIO port instance (e.g. GPIOA, GPIOB, ...),
 *                   or NULL if the index is out of range.
 */
static inline GPIO_TypeDef* decode_gpio_port(uint32_t index) {
    switch (index) {
        case 0: return GPIOA;
#ifdef GPIOB
        case 1: return GPIOB;
#endif
#ifdef GPIOC
        case 2: return GPIOC;
#endif
#ifdef GPIOD
        case 3: return GPIOD;
#endif
#ifdef GPIOE
        case 4: return GPIOE;
#endif
#ifdef GPIOF
        case 5: return GPIOF;
#endif
#ifdef GPIOG
        case 6: return GPIOG;
#endif
#ifdef GPIOH
        case 7: return GPIOH;
#endif
        default: return 0;
    }
}

/**
 * @brief Packs a GPIO port instance and a HAL pin into a single 32-bit value.
 *
 * @details The port index occupies the high 16 bits and the pin the low 16
 * bits. The resulting value is the `pin` format accepted by the STM32Hal
 * pin functions.
 *
 * @param[in] port  The GPIO port instance (e.g. GPIOA).
 * @param[in] pin   The HAL pin value (e.g. GPIO_PIN_1).
 */
#define ENCODE_PIN(port, pin) ((encode_gpio_port(port) << 16) | (uint32_t)(pin))

/**
 * @brief Extracts the GPIO port instance from a value produced by ENCODE_PIN.
 *
 * @param[in] encoded  A 32-bit value produced by ENCODE_PIN.
 */
#define GET_PORT(encoded) decode_gpio_port((encoded) >> 16)

/**
 * @brief Extracts the HAL pin from a value produced by ENCODE_PIN.
 *
 * @param[in] encoded  A 32-bit value produced by ENCODE_PIN.
 */
#define GET_PIN(encoded) ((uint16_t)((encoded) & 0xFFFF))


/**
 * @brief STM32 implementation of the RadioLib HAL (Hal.h) interface.
 *
 * @details
 * A thin adapter that exposes the STM32 CubeMX/HAL GPIO and SPI driver APIs
 * to RadioLib. A single SPI handle is passed to the constructor and reused
 * for all SPI transfers. GPIO pins are addressed through a 32-bit value
 * produced with ENCODE_PIN; RADIOLIB_NC may instead be passed in place of a
 * pin to indicate "not connected", in which case no physical GPIO is used.
 *
 * Attaching/detaching interrupts is not supported, and the SPI begin/end and
 * transaction hooks are intentionally empty.
 */
class STM32Hal : public RadioLibHal
{
public:
    /**
     * @brief Constructs the driver bound to an existing SPI handle.
     *
     * @param[in] hspi  A pointer to an initialized SPI handle, used for all
     *                  SPI transfers performed by this driver.
     */
    STM32Hal(SPI_HandleTypeDef* hspi);

    /**
     * @brief Configures the mode (output or input) of a pin.
     *
     * @param[in] pin   An encoded pin (see ENCODE_PIN), or RADIOLIB_NC to skip.
     * @param[in] mode  The requested RadioLib pin mode. GpioModeOutput configures
     *                  the pin as a push-pull output; any other value configures
     *                  it as an input.
     */
    void pinMode(uint32_t pin, uint32_t mode) override;
    /**
     * @brief Drives a pin HIGH or LOW.
     *
     * A RADIOLIB_NC pin is ignored.
     *
     * @param[in] pin    An encoded pin (see ENCODE_PIN), or RADIOLIB_NC to skip.
     * @param[in] value  The level to drive (HIGH/LOW).
     */
    void digitalWrite(uint32_t pin, uint32_t value) override;
    /**
     * @brief Reads the current level of a pin.
     *
     * @param[in] pin  An encoded pin (see ENCODE_PIN), or RADIOLIB_NC.
     * @return         The pin level (1 for HIGH, 0 for LOW / RADIOLIB_NC).
     */
    uint32_t digitalRead(uint32_t pin) override;
    /**
     * @brief Measures the width of a pulse on a pin.
     *
     * Waits (up to @p timeout) until the pin reaches @p state, then measures
     * the pulse until the pin leaves that state.
     *
     * @param[in] pin      An encoded pin (see ENCODE_PIN), or RADIOLIB_NC.
     * @param[in] state    The level to measure (HIGH/LOW).
     * @param[in] timeout  Maximum time to wait, in microseconds.
     * @return             The pulse width in microseconds, or 0 on timeout / error.
     * @note The resolution is limited to milliseconds because timing is derived
     *       from millis() / micros(), which are based on HAL_GetTick().
     */
    long pulseIn(uint32_t pin, uint32_t state, RadioLibTime_t timeout) override;

    /**
     * @brief Blocks execution for the given number of milliseconds.
     *
     * @param[in] ms  Delay length in milliseconds.
     */
    void delay(RadioLibTime_t ms) override;
    /**
     * @brief Blocks execution for the given number of microseconds.
     *
     * @details Implemented as a tight busy-wait loop; the duration is a
     * best-effort estimate that assumes each loop iteration takes about 4
     * clock cycles.
     *
     * @param[in] us  Delay length in microseconds.
     */
    void delayMicroseconds(RadioLibTime_t us) override;
    /**
     * @brief Returns the number of milliseconds since start.
     *
     * @return Elapsed time in milliseconds (from HAL_GetTick()).
     */
    RadioLibTime_t millis() override;
    /**
     * @brief Returns the number of microseconds since start.
     *
     * @details Pseudo-implementation: returns HAL_GetTick() scaled by 1000, so
     * the effective resolution is only millisecond-based.
     *
     * @return Elapsed time in microseconds (millisecond resolution).
     */
    RadioLibTime_t micros() override;

    // SPI begin/end and transaction hooks are not required by this driver and
    // are intentionally left as empty no-ops.
    void spiBegin() {};
    void spiBeginTransaction() {};
    void spiEndTransaction() {};
    void spiEnd() {};

    /**
     * @brief Transmits data over SPI, optionally receiving simultaneously.
     *
     * @param[in,out] outBuf  Buffer with the data to transmit.
     * @param[in]     len     Number of bytes to transfer.
     * @param[in,out] inBuf   Buffer to receive data into, or NULL to transmit
     *                        only (no receive).
     */
    void spiTransfer(uint8_t* outBuf, size_t len, uint8_t* inBuf) override;

    /**
     * @brief Attaches an interrupt (not supported by this driver).
     *
     * This override is an empty no-op; attaching interrupts is not implemented.
     *
     * @param[in] interruptNum  Interrupt number (ignored).
     * @param[in] interruptCb  Interrupt callback (ignored).
     * @param[in] mode         Interrupt mode (ignored).
     */
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override;
    /**
     * @brief Detaches an interrupt (not supported by this driver).
     *
     * This override is an empty no-op; detaching interrupts is not implemented.
     *
     * @param[in] interruptNum  Interrupt number (ignored).
     */
    void detachInterrupt(uint32_t interruptNum) override;

    /**
     * @brief Returns the status code of the most recent SPI transfer.
     *
     * @return The HAL_StatusTypeDef recorded by the last spiTransfer() call.
     */
    HAL_StatusTypeDef getLastSpiTransferStatus() const {
        return (HAL_StatusTypeDef)_lastSpiStatus;
    }

  private:
    /// The SPI handle used for all transfers (set in the constructor).
    SPI_HandleTypeDef* _hspi;
    /// Status code (as uint32_t) of the most recent SPI transfer, readable via
    /// getLastSpiTransferStatus().
    volatile uint32_t _lastSpiStatus;
};
#endif
