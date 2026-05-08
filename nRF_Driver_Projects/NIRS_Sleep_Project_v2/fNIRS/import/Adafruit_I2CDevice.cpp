#include "Adafruit_I2CDevice.h"

/*
 * nRF52832 project-specific BusIO I2C implementation:
 * - fixed Wire buffer budget (32 bytes)
 * - single requestFrom variant
 * - no Arduino platform-specific branches/macros
 */

Adafruit_I2CDevice::Adafruit_I2CDevice(uint8_t addr, TwoWire *theWire)
    : _addr(addr), _wire(theWire), _begun(false), _maxBufferSize(32u) {}

bool Adafruit_I2CDevice::begin(bool addr_detect)
{
    _wire->begin();
    _begun = true;
    if (addr_detect)
    {
        return detected();
    }
    return true;
}

void Adafruit_I2CDevice::end(void)
{
    _begun = false;
}

/* Used to detect whether the given device (whose address is prvided) is on the I2C line. */
bool Adafruit_I2CDevice::detected(void)
{
    if (!_begun && !begin()) {
        return false;
    }
    /* Transmit only the address and wait for acknowledgement -> device pulls down SDA to 0 */
    _wire->beginTransmission(_addr);
    return (_wire->endTransmission() == 0);
}

bool Adafruit_I2CDevice::write(const uint8_t *buffer, size_t len, bool stop,
                               const uint8_t *prefix_buffer,
                               size_t prefix_len)
{
    if ((len + prefix_len) > maxBufferSize()) {
        return false;
    }

    _wire->beginTransmission(_addr);

    if ((prefix_len != 0u) && (prefix_buffer != nullptr))  {
        if (_wire->write(prefix_buffer, prefix_len) != prefix_len) {
            return false;
        }
    }

    if (_wire->write(buffer, len) != len) {
        return false;
    }

    return (_wire->endTransmission(stop) == 0);
}

bool Adafruit_I2CDevice::read(uint8_t *buffer, size_t len, bool stop)
{
    size_t pos = 0u;
    while (pos < len) {
        const size_t read_len =
            ((len - pos) > maxBufferSize()) ? maxBufferSize() : (len - pos);
        const bool read_stop = (pos < (len - read_len)) ? false : stop;
        if (!_read(buffer + pos, read_len, read_stop))
        {
            return false;
        }
        pos += read_len;
    }
    return true;
}

bool Adafruit_I2CDevice::_read(uint8_t *buffer, size_t len, bool stop)
{
    const size_t recv =
        _wire->requestFrom((uint8_t)_addr, (uint8_t)len, (uint8_t)stop);

    if (recv != len) {
        return false;
    }

    for (size_t i = 0u; i < len; i++) {
        buffer[i] = _wire->read();
    }

    return true;
}

bool Adafruit_I2CDevice::write_then_read(const uint8_t *write_buffer,
                                         size_t write_len, uint8_t *read_buffer,
                                         size_t read_len, bool stop)
{
    if (!write(write_buffer, write_len, stop)) {
        return false;
    }

    return read(read_buffer, read_len);
}

uint8_t Adafruit_I2CDevice::address(void) {
    return _addr;
}

bool Adafruit_I2CDevice::setSpeed(uint32_t desiredclk)
{
    (void)desiredclk;
    return false;
}
