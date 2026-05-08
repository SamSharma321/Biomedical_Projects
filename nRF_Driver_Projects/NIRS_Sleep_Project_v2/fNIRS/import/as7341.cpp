#include "as7341.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#define colorsensor DT_NODELABEL(as7341)

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(colorsensor);

LOG_MODULE_REGISTER(as7341,LOG_LEVEL_DBG);

int as7341_read(uint8_t regaddr, uint8_t *buf){
    int ret = i2c_write_read_dt(&dev_i2c, &regaddr, 1, buf, 1);
    
    if(ret != 0){
        LOG_INF("Failed to write/read I2C device address %x at Reg. %x \n\r", dev_i2c.addr, regaddr);
    } 
    return ret;
}

int as7341_readBytes(uint8_t regaddr, uint8_t *buf, uint8_t length){
    int ret = i2c_write_read_dt(&dev_i2c, &regaddr, 1, buf, length);
    
    if(ret != 0){
        LOG_INF("Failed to write/read I2C device address %x at Reg. %x \n\r", dev_i2c.addr, regaddr);
    } 
    return ret;
}

int as7341_write(uint8_t regaddr, uint8_t data){
    uint8_t wtbuf[2]={regaddr, data};
    int ret = i2c_write_dt(&dev_i2c, wtbuf, sizeof(wtbuf));
    
    if(ret != 0){
        LOG_INF("Failed to write/read I2C device address %x at Reg. %x \n\r", dev_i2c.addr, regaddr);
    } 
    return ret;
}

int as7341_writeBit(uint8_t regaddr, uint8_t bitPosition, bool enable){
    uint8_t readbuf = 0;
    i2c_write_read_dt(&dev_i2c, &regaddr, 1, &readbuf, 1);

    if (enable){
        readbuf |= (1 << bitPosition);
    } else{
        readbuf &= ~(1 << bitPosition);
    }
    uint8_t wtbuf[2]={regaddr, readbuf};
    int ret = i2c_write_dt(&dev_i2c, wtbuf, sizeof(wtbuf));
    
    if(ret != 0){
        LOG_INF("Failed to write/read I2C device address %x at Reg. %x \n\r", dev_i2c.addr, regaddr);
    } 
    return ret;
}

/**
 * @brief Construct a new Adafruit_AS7341::Adafruit_AS7341 object
 *
 */
Adafruit_AS7341::Adafruit_AS7341(void) {}

/**
 * @brief Destroy the Adafruit_AS7341::Adafruit_AS7341 object
 *
 */
Adafruit_AS7341::~Adafruit_AS7341(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  i2c_address
 *            The I2C address to be used.
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @param  sensor_id
 *            The unique ID to differentiate the sensors from others
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_AS7341::begin(){
    if (!device_is_ready(dev_i2c.bus)){
        LOG_INF("I2C bus %s is not read", dev_i2c.bus->name);
        return false;
    }

    return _init();
}

/*!  @brief Initializer for post i2c/spi init
 *   @param sensor_id Optional unique ID for the sensor set
 *   @returns True if chip identified and initialized
 */
bool Adafruit_AS7341::_init() {
    uint8_t readbuf = 0;
    uint8_t regaddr = AS7341_ID;
    int ret = as7341_read(regaddr, &readbuf);

    // make sure we're talking to the right chip
    if ((readbuf & 0xFC) != (AS7341_CHIP_ID << 2)) {
        return false;
    }

    powerEnable(true);
    LOG_INF("ID Register: %x", readbuf>>2); // 0x92 register 1:0 bit reserved

    return (ret==0);
}

/**
 * @brief Sets the power state of the sensor
 *
 * @param enable_power true: on false: off
 */
void Adafruit_AS7341::powerEnable(bool enable_power) {
    uint8_t readbuf = 0;
    uint8_t regaddr = AS7341_ENABLE;
    int ret = as7341_writeBit(regaddr, 0, true); // Enable register PON bit
    ret = as7341_read(regaddr, &readbuf);

    if (ret==0){
        LOG_INF("power enabled"); 
    }

}



/**
 * @brief Enables measurement of spectral data
 *
 * @param enable_measurement true: enabled false: disabled
 * @return true: success false: failure
 */
bool Adafruit_AS7341::enableSpectralMeasurement(bool enable_measurement) {
    uint8_t regaddr = AS7341_ENABLE;
    int ret = as7341_writeBit(regaddr, 1, enable_measurement);

    return (ret == 0);
}

void Adafruit_AS7341::setSMUXLowChannels(bool f1_f4) {
    enableSpectralMeasurement(false);
    setSMUXCommand(AS7341_SMUX_CMD_WRITE);
    if (f1_f4) {
        setup_F1F4_Clear_NIR();
    } else {
        setup_F5F8_Clear_NIR();
    }
    enableSMUX();
}

bool Adafruit_AS7341::setSMUXCommand(as7341_smux_cmd_t command) {
    uint8_t regaddr = AS7341_CFG6;
    uint8_t data = 0;
    int ret = as7341_read(regaddr, &data);
    ret = as7341_write(regaddr, ((data & 0xF3) | command << 3)); // CFG6 Register set
    return (ret == 0);
}

/**
 * @brief Returns the ADC data for a given channel
 *
 * @param channel The ADC channel to read
 * @return uint16_t The measured data for the currently configured sensor
 */
uint16_t Adafruit_AS7341::readChannel(as7341_adc_channel_t channel) {
    // each channel has two s, so offset by two for each next channel

    uint8_t regaddr = AS7341_CH0_DATA_L + 2 * channel;
    uint8_t datal, datah = 0;
    int retl = as7341_read(regaddr, &datal);
    int reth = as7341_read(regaddr+1, &datah);

    if((reth|retl) != 0){
        LOG_INF("Failed to write/read I2C device address %x at Reg. %x \n\r", dev_i2c.addr, regaddr);
    } 
    
    uint16_t data = (datah<<8 | datal);
    // LOG_INF("Channel %d data: %d", (int) channel, data);
    return data;

}

/**
 * @brief fills the provided buffer with the current measurements for Spectral
 * channels F1-8, Clear and NIR
 *
 * @param readings_buffer Pointer to a buffer of length 10 or more to fill with
 * sensor data
 * @return true: success false: failure
 */
bool Adafruit_AS7341::readAllChannels_ori(uint16_t *readings_buffer) {
    bool retl, reth;
    setSMUXLowChannels(true);        // Configure SMUX to read low channels
    retl = enableSpectralMeasurement(true); // Start integration
    delayForData(0);                 // I'll wait for you for all time
    // for (int i=0;i<6;i++){
	// 	readings_buffer[i] = this->readChannel((as7341_adc_channel_t)i);
	// }
    uint8_t temp_buf[12] = {0};
    as7341_readBytes(AS7341_CH0_DATA_L, temp_buf, 12);
    for (int i=0;i<6;i=i+2){
        readings_buffer[i] = temp_buf[i+1] << 8 | temp_buf[i];
    }

    setSMUXLowChannels(false);       // Configure SMUX to read high channels
    reth = enableSpectralMeasurement(true); // Start integration
    delayForData(0);                 // I'll wait for you for all time
    for (int i=0;i<6;i++){
		readings_buffer[i+6] = this->readChannel((as7341_adc_channel_t)i);
	}
    LOG_INF("415nm:%d,\t445nm:%d,\t480nm:%d,\t515nm:%d,\t555nm:%d,\t590nm:%d,\t630nm:%d,\t680nm:%d,\tClear:%d,\tNIR:%d", 
    readings_buffer[AS7341_CHANNEL_415nm_F1], 
    readings_buffer[AS7341_CHANNEL_445nm_F2], 
    readings_buffer[AS7341_CHANNEL_480nm_F3], 
    readings_buffer[AS7341_CHANNEL_515nm_F4], 
    readings_buffer[AS7341_CHANNEL_555nm_F5], 
    readings_buffer[AS7341_CHANNEL_590nm_F6], 
    readings_buffer[AS7341_CHANNEL_630nm_F7], 
    readings_buffer[AS7341_CHANNEL_680nm_F8], 
    readings_buffer[AS7341_CHANNEL_CLEAR], 
    readings_buffer[AS7341_CHANNEL_NIR]);

    return (retl|reth);
}

bool Adafruit_AS7341::readAllChannels(uint16_t *readings_buffer) {
    bool retl, reth;
    setSMUXLowChannels(true);        // Configure SMUX to read low channels
    retl = enableSpectralMeasurement(true); // Start integration
    delayForData(0);                 // I'll wait for you for all time

    uint8_t temp_buf[12] = {0};
    as7341_readBytes(AS7341_CH0_DATA_L, temp_buf, 12);
    for (int i=0;i<12;i=i+2){
        readings_buffer[i/2] = temp_buf[i+1] << 8 | temp_buf[i];
    }

    setSMUXLowChannels(false);       // Configure SMUX to read high channels
    reth = enableSpectralMeasurement(true); // Start integration
    delayForData(0);                 // I'll wait for you for all time
    as7341_readBytes(AS7341_CH0_DATA_L, temp_buf, 12);
    for (int i=0;i<12;i=i+2){
        readings_buffer[(i/2)+6] = temp_buf[i+1] << 8 | temp_buf[i];
    }

    LOG_INF("415nm:%d,\t445nm:%d,\t480nm:%d,\t515nm:%d,\t555nm:%d,\t590nm:%d,\t630nm:%d,\t680nm:%d,\tClear:%d,\tNIR:%d", 
    readings_buffer[AS7341_CHANNEL_415nm_F1], 
    readings_buffer[AS7341_CHANNEL_445nm_F2], 
    readings_buffer[AS7341_CHANNEL_480nm_F3], 
    readings_buffer[AS7341_CHANNEL_515nm_F4], 
    readings_buffer[AS7341_CHANNEL_555nm_F5], 
    readings_buffer[AS7341_CHANNEL_590nm_F6], 
    readings_buffer[AS7341_CHANNEL_630nm_F7], 
    readings_buffer[AS7341_CHANNEL_680nm_F8], 
    readings_buffer[AS7341_CHANNEL_CLEAR], 
    readings_buffer[AS7341_CHANNEL_NIR]);

    return (retl|reth);
}

bool Adafruit_AS7341::readCurrentMUXChannels(uint16_t *readings_buffer) {
    delayForData(0);                 // I'll wait for you for all time

    uint8_t temp_buf[12] = {0};
    int ret = as7341_readBytes(AS7341_CH0_DATA_L, temp_buf, 12);
    for (int i=0;i<12;i=i+2){
        readings_buffer[i/2] = temp_buf[i+1] << 8 | temp_buf[i];
    }
    return (ret == 0);
}


/**
 * @brief Configure SMUX for sensors F1-4, Clear and NIR
 *
 */
void Adafruit_AS7341::setup_F1F4_Clear_NIR() {
    // SMUX Config for F1,F2,F3,F4,NIR,Clear
    as7341_write(0x00, 0x30); // F3 left set to ADC2
    as7341_write(0x01, 0x01); // F1 left set to ADC0
    as7341_write(0x02, 0x00); // Reserved or disabled
    as7341_write(0x03, 0x00); // F8 left disabled
    as7341_write(0x04, 0x00); // F6 left disabled
    as7341_write(0x05, 0x42); // F4 left connected to ADC3/f2 left connected to ADC1
    as7341_write(0x06, 0x00); // F5 left disbled
    as7341_write(0x07, 0x00); // F7 left disbled
    as7341_write(0x08, 0x50); // CLEAR connected to ADC4
    as7341_write(0x09, 0x00); // F5 right disabled
    as7341_write(0x0A, 0x00); // F7 right disabled
    as7341_write(0x0B, 0x00); // Reserved or disabled
    as7341_write(0x0C, 0x20); // F2 right connected to ADC1
    as7341_write(0x0D, 0x04); // F4 right connected to ADC3
    as7341_write(0x0E, 0x00); // F6/F8 right disabled
    as7341_write(0x0F, 0x30); // F3 right connected to AD2
    as7341_write(0x10, 0x01); // F1 right connected to AD0
    as7341_write(0x11, 0x50); // CLEAR right connected to AD4
    as7341_write(0x12, 0x00); // Reserved or disabled
    as7341_write(0x13, 0x06); // NIR connected to ADC5
}

/**
 * @brief Configure SMUX for sensors F5-8, Clear and NIR
 *
 */
void Adafruit_AS7341::setup_F5F8_Clear_NIR() {
    // SMUX Config for F5,F6,F7,F8,NIR,Clear
    as7341_write(0x00, 0x00); // F3 left disable
    as7341_write(0x01, 0x00); // F1 left disable
    as7341_write(0x02, 0x00); // reserved/disable
    as7341_write(0x03, 0x40); // F8 left connected to ADC3
    as7341_write(0x04, 0x02); // F6 left connected to ADC1
    as7341_write(0x05, 0x00); // F4/ F2 disabled
    as7341_write(0x06, 0x10); // F5 left connected to ADC0
    as7341_write(0x07, 0x03); // F7 left connected to ADC2
    as7341_write(0x08, 0x50); // CLEAR Connected to ADC4
    as7341_write(0x09, 0x10); // F5 right connected to ADC0
    as7341_write(0x0A, 0x03); // F7 right connected to ADC2
    as7341_write(0x0B, 0x00); // Reserved or disabled
    as7341_write(0x0C, 0x00); // F2 right disabled
    as7341_write(0x0D, 0x00); // F4 right disabled
    as7341_write(0x0E, 0x24); // F8 right connected to ADC2/ F6 right connected to ADC1
    as7341_write(0x0F, 0x00); // F3 right disabled
    as7341_write(0x10, 0x00); // F1 right disabled
    as7341_write(0x11, 0x50); // CLEAR right connected to AD4
    as7341_write(0x12, 0x00); // Reserved or disabled
    as7341_write(0x13, 0x06); // NIR connected to ADC5
}

bool Adafruit_AS7341::enableSMUX(void) {
    uint8_t regaddr = AS7341_ENABLE;
    uint8_t data = 0;
    int ret = as7341_writeBit(regaddr, 4, true);
    
    int timeOut = 1000; // Arbitrary value, but if it takes 1000 milliseconds then something is wrong
    int count = 0;
    while ((data & 0x03) && count < timeOut) {
        as7341_read(regaddr, &data);
        k_msleep(1);
        count++;
    }
    if (count >= timeOut)
        return false;
    else
        return (ret==0);
}

/**
 * @brief Delay while waiting for data, with option to time out and recover
 *
 * @param waitTime the maximum amount of time to wait
 * @return none
 */
void Adafruit_AS7341::delayForData(int waitTime) {
  if (waitTime == 0) // Wait forever
  {
    while (!getIsDataReady()) {
      k_msleep(1);
    }
    return;
  }
  if (waitTime > 0) // Wait for that many milliseconds
  {
    uint32_t elapsedMillis = 0;
    while (!getIsDataReady() && elapsedMillis < (uint32_t)waitTime) {
      k_msleep(1);
      elapsedMillis++;
    }
    return;
  }
  if (waitTime < 0) {
    // For future use?
    return;
  }
}

/**
 * @brief
 *
 * @return true: success false: failure
 */
bool Adafruit_AS7341::getIsDataReady() {
    uint8_t regaddr = AS7341_STATUS2;
    uint8_t data = 0;
    int ret = as7341_read(regaddr, &data);
    if (ret !=0){
        return false;
    }
    return (data & 0x40);
}

/**
 * @brief Sets the integration time step count
 *
 * Total integration time will be `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @param atime_value The integration time step count
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setATIME(uint8_t atime_value) {
    int ret = as7341_write(AS7341_ATIME, atime_value);
    return (ret==0);
}

/**
 * @brief Returns the integration time step count
 *
 * Total integration time will be `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @return uint8_t The current integration time step count
 */
uint8_t Adafruit_AS7341::getATIME() {
    uint8_t atime=0;
    as7341_read(AS7341_ATIME, &atime);
    return atime;
}

/**
 * @brief Sets the integration time step size
 *
 * @param astep_value Integration time step size in 2.78 microsecon increments
 * Step size is `(astep_value+1) * 2.78 uS`
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setASTEP(uint16_t astep_value) {
    int ret = as7341_write(AS7341_ASTEP_L, (uint8_t)(astep_value & 0xff));
    ret |= as7341_write(AS7341_ASTEP_H, (uint8_t)(astep_value>>8));
    return (ret==0);
}

/**
 * @brief Returns the integration time step size
 *
 * Step size is `(astep_value+1) * 2.78 uS`
 *
 * @return uint16_t The current integration time step size
 */
uint16_t Adafruit_AS7341::getASTEP() {
    uint8_t astepl, asteph=0;
    as7341_read(AS7341_ASTEP_H, &asteph);
    as7341_read(AS7341_ASTEP_L, &astepl);
    return (asteph<<8 | astepl);
}

/**
 * @brief Sets the ADC gain multiplier
 *
 * @param gain_value The gain amount. must be an `as7341_gain_t`
 * @return true: success false: failure
 */
bool Adafruit_AS7341::setGain(as7341_gain_t gain_value) {
    // AGAIN bitfield is only[0:4] but the rest is empty
    int ret = as7341_write(AS7341_CFG1, gain_value);
    return (ret==0);
}

/**
 * @brief Returns the ADC gain multiplier
 *
 * @return as7341_gain_t The current ADC gain multiplier
 */
as7341_gain_t Adafruit_AS7341::getGain() {
    uint8_t readbuf = 0;
    as7341_read(AS7341_CFG1, &readbuf);
    return (as7341_gain_t)readbuf;
}

/**
 * @brief Returns the integration time
 *
 * The integration time is `(ATIME + 1) * (ASTEP + 1) * 2.78µS`
 *
 * @return long The current integration time in ms
 */
long Adafruit_AS7341::getTINT() {
  long astep = getASTEP();
  long atime = getATIME();

  return (atime + 1) * (astep + 1) * 2.78 / 1000;
}