/***************************************************************************
  This is a library for the LSM9DS1 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM9DS1 Breakouts

  These sensors use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include "roboy_plexus/sensors/Adafruit_LSM9DS1.hpp"

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

void Adafruit_LSM9DS1::initI2C(int32_t sensorID) {
    _i2c = true;
    _lsm9dso_sensorid_accel = sensorID + 1;
    _lsm9dso_sensorid_mag = sensorID + 2;
    _lsm9dso_sensorid_gyro = sensorID + 3;
    _lsm9dso_sensorid_temp = sensorID + 4;
    _accelSensor = Sensor(this, &Adafruit_LSM9DS1::readAccel, &Adafruit_LSM9DS1::getAccelEvent,
                          &Adafruit_LSM9DS1::getAccelSensor);
    _magSensor = Sensor(this, &Adafruit_LSM9DS1::readMag, &Adafruit_LSM9DS1::getMagEvent,
                        &Adafruit_LSM9DS1::getMagSensor);
    _gyroSensor = Sensor(this, &Adafruit_LSM9DS1::readGyro, &Adafruit_LSM9DS1::getGyroEvent,
                         &Adafruit_LSM9DS1::getGyroSensor);
    _tempSensor = Sensor(this, &Adafruit_LSM9DS1::readTemp, &Adafruit_LSM9DS1::getTempEvent,
                         &Adafruit_LSM9DS1::getTempSensor);
}


// default
Adafruit_LSM9DS1::Adafruit_LSM9DS1(int32_t sensorID, void *i2c_base_address) {
    i2c = new I2C(i2c_base_address);
    initI2C(sensorID);
    t0 = high_resolution_clock::now();
}

bool Adafruit_LSM9DS1::begin() {
    // soft reset & reboot accel/gyro
    write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG8, 0x05);
    // soft reset & reboot magnetometer
    write8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_CTRL_REG2_M, 0x0C);
    /*
    for (uint8_t i=0; i<0x30; i++) {
      Serial.print("XG $"); Serial.print(i, HEX); Serial.print(" = 0x");
      Serial.println(read8(LSM9DS1_ADDRESS_ACCELGYRO, i), HEX);
    }
    for (uint8_t i=0; i<0x30; i++) {
      Serial.print("M $"); Serial.print(i, HEX); Serial.print(" = 0x");
      Serial.println(read8(LSM9DS1_ADDRESS_MAG, i), HEX);
    }
    */

    uint8_t id1 = read8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_WHO_AM_I_XG);
    printf("%x\t",id1);
    uint8_t id2 = read8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_WHO_AM_I_M);
    printf("%x\n",id2);
    if ((id1 != LSM9DS1_XG_ID) || (id2 != LSM9DS1_MAG_ID))
        return false;

    // enable gyro continuous
    write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG1_G, 0xC0); // on XYZ

    // Enable the accelerometer continous
    write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG5_XL, 0x38); // enable X Y and Z axis
    write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG6_XL, 0xC0); // 1 KHz out data rate, BW set by ODR, 408Hz anti-aliasing


    // enable mag continuous
    //write8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_CTRL_REG1_M, 0xFC); // high perf XY, 80 Hz ODR
    write8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_CTRL_REG3_M, 0x00); // continuous mode
    //write8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_CTRL_REG4_M, 0x0C); // high perf Z mode



    // Set default ranges for the various sensors
    setupAccel(LSM9DS1_ACCELRANGE_2G);
    setupMag(LSM9DS1_MAGGAIN_4GAUSS);
    setupGyro(LSM9DS1_GYROSCALE_245DPS);

    return true;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS1::read() {
    /* Read all the sensors. */
    readAccel();
    usleep(100);
    readMag();
    usleep(100);
    readGyro();
    usleep(100);
    readTemp();
}

void Adafruit_LSM9DS1::readAccel() {
    // Read the accelerometer
    byte buffer[6];
    readBuffer(LSM9DS1_ADDRESS_ACCELGYRO,
               0x80 | LSM9DS1_REGISTER_OUT_X_L_XL,
               6, buffer);

    uint8_t xlo = buffer[0];
    int16_t xhi = buffer[1];
    uint8_t ylo = buffer[2];
    int16_t yhi = buffer[3];
    uint8_t zlo = buffer[4];
    int16_t zhi = buffer[5];

    // Shift values to create properly formed integer (low byte first)
    xhi <<= 8;
    xhi |= xlo;
    yhi <<= 8;
    yhi |= ylo;
    zhi <<= 8;
    zhi |= zlo;
    accelData.x = xhi;
    accelData.y = yhi;
    accelData.z = zhi;
}

void Adafruit_LSM9DS1::readMag() {
    // Read the magnetometer
    byte buffer[6];
    readBuffer(LSM9DS1_ADDRESS_MAG,
               0x80 | LSM9DS1_REGISTER_OUT_X_L_M,
               6, buffer);

    uint8_t xlo = buffer[0];
    int16_t xhi = buffer[1];
    uint8_t ylo = buffer[2];
    int16_t yhi = buffer[3];
    uint8_t zlo = buffer[4];
    int16_t zhi = buffer[5];

    // Shift values to create properly formed integer (low byte first)
    xhi <<= 8;
    xhi |= xlo;
    yhi <<= 8;
    yhi |= ylo;
    zhi <<= 8;
    zhi |= zlo;
    magData.x = xhi;
    magData.y = yhi;
    magData.z = zhi;
}

void Adafruit_LSM9DS1::readGyro() {
    // Read gyro
    byte buffer[6];
    readBuffer(LSM9DS1_ADDRESS_ACCELGYRO,
               0x80 | LSM9DS1_REGISTER_OUT_X_L_G,
               6, buffer);

    uint8_t xlo = buffer[0];
    int16_t xhi = buffer[1];
    uint8_t ylo = buffer[2];
    int16_t yhi = buffer[3];
    uint8_t zlo = buffer[4];
    int16_t zhi = buffer[5];

    // Shift values to create properly formed integer (low byte first)
    xhi <<= 8;
    xhi |= xlo;
    yhi <<= 8;
    yhi |= ylo;
    zhi <<= 8;
    zhi |= zlo;

    gyroData.x = xhi;
    gyroData.y = yhi;
    gyroData.z = zhi;
}

void Adafruit_LSM9DS1::readTemp() {
    // Read temp sensor
    byte buffer[2];
    readBuffer(LSM9DS1_ADDRESS_ACCELGYRO,
               0x80 | LSM9DS1_REGISTER_TEMP_OUT_L,
               2, buffer);
    uint8_t xlo = buffer[0];
    int16_t xhi = buffer[1];

    xhi <<= 8;
    xhi |= xlo;

    // Shift values to create properly formed integer (low byte first)
    temperature = xhi;
}

void Adafruit_LSM9DS1::setupAccel(lsm9ds1AccelRange_t range) {
    uint8_t reg = read8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG6_XL);
    reg &= ~(0b00011000);
    reg |= range;
    //Serial.println("set range: ");
    write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG6_XL, reg);

    switch (range) {
        case LSM9DS1_ACCELRANGE_2G:
            _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
            break;
        case LSM9DS1_ACCELRANGE_4G:
            _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
            break;
        case LSM9DS1_ACCELRANGE_8G:
            _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
            break;
        case LSM9DS1_ACCELRANGE_16G:
            _accel_mg_lsb = LSM9DS1_ACCEL_MG_LSB_16G;
            break;
    }
}

void Adafruit_LSM9DS1::setupMag(lsm9ds1MagGain_t gain) {
    uint8_t reg = read8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_CTRL_REG2_M);
    reg &= ~(0b01100000);
    reg |= gain;
    write8(LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_CTRL_REG2_M, reg);

    switch (gain) {
        case LSM9DS1_MAGGAIN_4GAUSS:
            _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_4GAUSS;
            break;
        case LSM9DS1_MAGGAIN_8GAUSS:
            _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_8GAUSS;
            break;
        case LSM9DS1_MAGGAIN_12GAUSS:
            _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_12GAUSS;
            break;
        case LSM9DS1_MAGGAIN_16GAUSS:
            _mag_mgauss_lsb = LSM9DS1_MAG_MGAUSS_16GAUSS;
            break;
    }
}

void Adafruit_LSM9DS1::setupGyro(lsm9ds1GyroScale_t scale) {
    uint8_t reg = read8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG1_G);
    reg &= ~(0b00110000);
    reg |= scale;
    write8(LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_CTRL_REG1_G, reg);

    switch (scale) {
        case LSM9DS1_GYROSCALE_245DPS:
            _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_245DPS;
            break;
        case LSM9DS1_GYROSCALE_500DPS:
            _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_500DPS;
            break;
        case LSM9DS1_GYROSCALE_2000DPS:
            _gyro_dps_digit = LSM9DS1_GYRO_DPS_DIGIT_2000DPS;
            break;
    }
}


/***************************************************************************
 UNIFIED SENSOR FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Gets the most recent accel sensor event
*/
/**************************************************************************/
bool Adafruit_LSM9DS1::getEvent(sensors_event_t *accelEvent,
                                sensors_event_t *magEvent,
                                sensors_event_t *gyroEvent,
                                sensors_event_t *tempEvent) {
    /* Grab new sensor reading and timestamp. */
    read();
    t1 = high_resolution_clock::now();
    milliseconds time_span = duration_cast<milliseconds>(t1 - t0);
    uint32_t timestamp = time_span.count();

    /* Update appropriate sensor events. */
    if (accelEvent) getAccelEvent(accelEvent, timestamp);
    if (magEvent) getMagEvent(magEvent, timestamp);
    if (gyroEvent) getGyroEvent(gyroEvent, timestamp);
    if (tempEvent) getTempEvent(tempEvent, timestamp);

    return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_LSM9DS1::getSensor(sensor_t *accel, sensor_t *mag,
                                 sensor_t *gyro, sensor_t *temp) {
    /* Update appropriate sensor metadata. */
    if (accel) getAccelSensor(accel);
    if (mag) getMagSensor(mag);
    if (gyro) getGyroSensor(gyro);
    if (temp) getTempSensor(temp);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void Adafruit_LSM9DS1::write8(byte address, byte reg, byte value) {
    i2c->write(address, (reg<<24|value<<16), 2);
}

byte Adafruit_LSM9DS1::read8(byte address, byte reg) {
    vector<uint8_t> data;
    i2c->read(address, reg, 1, data);

    return data[0];
}

byte Adafruit_LSM9DS1::readBuffer(byte address, byte reg, byte len, uint8_t *buffer) {
    vector<uint8_t> data;
    i2c->read(address, reg, len, data);
    memcpy(buffer,data.data(),data.size());

    return data.size();
}

void Adafruit_LSM9DS1::getAccelEvent(sensors_event_t *event, uint32_t timestamp) {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_accel;
    event->type = SENSOR_TYPE_ACCELEROMETER;
    event->timestamp = timestamp;
    event->acceleration.x = accelData.x * _accel_mg_lsb;
    event->acceleration.x /= 1000;
    event->acceleration.x *= SENSORS_GRAVITY_STANDARD;
    event->acceleration.y = accelData.y * _accel_mg_lsb;
    event->acceleration.y /= 1000;
    event->acceleration.y *= SENSORS_GRAVITY_STANDARD;
    event->acceleration.z = accelData.z * _accel_mg_lsb;
    event->acceleration.z /= 1000;
    event->acceleration.z *= SENSORS_GRAVITY_STANDARD;
}

void Adafruit_LSM9DS1::getMagEvent(sensors_event_t *event, uint32_t timestamp) {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_mag;
    event->type = SENSOR_TYPE_MAGNETIC_FIELD;
    event->timestamp = timestamp;
    event->magnetic.x = magData.x * _mag_mgauss_lsb;
    event->magnetic.x /= 1000;
    event->magnetic.y = magData.y * _mag_mgauss_lsb;
    event->magnetic.y /= 1000;
    event->magnetic.z = magData.z * _mag_mgauss_lsb;
    event->magnetic.z /= 1000;
}

void Adafruit_LSM9DS1::getGyroEvent(sensors_event_t *event, uint32_t timestamp) {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_accel;
    event->type = SENSOR_TYPE_GYROSCOPE;
    event->timestamp = timestamp;
    event->gyro.x = gyroData.x * _gyro_dps_digit;
    event->gyro.y = gyroData.y * _gyro_dps_digit;
    event->gyro.z = gyroData.z * _gyro_dps_digit;
}

void Adafruit_LSM9DS1::getTempEvent(sensors_event_t *event, uint32_t timestamp) {
    memset(event, 0, sizeof(sensors_event_t));
    event->version = sizeof(sensors_event_t);
    event->sensor_id = _lsm9dso_sensorid_temp;
    event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    event->timestamp = timestamp;
    // This is just a guess since the staring point (21C here) isn't documented :(
    event->temperature = 21.0 + (float) temperature / 8;
    //event->temperature /= LSM9DS1_TEMP_LSB_DEGREE_CELSIUS;
}

void Adafruit_LSM9DS1::getAccelSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_A", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_accel;
    sensor->type = SENSOR_TYPE_ACCELEROMETER;
    sensor->min_delay = 0;
    sensor->max_value = 0.0;  // ToDo
    sensor->min_value = 0.0;  // ToDo
    sensor->resolution = 0.0;  // ToDo
}

void Adafruit_LSM9DS1::getMagSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_M", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_mag;
    sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
    sensor->min_delay = 0;
    sensor->max_value = 0.0;  // ToDo
    sensor->min_value = 0.0;  // ToDo
    sensor->resolution = 0.0;  // ToDo
}

void Adafruit_LSM9DS1::getGyroSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_G", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_gyro;
    sensor->type = SENSOR_TYPE_GYROSCOPE;
    sensor->min_delay = 0;
    sensor->max_value = 0.0;  // ToDo
    sensor->min_value = 0.0;  // ToDo
    sensor->resolution = 0.0;  // ToDo
}

void Adafruit_LSM9DS1::getTempSensor(sensor_t *sensor) {
    memset(sensor, 0, sizeof(sensor_t));
    strncpy(sensor->name, "LSM9DS1_T", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = _lsm9dso_sensorid_temp;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->max_value = 0.0;  // ToDo
    sensor->min_value = 0.0;  // ToDo
    sensor->resolution = 0.0;  // ToDo
}
