// Jan 22, 2023
#include <Arduino.h>
#include <xbus.h>
#include <simple_timer.hpp>

#define HW_SERIAL_INTERFACE_XBUS Serial2
#define HW_SERIAL_BAUD_RATE_XBUS 2000000

#define PRINT_BINARY_DATA 0

xsens::Xbus xbus;

struct mti1_xbus_data
{
    mti1_xbus_data()
    {
        memset(this, 0, sizeof(mti1_xbus_data));
    }
    double accelerometer[3]; // 1000 Hz
    double gyroscope[3]; // 1000 Hz
    double magnetometer[3]; // 1000 Hz
    double temperature; // 100 Hz
} mti1;

struct Timers
{
    SimpleTimer accel;
    SimpleTimer gyro;
    SimpleTimer mag;
} timers;

unsigned long t_begin;

void setup()
{
    Serial.begin(2000000);

    // Initialize Xbus data streaming
    xbus.begin(HW_SERIAL_INTERFACE_XBUS, HW_SERIAL_BAUD_RATE_XBUS);

    t_begin = micros();
}

void loop()
{
    static unsigned long t_enter;

    // Polling Xbus data streaming
    if (xbus.read() == xsens::Xbus::XD_OK)
    {
        // Do something when a data packet is successfully parsed.
        t_enter = micros();

        if (xbus.get_accel_hr_double(mti1.accelerometer))
        {
            // If high-rate accelerometer measurements are updated
            timers.accel.elapsed();
            timers.accel.begin();
        }
        if (xbus.get_gyro_hr_double(mti1.gyroscope))
        {
            // If high-rate gyroscope measurements are updated
            timers.gyro.elapsed();
            timers.gyro.begin();
        }
        memcpy(mti1.magnetometer, xbus.mag.f64, sizeof(mti1.magnetometer));
        mti1.temperature = xbus.temperature.f64;
    }
    else
    {
        if (micros() - t_enter >= 1000000)
        {
            t_enter = micros();
            Serial.printf("No Xbus data stream ... %d\n", HW_SERIAL_INTERFACE_XBUS.available());
        }
    }

    if (micros() - t_begin >= 1000)
    {
        t_begin = micros();

#if PRINT_BINARY_DATA
        const uint8_t start_byte = 0xAA;
        const uint8_t finish_byte = 0xBB;
        const uint8_t packet_size = 80U;

        Serial.write(&start_byte, 1);
        Serial.write(&packet_size, 1);
        Serial.write((uint8_t *)xbus_data.accelerometer, sizeof(xbus_data.accelerometer));
        Serial.write((uint8_t *)xbus_data.gyroscope, 24U);
        Serial.write((uint8_t *)xbus_data.magnetometer, 24U);
        Serial.write((uint8_t *)&xbus_data.temperature, 8U);
        Serial.write(&finish_byte, 1);
        Serial.flush();
#else
        Serial.print(timers.accel.get_time_duration());
        Serial.print(" ");
        for (int i = 0; i < 3; i++)
        {
            Serial.print(mti1.accelerometer[i]);
            Serial.print(" ");
        }
        Serial.print(timers.gyro.get_time_duration());
        Serial.print(" ");
        for (int i = 0; i < 3; i++)
        {
            Serial.print(mti1.gyroscope[i]);
            Serial.print(" ");
        }
        for (int i = 0; i < 3; i++)
        {
            Serial.print(mti1.magnetometer[i]);
            Serial.print(" ");
        }
        Serial.print(mti1.temperature);
        Serial.println();
        Serial.flush();
#endif
    }
}