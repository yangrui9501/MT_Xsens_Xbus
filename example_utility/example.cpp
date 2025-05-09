#include <Arduino.h>
#include <xbus.h>

xsens::Xbus xbus;
auto &MTSerial = Serial4;
const uint32_t MT_BAUD_RATE = 2000000;
uint8_t mt_buffer[512];

void print_mt_data()
{
	// Orientation: Quaternion
	Serial.print("Quaternion : ");
	for (int i = 0; i < 4; i++)
	{
		Serial.print(xbus.quat.f64[i]);
		Serial.print(" ");
	}
	Serial.println();

	// Inertial data: angular velocity
	Serial.print("Angular velocity : ");
	for (int i = 0; i < 3; i++)
	{
		Serial.print(xbus.gyro.f64[i]);
		Serial.print(" ");
	}

	// High-rate angular velocity
	Serial.print("High-rate angular velocity : ");
	for (int i = 0; i < 3; i++)
	{
		Serial.print(xbus.gyro_hr[i]);
		Serial.print(" ");
	}
	Serial.println();

	// Inertial data: acceleration
	Serial.print("Acceleration : ");
	for (int i = 0; i < 3; i++)
	{
		Serial.print(xbus.accel.f64[i]);
		Serial.print(" ");
	}

	// High-rate acceleration
	Serial.print("High-rate acceleration : ");
	for (int i = 0; i < 3; i++)
	{
		Serial.print(xbus.accel_hr[i]);
		Serial.print(" ");
	}
	Serial.println();

	// Free acceleration
	Serial.print("Free acceleration : ");
	for (int i = 0; i < 3; i++)
	{
		Serial.print(xbus.free_accel.f64[i]);
		Serial.print(" ");
	}
	Serial.println();
}

void setup()
{
	Serial.begin(115200);
	MTSerial.begin(MT_BAUD_RATE);

	pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
	static unsigned long last_update; // For monitoring data update status
	auto available_ = MTSerial.available();
	if (available_ > 0)
	{
		// Saturate available_ to the buffer size
		available_ = min(available_, sizeof(mt_buffer));

		auto length_ = MTSerial.readBytes(mt_buffer, available_);
		if (xbus.parse(mt_buffer, length_) == xsens::Xbus::XD_OK)
		{
			last_update = micros();

			// Print data
		}
	}
	else
	{
		if (micros() - last_update >= 1000000)
		{
			last_update = micros();
			Serial.println("No MT data available ...");
		}
	}

	// For system status monitoring
	static unsigned long last_time = 0;
	if (micros() - last_time >= 500000)
	{
		last_time = micros();

		// Toggle the LED when the system is running
		digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
	}
}