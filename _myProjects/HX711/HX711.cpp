#include <Arduino.h>
#include "HX711.h"


// TEENSYDUINO has a port of Dean Camera's ATOMIC_BLOCK macros for AVR to ARM Cortex M3.
#define HAS_ATOMIC_BLOCK (defined(ARDUINO_ARCH_AVR) || defined(TEENSYDUINO))

// Whether we are running on either the ESP8266 or the ESP32.
#define ARCH_ESPRESSIF (defined(ARDUINO_ARCH_ESP8266) || defined(ARDUINO_ARCH_ESP32))

// Whether we are actually running on FreeRTOS.
#define IS_FREE_RTOS defined(ARDUINO_ARCH_ESP32)

// Define macro designating whether we're running on a reasonable
// fast CPU and so should slow down sampling from GPIO.
#define FAST_CPU \
    ( \
    ARCH_ESPRESSIF || \
    defined(ARDUINO_ARCH_SAM)     || defined(ARDUINO_ARCH_SAMD) || \
    defined(ARDUINO_ARCH_STM32)   || defined(TEENSYDUINO) \
    )

#if HAS_ATOMIC_BLOCK
// Acquire AVR-specific ATOMIC_BLOCK(ATOMIC_RESTORESTATE) macro.
#include <util/atomic.h>
#endif

#if FAST_CPU
// Make shiftIn() be aware of clockspeed for
// faster CPUs like ESP32, Teensy 3.x and friends.
// See also:
// - https://github.com/bogde/HX711/issues/75
// - https://github.com/arduino/Arduino/issues/6561
// - https://community.hiveeyes.org/t/using-bogdans-canonical-hx711-library-on-the-esp32/539
uint8_t shiftInSlow(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
    uint8_t value = 0;
    uint8_t i;

    for(i = 0; i < 8; ++i) {
        digitalWrite(clockPin, HIGH);
        delayMicroseconds(1);
        if(bitOrder == LSBFIRST)
            value |= digitalRead(dataPin) << i;
        else
            value |= digitalRead(dataPin) << (7 - i);
        digitalWrite(clockPin, LOW);
        delayMicroseconds(1);
    }
    return value;
}
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftInSlow(data,clock,order)
#else
#define SHIFTIN_WITH_SPEED_SUPPORT(data,clock,order) shiftIn(data,clock,order)
#endif

HX711::HX711() 
{

}

HX711::~HX711() 
{

}

// 1.初始化DT SCK引脚，设置增益倍数
void HX711::begin(byte dout, byte pd_sck, byte gain) 
{
	PD_SCK = pd_sck;
	DOUT = dout;

	pinMode(PD_SCK, OUTPUT);// SCK引脚-->输出(OUTPUT)模式
	pinMode(DOUT, INPUT_PULLUP);// DT引脚--->输入上拉（INPUT_PULLUP）模式

	set_gain(gain); // 设置增益
}

// 2.检查HX711是否准备好
bool HX711::is_ready() 
{
	return digitalRead(DOUT) == LOW;// 当管脚 DOUT 从高电平变低电平后，HX711准备好了
}

// 3.设置增益
void HX711::set_gain(byte gain) 
{
	switch (gain) {
		case 128:		// channel A, gain factor 128
			GAIN = 1;
			break;
		case 64:		// channel A, gain factor 64
			GAIN = 3;
			break;
		case 32:		// channel B, gain factor 32
			GAIN = 2;
			break;
	}
}

// 4.读取
long HX711::read() 
{

	// 等待芯片准备好
	wait_ready();

	unsigned long value = 0;
	uint8_t data[3] = { 0 };
	uint8_t filler = 0x00;

	// Protect the read sequence from system interrupts.  If an interrupt occurs during
	// the time the PD_SCK signal is high it will stretch the length of the clock pulse.
	// If the total pulse time exceeds 60 uSec this will cause the HX711 to enter
	// power down mode during the middle of the read sequence.  While the device will
	// wake up when PD_SCK goes low again, the reset starts a new conversion cycle which
	// forces DOUT high until that cycle is completed.
	//
	// The result is that all subsequent bits read by shiftIn() will read back as 1,
	// corrupting the value returned by read().  The ATOMIC_BLOCK macro disables
	// interrupts during the sequence and then restores the interrupt mask to its previous
	// state after the sequence completes, insuring that the entire read-and-gain-set
	// sequence is not interrupted.  The macro has a few minor advantages over bracketing
	// the sequence between `noInterrupts()` and `interrupts()` calls.
	#if HAS_ATOMIC_BLOCK
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{

	#elif IS_FREE_RTOS
	// Begin of critical section.
	// Critical sections are used as a valid protection method
	// against simultaneous access in vanilla FreeRTOS.
	// Disable the scheduler and call portDISABLE_INTERRUPTS. This prevents
	// context switches and servicing of ISRs during a critical section.
	portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
	portENTER_CRITICAL(&mux);

	#else
	// Disable interrupts.
	noInterrupts();
	#endif

	// Pulse the clock pin 24 times to read the data.
	data[2] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
	data[1] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);
	data[0] = SHIFTIN_WITH_SPEED_SUPPORT(DOUT, PD_SCK, MSBFIRST);

	// Set the channel and the gain factor for the next reading using the clock pin.
	for (unsigned int i = 0; i < GAIN; i++) 
	{
		digitalWrite(PD_SCK, HIGH);
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
		digitalWrite(PD_SCK, LOW);
		#if ARCH_ESPRESSIF
		delayMicroseconds(1);
		#endif
	}

	#if IS_FREE_RTOS
	// End of critical section.
	portEXIT_CRITICAL(&mux);

	#elif HAS_ATOMIC_BLOCK
	}

	#else
	// Enable interrupts again.
	interrupts();
	#endif

	// Replicate the most significant bit to pad out a 32-bit signed integer
	if (data[2] & 0x80) {
		filler = 0xFF;
	} else {
		filler = 0x00;
	}

	// Construct a 32-bit signed integer
	value = ( static_cast<unsigned long>(filler) << 24
			| static_cast<unsigned long>(data[2]) << 16
			| static_cast<unsigned long>(data[1]) << 8
			| static_cast<unsigned long>(data[0]) );

	return static_cast<long>(value);
}

// 等待芯片准备好，准备好会跳出while循环，执行下一步
void HX711::wait_ready(unsigned long delay_ms) {
	while (!is_ready()) // 调用 bool is_ready()
	{
		delay(delay_ms);
	}
}
// 等待芯片准备好，重试指定的次数
bool HX711::wait_ready_retry(int retries, unsigned long delay_ms) {
	int count = 0;
	while (count < retries) 
	{
		if (is_ready()) 
		{
			return true;
		}
		delay(delay_ms);
		count++;
	}
	return false;
}
// 等待芯片准备好，直到设定的超时
bool HX711::wait_ready_timeout(unsigned long timeout, unsigned long delay_ms) {
	unsigned long millisStarted = millis();
	while (millis() - millisStarted < timeout) {
		if (is_ready()) {
			return true;
		}
		delay(delay_ms);
	}
	return false;
}

// 返回读取设定次数内读数的平均值
long HX711::read_average(byte times) 
{
	long sum = 0;
	for (byte i = 0; i < times; i++) 
	{
		sum += read();
		delay(0);
	}
	return sum / times;
}

// 取平均后值减去调零值
double HX711::get_value(byte times) 
{
	return read_average(times) - OFFSET;
}

// 转化为应变
double HX711::toStrain(byte times)
{
	double strain;
	strain=4*get_value(times)/(GAIN*K*16777216);
	return strain;
}

// 转化为应力
double HX711::toStress(byte times)
{
	double stress;
	stress=E*toStrain(times);
	return stress;
}

// 多次取平均获取调零值
void HX711::tare(byte times) 
{
	double sum = read_average(times);
	set_offset(sum);// 设置调零值
}

// 设置调零值
void HX711::set_offset(long offset) 
{
	OFFSET = offset;
}

// 获取调零值
long HX711::get_offset() 
{
	return OFFSET;
}

// 将芯片置于掉电模式
void HX711::power_down() 
{
	digitalWrite(PD_SCK, LOW);
	digitalWrite(PD_SCK, HIGH);
}

// 掉电模式后唤醒芯片
void HX711::power_up() 
{
	digitalWrite(PD_SCK, LOW);
}
