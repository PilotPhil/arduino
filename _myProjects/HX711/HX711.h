#ifndef HX711_h
#define HX711_h

#include "Arduino.h"

// HX711类
class HX711
{
	private:
		byte PD_SCK;	// Power Down and Serial Clock Input Pin
		byte DOUT;		// Serial Data Output Pin
		byte GAIN;		// amplification factor
		long OFFSET = 0;	// 调零值
		float K=2.11;	// 应变片灵敏度
		float E=2.1E8;	// 杨氏模量

	public:
		HX711();

		virtual ~HX711();

		// 初始化DT SCK引脚，设置增益倍数。通道设置由增益倍数决定。设置增益为64或128，走A通道。设置增益为32，走B通道
		void begin(byte dout, byte pd_sck, byte gain = 128);

		// 检查HX711是否准备好
		// 当管脚DOUT为高电平时，表示HX711还未准备好输出数据，此时串口时钟输入信号SCK需要置为低电平。当管脚 DOUT 从高电平变低电平后，HX711准备好了
		bool is_ready();

		// 等待芯片准备好
		void wait_ready(unsigned long delay_ms = 0);
		bool wait_ready_retry(int retries = 3, unsigned long delay_ms = 0);
		bool wait_ready_timeout(unsigned long timeout = 1000, unsigned long delay_ms = 0);

		// 设置增益
		void set_gain(byte gain = 128);

		// waits for the chip to be ready and returns a reading
		long read();

		// 返回读取设定次数内读数的平均值
		long read_average(byte times = 10);

		// 取平均后值减去调零值
		double get_value(byte times = 1);

		// 转化为应变
		double toStrain(byte times = 1);

		// 转化为应力
		double toStress(byte times = 1);

		// 多次取平均设置调零值
		void tare(byte times = 10);

		// 设置调零值
		void set_offset(long offset = 0);

		// 获取调零值
		long get_offset();

		// 将芯片置于掉电模式
		void power_down();

		// 掉电模式后唤醒芯片
		void power_up();
};

#endif
