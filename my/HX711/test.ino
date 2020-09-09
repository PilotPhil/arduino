#include "HX711.h"

const int DT_PIN=2;
const int SCK_PIN=3;

HX711 test;

void setup() 
{
  Serial.begin(9600);
  test.begin(DT_PIN,SCK_PIN,128);// 初始化DT SCK引脚，设置增益倍数
  test.tare(50);
}

void loop() 
{
  if(test.is_ready())
  {
    // long read=test.toStrain();
    long read=test.get_value();
    Serial.println(read);
  }
  else
  {
    Serial.println("HX711 not found");
  }

  delay(100);
  
}
