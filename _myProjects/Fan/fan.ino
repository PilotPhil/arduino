//

int lightSensorPin=7;//光敏模拟输入
int lightSensorVal=0;//光敏传感器数值

int relayPin=2;//继电器控制引脚
int switchVal=800;//

void setup() 
{
  Serial.begin(9600);

  pinMode(lightSensorPin,INPUT);//设置光敏传感器引脚输入
  pinMode(relayPin,OUTPUT);//继电器引脚输出

}

void loop() 
{
  lightSensorVal=analogRead(lightSensorPin);//读取光敏传感器数值
  Serial.println(lightSensorVal);//打印光强

  if (lightSensorVal>switchVal)//较暗
  {
    digitalWrite(relayPin,LOW);//开
  }
  else if (lightSensorVal<switchVal)//较亮
  {
    digitalWrite(relayPin,HIGH);//关
  }
  
  delay(100);
}
