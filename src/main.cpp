#include <Arduino.h>
#include <PID_v1.h>
#define PIN_OUTPUT 3
double Setpoint, Input, Output;
//double Kp = 10, Ki = 5, Kd = 6;
double Kp = 6, Ki = 8, Kd = 0;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
uint8_t detection = 0;
int Value = 0;
int last_CH1_state = 0;

float stien_three(byte Thermistor_Pin, float Rdivid, float A, float B, float C)
{
  pinMode(Thermistor_Pin, INPUT);
  int Raw_Data = 0;
  unsigned long sum = 0;
  float tmp = 0.0;
  float NTC_Resistor = 0.0;
  double lnR2;
  float T;
  for (uint8_t i = 0; i < 50; i++)
  {
    Raw_Data = analogRead(Thermistor_Pin);
    sum += Raw_Data;
  }
  tmp = float(sum) / float(50);
  NTC_Resistor = Rdivid * (tmp / (1024.0 - tmp));
  lnR2 = log(NTC_Resistor);
  T = 1 / (A + B * lnR2 + C * lnR2 * lnR2 * lnR2);
  T = (T - 273.15);
  return T;
}

void setup()
{
  pinMode(A4, INPUT);
  pinMode(PIN_OUTPUT, OUTPUT);
  Serial.begin(9600);
  Input = stien_three(A4, 10000.0, 7.8786994030E-04, 2.8985556847E-04, -1.3697290359E-07);
  Setpoint = 36;
  myPID.SetMode(AUTOMATIC);
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
}
void loop()
{
  Input = stien_three(A4, 10000.0, 7.8786994030E-04, 2.8985556847E-04, -1.3697290359E-07);
  myPID.Compute();
  Value = map(Output, 0, 255, 5000, 10);
  //  ....map(value, fromLow, fromHigh, toLow, toHigh)
  Serial.print(Input);
  Serial.print(" ");
  Serial.print(Output / 8);
  Serial.print(" ");
  Serial.println(Setpoint);
}

ISR(PCINT0_vect)
{
  if (PINB & B00000001)
  {
    if (last_CH1_state == 0)
    {
      detection = 1;
    }
  }
  else if (last_CH1_state == 1)
  {
    detection = 1;
    last_CH1_state = 0;
  }
  while (detection)
  {
    // delayMicroseconds(Value);
    // digitalWrite(PIN_OUTPUT, HIGH);
    // delayMicroseconds(100);
    // digitalWrite(PIN_OUTPUT, LOW);
    delayMicroseconds( Value);
    digitalWrite(PIN_OUTPUT, HIGH);
    delayMicroseconds(5000 -Value);
    digitalWrite(PIN_OUTPUT, LOW);
    detection = 0;
  }
}