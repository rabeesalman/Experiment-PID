#include <Arduino.h>
#include <PID_v1.h>
#define PIN_OUTPUT 3
double Setpoint, Input, Output;
// double Kp = 10, Ki = 5, Kd = 6;
double Kp = 60, Ki = 5, Kd = 100;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
volatile bool detection = 0;
volatile unsigned long Value = 0;
volatile bool last_CH1_state = 0;
//

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

// // void rabee()
// // {
// //   if (!last_CH1_state)
// //   {
// //     detection = 1;
// //   }
// //   else if (last_CH1_state)
// //   {
// //     detection = 0;
// //     last_CH1_state = 0;
// //   }
// // }

void setup()
{
  pinMode(A5, INPUT);
  pinMode(PIN_OUTPUT, OUTPUT);
  digitalWrite(PIN_OUTPUT, 0);
  Serial.begin(1000000);
  Input = stien_three(A5, 10000.0, 7.8786994030E-04, 2.8985556847E-04, -1.3697290359E-07);
  Setpoint = 35.0;
  myPID.SetMode(AUTOMATIC);
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);
  // attachInterrupt(digitalPinToInterrupt(3), rabee, RISING);
}

void loop()
{
  Input = stien_three(A5, 10000.0, 7.8786994030E-04, 2.8985556847E-04, -1.3697290359E-07);
  myPID.Compute();
  // Value = map(analogRead(A5), 250, 1000, 7200, 6);
  //  ....map(value, fromLow, fromHigh, toLow, toHigh)

  Value = map(Output, 0, 255, 7200, 6);

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
    if (!last_CH1_state)
    {
      detection = 1;
    }
  }
  else if (last_CH1_state)
  {
    detection = 1;
    last_CH1_state = 0;
  }
   if (detection)
  {
    delayMicroseconds(Value);
    digitalWrite(PIN_OUTPUT, HIGH);
    delayMicroseconds(100);
    digitalWrite(PIN_OUTPUT, LOW);
    detection = 0;
  }
}


