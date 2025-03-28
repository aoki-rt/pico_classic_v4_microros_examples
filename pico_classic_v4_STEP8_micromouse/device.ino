// Copyright 2025 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

hw_timer_t * g_timer0 = NULL;
hw_timer_t * g_timer1 = NULL;
hw_timer_t * g_timer2 = NULL;
hw_timer_t * g_timer3 = NULL;

volatile bool g_motor_move = 0;
volatile unsigned int g_step_r, g_step_l;
unsigned short g_step_hz_r = 30;
unsigned short g_step_hz_l = 30;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void stepHzSetR(short data) { g_step_hz_r = data; }
void stepHzSetL(short data) { g_step_hz_l = data; }
void stepClearR(void) { g_step_r = 0; }
void stepClearL(void) { g_step_l = 0; }
unsigned int stepGetR(void) { return g_step_r; }
unsigned int stepGetL(void) { return g_step_l; }
void motorMoveSet(bool data) { g_motor_move = data; }
bool motorMoveGet(void) {  return g_motor_move;}


void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  controlInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR onTimer1(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  sensorInterrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

#ifndef PCC4
void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    if (g_step_hz_r < 30) g_step_hz_r = 30;
    timerAlarm(g_timer2, 2000000 / g_step_hz_r, true, 0);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
    g_step_r++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);
  if (g_motor_move) {
    if (g_step_hz_l < 30) g_step_hz_l = 30;
    timerAlarm(g_timer3, 2000000 / g_step_hz_l, true, 0);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
    g_step_l++;
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);
}
#endif

void controlInterruptStart(void) { timerStart(g_timer0); }
void controlInterruptStop(void) { timerStop(g_timer0); }

void sensorInterruptStart(void) { timerStart(g_timer1); }
void sensorInterruptStop(void) { timerStop(g_timer1); }

void PWMInterruptStart(void)
{
  timerStart(g_timer2);
  timerStart(g_timer3);
}
void PWMInterruptStop(void)
{
  timerStop(g_timer2);
  timerStop(g_timer3);
}

void deviceInit(void)
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(BLED0, OUTPUT);
  pinMode(BLED1, OUTPUT);

  pinMode(SW_L, INPUT_PULLUP);
  pinMode(SW_C, INPUT_PULLUP);
  pinMode(SW_R, INPUT_PULLUP);

  ledcAttach(BUZZER, 440, 10);
  ledcWrite(BUZZER, 1024);

  pinMode(SLED_FR, OUTPUT);
  pinMode(SLED_FL, OUTPUT);
  pinMode(SLED_R, OUTPUT);
  pinMode(SLED_L, OUTPUT);
  digitalWrite(SLED_FR, LOW);
  digitalWrite(SLED_FL, LOW);
  digitalWrite(SLED_R, LOW);
  digitalWrite(SLED_L, LOW);

  pinMode(MOTOR_EN, OUTPUT);
  digitalWrite(MOTOR_EN, HIGH);
#ifndef PCC4
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);
  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(PWM_L, LOW);
#endif

  g_timer0 = timerBegin(1000000);
  timerAttachInterrupt(g_timer0, &onTimer0);
  timerAlarm(g_timer0, 1000, true, 0);
  timerStart(g_timer0);

  g_timer1 = timerBegin(1000000);
  timerAttachInterrupt(g_timer1, &onTimer1);
  timerAlarm(g_timer1, 500, true, 0);
  timerStart(g_timer1);

#ifndef PCC4
  g_timer2 = timerBegin(2000000);
  timerAttachInterrupt(g_timer2, &isrR);
  timerAlarm(g_timer2, 13333, true, 0);
  timerStart(g_timer2);

  g_timer3 = timerBegin(2000000);
  timerAttachInterrupt(g_timer3, &isrL);
  timerAlarm(g_timer3, 13333, true, 0);
  timerStart(g_timer3);
#endif

  Serial.begin(115200);
}

//LED
void ledSet(unsigned char data)
{
  digitalWrite(LED0, data & 0x01);
  digitalWrite(LED1, (data >> 1) & 0x01);
  digitalWrite(LED2, (data >> 2) & 0x01);
  digitalWrite(LED3, (data >> 3) & 0x01);
}

void bledSet(char data)
{
  if (data & 0x01) {
    digitalWrite(BLED0, HIGH);
  } else {
    digitalWrite(BLED0, LOW);
  }
  if (data & 0x02) {
    digitalWrite(BLED1, HIGH);
  } else {
    digitalWrite(BLED1, LOW);
  }
}

//Buzzer
void buzzerEnable(short f) { ledcWriteTone(BUZZER, f); }

void buzzerDisable(void)
{
  ledcWrite(BUZZER, 1024);  //duty 100% Buzzer OFF
}

//motor
void motorEnable(void)
{
#ifdef PCC4
  digitalWrite(MOTOR_EN, LOW);  //Power ON
#else
  digitalWrite(MOTOR_EN, HIGH);  //Power ON
#endif
}
void motorDisable(void)
{
#ifdef PCC4
  digitalWrite(MOTOR_EN, HIGH);  //Power OFF
#else
  digitalWrite(MOTOR_EN, LOW);   //Power OFF
#endif
}

#ifndef PCC4
void motorDirectionSet(t_CW_CCW left_CW, t_CW_CCW right_CW)
{  //左右のモータの回転方向を指示する
  if (right_CW == MOT_FORWARD) {
    digitalWrite(CW_R, LOW);
  } else {
    digitalWrite(CW_R, HIGH);  }

  if (left_CW == MOT_FORWARD) {
    digitalWrite(CW_L, LOW);
  } else {
    digitalWrite(CW_L, HIGH);
  }
}
#endif

//SWITCH
unsigned char switchGet(void)
{
  unsigned char ret = 0;
  if (digitalRead(SW_R) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_R) == LOW);
    ret |= SW_RM;
  }
  if (digitalRead(SW_C) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_C) == LOW);
    ret |= SW_CM;
  }
  if (digitalRead(SW_L) == LOW) {
    do {
      delay(20);
    } while (digitalRead(SW_L) == LOW);
    ret |= SW_LM;
  }
  return ret;
}

//sensor
unsigned short sensorGetR(void)
{
  digitalWrite(SLED_R, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD3);
  digitalWrite(SLED_R, LOW);
  return tmp;
}
unsigned short sensorGetL(void)
{
  digitalWrite(SLED_L, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD4);
  digitalWrite(SLED_L, LOW);
  return tmp;
}
unsigned short sensorGetFL(void)
{
  digitalWrite(SLED_FL, HIGH);  //LED点灯
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD2);
  digitalWrite(SLED_FL, LOW);  //LED消灯
  return tmp;
}
unsigned short sensorGetFR(void)
{
  digitalWrite(SLED_FR, HIGH);
  for (int i = 0; i < WAITLOOP_SLED; i++) {
    asm("nop \n");
  }
  unsigned short tmp = analogRead(AD1);
  digitalWrite(SLED_FR, LOW);
  return tmp;
}
short batteryVoltGet(void)
{
  return (double)analogReadMilliVolts(AD0) / 10.0 * (10.0 + 51.0) * 1.01;  //1.01は補正値;
}
