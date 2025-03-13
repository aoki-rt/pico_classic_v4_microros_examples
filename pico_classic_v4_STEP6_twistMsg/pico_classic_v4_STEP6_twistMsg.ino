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

// clang-format off
// メッセージヘッダーファイルを見つけるため、micro_ros_arduino.hを先にインクルードすること
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include "run.h"
#include "SPI.h"
#include <stdio.h>
#include "TMC5240.h"
// clang-format on

//Pi:co Classic3で使用する時は、#define PCC4をコメントアウトする
//#define PCC4

geometry_msgs__msg__Twist g_msg;

rcl_subscription_t g_subscriber;
rclc_executor_t g_executor;
rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;



#define MIN_SPEED 30
#define TIRE_DIAMETER (48.00)
#define TREAD_WIDTH (65.0)

#ifdef PCC4
#define LED0 13
#define LED1 14
#define LED2 47
#define LED3 48
#define MOTOR_EN 17
#define SPI_CLK 39
#define SPI_MOSI 42
#define SPI_MISO 41
#define SPI_CS_L 40   //左モータ
#define SPI_CS_R 3   //右モータ
#define SPI_CS_J 46  //ジャイロ
#define PULSE TMC5240_PULSE
#else
#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41
#define MOTOR_EN 9
#define CW_R 14
#define CW_L 21
#define PWM_R 13
#define PWM_L 45
#define PULSE (TIRE_DIAMETER * PI / 400.0)
#endif

hw_timer_t * g_timer0 = NULL;
hw_timer_t * g_timer2 = NULL;
hw_timer_t * g_timer3 = NULL;

portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void errorLoop()
{
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

//割り込み
//目標値の更新周期1kHz
void IRAM_ATTR onTimer0(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  g_run.interrupt();
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}

#ifndef PCC4 
//Rモータの周期数割り込み
void IRAM_ATTR isrR(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  if (g_run.motor_move) {
    timerAlarm(g_timer2, 2000000 / g_run.step_hz_r, true, 0);
    digitalWrite(PWM_R, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    }
    digitalWrite(PWM_R, LOW);
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}

//Lモータの周期数割り込み
void IRAM_ATTR isrL(void)
{
  portENTER_CRITICAL_ISR(&g_timer_mux);  //割り込み禁止
  if (g_run.motor_move) {
    timerAlarm(g_timer3, 2000000 / g_run.step_hz_l, true, 0);
    digitalWrite(PWM_L, HIGH);
    for (int i = 0; i < 100; i++) {
      asm("nop \n");
    };
    digitalWrite(PWM_L, LOW);
  }
  portEXIT_CRITICAL_ISR(&g_timer_mux);  //割り込み許可
}
#endif

//twist message cb
void subscriptionCallback(const void * msgin)
{
  const geometry_msgs__msg__Twist * g_msg = (const geometry_msgs__msg__Twist *)msgin;

  g_run.speed = g_msg->linear.x * 1000.0;
  g_run.omega = g_msg->angular.z;
}

void setup()
{
  //  set_microros_transports();
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  //motor disable
  pinMode(MOTOR_EN, OUTPUT);


#ifdef PCC4 
  digitalWrite(MOTOR_EN, HIGH);
  g_tmc5240.init();
#else
  digitalWrite(MOTOR_EN, LOW);
  pinMode(CW_R, OUTPUT);
  pinMode(CW_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  digitalWrite(CW_R, LOW);
  digitalWrite(CW_L, LOW);
  digitalWrite(PWM_R, LOW);
  digitalWrite(PWM_L, LOW);
#endif
  Serial.begin(115200);

  digitalWrite(LED1, HIGH);
  set_microros_wifi_transports("JCOM_HOSI", "890927221266", "192.168.40.250", 8888);
//  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);
  digitalWrite(LED2, HIGH);

  delay(2000);

  g_timer0 = timerBegin(1000000);  //1MHz(1us)
  timerAttachInterrupt(g_timer0, &onTimer0);
  timerAlarm(g_timer0, 1000, true, 0);  //1000 * 1us = 1000us(1kHz)
  timerStart(g_timer0);

#ifndef PCC4 
  g_timer2 = timerBegin(2000000);  //2MHz(0.5us)
  timerAttachInterrupt(g_timer2, &isrR);
  timerAlarm(g_timer2, 13333, true, 0);  //13333 * 0.5us = 6666us(150Hz)
  timerStart(g_timer2);

  g_timer3 = timerBegin(2000000);  //2MHz(0.5us)
  timerAttachInterrupt(g_timer3, &isrL);
  timerAlarm(g_timer3, 13333, true, 0);  //13333 * 0.5us = 6666us(150Hz)
  timerStart(g_timer3);
#endif

  g_allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

  // create g_node
  RCCHECK(rclc_node_init_default(&g_node, "micro_ros_pico_node", "", &g_support));

  // create g_subscriber
  RCCHECK(rclc_subscription_init_default(
    &g_subscriber, &g_node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));

  // create g_executor
  RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));
  RCCHECK(rclc_executor_add_subscription(
    &g_executor, &g_subscriber, &g_msg, &subscriptionCallback, ON_NEW_DATA));

#ifdef PCC4 
  digitalWrite(MOTOR_EN, LOW);
#else
  digitalWrite(MOTOR_EN, HIGH);
#endif
}

void loop()
{
  delay(10);
  RCCHECK(rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100)));
  Serial.printf("%f %f\n\r",g_run.speed_l,g_run.speed_r);
}
