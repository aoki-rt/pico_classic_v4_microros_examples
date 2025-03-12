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
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <std_msgs/msg/int32.h>
// clang-format off

//Pi:co Classic3で使用する時は、#define PCC4をコメントアウトする
#define PCC4

std_msgs__msg__Int32 g_msg;

rcl_subscription_t g_subscriber;
rclc_executor_t g_executor;
rclc_support_t g_support;
rcl_allocator_t g_allocator;
rcl_node_t g_node;

#ifdef PCC4
#define LED0 13
#define LED1 14
#define LED2 47
#define LED3 48
#else
#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){errorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void errorLoop()
{
  while (1) {
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  digitalWrite(LED0, (((msg->data)&0x01) == 0) ? LOW : HIGH);  
  digitalWrite(LED1, (((msg->data)&0x02) == 0) ? LOW : HIGH);  
  digitalWrite(LED2, (((msg->data)&0x04) == 0) ? LOW : HIGH);  
  digitalWrite(LED3, (((msg->data)&0x08) == 0) ? LOW : HIGH);  
}

void setup()
{
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  digitalWrite(LED1, HIGH);
  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);
  digitalWrite(LED2, HIGH);

  delay(2000);

  g_allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

  // create g_node
  RCCHECK(rclc_node_init_default(&g_node, "micro_ros_pico_node", "", &g_support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &g_subscriber,
    &g_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // create executor
  RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));
  RCCHECK(rclc_executor_add_subscription(&g_executor, &g_subscriber, &g_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(10);
  RCCHECK(rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(100)));   
}
