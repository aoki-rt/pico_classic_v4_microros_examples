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
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <stdio.h>
#include <std_msgs/msg/int32.h>
// clang-format off

std_msgs__msg__Int32 g_msg;

rcl_publisher_t g_publisher;
rclc_support_t g_support;
rcl_allocator_t g_allocator;
rcl_node_t g_node;

#define PCC4

#ifdef PCC4
#define LED0 13
#define LED1 14
#define LED2 47
#define LED3 48
#define SW_L 16
#define SW_C 15
#define SW_R 18
#else
#define LED0 1
#define LED1 2
#define LED2 42
#define LED3 41
#define SW_L 10
#define SW_C 11
#define SW_R 12
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED0, !digitalRead(LED0));
    delay(100);
  }
}

void setup() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW_L, INPUT_PULLUP);
  pinMode(SW_C, INPUT_PULLUP);
  pinMode(SW_R, INPUT_PULLUP);

  digitalWrite(LED1, HIGH);
  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);
  digitalWrite(LED2, HIGH);

  delay(2000);

  g_allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

  // create g_node
  RCCHECK(rclc_node_init_default(&g_node, "micro_ros_pico_node", "", &g_support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &g_publisher,
    &g_node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "topic_name"));

}

void loop() {
    RCSOFTCHECK(rcl_publish(&g_publisher, &g_msg, NULL));
    g_msg.data=digitalRead(SW_L)&digitalRead(SW_C)&digitalRead(SW_R);
}
