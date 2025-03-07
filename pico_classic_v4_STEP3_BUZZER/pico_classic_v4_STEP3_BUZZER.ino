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
#include <std_srvs/srv/trigger.h>
// clang-format off

#define PCC4

std_srvs__srv__Trigger_Response g_res;
std_srvs__srv__Trigger_Request g_req;
const int capacity = 32;
char data[capacity] = "success";

rclc_executor_t g_executor;
rclc_support_t g_support;
rcl_allocator_t g_allocator;
rcl_node_t g_node;
rcl_service_t g_service;

#define BUZZER 38
#define FREQ_C 523  //ド
#define FREQ_D 587  //レ
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

void service_callback(const void * reqin, void * resin){
  static bool result = false;
  std_srvs__srv__Trigger_Request *req1 = (std_srvs__srv__Trigger_Request *)reqin;
  std_srvs__srv__Trigger_Response *res1=(std_srvs__srv__Trigger_Response *)resin;
  res1->success = !result;
  if (result == true) { ledcWriteTone(BUZZER, FREQ_C);
  } else {              ledcWriteTone(BUZZER, FREQ_D);
  }
  delay(1000);
  ledcWrite(BUZZER, 1024);
  result = res1->success;
  res1->message.capacity = capacity;
  res1->message.size = strlen(data);
  res1->message.data = data;
}


void setup() {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);

  pinMode(SW_L, INPUT_PULLUP);
  pinMode(SW_C, INPUT_PULLUP);
  pinMode(SW_R, INPUT_PULLUP);

  ledcAttach(BUZZER, 440, 10);
  ledcWrite(BUZZER, 1024);

  digitalWrite(LED1, HIGH);
  set_microros_wifi_transports("使用するWiFiのAP名", "Wi-Fiのパスワード", "PCのIPアドレス", 8888);
  digitalWrite(LED2, HIGH);

  delay(2000);

  g_allocator = rcl_get_default_allocator();

 //create init_options
  RCCHECK(rclc_support_init(&g_support, 0, NULL, &g_allocator));

 // create node
  RCCHECK(rclc_node_init_default(&g_node, "micro_ros_arduino_node", "", &g_support));
 // create service
  RCCHECK(rclc_service_init_default(
    &g_service,
    &g_node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
   "micro_ros_arduino_service"));
// create executor
  RCCHECK(rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator));
  RCCHECK(rclc_executor_add_service(&g_executor, &g_service, &g_req, &g_res, service_callback));
}

void loop()
{
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&g_executor,RCL_MS_TO_NS(100)));
}
