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

//microROSを使用しないときはコメントアウトする
#define USE_MICRO_ROS

//Pi:co Classic3で使用する時は、#define PCC4をコメントアウトする
//#define PCC4

//マイクロマウスで使用するヘッダ
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <SPI.h>
#include <WiFi.h>

#include "SPIFFS.h"
#include "TMC5240.h"
#include "adjust.h"
#include "device.h"
#include "fast.h"
#include "map_manager.h"
#include "misc.h"
#include "parameter.h"
#include "run.h"
#include "search.h"
#include "sensor.h"

//micro-ROSで使用するヘッダ
// clang-format off
// メッセージヘッダーファイルを見つけるため、micro_ros_arduino.hを先にインクルードすること
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <pico_msgs/msg/light_sensor.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/int16.h>
#include <stdio.h>
#include <tf2_msgs/msg/tf_message.h>
#include <visualization_msgs/msg/marker.h>
// clang-format on



//micro-ROSで使用する変数
volatile double g_position_r, g_position_l;
volatile double g_odom_x, g_odom_y, g_odom_theta;
short g_publish_x, g_publish_y;
volatile bool g_theta_adj;

volatile bool g_fast_task;          //最短経路のマーカーを表示するときに使用
volatile int g_start_x, g_start_y;  //最短経路のマーカーの初期値で使用
volatile t_global_direction g_start_dir;  //最短経路のマーカーの初期値で使用

void setup()
{
  // put your setup code here, to run once:
  deviceInit();
  flashBegin();
  paramRead();
#ifdef PCC4
  g_tmc5240.init();
#endif

  motorDisable();
  buzzerEnable(INC_FREQ);
  delay(80);
  buzzerDisable();

  g_misc.mode_select = 1;

  g_fast_task = false;
  g_theta_adj = false;

#if defined(USE_MICRO_ROS)
  microROSInit();
#endif
}

void loop()
{
  // put your main code here, to run repeatedly:
  ledSet(g_misc.mode_select);
  switch (switchGet()) {
    case SW_RM:
      g_misc.mode_select = g_misc.buttonInc(g_misc.mode_select, 15, 1);
      break;
    case SW_LM:
      g_misc.mode_select = g_misc.buttonDec(g_misc.mode_select, 1, 15);
      break;
    case SW_CM:
      g_misc.buttonOk();
      g_misc.modeExec(g_misc.mode_select);
      break;
  }
  delay(1);
}
