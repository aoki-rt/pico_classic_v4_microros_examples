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

#include "TMC5240.h"
#include "run.h"

RUN g_run;

RUN::RUN()
{
  speed=0.0;
  speed_r=0.0;
  speed_l=0.0;
  motor_move = 0.0;
  omega = 0.0;
}



void RUN::dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right)
{
#ifdef PCC4
  g_tmc5240.write(TMC5240_RAMPMODE, dir_left, dir_right);
#else
  if (dir_left == MOT_FORWARD) {
    digitalWrite(CW_L, LOW);
  } else {
    digitalWrite(CW_L, HIGH);
  }
  if (dir_right == MOT_FORWARD) {
    digitalWrite(CW_R, LOW);
  } else {
    digitalWrite(CW_R, HIGH);
  }
#endif
}

void RUN::speedSet(double l_speed, double r_speed) {
  l_speed = fabs(l_speed);
  if(l_speed < MIN_SPEED)l_speed=MIN_SPEED;
  r_speed = fabs(r_speed);
  if(r_speed < MIN_SPEED)r_speed=MIN_SPEED;
#ifdef PCC4
  g_tmc5240.write(TMC5240_VMAX, (unsigned int)(l_speed / (PULSE * 0.787)), (unsigned int)(r_speed / (PULSE * 0.787)));
#else
  step_hz_r=(unsigned short)(r_speed/PULSE);
  step_hz_l=(unsigned short)(l_speed/PULSE);
#endif
}

void RUN::stop(void)
{
#ifdef PCC4
  g_tmc5240.write(TMC5240_VMAX, 0, 0);
#else
  motor_move = 0;
#endif
}

void RUN::interrupt(void)
{
  // 直進速度と回転速度から、左右のモータ速度を求める
  speed_r = speed + omega * TREAD_WIDTH / 2.0;
  speed_l = speed - omega * TREAD_WIDTH / 2.0;

  // 左右両方のモータ速度がMIN_SPEED以下の場合は走行を停止する
  if (fabs(speed_r) < MIN_SPEED && fabs(speed_l) < MIN_SPEED) {
    stop();
    return;
  }

  motor_move = 1;

  // モータ速度をMIN_SPEED以上に制限する
  if (fabs(speed_r) < MIN_SPEED) {
    speed_r = (speed_r > 0.0) ? MIN_SPEED : -1.0 * MIN_SPEED;
  }
  if (fabs(speed_l) < MIN_SPEED) {
    speed_l = (speed_l > 0.0) ? MIN_SPEED : -1.0 * MIN_SPEED;
  }

  // 制限されたモータ速度から、直進速度と回転速度を求める
  const double forward_speed = (speed_r + speed_l) / 2.0;
  const double omega = (speed_r - speed_l) / TREAD_WIDTH;

  const double UPDATE_INTERVAL = 0.001;
  g_odom_x += forward_speed * UPDATE_INTERVAL * cos(g_odom_theta) * UPDATE_INTERVAL;
  g_odom_y += forward_speed * UPDATE_INTERVAL * sin(g_odom_theta) * UPDATE_INTERVAL;
  g_odom_theta += omega * UPDATE_INTERVAL;
  g_position_r += speed_r * UPDATE_INTERVAL / (TIRE_DIAMETER * PI) * 2 * PI;
  g_position_l -= speed_l * UPDATE_INTERVAL / (TIRE_DIAMETER * PI) * 2 * PI;

  if ((speed_r > 0) && (speed_l > 0)) {
    dirSet(MOT_FORWARD, MOT_FORWARD);
  } else if ((speed_r < 0) && (speed_l > 0)) {
    dirSet(MOT_FORWARD, MOT_BACK);
  } else if ((speed_r > 0) && (speed_l < 0)) {
    dirSet(MOT_BACK, MOT_FORWARD);
  } else if ((speed_r < 0) && (speed_l < 0)) {
    dirSet(MOT_BACK, MOT_BACK);
  } else {
    stop();
  }
  speedSet(speed_l, speed_r);
}
