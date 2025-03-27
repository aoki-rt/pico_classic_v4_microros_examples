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
#include "parameter.h"
#include "run.h"

RUN g_run;

RUN::RUN()
{
  speed = 0.0;
  accel = 0.0;
}

//割り込み
void controlInterrupt(void) { g_run.interrupt(); }

void RUN::interrupt(void)
{                            //割り込み内からコール
  static char temp_cnt = 0;  //オドメトリで角度がずれるので直進を走っている時間を計測して補正する

  speed += accel;

  if (speed > upper_speed_limit) {
    speed = upper_speed_limit;
  }
  if (speed < lower_speed_limit) {
    speed = lower_speed_limit;
  }

  if (con_wall.enable == true) {
    con_wall.p_error = con_wall.error;
    if ((g_sensor.sen_r.is_control == true) && (g_sensor.sen_l.is_control == true)) {
      con_wall.error = g_sensor.sen_r.error - g_sensor.sen_l.error;
    } else {
      con_wall.error = 2.0 * (g_sensor.sen_r.error - g_sensor.sen_l.error);
    }
    con_wall.diff = con_wall.error - con_wall.p_error;
    con_wall.sum += con_wall.error;
    if (con_wall.sum > con_wall.sum_max) {
      con_wall.sum = con_wall.sum_max;
    } else if (con_wall.sum < (-con_wall.sum_max)) {
      con_wall.sum = -con_wall.sum_max;
    }
    con_wall.control = 0.001 * speed * con_wall.kp * con_wall.error;
    speed_target_r = speed + con_wall.control;
    speed_target_l = speed - con_wall.control;
  } else {
    speed_target_r = speed;
    speed_target_l = speed;
  }

  if (motorMoveGet()) {
    if (speed_target_r < MIN_SPEED) speed_target_r = MIN_SPEED;
    if (speed_target_l < MIN_SPEED) speed_target_l = MIN_SPEED;
  } else {
    speed_target_r = speed_target_l = 0;
  }

  //odom
  //xは方向
  double speed2 = (speed_target_r * motor_signed_r + speed_target_l * motor_signed_l) / 2.0;
  double omega =
    (speed_target_r * motor_signed_r - speed_target_l * motor_signed_l) / tread_width * 1.00;
  g_odom_x += speed2 * 0.001 * cos(g_odom_theta) * 0.001;
  g_odom_y += speed2 * 0.001 * sin(g_odom_theta) * 0.001;
  g_odom_theta += omega * 0.001;

  //角度、位置のズレを修正
  if (
    ((g_sensor.sen_r.value < (g_sensor.sen_r.ref + 20)) ||
     (g_sensor.sen_l.value < (g_sensor.sen_l.ref + 20))) &&
    ((g_sensor.sen_r.value > (g_sensor.sen_r.ref - 20)) ||
     (g_sensor.sen_l.value > (g_sensor.sen_l.ref - 20)))) {
    temp_cnt++;
    if (temp_cnt > 5) {
      switch (g_map.mypos.dir) {
        case north:
          if (g_theta_adj == true) {
            g_odom_theta = 0.0;
          }
          break;
        case east:
          if (g_theta_adj == true) {
            g_odom_theta = -1.57;
          }
          break;
        case south:
          if (g_theta_adj == true) {
            g_odom_theta = 3.14;
          }
          break;
        case west:
          if (g_theta_adj == true) {
            g_odom_theta = 1.57;
          }
          break;
      }
      temp_cnt = 0;
    }
  } else {
    temp_cnt = 0;
  }

  g_position_r += speed_target_r * 0.001 * motor_signed_r / (tire_diameter * PI) * 2 * PI;
  g_position_l -= speed_target_l * 0.001 * motor_signed_l / (tire_diameter * PI) * 2 * PI;
}

void RUN::dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right)
{
  if (dir_right == MOT_FORWARD) {
    motor_signed_r = 1.0;
  } else {
    motor_signed_r = -1.0;
  }
  if (dir_left == MOT_FORWARD) {
    motor_signed_l = 1.0;
  } else {
    motor_signed_l = -1.0;
  }

#ifdef PCC4
  g_tmc5240.write(TMC5240_RAMPMODE, dir_left, dir_right);
#else
  motorDirectionSet(dir_left, dir_right);
#endif
}

void RUN::counterClear(void)
{
#ifdef PCC4
  g_tmc5240.write(TMC5240_XACTUAL, 0, 0);
#else
  stepClearR();
  stepClearL();
#endif
}

void RUN::speedSet(double l_speed, double r_speed)
{
#ifdef PCC4
  g_tmc5240.write(
    TMC5240_VMAX, (unsigned int)(l_speed / (pulse * 0.787)),
    (unsigned int)(r_speed / (pulse * 0.787)));
#else
  stepHzSetR(r_speed / pulse);
  stepHzSetL(l_speed / pulse);
#endif
}

void RUN::stay(float l_speed)
{
  controlInterruptStop();
  upper_speed_limit = lower_speed_limit = speed = (double)l_speed;
  accel = 0.0;
  counterClear();
  speedSet(l_speed, l_speed);
  controlInterruptStart();
}

void RUN::stepGet(void)
{
#ifdef PCC4
  step_lr = g_tmc5240.readXactual();
#else
  step_lr = stepGetR() + stepGetL();
#endif
  step_lr_len = (int)((float)step_lr / 2.0 * pulse);
}

void RUN::stop(void)
{
  motorMoveSet(0);
#ifdef PCC4
  g_tmc5240.write(TMC5240_VMAX, 0, 0);
#endif
  delay(300);
}

void RUN::straight(int len, int init_speed, int max_sp, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  upper_speed_limit = (double)max_sp;
  accel = (double)search_accel;

  if (init_speed < MIN_SPEED) {
    speed = (double)MIN_SPEED;
  } else {
    speed = (double)init_speed;
  }
  if (finish_speed < MIN_SPEED) {
    finish_speed = MIN_SPEED;
  }
  lower_speed_limit = (double)finish_speed;

  con_wall.enable = true;
  speedSet(speed, speed);
  dirSet(MOT_FORWARD, MOT_FORWARD);
  obj_step = (int)((float)len * 2.0 / pulse);
  if (len > HALF_SECTION) {
    g_theta_adj = true;
  }
  controlInterruptStart();
  motorMoveSet(1);

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (
      (int)(len - step_lr_len) <
      (int)(((speed * speed) - (lower_speed_limit * lower_speed_limit)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1.0 * search_accel;

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  g_theta_adj = false;

  if (finish_speed == MIN_SPEED) {
    stop();
  } else {
    stay(finish_speed);
  }
}

void RUN::accelerate(int len, int finish_speed)
{
  int obj_step;

  controlInterruptStop();
  accel = search_accel;
  speed = lower_speed_limit = (double)MIN_SPEED;
  upper_speed_limit = (double)finish_speed;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  counterClear();
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  motorMoveSet(1);

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  stay(finish_speed);
}

void RUN::oneStep(int len, int init_speed)
{
  int obj_step;

  controlInterruptStop();
  accel = 0.0;
  speed = lower_speed_limit = upper_speed_limit = (double)init_speed;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  obj_step = (int)((float)len * 2.0 / pulse);
  g_theta_adj = true;
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }

  stay(init_speed);

  controlInterruptStop();
  g_theta_adj = false;
  controlInterruptStart();
}

void RUN::decelerate(int len, int init_speed)
{
  int obj_step;

  controlInterruptStop();
  accel = search_accel;
  speed = upper_speed_limit = init_speed;
  lower_speed_limit = MIN_SPEED;
  con_wall.enable = true;
  dirSet(MOT_FORWARD, MOT_FORWARD);
  speedSet(speed, speed);
  obj_step = (int)((float)len * 2.0 / pulse);
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (
      (int)(len - step_lr_len) <
      (int)(((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1 * search_accel;

  while (1) {
    stepGet();
    speedSet(speed, speed);
    if (step_lr > obj_step) {
      break;
    }
  }

  stop();
}

void RUN::rotate(t_local_direction dir, int times)
{
  int obj_step;

  controlInterruptStop();
  accel = turn_accel;
  upper_speed_limit = search_speed;
  speed = lower_speed_limit = MIN_SPEED;
  con_wall.enable = false;
  obj_step = (int)(tread_width * PI / 4.0 * (float)times * 2.0 / pulse);
  switch (dir) {
    case right:
      dirSet(MOT_FORWARD, MOT_BACK);
      motorMoveSet(1);
      break;
    case left:
      dirSet(MOT_BACK, MOT_FORWARD);
      motorMoveSet(1);
      break;
    default:
      dirSet(MOT_FORWARD, MOT_FORWARD);
      motorMoveSet(0);
      break;
  }
  speedSet(MIN_SPEED, MIN_SPEED);
  counterClear();
  controlInterruptStart();

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (
      (int)((tread_width * PI / 4.0 * times) - step_lr_len) <
      (int)(((speed * speed) - (MIN_SPEED * MIN_SPEED)) / (2.0 * 1000.0 * accel))) {
      break;
    }
  }

  accel = -1.0 * turn_accel;

  while (1) {
    stepGet();
    speedSet(speed_target_l, speed_target_r);
    if (step_lr > obj_step) {
      break;
    }
  }
  stop();
}
