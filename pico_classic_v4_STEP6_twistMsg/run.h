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


#ifndef SRC_RUN_H_
#define SRC_RUN_H_

typedef enum {
  MOT_FORWARD = 1,  //TMC5240の方向に合わせた数字
  MOT_BACK = 2
} t_CW_CCW;

class RUN {
private:

public:
  volatile double speed;
  volatile double speed_r, speed_l;
  volatile bool motor_move;
  volatile double omega;
  volatile unsigned short step_hz_r;
  volatile unsigned short step_hz_l;

  RUN();
  void interrupt(void);
  void dirSet(t_CW_CCW dir_left, t_CW_CCW dir_right);
  void stop(void);
  void speedSet(double l_speed, double r_speed);

};


extern RUN g_run;

#endif /* SRC_RUN_H_ */