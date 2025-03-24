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

#include "fast.h"

FAST g_fast;

void FAST::run(short gx, short gy)
{
  t_global_direction glob_nextdir;
  int straight_count = 0;
  int i=0;

  //RVizに表示するマーカーの座標の初期化
  g_start_x = g_map.mypos.x;
  g_start_y = g_map.mypos.y;
  g_start_dir = g_map.mypos.dir;

  //マーカー用のデータを作成
  t_local_direction temp_next_dir = g_map.nextDir2Get(gx, gy, &glob_nextdir);
  g_map.mypos.dir = glob_nextdir;
  g_map.axisUpdate();
  while ((g_map.mypos.x != gx) || (g_map.mypos.y != gy)) {
    switch (g_map.nextDir2Get(gx, gy, &glob_nextdir)) {
      case front:
        straight_count++;
        break;
      case right:
        second_run_pattern[i++] = straight_count;
        second_run_pattern[i++] = R90;
        straight_count = 0;
        break;
      case left:
        second_run_pattern[i++] = straight_count;
        second_run_pattern[i++] = L90;
        straight_count = 0;
        break;
      default:
        break;
    }
    g_map.mypos.dir = glob_nextdir;
    g_map.axisUpdate();
  }

  second_run_pattern[i++] = straight_count;
  second_run_pattern[i++] = 127;

  g_map.mypos.dir = g_start_dir;

  //second_runにあるデータに沿って走行する。
  switch (temp_next_dir) {
    case right:
      g_run.rotate(right, 1);  //右に曲がって
      g_map.nextDir(right);  //RVizで向きを揃えるため方向をセットする必要がある
      break;
    case left:
      g_run.rotate(left, 1);  //左に曲がって
      g_map.nextDir(left);  //RVizで向きを揃えるため方向をセットする必要がある
      break;
    case rear:
      g_run.rotate(right, 2);  //180度に旋回して
      g_map.nextDir(right);  //RVizで向きを揃えるため方向をセットする必要がある
      g_map.nextDir(right);
      break;
    default:
      break;
  }

  g_fast_task = true;  //rvizに最短経路のマーカーを表示
  delay(10);

  g_run.accelerate(HALF_SECTION, g_run.search_speed);
  i = 0;
  while (1) {
    if (second_run_pattern[i] > 0) {
      g_run.straight(
        second_run_pattern[i] * SECTION, g_run.search_speed, g_run.max_speed, g_run.search_speed);
    }
    i++;
    if (second_run_pattern[i] == 127) {
      break;
    } else if (second_run_pattern[i] == R90) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_map.nextDir(right);
      g_run.rotate(right, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    } else if (second_run_pattern[i] == L90) {
      g_run.decelerate(HALF_SECTION, g_run.search_speed);
      g_map.nextDir(left);
      g_run.rotate(left, 1);
      g_run.accelerate(HALF_SECTION, g_run.search_speed);
    }
    i++;
  }
  g_run.decelerate(HALF_SECTION, g_run.search_speed);
}
