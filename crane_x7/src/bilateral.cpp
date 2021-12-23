#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <iostream>
#include <string>

#include "crane.h"
#include "crane_x7_comm.h"
#include "params.h"

using namespace std;

static char ch = 'p';

const char *devicename1 = "/dev/ttyUSB0";  // slave
const char *devicename2 = "/dev/ttyUSB1";  // master
CR7 crane_s(devicename1, goal_pose, SLAVE);
CR7 crane_m(devicename2, goal_pose, MASTER);

void *bilateral_control(CR7 *crane_s, CR7 *crane_m, bool slave) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  bool finishFlag = false;
  int count = 0;
  static double ts = 0.002;
  static double time = 0.0;

  if (!crane_s->Set_port_baudrate()) {  // 通信レートの設定
    crane_s->Close_port();              // 通信ポートを閉じる
    return NULL;
  }

  // position control mode
  crane_s->Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s->Enable_Dynamixel_Torque(ID);

  // move to initial pose
  crane_s->Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s->Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // current control mode
  crane_s->Disable_Dynamixel_Torque(ID);
  crane_s->Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_s->Enable_Dynamixel_Torque(ID);

  // initialize
  crane_s->Readpresent_position(ID);
  for (int j = 0; j < JOINT_NUM2; j++)
    crane_s->d_theta_temp[j] = crane_s->theta_res[j];

  while (true) {
    gettimeofday(&start_time_s, NULL);

    // position observation
    crane_s->Readpresent_position(ID);

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s->omega_res[i] =
          (crane_s->theta_res[i] - crane_s->d_theta_temp[i]) * g[i];
      crane_s->d_theta_temp[i] += crane_s->omega_res[i] * ts;
    }

    // forced termination
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (crane_s->theta_res[i] == 0.0) {
        cout << (slave ? "slave" : "master");
        cout << "読み込み怪しいので終了" << endl;
        finishFlag = true;
      }

      if (fabs(crane_s->omega_res[i]) >= LIMIT_SPEED[i]) {
        cout << (slave ? "slave" : "master");
        printf("の軸%dが速いので終了: %lf [rad/s]\n", i, crane_s->omega_res[i]);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // move to initial pose
    if (ch == 'p') {
      crane_s->position_control(goal_pose);

      // bilateral control
    } else if (ch == 'b') {
      crane_s->force_control(crane_m->theta_res, crane_m->omega_res,
                             crane_m->tau_res);

      // finish
    } else {
      break;
    }

    // print
    if (slave && (count % 10) == 0) printf("mode: %c  time: %lf\n", ch, time);

    // calculate sleep time
    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
    if (sleep_time_s < 0) sleep_time_s = 0;

    // write current state to csv
    crane_s->write_csv(time, sleep_time_s, control_time_s);

    usleep(sleep_time_s);

    // time count
    if (ch == 'b') {
      time += ts;
      count++;
    }
  }

  // position control mode
  crane_s->Disable_Dynamixel_Torque(ID);
  crane_s->Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s->Enable_Dynamixel_Torque(ID);

  // move to finish pose
  crane_s->Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s->Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  crane_s->Disable_Dynamixel_Torque(ID);
  crane_s->Close_port();
  fclose(crane_s->ffp);
  return NULL;
}

void *slave_control(void *) {
  return bilateral_control(&crane_s, &crane_m, true);
}

void *master_control(void *) {
  return bilateral_control(&crane_m, &crane_s, false);
}

void *keyboard_check(void *) {
  char key;

  while (ch != 'q') {
    key = getch();

    // P MODE
    if (key == 'p') {
      ch = 'p';
      printf("MODE P ACTIVE\n");
    }

    // B MODE
    if (key == 'b') {
      ch = 'b';
      printf("MODE B ACTIVE\n");
    }

    // Q MODE
    else if (key == 'q') {
      ch = 'q';
      printf("MODE Q ACTIVE\n");
      break;
    }
  }
  return NULL;
}

/**
 * @fn		main()
 * @brief	main
 */
int main() {
  pthread_t master_thread, slave_thread, getch_thread;

  // マスター制御のスレッド設定
  if (pthread_create(&master_thread, NULL, &master_control, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }
  // スレーブ制御のスレッド設定
  if (pthread_create(&slave_thread, NULL, &slave_control, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }
  // キーボード入力監視のスレッド設定
  if (pthread_create(&getch_thread, NULL, &keyboard_check, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }

  // スレッド開始
  pthread_join(master_thread, NULL);
  pthread_join(slave_thread, NULL);
  pthread_join(getch_thread, NULL);

  return 0;
}
