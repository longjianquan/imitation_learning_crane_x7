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

// ソケット通信
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

// static int connect_mode = 0;

static double ts = 0.002;
// スレーブスレッドのループ回数（=データの管理番号）
int ttt = 0;
static char ch = 'p';
static double passtime = 0.0;

const char *devicename1 = "/dev/ttyUSB0";  //こっちがスレーブ
const char *devicename2 = "/dev/ttyUSB1";  //こっちがマスター

using namespace std;

CR7 crane_s(devicename1, SLAVE);
CR7 crane_m(devicename2, MASTER);

void *crane_s_control(void *) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  bool finishFlag = false;

  /* --------- CR7 の型でcrane_sを定義 */
  if (!crane_s.Set_port_baudrate()) {     // 通信レートの設定
    crane_s.Close_port();                 // 通信ポートを閉じる
    return NULL;
  }

  // position control mode
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  // move to goal pose
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s.Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // 初期位置を設定
  for (int i = 0; i < JOINT_NUM2; i++) {
    crane_s.theta_ref[i] = goal_pose[i];
    crane_s.omega_ref[i] = 0.0;
    crane_s.tau_ref[i] = 0.0;
  }

  // current control mode
  crane_s.Disable_Dynamixel_Torque(ID);
  // crane_s.position_control(goal_pose);
  crane_s.Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  printf("========== slave position control start ==========\n");

  // position observation
  crane_s.Readtheta_res(ID);
  for (int j = 0; j < JOINT_NUM2; j++) {
    crane_s.d_theta_temp[j] = crane_s.theta_res[j];
  }

  /*********************************************************
                         P MODE
  **********************************************************/
  while (ch == 'p') {
    gettimeofday(&start_time_s, NULL);

    // position observation
    crane_s.Readtheta_res(ID);

    if ((crane_s.theta_res[0] == 0.0) || (crane_s.theta_res[7] == 0.0)) {
      printf("crane_s読み込み怪しいので終了\n");
      break;
    }

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s.omega_res[i] =
          (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
      crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
    }

    // speed limit
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
        printf("slaveの軸%dが速いので終了: %lf\n", i, crane_s.omega_res[i]);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // calculate input torque
    crane_s.position_control(goal_pose);

    // 秒単位の時間を取得
    gettimeofday(&end_time_s, NULL);
    // (終了時間 - 開始時間) + (終了時間 - 開始時間) * 0.000,001
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    // スリープ時間 = ループ周期(20[ms]) - 制御時間 * 1,000,000.0
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
    // スリープ時間が0より下なら 0 にリセット
    if (sleep_time_s < 0) sleep_time_s = 0;

    // write current state to csv
    crane_s.write_csv(passtime, sleep_time_s, control_time_s);

    usleep(sleep_time_s);
  }

  if (ch != 'b') {
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
    crane_s.Enable_Dynamixel_Torque(ID);
    crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    printf("crane_s_バイラテ前にqで終了\n");
    sleep(5);
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Close_port();
    fclose(crane_s.ffp);
    return NULL;
  }

  /*********************************************************
                        B MODE
  **********************************************************/
  printf("========== slave bilateral control start==========\n");
  while (ch == 'b') {  //データ取得の開始
    gettimeofday(&start_time_s, NULL);

    // crane_s.datareadflag = 0;

    // position observation
    crane_s.Readtheta_res(ID);

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s.omega_res[i] =
          (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
      crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
    }

    // speed limit

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
        printf("crane_sの軸%dが速いので終了\n", i);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // calculate input torque
    crane_s.torque_control(crane_m.theta_res, crane_m.omega_res,
                           crane_m.tau_res);

    if ((ttt % 10) == 0) printf("time: %lf\n", passtime);

    /**********************************************************
      処理時間とループ時間からスリープ時間を割り出す(Bモード)
    ***********************************************************/
    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    // スリープ時間 = ループ周期(2000[us]=2[ms]) - 制御時間 * 1,000,000.0
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);

    if (sleep_time_s < 0) sleep_time_s = 0;

    // write current state to csv
    crane_s.write_csv(passtime, sleep_time_s, control_time_s);

    usleep(sleep_time_s);
    // ts = 0.002 [sec] = 2[ms]
    passtime += ts;
    ttt++;
  }

  // poosition control mode
  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  // move to finish pose
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Close_port();
  fclose(crane_s.ffp);
  return NULL;
}

void *master_control(void *) {
  double control_time_m;
  long sleep_time_m;
  struct timeval start_time_m;
  struct timeval end_time_m;
  bool finishFlag = false;

  // COMポートを開く
  if (!crane_m.Open_port()) return NULL;
  //通信レートの設定
  if (!crane_m.Set_port_baudrate()) {
    //通信ポートを閉じる
    crane_m.Close_port();
    return NULL;
  }

  // position control mode
  crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_m.Enable_Dynamixel_Torque(ID);

  // move to goal pose
  crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_m.Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // 初期位置を設定
  for (int i = 0; i < JOINT_NUM2; i++) {
    crane_m.theta_ref[i] = goal_pose[i];
    crane_m.omega_ref[i] = 0.0;
    crane_m.tau_ref[i] = 0.0;
  }

  // current control mode
  crane_m.Disable_Dynamixel_Torque(ID);
  // crane_m.position_control(goal_pose);
  crane_m.Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_m.Enable_Dynamixel_Torque(ID);

  printf("Press b  to start (or press q to quit)\n");
  printf("========== master position control start==========\n");

  // position observation
  crane_m.Readtheta_res(ID);
  for (int j = 0; j < JOINT_NUM2; j++)
    crane_m.d_theta_temp[j] = crane_m.theta_res[j];

  /*********************************************************
            P MODE
  **********************************************************/
  while (ch == 'p') {
    gettimeofday(&start_time_m, NULL);

    // position observation
    crane_m.Readtheta_res(ID);

    if ((crane_m.theta_res[0] == 0.0) || (crane_m.theta_res[7] == 0.0)) {
      printf("master読み込み怪しいので終了\n");
      break;
    }

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_m.omega_res[i] =
          (crane_m.theta_res[i] - crane_m.d_theta_temp[i]) * g[i];
      crane_m.d_theta_temp[i] += crane_m.omega_res[i] * ts;
    }

    // speed limit
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_m.omega_res[i]) >= LIMIT_SPEED[i]) {
        printf("masterの軸%dが速いので終了\n", i);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // calculate input torque
    crane_m.position_control(goal_pose);

    /**********************************************************
      処理時間とループ時間からスリープ時間を割り出す(Pモード)
    ***********************************************************/
    gettimeofday(&end_time_m, NULL);
    control_time_m = (end_time_m.tv_sec - start_time_m.tv_sec +
                      (end_time_m.tv_usec - start_time_m.tv_usec) * 0.000001);
    sleep_time_m = LOOPTIME - (long)(control_time_m * 1000000.0);

    if (sleep_time_m < 0) sleep_time_m = 0;

    // write current state to csv file
    crane_m.write_csv(passtime, sleep_time_m, control_time_m);

    usleep(sleep_time_m);
  }

  if (ch != 'b') {
    crane_m.Disable_Dynamixel_Torque(ID);
    crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
    crane_m.Enable_Dynamixel_Torque(ID);
    crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    printf("master_バイラテ前にqで終了\n");
    crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crane_m.Disable_Dynamixel_Torque(ID);
    crane_m.Close_port();
    fclose(crane_m.ffp);
    return NULL;
  }

  /*********************************************************
            B MODE (ここからがバイラテ)
  **********************************************************/
  printf("================= master bilateral control start==================\n");
  while (ch == 'b') {
    gettimeofday(&start_time_m, NULL);

    // crane_m.datareadflag = 0;

    // position observation
    crane_m.Readtheta_res(ID);

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_m.omega_res[i] =
          (crane_m.theta_res[i] - crane_m.d_theta_temp[i]) * g[i];
      crane_m.d_theta_temp[i] += crane_m.omega_res[i] * ts;
    }

    // speed limit
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_m.omega_res[i]) >= LIMIT_SPEED[i]) {
        printf("masterの軸%dが速いので終了\n", i);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // calculate input torque
    crane_m.torque_control(crane_m.theta_res, crane_m.omega_res,
                           crane_m.tau_res);

    /**********************************************************
      処理時間とループ時間からスリープ時間を割り出す(Bモード)
    ***********************************************************/
    gettimeofday(&end_time_m, NULL);
    control_time_m = (end_time_m.tv_sec - start_time_m.tv_sec +
                      (end_time_m.tv_usec - start_time_m.tv_usec) * 0.000001);
    sleep_time_m = LOOPTIME - (long)(control_time_m * 1000000.0);
    if (sleep_time_m < 0) {
      sleep_time_m = 0;
    }

    // write current state to csv file
    crane_m.write_csv(passtime, sleep_time_m, control_time_m);

    usleep(sleep_time_m);
  }

  // position control mode
  crane_m.Disable_Dynamixel_Torque(ID);
  crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_m.Enable_Dynamixel_Torque(ID);

  // move to finish pose
  crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  crane_m.Disable_Dynamixel_Torque(ID);
  crane_m.Close_port();
  fclose(crane_m.ffp);

  return NULL;
}

static char pre_ch = 'p';

void *keyboard_check(void *) {
  char key;

  while (ch != 'q') {
    key = getch();
    /************************
     *     B MODE
     *************************/
    if (key == 'b') {
      ch = 'b';
      // pからbに移行した時だけカメラカウントをリセット
      if (pre_ch == 'p') {
        printf("MODE B ACTIVE\n");
        // t_camera = 0;
        // camera_active_flag = true;
      }
    }
    /************************
     *     Q MODE
     *************************/
    else if (key == 'q') {
      ch = 'q';
      printf("MODE Q ACTIVE\n");
      // dprintf(sock, "%s", "**");
      break;
    }
    pre_ch = ch;
  }
  return NULL;
}

/**
 * @fn		main()
 * @brief	main
 */
int main() {
  pthread_t master_thread, crane_s_thread, getch_thread;

  // マスター制御のスレッド設定
  if (pthread_create(&master_thread, NULL, &master_control, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }
  // スレーブ制御のスレッド設定
  if (pthread_create(&crane_s_thread, NULL, &crane_s_control, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }
  // キーボード入力監視のスレッド設定
  if (pthread_create(&getch_thread, NULL, &keyboard_check, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }
  // カメラ制御のスレッド設定
  /*if (pthread_create(&camera_thread, NULL, &camera_control, NULL) != 0)
  {
      fprintf(stderr, "cannot create control thread\n");
      return 1;
  }*/
  // スレッド開始
  pthread_join(master_thread, NULL);
  pthread_join(crane_s_thread, NULL);
  pthread_join(getch_thread, NULL);
  // pthread_join(camera_thread, NULL);
  return 0;
}
