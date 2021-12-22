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

#include "crane.h"
#include "crane_x7_comm.h"
#include "params.h"

// socket communication
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

static double ts = 0.002;
int rnn_ts = 20;
static char ch = 'p';
static double passtime = 0.0;

// static double p_th_m_res[JOINT_NUM], p_dth_m_res[JOINT_NUM],
//     p_ddth_m_res[JOINT_NUM];
// static double p_th_s_res[JOINT_NUM], p_dth_s_res[JOINT_NUM],
//     p_ddth_s_res[JOINT_NUM];
// static double p_tau_m_res[JOINT_NUM], p_tau_s_res[JOINT_NUM];

const char *devicename1 = "/dev/ttyUSB0";
// const char *devicename1 = "/dev/ttyUSB1";

// socket communication
double a[21];
struct sockaddr_in addr;
int sock;
char rbuf[4096];

int l;
int ret;
bool sendf = true;
char *tp;
fd_set fds, fdw, fdr;

using namespace std;

void *slave_control(void *) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  int t1 = 0;

  ///// socket /////
  sock = socket(AF_INET, SOCK_STREAM, 0);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(10051);
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  connect(sock, (struct sockaddr *)&addr, sizeof(addr));

  FD_ZERO(&fds);
  FD_SET(sock, &fds);

  if (FD_ISSET(sock, &fdr) && sendf == false) {
    l = recv(sock, rbuf, sizeof(rbuf), 0);
    *(rbuf + l) = 0;

    printf("-> %s\n", rbuf);
  }
  ////////////////////

  CR7 crane_s(devicename1, SLAVE);
  if (!crane_s.Open_port()) return NULL;  // COMポートを開く
  if (!crane_s.Set_port_baudrate()) {
    crane_s.Close_port();
    return NULL;
  }  //ポートの通信レートを設定

  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);

  // for (int i = 0; i < JOINT_NUM2; i++) {
  //   crane_s.theta_ref[i] = goal_pose[i];
  //   crane_s.omega_ref[i] = 0.0;
  //   crane_s.tau_ref[i] = 0.0;
  //   p_th_s_res[i] = crane_s.theta_res[i];
  //   p_dth_s_res[i] = crane_s.omega_res[i];
  //   p_tau_s_res[i] = crane_s.tau_res[i];
  // }

  // cout << "kokomae" << endl;
  sleep(5);
  cout << "slave_JOINT_NUM : " << JOINT_NUM << endl;
  cout << "slave_JOINT_NUM2 : " << JOINT_NUM2 << endl;
  crane_s.Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  // cout << "kokoato" << endl;
  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  printf("=========slave_p_controlstart==========\n");

  // crane_s.ffp = fopen(crane_s.filename2.c_str(), "w");
  // fprintf(crane_s.ffp,
  //         "time,s_presentposition[0],s_presentposition[1],s_presentposition[2],"
  //         "s_presentposition[3],s_presentposition[4],s_presentposition[5],s_"
  //         "presentposition[6],s_presentposition[7],");
  // fprintf(crane_s.ffp,
  //         "s_presentvelocity[0],s_presentvelocity[1],s_presentvelocity[2],s_"
  //         "presentvelocity[3],s_presentvelocity[4],s_presentvelocity[5],s_"
  //         "presentvelocity[6],s_presentvelocity[7],");
  // fprintf(crane_s.ffp,
  //         "s_tau_res[0],s_tau_res[1],s_tau_res[2],s_tau_res[3],s_tau_res[4],s_"
  //         "tau_res[5],s_tau_res[6],s_tau_res[7],");
  // fprintf(crane_s.ffp,
  //         "m_presentposition[0],m_presentposition[1],m_presentposition[2],m_"
  //         "presentposition[3],m_presentposition[4],m_presentposition[5],m_"
  //         "presentposition[6],m_presentposition[7],");
  // fprintf(crane_s.ffp,
  //         "m_presentvelocity[0],m_presentvelocity[1],m_presentvelocity[2],m_"
  //         "presentvelocity[3],m_presentvelocity[4],m_presentvelocity[5],m_"
  //         "presentvelocity[6],m_presentvelocity[7],");
  // fprintf(crane_s.ffp,
  //         "m_tau_res[0],m_tau_res[1],m_tau_res[2],m_tau_res[3],m_tau_res[4],m_"
  //         "tau_res[5],m_tau_res[6],m_tau_res[7],");
  // fprintf(crane_s.ffp, "a[0],a[1],a[2],a[3],a[4],a[5],a[6],");
  // fprintf(crane_s.ffp, "a[7],a[8],a[9],a[10],a[11],a[12],a[13],");
  // fprintf(crane_s.ffp, "a[14],a[15],a[16],a[17],a[18],a[19],a[20],");
  // fprintf(crane_s.ffp, "sleeptime,controltime\n");

  crane_s.Readtheta_res(ID);

  if ((crane_s.theta_res[0] == 0.0) || (crane_s.theta_res[7] == 0.0)) {
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
    crane_s.Enable_Dynamixel_Torque(ID);
    crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    printf("slave読み込み怪しいので終了\n");
    crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    dprintf(sock, "%s", "**");
    close(sock);
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Close_port();
    fclose(crane_s.ffp);
    return NULL;
  }

  for (int j = 0; j < JOINT_NUM2; j++)
    crane_s.d_theta_temp[j] = crane_s.theta_res[j];

  while (ch == 'p') {
    gettimeofday(&start_time_s, NULL);

    crane_s.Readtheta_res(ID);
    if ((crane_s.theta_res[0] == 0.0) || (crane_s.theta_res[7] == 0.0)) {
      crane_s.Disable_Dynamixel_Torque(ID);
      crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
      crane_s.Enable_Dynamixel_Torque(ID);
      crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      printf("slave読み込み怪しいので終了\n");
      crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      dprintf(sock, "%s", "**");
      close(sock);
      crane_s.Disable_Dynamixel_Torque(ID);
      crane_s.Close_port();
      fclose(crane_s.ffp);
      return NULL;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s.omega_res[i] =
          (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
      crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
        crane_s.Enable_Dynamixel_Torque(ID);
        crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        printf("crslaveの軸%dが速いので終了\n", i);
        crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        dprintf(sock, "%s", "**");
        close(sock);
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Close_port();
        fclose(crane_s.ffp);
        return NULL;
      }
    }

    // for (int i = 0; i < JOINT_NUM2; i++) {
    //   crane_s.tau_p[i] =
    //       J[SLAVE][i] *
    //       (Kp[SLAVE][i] *
    //            (crane_s.theta_ref[i] - crane_s.theta_res[i]) +
    //        Kd[SLAVE][i] * (crane_s.omega_ref[i] - crane_s.omega_res[i]));
    //   //力制御によるトルク参照値
    //   crane_s.tau_f[i] =
    //       Kf[SLAVE][i] * (-crane_s.tau_ref[i] - crane_s.tau_res[i]);
    //   if (i == 2) {
    //     crane_s.goal_torque[i] =
    //         crane_s.tau_p[i] + crane_s.tau_f[i] + crane_s.tau_dis[i];
    //   } else {
    //     crane_s.goal_torque[i] =
    //         crane_s.tau_p[i] + crane_s.tau_f[i] + crane_s.tau_dis[i];
    //   }

    //   // DOB
    //   crane_s.dob0[i] =
    //       crane_s.goal_torque[i] + g[i] * J[SLAVE][i] * crane_s.omega_res[i];
    //   crane_s.dob1[i] = g[i] * (crane_s.dob0[i] - crane_s.dob2[i]);
    //   crane_s.dob2[i] += crane_s.dob1[i] * ts;

    //   //外乱トルクの算出
    //   crane_s.tau_dis[i] =
    //       crane_s.dob2[i] - g[i] * J[SLAVE][i] * crane_s.omega_res[i];
    // }

    // crane_s.tau_res[0] =
    //     crane_s.tau_dis[0] - D[SLAVE][0] * crane_s.omega_res[0];
    // crane_s.tau_res[1] =
    //     crane_s.tau_dis[1] - M[SLAVE][0] * sin(crane_s.theta_res[1]) +
    //     M[SLAVE][1] *
    //         sin(crane_s.theta_res[1] + crane_s.theta_res[3]);
    // crane_s.tau_res[2] =
    //     crane_s.tau_dis[2] - D[SLAVE][1] * crane_s.omega_res[2];
    // crane_s.tau_res[3] =
    //     crane_s.tau_dis[3] + M[SLAVE][2] * sin(crane_s.theta_res[1] +
    //                                            crane_s.theta_res[3]);
    // crane_s.tau_res[4] =
    //     crane_s.tau_dis[4] - D[SLAVE][2] * crane_s.omega_res[4];
    // crane_s.tau_res[5] =
    //     crane_s.tau_dis[5] - D[SLAVE][3] * crane_s.omega_res[5];
    // crane_s.tau_res[6] =
    //     crane_s.tau_dis[6] - D[SLAVE][4] * crane_s.omega_res[6];
    // crane_s.tau_res[7] =
    //     crane_s.tau_dis[7] - D[SLAVE][5] * crane_s.omega_res[7];

    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", passtime,
    //         crane_s.theta_res[0], crane_s.theta_res[1],
    //         crane_s.theta_res[2], crane_s.theta_res[3],
    //         crane_s.theta_res[4], crane_s.theta_res[5],
    //         crane_s.theta_res[6], crane_s.theta_res[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.omega_res[0], crane_s.omega_res[1],
    //         crane_s.omega_res[2], crane_s.omega_res[3],
    //         crane_s.omega_res[4], crane_s.omega_res[5],
    //         crane_s.omega_res[6], crane_s.omega_res[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    // crane_s.tau_res[0],
    //         crane_s.tau_res[1], crane_s.tau_res[2], crane_s.tau_res[3],
    //         crane_s.tau_res[4], crane_s.tau_res[5], crane_s.tau_res[6],
    //         crane_s.tau_res[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.theta_ref[0], crane_s.theta_ref[1],
    //         crane_s.theta_ref[2], crane_s.theta_ref[3],
    //         crane_s.theta_ref[4], crane_s.theta_ref[5],
    //         crane_s.theta_ref[6], crane_s.theta_ref[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.omega_ref[0], crane_s.omega_ref[1],
    //         crane_s.omega_ref[2], crane_s.omega_ref[3],
    //         crane_s.omega_ref[4], crane_s.omega_ref[5],
    //         crane_s.omega_ref[6], crane_s.omega_ref[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.tau_ref[0], crane_s.tau_ref[1],
    //         crane_s.tau_ref[2], crane_s.tau_ref[3],
    //         crane_s.tau_ref[4], crane_s.tau_ref[5],
    //         crane_s.tau_ref[6], crane_s.tau_ref[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[0], a[1], a[2],
    // a[3],
    //         a[4], a[5], a[6]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[7], a[8], a[9],
    //         a[10], a[11], a[12], a[13]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[14], a[15], a[16],
    //         a[17], a[18], a[19], a[20]);

    // crane_s.setCranex7Torque(crane_s.goal_torque, ID);

    // calculate input torque
    crane_s.position_control(goal_pose);

    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);

    if (sleep_time_s < 0) sleep_time_s = 0;

    // fprintf(crane_s.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    crane_s.write_csv(passtime, sleep_time_s, control_time_s);

    usleep(sleep_time_s);
    t1++;

    // 1000サンプリングなので、2秒
    if (t1 == 1000) ch = 'b';
  }

  if (ch != 'b') {
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
    crane_s.Enable_Dynamixel_Torque(ID);
    crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    dprintf(sock, "%s", "**");
    close(sock);
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Close_port();
    fclose(crane_s.ffp);
    return NULL;
  }

  ////////////////////////////////ここからバイラテ/////////////////////////////////////////
  printf("==========slave_controlstart==========\n");

  while (ch == 'b') {  //データ取得の開始

    gettimeofday(&start_time_s, NULL);
    crane_s.datareadflag = 0;

    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s.dxl_addparam_result = crane_s.groupBulkRead->addParam(
          ID[i], THETA_RES_ADDRESS,
          THETA_RES_DATA_LENGTH);  //読み込みのデータを設定(現在角度)
    }

    // Bulkread present position
    crane_s.dxl_comm_result =
        crane_s.groupBulkRead->txRxPacket();  //返信データの読み込み
    if (crane_s.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
    // Check if groupbulkread data of Dynamixel is available
    for (int i = 0; i < JOINT_NUM2; i++) {  //返信データが利用できるか確認
      crane_s.dxl_getdata_result = crane_s.groupBulkRead->isAvailable(
          ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);
      if (crane_s.dxl_getdata_result != true) {
        crane_s.datareadflag++;
      }
    }
    if (crane_s.datareadflag == 0) {
      for (int i = 0; i < JOINT_NUM2; i++) {
        crane_s.dxl_theta_res = crane_s.groupBulkRead->getData(
            ID[i], THETA_RES_ADDRESS,
            THETA_RES_DATA_LENGTH);  //返信データから指定のデータを読む
        crane_s.theta_res[i] = dxlvalue2rad(crane_s.dxl_theta_res);
      }
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s.omega_res[i] =
          (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
      crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
        crane_s.Enable_Dynamixel_Torque(ID);
        crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        printf("crslaveの軸%dが速いので終了\n", i);
        crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        dprintf(sock, "%s", "**");
        close(sock);
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Close_port();
        fclose(crane_s.ffp);
        return NULL;
      }
    }

    memcpy(&fdw, &fds, sizeof(fd_set));
    memcpy(&fdr, &fds, sizeof(fd_set));

    // はじめから6秒経つまで通信はとりあえず行わないようにしている
    // socket
    if (passtime >= 2.0) {
      ret = select(sock + 1, &fdr, &fdw, NULL, NULL);

      if (t1 % rnn_ts == 0) {
        if (FD_ISSET(sock, &fdw) && sendf == true) {
          dprintf(sock,
                  "%5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f "
                  "%5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f "
                  "%5.4f %5.4f %5.4f %5.4f %5.4f",
                  (float)crane_s.theta_res[0], (float)crane_s.theta_res[1],
                  (float)crane_s.theta_res[2], (float)crane_s.theta_res[3],
                  (float)crane_s.theta_res[4], (float)crane_s.theta_res[5],
                  (float)crane_s.theta_res[6], (float)crane_s.theta_res[7],
                  (float)crane_s.omega_res[0], (float)crane_s.omega_res[1],
                  (float)crane_s.omega_res[2], (float)crane_s.omega_res[3],
                  (float)crane_s.omega_res[4], (float)crane_s.omega_res[5],
                  (float)crane_s.omega_res[6], (float)crane_s.omega_res[7],
                  (float)crane_s.tau_res[0], (float)crane_s.tau_res[1],
                  (float)crane_s.tau_res[2], (float)crane_s.tau_res[3],
                  (float)crane_s.tau_res[4], (float)crane_s.tau_res[5],
                  (float)crane_s.tau_res[6], (float)crane_s.tau_res[7],
                  (float)passtime);

          // C++→pythonに送ったものを表示して確認
          printf("送った\n");
          printf(
              "\nangle\t\t:\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\n",
              (float)crane_s.theta_res[0], (float)crane_s.theta_res[1],
              (float)crane_s.theta_res[3], (float)crane_s.theta_res[4],
              (float)crane_s.theta_res[5], (float)crane_s.theta_res[6],
              (float)crane_s.theta_res[7]);
          printf(
              "omega_res\t\t:\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5."
              "4f\n",
              (float)crane_s.omega_res[0], (float)crane_s.omega_res[1],
              (float)crane_s.omega_res[3], (float)crane_s.omega_res[4],
              (float)crane_s.omega_res[5], (float)crane_s.omega_res[6],
              (float)crane_s.omega_res[7]);
          printf(
              "tau_res\t\t:\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\n",
              (float)crane_s.tau_res[0], (float)crane_s.tau_res[1],
              (float)crane_s.tau_res[3], (float)crane_s.tau_res[4],
              (float)crane_s.tau_res[5], (float)crane_s.tau_res[6],
              (float)crane_s.tau_res[7]);
          printf("passtime\t\t:\t%5.4f\n", (float)passtime);
          printf("\n\n");
          sendf = false;
        }
      } else if ((t1 + 1) % rnn_ts == 0) {
        if (FD_ISSET(sock, &fdr) && sendf == false) {
          l = recv(sock, rbuf, sizeof(rbuf), 0);
          *(rbuf + l) = 0;

          printf("-> %s\n", rbuf);
          sendf = true;
          tp = strtok(rbuf, ",");
          // printf("a\n");

          if (tp == NULL) {
            cout << "break2だよ\n" << endl;
            break;
          }
          // a[0] = atof(tp);
          // printf("b\n");

          for (int l = 0; l < 21; l++) {
            // while (tp != NULL) {
            a[l] = atof(tp);
            printf("a[%d] = %5.4f\n", l, a[l]);
            tp = strtok(NULL, ",");
            // printf("%s\n", tp);
            // l++;
          }
          // }
          printf("受け取った\n");
        }
      }
    }

    // pthread_mutex_lock(&mutex);// すぐなら4.2秒、少し待つなら5.5

    // 4.2     通信始めてからLSTMがなれるまでマージンとってる
    double theta_ref[JOINT_NUM2] = {0.0};
    double omega_ref[JOINT_NUM2] = {0.0};
    double tau_ref[JOINT_NUM2] = {0.0};
    if (passtime <= 4.3) {
      for (int i = 0; i < JOINT_NUM2; i++) {
        crane_s.theta_ref[i] = goal_pose[i];
        crane_s.omega_ref[i] = 0.0;
        crane_s.tau_ref[i] = 0.0;
      }
    } else {
      for (int i = 0; i < JOINT_NUM2; i++) {
        if (i == 2) {
          crane_s.theta_ref[i] = 3.14;
          crane_s.omega_ref[i] = 0.0;
          crane_s.tau_ref[i] = 0.0;
        } else if (i < 2) {
          crane_s.theta_ref[i] = a[i];
          crane_s.omega_ref[i] = a[i + (JOINT_NUM2 - 1) * 1];
          crane_s.tau_ref[i] = a[i + (JOINT_NUM2 - 1) * 2];
        } else {
          crane_s.theta_ref[i] = a[i - 1];
          crane_s.omega_ref[i] = a[i - 1 + (JOINT_NUM2 - 1) * 1];
          crane_s.tau_ref[i] = a[i - 1 + (JOINT_NUM2 - 1) * 2];
        }
      }
    }
    // printf("b\n");
    // pthread_mutex_unlock(&mutex);
    // printf("c\n");

    // for (int i = 0; i < JOINT_NUM2; i++) {
    //   crane_s.tau_p[i] =
    //       J[SLAVE][i] / 2.0 *
    //       (Kp[SLAVE][i] * (crane_s.theta_ref[i] - crane_s.theta_res[i]) +
    //        Kd[SLAVE][i] * (crane_s.omega_ref[i] - crane_s.omega_res[i]));
    //   //力制御によるトルク参照値
    //   crane_s.tau_f[i] =
    //       Kf[SLAVE][i] / 2.0 * (-crane_s.tau_ref[i] - crane_s.tau_res[i]);
    //   crane_s.goal_torque[i] =
    //       crane_s.tau_p[i] + crane_s.tau_f[i] + crane_s.tau_dis[i];

    //   // DOB
    //   crane_s.dob0[i] =
    //       crane_s.goal_torque[i] + g[i] * J[SLAVE][i] * crane_s.omega_res[i];
    //   crane_s.dob1[i] = g[i] * (crane_s.dob0[i] - crane_s.dob2[i]);
    //   crane_s.dob2[i] += crane_s.dob1[i] * ts;

    //   //外乱トルクの算出
    //   crane_s.tau_dis[i] =
    //       crane_s.dob2[i] - g[i] * J[SLAVE][i] * crane_s.omega_res[i];
    // }

    // crane_s.tau_res[0] =
    //     crane_s.tau_dis[0] - D[SLAVE][0] * crane_s.omega_res[0];
    // crane_s.tau_res[1] =
    //     crane_s.tau_dis[1] - M[SLAVE][0] * sin(crane_s.theta_res[1]) +
    //     M[SLAVE][1] * sin(crane_s.theta_res[1] + crane_s.theta_res[3]);
    // crane_s.tau_res[2] =
    //     crane_s.tau_dis[2] - D[SLAVE][1] * crane_s.omega_res[2];
    // crane_s.tau_res[3] =
    //     crane_s.tau_dis[3] +
    //     M[SLAVE][2] * sin(crane_s.theta_res[1] + crane_s.theta_res[3]);
    // crane_s.tau_res[4] =
    //     crane_s.tau_dis[4] - D[SLAVE][2] * crane_s.omega_res[4];
    // crane_s.tau_res[5] =
    //     crane_s.tau_dis[5] - D[SLAVE][3] * crane_s.omega_res[5];
    // crane_s.tau_res[6] =
    //     crane_s.tau_dis[6] - D[SLAVE][4] * crane_s.omega_res[6];
    // crane_s.tau_res[7] =
    //     crane_s.tau_dis[7] - D[SLAVE][5] * crane_s.omega_res[7];

    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", passtime,
    //         crane_s.theta_res[0], crane_s.theta_res[1], crane_s.theta_res[2],
    //         crane_s.theta_res[3], crane_s.theta_res[4], crane_s.theta_res[5],
    //         crane_s.theta_res[6], crane_s.theta_res[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.omega_res[0], crane_s.omega_res[1], crane_s.omega_res[2],
    //         crane_s.omega_res[3], crane_s.omega_res[4], crane_s.omega_res[5],
    //         crane_s.omega_res[6], crane_s.omega_res[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    // crane_s.tau_res[0],
    //         crane_s.tau_res[1], crane_s.tau_res[2], crane_s.tau_res[3],
    //         crane_s.tau_res[4], crane_s.tau_res[5], crane_s.tau_res[6],
    //         crane_s.tau_res[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.theta_ref[0], crane_s.theta_ref[1], crane_s.theta_ref[2],
    //         crane_s.theta_ref[3], crane_s.theta_ref[4], crane_s.theta_ref[5],
    //         crane_s.theta_ref[6], crane_s.theta_ref[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    //         crane_s.omega_ref[0], crane_s.omega_ref[1], crane_s.omega_ref[2],
    //         crane_s.omega_ref[3], crane_s.omega_ref[4], crane_s.omega_ref[5],
    //         crane_s.omega_ref[6], crane_s.omega_ref[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
    // crane_s.tau_ref[0],
    //         crane_s.tau_ref[1], crane_s.tau_ref[2], crane_s.tau_ref[3],
    //         crane_s.tau_ref[4], crane_s.tau_ref[5], crane_s.tau_ref[6],
    //         crane_s.tau_ref[7]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[0], a[1], a[2],
    // a[3],
    //         a[4], a[5], a[6]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[7], a[8], a[9],
    //         a[10], a[11], a[12], a[13]);
    // fprintf(crane_s.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[14], a[15], a[16],
    //         a[17], a[18], a[19], a[20]);

    // crane_s.setCranex7Torque(crane_s.goal_torque, ID);
    // calculate input torque
    crane_s.torque_control(theta_ref, omega_ref, tau_ref);

    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);

    if (sleep_time_s < 0) sleep_time_s = 0;

    // fprintf(crane_s.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    crane_s.write_csv(passtime, sleep_time_s, control_time_s);

    usleep(sleep_time_s);
    passtime += ts;
    t1++;
    printf("time: %lf\n", passtime);
  }

  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  dprintf(sock, "%s", "**");
  close(sock);
  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Close_port();
  fclose(crane_s.ffp);
  return NULL;
}

void *keyboard_check(void *) {
  char key;

  while (ch != 'q') {
    key = getch();

    if (key == 'b') {
      ch = 'b';
    } else if (key == 'q') {
      ch = 'q';
      dprintf(sock, "%s", "**");
      close(sock);

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
  pthread_t slave_thread, getch_thread;

  // for (int i = 0; i < JOINT_NUM; i++) {
  //   p_th_m_res[i] = 0.0;
  //   p_dth_m_res[i] = 0.0;
  //   p_ddth_m_res[i] = 0.0;
  //   p_th_s_res[i] = 0.0;
  //   p_dth_s_res[i] = 0.0;
  //   p_ddth_s_res[i] = 0.0;
  //   p_tau_m_res[i] = 0.0;
  //   p_tau_s_res[i] = 0.0;
  // }

  if (pthread_create(&slave_thread, NULL, &slave_control, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }

  if (pthread_create(&getch_thread, NULL, &keyboard_check, NULL) != 0) {
    fprintf(stderr, "cannot create control thread\n");
    return 1;
  }

  // pthread_join(master_thread, NULL);
  pthread_join(slave_thread, NULL);
  pthread_join(getch_thread, NULL);

  return 0;
}
