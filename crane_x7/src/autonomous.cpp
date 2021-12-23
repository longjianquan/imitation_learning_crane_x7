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

using namespace std;

static char ch = 'p';
int sock;

void *autonomous_control(void *) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  bool finishFlag = false;

  static double ts = 0.002;
  int rnn_ts = 20;

  static double time = 0.0;
  int count = 0;

  const char *devicename1 = "/dev/ttyUSB0";

  // socket communication
  double a[21];
  struct sockaddr_in addr;

  char rbuf[4096];

  int l;
  int ret;
  bool sendf = true;
  char *tp;
  fd_set fds, fdw, fdr;

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

  CR7 crane_s(devicename1, goal_pose, SLAVE);
  if (!crane_s.Open_port()) return NULL;  // COMポートを開く
  if (!crane_s.Set_port_baudrate()) {
    crane_s.Close_port();
    return NULL;
  }  //ポートの通信レートを設定

  // position control mode
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  // move to initial pose
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s.Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // current control mode
  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  crane_s.Readtheta_res(ID);
  for (int j = 0; j < JOINT_NUM2; j++)
    crane_s.d_theta_temp[j] = crane_s.theta_res[j];

  while (true) {
    gettimeofday(&start_time_s, NULL);

    // position observation
    crane_s.Readtheta_res(ID);

    if ((crane_s.theta_res[0] == 0.0) || (crane_s.theta_res[7] == 0.0)) {
      cout << "読み込み怪しいので終了" << endl;
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
        printf("軸%dが速いので終了: %lf [rad/s]\n", i, crane_s.omega_res[i]);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // calculate input torque
    // crane_s.position_control(goal_pose);
    // move to initial pose
    if (ch == 'p') {
      crane_s.position_control(goal_pose);

    // bilateral control
    } else if (ch == 'b') {
      memcpy(&fdw, &fds, sizeof(fd_set));
      memcpy(&fdr, &fds, sizeof(fd_set));

      // はじめから6秒経つまで通信はとりあえず行わないようにしている
      // socket
      if (time >= 2.0) {
        ret = select(sock + 1, &fdr, &fdw, NULL, NULL);

        if (count % rnn_ts == 0) {
          if (FD_ISSET(sock, &fdw) && sendf == true) {
            string str = "";
            int I = JOINT_NUM;
            for (int i = 0; i < I; i++)
              // dprintf(sock, "%5.4f ", crane_s.theta_res[i]);
              str += to_string(crane_s.theta_res[i]) + ",";
            for (int i = 0; i < I; i++)
              // dprintf(sock, "%5.4f ", crane_s.omega_res[i]);
              str += to_string(crane_s.omega_res[i]) + ",";
            for (int i = 0; i < I; i++)
              // dprintf(sock, "%5.4f ", crane_s.tau_res[i]);
              str += to_string(crane_s.tau_res[i]) + ",";
            str += to_string(time);
            dprintf(sock, "%s", str.c_str());
            printf("%s", str.c_str());

            // C++→pythonに送ったものを表示して確認
            printf("\nsend");
            printf("\ntheta_res: ");
            for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.theta_res[i]);
            printf("\nomega_res: ");
            for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.omega_res[i]);
            printf("\ntau_res  : ");
            for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.tau_res[i]);
            printf("\npasstime : %5.4f\n\n", time);
            sendf = false;
          }
        } else if ((count + 1) % rnn_ts == 0) {
          if (FD_ISSET(sock, &fdr) && sendf == false) {
            l = recv(sock, rbuf, sizeof(rbuf), 0);
            *(rbuf + l) = 0;

            printf("receive\n");
            printf("-> %s\n", rbuf);
            sendf = true;
            tp = strtok(rbuf, ",");

            if (tp == NULL) {
              cout << "receive error\n" << endl;
              break;
            }

            for (int l = 0; l < 21; l++) {
              a[l] = atof(tp);
              printf("a[%d] = %5.4f\n", l, a[l]);
              tp = strtok(NULL, ",");
            }
          }
        }
      }

      // 4.2     通信始めてからLSTMがなれるまでマージンとってる
      if (time <= 4.3) {
        // for (int i = 0; i < JOINT_NUM2; i++) {
        //   theta_ref[i] = goal_pose[i];
        //   omega_ref[i] = 0.0;
        //   tau_ref[i] = 0.0;
        // }
        crane_s.position_control(goal_pose);
      } else {
        double theta_ref[JOINT_NUM2] = {0.0};
        double omega_ref[JOINT_NUM2] = {0.0};
        double tau_ref[JOINT_NUM2] = {0.0};
        for (int i = 0; i < JOINT_NUM2; i++) {
          if (i == 2) {
            theta_ref[i] = 3.14;
            omega_ref[i] = 0.0;
            tau_ref[i] = 0.0;
          } else if (i < 2) {
            theta_ref[i] = a[i];
            omega_ref[i] = a[i + (JOINT_NUM2 - 1) * 1];
            tau_ref[i] = a[i + (JOINT_NUM2 - 1) * 2];
          } else {
            theta_ref[i] = a[i - 1];
            omega_ref[i] = a[i - 1 + (JOINT_NUM2 - 1) * 1];
            tau_ref[i] = a[i - 1 + (JOINT_NUM2 - 1) * 2];
          }
        }
        crane_s.force_control(theta_ref, omega_ref, tau_ref);
      }
    
    // finish
    } else {
      break;
    }

    // calculate sleep time
    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
    if (sleep_time_s < 0) sleep_time_s = 0;

    // write current state to csv
    crane_s.write_csv(time, sleep_time_s, control_time_s);

    usleep(sleep_time_s);

    // 1000サンプリングなので、2秒
    if (count == 1000) ch = 'b';

    // print
    printf("mode: %c  time: %lf\n", ch, time);

    // time count
    if (ch == 'b') {
        time += ts;
    }
    count++;
  }

  // if (ch != 'b') {
  //   crane_s.Disable_Dynamixel_Torque(ID);
  //   crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  //   crane_s.Enable_Dynamixel_Torque(ID);
  //   crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  //   sleep(5);
  //   crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  //   sleep(5);
  //   dprintf(sock, "%s", "**");
  //   close(sock);
  //   crane_s.Disable_Dynamixel_Torque(ID);
  //   crane_s.Close_port();
  //   fclose(crane_s.ffp);
  //   return NULL;
  // }

  // while (ch == 'b') {  //データ取得の開始

  //   gettimeofday(&start_time_s, NULL);
  //   crane_s.datareadflag = 0;

  //   for (int i = 0; i < JOINT_NUM2; i++) {
  //     crane_s.dxl_addparam_result = crane_s.groupBulkRead->addParam(
  //         ID[i], THETA_RES_ADDRESS,
  //         THETA_RES_DATA_LENGTH);  //読み込みのデータを設定(現在角度)
  //   }

  //   // Bulkread present position
  //   crane_s.dxl_comm_result =
  //       crane_s.groupBulkRead->txRxPacket();  //返信データの読み込み
  //   if (crane_s.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
  //   // Check if groupbulkread data of Dynamixel is available
  //   for (int i = 0; i < JOINT_NUM2; i++) {  //返信データが利用できるか確認
  //     crane_s.dxl_getdata_result = crane_s.groupBulkRead->isAvailable(
  //         ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);
  //     if (crane_s.dxl_getdata_result != true) {
  //       crane_s.datareadflag++;
  //     }
  //   }
  //   if (crane_s.datareadflag == 0) {
  //     for (int i = 0; i < JOINT_NUM2; i++) {
  //       crane_s.dxl_theta_res = crane_s.groupBulkRead->getData(
  //           ID[i], THETA_RES_ADDRESS,
  //           THETA_RES_DATA_LENGTH);  //返信データから指定のデータを読む
  //       crane_s.theta_res[i] = dxlvalue2rad(crane_s.dxl_theta_res);
  //     }
  //   }

  //   for (int i = 0; i < JOINT_NUM2; i++) {
  //     crane_s.omega_res[i] =
  //         (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
  //     crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
  //   }

  //   for (int i = 0; i < JOINT_NUM2; i++) {
  //     if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
  //       crane_s.Disable_Dynamixel_Torque(ID);
  //       crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  //       crane_s.Enable_Dynamixel_Torque(ID);
  //       crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  //       sleep(5);
  //       printf("crslaveの軸%dが速いので終了\n", i);
  //       crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  //       sleep(5);
  //       dprintf(sock, "%s", "**");
  //       close(sock);
  //       crane_s.Disable_Dynamixel_Torque(ID);
  //       crane_s.Close_port();
  //       fclose(crane_s.ffp);
  //       return NULL;
  //     }
  //   }

  //   memcpy(&fdw, &fds, sizeof(fd_set));
  //   memcpy(&fdr, &fds, sizeof(fd_set));

  //   // はじめから6秒経つまで通信はとりあえず行わないようにしている
  //   // socket
  //   if (passtime >= 2.0) {
  //     ret = select(sock + 1, &fdr, &fdw, NULL, NULL);

  //     if (t1 % rnn_ts == 0) {
  //       if (FD_ISSET(sock, &fdw) && sendf == true) {
  //         int I = JOINT_NUM;
  //         for (int i = 0; i < I; i++)
  //           dprintf(sock, "%5.4f ", crane_s.theta_res[i]);
  //         for (int i = 0; i < I; i++)
  //           dprintf(sock, "%5.4f ", crane_s.omega_res[i]);
  //         for (int i = 0; i < I; i++)
  //           dprintf(sock, "%5.4f ", crane_s.tau_res[i]);
  //         dprintf(sock, "%5.4f ", passtime);

  //         // C++→pythonに送ったものを表示して確認
  //         printf("send\n");
  //         printf("\ntheta_res: ");
  //         for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.theta_res[i]);
  //         printf("\nomega_res: ");
  //         for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.omega_res[i]);
  //         printf("\ntau_res: ");
  //         for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.tau_res[i]);
  //         printf("passtime: %5.4f\n\n\n", passtime);
  //         sendf = false;
  //       }
  //     } else if ((t1 + 1) % rnn_ts == 0) {
  //       if (FD_ISSET(sock, &fdr) && sendf == false) {
  //         l = recv(sock, rbuf, sizeof(rbuf), 0);
  //         *(rbuf + l) = 0;

  //         printf("-> %s\n", rbuf);
  //         sendf = true;
  //         tp = strtok(rbuf, ",");

  //         if (tp == NULL) {
  //           cout << "break2だよ\n" << endl;
  //           break;
  //         }

  //         for (int l = 0; l < 21; l++) {
  //           a[l] = atof(tp);
  //           printf("a[%d] = %5.4f\n", l, a[l]);
  //           tp = strtok(NULL, ",");
  //         }
  //         printf("受け取った\n");
  //       }
  //     }
  //   }

  //   // 4.2     通信始めてからLSTMがなれるまでマージンとってる
  //   double theta_ref[JOINT_NUM2] = {0.0};
  //   double omega_ref[JOINT_NUM2] = {0.0};
  //   double tau_ref[JOINT_NUM2] = {0.0};
  //   if (passtime <= 4.3) {
  //     for (int i = 0; i < JOINT_NUM2; i++) {
  //       theta_ref[i] = goal_pose[i];
  //       omega_ref[i] = 0.0;
  //       tau_ref[i] = 0.0;
  //     }
  //   } else {
  //     for (int i = 0; i < JOINT_NUM2; i++) {
  //       if (i == 2) {
  //         theta_ref[i] = 3.14;
  //         omega_ref[i] = 0.0;
  //         tau_ref[i] = 0.0;
  //       } else if (i < 2) {
  //         theta_ref[i] = a[i];
  //         omega_ref[i] = a[i + (JOINT_NUM2 - 1) * 1];
  //         tau_ref[i] = a[i + (JOINT_NUM2 - 1) * 2];
  //       } else {
  //         theta_ref[i] = a[i - 1];
  //         omega_ref[i] = a[i - 1 + (JOINT_NUM2 - 1) * 1];
  //         tau_ref[i] = a[i - 1 + (JOINT_NUM2 - 1) * 2];
  //       }
  //     }
  //   }

  //   // calculate input torque
  //   crane_s.torque_control(theta_ref, omega_ref, tau_ref);

  //   // calculate sleep time
  //   gettimeofday(&end_time_s, NULL);
  //   control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
  //                     (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
  //   sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
  //   if (sleep_time_s < 0) sleep_time_s = 0;

  //   // write current state to csv
  //   crane_s.write_csv(passtime, sleep_time_s, control_time_s);

  //   usleep(sleep_time_s);

  //   // time count
  //   passtime += ts;
  //   t1++;

  //   // print
  //   printf("time: %lf\n", passtime);
  // }

  // position control mode
  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  // move to finish pose
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

  if (pthread_create(&slave_thread, NULL, &autonomous_control, NULL) != 0) {
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
