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

  static double ts = 0.002;
  int rnn_ts = 20;

  static double time = 0.0;
  int count = 0;

  const char *devicename1 = "/dev/ttyUSB0";

  // socket communication
  struct sockaddr_in addr;

  char rbuf[4096];

  int l;
  bool sendf = true;
  char *tp;
  fd_set fds, fdw, fdr;

  double theta_ref[JOINT_NUM2] = {0.0};
  double omega_ref[JOINT_NUM2] = {0.0};
  double tau_ref[JOINT_NUM2] = {0.0};

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

  crane_s.Readpresent_position(ID);
  for (int j = 0; j < JOINT_NUM2; j++)
    crane_s.d_theta_temp[j] = crane_s.theta_res[j];

  /***************************************************************************
   *                              main loop
   ***************************************************************************/
  while (true) {
    // start time
    gettimeofday(&start_time_s, NULL);

    // position observation
    crane_s.Readpresent_position(ID);

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s.omega_res[i] =
          (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
      crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
    }

    // forced termination
    bool finishFlag = false;
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (crane_s.theta_res[i] == 0.0) {
        cout << "読み込み怪しいので終了" << endl;
        finishFlag = true;
      }

      if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
        printf("軸%dが速いので終了: %lf [rad/s]\n", i, crane_s.omega_res[i]);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // socket communication
    memcpy(&fdw, &fds, sizeof(fd_set));
    memcpy(&fdr, &fds, sizeof(fd_set));
    if (time >= 2.0) {
      // change mode to autonomous control
      if (count == 500 * 2) ch = 'b';

      if (count % rnn_ts == 0) {  // send
        if (FD_ISSET(sock, &fdw) && sendf == true) {
          string str = "";
          int I = JOINT_NUM;
          for (int i = 0; i < I; i++)
            str += to_string(crane_s.theta_res[i]) + ",";
          for (int i = 0; i < I; i++)
            str += to_string(crane_s.omega_res[i]) + ",";
          for (int i = 0; i < I; i++)
            str += to_string(crane_s.tau_res[i]) + ",";
          str += to_string(time);
          dprintf(sock, "%s", str.c_str());
          printf("%s", str.c_str());

          // C++→pythonに送ったものを表示して確認
          // printf("\nsend");
          // printf("\ntheta_res: ");
          // for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.theta_res[i]);
          // printf("\nomega_res: ");
          // for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.omega_res[i]);
          // printf("\ntau_res  : ");
          // for (int i = 0; i < I; i++) printf("%5.4f ", crane_s.tau_res[i]);
          // printf("\npasstime : %5.4f\n\n", time);

          sendf = false;
        }
      } else if ((count + 1) % rnn_ts == 0) {  // receive
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

          double a[24] = {0};
          for (int l = 0; l < 24; l++) {
            a[l] = atof(tp);
            printf("a[%d] = %5.4f\n", l, a[l]);
            tp = strtok(NULL, ",");
          }

          for (int i = 0; i < JOINT_NUM2; i++) {
            theta_ref[i] = a[i];
            omega_ref[i] = a[i + JOINT_NUM2 * 1];
            tau_ref[i] = a[i + JOINT_NUM2 * 2];
          }
        }
      }
    }

    // action
    if (ch == 'p') {  // move to initial pose
      crane_s.position_control(goal_pose);

    } else if (ch == 'b') {  // autonomous control
      crane_s.force_control(theta_ref, omega_ref, tau_ref);

    } else {  // finish
      break;
    }

    // end time
    gettimeofday(&end_time_s, NULL);

    // calculate sleep time
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
    if (sleep_time_s < 0) sleep_time_s = 0;

    // write current state to csv
    crane_s.write_csv(time, sleep_time_s, control_time_s);

    // print
    printf("mode: %c  time: %lf\n", ch, time);

    // sleep
    usleep(sleep_time_s);

    // time count
    if (ch == 'b') time += ts;
    count++;
  }

  // position control mode
  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  // move to finish pose
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // Torque off
  crane_s.Disable_Dynamixel_Torque(ID);

  // finish socket communication
  dprintf(sock, "%s", "**");
  close(sock);

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
