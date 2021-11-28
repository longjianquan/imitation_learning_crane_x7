
// *********    Example teaching play back    *********
//
//
#include <arpa/inet.h>
#include <fcntl.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <iostream>
// Dynamixel SDK libraryのインクルード
#include "crane.h"
#include "crane_x7_comm.h"
//#include "dynamixel_sdk.h"

static int ID[JOINT_NUM] = {2, 3, 4, 5, 6, 7, 8, 9};  // サーボモータのID
static double JOINT_MIN[JOINT_NUM] = {
    262, 1024, 262, 228,
    262, 1024, 148, 1991};  // サーボモータの最小動作角(value)
static double JOINT_MAX[JOINT_NUM] = {
    3834, 3072, 3834, 2048,
    3834, 3072, 3948, 3072};  // サーボモータの最大動作角(value)
// static double	save_pose[JOINT_NUM]	=
// {	2.10,		3.14,		3.54,		1.415,		3.14,		3.34,
// 3.14,		3.327};	//
// Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる static double
// goal_pose[JOINT_NUM]	=
// {	3.14,		3.14,		3.14,		1.415,		3.14,		3.34,
// 1.67,		3.327}; //
// Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる
static double save_pose[JOINT_NUM] = {
    1.68, 3.14, 3.88, 1.71,
    3.14, 3.14, 3.14, 3.49134};  // 位置制御モードで一旦行く位置(rad)
static double goal_pose[JOINT_NUM] = {3.14, 3.14, 3.14, 1.38,
                                      3.14, 3.14, 3.14, 4.0};
static double finish_pose[JOINT_NUM] = {
    2.10, 3.10, 3.54, 0.45, 3.16,
    2.94, 3.14, 3.327};  // Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる
static double LIMIT_SPEED[JOINT_NUM] = {
    3.00, 2.50, 3.00, 3.00, 4.50,
    5.00, 6.00, 6.00};  // 速度の上限値暴れた時ように教師データ収集時より下げている

// static double	initial_positioin[JOINT_NUM]	=
// {	3.14,		3.14,		3.14,		1.415,		3.14,		3.34,
// 1.67,		3.327}; //
// Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる

//ココはdegreeなので0中心
static double ts = 0.002;
static double g[JOINT_NUM] = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
int rnn_ts = 10;
static char ch = 'p';
static double passtime = 0.0;

// static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// static double J[2][JOINT_NUM] = {{1, 1, 1, 1, 1, 1, 1, 1},{0.5, 0.5, 0.5,
// 0.5, 0.5, 0.5, 0.5, 0.5}};
static double J[2][JOINT_NUM] = {
    {0.012258, 0.11299, 0.012028, 0.04, 0.005676, 0.0066, 0.006281, 0.006891},
    {0.012258, 0.11299, 0.012028, 0.04, 0.005676, 0.0066, 0.006281, 0.006891}};
static double M[2][3] = {{2.094457, 1.1505, 1.18337},
                         {2.094457, 1.1505, 1.38337}};
static double D[2][6] = {{0.0501, 0.242, 0.020, 0.0391, 0.0300, 0.021},
                         {0.0501, 0.242, 0.020, 0.0391, 0.0300, 0.021}};

static double Kp[2][JOINT_NUM] = {
    {256., 196, 900., 81., 256., 256., 100., 256.},
    {256., 196, 625., 81., 256., 256., 100., 256.}};

static double Kd[2][JOINT_NUM] = {{32., 28., 60., 18., 32., 32., 20., 32.},
                                  {32., 28., 25., 18., 32., 32., 20., 32.}};

static double Kf[2][JOINT_NUM] = {{0.50, 0.50, 1, 0.50, 0.85, 0.75, 0.75, 1.0},
                                  {0.50, 0.50, 1, 0.50, 0.85, 0.75, 0.75, 1.0}};

static double p_th_m_res[JOINT_NUM], p_dth_m_res[JOINT_NUM],
    p_ddth_m_res[JOINT_NUM];
static double p_th_s_res[JOINT_NUM], p_dth_s_res[JOINT_NUM],
    p_ddth_s_res[JOINT_NUM];
static double p_tau_m_res[JOINT_NUM], p_tau_s_res[JOINT_NUM];

// const char* devicename1 = "/dev/ttyUSB0";//こっちがスレーブ
// const char* devicename2 = "/dev/ttyUSB1";//こっちがマスター

const char *devicename1 = "/dev/ttyUSB0";  //こっちがスレーブ

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

///////////////////////////////*******************************************//////////////////////////////////////////
////////////////////////////////*************slave_control()*********************:////////////////////////////////
void *slave_control(void *) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  int t1 = 0;

  sock = socket(AF_INET, SOCK_STREAM, 0);

  addr.sin_family = AF_INET;
  addr.sin_port = htons(10051);
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");

  connect(sock, (struct sockaddr *)&addr, sizeof(addr));

  FD_ZERO(&fds);
  FD_SET(sock, &fds);

  CR7 crslave(devicename1, SLAVE);
  if (!crslave.Open_port()) return NULL;  // COMポートを開く
  if (!crslave.Set_port_baudrate()) {
    crslave.Close_port();
    return NULL;
  }  //ポートの通信レートを設定

  crslave.Setoperation(POSITION_CONTROL_MODE, ID);
  crslave.Enable_Dynamixel_Torque(ID);
  crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);

  for (int i = 0; i < JOINT_NUM2; i++) {
    crslave.goal_position[i] = goal_pose[i];
    crslave.goal_velocity[i] = 0.0;
    crslave.target_torque[i] = 0.0;
    p_th_s_res[i] = crslave.present_position[i];
    ;
    p_dth_s_res[i] = crslave.d_theta_res[i];
    p_tau_s_res[i] = crslave.tau_res[i];
  }

  // cout << "kokomae" << endl;
  sleep(5);
  cout << "slave_JOINT_NUM : " << JOINT_NUM << endl;
  cout << "slave_JOINT_NUM2 : " << JOINT_NUM2 << endl;
  crslave.Move_Goal_Position(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  // cout << "kokoato" << endl;
  crslave.Disable_Dynamixel_Torque(ID);
  crslave.Setoperation(CURRENT_CONTROL_MODE, ID);
  crslave.Enable_Dynamixel_Torque(ID);

  printf(
      "===================================slave_p_controlstart================="
      "===========\n");

  // int k = 0;
  crslave.ffp = fopen(crslave.filename2.c_str(), "w");
  fprintf(crslave.ffp,
          "time,s_presentposition[0],s_presentposition[1],s_presentposition[2],"
          "s_presentposition[3],s_presentposition[4],s_presentposition[5],s_"
          "presentposition[6],s_presentposition[7],");
  fprintf(crslave.ffp,
          "s_presentvelocity[0],s_presentvelocity[1],s_presentvelocity[2],s_"
          "presentvelocity[3],s_presentvelocity[4],s_presentvelocity[5],s_"
          "presentvelocity[6],s_presentvelocity[7],");
  fprintf(crslave.ffp,
          "s_tau_res[0],s_tau_res[1],s_tau_res[2],s_tau_res[3],s_tau_res[4],s_"
          "tau_res[5],s_tau_res[6],s_tau_res[7],");
  fprintf(crslave.ffp,
          "m_presentposition[0],m_presentposition[1],m_presentposition[2],m_"
          "presentposition[3],m_presentposition[4],m_presentposition[5],m_"
          "presentposition[6],m_presentposition[7],");
  fprintf(crslave.ffp,
          "m_presentvelocity[0],m_presentvelocity[1],m_presentvelocity[2],m_"
          "presentvelocity[3],m_presentvelocity[4],m_presentvelocity[5],m_"
          "presentvelocity[6],m_presentvelocity[7],");
  fprintf(crslave.ffp,
          "m_tau_res[0],m_tau_res[1],m_tau_res[2],m_tau_res[3],m_tau_res[4],m_"
          "tau_res[5],m_tau_res[6],m_tau_res[7],");
  fprintf(crslave.ffp, "a[0],a[1],a[2],a[3],a[4],a[5],a[6],");
  fprintf(crslave.ffp, "a[7],a[8],a[9],a[10],a[11],a[12],a[13],");
  fprintf(crslave.ffp, "a[14],a[15],a[16],a[17],a[18],a[19],a[20],");
  // fprintf( crslave.ffp,
  // "tau_p[0],tau_p[1],tau_p[2],tau_p[3],tau_p[4],tau_p[5],tau_p[6],tau_p[7],"
  // ); fprintf( crslave.ffp,
  // "tau_f[0],tau_f[1],tau_f[2],tau_f[3],tau_f[4],tau_f[5],tau_f[6],tau_f[7],"
  // ); fprintf( crslave.ffp,
  // "tau_dis[0],tau_dis[1],tau_dis[2],tau_dis[3],tau_dis[4],tau_dis[5],tau_dis[6],tau_dis[7],"
  // ); fprintf( crslave.ffp,
  // "goalcurrent[0],goalcurrent[1],goalcurrent[2],goalcurrent[3],goalcurrent[4],goalcurrent[5],goalcurrent[6],goalcurrent[7],"
  // );
  fprintf(crslave.ffp, "sleeptime,controltime\n");

  crslave.Readpresent_position(ID);
  if ((crslave.present_position[0] == 0.0) ||
      (crslave.present_position[7] == 0.0)) {
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Setoperation(POSITION_CONTROL_MODE, ID);
    crslave.Enable_Dynamixel_Torque(ID);
    crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    printf("slave読み込み怪しいので終了\n");
    crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    dprintf(sock, "%s", "**");
    close(sock);
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Close_port();
    fclose(crslave.ffp);
    return NULL;
  }
  for (int j = 0; j < JOINT_NUM2; j++) {
    crslave.d_theta_temp[j] = crslave.present_position[j];
  }
  while (ch == 'p') {
    gettimeofday(&start_time_s, NULL);

    crslave.Readpresent_position(ID);
    if ((crslave.present_position[0] == 0.0) ||
        (crslave.present_position[7] == 0.0)) {
      crslave.Disable_Dynamixel_Torque(ID);
      crslave.Setoperation(POSITION_CONTROL_MODE, ID);
      crslave.Enable_Dynamixel_Torque(ID);
      crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      printf("slave読み込み怪しいので終了\n");
      crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      dprintf(sock, "%s", "**");
      close(sock);
      crslave.Disable_Dynamixel_Torque(ID);
      crslave.Close_port();
      fclose(crslave.ffp);
      return NULL;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.d_theta_res[i] =
          (crslave.present_position[i] - crslave.d_theta_temp[i]) * g[i];
      crslave.d_theta_temp[i] += crslave.d_theta_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crslave.d_theta_res[i]) >= LIMIT_SPEED[i]) {
        crslave.Disable_Dynamixel_Torque(ID);
        crslave.Setoperation(POSITION_CONTROL_MODE, ID);
        crslave.Enable_Dynamixel_Torque(ID);
        crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        printf("crslaveの軸%dが速いので終了\n", i);
        crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        dprintf(sock, "%s", "**");
        close(sock);
        crslave.Disable_Dynamixel_Torque(ID);
        crslave.Close_port();
        fclose(crslave.ffp);
        return NULL;
      }
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.tau_p[i] =
          J[SLAVE][i] *
          (Kp[SLAVE][i] *
               (crslave.goal_position[i] - crslave.present_position[i]) +
           Kd[SLAVE][i] * (crslave.goal_velocity[i] - crslave.d_theta_res[i]));
      //力制御によるトルク参照値
      crslave.tau_f[i] =
          Kf[SLAVE][i] * (-crslave.target_torque[i] - crslave.tau_res[i]);
      if (i == 2) {
        crslave.goal_torque[i] =
            crslave.tau_p[i] + crslave.tau_f[i] + crslave.tau_dis[i];
      } else {
        crslave.goal_torque[i] =
            crslave.tau_p[i] + crslave.tau_f[i] + crslave.tau_dis[i];
      }

      // DOB
      crslave.dob0[i] =
          crslave.goal_torque[i] + g[i] * J[SLAVE][i] * crslave.d_theta_res[i];
      crslave.dob1[i] = g[i] * (crslave.dob0[i] - crslave.dob2[i]);
      crslave.dob2[i] += crslave.dob1[i] * ts;

      //外乱トルクの算出
      crslave.tau_dis[i] =
          crslave.dob2[i] - g[i] * J[SLAVE][i] * crslave.d_theta_res[i];
    }

    crslave.tau_res[0] =
        crslave.tau_dis[0] - D[SLAVE][0] * crslave.d_theta_res[0];
    crslave.tau_res[1] =
        crslave.tau_dis[1] - M[SLAVE][0] * sin(crslave.present_position[1]) +
        M[SLAVE][1] *
            sin(crslave.present_position[1] + crslave.present_position[3]);
    crslave.tau_res[2] =
        crslave.tau_dis[2] - D[SLAVE][1] * crslave.d_theta_res[2];
    crslave.tau_res[3] =
        crslave.tau_dis[3] + M[SLAVE][2] * sin(crslave.present_position[1] +
                                               crslave.present_position[3]);
    crslave.tau_res[4] =
        crslave.tau_dis[4] - D[SLAVE][2] * crslave.d_theta_res[4];
    crslave.tau_res[5] =
        crslave.tau_dis[5] - D[SLAVE][3] * crslave.d_theta_res[5];
    crslave.tau_res[6] =
        crslave.tau_dis[6] - D[SLAVE][4] * crslave.d_theta_res[6];
    crslave.tau_res[7] =
        crslave.tau_dis[7] - D[SLAVE][5] * crslave.d_theta_res[7];

    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", passtime,
            crslave.present_position[0], crslave.present_position[1],
            crslave.present_position[2], crslave.present_position[3],
            crslave.present_position[4], crslave.present_position[5],
            crslave.present_position[6], crslave.present_position[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.d_theta_res[0], crslave.d_theta_res[1],
            crslave.d_theta_res[2], crslave.d_theta_res[3],
            crslave.d_theta_res[4], crslave.d_theta_res[5],
            crslave.d_theta_res[6], crslave.d_theta_res[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", crslave.tau_res[0],
            crslave.tau_res[1], crslave.tau_res[2], crslave.tau_res[3],
            crslave.tau_res[4], crslave.tau_res[5], crslave.tau_res[6],
            crslave.tau_res[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.goal_position[0], crslave.goal_position[1],
            crslave.goal_position[2], crslave.goal_position[3],
            crslave.goal_position[4], crslave.goal_position[5],
            crslave.goal_position[6], crslave.goal_position[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.goal_velocity[0], crslave.goal_velocity[1],
            crslave.goal_velocity[2], crslave.goal_velocity[3],
            crslave.goal_velocity[4], crslave.goal_velocity[5],
            crslave.goal_velocity[6], crslave.goal_velocity[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.target_torque[0], crslave.target_torque[1],
            crslave.target_torque[2], crslave.target_torque[3],
            crslave.target_torque[4], crslave.target_torque[5],
            crslave.target_torque[6], crslave.target_torque[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[0], a[1], a[2], a[3],
            a[4], a[5], a[6]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[7], a[8], a[9],
            a[10], a[11], a[12], a[13]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[14], a[15], a[16],
            a[17], a[18], a[19], a[20]);
    // fprintf( crslave.ffp,
    // "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",crslave.tau_p[0],crslave.tau_p[1],crslave.tau_p[2],crslave.tau_p[3],crslave.tau_p[4],crslave.tau_p[5],crslave.tau_p[6],crslave.tau_p[7]);
    // fprintf( crslave.ffp,
    // "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",crslave.tau_f[0],crslave.tau_f[1],crslave.tau_f[2],crslave.tau_f[3],crslave.tau_f[4],crslave.tau_f[5],crslave.tau_f[6],crslave.tau_f[7]);
    // fprintf( crslave.ffp,
    // "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",crslave.tau_dis[0],crslave.tau_dis[1],crslave.tau_dis[2],crslave.tau_dis[3],crslave.tau_dis[4],crslave.tau_dis[5],crslave.tau_dis[6],crslave.tau_dis[7]);

    crslave.setCranex7Torque(crslave.goal_torque,
                             ID);  //とりあえず怖いのでトルクOFF

    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);

    if (sleep_time_s < 0) {
      sleep_time_s = 0;
    }
    fprintf(crslave.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    usleep(sleep_time_s);
    t1++;
    if (t1 == 1000) {  // 1000サンプリングなので、2秒
      ch = 'b';
    }
  }

  if (ch != 'b') {
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Setoperation(POSITION_CONTROL_MODE, ID);
    crslave.Enable_Dynamixel_Torque(ID);
    crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    dprintf(sock, "%s", "**");
    close(sock);
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Close_port();
    fclose(crslave.ffp);
    return NULL;
  }

  ////////////////////////////////ここからバイラテ/////////////////////////////////////////
  printf(
      "===================================slave_controlstart==================="
      "=========\n");

  // int k = 0;

  while (ch == 'b') {  //データ取得の開始

    gettimeofday(&start_time_s, NULL);
    crslave.datareadflag = 0;

    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.dxl_addparam_result = crslave.groupBulkRead->addParam(
          ID[i], PRESENT_POSITION_ADDRESS,
          PRESENT_POSITION_DATA_LENGTH);  //読み込みのデータを設定(現在角度)
    }

    // Bulkread present position
    crslave.dxl_comm_result =
        crslave.groupBulkRead->txRxPacket();  //返信データの読み込み
    if (crslave.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
    // Check if groupbulkread data of Dynamixel is available
    for (int i = 0; i < JOINT_NUM2; i++) {  //返信データが利用できるか確認
      crslave.dxl_getdata_result = crslave.groupBulkRead->isAvailable(
          ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
      if (crslave.dxl_getdata_result != true) {
        crslave.datareadflag++;
      }
    }
    if (crslave.datareadflag == 0) {
      for (int i = 0; i < JOINT_NUM2; i++) {
        crslave.dxl_present_position = crslave.groupBulkRead->getData(
            ID[i], PRESENT_POSITION_ADDRESS,
            PRESENT_POSITION_DATA_LENGTH);  //返信データから指定のデータを読む
        crslave.present_position[i] =
            dxlvalue2rad(crslave.dxl_present_position);
      }
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.d_theta_res[i] =
          (crslave.present_position[i] - crslave.d_theta_temp[i]) * g[i];
      crslave.d_theta_temp[i] += crslave.d_theta_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crslave.d_theta_res[i]) >= LIMIT_SPEED[i]) {
        crslave.Disable_Dynamixel_Torque(ID);
        crslave.Setoperation(POSITION_CONTROL_MODE, ID);
        crslave.Enable_Dynamixel_Torque(ID);
        crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        printf("crslaveの軸%dが速いので終了\n", i);
        crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        dprintf(sock, "%s", "**");
        close(sock);
        crslave.Disable_Dynamixel_Torque(ID);
        crslave.Close_port();
        fclose(crslave.ffp);
        return NULL;
      }
    }

    memcpy(&fdw, &fds, sizeof(fd_set));
    memcpy(&fdr, &fds, sizeof(fd_set));

    if (passtime >=
        2.0) {  ////はじめから6秒経つまで通信はとりあえず行わないようにしている
      // nanosleep( &t ,NULL );// don't remove.
      ret = select(sock + 1, &fdr, &fdw, NULL, NULL);

      // if ( ret == 0 )	continue;
      // printf("///////  time : %5.3f  ///////\n",t2);

      if (t1 % rnn_ts == 0) {
        if (FD_ISSET(sock, &fdw) && sendf == true) {
          dprintf(sock,
                  "%5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f "
                  "%5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f %5.4f "
                  "%5.4f %5.4f %5.4f %5.4f %5.4f",
                  (float)crslave.present_position[0],
                  (float)crslave.present_position[1],
                  (float)crslave.present_position[2],
                  (float)crslave.present_position[3],
                  (float)crslave.present_position[4],
                  (float)crslave.present_position[5],
                  (float)crslave.present_position[6],
                  (float)crslave.present_position[7],
                  (float)crslave.d_theta_res[0], (float)crslave.d_theta_res[1],
                  (float)crslave.d_theta_res[2], (float)crslave.d_theta_res[3],
                  (float)crslave.d_theta_res[4], (float)crslave.d_theta_res[5],
                  (float)crslave.d_theta_res[6], (float)crslave.d_theta_res[7],
                  (float)crslave.tau_res[0], (float)crslave.tau_res[1],
                  (float)crslave.tau_res[2], (float)crslave.tau_res[3],
                  (float)crslave.tau_res[4], (float)crslave.tau_res[5],
                  (float)crslave.tau_res[6], (float)crslave.tau_res[7],
                  (float)passtime);

          // C++→pythonに送ったものを表示して確認
          printf("送った\n");
          printf(
              "\nangle\t\t:\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\n",
              (float)crslave.present_position[0],
              (float)crslave.present_position[1],
              (float)crslave.present_position[3],
              (float)crslave.present_position[4],
              (float)crslave.present_position[5],
              (float)crslave.present_position[6],
              (float)crslave.present_position[7]);
          printf(
              "d_theta_res\t\t:\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5."
              "4f\n",
              (float)crslave.d_theta_res[0], (float)crslave.d_theta_res[1],
              (float)crslave.d_theta_res[3], (float)crslave.d_theta_res[4],
              (float)crslave.d_theta_res[5], (float)crslave.d_theta_res[6],
              (float)crslave.d_theta_res[7]);
          printf(
              "tau_res\t\t:\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\t%5.4f\n",
              (float)crslave.tau_res[0], (float)crslave.tau_res[1],
              (float)crslave.tau_res[3], (float)crslave.tau_res[4],
              (float)crslave.tau_res[5], (float)crslave.tau_res[6],
              (float)crslave.tau_res[7]);
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

          if (tp == NULL) {
            cout << "break2だよ\n" << endl;
            break;
          }
          a[0] = atof(tp);

          for (
              int l = 1; l < 21;
              l++)  ////////////////////ココの数字は学習している軸（次元）の数で変更して
          {
            while (tp = strtok(NULL, ",")) {
              // printf("%s\n", tp);
              a[l] = atof(tp);
              // printf("a[%d] = %5.4f\n",l, a[l]);
              l++;
            }
          }
          printf("受け取った\n");
        }
      }
    }

    printf("a1\n");
    // pthread_mutex_lock(&mutex);// すぐなら4.2秒、少し待つなら5.5
    printf("a2\n");

    if (passtime <=
        4.3) {  // 4.2     通信始めてからLSTMがなれるまでマージンとってる
      for (int i = 0; i < JOINT_NUM2; i++) {
        crslave.goal_position[i] = goal_pose[i];
        crslave.goal_velocity[i] = 0.0;
        crslave.target_torque[i] = 0.0;
        p_th_s_res[i] = crslave.present_position[i];
        ;
        p_dth_s_res[i] = crslave.d_theta_res[i];
        p_tau_s_res[i] = crslave.tau_res[i];
      }
    } else {
      for (int i = 0; i < JOINT_NUM2; i++) {
        if (i == 2) {
          // p_th_s_res[i] = crslave.present_position[i];
          // p_dth_s_res[i] = crslave.d_theta_res[i];
          // p_tau_s_res[i] = crslave.tau_res[i];
          crslave.goal_position[i] = 3.14;
          crslave.goal_velocity[i] = 0.0;
          crslave.target_torque[i] = 0.0;
        } else if (i < 2) {
          // p_th_s_res[i] = crslave.present_position[i];
          // p_dth_s_res[i] = crslave.d_theta_res[i];
          // p_tau_s_res[i] = crslave.tau_res[i];
          crslave.goal_position[i] = a[i];
          crslave.goal_velocity[i] = a[i + (JOINT_NUM2 - 1) * 1];
          crslave.target_torque[i] = a[i + (JOINT_NUM2 - 1) * 2];

        } else {
          // p_th_s_res[i] = crslave.present_position[i];
          // p_dth_s_res[i] = crslave.d_theta_res[i];
          // p_tau_s_res[i] = crslave.tau_res[i];
          crslave.goal_position[i] = a[i - 1];
          crslave.goal_velocity[i] = a[i - 1 + (JOINT_NUM2 - 1) * 1];
          crslave.target_torque[i] = a[i - 1 + (JOINT_NUM2 - 1) * 2];
        }
      }
    }
    printf("b\n");
    // pthread_mutex_unlock(&mutex);
    printf("c\n");

    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.tau_p[i] =
          J[SLAVE][i] / 2.0 *
          (Kp[SLAVE][i] *
               (crslave.goal_position[i] - crslave.present_position[i]) +
           Kd[SLAVE][i] * (crslave.goal_velocity[i] - crslave.d_theta_res[i]));
      //力制御によるトルク参照値
      crslave.tau_f[i] =
          Kf[SLAVE][i] / 2.0 * (-crslave.target_torque[i] - crslave.tau_res[i]);
      // if(i == 2){
      //    crslave.goal_torque[i] = crslave.tau_p[i] + crslave.tau_f[i];
      //}
      // else{
      crslave.goal_torque[i] =
          crslave.tau_p[i] + crslave.tau_f[i] + crslave.tau_dis[i];
      //	}

      // DOB
      crslave.dob0[i] =
          crslave.goal_torque[i] + g[i] * J[SLAVE][i] * crslave.d_theta_res[i];
      crslave.dob1[i] = g[i] * (crslave.dob0[i] - crslave.dob2[i]);
      crslave.dob2[i] += crslave.dob1[i] * ts;

      //外乱トルクの算出
      crslave.tau_dis[i] =
          crslave.dob2[i] - g[i] * J[SLAVE][i] * crslave.d_theta_res[i];
    }

    crslave.tau_res[0] =
        crslave.tau_dis[0] - D[SLAVE][0] * crslave.d_theta_res[0];
    crslave.tau_res[1] =
        crslave.tau_dis[1] - M[SLAVE][0] * sin(crslave.present_position[1]) +
        M[SLAVE][1] *
            sin(crslave.present_position[1] + crslave.present_position[3]);
    crslave.tau_res[2] =
        crslave.tau_dis[2] - D[SLAVE][1] * crslave.d_theta_res[2];
    crslave.tau_res[3] =
        crslave.tau_dis[3] + M[SLAVE][2] * sin(crslave.present_position[1] +
                                               crslave.present_position[3]);
    crslave.tau_res[4] =
        crslave.tau_dis[4] - D[SLAVE][2] * crslave.d_theta_res[4];
    crslave.tau_res[5] =
        crslave.tau_dis[5] - D[SLAVE][3] * crslave.d_theta_res[5];
    crslave.tau_res[6] =
        crslave.tau_dis[6] - D[SLAVE][4] * crslave.d_theta_res[6];
    crslave.tau_res[7] =
        crslave.tau_dis[7] - D[SLAVE][5] * crslave.d_theta_res[7];

    // printf("crslave.target_torque[2] : %lf", crslave.target_torque[2]);

    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", passtime,
            crslave.present_position[0], crslave.present_position[1],
            crslave.present_position[2], crslave.present_position[3],
            crslave.present_position[4], crslave.present_position[5],
            crslave.present_position[6], crslave.present_position[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.d_theta_res[0], crslave.d_theta_res[1],
            crslave.d_theta_res[2], crslave.d_theta_res[3],
            crslave.d_theta_res[4], crslave.d_theta_res[5],
            crslave.d_theta_res[6], crslave.d_theta_res[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", crslave.tau_res[0],
            crslave.tau_res[1], crslave.tau_res[2], crslave.tau_res[3],
            crslave.tau_res[4], crslave.tau_res[5], crslave.tau_res[6],
            crslave.tau_res[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.goal_position[0], crslave.goal_position[1],
            crslave.goal_position[2], crslave.goal_position[3],
            crslave.goal_position[4], crslave.goal_position[5],
            crslave.goal_position[6], crslave.goal_position[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.goal_velocity[0], crslave.goal_velocity[1],
            crslave.goal_velocity[2], crslave.goal_velocity[3],
            crslave.goal_velocity[4], crslave.goal_velocity[5],
            crslave.goal_velocity[6], crslave.goal_velocity[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crslave.target_torque[0], crslave.target_torque[1],
            crslave.target_torque[2], crslave.target_torque[3],
            crslave.target_torque[4], crslave.target_torque[5],
            crslave.target_torque[6], crslave.target_torque[7]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[0], a[1], a[2], a[3],
            a[4], a[5], a[6]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[7], a[8], a[9],
            a[10], a[11], a[12], a[13]);
    fprintf(crslave.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,", a[14], a[15], a[16],
            a[17], a[18], a[19], a[20]);
    // fprintf( crslave.ffp,
    // "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",crslave.tau_p[0],crslave.tau_p[1],crslave.tau_p[2],crslave.tau_p[3],crslave.tau_p[4],crslave.tau_p[5],crslave.tau_p[6],crslave.tau_p[7]);
    // fprintf( crslave.ffp,
    // "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",crslave.tau_f[0],crslave.tau_f[1],crslave.tau_f[2],crslave.tau_f[3],crslave.tau_f[4],crslave.tau_f[5],crslave.tau_f[6],crslave.tau_f[7]);
    // fprintf( crslave.ffp,
    // "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",crslave.tau_dis[0],crslave.tau_dis[1],crslave.tau_dis[2],crslave.tau_dis[3],crslave.tau_dis[4],crslave.tau_dis[5],crslave.tau_dis[6],crslave.tau_dis[7]);

    crslave.setCranex7Torque(crslave.goal_torque, ID);  //こっちもトルクOFF

    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);

    if (sleep_time_s < 0) {
      sleep_time_s = 0;
    }
    fprintf(crslave.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    usleep(sleep_time_s);
    passtime += ts;
    t1++;
    printf("time: %lf\n", passtime);
  }

  crslave.Disable_Dynamixel_Torque(ID);
  crslave.Setoperation(POSITION_CONTROL_MODE, ID);
  crslave.Enable_Dynamixel_Torque(ID);
  crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  dprintf(sock, "%s", "**");
  close(sock);
  crslave.Disable_Dynamixel_Torque(ID);
  crslave.Close_port();
  fclose(crslave.ffp);
  return NULL;
}
////////////////////////////////**********************************:////////////////////////////////
////////////////////*****************************keyboard_check()***************/////////////////////////////////////////////////////

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
////////////////////////////////**********************************:////////////////////////////////
////////////////////////////////*****************main()*************:////////////////////////////////

/**
 * @fn		main()
 * @brief	main
 */
int main() {
  pthread_t slave_thread, getch_thread;

  for (int i = 0; i < JOINT_NUM; i++) {
    p_th_m_res[i] = 0.0;
    p_dth_m_res[i] = 0.0;
    p_ddth_m_res[i] = 0.0;
    p_th_s_res[i] = 0.0;
    p_dth_s_res[i] = 0.0;
    p_ddth_s_res[i] = 0.0;
    p_tau_m_res[i] = 0.0;
    p_tau_s_res[i] = 0.0;
  }

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
////////////////////////////////**********************************:////////////////////////////////