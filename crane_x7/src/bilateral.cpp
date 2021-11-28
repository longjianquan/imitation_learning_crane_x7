// *********    Example teaching play back    *********
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

// Dynamixel SDK libraryのインクルード
#include "crane.h"
#include "crane_x7_comm.h"
//#include "dynamixel_sdk.h"

// ソケット通信
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>


static int connect_mode = 0;
static int ID[JOINT_NUM] = {2, 3, 4, 5, 6, 7, 8, 9};  // サーボモータのID
static double JOINT_MIN[JOINT_NUM] = {
    262, 1024, 262, 228,
    262, 1024, 148, 1991};  // サーボモータの最小動作角(value)
static double JOINT_MAX[JOINT_NUM] = {
    3834, 3072, 3834, 2048,
    3834, 3072, 3948, 3072};  // サーボモータの最大動作角(value)
// static double	save_pose[JOINT_NUM]	=
// {	1.00/*1.68*/,		3.14/*3.27*/,		3.88,		1.40,
// 3.14, 3.14,		3.14,		3.9};//
// Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる static double
// goal_pose[JOINT_NUM]	=
// {	3.14,		3.32,		3.14,		1.57,		3.14,		2.0,
// 3.14,		3.84};//
// Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる static double
// finish_pose[JOINT_NUM]	=
// {	1.68,		3.14/*2.81*/,		3.14,
// 0.45/*0.81*/,		3.16,		3.14,		3.14,		3.9};//
// Move_goal_position関数の引数(rad)後でバイラテ用の位置調べる static double
// LIMIT_SPEED[JOINT_NUM]	=
// {	4.00,   2.50,	3.00,	4.00,	4.50,	5.00,	4.00,	4.00};
// // 速度の上限値

//********************サーボの位置制御モードでの動作位置の設定********************//
static double save_pose[JOINT_NUM] = {
    1.68, 3.14, 3.88, 1.71,
    3.14, 3.14, 3.14, 3.49134};  // 位置制御モードで一旦行く位置(rad)
static double goal_pose[JOINT_NUM] = {
    3.14, 3.14, 3.14, 1.38, 3.14,
    3.14, 3.14, 4.0};  // 位置制御モードからトルク制御モードに切り替わる時の位置(rad)
static double finish_pose[JOINT_NUM] = {
    1.68, 2.81, 3.14, 0.81,
    3.16, 3.14, 3.14, 3.49134};  // 動作終了時の位置(rad)

//********************サーボのトルク制御モードでの速度リミットの設定********************//
// static double	LIMIT_SPEED[JOINT_NUM]	=
// {	5.00,   2.50,	3.00,	4.50,	5.50,	5.50,	5.00,	5.00};
// // 速度リミット、これを超えたら動作が自動で停止する
static double LIMIT_SPEED[JOINT_NUM] = {
    3.00, 2.00, 2.00, 2.50, 4.50,
    4.50, 4.00, 6.00};  // 速度リミット、これを超えたら動作が自動で停止する

//ココはdegreeなので0中心
// システム時間　0.002[sec] ?
static double ts = 0.002;
// スレーブスレッドのループ回数（=データの管理番号）
int ttt = 0;
// static double g[JOINT_NUM] =
// { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
static char ch = 'p';
static double passtime = 0.0;

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
// static double J[2][JOINT_NUM] = {{0.012258, 0.11299, 0.012028, 0.04,
// 0.005676, 0.0066, 0.006281, 0.006891},{0.012258, 0.11299, 0.012028, 0.04,
// 0.005676, 0.0066, 0.006281, 0.006891}}; static double M[2][3] =
// {{2.094457, 1.1505, 1.18337},{2.094457, 1.1505, 1.18337}}; static double
// D[2][6] =
// {{0.0501,0.242,0.020,0.0391,0.0300,0.021},{0.0501,0.242,0.020,0.0391,0.0300,0.021}};

// static double Kp[2][JOINT_NUM] = {{256., 196, 900., 81., 256., 256., 100.,
// 256.},{256., 196, 625., 81., 256., 256., 100., 256.}}; static double
// Kd[2][JOINT_NUM] =
// {{32., 28., 60., 18., 32., 32., 20., 32.},{32., 28., 25., 18., 32., 32., 20.,
// 32.}}; static double Kf[2][JOINT_NUM] = {{0.50, 0.50, 1, 0.50, 0.85, 0.75,
// 0.75, 1.0},{0.50, 0.50, 1, 0.50, 0.85, 0.75, 0.75, 1.0}};

//********************物理パラメータ********************//
static double J[2][JOINT_NUM] = {
    {0.012258, 0.11299, 0.012028, 0.04, 0.005676, 0.0066, 0.006281, 0.006891},
    {0.012258, 0.11299, 0.012028, 0.04, 0.005676, 0.0066, 0.006281,
     0.006891}};  //慣性
static double M[2][3] = {{2.094457, 1.1505, 1.18337},
                         {2.094457, 1.1505, 1.18337}};  //重力補償係数
static double D[2][6] = {
    {0.0501, 0.242, 0.040, 0.0391, 0.0500, 0.021},
    {0.0501, 0.242, 0.040, 0.0391, 0.0500, 0.021}};  //摩擦補償係数

//********************ゲインとカットオフ周波数********************//
// static double Kp[2][JOINT_NUM] = {{196., 196, 900., 81., 256., 256., 100.,
// 256.},{196., 196, 625., 81., 256., 256., 100., 256.}}; //usleep時代のPゲイン
static double Kp[2][JOINT_NUM] = {
    {256., 196, 961., 144., 289., 324., 144., 324.},
    {256., 196, 961., 144., 289., 324., 144., 324.}};  // nanosleep時代のPゲイン
// static double Kd[2][JOINT_NUM] =
// {{28., 28., 60., 18., 32., 32., 20., 32.},{28., 28., 25., 18., 32., 32., 20.,
// 32.}};//usleep時代のDゲイン
static double Kd[2][JOINT_NUM] = {
    {40., 28., 66., 24., 34., 36., 24., 36.},
    {40., 28., 66., 24., 34., 36., 24., 36.}};  // nanosleep時代のDゲイン
// static double Kf[2][JOINT_NUM] = {{0.50, 0.50, 1, 0.50, 0.85, 0.75,
// 0.75, 1.0},{0.50, 0.50, 1, 0.50, 0.85, 0.75,
// 0.75, 1.0}};//usleep時代のFゲイン
static double Kf[2][JOINT_NUM] = {
    {0.70, 0.70, 1.0, 1.0, 0.80, 1.0, 0.80, 1.0},
    {0.70, 0.70, 1.0, 1.0, 0.80, 1.0, 0.80, 1.0}};  // nanosleep時代のFゲイン
static double g[JOINT_NUM] = {
    15.0, 15.0, 20.0, 20.0,
    20.0, 20.0, 20.0, 20.0};  //擬似微分、DOBともにカットオフ周波数

static double p_th_m_res[JOINT_NUM], p_dth_m_res[JOINT_NUM],
    p_ddth_m_res[JOINT_NUM];
static double p_th_s_res[JOINT_NUM], p_dth_s_res[JOINT_NUM],
    p_ddth_s_res[JOINT_NUM];
static double p_tau_m_res[JOINT_NUM], p_tau_s_res[JOINT_NUM];

const char *devicename1 = "/dev/ttyUSB0";  //こっちがスレーブ
const char *devicename2 = "/dev/ttyUSB1";  //こっちがマスター

using namespace std;

/************************************************
*************************************************
               camera_control()
*************************************************
*************************************************/
// ---------------------------- ソケット通信　グローバル変数
struct sockaddr_in addr;
int sock;
char rbuf[128];
// ファイルディスクリプタ集合
fd_set fds, fdw, fdr;
int ll;
int ret;
char *tp;
double a[1];
// ---------------------------- 周期時間管理
// カメラとのソケット通信で
// 40[ms]周期でカメラに信号（こちらのシステム時間）を送信する処理と
// 41[ms]周期でカメラから信号（ utf8("1") ）を受信する
// 上記の処理を分けて実行するためにカメラスレッドの実行周期を1[ms]にし
// カウンタによる制御から分けて実行する
static int t_camera = 0;
// また上記の処理を一方のみ実行するためにフラグを管理
static int sendf = true;
// ---------------------------- カメラ制御　グローバル変数
#define SEND_MODE 0
#define RECV_MODE 1
int sock_sendrecv_flag = SEND_MODE;
// カメラスレッドの初回実行かどうかの確認
static int camera_count = 0;
int camera_active_flag = true;
int CameraTS = 20;

/************************************************
*************************************************
               slave_control()
*************************************************
*************************************************/
void *slave_control(void *) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  // cout << "a";

  // -------------- camera ------------------------
  // double control_time_camera;
  // long sleep_time_camera;
  // struct timeval start_time_camera;
  // struct timeval end_time_camera;

  /* --------- CR7 の型でcrslaveを定義 ----------- */
  CR7 crslave(devicename1, SLAVE);
  if (!crslave.Open_port()) return NULL;  // COMポートを開く
  if (!crslave.Set_port_baudrate()) {     // 通信レートの設定
    crslave.Close_port();                 // 通信ポートを閉じる
    return NULL;
  }
  crslave.Setoperation(POSITION_CONTROL_MODE, ID);
  // 全サーボのトルクをON
  crslave.Enable_Dynamixel_Torque(ID);
  // 設定されているgoal positionに移動（1回目ー＞アームが横に向く）
  crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);

  // 初期位置を設定
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
  // cout << "slave_JOINT_NUM : " << JOINT_NUM << endl;
  // cout << "slave_JOINT_NUM2 : " << JOINT_NUM2 << endl
  // 設定されているgoal positionに移動（1回目ー＞アームが正面に向く）;
  crslave.Move_Goal_Position(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  // cout << "kokoato" << endl;
  // 電源をOFFにしてから電流制御モードに移行
  crslave.Disable_Dynamixel_Torque(ID);
  // 電流制御モード
  crslave.Setoperation(CURRENT_CONTROL_MODE, ID);
  crslave.Enable_Dynamixel_Torque(ID);

  /**********************************************************
   *    モーションデータをテキストに保存（初期位置）         *
   ***********************************************************/
  printf(
      "===================================slave_p_controlstart============"
      "====="
      "===========\n");
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
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Close_port();
    fclose(crslave.ffp);
    return NULL;
  }
  for (int j = 0; j < JOINT_NUM2; j++) {
    crslave.d_theta_temp[j] = crslave.present_position[j];
  }

  /*********************************************************
    初回実行時のみ，カメラ保存用ソケットを追加する
  **********************************************************/
  if (camera_count == 0) {
    printf("CONNECT SOCK \n");
    // ソケットの作成
    sock = socket(AF_INET, SOCK_STREAM, 0);
    // 接続先指定用構造体の準備(python側の設定を書く。宛先の設定)
    addr.sin_family = AF_INET;
    // ポート番号
    addr.sin_port = htons(10051);
    // このIPは自分自身(PC)を示す
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    // サーバに接続
    connect_mode = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    // ファイルディスクリプタ集合の中身をクリア
    FD_ZERO(&fds);
    // ファイルディスクリプタ集合を設定
    FD_SET(sock, &fds);
    printf("CONNECT SOCK DONE\n");
    camera_count++;
  }

  /*********************************************************
                         P MODE
  **********************************************************/
  while (ch == 'p') {
    // カメラとの接続が怪しい場合は終了
    if ((connect_mode == -1) || (connect_mode == EINPROGRESS)) {
      crslave.Disable_Dynamixel_Torque(ID);
      crslave.Setoperation(POSITION_CONTROL_MODE, ID);
      crslave.Enable_Dynamixel_Torque(ID);
      crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      printf("カメラの読み込み怪しいので終了（S）\n");
      crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      crslave.Disable_Dynamixel_Torque(ID);
      crslave.Close_port();
      fclose(crslave.ffp);
      return NULL;
    }
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
      // 力制御によるトルク参照値
      crslave.tau_f[i] =
          Kf[SLAVE][i] * (-crslave.target_torque[i] - crslave.tau_res[i]);
      if (i == 7) {
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

    /*******************************************************
     *    モーションデータをテキストに保存(Pモード)         *
     ********************************************************/
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
    crslave.setCranex7Torque(crslave.goal_torque, ID);
    // 秒単位の時間を取得
    gettimeofday(&end_time_s, NULL);
    // (終了時間 - 開始時間) + (終了時間 - 開始時間) * 0.000,001
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    // スリープ時間 = ループ周期(20[ms]) - 制御時間 * 1,000,000.0
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
    // スリープ時間が0より下なら 0 にリセット
    if (sleep_time_s < 0) {
      sleep_time_s = 0;
    }

    fprintf(crslave.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    usleep(sleep_time_s);
  }
  if (ch != 'b') {
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Setoperation(POSITION_CONTROL_MODE, ID);
    crslave.Enable_Dynamixel_Torque(ID);
    crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    printf("slave_バイラテ前にqで終了\n");
    sleep(5);
    crslave.Disable_Dynamixel_Torque(ID);
    crslave.Close_port();
    fclose(crslave.ffp);
    return NULL;
  }

  /*********************************************************
                        B MODE
  **********************************************************/
  printf(
      "===================================slave_b_controlstart============"
      "====="
      "===========\n");
  while (ch == 'b') {  //データ取得の開始
    gettimeofday(&start_time_s, NULL);
    crslave.datareadflag = 0;
    /***************************************************
    pythonとソケット通信（pythonでカメラ保存）
    ****************************************************/
    memcpy(&fdw, &fds, sizeof(fd_set));
    memcpy(&fdr, &fds, sizeof(fd_set));
    // カメラドライバが画像データの準備を完了するまでアプリケーションをウェイトさせておくには、
    // select()システムコールを利用します
    ret = select(sock + 1, &fdr, &fdw, NULL, NULL);

    if ((t_camera % CameraTS) == 0) {
      // FD_ISSET ファイルディスクリプタがあるかどうか
      // fdwの中にsockの値が含まれているか調べる
      if (FD_ISSET(sock, &fdw) && sendf == true) {
        // ファイルディスクリプターに文字出力する
        // fnum: 保存するファイルのナンバー
        // passtime:システム経過時間 けど時刻の更新周期が
        // 別スレッドで2[ms]周期なので,同期が取れていない
        // 本スレッドではモーションとの時間計測誤差にプラマイ2[ms]
        // の誤差が生じる。この誤差自体が影響を与えることは少ないが
        // 画像データのファイル名に記録する時間が不安定だと
        // 別プログラムで参照するときに面倒となる
        // したがって本スレッド専用の時間変数を用意し、利用する
        dprintf(sock, "%d %5.4f", (int)1, (float)passtime);
        printf("送信\n");
        printf("\ntime\t\t:\tsave%d\t\t%8.4f\n", 1, passtime);
        // 次のデータを受け取るまで待つため，送信するフラグを下げる
        sendf = false;
      }
    } else if (((t_camera + 1) % CameraTS) == 0) {
      // FD_ISSET ファイルディスクリプタがあるかどうか
      // fdwの中にsockの値が含まれているか調べる
      if (FD_ISSET(sock, &fdr) && sendf == false) {
        // sock: ソケット記述子
        // rbuf: データを受け取るバッファへのポインタ
        // ll: メッセージまたはデータグラムの長さ (バイト単位) を戻す
        ll = recv(sock, rbuf, sizeof(rbuf), 0);
        printf("受信\n");
        // データの終わり地点に0を入れる
        *(rbuf + ll) = 0;
        sendf = true;
        // 分解対象文字列 rbuf を "," を区切りに字句に分解
        // 字句（文字列の先頭）へのポインタを返す
        tp = strtok(rbuf, ",");
        // double型に変換
        a[0] = atof(tp);
      }
    }
    // カメラ実行周期(1[ms])のループカウンタ
    t_camera++;
    //-----------------------------------------------------------

    for (int i = 0; i < JOINT_NUM2; i++) {
      // 読み込みのデータを設定(現在角度)
      crslave.dxl_addparam_result = crslave.groupBulkRead->addParam(
          ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
    }

    // Bulkread present position (返信データの読み込み)
    crslave.dxl_comm_result = crslave.groupBulkRead->txRxPacket();
    if (crslave.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
    // Check if groupbulkread data of Dynamixel is available
    for (int i = 0; i < JOINT_NUM2; i++) {  // 返信データが利用できるか確認
      crslave.dxl_getdata_result = crslave.groupBulkRead->isAvailable(
          ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
      if (crslave.dxl_getdata_result != true) {
        crslave.datareadflag++;
      }
    }
    if (crslave.datareadflag == 0) {
      for (int i = 0; i < JOINT_NUM2;
           i++) {  // 返信データから指定のデータを読む
        crslave.dxl_present_position = crslave.groupBulkRead->getData(
            ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
        crslave.present_position[i] =
            dxlvalue2rad(crslave.dxl_present_position);
      }
    }
    //------------------------------ 角度の擬似微分
    //--------------------------------
    // g = カットオフ周波数[rad/s] = 40
    // ts = 時間[s] = 0.002 (ロボットの動作周期 1[ms])
    // [考え方]
    // その1
    // 1.フィードバックループの基本構造を考える
    //      +
    // x -----〇-- A ----> z
    //      - |____B___|
    // z/x = A / (1 + AB) これがフィードバックループの基本式
    //
    // 2.ここでローパスフィルタの伝達関数について考える
    // ローパスの伝達関数は g / (s + g)
    // これを分母分子をsで割って　(g/s) / (1 + (g/s))
    //
    // URL:カットオフ周波数について：https://suzumushi0.hatenablog.com/entry/2017/06/25/093113
    // URL:基本的な考え方について：https://www.maizuru-ct.ac.jp/control/kawata/study/book_lego/pdf_files/doc_002_support_package.pdf
    // URL:時系列データの入力方法：https://granite.phys.s.u-tokyo.ac.jp/tsubono/hiramatsu_seminar/%93d%8eq%89%f1%98H%81i10%8f%cd%8eb%81j.pdf
    //
    // 3.[1]のフィードバックループの構造に当てはめて考える
    // z/x = A / (1 + AB)  にあてはめると
    // z/x = (g/s) / (1 + (g/s)) となるため
    // A = g/s, B = 1 となる
    //
    // 4.構造について考える
    //       +    r
    // x(t) ---〇--- [g] --- [1/s] ------> z(t)
    //       - |                    |
    //         L____________________」
    // r = x(t) - z(t-1)
    // ここで 1/s はラプラスの式なので積分を表す
    // よって  z(t) = ∫rg dx
    // 積分なのでこれまでのサンプリング値にサンプリング時間を乗算したものの和をとればいい
    // これが出力 z(t) を示す
    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.d_theta_res[i] =
          (crslave.present_position[i] - crslave.d_theta_temp[i]) * g[i];
      crslave.d_theta_temp[i] += crslave.d_theta_res[i] * ts;
    }
    // 速度制限->  LIMIT_SPEED以上なら停止
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
        crslave.Disable_Dynamixel_Torque(ID);
        crslave.Close_port();
        fclose(crslave.ffp);
        return NULL;
      }
    }
    // マスタ値を制御目標値にセット
    pthread_mutex_lock(&mutex);
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (i == 2) {
        p_th_s_res[i] = crslave.present_position[i];
        p_dth_s_res[i] = crslave.d_theta_res[i];
        p_tau_s_res[i] = crslave.tau_res[i];
        crslave.goal_position[i] = 3.14;
        crslave.goal_velocity[i] = 0.0;
        crslave.target_torque[i] = 0.0;
      } else {
        p_th_s_res[i] = crslave.present_position[i];
        p_dth_s_res[i] = crslave.d_theta_res[i];
        p_tau_s_res[i] = crslave.tau_res[i];
        crslave.goal_position[i] = p_th_m_res[i];
        crslave.goal_velocity[i] = p_dth_m_res[i];
        crslave.target_torque[i] = p_tau_m_res[i];
      }
    }

    pthread_mutex_unlock(&mutex);
    for (int i = 0; i < JOINT_NUM2; i++) {
      crslave.tau_p[i] =
          J[SLAVE][i] / 2.0 *
          (Kp[SLAVE][i] *
               (crslave.goal_position[i] - crslave.present_position[i]) +
           Kd[SLAVE][i] * (crslave.goal_velocity[i] - crslave.d_theta_res[i]));
      //力制御によるトルク参照値
      crslave.tau_f[i] =
          Kf[SLAVE][i] / 2.0 * (-crslave.target_torque[i] - crslave.tau_res[i]);
      if (i == 7) {
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
    //トルク応答の算出
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

    /*******************************************************
     *    モーションデータをテキストに保存(Bモード)         *
     ********************************************************/
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
    // トルクをセット
    crslave.setCranex7Torque(crslave.goal_torque, ID);

    if ((ttt % 10) == 0) {
      printf("time: %lf\n", passtime);
    }
    /**********************************************************
      処理時間とループ時間からスリープ時間を割り出す(Bモード)
    ***********************************************************/
    gettimeofday(&end_time_s, NULL);
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    // スリープ時間 = ループ周期(2000[us]=2[ms]) - 制御時間 * 1,000,000.0
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);

    if (sleep_time_s < 0) {
      sleep_time_s = 0;
    }
    fprintf(crslave.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    usleep(sleep_time_s);
    // ts = 0.002 [sec] = 2[ms]
    passtime += ts;
    ttt++;
  }
  crslave.Disable_Dynamixel_Torque(ID);
  crslave.Setoperation(POSITION_CONTROL_MODE, ID);
  crslave.Enable_Dynamixel_Torque(ID);
  crslave.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crslave.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crslave.Disable_Dynamixel_Torque(ID);
  crslave.Close_port();
  fclose(crslave.ffp);
  return NULL;
}

/****************************************************************
*****************************************************************
                  master_control()
*****************************************************************
*****************************************************************/
void *master_control(void *) {
  double control_time_m;
  long sleep_time_m;
  struct timeval start_time_m;
  struct timeval end_time_m;

  CR7 crmaster(devicename2, MASTER);
  // COMポートを開く
  if (!crmaster.Open_port()) return NULL;
  //通信レートの設定
  if (!crmaster.Set_port_baudrate()) {
    //通信ポートを閉じる
    crmaster.Close_port();
    return NULL;
  }
  crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
  // 全サーボのトルクをON
  crmaster.Enable_Dynamixel_Torque(ID);
  // 設定されているgoal positionに移動
  crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);

  for (int i = 0; i < JOINT_NUM2; i++) {
    crmaster.goal_position[i] = goal_pose[i];
    crmaster.goal_velocity[i] = 0.0;
    crmaster.target_torque[i] = 0.0;
    p_th_m_res[i] = crmaster.present_position[i];
    p_dth_m_res[i] = crmaster.d_theta_res[i];
    p_tau_m_res[i] = crmaster.tau_res[i];
  }
  sleep(5);
  crmaster.Move_Goal_Position(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crmaster.Disable_Dynamixel_Torque(ID);
  crmaster.Setoperation(CURRENT_CONTROL_MODE, ID);
  crmaster.Enable_Dynamixel_Torque(ID);

  printf("Press b  to start (or press q to quit)\n");
  printf(
      "===================================master_p_controlstart==========="
      "====="
      "============\n");
  crmaster.ffp = fopen(crmaster.filename2.c_str(), "w");
  fprintf(crmaster.ffp,
          "time,m_presentposition[0],m_presentposition[1],m_presentposition[2],"
          "m_presentposition[3],m_presentposition[4],m_presentposition[5],m_"
          "presentposition[6],m_presentposition[7],");
  fprintf(crmaster.ffp,
          "m_presentvelocity[0],m_presentvelocity[1],m_presentvelocity[2],m_"
          "presentvelocity[3],m_presentvelocity[4],m_presentvelocity[5],m_"
          "presentvelocity[6],m_presentvelocity[7],");
  fprintf(crmaster.ffp,
          "m_tau_res[0],m_tau_res[1],m_tau_res[2],m_tau_res[3],m_tau_res[4],m_"
          "tau_res[5],m_tau_res[6],m_tau_res[7],");
  fprintf(crmaster.ffp,
          "s_presentposition[0],s_presentposition[1],s_presentposition[2],s_"
          "presentposition[3],s_presentposition[4],s_presentposition[5],s_"
          "presentposition[6],s_presentposition[7],");
  fprintf(crmaster.ffp,
          "s_presentvelocity[0],s_presentvelocity[1],s_presentvelocity[2],s_"
          "presentvelocity[3],s_presentvelocity[4],s_presentvelocity[5],s_"
          "presentvelocity[6],s_presentvelocity[7],");
  fprintf(crmaster.ffp,
          "s_tau_res[0],s_tau_res[1],s_tau_res[2],s_tau_res[3],s_tau_res[4],s_"
          "tau_res[5],s_tau_res[6],s_tau_res[7],");
  fprintf(crmaster.ffp, "sleeptime,controltime\n");

  crmaster.Readpresent_position(ID);
  if ((crmaster.present_position[0] == 0.0) ||
      (crmaster.present_position[7] == 0.0)) {
    crmaster.Disable_Dynamixel_Torque(ID);
    crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
    crmaster.Enable_Dynamixel_Torque(ID);
    crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
    printf("master読み込み怪しいので終了\n");
    sleep(5);
    crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crmaster.Disable_Dynamixel_Torque(ID);
    crmaster.Close_port();
    fclose(crmaster.ffp);
    return NULL;
  }
  for (int j = 0; j < JOINT_NUM2; j++) {
    crmaster.d_theta_temp[j] = crmaster.present_position[j];
  }
  /*********************************************************
            P MODE
  **********************************************************/
  while (ch == 'p') {
    // カメラとの接続が怪しい場合は終了
    if ((connect_mode == -1) || (connect_mode == EINPROGRESS)) {
      crmaster.Disable_Dynamixel_Torque(ID);
      crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
      crmaster.Enable_Dynamixel_Torque(ID);
      crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
      printf("ソケット読み込み怪しいので終了（M）\n");
      sleep(5);
      crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      crmaster.Disable_Dynamixel_Torque(ID);
      crmaster.Close_port();
      fclose(crmaster.ffp);
      return NULL;
    }
    gettimeofday(&start_time_m, NULL);
    crmaster.Readpresent_position(ID);
    if ((crmaster.present_position[0] == 0.0) ||
        (crmaster.present_position[7] == 0.0)) {
      crmaster.Disable_Dynamixel_Torque(ID);
      crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
      crmaster.Enable_Dynamixel_Torque(ID);
      crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
      printf("master読み込み怪しいので終了\n");
      sleep(5);
      crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      crmaster.Disable_Dynamixel_Torque(ID);
      crmaster.Close_port();
      fclose(crmaster.ffp);
      return NULL;
    }
    /////ここから位置制御入れる
    for (int i = 0; i < JOINT_NUM2; i++) {
      crmaster.d_theta_res[i] =
          (crmaster.present_position[i] - crmaster.d_theta_temp[i]) * g[i];
      crmaster.d_theta_temp[i] += crmaster.d_theta_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crmaster.d_theta_res[i]) >= LIMIT_SPEED[i]) {
        crmaster.Disable_Dynamixel_Torque(ID);
        crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
        crmaster.Enable_Dynamixel_Torque(ID);
        crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
        printf("masterの軸%dが速いので終了\n", i);
        sleep(5);
        crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        crmaster.Disable_Dynamixel_Torque(ID);
        crmaster.Close_port();
        fclose(crmaster.ffp);
        return NULL;
      }
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crmaster.tau_p[i] =
          J[MASTER][i] * (Kp[MASTER][i] * (crmaster.goal_position[i] -
                                           crmaster.present_position[i]) +
                          Kd[MASTER][i] * (crmaster.goal_velocity[i] -
                                           crmaster.d_theta_res[i]));
      //力制御によるトルク参照値
      crmaster.tau_f[i] =
          Kf[MASTER][i] * (-crmaster.target_torque[i] - crmaster.tau_res[i]);
      if (i == 7) {
        crmaster.goal_torque[i] =
            crmaster.tau_p[i] + crmaster.tau_f[i] + crmaster.tau_dis[i];
      } else {
        crmaster.goal_torque[i] =
            crmaster.tau_p[i] + crmaster.tau_f[i] + crmaster.tau_dis[i];
      }
      // DOB
      crmaster.dob0[i] = crmaster.goal_torque[i] +
                         g[i] * J[MASTER][i] * crmaster.d_theta_res[i];
      crmaster.dob1[i] = g[i] * (crmaster.dob0[i] - crmaster.dob2[i]);
      crmaster.dob2[i] += crmaster.dob1[i] * ts;
      //外乱トルクの算出
      crmaster.tau_dis[i] =
          crmaster.dob2[i] - g[i] * J[MASTER][i] * crmaster.d_theta_res[i];
    }
    //トルク応答の算出
    crmaster.tau_res[0] =
        crmaster.tau_dis[0] - D[MASTER][0] * crmaster.d_theta_res[0];
    crmaster.tau_res[1] =
        crmaster.tau_dis[1] - M[MASTER][0] * sin(crmaster.present_position[1]) +
        M[MASTER][1] *
            sin(crmaster.present_position[1] + crmaster.present_position[3]);
    crmaster.tau_res[2] =
        crmaster.tau_dis[2] - D[MASTER][1] * crmaster.d_theta_res[2];
    crmaster.tau_res[3] =
        crmaster.tau_dis[3] + M[MASTER][2] * sin(crmaster.present_position[1] +
                                                 crmaster.present_position[3]);
    crmaster.tau_res[4] =
        crmaster.tau_dis[4] - D[MASTER][2] * crmaster.d_theta_res[4];
    crmaster.tau_res[5] =
        crmaster.tau_dis[5] - D[MASTER][3] * crmaster.d_theta_res[5];
    crmaster.tau_res[6] =
        crmaster.tau_dis[6] - D[MASTER][4] * crmaster.d_theta_res[6];
    crmaster.tau_res[7] =
        crmaster.tau_dis[7] - D[MASTER][5] * crmaster.d_theta_res[7];
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", passtime,
            crmaster.present_position[0], crmaster.present_position[1],
            crmaster.present_position[2], crmaster.present_position[3],
            crmaster.present_position[4], crmaster.present_position[5],
            crmaster.present_position[6], crmaster.present_position[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.d_theta_res[0], crmaster.d_theta_res[1],
            crmaster.d_theta_res[2], crmaster.d_theta_res[3],
            crmaster.d_theta_res[4], crmaster.d_theta_res[5],
            crmaster.d_theta_res[6], crmaster.d_theta_res[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.tau_res[0], crmaster.tau_res[1], crmaster.tau_res[2],
            crmaster.tau_res[3], crmaster.tau_res[4], crmaster.tau_res[5],
            crmaster.tau_res[6], crmaster.tau_res[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.goal_position[0], crmaster.goal_position[1],
            crmaster.goal_position[2], crmaster.goal_position[3],
            crmaster.goal_position[4], crmaster.goal_position[5],
            crmaster.goal_position[6], crmaster.goal_position[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.goal_velocity[0], crmaster.goal_velocity[1],
            crmaster.goal_velocity[2], crmaster.goal_velocity[3],
            crmaster.goal_velocity[4], crmaster.goal_velocity[5],
            crmaster.goal_velocity[6], crmaster.goal_velocity[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.target_torque[0], crmaster.target_torque[1],
            crmaster.target_torque[2], crmaster.target_torque[3],
            crmaster.target_torque[4], crmaster.target_torque[5],
            crmaster.target_torque[6], crmaster.target_torque[7]);
    crmaster.setCranex7Torque(crmaster.goal_torque, ID);
    /**********************************************************
      処理時間とループ時間からスリープ時間を割り出す(Pモード)
    ***********************************************************/
    gettimeofday(&end_time_m, NULL);
    control_time_m = (end_time_m.tv_sec - start_time_m.tv_sec +
                      (end_time_m.tv_usec - start_time_m.tv_usec) * 0.000001);
    sleep_time_m = LOOPTIME - (long)(control_time_m * 1000000.0);

    if (sleep_time_m < 0) {
      sleep_time_m = 0;
    }

    fprintf(crmaster.ffp, "%ld,%lf\n", sleep_time_m, control_time_m);
    usleep(sleep_time_m);
  }

  if (ch != 'b') {
    crmaster.Disable_Dynamixel_Torque(ID);
    crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
    crmaster.Enable_Dynamixel_Torque(ID);
    crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    printf("master_バイラテ前にqで終了\n");
    crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crmaster.Disable_Dynamixel_Torque(ID);
    crmaster.Close_port();
    fclose(crmaster.ffp);
    return NULL;
  }

  /*********************************************************
            B MODE (ここからがバイラテ)
  **********************************************************/
  printf("=================master_b_controlstart==================\n");
  while (ch == 'b') {
    gettimeofday(&start_time_m, NULL);
    crmaster.datareadflag = 0;
    for (int i = 0; i < JOINT_NUM2; i++) {
      crmaster.dxl_addparam_result = crmaster.groupBulkRead->addParam(
          ID[i], PRESENT_POSITION_ADDRESS,
          PRESENT_POSITION_DATA_LENGTH);  //読み込みのデータを設定(現在角度)
    }

    // Bulkread present position
    crmaster.dxl_comm_result =
        crmaster.groupBulkRead->txRxPacket();  //返信データの読み込み
    if (crmaster.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
    // Check if groupbulkread data of Dynamixel is available
    for (int i = 0; i < JOINT_NUM2; i++) {  //返信データが利用できるか確認
      crmaster.dxl_getdata_result = crmaster.groupBulkRead->isAvailable(
          ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
      if (crmaster.dxl_getdata_result != true) {
        crmaster.datareadflag++;
      }
    }
    if (crmaster.datareadflag == 0) {
      for (int i = 0; i < JOINT_NUM2; i++) {
        crmaster.dxl_present_position = crmaster.groupBulkRead->getData(
            ID[i], PRESENT_POSITION_ADDRESS,
            PRESENT_POSITION_DATA_LENGTH);  //返信データから指定のデータを読む
        crmaster.present_position[i] =
            dxlvalue2rad(crmaster.dxl_present_position);
      }
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crmaster.d_theta_res[i] =
          (crmaster.present_position[i] - crmaster.d_theta_temp[i]) * g[i];
      crmaster.d_theta_temp[i] += crmaster.d_theta_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crmaster.d_theta_res[i]) >= LIMIT_SPEED[i]) {
        crmaster.Disable_Dynamixel_Torque(ID);
        crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
        crmaster.Enable_Dynamixel_Torque(ID);
        crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
        printf("masterの軸%dが速いので終了\n", i);
        sleep(5);
        crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        crmaster.Disable_Dynamixel_Torque(ID);
        crmaster.Close_port();
        fclose(crmaster.ffp);
        return NULL;
      }
    }

    pthread_mutex_lock(&mutex);
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (i == 2) {
        p_th_m_res[i] = crmaster.present_position[i];
        p_dth_m_res[i] = crmaster.d_theta_res[i];
        p_tau_m_res[i] = crmaster.tau_res[i];
        crmaster.goal_position[i] = 3.14;
        crmaster.goal_velocity[i] = 0.0;
        crmaster.target_torque[i] = 0.0;
      } else {
        p_th_m_res[i] = crmaster.present_position[i];
        p_dth_m_res[i] = crmaster.d_theta_res[i];
        p_tau_m_res[i] = crmaster.tau_res[i];
        crmaster.goal_position[i] = p_th_s_res[i];
        crmaster.goal_velocity[i] = p_dth_s_res[i];
        crmaster.target_torque[i] = p_tau_s_res[i];
      }
    }
    pthread_mutex_unlock(&mutex);

    for (int i = 0; i < JOINT_NUM2; i++) {
      crmaster.tau_p[i] =
          J[MASTER][i] / 2.0 *
          (Kp[MASTER][i] *
               (crmaster.goal_position[i] - crmaster.present_position[i]) +
           Kd[MASTER][i] *
               (crmaster.goal_velocity[i] - crmaster.d_theta_res[i]));
      //力制御によるトルク参照値
      crmaster.tau_f[i] = Kf[MASTER][i] / 2.0 *
                          (-crmaster.target_torque[i] - crmaster.tau_res[i]);
      if (i == 7) {
        crmaster.goal_torque[i] =
            crmaster.tau_p[i] + crmaster.tau_f[i] + crmaster.tau_dis[i];
      } else {
        crmaster.goal_torque[i] =
            crmaster.tau_p[i] + crmaster.tau_f[i] + crmaster.tau_dis[i];
      }
      // DOB
      crmaster.dob0[i] = crmaster.goal_torque[i] +
                         g[i] * J[MASTER][i] * crmaster.d_theta_res[i];
      crmaster.dob1[i] = g[i] * (crmaster.dob0[i] - crmaster.dob2[i]);
      crmaster.dob2[i] += crmaster.dob1[i] * ts;
      //外乱トルクの算出
      crmaster.tau_dis[i] =
          crmaster.dob2[i] - g[i] * J[MASTER][i] * crmaster.d_theta_res[i];
    }

    crmaster.tau_res[0] =
        crmaster.tau_dis[0] - D[MASTER][0] * crmaster.d_theta_res[0];
    crmaster.tau_res[1] =
        crmaster.tau_dis[1] - M[MASTER][0] * sin(crmaster.present_position[1]) +
        M[MASTER][1] *
            sin(crmaster.present_position[1] + crmaster.present_position[3]);
    crmaster.tau_res[2] =
        crmaster.tau_dis[2] - D[MASTER][1] * crmaster.d_theta_res[2];
    crmaster.tau_res[3] =
        crmaster.tau_dis[3] + M[MASTER][2] * sin(crmaster.present_position[1] +
                                                 crmaster.present_position[3]);
    crmaster.tau_res[4] =
        crmaster.tau_dis[4] - D[MASTER][2] * crmaster.d_theta_res[4];
    crmaster.tau_res[5] =
        crmaster.tau_dis[5] - D[MASTER][3] * crmaster.d_theta_res[5];
    crmaster.tau_res[6] =
        crmaster.tau_dis[6] - D[MASTER][4] * crmaster.d_theta_res[6];
    crmaster.tau_res[7] =
        crmaster.tau_dis[7] - D[MASTER][5] * crmaster.d_theta_res[7];

    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,", passtime,
            crmaster.present_position[0], crmaster.present_position[1],
            crmaster.present_position[2], crmaster.present_position[3],
            crmaster.present_position[4], crmaster.present_position[5],
            crmaster.present_position[6], crmaster.present_position[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.d_theta_res[0], crmaster.d_theta_res[1],
            crmaster.d_theta_res[2], crmaster.d_theta_res[3],
            crmaster.d_theta_res[4], crmaster.d_theta_res[5],
            crmaster.d_theta_res[6], crmaster.d_theta_res[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.tau_res[0], crmaster.tau_res[1], crmaster.tau_res[2],
            crmaster.tau_res[3], crmaster.tau_res[4], crmaster.tau_res[5],
            crmaster.tau_res[6], crmaster.tau_res[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.goal_position[0], crmaster.goal_position[1],
            crmaster.goal_position[2], crmaster.goal_position[3],
            crmaster.goal_position[4], crmaster.goal_position[5],
            crmaster.goal_position[6], crmaster.goal_position[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.goal_velocity[0], crmaster.goal_velocity[1],
            crmaster.goal_velocity[2], crmaster.goal_velocity[3],
            crmaster.goal_velocity[4], crmaster.goal_velocity[5],
            crmaster.goal_velocity[6], crmaster.goal_velocity[7]);
    fprintf(crmaster.ffp, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,",
            crmaster.target_torque[0], crmaster.target_torque[1],
            crmaster.target_torque[2], crmaster.target_torque[3],
            crmaster.target_torque[4], crmaster.target_torque[5],
            crmaster.target_torque[6], crmaster.target_torque[7]);
    crmaster.setCranex7Torque(crmaster.goal_torque, ID);
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
    fprintf(crmaster.ffp, "%ld,%lf\n", sleep_time_m, control_time_m);
    usleep(sleep_time_m);
  }

  crmaster.Disable_Dynamixel_Torque(ID);
  crmaster.Setoperation(POSITION_CONTROL_MODE, ID);
  crmaster.Enable_Dynamixel_Torque(ID);
  crmaster.Move_Goal_Position(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crmaster.Move_Goal_Position(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crmaster.Disable_Dynamixel_Torque(ID);
  crmaster.Close_port();
  fclose(crmaster.ffp);
  return NULL;
}

/*****************************
   keyboard_check()
******************************/
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
        t_camera = 0;
        camera_active_flag = true;
      }
    }
    /************************
     *     Q MODE
     *************************/
    else if (key == 'q') {
      ch = 'q';
      printf("MODE Q ACTIVE\n");
      dprintf(sock, "%s", "**");
      break;
    }
    pre_ch = ch;
  }
  return NULL;
}

/*****************************
    main()
******************************/
/**
 * @fn		main()
 * @brief	main
 */
int main() {
  pthread_t master_thread, slave_thread, getch_thread;
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
  // カメラ制御のスレッド設定
  /*if (pthread_create(&camera_thread, NULL, &camera_control, NULL) != 0)
  {
      fprintf(stderr, "cannot create control thread\n");
      return 1;
  }*/
  // スレッド開始
  pthread_join(master_thread, NULL);
  pthread_join(slave_thread, NULL);
  pthread_join(getch_thread, NULL);
  // pthread_join(camera_thread, NULL);
  return 0;
}
