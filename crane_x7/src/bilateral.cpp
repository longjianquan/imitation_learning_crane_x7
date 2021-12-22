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

static int connect_mode = 0;

static double ts = 0.002;
// スレーブスレッドのループ回数（=データの管理番号）
int ttt = 0;
static char ch = 'p';
static double passtime = 0.0;

// static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

// static double p_th_m_res[JOINT_NUM], p_dth_m_res[JOINT_NUM],
//     p_ddth_m_res[JOINT_NUM];
// static double p_th_s_res[JOINT_NUM], p_dth_s_res[JOINT_NUM],
//     p_ddth_s_res[JOINT_NUM];
// static double p_tau_m_res[JOINT_NUM], p_tau_s_res[JOINT_NUM];

const char *devicename1 = "/dev/ttyUSB0";  //こっちがスレーブ
const char *devicename2 = "/dev/ttyUSB1";  //こっちがマスター

using namespace std;

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
// static int t_camera = 0;
// また上記の処理を一方のみ実行するためにフラグを管理
// static int sendf = true;
// ---------------------------- カメラ制御　グローバル変数
#define SEND_MODE 0
#define RECV_MODE 1
int sock_sendrecv_flag = SEND_MODE;
// カメラスレッドの初回実行かどうかの確認
// static int camera_count = 0;
// int camera_active_flag = true;
// int CameraTS = 20;

CR7 crane_s(devicename1, SLAVE);
CR7 crane_m(devicename2, MASTER);

void *crane_s_control(void *) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;

  // -------------- camera ------------------------
  // double control_time_camera;
  // long sleep_time_camera;
  // struct timeval start_time_camera;
  // struct timeval end_time_camera;

  /* --------- CR7 の型でcrane_sを定義 ----------- */
  if (!crane_s.Open_port()) return NULL;  // COMポートを開く
  if (!crane_s.Set_port_baudrate()) {     // 通信レートの設定
    crane_s.Close_port();                 // 通信ポートを閉じる
    return NULL;
  }
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  // 全サーボのトルクをON
  crane_s.Enable_Dynamixel_Torque(ID);
  // 設定されているgoal positionに移動（1回目ー＞アームが横に向く）
  crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);

  // 初期位置を設定
  // for (int i = 0; i < JOINT_NUM2; i++) {
  //   crane_s.theta_ref[i] = goal_pose[i];
  //   crane_s.omega_ref[i] = 0.0;
  //   crane_s.tau_ref[i] = 0.0;
  //   // p_th_s_res[i] = crane_s.theta_res[i];
  //   // p_dth_s_res[i] = crane_s.omega_res[i];
  //   // p_tau_s_res[i] = crane_s.tau_res[i];
  // }

  sleep(5);
  // 設定されているgoal positionに移動（1回目ー＞アームが正面に向く）;
  crane_s.Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  // 電源をOFFにしてから電流制御モードに移行
  crane_s.Disable_Dynamixel_Torque(ID);
  // 電流制御モード
  crane_s.Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);

  /**********************************************************
   *    モーションデータをテキストに保存（初期位置）         *
   ***********************************************************/
  printf("==========crane_s_p_controlstart==========\n");

  crane_s.Readtheta_res(ID);
  if ((crane_s.theta_res[0] == 0.0) || (crane_s.theta_res[7] == 0.0)) {
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
    crane_s.Enable_Dynamixel_Torque(ID);
    crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    printf("crane_s読み込み怪しいので終了\n");
    crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crane_s.Disable_Dynamixel_Torque(ID);
    crane_s.Close_port();
    fclose(crane_s.ffp);
    return NULL;
  }
  for (int j = 0; j < JOINT_NUM2; j++) {
    crane_s.d_theta_temp[j] = crane_s.theta_res[j];
  }

  /*********************************************************
    初回実行時のみ，カメラ保存用ソケットを追加する
  **********************************************************/
  // if (camera_count == 0) {
  //   printf("CONNECT SOCK \n");
  //   // ソケットの作成
  //   sock = socket(AF_INET, SOCK_STREAM, 0);
  //   // 接続先指定用構造体の準備(python側の設定を書く。宛先の設定)
  //   addr.sin_family = AF_INET;
  //   // ポート番号
  //   addr.sin_port = htons(10051);
  //   // このIPは自分自身(PC)を示す
  //   addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  //   // サーバに接続
  //   connect_mode = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
  //   // ファイルディスクリプタ集合の中身をクリア
  //   FD_ZERO(&fds);
  //   // ファイルディスクリプタ集合を設定
  //   FD_SET(sock, &fds);
  //   printf("CONNECT SOCK DONE\n");
  //   camera_count++;
  // }

  /*********************************************************
                         P MODE
  **********************************************************/
  while (ch == 'p') {
    // カメラとの接続が怪しい場合は終了
    // if ((connect_mode == -1) || (connect_mode == EINPROGRESS)) {
    //   crane_s.Disable_Dynamixel_Torque(ID);
    //   crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
    //   crane_s.Enable_Dynamixel_Torque(ID);
    //   crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    //   sleep(5);
    //   printf("カメラの読み込み怪しいので終了（S）\n");
    //   crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    //   sleep(5);
    //   crane_s.Disable_Dynamixel_Torque(ID);
    //   crane_s.Close_port();
    //   fclose(crane_s.ffp);
    //   return NULL;
    // }

    gettimeofday(&start_time_s, NULL);
    crane_s.Readtheta_res(ID);
    if ((crane_s.theta_res[0] == 0.0) || (crane_s.theta_res[7] == 0.0)) {
      crane_s.Disable_Dynamixel_Torque(ID);
      crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
      crane_s.Enable_Dynamixel_Torque(ID);
      crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      printf("crane_s読み込み怪しいので終了\n");
      crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
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
        printf("crane_sの軸%dが速いので終了\n", i);
        crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Close_port();
        fclose(crane_s.ffp);
        return NULL;
      }
    }

    // calculate input torque
    crane_s.position_control(goal_pose);

    // set torque
    // crane_s.setCranex7Torque(crane_s.goal_torque, ID);

    // 秒単位の時間を取得
    gettimeofday(&end_time_s, NULL);
    // (終了時間 - 開始時間) + (終了時間 - 開始時間) * 0.000,001
    control_time_s = (end_time_s.tv_sec - start_time_s.tv_sec +
                      (end_time_s.tv_usec - start_time_s.tv_usec) * 0.000001);
    // スリープ時間 = ループ周期(20[ms]) - 制御時間 * 1,000,000.0
    sleep_time_s = LOOPTIME - (long)(control_time_s * 1000000.0);
    // スリープ時間が0より下なら 0 にリセット
    if (sleep_time_s < 0) sleep_time_s = 0;

    // fprintf(crane_s.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
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
  printf("==========crane_s_b_controlstart==========\n");
  while (ch == 'b') {  //データ取得の開始
    gettimeofday(&start_time_s, NULL);
    crane_s.datareadflag = 0;

    // /***************************************************
    // pythonとソケット通信（pythonでカメラ保存）
    // ****************************************************/
    // memcpy(&fdw, &fds, sizeof(fd_set));
    // memcpy(&fdr, &fds, sizeof(fd_set));
    // //
    // カメラドライバが画像データの準備を完了するまでアプリケーションをウェイトさせておくには、
    // // select()システムコールを利用します
    // ret = select(sock + 1, &fdr, &fdw, NULL, NULL);

    // if ((t_camera % CameraTS) == 0) {
    //   // FD_ISSET ファイルディスクリプタがあるかどうか
    //   // fdwの中にsockの値が含まれているか調べる
    //   if (FD_ISSET(sock, &fdw) && sendf == true) {
    //     // ファイルディスクリプターに文字出力する
    //     // fnum: 保存するファイルのナンバー
    //     // passtime:システム経過時間 けど時刻の更新周期が
    //     // 別スレッドで2[ms]周期なので,同期が取れていない
    //     // 本スレッドではモーションとの時間計測誤差にプラマイ2[ms]
    //     // の誤差が生じる。この誤差自体が影響を与えることは少ないが
    //     // 画像データのファイル名に記録する時間が不安定だと
    //     // 別プログラムで参照するときに面倒となる
    //     // したがって本スレッド専用の時間変数を用意し、利用する
    //     dprintf(sock, "%d %5.4f", (int)1, (float)passtime);
    //     printf("送信\n");
    //     printf("\ntime\t\t:\tsave%d\t\t%8.4f\n", 1, passtime);
    //     // 次のデータを受け取るまで待つため，送信するフラグを下げる
    //     sendf = false;
    //   }
    // } else if (((t_camera + 1) % CameraTS) == 0) {
    //   // FD_ISSET ファイルディスクリプタがあるかどうか
    //   // fdwの中にsockの値が含まれているか調べる
    //   if (FD_ISSET(sock, &fdr) && sendf == false) {
    //     // sock: ソケット記述子
    //     // rbuf: データを受け取るバッファへのポインタ
    //     // ll: メッセージまたはデータグラムの長さ (バイト単位) を戻す
    //     ll = recv(sock, rbuf, sizeof(rbuf), 0);
    //     printf("受信\n");
    //     // データの終わり地点に0を入れる
    //     *(rbuf + ll) = 0;
    //     sendf = true;
    //     // 分解対象文字列 rbuf を "," を区切りに字句に分解
    //     // 字句（文字列の先頭）へのポインタを返す
    //     tp = strtok(rbuf, ",");
    //     // double型に変換
    //     a[0] = atof(tp);
    //   }
    // }
    // // カメラ実行周期(1[ms])のループカウンタ
    // t_camera++;
    // //-----------------------------------------------------------

    for (int i = 0; i < JOINT_NUM2; i++) {
      // 読み込みのデータを設定(現在角度)
      crane_s.dxl_addparam_result = crane_s.groupBulkRead->addParam(
          ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);
    }

    // Bulkread present position (返信データの読み込み)
    crane_s.dxl_comm_result = crane_s.groupBulkRead->txRxPacket();
    if (crane_s.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
    // Check if groupbulkread data of Dynamixel is available
    for (int i = 0; i < JOINT_NUM2; i++) {  // 返信データが利用できるか確認
      crane_s.dxl_getdata_result = crane_s.groupBulkRead->isAvailable(
          ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);
      if (crane_s.dxl_getdata_result != true) {
        crane_s.datareadflag++;
      }
    }
    if (crane_s.datareadflag == 0) {
      for (int i = 0; i < JOINT_NUM2;
           i++) {  // 返信データから指定のデータを読む
        crane_s.dxl_theta_res = crane_s.groupBulkRead->getData(
            ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);
        crane_s.theta_res[i] = dxlvalue2rad(crane_s.dxl_theta_res);
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
      crane_s.omega_res[i] =
          (crane_s.theta_res[i] - crane_s.d_theta_temp[i]) * g[i];
      crane_s.d_theta_temp[i] += crane_s.omega_res[i] * ts;
    }

    // 速度制限->  LIMIT_SPEED以上なら停止
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_s.omega_res[i]) >= LIMIT_SPEED[i]) {
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
        crane_s.Enable_Dynamixel_Torque(ID);
        crane_s.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        printf("crane_sの軸%dが速いので終了\n", i);
        crane_s.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        crane_s.Disable_Dynamixel_Torque(ID);
        crane_s.Close_port();
        fclose(crane_s.ffp);
        return NULL;
      }
    }

    // マスタ値を制御目標値にセット
    // pthread_mutex_lock(&mutex);
    // for (int i = 0; i < JOINT_NUM2; i++) {
    //   if (i == 2) {
    //     p_th_s_res[i] = crane_s.theta_res[i];
    //     p_dth_s_res[i] = crane_s.omega_res[i];
    //     p_tau_s_res[i] = crane_s.tau_res[i];
    //     crane_s.theta_ref[i] = 3.14;
    //     crane_s.omega_ref[i] = 0.0;
    //     crane_s.tau_ref[i] = 0.0;
    //   } else {
    //     p_th_s_res[i] = crane_s.theta_res[i];
    //     p_dth_s_res[i] = crane_s.omega_res[i];
    //     p_tau_s_res[i] = crane_s.tau_res[i];
    //     crane_s.theta_ref[i] = p_th_m_res[i];
    //     crane_s.omega_ref[i] = p_dth_m_res[i];
    //     crane_s.tau_ref[i] = p_tau_m_res[i];
    //   }
    // }
    // pthread_mutex_unlock(&mutex);

    // calculate input torque
    crane_s.torque_control(crane_m.theta_res, crane_m.omega_res,
                           crane_m.tau_res);

    // set torque
    // crane_s.setCranex7Torque(crane_s.goal_torque, ID);

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

    // fprintf(crane_s.ffp, "%ld,%lf\n", sleep_time_s, control_time_s);
    crane_s.write_csv(passtime, sleep_time_s, control_time_s);

    usleep(sleep_time_s);
    // ts = 0.002 [sec] = 2[ms]
    passtime += ts;
    ttt++;
  }

  crane_s.Disable_Dynamixel_Torque(ID);
  crane_s.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s.Enable_Dynamixel_Torque(ID);
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

  // COMポートを開く
  if (!crane_m.Open_port()) return NULL;
  //通信レートの設定
  if (!crane_m.Set_port_baudrate()) {
    //通信ポートを閉じる
    crane_m.Close_port();
    return NULL;
  }
  crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
  // 全サーボのトルクをON
  crane_m.Enable_Dynamixel_Torque(ID);
  // 設定されているgoal positionに移動
  crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);

  // for (int i = 0; i < JOINT_NUM2; i++) {
  //   crane_m.theta_ref[i] = goal_pose[i];
  //   crane_m.omega_ref[i] = 0.0;
  //   crane_m.tau_ref[i] = 0.0;
  //   // p_th_m_res[i] = crane_m.theta_res[i];
  //   // p_dth_m_res[i] = crane_m.omega_res[i];
  //   // p_tau_m_res[i] = crane_m.tau_res[i];
  // }
  sleep(5);
  crane_m.Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_m.Disable_Dynamixel_Torque(ID);
  crane_m.Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_m.Enable_Dynamixel_Torque(ID);

  printf("Press b  to start (or press q to quit)\n");
  printf("==========master_p_controlstart==========\n");

  crane_m.Readtheta_res(ID);
  if ((crane_m.theta_res[0] == 0.0) || (crane_m.theta_res[7] == 0.0)) {
    crane_m.Disable_Dynamixel_Torque(ID);
    crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
    crane_m.Enable_Dynamixel_Torque(ID);
    crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
    printf("master読み込み怪しいので終了\n");
    sleep(5);
    crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
    sleep(5);
    crane_m.Disable_Dynamixel_Torque(ID);
    crane_m.Close_port();
    fclose(crane_m.ffp);
    return NULL;
  }
  for (int j = 0; j < JOINT_NUM2; j++) {
    crane_m.d_theta_temp[j] = crane_m.theta_res[j];
  }
  /*********************************************************
            P MODE
  **********************************************************/
  while (ch == 'p') {
    // カメラとの接続が怪しい場合は終了
    if ((connect_mode == -1) || (connect_mode == EINPROGRESS)) {
      crane_m.Disable_Dynamixel_Torque(ID);
      crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
      crane_m.Enable_Dynamixel_Torque(ID);
      crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
      printf("ソケット読み込み怪しいので終了（M）\n");
      sleep(5);
      crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      crane_m.Disable_Dynamixel_Torque(ID);
      crane_m.Close_port();
      fclose(crane_m.ffp);
      return NULL;
    }
    gettimeofday(&start_time_m, NULL);
    crane_m.Readtheta_res(ID);
    if ((crane_m.theta_res[0] == 0.0) || (crane_m.theta_res[7] == 0.0)) {
      crane_m.Disable_Dynamixel_Torque(ID);
      crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
      crane_m.Enable_Dynamixel_Torque(ID);
      crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
      printf("master読み込み怪しいので終了\n");
      sleep(5);
      crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
      sleep(5);
      crane_m.Disable_Dynamixel_Torque(ID);
      crane_m.Close_port();
      fclose(crane_m.ffp);
      return NULL;
    }
    /////ここから位置制御入れる
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_m.omega_res[i] =
          (crane_m.theta_res[i] - crane_m.d_theta_temp[i]) * g[i];
      crane_m.d_theta_temp[i] += crane_m.omega_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_m.omega_res[i]) >= LIMIT_SPEED[i]) {
        crane_m.Disable_Dynamixel_Torque(ID);
        crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
        crane_m.Enable_Dynamixel_Torque(ID);
        crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
        printf("masterの軸%dが速いので終了\n", i);
        sleep(5);
        crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        crane_m.Disable_Dynamixel_Torque(ID);
        crane_m.Close_port();
        fclose(crane_m.ffp);
        return NULL;
      }
    }

    // calculate input torque
    crane_m.position_control(goal_pose);

    // set torque
    // crane_m.setCranex7Torque(crane_m.goal_torque, ID);

    /**********************************************************
      処理時間とループ時間からスリープ時間を割り出す(Pモード)
    ***********************************************************/
    gettimeofday(&end_time_m, NULL);
    control_time_m = (end_time_m.tv_sec - start_time_m.tv_sec +
                      (end_time_m.tv_usec - start_time_m.tv_usec) * 0.000001);
    sleep_time_m = LOOPTIME - (long)(control_time_m * 1000000.0);

    if (sleep_time_m < 0) sleep_time_m = 0;

    // fprintf(crane_m.ffp, "%ld,%lf\n", sleep_time_m, control_time_m);
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
  printf("=================master_b_controlstart==================\n");
  while (ch == 'b') {
    gettimeofday(&start_time_m, NULL);
    crane_m.datareadflag = 0;
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_m.dxl_addparam_result = crane_m.groupBulkRead->addParam(
          ID[i], THETA_RES_ADDRESS,
          THETA_RES_DATA_LENGTH);  //読み込みのデータを設定(現在角度)
    }

    // Bulkread present position
    crane_m.dxl_comm_result =
        crane_m.groupBulkRead->txRxPacket();  //返信データの読み込み
    if (crane_m.dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");
    // Check if groupbulkread data of Dynamixel is available
    for (int i = 0; i < JOINT_NUM2; i++) {  //返信データが利用できるか確認
      crane_m.dxl_getdata_result = crane_m.groupBulkRead->isAvailable(
          ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);
      if (crane_m.dxl_getdata_result != true) {
        crane_m.datareadflag++;
      }
    }
    if (crane_m.datareadflag == 0) {
      for (int i = 0; i < JOINT_NUM2; i++) {
        crane_m.dxl_theta_res = crane_m.groupBulkRead->getData(
            ID[i], THETA_RES_ADDRESS,
            THETA_RES_DATA_LENGTH);  //返信データから指定のデータを読む
        crane_m.theta_res[i] = dxlvalue2rad(crane_m.dxl_theta_res);
      }
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_m.omega_res[i] =
          (crane_m.theta_res[i] - crane_m.d_theta_temp[i]) * g[i];
      crane_m.d_theta_temp[i] += crane_m.omega_res[i] * ts;
    }

    for (int i = 0; i < JOINT_NUM2; i++) {
      if (fabs(crane_m.omega_res[i]) >= LIMIT_SPEED[i]) {
        crane_m.Disable_Dynamixel_Torque(ID);
        crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
        crane_m.Enable_Dynamixel_Torque(ID);
        crane_m.Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
        printf("masterの軸%dが速いので終了\n", i);
        sleep(5);
        crane_m.Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
        sleep(5);
        crane_m.Disable_Dynamixel_Torque(ID);
        crane_m.Close_port();
        fclose(crane_m.ffp);
        return NULL;
      }
    }

    // pthread_mutex_lock(&mutex);
    // for (int i = 0; i < JOINT_NUM2; i++) {
    //   if (i == 2) {
    //     p_th_m_res[i] = crane_m.theta_res[i];
    //     p_dth_m_res[i] = crane_m.omega_res[i];
    //     p_tau_m_res[i] = crane_m.tau_res[i];
    //     crane_m.theta_ref[i] = 3.14;
    //     crane_m.omega_ref[i] = 0.0;
    //     crane_m.tau_ref[i] = 0.0;
    //   } else {
    //     p_th_m_res[i] = crane_m.theta_res[i];
    //     p_dth_m_res[i] = crane_m.omega_res[i];
    //     p_tau_m_res[i] = crane_m.tau_res[i];
    //     crane_m.theta_ref[i] = p_th_s_res[i];
    //     crane_m.omega_ref[i] = p_dth_s_res[i];
    //     crane_m.tau_ref[i] = p_tau_s_res[i];
    //   }
    // }
    // pthread_mutex_unlock(&mutex);

    // calculate input torque
    crane_m.torque_control(crane_m.theta_res, crane_m.omega_res,
                           crane_m.tau_res);

    // set torque
    // crane_m.setCranex7Torque(crane_m.goal_torque, ID);

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

    // fprintf(crane_m.ffp, "%ld,%lf\n", sleep_time_m, control_time_m);
    crane_m.write_csv(passtime, sleep_time_m, control_time_m);

    usleep(sleep_time_m);
  }

  crane_m.Disable_Dynamixel_Torque(ID);
  crane_m.Setoperation(POSITION_CONTROL_MODE, ID);
  crane_m.Enable_Dynamixel_Torque(ID);
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
      dprintf(sock, "%s", "**");
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
