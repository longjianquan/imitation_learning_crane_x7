/**
 * @file crane_x7_comm.h
 * @brief Wrapper functions of DynamixelSDK
 * @author RT Corporation
 * @date 2019-2020
 * @copyright License: Apache License, Version 2.0
 */
// Copyright 2020 RT Corporation
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

#ifndef CRANE_H_
#define CRANE_H_

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

#include "dynamixel_sdk.h"

#ifndef JOINT_NUM
#define JOINT_NUM (8)
#endif

/**
 * @brief CRANE-x7 動作用クラス
 */
class CR7 {
 public:
  // Initialize PortHandler instance
  // Set the port path
  // Get methods and members of PortHandlerLinux or PortHandlerWindows
  // 使用するポートの各種設定(Dynamixel SDK)
  dynamixel::PortHandler *portHandler;

  // Initialize PacketHandler instance
  // Set the protocol version
  // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
  // サーボモータと通信するためのパケットの設定(Dynamixel SDK)
  dynamixel::PacketHandler *packetHandler;

  // Initialize GroupBulkWrite instance
  // 複数サーボの書き込み用関数の呼び出し(Dynamixel SDK)
  dynamixel::GroupBulkWrite *groupBulkWrite;

  // Initialize GroupBulkRead instance
  // 複数サーボの読み込み用関数の呼び出し(Dynamixel SDK)
  dynamixel::GroupBulkRead *groupBulkRead;

  CR7();  //コンストラクタ
  CR7(const char *devicename, int masterorslave);
  ~CR7() { printf("finish CR7\n"); }

  int ms;

  //各種エラー出力用変数
  int dxl_comm_result;       // Communication result
  bool dxl_addparam_result;  // addParam result
  bool dxl_getdata_result;   // GetParam result
  uint8_t dxl_error;         // Dynamixel error

  uint8_t param_theta_ref[4];       //通信パケット用に変換したtheta_refの変数
  uint8_t param_goal_current[4];    //通信パケット用に変換したtheta_refの変数
  uint8_t param_value[4];           //通信パケット用に変換したvalueの変数

  int32_t dxl_theta_res;            //サーボの現在位置取得用の変数
  int16_t dxl_present_velocity;     //サーボの現在位置取得用の変数
  int16_t dxl_present_torque;       //サーボの現在位置取得用の変数
  int16_t goal_current[JOINT_NUM];  // = {0,0,0,0,0,0,0,0};

  double theta_res[JOINT_NUM];
  double present_velocity[JOINT_NUM];
  double present_torque[JOINT_NUM];

  double theta_ref[JOINT_NUM];
  double omega_ref[JOINT_NUM];
  double tau_ref[JOINT_NUM];
  double goal_torque[JOINT_NUM];

  double omega_res[JOINT_NUM];
  double d_theta_temp[JOINT_NUM];
  double tau_p[JOINT_NUM];
  double tau_f[JOINT_NUM];
  double tau_dis[JOINT_NUM];
  double tau_res[JOINT_NUM];

  double dob0[JOINT_NUM];
  double dob1[JOINT_NUM];
  double dob2[JOINT_NUM];

  FILE *ffp;
  // const char *fname;
  std::string filename;
  int datareadflag;

  //　play back用テキストデータの定義
  // FILE *fp;
  // const char *fname = "data.txt";

  bool Open_port();                                 //通信ポートを開く
  bool Set_port_baudrate();                         //通信レートの設定
  void Enable_Dynamixel_Torque(int ID[JOINT_NUM]);  //全サーボのトルクをON
  void Disable_Dynamixel_Torque(int ID[JOINT_NUM]);  //全サーボのトルクをOFF
  void Move_Theta_Ref(
      double *goal_pose, int ID[JOINT_NUM], double JOINT_MIN[JOINT_NUM],
      double JOINT_MAX[JOINT_NUM]);  //設定されているgoal positionに移動
  void Move_Offset_Position(
      int ID[JOINT_NUM]);  //初期位置(出力軸中心角)への移動
  void Close_port();       //通信ポートを閉じる

  int Readtheta_res(int ID[JOINT_NUM]);
  int Readpresent_velocity(int ID[JOINT_NUM]);
  int Readpresent_torque(int ID[JOINT_NUM]);
  // int getCranex7JointState();

  int setCranex7Torque(double *torque_array);  //,FILE
  bool Setoperation(int Operationmode, int ID[JOINT_NUM]);
  // int positioncontrol(FILE *fp);
  // void *bilateralcontrol(void *);
  void write_csv(double time, long sleep_time, double control_time);
  void position_control(double theta_ref[JOINT_NUM]);
  void torque_control(double theta_ref[JOINT_NUM], double omega_ref[JOINT_NUM],
                      double tau_ref[JOINT_NUM]);
  void controller();
};

#endif
