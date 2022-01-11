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

#include <string>

#include "dynamixel_sdk.h"

#ifndef JOINT_NUM
#define JOINT_NUM (8)
#endif

using namespace std;

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

  //コンストラクタ
  CR7(const char *devicename, double initial_pose[JOINT_NUM],
      int masterorslave);

  //デストラクタ
  ~CR7();

  int ms;

  //各種エラー出力用変数
  int dxl_comm_result;       // Communication result
  bool dxl_addparam_result;  // addParam result
  bool dxl_getdata_result;   // GetParam result
  uint8_t dxl_error;         // Dynamixel error

  uint8_t param_theta_ref[4];  //通信パケット用に変換したtheta_refの変数
  uint8_t param_goal_current[4];  //通信パケット用に変換したtheta_refの変数
  uint8_t param_value[4];  //通信パケット用に変換したvalueの変数

  int32_t dxl_theta_res;            //サーボの現在位置取得用の変数
  int16_t dxl_present_velocity;     //サーボの現在位置取得用の変数
  int16_t dxl_present_torque;       //サーボの現在位置取得用の変数
  int16_t goal_current[JOINT_NUM];  // = {0,0,0,0,0,0,0,0};

  // response
  double theta_res[JOINT_NUM] = {0};
  double present_velocity[JOINT_NUM] = {0};
  double present_torque[JOINT_NUM] = {0};
  double omega_res[JOINT_NUM] = {0};
  double tau_res[JOINT_NUM] = {0};

  // refarence
  double theta_ref[JOINT_NUM] = {0};
  double omega_ref[JOINT_NUM] = {0};
  double tau_ref[JOINT_NUM] = {0};

  double goal_torque[JOINT_NUM] = {0};

  double d_theta_temp[JOINT_NUM] = {0};

  // DOB
  double dob0[JOINT_NUM] = {0};
  double dob1[JOINT_NUM] = {0};
  double dob2[JOINT_NUM] = {0};
  double tau_dis[JOINT_NUM] = {0};

  FILE *ffp;
  std::string filename;
  int datareadflag;

  bool Open_port();                                 // 通信ポートを開く
  bool Set_port_baudrate();                         // 通信レートの設定
  void Enable_Dynamixel_Torque(int ID[JOINT_NUM]);  // 全サーボのトルクをON
  void Disable_Dynamixel_Torque(int ID[JOINT_NUM]);  // 全サーボのトルクをOFF

  //設定されているgoal positionに移動
  void Move_Theta_Ref(double *goal_pose, int ID[JOINT_NUM],
                      double JOINT_MIN[JOINT_NUM], double JOINT_MAX[JOINT_NUM]);

  //初期位置(出力軸中心角)への移動
  void Move_Offset_Position(int ID[JOINT_NUM]);
  void Close_port();  //通信ポートを閉じる

  int Readpresent_position(int ID[JOINT_NUM]);
  int Readpresent_velocity(int ID[JOINT_NUM]);
  int Readpresent_torque(int ID[JOINT_NUM]);

  int setCranex7Torque(double *torque_array);
  bool Setoperation(int Operationmode, int ID[JOINT_NUM]);

  void write_csv(double time, long sleep_time, double control_time);
  void position_control(double theta_ref[JOINT_NUM]);
  void force_control(double theta_ref[JOINT_NUM], double omega_ref[JOINT_NUM],
                     double tau_ref[JOINT_NUM]);
  void controller();
};

#endif
