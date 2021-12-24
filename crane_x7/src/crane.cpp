/**
 * @file crane_x7_comm.c
 * @brief Wrapper functions of DynamixelSDK (for communicating with the
 * CRANE-X7)
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

//// Header files //////
#include "crane.h"

#include "control_params.h"
#include "crane_x7_comm.h"
#include "dynamixel_sdk.h"

/**
 * @brief コンストラクタ
 * @param devicename デバイス名
 * @param initial_pose 初期姿勢
 * @param masterorslave マスターなら１　スレーブなら０
 */
CR7::CR7(const char *devicename, double initial_pose[JOINT_NUM],
         int masterorslave) {
  ms = masterorslave;
  portHandler = dynamixel::PortHandler::getPortHandler(devicename);
  packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  groupBulkWrite = new dynamixel::GroupBulkWrite(portHandler, packetHandler);
  groupBulkRead = new dynamixel::GroupBulkRead(portHandler, packetHandler);

  dxl_comm_result = COMM_TX_FAIL;  // Communication result
  dxl_addparam_result = false;     // addParam result
  dxl_getdata_result = false;      // GetParam result
  dxl_error = 0;                   // Dynamixel error
  datareadflag = 0;

  dxl_theta_res = {0};
  dxl_present_velocity = {0};
  dxl_present_torque = {0};

  for (int i = 0; i < JOINT_NUM; i++) theta_ref[i] = initial_pose[i];

  /////////////////// csv //////////////////////
  if (ms == 0) {
    filename = "master.csv";
  } else {
    filename = "slave.csv";
  }

  ffp = fopen(filename.c_str(), "w");
  fprintf(ffp, "time,");

  int I = JOINT_NUM;

  for (int i = 0; i < I; i++) fprintf(ffp, "theta_res[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "omega_res[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "tau_res[%d],", i);

  for (int i = 0; i < I; i++) fprintf(ffp, "theta_ref[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "omega_ref[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "tau_ref[%d],", i);

  for (int i = 0; i < I; i++) fprintf(ffp, "goal_torque[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "tau_dis[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "dob0[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "dob1[%d],", i);
  for (int i = 0; i < I; i++) fprintf(ffp, "dob2[%d],", i);

  fprintf(ffp, "sleeptime,controltime\n");
  //////////////////////////////////////////////
}

/**
 * @brief デストラクタ
 */
CR7::~CR7() {
  Close_port();
  fclose(ffp);
  printf("finish CR7\n");
}

int CR7::Readpresent_position(int ID[JOINT_NUM]) {
  for (int i = 0; i < JOINT_NUM2; i++) {  //返信データが利用できるか確認
    //読み込みのデータを設定(現在角度)
    dxl_addparam_result = groupBulkRead->addParam(ID[i], THETA_RES_ADDRESS,
                                                  THETA_RES_DATA_LENGTH);

    // Bulkread present position
    dxl_comm_result = groupBulkRead->txRxPacket();  //返信データの読み込み
    if (dxl_comm_result != COMM_SUCCESS) printf(" disconnect \n");

    // Check if groupbulkread data of Dynamixel is available
    dxl_getdata_result = groupBulkRead->isAvailable(ID[i], THETA_RES_ADDRESS,
                                                    THETA_RES_DATA_LENGTH);
    if (!dxl_getdata_result)
      printf(" ID[%d] : groupBulkRead getdata failed\n", ID[i]);

    //返信データから指定のデータを読む
    dxl_theta_res =
        groupBulkRead->getData(ID[i], THETA_RES_ADDRESS, THETA_RES_DATA_LENGTH);

    theta_res[i] = dxlvalue2rad(dxl_theta_res);
  }
  return 0;
}

int CR7::Readpresent_velocity(int ID[JOINT_NUM]) {
  for (int i = 0; i < JOINT_NUM2; i++) {
    //読み込みのデータを設定(現在角度)
    dxl_addparam_result = groupBulkRead->addParam(
        ID[i], PRESENT_VELOCITY_ADDRESS, PRESENT_VELOCITY_DATA_LENGTH);

    // Bulkread present position
    dxl_comm_result = groupBulkRead->txRxPacket();  //返信データの読み込み
    if (dxl_comm_result != COMM_SUCCESS) printf(" disconnect \n");

    // Check if groupbulkread data of Dynamixel is available
    // //返信データが利用できるか確認
    dxl_getdata_result = groupBulkRead->isAvailable(
        ID[i], PRESENT_VELOCITY_ADDRESS, PRESENT_VELOCITY_DATA_LENGTH);
    if (dxl_getdata_result != true)
      printf(" ID[%d] : groupBulkRead getdata failed\n", ID[i]);

    //返信データから指定のデータを読む
    dxl_present_velocity = groupBulkRead->getData(
        ID[i], PRESENT_VELOCITY_ADDRESS, PRESENT_VELOCITY_DATA_LENGTH);

    present_velocity[i] = dxlvalue2angularvel(dxl_present_velocity);
  }
  return 0;
}

int CR7::Readpresent_torque(int ID[JOINT_NUM]) {
  for (int i = 0; i < JOINT_NUM2; i++) {
    //読み込みのデータを設定(現在角度)
    dxl_addparam_result = groupBulkRead->addParam(
        ID[i], PRESENT_CURRENT_ADDRESS, PRESENT_CURRENT_DATA_LENGTH);

    // Bulkread present position
    dxl_comm_result = groupBulkRead->txRxPacket();  //返信データの読み込み
    if (dxl_comm_result != COMM_SUCCESS) printf(" disconnect \n");

    // Check if groupbulkread data of Dynamixel is available
    // //返信データが利用できるか確認
    dxl_getdata_result = groupBulkRead->isAvailable(
        ID[i], PRESENT_CURRENT_ADDRESS, PRESENT_CURRENT_DATA_LENGTH);
    if (dxl_getdata_result != true)
      printf(" ID[%d] : groupBulkRead getdata failed\n", ID[i]);

    //返信データから指定のデータを読む
    dxl_present_torque = groupBulkRead->getData(ID[i], PRESENT_CURRENT_ADDRESS,
                                                PRESENT_CURRENT_DATA_LENGTH);

    if (i == XM540_W270_JOINT) {
      present_torque[i] =
          torque2currentXM540W270(dxlvalue2current(dxl_present_torque));
    } else {
      present_torque[i] =
          current2torqueXM430W350(dxlvalue2current(dxl_present_torque));
    }
  }
  return 0;
}

/**
 * @fn		bool Open_port()
 * @brief	設定した DEVICENAMEのポートを開く
 * @return	bool 1:成功　0:失敗
 */
bool CR7::Open_port() {
  if (portHandler->openPort()) {
    printf("Succeeded to open the port!\n");
    return 1;
  } else {
    printf("Failed to open the port!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
}

/**
 * @fn		bool Set_port_baudrate()
 * @brief	設定した BAUDRATEで通信の設定をする
 * @return	bool 1:成功 0:失敗
 */
bool CR7::Set_port_baudrate() {
  // Set port baudrate
  if (portHandler->setBaudRate(BAUDRATE)) {
    printf("Succeeded to change the baudrate!\n");
    return 1;
  } else {
    printf("Failed to change the baudrate!\n");
    printf("Press any key to terminate...\n");
    getch();
    return 0;
  }
}

int CR7::setCranex7Torque(double *torque_array) {
  for (int i = 0; i < JOINT_NUM2; i++) {
    if (i == XM540_W270_JOINT) {
      goal_current[i] =
          (int16_t)current2dxlvalue(torque2currentXM540W270(torque_array[i]));
      if ((goal_current[i] > 1006)) {  ////MAX1258.2の80%
        goal_current[i] = 1006;
        printf("joint%d::overcurrent\n", i);
      } else if ((goal_current[i] < -1006)) {
        goal_current[i] = -1006;
        printf("joint%d::overcurrent\n", i);
      }
    } else {
      goal_current[i] =
          (int16_t)current2dxlvalue(torque2currentXM430W350(torque_array[i]));
      if ((goal_current[i] > 526)) {  /////MAX657の80%
        goal_current[i] = 526;
        printf("joint%d::overcurrent\n", i);
      } else if ((goal_current[i] < -526)) {
        goal_current[i] = -526;
        printf("joint%d::overcurrent\n", i);
      }
    }
  }
  for (int j = 0; j < JOINT_NUM2; j++) {
    param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD((goal_current[j])));
    param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD((goal_current[j])));
    param_goal_current[2] = DXL_LOBYTE(DXL_HIWORD((goal_current[j])));
    param_goal_current[3] = DXL_HIBYTE(DXL_HIWORD((goal_current[j])));
    int ID = j + 2;
    dxl_addparam_result = groupBulkWrite->addParam(
        ID, GOAL_CURRENT_ADDRESS, GOAL_CURRENT_DATA_LENGTH, param_goal_current);
    if (dxl_addparam_result != true) printf("goal pose error!\n");
  }
  printf("\n");

  // Bulkwrite goal position
  dxl_comm_result = groupBulkWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear bulkwrite parameter storage
  groupBulkWrite->clearParam();

  return 0;
}

bool CR7::Setoperation(int Operationmode, int ID[JOINT_NUM]) {
  // operationchange
  for (int i = 0; i < JOINT_NUM2; i++)
    //該当IDのサーボのトルク管理のアドレスにOFFを書き込む
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, ID[i], OPERATING_MODE_ADDRESS, Operationmode, &dxl_error);

  // Bulkwrite operationmode
  dxl_comm_result = groupBulkWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear bulkwrite parameter storage
  groupBulkWrite->clearParam();

  return 0;
}

/**
 * @fn		void Enable_Dynamixel_Torque()
 * @brief	全サーボのトルクをONにする
 *			全サーボの回転速度をDXL_PROFILE_VELOCITYに設定
 */
void CR7::Enable_Dynamixel_Torque(int ID[JOINT_NUM]) {
  // Enable Dynamixel Torque
  for (int i = 0; i < JOINT_NUM2; i++) {
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, ID[i], TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE,
        &dxl_error);  //該当IDのサーボのトルク管理のアドレスにONを書き込む

    //設定した回転速度を通信パケット用にデータを分ける
    param_value[0] = DXL_LOBYTE(DXL_LOWORD(DXL_PROFILE_VELOCITY));
    param_value[1] = DXL_HIBYTE(DXL_LOWORD(DXL_PROFILE_VELOCITY));
    param_value[2] = DXL_LOBYTE(DXL_HIWORD(DXL_PROFILE_VELOCITY));
    param_value[3] = DXL_HIBYTE(DXL_HIWORD(DXL_PROFILE_VELOCITY));

    //書き込み用のパケットに作成したデータを追加
    dxl_addparam_result =
        groupBulkWrite->addParam(ID[i], PROFILE_VELOCITY_ADDRESS,
                                 PROFILE_VELOCITY_DATA_LENGTH, param_value);

    //各サーボが送信したパケットどうりに動いているか確認
    printf("[ ID : %d : ", ID[i]);
    if (dxl_comm_result != COMM_SUCCESS)
      //正しいコマンドが送信されているか確認
      printf(" result : %s", packetHandler->getTxRxResult(dxl_comm_result));
    else if (dxl_error != 0)
      //エラーが発生した場合のコメント
      printf(" error : %s", packetHandler->getRxPacketError(dxl_error));
    else
      printf(" successfully connected ");  //正常にサーボがトルクON
    printf(" ]\n");
  }

  // Bulkwrite goal position
  dxl_comm_result = groupBulkWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear bulkwrite parameter storage
  groupBulkWrite->clearParam();
}

/**
 * @fn		void Disable_Dynamixel_Torque()
 * @brief	全サーボのトルクをOFFにする
 */
void CR7::Disable_Dynamixel_Torque(int ID[JOINT_NUM]) {
  // Disable Dynamixel Torque
  for (int i = 0; i < JOINT_NUM2; i++)
    dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler, ID[i], TORQUE_ENABLE_ADDRESS, TORQUE_DISABLE,
        &dxl_error);  //該当IDのサーボのトルク管理のアドレスにOFFを書き込む

  // Bulkwrite goal position
  dxl_comm_result = groupBulkWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear bulkwrite parameter storage
  groupBulkWrite->clearParam();
}

/**
 * @fn		void Move_Theta_Ref()
 * @brief	設定してあるGoal Positionへ移動
 * @param	goal_pose[8](static double goal_pose[8])
 * サーボの個数分のデータ(deg)
 */
void CR7::Move_Theta_Ref(double *goal_pose, int ID[JOINT_NUM],
                         double JOINT_MIN[JOINT_NUM],
                         double JOINT_MAX[JOINT_NUM]) {
  // Move target goal position

  for (int i = 0; i < JOINT_NUM2; i++) {
    //指定したサーボとデータの確認
    printf("[ ID[%d] : %lf ]", ID[i], goal_pose[i]);

    //動作角度外の角度が入力された場合
    if ((JOINT_MIN[i] > rad2dxlvalue(goal_pose[i])) ||
        (JOINT_MAX[i] < rad2dxlvalue(goal_pose[i]))) {
      printf("over range!\n");
      sleep(200);
      printf("プログラムを終了します");
      printf("停止ボタンを押して、CRANEを安全な姿勢にしてCtrl+c !!\n");
      sleep(4);
      printf("あと4秒\n");
      Disable_Dynamixel_Torque(ID);
      Close_port();
      exit(1);
    }

    //通信用にデータを分ける
    param_theta_ref[0] = DXL_LOBYTE(DXL_LOWORD(rad2dxlvalue(goal_pose[i])));
    param_theta_ref[1] = DXL_HIBYTE(DXL_LOWORD(rad2dxlvalue(goal_pose[i])));
    param_theta_ref[2] = DXL_LOBYTE(DXL_HIWORD(rad2dxlvalue(goal_pose[i])));
    param_theta_ref[3] = DXL_HIBYTE(DXL_HIWORD(rad2dxlvalue(goal_pose[i])));

    //書き込み用のパケットに追加
    dxl_addparam_result = groupBulkWrite->addParam(
        ID[i], THETA_REF_ADDRESS, THETA_REF_DATA_LENGTH, param_theta_ref);
    if (dxl_addparam_result != true) printf("goal pose error!\n");
  }
  printf("\n");

  // Bulkwrite goal position
  dxl_comm_result = groupBulkWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear bulkwrite parameter storage
  groupBulkWrite->clearParam();
}

/**
 * @fn		void Move_Offset_Position()
 * @brief	サーボの初期位置(出力軸中心角度)へ移動
 */
void CR7::Move_Offset_Position(int ID[JOINT_NUM]) {
  // Move offset position

  for (int i = 0; i < JOINT_NUM2 - 2; i++) {
    //通信用にデータを分ける
    param_theta_ref[0] = DXL_LOBYTE(DXL_LOWORD(DXL_CENTER_POSITION_VALUE));
    param_theta_ref[1] = DXL_HIBYTE(DXL_LOWORD(DXL_CENTER_POSITION_VALUE));
    param_theta_ref[2] = DXL_LOBYTE(DXL_HIWORD(DXL_CENTER_POSITION_VALUE));
    param_theta_ref[3] = DXL_HIBYTE(DXL_HIWORD(DXL_CENTER_POSITION_VALUE));

    //書き込み用のパケットに追加
    dxl_addparam_result = groupBulkWrite->addParam(
        ID[i], THETA_REF_ADDRESS, THETA_REF_DATA_LENGTH, param_theta_ref);
  }
  if (dxl_addparam_result != true) printf("offset error!\n");

  // Bulkwrite goal position
  dxl_comm_result = groupBulkWrite->txPacket();
  if (dxl_comm_result != COMM_SUCCESS)
    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

  // Clear bulkwrite parameter storage
  groupBulkWrite->clearParam();
}

/**
 * @fn		void Close_port()
 * @brief	通信ポートを閉じる
 */
void CR7::Close_port() {
  portHandler->closePort();
  printf("port close and exit program\n");
}

/**
 * @fn		void write_csv()
 * @brief  write current state to csv file
 */
void CR7::write_csv(double time, long sleep_time, double control_time) {
  fprintf(ffp, "%lf,", time);

  int I = JOINT_NUM;
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", theta_res[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", omega_res[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", tau_res[i]);

  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", theta_ref[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", omega_ref[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", tau_ref[i]);

  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", goal_torque[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", tau_dis[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", dob0[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", dob1[i]);
  for (int i = 0; i < I; i++) fprintf(ffp, "%lf,", dob2[i]);

  fprintf(ffp, "%ld,%lf\n", sleep_time, control_time);
}

/**
 * @fn		void position_control()
 * @brief  position control
 */
void CR7::position_control(double input_theta_ref[JOINT_NUM]) {
  for (int i = 0; i < JOINT_NUM; i++) {
    // set target value
    theta_ref[i] = input_theta_ref[i];
    omega_ref[i] = 0.0;
    tau_ref[i] = 0.0;

    // disable joint 2
    if (i == 2) {
      theta_ref[i] = 3.14;
      omega_ref[i] = 0.0;
      tau_ref[i] = 0.0;
    }
  }

  controller();
}

/**
 * @fn    void torque_control()
 * @brief position and torque control
 */
void CR7::force_control(double input_theta_ref[JOINT_NUM],
                        double input_omega_ref[JOINT_NUM],
                        double input_tau_ref[JOINT_NUM]) {
  for (int i = 0; i < JOINT_NUM; i++) {
    // set target value
    theta_ref[i] = input_theta_ref[i];
    omega_ref[i] = input_omega_ref[i];
    tau_ref[i] = input_tau_ref[i];

    // disable joint 2
    if (i == 2) {
      theta_ref[i] = 3.14;
      omega_ref[i] = 0.0;
      tau_ref[i] = 0.0;
    }
  }

  controller();
}

/**
 * @fn    void controller()
 * @brief position and force controller
 */
void CR7::controller() {
  for (int i = 0; i < JOINT_NUM; i++) {
    // position control
    double tau_p = J[i] / 2.0 * Kp[i] * (theta_ref[i] - theta_res[i]);

    // velocity control
    double tau_v = J[i] / 2.0 * Kd[i] * (omega_ref[i] - omega_res[i]);

    // force control
    double tau_f = Kf[i] / 2.0 * (-tau_ref[i] - tau_res[i]);

    // input torque
    goal_torque[i] = tau_p + tau_v + tau_f + tau_dis[i];

    // DOB
    dob0[i] = goal_torque[i] + g[i] * J[i] * omega_res[i];
    dob1[i] = g[i] * (dob0[i] - dob2[i]);
    double ts = 0.002;
    dob2[i] += dob1[i] * ts;
    tau_dis[i] = dob2[i] - g[i] * J[i] * omega_res[i];

    // friction
    tau_res[i] = tau_dis[i] - D[i] * omega_res[i];
  }

  // gravity
  double theta_3 = theta_res[1] + theta_res[3];
  tau_res[1] = tau_dis[1] - M[0] * sin(theta_res[1]) + M[1] * sin(theta_3);
  tau_res[3] = tau_dis[3] + M[2] * sin(theta_3);

  setCranex7Torque(goal_torque);
}
