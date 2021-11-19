/**
 * @file crane_x7_comm.c
 * @brief Wrapper functions of DynamixelSDK (for communicating with the CRANE-X7)
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
//
#include "crane_x7_comm.h"
#include "crane.h"
#include "dynamixel_sdk.h"

/**
 * @brief コンストラクタ
 */
CR7::CR7(){

	portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	groupBulkWrite = new dynamixel::GroupBulkWrite( portHandler, packetHandler);
	groupBulkRead = new dynamixel::GroupBulkRead( portHandler, packetHandler);

	dxl_comm_result = COMM_TX_FAIL;             // Communication result
	dxl_addparam_result = false;               // addParam result
	dxl_getdata_result = false;                // GetParam result
	dxl_error = 0;                          // Dynamixel error
	datareadflag = 0;

	dxl_present_position = {0};
	dxl_present_velocity = {0};
	dxl_present_torque = {0};

    for(int j =0; j<JOINT_NUM;j++){

    present_position[j] = 0;
	present_velocity[j] = 0;
	present_torque[j] = 0;
	goal_current[j] = 0;

    goal_position[j] = 0;
	goal_velocity[j] = 0;
	goal_torque[j] = 0;
	target_torque[j] = 0;
    d_theta_res[j] = 0;
	d_theta_temp[j] = 0;
	tau_p[j] = 0;
	tau_f[j] = 0;
	tau_dis[j] = 0;
	tau_res[j] = 0;

	dob0[j] = 0;
	dob1[j] = 0;
	dob2[j] = 0;

	}
}

CR7::CR7(const char* devicename,int masterorslave){

	ms = masterorslave;
	portHandler = dynamixel::PortHandler::getPortHandler(devicename);
	packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
	groupBulkWrite = new dynamixel::GroupBulkWrite( portHandler, packetHandler);
	groupBulkRead = new dynamixel::GroupBulkRead( portHandler, packetHandler);

	dxl_comm_result = COMM_TX_FAIL;             // Communication result
	dxl_addparam_result = false;               // addParam result
	dxl_getdata_result = false;                // GetParam result
	dxl_error = 0;                          // Dynamixel error
	datareadflag = 0;

	dxl_present_position = {0};
	dxl_present_velocity = {0};
	dxl_present_torque = {0};

    for(int j =0; j<JOINT_NUM;j++){

    present_position[j] = 0;
	present_velocity[j] = 0;
	present_torque[j] = 0;
	goal_current[j] = 0;

    goal_position[j] = 0;
	goal_velocity[j] = 0;
	target_torque[j] = 0;
	goal_torque[j] = 0;
    d_theta_res[j] = 0;
	d_theta_temp[j] = 0;
	tau_p[j] = 0;
	tau_f[j] = 0;
	tau_dis[j] = 0;
	tau_res[j] = 0;

	dob0[j] = 0;
	dob1[j] = 0;
	dob2[j] = 0;
	}
	if(ms == 0){
		//sleep(4);
	    //cout << "マスターのファイル名を入力して下さい"<<endl;
	    //cin >> filename2;
	    filename2 = "master.csv";	
	}
	else{
		//sleep(4);
		//cout << "スレーブのファイル名を入力して下さい"<<endl;
	    //cin >> filename2;
	    filename2 = "slave.csv";
	}

}

int CR7::Readpresent_position(int ID[JOINT_NUM]){

			for(int i=0;i<JOINT_NUM2;i++){
			    dxl_addparam_result = groupBulkRead->addParam(ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);					//読み込みのデータを設定(現在角度)
			    //if( dxl_addparam_result != true){
				//    fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed", ID[i]);
			    //} 
			}

			//Bulkread present position 
			dxl_comm_result = groupBulkRead->txRxPacket();																				//返信データの読み込み
			if(dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");

			//Check if groupbulkread data of Dynamixel is available
			for (int i = 0; i < JOINT_NUM2; i++)
            {																		//返信データが利用できるか確認
			    dxl_getdata_result = groupBulkRead->isAvailable(ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
			    if(dxl_getdata_result != true){
					 printf("bbbbbbbbbbb ID[%d] : groupBulkRead getdata failed\n", ID[i]);
			    }
			}
			for (int i = 0; i < JOINT_NUM2; i++)
            {	
 			    dxl_present_position = groupBulkRead->getData(ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);					//返信データから指定のデータを読む
			    present_position[i] = dxlvalue2rad(dxl_present_position);
			}
			return 0;
}

/*
int CR7::getCranex7JointState()
{

  groupBulkRead = groupBulkRead(portHandler, PROTOCOL_VERSION);
  // set bulk read parameter (present positon, present velosity, present current)
  for (int i = 0; i < JOINT_NUM2; i++)
  {
    dxl_addparam_result = groupBulkReadAddParam(groupBulkRead, ID[i], PRESENT_VALUE_ADDRESS, PRESENT_VALUE_DATA_LENGTH);
    if (dxl_addparam_result != True)
    {
      fprintf(stderr, "[ID:%03d] groupBulkRead addparam failed", ID[i]);
      return 1;
    }
  }
  // data request and receive
  groupBulkReadTxRxPacket(groupBulkRead);
  if ((dxl_comm_result = getLastTxRxResult(portHandler, PROTOCOL_VERSION)) != COMM_SUCCESS)
    printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

  // verification of received data
  for (int i = 0; i < JOINT_NUM2; i++)
  {
    dxl_getdata_result = groupBulkReadIsAvailable(groupBulkRead, ID[i], PRESENT_VALUE_ADDRESS, PRESENT_VALUE_DATA_LENGTH);
    if (dxl_getdata_result != True)
    {
      fprintf(stderr, "[ID:%03d] groupBulkRead getdata trq failed", ID[i]);
      return 1;
    }
  }
  // pick up present position data
  for (int i = 0; i < JOINT_NUM2; i++)
  {
    dxl_present_position = groupBulkReadGetData(groupBulkRead, ID[i], PRESENT_POSITION_ADDRESS, PRESENT_POSITION_DATA_LENGTH);
  
    present_position[i] = dxlvalue2rad((double)(dxl_present_position));
   
  }
  return 0;
}
*/

int CR7::Readpresent_velocity(int ID[JOINT_NUM]){

			for(int i=0;i<JOINT_NUM2;i++){
			dxl_addparam_result = groupBulkRead->addParam(ID[i], PRESENT_VELOCITY_ADDRESS,  PRESENT_VELOCITY_DATA_LENGTH);					//読み込みのデータを設定(現在角度)
			//if( dxl_addparam_result != true) printf(" ID[%d] : groupBulkRead addParam failed\n", ID[i]);

			//Bulkread present position 
			dxl_comm_result = groupBulkRead->txRxPacket();																				//返信データの読み込み
			if(dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");

			//Check if groupbulkread data of Dynamixel is available																		//返信データが利用できるか確認
			dxl_getdata_result = groupBulkRead->isAvailable(ID[i], PRESENT_VELOCITY_ADDRESS,  PRESENT_VELOCITY_DATA_LENGTH);
			if(dxl_getdata_result != true) printf(" ID[%d] : groupBulkRead getdata failed\n", ID[i]);

			dxl_present_velocity = groupBulkRead->getData(ID[i], PRESENT_VELOCITY_ADDRESS,  PRESENT_VELOCITY_DATA_LENGTH);					//返信データから指定のデータを読む
			//printf("[ ID[%d] : %lf ]", ID[i], value2deg(dxl_present_velocity));
			present_velocity[i] = dxlvalue2angularvel(dxl_present_velocity);
			}
			return 0;
}

int CR7::Readpresent_torque(int ID[JOINT_NUM]){

			for(int i=0;i<JOINT_NUM2;i++){
			dxl_addparam_result = groupBulkRead->addParam(ID[i], PRESENT_CURRENT_ADDRESS,  PRESENT_CURRENT_DATA_LENGTH);					//読み込みのデータを設定(現在角度)
			//if( dxl_addparam_result != true) printf(" ID[%d] : groupBulkRead addParam failed\n", ID[i]);

			//Bulkread present position 
			dxl_comm_result = groupBulkRead->txRxPacket();																				//返信データの読み込み
			if(dxl_comm_result != COMM_SUCCESS) printf(" discommect \n");

			//Check if groupbulkread data of Dynamixel is available																		//返信データが利用できるか確認
			dxl_getdata_result = groupBulkRead->isAvailable(ID[i], PRESENT_CURRENT_ADDRESS,  PRESENT_CURRENT_DATA_LENGTH);
			if(dxl_getdata_result != true) printf(" ID[%d] : groupBulkRead getdata failed\n", ID[i]);

			dxl_present_torque = groupBulkRead->getData(ID[i], PRESENT_CURRENT_ADDRESS,  PRESENT_CURRENT_DATA_LENGTH);					//返信データから指定のデータを読む
			//printf("[ ID[%d] : %lf ]", ID[i], value2deg(dxl_present_torque));

            if(i ==  XM540_W270_JOINT){
			    present_torque[i] = torque2currentXM540W270(dxlvalue2current(dxl_present_torque));	
			}
            else{
			    present_torque[i] = current2torqueXM430W350(dxlvalue2current(dxl_present_torque));	
			}

			}
			return 0;
}


/**
 * @fn		bool Open_port()
 * @brief	設定した DEVICENAMEのポートを開く
 * @return	bool 1:成功　0:失敗
 */
bool CR7::Open_port(){
	//Open port
	if (portHandler->openPort())
	{
		printf("Succeeded to open the port!\n");
		return 1;
	}
	else
	{
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
bool CR7::Set_port_baudrate(){
	// Set port baudrate
	if (portHandler->setBaudRate(BAUDRATE))
	{
		printf("Succeeded to change the baudrate!\n");
		return 1;
	}
	else
	{
		printf("Failed to change the baudrate!\n");
		printf("Press any key to terminate...\n");
		getch();
		return 0;
	}
}


int CR7::setCranex7Torque(double *torque_array, int ID[JOINT_NUM]){//,FILE *fp){
	//int16_t goal_current[JOINT_NUM] = {0,0,0,0,0,0,0,0};
 for (int i = 0; i < JOINT_NUM2; i++)
  {
    if (i == XM540_W270_JOINT)
    {
      goal_current[i] = (int16_t)current2dxlvalue(torque2currentXM540W270(torque_array[i]));
	  if((goal_current[i] > 1006)){////MAX1258.2の80%
		  goal_current[i] = 1006;
		  printf("joint%d::overcurrent\n",i);
      }
	else if((goal_current[i] < -1006)){
        goal_current[i] = -1006;
		printf("joint%d::overcurrent\n",i);
	  }
	}
    else
    {
      goal_current[i] = (int16_t)current2dxlvalue(torque2currentXM430W350(torque_array[i]));
	  if((goal_current[i]>526)){/////MAX657の80%
		  goal_current[i] = 526;
		  printf("joint%d::overcurrent\n",i);
      }
	  else if((goal_current[i] < -526)){
		  goal_current[i] = -526;
		  printf("joint%d::overcurrent\n",i);
	  }
    }
  }
  //fprintf( ffp, "%d,%d,%d,%d,%d,%d,%d,%d,",goal_current[0],goal_current[1],goal_current[2],goal_current[3],goal_current[4],goal_current[5],goal_current[6],goal_current[7]);
  for(int k=0;k<JOINT_NUM2;k++){

		param_goal_current[0] = DXL_LOBYTE(DXL_LOWORD((goal_current[k])));														//�ʐM�p�Ƀf�[�^�𕪂���
		param_goal_current[1] = DXL_HIBYTE(DXL_LOWORD((goal_current[k])));
		param_goal_current[2] = DXL_LOBYTE(DXL_HIWORD((goal_current[k])));
		param_goal_current[3] = DXL_HIBYTE(DXL_HIWORD((goal_current[k])));

		dxl_addparam_result = groupBulkWrite->addParam(ID[k], GOAL_CURRENT_ADDRESS, GOAL_CURRENT_DATA_LENGTH, param_goal_current);		//�������ݗp�̃p�P�b�g�ɒǉ�
		if(dxl_addparam_result != true) printf("goal pose error!\n");

	}printf("\n");
	
	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();

	return 0;
}



bool CR7::Setoperation(int Operationmode, int ID[JOINT_NUM]){
	//operationchange
	for(int i=0;i<JOINT_NUM2;i++)
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], OPERATING_MODE_ADDRESS, Operationmode, &dxl_error);		//該当IDのサーボのトルク管理のアドレスにOFFを書き込む

	// Bulkwrite operationmode 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();

	return 0;
}
/**
 * @fn		void Enable_Dynamixel_Torque()
 * @brief	全サーボのトルクをONにする
 *			全サーボの回転速度をDXL_PROFILE_VELOCITYに設定
 */
void CR7::Enable_Dynamixel_Torque(int ID[JOINT_NUM]){
	// Enable Dynamixel Torque
	for(int i=0;i<JOINT_NUM2;i++){
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], TORQUE_ENABLE_ADDRESS, TORQUE_ENABLE, &dxl_error);			//該当IDのサーボのトルク管理のアドレスにONを書き込む
		param_value[0] = DXL_LOBYTE(DXL_LOWORD(DXL_PROFILE_VELOCITY));																	//設定した回転速度を通信パケット用にデータを分ける
		param_value[1] = DXL_HIBYTE(DXL_LOWORD(DXL_PROFILE_VELOCITY));
		param_value[2] = DXL_LOBYTE(DXL_HIWORD(DXL_PROFILE_VELOCITY));
		param_value[3] = DXL_HIBYTE(DXL_HIWORD(DXL_PROFILE_VELOCITY));
		dxl_addparam_result = groupBulkWrite->addParam(ID[i], PROFILE_VELOCITY_ADDRESS, PROFILE_VELOCITY_DATA_LENGTH, param_value);		//書き込み用のパケットに作成したデータを追加
		printf("[ ID : %d : ", ID[i]);																									//各サーボが送信したパケットどうりに動いているか確認
		if (dxl_comm_result != COMM_SUCCESS) printf(" result : %s", packetHandler->getTxRxResult(dxl_comm_result));						//正しいコマンドが送信されているか確認
		else if (dxl_error != 0) printf(" error : %s", packetHandler->getRxPacketError(dxl_error));										//エラーが発生した場合のコメント
		else printf(" successfully connected ");																						//正常にサーボがトルクON
		printf(" ]\n");
	}

	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		void Disable_Dynamixel_Torque()
 * @brief	全サーボのトルクをOFFにする
 */
void CR7::Disable_Dynamixel_Torque(int ID[JOINT_NUM]){
	// Disable Dynamixel Torque
	for(int i=0;i<JOINT_NUM2;i++)
		dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, ID[i], TORQUE_ENABLE_ADDRESS, TORQUE_DISABLE, &dxl_error);		//該当IDのサーボのトルク管理のアドレスにOFFを書き込む

	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		void Move_Goal_Position()
 * @brief	設定してあるGoal Positionへ移動
 * @param	goal_pose[8](static double goal_pose[8]) サーボの個数分のデータ(deg)
 */
/*
void CR7::Move_Goal_Position( double *goal_pose, int ID[JOINT_NUM], double JOINT_MIN[JOINT_NUM], double JOINT_MAX[JOINT_NUM]){
	//Move target goal position

	for(int i=0;i<JOINT_NUM2;i++){
		printf("[ ID[%d] : %lf ]", ID[i], goal_pose[i]);																				//指定したサーボとデータの確認
		if((JOINT_MIN[i] > deg2value(goal_pose[i])) || (JOINT_MAX[i] < deg2value(goal_pose[i]))){											//動作角度外の角度が入力された場合
		    printf("over range!\n");
			for(int j=0;j<JOINT_NUM2;j++){
				if(j == 1){
				    goal_pose[j] = -10.0;
				}
				else if(j == 3){
					goal_pose[j] = -158.0;
				}
				else{
				    goal_pose[j] = 0.0;
				}
			
	    	    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(deg2value(goal_pose[j])));														//通信用にデータを分ける
	    	    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(deg2value(goal_pose[j])));
	    	    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(deg2value(goal_pose[j])));
	    	    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(deg2value(goal_pose[j])));
        
	    	    dxl_addparam_result = groupBulkWrite->addParam(ID[j], GOAL_POSITION_ADDRESS, GOAL_POSITION_DATA_LENGTH, param_goal_position);		//書き込み用のパケットに追加
	    	    if(dxl_addparam_result != true) printf("goal pose error!\n");
    
	        }printf("\n");
	        
	        // Bulkwrite goal position 
	        dxl_comm_result = groupBulkWrite->txPacket();
	        if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        
	        // Clear bulkwrite parameter storage
	        groupBulkWrite->clearParam();
			sleep(4);
			Disable_Dynamixel_Torque(ID);
			Close_port();
			exit(1);
		}
	

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(deg2value(goal_pose[i])));														//通信用にデータを分ける
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(deg2value(goal_pose[i])));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(deg2value(goal_pose[i])));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(deg2value(goal_pose[i])));

		dxl_addparam_result = groupBulkWrite->addParam(ID[i], GOAL_POSITION_ADDRESS, GOAL_POSITION_DATA_LENGTH, param_goal_position);		//書き込み用のパケットに追加
		if(dxl_addparam_result != true) printf("goal pose error!\n");

	}printf("\n");
	
	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}
*/

/**
 * @fn		void Move_Goal_Position()
 * @brief	設定してあるGoal Positionへ移動
 * @param	goal_pose[8](static double goal_pose[8]) サーボの個数分のデータ(deg)
 */
void CR7::Move_Goal_Position( double *goal_pose, int ID[JOINT_NUM], double JOINT_MIN[JOINT_NUM], double JOINT_MAX[JOINT_NUM]){
	//Move target goal position

	for(int i=0;i<JOINT_NUM2;i++){
		printf("[ ID[%d] : %lf ]", ID[i], goal_pose[i]);																				//指定したサーボとデータの確認
		if((JOINT_MIN[i] > rad2dxlvalue(goal_pose[i])) || (JOINT_MAX[i] < rad2dxlvalue(goal_pose[i]))){											//動作角度外の角度が入力された場合
		    printf("over range!\n");
			sleep(200);
			printf("リーチングで変な値が入ったので200秒停止してプログラムを終了します。\n");
			printf("停止ボタンを押して、CRANEを安全な姿勢にしてCtrl+c !!\n");
			/*
			for(int j=0;i<JOINT_NUM2;j++){
				if(j == 1){
				    goal_pose[j] = 2.97;
				}
				else if(j == 3){
					goal_pose[j] = 0.38;
				}
				else{
				    goal_pose[j] = 0.0;
				}
			
	    	    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(rad2dxlvalue(goal_pose[i])));														//通信用にデータを分ける
	    	    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(rad2dxlvalue(goal_pose[i])));
	    	    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(rad2dxlvalue(goal_pose[i])));
	    	    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(rad2dxlvalue(goal_pose[i])));
        
	    	    dxl_addparam_result = groupBulkWrite->addParam(ID[i], GOAL_POSITION_ADDRESS, GOAL_POSITION_DATA_LENGTH, param_goal_position);		//書き込み用のパケットに追加
	    	    if(dxl_addparam_result != true) printf("goal pose error!\n");
    
	        }printf("\n");
	        
	        // Bulkwrite goal position 
	        dxl_comm_result = groupBulkWrite->txPacket();
	        if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        
	        // Clear bulkwrite parameter storage
	        groupBulkWrite->clearParam();
			*/

			sleep(4);
			printf("あと4秒\n");
			Disable_Dynamixel_Torque(ID);
			Close_port();
			exit(1);
		}
	

		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(rad2dxlvalue(goal_pose[i])));														//通信用にデータを分ける
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(rad2dxlvalue(goal_pose[i])));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(rad2dxlvalue(goal_pose[i])));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(rad2dxlvalue(goal_pose[i])));

		dxl_addparam_result = groupBulkWrite->addParam(ID[i], GOAL_POSITION_ADDRESS, GOAL_POSITION_DATA_LENGTH, param_goal_position);		//書き込み用のパケットに追加
		if(dxl_addparam_result != true) printf("goal pose error!\n");

	}printf("\n");
	
	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}


/**
 * @fn		void Move_Offset_Position()
 * @brief	サーボの初期位置(出力軸中心角度)へ移動
 */
void CR7::Move_Offset_Position(int ID[JOINT_NUM]){
	// Move offset position

	for(int i=0;i<JOINT_NUM2-2;i++){ 
		param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(DXL_CENTER_POSITION_VALUE));														//通信用にデータを分ける
		param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(DXL_CENTER_POSITION_VALUE));
		param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(DXL_CENTER_POSITION_VALUE));
		param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(DXL_CENTER_POSITION_VALUE));

		dxl_addparam_result = groupBulkWrite->addParam(ID[i], GOAL_POSITION_ADDRESS, GOAL_POSITION_DATA_LENGTH, param_goal_position);		//書き込み用のパケットに追加
	}
	if(dxl_addparam_result != true) printf("offset error!\n");

	// Bulkwrite goal position 
	dxl_comm_result = groupBulkWrite->txPacket();
	if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));

	// Clear bulkwrite parameter storage
	groupBulkWrite->clearParam();
}

/**
 * @fn		void Close_port()
 * @brief	通信ポートを閉じる
 */
void CR7::Close_port(){
	// Close port
	portHandler->closePort();
	printf("port close and exit program\n");
}