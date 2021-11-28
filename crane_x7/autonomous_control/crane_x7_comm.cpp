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
//
#include "crane_x7_comm.h"

#include "dynamixel_sdk.h"

double deg2value(double deg) {
  return (deg + 180) * 4096 / 360;
}  // degをvalueに変換(サーボ出力軸の基準を0)(value)
double value2deg(double value) {
  return value * 360 / 4096 - 180;
}  // valueをdegに変換(サーボ出力軸の基準を0)(deg)
//
/**
 * @fn static double rad2dxlvalue(double)
 * @brief Angle unit conversion function from rad to dynamixel value
 * @param[in] rad :angle[rad/s]
 * @return value :angle[dynamixel value]
 */
double rad2dxlvalue(double rad) {
  double value = rad / (DXL_VALUE_TO_RADIAN);
  return value;
}

/**
 * @fn static double dxlvalue2rad(double)
 * @brief Angle unit conversion function from dynamixel value to rad
 * @param[in] value :angle[dynamixel value]
 * @return rad :angle[rad/s]
 */
double dxlvalue2rad(double value) {
  double rad = value * (DXL_VALUE_TO_RADIAN);
  return rad;
}

/**
 * @fn static double angularvel2dxlvalue(double)
 * @brief Anglular velocity unit conversion function from rad/s to dynamixel
 * value
 * @param[in] angular_velocity :anglular velocity[rad/s]
 * @return value :anglular velocity[dynamixel value]
 */
double angularvel2dxlvalue(double angular_velocity) {
  double value = angular_velocity / (DXL_VALUE_TO_ANGULARVEL);
  return value;
}

/**
 * @fn static double dxlvalue2angularvel(double)
 * @brief Anglular velocity unit conversion function from dynamixel value to
 * rad/s
 * @param[in] value :anglular velocity[dynamixel value]
 * @return angular_velocity :anglular velocity[rad/s]
 */
double dxlvalue2angularvel(double value) {
  double angular_velocity = value * (DXL_VALUE_TO_ANGULARVEL);
  return angular_velocity;
}

/**
 * @fn static double current2dxlvalue(double)
 * @brief Current unit conversion function from A to dynamixel value
 * @param[in] current :current[A]
 * @return value :current[dynamixel value]
 */
double current2dxlvalue(double current) {
  double value = current / (DXL_VALUE_TO_CURRENT);
  return value;
}

/**
 * @fn static double dxlvalue2current(double)
 * @brief Current unit conversion function from dynamixel value to A
 * @param[in] value :current[dynamixel value]
 * @return current :current[A]
 */
double dxlvalue2current(double value) {
  double current = value * (DXL_VALUE_TO_CURRENT);
  return current;
}

/**
 * @fn static double current2torqueXM430W350(double)
 * @brief Conversion function from current[A] to torque[Nm] for XM430W350
 * @param[in] current :current[A]
 * @return torque :torque[Nm]
 */
double current2torqueXM430W350(double current) {
  double torque = current * (CURRENT_TO_TORQUE_XM430W350);
  return torque;
}

/**
 * @fn static double current2torqueXM540W270(double)
 * @brief Conversion function from current[A] to torque[Nm] for XM540W270
 * @param[in] current :current[A]
 * @return torque :torque[Nm]
 */
double current2torqueXM540W270(double current) {
  double torque = current * (CURRENT_TO_TORQUE_XM540W270);
  return torque;
}

/**
 * @fn static double torque2currentXM430W350(double)
 * @brief Conversion function from torque[Nm] to current[A] for XM430W350
 * @param[in] torque :torque[Nm]
 * @return current :current[A]
 */
double torque2currentXM430W350(double torque) {
  double current = torque / (CURRENT_TO_TORQUE_XM430W350);
  return current;
}

/**
 * @fn static double torque2currentXM540W270(double)
 * @brief Conversion function from torque[Nm] to current[A] for XM540W270
 * @param[in] torque :torque[Nm]
 * @return current :current[A]
 */
double torque2currentXM540W270(double torque) {
  double current = torque / (CURRENT_TO_TORQUE_XM540W270);
  return current;
}

/**
 * @fn		int getch()
 * @brief	キーボード入力用の関数(Dynamixel SDK sample)
 * @return	getchar() キー入力
 */
int getch() {
  struct termios oldt, newt;
  int ch;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return ch;
}
