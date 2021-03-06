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

#ifndef CRANE_X7_CONTROL_H_
#define CRANE_X7_CONTROL_H_

#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>

//// Definition of dynamixel ////

// Data address of dynamixel x
#define OPERATING_MODE_ADDRESS (11)
#define TORQUE_ENABLE_ADDRESS (64)
#define VELOCITY_I_GAIN_ADDRESS (76)
#define VELOCITY_P_GAIN_ADDRESS (78)
#define POSITION_D_GAIN_ADDRESS (80)
#define POSITION_I_GAIN_ADDRESS (82)
#define POSITION_P_GAIN_ADDRESS (84)
#define BUS_WATCHDOG_ADDRESS (98)
#define GOAL_CURRENT_ADDRESS (102)
#define OMEGA_REF_ADDRESS (104)
#define PROFILE_VELOCITY_ADDRESS (112)
#define THETA_REF_ADDRESS (116)
#define PRESENT_CURRENT_ADDRESS (126)
#define PRESENT_VELOCITY_ADDRESS (128)
#define THETA_RES_ADDRESS (132)
#define PRESENT_VALUE_ADDRESS (126)

// Data length
#define BUS_WATCHDOG_DATA_LENGTH (1)
#define THETA_REF_DATA_LENGTH (4)
#define THETA_RES_DATA_LENGTH (4)
#define OMEGA_REF_DATA_LENGTH (4)
#define PRESENT_VELOCITY_DATA_LENGTH (4)
#define GOAL_CURRENT_DATA_LENGTH (2)
#define PRESENT_CURRENT_DATA_LENGTH (2)
#define PRESENT_VALUE_DATA_LENGTH (10)
#define PROFILE_VELOCITY_DATA_LENGTH (4)
// Protocol version
#define PROTOCOL_VERSION (2.0)

// Control value
#define TORQUE_ENABLE (1)
#define TORQUE_DISABLE (0)
#define CURRENT_CONTROL_MODE (0)
#define VELOCITY_CONTROL_MODE (1)
#define POSITION_CONTROL_MODE (3)
#define DEFAULT_POSITION_P_GAIN (800)
#define DEFAULT_POSITION_I_GAIN (0)
#define DEFAULT_POSITION_D_GAIN (0)
#define DEFAULT_VELOCITY_P_GAIN (100)
#define DEFAULT_VELOCITY_I_GAIN (1920)
#define PROFILE_VELOCITY (60)

// Serial port setting
#define BAUDRATE (4000000)
#define SERIAL_PORT "/dev/ttyUSB0"  // Check the port which crane-x7 is conected
#define DEVICENAME "/dev/ttyUSB0"

//// Definition of crane-x7 ////
#define XM540_W270_JOINT \
  (1)  // only 2nd joint servo motor is XM540_W270 (other XM430_W350)

#ifndef JOINT_NUM
#define JOINT_NUM (8)  ////////////////////////////////////
#endif

#ifndef PI
#define PI (3.14159265)
#endif

#define DXL_MOVING_STATUS_THRESHOLD 20  // Dynamixel moving status threshold
#define DXL_CENTER_POSITION_VALUE 2048  // Offset(????????????????????????)???(value)
#define DXL_PROFILE_VELOCITY 30  // ????????????????????????????????????(rpm)
#define STDIN_FILENO 0

// Unit conversion
#define DXL_VALUE_TO_RADIAN ((2 * PI) / 4096)
#define DXL_VALUE_TO_ANGULARVEL ((0.229 * 2 * PI) / 60)
#define DXL_VALUE_TO_CURRENT (0.00269)
#define TORQUE_CORRECTION_FACTOR (1.3)
#define CURRENT_TO_TORQUE_XM430W350 (1.783 * TORQUE_CORRECTION_FACTOR)
#define CURRENT_TO_TORQUE_XM540W270 (2.409 * TORQUE_CORRECTION_FACTOR)

#define JOINT_NUM2 8  ///////////////////////////
#define LOOPTIME 2000
#define LOOPTIME_CAMERA 1000

#define MASTER 0
#define SLAVE 1

double deg2value(double deg);
double value2deg(double value);

double rad2dxlvalue(double rad);
double dxlvalue2rad(double value);
double angularvel2dxlvalue(double angular_velocity);
double dxlvalue2angularvel(double value);
double current2dxlvalue(double current);
double dxlvalue2current(double value);
double current2torqueXM430W350(double current);
double current2torqueXM540W270(double current);
double torque2currentXM430W350(double torque);
double torque2currentXM540W270(double torque);
int getch();

#endif
