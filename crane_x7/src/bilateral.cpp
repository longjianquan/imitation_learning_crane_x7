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

// image
#include <opencv2/opencv.hpp>
// #include <librealsense2/rs.hpp>

#include "crane.h"
#include "crane_x7_comm.h"
#include "params.h"

using namespace std;

static char ch = 'p';

const char *devicename1 = "/dev/ttyUSB0";  // slave
const char *devicename2 = "/dev/ttyUSB1";  // master
CR7 crane_s(devicename1, goal_pose, SLAVE);
CR7 crane_m(devicename2, goal_pose, MASTER);

// サーボの位置制御モードでの動作位置の設定
static double save_pose[JOINT_NUM] = {
    // 1.68, 3.14, 3.88, 1.71, 3.14, 3.14, 3.14, 3.49,
    1.57, 3.14, 3.14, 1.71, 3.14, 3.14, 3.14, 3.14,
};  // 位置制御モードで一旦行く位置(rad)
static double goal_pose[JOINT_NUM] = {
    3.14, 3.14, 3.14, 1.38, 3.14, 3.14, 3.14, 4.0,
};  // 位置制御モードからトルク制御モードに切り替わる時の位置(rad)
static double finish_pose[JOINT_NUM] = {
    // 1.68, 2.81, 3.14, 0.81, 3.16, 3.14, 3.14, 3.49,
    1.57, 2.81, 3.14, 0.4, 3.16, 3.14, 3.14, 3.14,
};  // 動作終了時の位置(rad)


void *bilateral_control(CR7 *crane_s, CR7 *crane_m, bool slave) {
  double control_time_s;
  long sleep_time_s;
  struct timeval start_time_s;
  struct timeval end_time_s;
  int count = 0;
  static double ts = 0.002;
  static double time = 0.0;

  if (!crane_s->Set_port_baudrate()) {  // 通信レートの設定
    crane_s->Close_port();              // 通信ポートを閉じる
    return NULL;
  }

  // position control mode
  crane_s->Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s->Enable_Dynamixel_Torque(ID);

  // move to initial pose
  crane_s->Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s->Move_Theta_Ref(goal_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // current control mode
  crane_s->Disable_Dynamixel_Torque(ID);
  crane_s->Setoperation(CURRENT_CONTROL_MODE, ID);
  crane_s->Enable_Dynamixel_Torque(ID);

  // initialize
  crane_s->Readpresent_position(ID);
  for (int j = 0; j < JOINT_NUM2; j++)
    crane_s->d_theta_temp[j] = crane_s->theta_res[j];

  /***************************************************************************
   *                              main loop
   ***************************************************************************/
  while (true) {
    // start time
    gettimeofday(&start_time_s, NULL);

    // position observation
    crane_s->Readpresent_position(ID);

    // calculate velocity
    for (int i = 0; i < JOINT_NUM2; i++) {
      crane_s->omega_res[i] =
          (crane_s->theta_res[i] - crane_s->d_theta_temp[i]) * g[i];
      crane_s->d_theta_temp[i] += crane_s->omega_res[i] * ts;
    }

    // forced termination
    bool finishFlag = false;
    for (int i = 0; i < JOINT_NUM2; i++) {
      if (crane_s->theta_res[i] == 0.0) {
        cout << (slave ? "slave" : "master");
        cout << "読み込み怪しいので終了" << endl;
        finishFlag = true;
      }

      if (fabs(crane_s->omega_res[i]) >= LIMIT_SPEED[i]) {
        cout << (slave ? "slave" : "master");
        printf("の軸%dが速いので終了: %lf [rad/s]\n", i, crane_s->omega_res[i]);
        finishFlag = true;
      }
    }
    if (finishFlag) break;

    // action
    if (ch == 'p') {  // move to initial pose
      crane_s->position_control(goal_pose);

    } else if (ch == 'b') {  // bilateral control
      crane_s->force_control(crane_m->theta_res, crane_m->omega_res,
                             crane_m->tau_res);

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
    crane_s->write_csv(time, sleep_time_s, control_time_s);

    // print
    if (slave && (count % 10) == 0) printf("mode: %c  time: %lf\n", ch, time);

    // sleep
    usleep(sleep_time_s);

    // time count
    if (ch == 'b') time += ts;
    count++;
  }

  // position control mode
  crane_s->Disable_Dynamixel_Torque(ID);
  crane_s->Setoperation(POSITION_CONTROL_MODE, ID);
  crane_s->Enable_Dynamixel_Torque(ID);

  // move to finish pose
  crane_s->Move_Theta_Ref(save_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);
  crane_s->Move_Theta_Ref(finish_pose, ID, JOINT_MIN, JOINT_MAX);
  sleep(5);

  // Torque off
  crane_s->Disable_Dynamixel_Torque(ID);

  return NULL;
}

void *slave_control(void *) {
  return bilateral_control(&crane_s, &crane_m, true);
}

void *master_control(void *) {
  return bilateral_control(&crane_m, &crane_s, false);
}

void save_image(string fname_color, string fname_depth, int width, int height) {
  rs2::frameset frames = pipe.wait_for_frames();

  rs2::frame color_frame = frames.get_color_frame();
  rs2::frame depth_frame = frames.get_depth_frame();

  rs2::frame depth_frame_color = depth_frame.apply_filter(color_map);

  cv::Mat color(cv::Size(width, height), CV_8UC3,
                (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
  cv::Mat depth(cv::Size(width, height), CV_8UC3,
                (void *)depth_frame_color.get_data(), cv::Mat::AUTO_STEP);

  cv::imwrite(fname_color, color);
  cv::imwrite(fname_depth, depth);
}

void *keyboard_check(void *) {
  char key;

  rs2::config cfg;
  int width = 640;
  int height = 360;
  cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, 30);
  cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, 30);
  rs2::pipeline pipe;
  pipe.start(cfg);
  rs2::colorizer color_map;

  while (ch != 'q') {
    key = getch();

    // P MODE
    if (key == 'p') {
      ch = 'p';
      // printf("MODE P ACTIVE\n");
      cout << "MODE P ACTIVE\n" << endl;
    }

    // B MODE
    if (key == 'b') {
      ch = 'b';
      // printf("MODE B ACTIVE\n");
      cout << "MODE B ACTIVE\n" << endl;
      save_image("./image/color/color_start.png",
                 "./image/depth/depth_start.png", width, height);
    }

    // Q MODE
    else if (key == 'q') {
      ch = 'q';
      // printf("MODE Q ACTIVE\n");
      cout << "MODE Q ACTIVE\n" << endl;
      save_image("./image/color/color_end.png", "./image/depth/depth_end.png",
                 width, height);
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
  pthread_t master_thread, slave_thread, getch_thread;

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

  // スレッド開始
  pthread_join(master_thread, NULL);
  pthread_join(slave_thread, NULL);
  pthread_join(getch_thread, NULL);

  return 0;
}
