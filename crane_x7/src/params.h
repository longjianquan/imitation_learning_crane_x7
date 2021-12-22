#include "crane.h"

static int ID[JOINT_NUM] = {2, 3, 4, 5, 6, 7, 8, 9};  // サーボモータのID

static double JOINT_MIN[JOINT_NUM] = {
    262, 1024, 262, 228, 262, 1024, 148, 1991,
};
static double JOINT_MAX[JOINT_NUM] = {
    3834, 3072, 3834, 2048, 3834, 3072, 3948, 3072,
};

// サーボの位置制御モードでの動作位置の設定
static double save_pose[JOINT_NUM] = {
    1.68, 3.14, 3.88, 1.71, 3.14, 3.14, 3.14, 3.49,
};  // 位置制御モードで一旦行く位置(rad)
static double goal_pose[JOINT_NUM] = {
    3.14, 3.14, 3.14, 1.38, 3.14, 3.14, 3.14, 4.0,
};  // 位置制御モードからトルク制御モードに切り替わる時の位置(rad)
static double finish_pose[JOINT_NUM] = {
    1.68, 2.81, 3.14, 0.81, 3.16, 3.14, 3.14, 3.49,
};  // 動作終了時の位置(rad)

// サーボのトルク制御モードでの速度リミットの設定
static double LIMIT_SPEED[JOINT_NUM] = {3.0, 2.0, 2.0, 2.5, 4.5, 4.5, 8.0, 8.0};

// 物理パラメータ
static double J[JOINT_NUM] = {
    0.012258, 0.11299, 0.012028, 0.04, 0.005676, 0.0066, 0.006281, 0.006891,
};  //慣性

static double D[JOINT_NUM] = {
    0.0501, 0.0, 0.242, 0.0, 0.040, 0.0391, 0.05, 0.021,
};  //摩擦補償係数

static double M[3] = {2.094457, 1.1505, 1.18337};  //重力補償係数

// pid gain
static double Kp[JOINT_NUM] = {256, 196, 961, 144, 289, 324, 144, 324};
static double Kd[JOINT_NUM] = {40, 28, 66, 24, 34, 36, 24, 36};
static double Kf[JOINT_NUM] = {0.70, 0.70, 1.0, 1.0, 0.80, 1.0, 0.80, 1.0};

// cat off frequency
static double g[JOINT_NUM] = {15, 15, 20, 20, 20, 20, 20, 20};
