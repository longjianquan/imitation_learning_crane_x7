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
