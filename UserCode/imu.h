#ifndef IMU_H
#define IMU_H

#include "bmi088.h"

class IMU {
 public:
  void acc_calculate();
  void gyro_calculate();
  void sensorDataTransform();
  void update();
  void setInstallationMatrix(const float R[3][3]);
  void initFromGravity();

  struct {
    float gyro[3];
    float accel[3];
  } raw_data_{};

  float gyro_sensor_[3]{};
  float accel_sensor_[3]{};
  float gyro_bias_[3]{};
  float accel_bias_[3]{};
  float accel_sen_ = 1.0f;
  float accel_filter_k_ = 0.9f;
  bool gyro_bias_ready_ = false;
  bool attitude_ready_ = false;
  float roll_deg_ = 0.0f;
  float pitch_deg_ = 0.0f;
  float yaw_deg_ = 0.0f;
  float q_[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  float mahony_kp_ = 2.0f;
  float mahony_ki_ = 0.0f;
  float mahony_bias_[3]{};

  uint8_t raw_range = 0;
  uint8_t rx_acc_data[6]{};
  uint8_t raw_gyro_range = 0;
  uint8_t rx_gyro_data[6]{};

  // 换安装方法的时候我要改这个矩阵：第 i 行是机体第 i 轴。按照飞书里的，默认 X、Z 取反，Y 不变。
  float R_imu_[3][3] = {{-1.0f, 0.0f, 0.0f},
                        {0.0f, 1.0f, 0.0f},
                        {0.0f, 0.0f, -1.0f}};

 private:
  static constexpr uint16_t kGyroBiasSamples = 1000;
  static constexpr float kGravity = 9.8f;
  static constexpr float kGravityTolerance = 2.0f;
  static constexpr float kDt = 0.001f;

  void mahonyUpdate();
  void quaternionToEuler();
  float gyro_bias_sum_[3]{};
  uint16_t gyro_bias_count_ = 0;
  bool accel_filter_init_ = false;
};

extern IMU board_imu;

#endif
