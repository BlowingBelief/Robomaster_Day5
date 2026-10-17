#include "imu.h"

#include <cmath>

IMU board_imu;

void IMU::acc_calculate() {
  // 1. 设置/读取 acc 0x41 寄存器中的量程 range 参数，并换算为量程系数
  bmi088_accel_read_reg(0x41, &raw_range, 1);
  // code here
  float acc_range_g = 6.0f;
  switch (raw_range & 0x03) {
  case 0x00: acc_range_g = 3.0f; break;
  case 0x01: acc_range_g = 6.0f; break;
  case 0x02: acc_range_g = 12.0f; break;
  case 0x03: acc_range_g = 24.0f; break;
  default: break;
  }

  // 2. 读取 acc 0x12 寄存器中的 6 字节 acc 数据
  bmi088_accel_read_reg(0x12, rx_acc_data, 6);

  // 3. 用量程系数将原始数据转换为常用单位，写入 raw_data_.accel，单位 m/s^2
  // code here
  for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((rx_acc_data[2 * i + 1] << 8) | rx_acc_data[2 * i]);
    raw_data_.accel[i] = (float)raw / 32768.0f * acc_range_g * 9.8f;
  }
}

void IMU::gyro_calculate() {
  // 1. 设置/读取 gyro 0x0F 寄存器中的量程 range 参数，并换算为量程系数
  // 2. 读取 gyro 0x02 寄存器中的 6 字节 gyro 数据
  // 3. 用量程系数将原始数据转换为常用单位，写入 raw_data_.gyro，单位 rad/s

  bmi088_gyro_read_reg(0x0F, &raw_gyro_range, 1);
  float gyro_lsb = 16.384f;
  switch (raw_gyro_range & 0x07) {
  case 0x00: gyro_lsb = 16.384f; break;
  case 0x01: gyro_lsb = 32.768f; break;
  case 0x02: gyro_lsb = 65.536f; break;
  case 0x03: gyro_lsb = 131.072f; break;
  case 0x04: gyro_lsb = 262.144f; break;
  default: break;
  }
  bmi088_gyro_read_reg(0x02, rx_gyro_data, 6);
  const float deg2rad = 3.1415926f / 180.0f;
  for (uint8_t i = 0; i < 3; i++) {
    int16_t raw = (int16_t)((rx_gyro_data[2 * i + 1] << 8) | rx_gyro_data[2 * i]);
    raw_data_.gyro[i] = ((float)raw / gyro_lsb) * deg2rad;
  }
}

void IMU::setInstallationMatrix(const float R[3][3]) {
  for (uint8_t row = 0; row < 3; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      R_imu_[row][col] = R[row][col];
    }
  }
  for (uint8_t i = 0; i < 3; i++) {
    gyro_bias_[i] = 0.0f;
    gyro_bias_sum_[i] = 0.0f;
    mahony_bias_[i] = 0.0f;
  }
  gyro_bias_count_ = 0;
  gyro_bias_ready_ = false;
  accel_filter_init_ = false;
  attitude_ready_ = false;
  roll_deg_ = 0.0f;
  pitch_deg_ = 0.0f;
  yaw_deg_ = 0.0f;
  q_[0] = 1.0f;
  q_[1] = 0.0f;
  q_[2] = 0.0f;
  q_[3] = 0.0f;
}

void IMU::initFromGravity() {
  if (attitude_ready_ || !gyro_bias_ready_) {
    return;
  }

  const float ax = accel_sensor_[0];
  const float ay = accel_sensor_[1];
  const float az = accel_sensor_[2];
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (fabsf(norm - kGravity) > kGravityTolerance) {
    return;
  }

  const float roll = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
  const float cr = cosf(roll * 0.5f);
  const float sr = sinf(roll * 0.5f);
  const float cp = cosf(pitch * 0.5f);
  const float sp = sinf(pitch * 0.5f);

  q_[0] = cp * cr;
  q_[1] = cp * sr;
  q_[2] = sp * cr;
  q_[3] = -sp * sr;
  quaternionToEuler();
  attitude_ready_ = true;
}

void IMU::quaternionToEuler() {
  const float qw = q_[0];
  const float qx = q_[1];
  const float qy = q_[2];
  const float qz = q_[3];
  const float up_x = 2.0f * (qx * qz - qw * qy);
  const float up_y = 2.0f * (qy * qz + qw * qx);
  const float up_z = 1.0f - 2.0f * (qx * qx + qy * qy);
  const float roll = atan2f(up_y, up_z);
  const float pitch = atan2f(-up_x, sqrtf(up_y * up_y + up_z * up_z));
  const float yaw = atan2f(2.0f * (qw * qz + qx * qy),
                           qw * qw + qx * qx - qy * qy - qz * qz);
  const float rad2deg = 180.0f / 3.1415926f;
  roll_deg_ = roll * rad2deg;
  pitch_deg_ = pitch * rad2deg;
  yaw_deg_ = yaw * rad2deg;
}

void IMU::mahonyUpdate() {
  float omega_x = gyro_sensor_[0] - mahony_bias_[0];
  float omega_y = gyro_sensor_[1] - mahony_bias_[1];
  float omega_z = gyro_sensor_[2] - mahony_bias_[2];

  const float ax = accel_sensor_[0];
  const float ay = accel_sensor_[1];
  const float az = accel_sensor_[2];
  const float norm = sqrtf(ax * ax + ay * ay + az * az);
  if (norm > 1.0f && fabsf(norm - kGravity) <= kGravityTolerance) {
    const float um_x = ax / norm;
    const float um_y = ay / norm;
    const float um_z = az / norm;
    const float qw = q_[0];
    const float qx = q_[1];
    const float qy = q_[2];
    const float qz = q_[3];
    const float up_x = 2.0f * (qx * qz - qw * qy);
    const float up_y = 2.0f * (qy * qz + qw * qx);
    const float up_z = 1.0f - 2.0f * (qx * qx + qy * qy);
    const float e_x = um_y * up_z - um_z * up_y;
    const float e_y = um_z * up_x - um_x * up_z;
    const float e_z = um_x * up_y - um_y * up_x;

    omega_x += mahony_kp_ * e_x;
    omega_y += mahony_kp_ * e_y;
    omega_z += mahony_kp_ * e_z;
    mahony_bias_[0] -= mahony_ki_ * e_x * kDt;
    mahony_bias_[1] -= mahony_ki_ * e_y * kDt;
    mahony_bias_[2] -= mahony_ki_ * e_z * kDt;
  }

  const float qw = q_[0];
  const float qx = q_[1];
  const float qy = q_[2];
  const float qz = q_[3];
  q_[0] += 0.5f * (-qx * omega_x - qy * omega_y - qz * omega_z) * kDt;
  q_[1] += 0.5f * (qw * omega_x + qy * omega_z - qz * omega_y) * kDt;
  q_[2] += 0.5f * (qw * omega_y - qx * omega_z + qz * omega_x) * kDt;
  q_[3] += 0.5f * (qw * omega_z + qx * omega_y - qy * omega_x) * kDt;

  const float q_norm = sqrtf(q_[0] * q_[0] + q_[1] * q_[1] + q_[2] * q_[2] + q_[3] * q_[3]);
  if (q_norm > 1.0e-6f) {
    const float inv_q = 1.0f / q_norm;
    q_[0] *= inv_q;
    q_[1] *= inv_q;
    q_[2] *= inv_q;
    q_[3] *= inv_q;
  }
  quaternionToEuler();
}

void IMU::update() {
  gyro_calculate();
  acc_calculate();
  sensorDataTransform();
  initFromGravity();
  if (attitude_ready_) {
    mahonyUpdate();
  }
}

void IMU::sensorDataTransform(void) {
  float filter_k = accel_filter_k_;
  if (filter_k < 0.0f) {
    filter_k = 0.0f;
  } else if (filter_k > 0.99f) {
    filter_k = 0.99f;
  }

  float gyro_body[3];
  for (uint8_t i = 0; i < 3; i++) {
    gyro_body[i] = raw_data_.gyro[0] * R_imu_[i][0] +
                   raw_data_.gyro[1] * R_imu_[i][1] +
                   raw_data_.gyro[2] * R_imu_[i][2];

    const float accel_body = accel_sen_ * (raw_data_.accel[0] * R_imu_[i][0] +
                                           raw_data_.accel[1] * R_imu_[i][1] +
                                           raw_data_.accel[2] * R_imu_[i][2]) -
                             accel_bias_[i];
    if (!accel_filter_init_) {
      accel_sensor_[i] = accel_body;
    } else {
      accel_sensor_[i] = filter_k * accel_sensor_[i] + (1.0f - filter_k) * accel_body;
    }
  }
  accel_filter_init_ = true;

  if (!gyro_bias_ready_) {
    for (uint8_t i = 0; i < 3; i++) {
      gyro_bias_sum_[i] += gyro_body[i];
    }
    gyro_bias_count_++;
    if (gyro_bias_count_ >= kGyroBiasSamples) {
      const float inv_n = 1.0f / static_cast<float>(gyro_bias_count_);
      for (uint8_t i = 0; i < 3; i++) {
        gyro_bias_[i] = gyro_bias_sum_[i] * inv_n;
      }
      gyro_bias_ready_ = true;
    }
  }

  for (uint8_t i = 0; i < 3; i++) {
    gyro_sensor_[i] = gyro_body[i] - gyro_bias_[i];
  }
}
