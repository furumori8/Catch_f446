#include <cmath>

class PIDController {
 public:
  PIDController(float Kp, float Ki, float Kd, float max_control)
      : Kp(Kp),
        Ki(Ki),
        Kd(Kd),
        max_control(max_control),
        integral(0),
        previous_error(0) {}

  float update(float setpoint, float measured_value) {
    // 誤差を計算
    float error = setpoint - measured_value;

    // 比例項
    float P = Kp * error;

    // 積分項
    integral += error;
    float I = Ki * integral;

    // 微分項
    float derivative = error - previous_error;
    float D = Kd * derivative;

    // 総出力を計算
    float output = P + I + D;

    // 前回の誤差を更新
    previous_error = error;

    // 出力を最大制御量に制限
    if (output > max_control) {
      output = max_control;
    } else if (output < -max_control) {
      output = -max_control;
    }

    return output;
  }

 private:
  float Kp, Ki, Kd, max_control;
  float integral;
  float previous_error;
};

class Theta_PIDController {
 public:
  Theta_PIDController(float Kp, float Ki, float Kd, float max_control)
      : Kp(Kp),
        Ki(Ki),
        Kd(Kd),
        max_control(max_control),
        integral(0),
        previous_error(0) {}

  float update(float setpoint, float measured_value) {
    // 誤差を計算
    float diff = setpoint - measured_value;
    float error;

    if (-M_PI / 2 < setpoint && setpoint < 0 && M_PI / 2 < measured_value &&
        measured_value < M_PI && measured_value - setpoint > M_PI) {
      error = diff;
    } else if (M_PI / 2 < setpoint && setpoint < M_PI &&
               -M_PI / 2 < measured_value && measured_value < 0 &&
               setpoint - measured_value > M_PI) {
      error = diff;
    } else {
      error = normalize_theta(diff);
    }

    // 比例項
    float P = Kp * error;

    // 積分項
    integral += error;
    float I = Ki * integral;

    // 微分項
    float derivative = error - previous_error;
    float D = Kd * derivative;

    // 総出力を計算
    float output = P + I + D;

    // 前回の誤差を更新
    previous_error = error;

    // 出力を最大制御量に制限
    if (output > max_control) {
      output = max_control;
    } else if (output < -max_control) {
      output = -max_control;
    }

    return output;
  }

  // 角度を正規化する関数
  float normalize_theta(float angle) {
    while (angle > M_PI) {
      angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
      angle += 2 * M_PI;
    }
    return angle;
  }

 private:
  float Kp, Ki, Kd, max_control;
  float integral;
  float previous_error;
};
