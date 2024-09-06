#include <mbed.h>
#include "RogiLinkFlex/UartLink.hpp"
#include "catch24/pid_controller.cpp"

UartLink uart1(USBTX, USBRX, 115200, nullptr, 1);

UartLinkSubscriber<float> sub(uart1, 2);
UartLinkPublisher<float> pub(uart1, 2);

void sub_callback(float a) {
  pub.publish(a);
  printf("Received: %f\n", a);
}
float goal_r = 0.4;
float goal_theta = 1.5;
float goal_z = 0.3;
float measured_r = 0.4;
float measured_theta = 0.0;
float measured_z = 0.3;
float next_r = 0.4;
float next_theta = 0.0;
float next_z = 0.3;

PIDController pid_r(0.001, 0.0, 0.0, 10.0);
PIDController pid_theta(0.001, 0.0, 0.0, 10.0);
PIDController pid_z(0.001, 0.0, 0.0, 10.0);

int main() {
  while(1) {
    sub.set_callback(sub_callback);
    next_r = measured_r + pid_r.update(goal_r, measured_r);
    next_theta = measured_theta + pid_theta.update(goal_theta, measured_theta);
    next_z = measured_z + pid_z.update(goal_z, measured_z);    
    printf("r: %d, theta: %d, z: %d\n", int(100*next_r), int(100*next_theta), int(100*next_z));
    measured_r = next_r;
    measured_theta = next_theta;
    measured_z = next_z;

  }
}