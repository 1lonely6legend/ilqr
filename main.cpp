#include <iostream>
#include <chrono>

#include "ilqr.h"

class SimpleVehicleILQR : public ILQR<3, 2> {
  //一个简单的车辆模型，状态为[x, y, theta]，控制为[v, delta]，接受控制输入，返回状态变化
  vec<3> dynamics(vec<3> state, vec<2> action, int) override {
    double frontAxelDistCog = 0.1125; //前轴到重心的距离
    double rearAxelDistCog = 0.1125; //后轴到重心的距离
    //重心到后轴的距离与重心到前轴的距离的比值
    double distRel = rearAxelDistCog / (frontAxelDistCog + rearAxelDistCog);

    double theta = state[2];
    double v = action[0];
    double delta = action[1];
    //计算beta角，beta角是车辆的侧偏角
    double beta = std::atan(distRel * std::tan(delta));
    //计算车辆的状态变化，也就是车辆的运动学模型
    vec<3> stateChange = vec<3>::Zero();
    stateChange[0] = v * std::cos(theta + beta);
    stateChange[1] = v * std::sin(theta + beta);
    stateChange[2] = v / rearAxelDistCog * std::sin(beta);
    //定义时间间隔
    double dt = 0.05;
    //返回状态变化
    return state.array() + stateChange.array() * dt;
  }

  //定义代价函数，这里的代价函数是状态和控制的二次型代价函数
  double costs(vec<3> x, vec<2> u, int) override {
    double frontAxelDistCog = 0.1125;
    double rearAxelDistCog = 0.1125;
    double distRel = rearAxelDistCog / (frontAxelDistCog + rearAxelDistCog);
    double beta = std::atan(distRel * std::tan(u[1]));
    double v = std::max(u[0], 0.001);
    double thetaChange = v / rearAxelDistCog * std::sin(beta);
    //返回代价函数，定义为状态和控制的二次型代价函数，具体为状态量位置xy和控制量u的平方和再乘以各自的系数
    return 0.05 * (x[0] * x[0] + x[1] * x[1] + std::pow(u[0] - 0.5, 2) * 10 + thetaChange * thetaChange);
  }
};

int main(int, char **) {
  const int horizon = 20;

  SimpleVehicleILQR ilqr;
  ilqr.x[0] = {2.0, 2.0, 0.0};

  std::chrono::steady_clock::time_point begin
      = std::chrono::steady_clock::now();

  ilqr.update();

  std::chrono::steady_clock::time_point end
      = std::chrono::steady_clock::now();

  std::cout
      << "Time difference = "
      << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()
      << "[us]"
      << std::endl;

  std::cout << "States:" << std::endl;
  for (int i = 0; i < horizon; i++) {
    std::cout << ilqr.x[i] << std::endl;
  }

  std::cout << "Actions:" << std::endl;
  for (int i = 0; i < horizon; i++) {
    std::cout << ilqr.u[i] << std::endl;
  }
}
