#include "kalman/Types.hpp"
#include "logging.h"
#include <iostream>
#include <mutex>
#include <stdio.h>

#include "PositionMeasurementModel.hpp"
#include <kalman/UnscentedKalmanFilter.hpp>

#include <evpp/buffer.h>
#include <evpp/tcp_conn.h>
#include <evpp/tcp_server.h>

typedef double T;

// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;

int main() {
  std::string addr = "0.0.0.0:9099";
  int thread_num = 3;
  evpp::EventLoop loop;
  evpp::TCPServer server(&loop, addr, "TCPEchoServer", thread_num);

  // make a mutex
  std::mutex mtx;

  // do UKF stuff here
  State x;
  x.setZero();
  SystemModel sys;
  Kalman::Covariance<State> state_cov;

  state_cov(State::X, State::X) = 0.01;
  state_cov(State::Y, State::Y) = 0.01;
  state_cov(State::VX, State::VX) = 0.1;
  state_cov(State::VY, State::VY) = 0.1; 
  
  sys.setCovariance(state_cov);
  
  Kalman::UnscentedKalmanFilter<State> ukf(1);
  Control c;
  c.setZero();

  ukf.init(x);
  double lasttime = 0;

  // dummy c, it's doesn't effect the function anyway
  auto x_ukf = ukf.predict(sys, c, 0.00);

  server.SetMessageCallback(
      [&](const evpp::TCPConnPtr &conn, evpp::Buffer *msg) {
        double timestamp, h, j, theta, d, del_t;

        std::string message = msg->ToString();

        // now deal with the things here, without ok this is finally good
        if (sscanf(message.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf\n", &timestamp, &h, &j, &theta, &d,
                   &del_t) != 6) {
          LOG_ERROR << "5 Items Not Matched" << conn->remote_addr();
          return;
        }

        std::cout << h << "," << j << "," << theta << "," << d << "," << del_t
                  << std::endl;

        // now we can do whatever we want exactly here, the goal is to run UKF
        // and send states and covariances to another server, probably run in
        // python

        PositionModel model(h, j, theta, d);

        Kalman::Covariance<PositionMeasurement> sensor_cov;
	sensor_cov(PositionMeasurement::DEL_T, PositionMeasurement::DEL_T) = (1.0 / 20000) * (1.0 / 20000);

        
        PositionMeasurement m;
        m.del_t() = del_t;

        /// Now lock mutex and use kalman filter
        mtx.lock();

	auto time_diff = timestamp - lasttime;

	if (time_diff < 0) return; // if you get past data, reuturn
        
        x_ukf = ukf.predict(sys, c, time_diff);

	x_ukf = ukf.update(model, m);

        lasttime = timestamp;


        std::cout << "Position: " << x_ukf(State::X) << " , " << x_ukf(State::Y)
                  << std::endl;
        

	auto cov = ukf.getCovariance();

        std::cout << "Covariance: " << cov(State::X, State::X)  << " , " << cov(State::Y, State::Y)
                  << std::endl;
        
        mtx.unlock();

        
        msg->Reset();
        // conn->Send(msg);
      });
  server.SetConnectionCallback([](const evpp::TCPConnPtr &conn) {
    if (conn->IsConnected()) {
      LOG_INFO << "A new connection from " << conn->remote_addr();
    } else {
      LOG_INFO << "Lost the connection from " << conn->remote_addr();
    }
  });
  server.Init();
  server.Start();
  loop.Run();
  return 0;
}
