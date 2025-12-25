
#include "kalman/Types.hpp"
#include "logging.h"
#include <iostream>
#include <mutex>
#include <stdio.h>
#include <chrono>
#include <vector>
#include <cmath>




#include "PositionMeasurementModel.hpp"
//#include "rerun/recording_stream.hpp"
#include <kalman/UnscentedKalmanFilter.hpp>

#include <evpp/buffer.h>
#include <evpp/tcp_conn.h>
#include <evpp/tcp_server.h>

#include <rerun.hpp>
//#include <rerun/demo_utils.hpp>
#include <string>


typedef double T;



// Some type shortcuts
typedef Robot1::State<T> State;
typedef Robot1::Control<T> Control;
typedef Robot1::SystemModel<T> SystemModel;

typedef Robot1::PositionMeasurement<T> PositionMeasurement;
typedef Robot1::PositionMeasurementModel<T> PositionModel;

// using namespace rerun::demo;


void log_ukf_ellipse(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const Eigen::Vector2f& mean,
    const Eigen::Matrix2f& covariance_2d);


int main() {
  std::string addr = "0.0.0.0:9099";
  int thread_num = 3;
  evpp::EventLoop loop;
  evpp::TCPServer server(&loop, addr, "TCPEchoServer", thread_num);

  // make a mutex
  std::mutex mtx;



  const auto rec = rerun::RecordingStream(std::string("Visualization"));


  // // Try to spawn a new viewer instance.
  // Don't do it for now, let's not make this complex
  rec.spawn().exit_on_failure();

  
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

  auto now = std::chrono::system_clock::now();

    // 2. Get duration since epoch
  auto duration = now.time_since_epoch();

    // 3. Convert duration to milliseconds and get the count
  auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
  
  long long lasttime = millis;

  std::cout << "Lasttime: " << lasttime << std::endl;
  
  // dummy c, it's doesn't effect the function anyway
  auto x_ukf = ukf.predict(sys, c, 0.00);

  server.SetMessageCallback(
      [&](const evpp::TCPConnPtr &conn, evpp::Buffer *msg) {
	long long timestamp;
        double h, j, theta, d, del_t;

        std::string message = msg->ToString();

        // now deal with the things here, without ok this is finally good
        if (sscanf(message.c_str(), "%lld,%lf,%lf,%lf,%lf,%lf\n", &timestamp, &h, &j, &theta, &d,
                   &del_t) != 6) {
          LOG_ERROR << "6 Items Not Matched" << conn->remote_addr();
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

        std::cout << "Locked Mutex" << std::endl;
        
	auto time_diff = timestamp - lasttime;

        if (time_diff < 0) {
	  std::cout << "Got Old Data" << std::endl;
          mtx.unlock();
	  return; // if you get past data, reuturn 
	}
        
        x_ukf = ukf.predict(sys, c, ((double) time_diff) / 1000.0);

	x_ukf = ukf.update(model, m);

        lasttime = timestamp;


        std::cout << "Position: " << x_ukf(State::X) << " , " << x_ukf(State::Y)
                  << std::endl;
        

	auto cov = ukf.getCovariance();

        std::cout << "Covariance: " << cov(State::X, State::X) << " , "
                  << cov(State::Y, State::Y) << std::endl;


        Eigen::Matrix2f covariance_2d;

        covariance_2d(0, 0) = cov(State::X, State::X);
        covariance_2d(0, 1) = cov(State::X, State::Y);
        covariance_2d(1, 0) = cov(State::Y, State::X);
        covariance_2d(1, 1) = cov(State::Y, State::Y);

        Eigen::Vector2f mean_vec;

        mean_vec(0) = x_ukf(State::X);
        mean_vec(1) = x_ukf(State::Y);
        

        log_ukf_ellipse(rec, "Kalman_Covariance", mean_vec, covariance_2d);
        
        mtx.unlock();
	std::cout << "Unlocked Mutex" << std::endl;
        
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


// Function to visualize UKF uncertainty as an ellipse in Rerun
void log_ukf_ellipse(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const Eigen::Vector2f& mean,
    const Eigen::Matrix2f& covariance_2d)
{
    // Extract variances (diagonal elements)
    float var_x = covariance_2d(0, 0);
    float var_y = covariance_2d(1, 1);
    float cov_xy = covariance_2d(0, 1);
    
    // Compute eigenvalues and eigenvectors of covariance matrix
    // This gives us the principal axes of the uncertainty ellipse
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance_2d);
    Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
    Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
    
    // Standard deviations along principal axes (1-sigma)
    float sigma1 = std::sqrt(std::abs(eigenvalues(0)));
    float sigma2 = std::sqrt(std::abs(eigenvalues(1)));
    
    // Generate ellipse points
    const int num_points = 50;
    std::vector<rerun::Position3D> positions;
    
    for (int i = 0; i <= num_points; ++i) {
        float angle = 2.0f * M_PI * i / num_points;
        
        // Point on unit circle
        Eigen::Vector2f circle_point(std::cos(angle), std::sin(angle));
        
        // Scale by standard deviations
        Eigen::Vector2f scaled_point(sigma1 * circle_point(0), sigma2 * circle_point(1));
        
        // Rotate by eigenvectors to align with covariance axes
        Eigen::Vector2f rotated_point = eigenvectors * scaled_point;
        
        // Translate to mean position
        Eigen::Vector2f final_point = mean + rotated_point;
        
        positions.push_back({final_point(0), final_point(1), 0.0f});
    }
    
    // Log the ellipse as a line strip (closed loop)
    rec.log(
        entity_path + "/ellipse",
        rerun::LineStrips3D(positions)
            .with_radii({0.02f})
            .with_colors({rerun::Rgba32(0, 200, 255, 200)})  // Cyan with transparency
    );
    
    // Log the mean position as a point
    rec.log(
        entity_path + "/mean",
        rerun::Points3D({{mean(0), mean(1), 0.0f}})
            .with_radii({0.05f})
            .with_colors({rerun::Rgba32(255, 0, 0)})  // Red point
    );
    
    // Optional: Log the principal axes as arrows
    Eigen::Vector2f axis1 = eigenvectors.col(0) * sigma1;
    Eigen::Vector2f axis2 = eigenvectors.col(1) * sigma2;
    
    rec.log(
        entity_path + "/axis1",
        rerun::Arrows3D::from_vectors({{axis1(0), axis1(1), 0.0f}})
            .with_origins({{mean(0), mean(1), 0.0f}})
            .with_colors({rerun::Rgba32(255, 100, 100, 150)})
    );
    
    rec.log(
        entity_path + "/axis2",
        rerun::Arrows3D::from_vectors({{axis2(0), axis2(1), 0.0f}})
            .with_origins({{mean(0), mean(1), 0.0f}})
            .with_colors({rerun::Rgba32(100, 255, 100, 150)})
    );
}

// Simpler version without axes visualization
void log_ukf_ellipse_simple(
    const rerun::RecordingStream& rec,
    const std::string& entity_path,
    const Eigen::Vector2f& mean,
    const Eigen::Matrix2f& covariance_2d)
{
    // Compute eigenvalues and eigenvectors
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance_2d);
    Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
    Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
    
    // Standard deviations (1-sigma)
    float sigma1 = std::sqrt(std::abs(eigenvalues(0)));
    float sigma2 = std::sqrt(std::abs(eigenvalues(1)));
    
    // Generate ellipse points
    const int num_points = 50;
    std::vector<rerun::Position3D> positions;
    
    for (int i = 0; i <= num_points; ++i) {
        float angle = 2.0f * M_PI * i / num_points;
        Eigen::Vector2f circle_point(std::cos(angle), std::sin(angle));
        Eigen::Vector2f scaled_point(sigma1 * circle_point(0), sigma2 * circle_point(1));
        Eigen::Vector2f rotated_point = eigenvectors * scaled_point;
        Eigen::Vector2f final_point = mean + rotated_point;
        
        positions.push_back({final_point(0), final_point(1), 0.0f});
    }
    
    // Log ellipse and mean
    rec.log(
        entity_path + "/uncertainty_ellipse",
        rerun::LineStrips3D(positions)
            .with_radii({0.02f})
            .with_colors({rerun::Rgba32(0, 200, 255, 200)})
    );
    
    rec.log(
        entity_path + "/position",
        rerun::Points3D({{mean(0), mean(1), 0.0f}})
            .with_radii({0.05f})
            .with_colors({rerun::Rgba32(255, 0, 0)})
    );
}
