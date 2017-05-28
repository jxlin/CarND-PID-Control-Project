#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <fstream>
#include <string>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  // TODO: Initialize the pid variable.
  PID pid;
  // Write initial config to file, avoid recompile while
  // tuning PID control parameters
  // 0.2 0.0 1.0 as a good start
  std::ofstream logfile;
  std::string logfilename = "./pid.log";
  logfile.open(logfilename);
  if (logfile.is_open()) {
    std::cout << "Writing log to" << logfilename << std::endl;
    logfile << "Start PID control\n";
  } else {
    std::cout << "Failed to open log file " << logfilename << std::endl;
    return 0;
  }

  // Coordinate Descent
  double p[3] = {1.91917,0.0,15.744};
  double dp[3] = {0.1, 0.00001, 0.1};
  double best_err = 999999;
  bool initialized = false;
  int cycle = 0;    // num of trained cycles
  int step = 0;     // num steps trained in this cycle
  int phase = 0;    // current phase in coordinate descent cycle

  h.onMessage([&pid, &cycle, &step, &p, &dp, &best_err, &logfile, &phase, &initialized](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        if (!initialized && step == 0) {
          // force reset at the start
          std::string reset_msg = "42[\"reset\", {}]";
          ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);

          pid.Init(p[0], p[1], p[2]);
          logfile << "Init: p[" << p[0] << "," << p[1] << "," << p[2] << "]" << std::endl;

        } else if (step == 0) {
          int direction = cycle % 3;    // current descent direction
          if (phase == 0){  // phase 0, start of a new cycle
            logfile << "Cycle " << cycle << ": dp[" << dp[0] << "," << dp[1] << "," << dp[2] << "]" << std::endl;
            p[direction] += dp[direction];
            pid.Init(p[0], p[1], p[2]);
          } else if (phase == 1) {  // phase 1, err >= best_err, try another direction
            p[direction] -= 2 * dp[direction];
            pid.Init(p[0], p[1], p[2]);
          }
          logfile << "\tp[" << p[0] << "," << p[1] << "," << p[2] << "]" << std::endl;
        }

        // Train PID control
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);

          json msgJson;
          msgJson["steering_angle"] = pid.steer_value;
          msgJson["throttle"] = pid.throttle;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }

        // On finish, compare error and descent dp
        step += 1;
        if (step % 100 == 0) {
          std::cout << "step:" << step << std::endl;
        }

        // End of this cycle
        if (step == 1200) {
          if (!initialized) {
            best_err = pid.TotalError();
            initialized = true;
            logfile << "Initial best_err: " << best_err << std::endl;
          } else {

            // Update delta according to error change
            int direction = cycle % 3;    // current descent direction
            double pid_err = pid.TotalError();
            logfile << "pid_err:" << pid_err << " best_err:" << best_err << std::endl;
            if (pid_err < best_err) {
              best_err = pid_err;
              dp[direction] *= 1.1;
              phase = 0;
              cycle += 1;
            } else {
              phase += 1;
              if (phase == 2) {  // already tried 2 directions
                p[direction] += dp[direction];
                dp[direction] *= 0.9;
                phase = 0;
                cycle += 1;
              }
            }
          }

          // Reset for next training cycle
          step = 0;
          std::string reset_msg = "42[\"reset\", {}]";
          ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h, &logfile](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    logfile.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
