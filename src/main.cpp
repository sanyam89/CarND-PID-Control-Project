#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <vector>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID steer_pid, speed_pid;
  /**
   * TODO: Initialize the pid variable.
   */

  steer_pid.Init(.1,.001,1);
  speed_pid.Init(.1,.001,1);
  
  
  h.onMessage([&steer_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value, speed_value, speed_error, target_speed;
          /**
           * TODO: Calculate steering value here, remember the steering value is
           *   [-1, 1].
           * NOTE: Feel free to play around with the throttle and speed.
           *   Maybe use another PID controller to control the speed!
           */
          
          steer_pid.UpdateError(cte);			//Update the Error for Steering	
          steer_value = steer_pid.Result();		//Calculate new steering angle based on the PID gains
          
          // A target speed of 85 mph was assumed and 35mph was chosen as the minimum speed for the sharpest turns.
          // The target speed should be more when there is less steering input
          // A gain of 2.5 was used to transfer steering wheel input to target speed, i.e. more steering input would lead to less variation in speed
          target_speed = 50*(1-2.5*steer_value) + 35;	
          speed_error = speed - target_speed;	// Calculate speed error
          
          speed_pid.UpdateError(speed_error);	//Update the Error for Speed
          speed_value = speed_pid.Result();		//Calculate new speed based on the PID gains
          
          // We want to run Twiddle after 100 times the error has been updated, 
          // so that we don't just keep on changing the PID gains without letting the vehicle to settle.
          // Also, we are running the Twiddle algorithm for only first 100 times per model input(steering angle, speed)
          
          if (steer_pid.update_count > 100 && steer_pid.twiddle_count <=100)
          {
            steer_pid.Twiddle();
            std::cout<<"Running Steer Twiddle" << std::endl;
          }
          if (speed_pid.update_count > 100 && speed_pid.twiddle_count <=100)
          {
            speed_pid.Twiddle();
            std::cout<<"Running Speed Twiddle" << std::endl;
          }
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
                    << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = speed_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}