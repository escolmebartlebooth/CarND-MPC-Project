#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// added to control when to print out additional helpers
int useDebug = 0;

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
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Evaluate a polynomial's first derivative.
double polyevalpderivative(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * pow(x, i-1);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          // capture the actuator state
          double delta = j[1]["steering_angle"];
          double alpha = j[1]["throttle"];

          // if debug is set print values
          if (useDebug == 1) {
            cout << "state: " << px << " " << py << " " << psi << " " << v << " " << delta << " " << alpha << endl;
          }
          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

          // first up convert the ptsx and ptsy vectors to car coordinates
          // this is done by subtracting the global px and py from each coordinate
          // and then using trig to calculate the car coord enquivalent position
          // we take to an EigenVector as that's what polyfit wants

          const size_t n_points = ptsx.size();
          auto car_ptsx = Eigen::VectorXd(n_points);
          auto car_ptsy = Eigen::VectorXd(n_points);

          for (unsigned int i=0; i < n_points; i++){
            double dx = ptsx[i] - px;
            double dy = ptsy[i] - py;

            car_ptsx(i) = dx * cos(-psi)-dy*sin(-psi);
            car_ptsy(i) = dx * sin(-psi)+dy*cos(-psi);

          }

          if (useDebug == 1) {
            cout << "car waypoints X: " << car_ptsx << endl;
            cout << "car waypoints Y: " << car_ptsy << endl;
          }

          // now calculate the coefficients of a 3rd order polynomial
          // fitted to the way pointts in car coords
          auto coeffs = polyfit(car_ptsx, car_ptsy, 3);
          if (useDebug == 1) {
            cout << "coeffs: " << coeffs << endl;
          }

          // now capture initial state
          // x, y, psi are all 0 in car coordinates
          // v is the current velocity of the vehicle
          double x0 = 0;
          double y0 = 0;
          double psi0 = 0;
          double v0 = v;

          // calculate the error terms
          // the cross-track error is how far off the y track the car is
          // this will be equivalent to polyeval(coeffs, 0) given initial state
          double cte0 = polyeval(coeffs, x0) - y0;
          // the angle error is calculated from the 1st derivative of the coeffs
          double epsi0 = psi0 - atan(polyevalpderivative(coeffs, x0));
          if (useDebug == 1) {
            cout << "car state: " << x0 << " " << y0 << " " << psi0 << " " << v0 << endl;
            cout << "cte: " << cte0 << endl;
            // didn't trust my function...
            double epsi1 = psi0 - atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * x0 * x0);
            cout << "epsi and x-check: " << epsi0 << " " << epsi1 << endl;
          }
          
          // model latency in the actuation with motion models
          double Lf = 2.67;
          double delay = 0.1;
          x0 = x0 + v0*cos(psi0)*delay;
          y0 = y0 + v0*sin(psi0)*delay;
          psi0 = psi0 - v0/Lf*delta*delay;
          v0 = v0 + alpha*delay;
          cte0 = cte0  + v0*sin(epsi0)*delay;
          epsi0 = epsi0 - v0/Lf*delta*delay;

          // create state vector
          Eigen::VectorXd state(6);
          state << x0, y0, psi0, v0, cte0, epsi0;
          if (useDebug == 1) {
            cout << "delayed state: " << state << endl;
          }
          // solve for current state
          auto vars = mpc.Solve(state, coeffs);
          if (useDebug == 1) {
            cout << "result: " << vars[0] << " " << vars[1] << endl;
          }

          double steer_value = -vars[0]/deg2rad(25);
          double throttle_value = vars[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          // we have the predicted way points in the vars result vector from mpc.Solve
          // x is N in length from position 2
          // y is N in length from position 2 + N
          // vars is 2 * N + 2 in length
          // TO DO should move N to a common value in mpc class
          int N = (vars.size() - 2) / 2;

          for (int i=0; i<N-1; i++) {
            mpc_x_vals.push_back(vars[2+i]);
            mpc_y_vals.push_back(vars[2+i+N]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          // for a few steps at a stride length calculate way points based on the coeffs
          // TO DO work out why we're not simply using car_ptsx and car_ptsy
          int step_size = 2;
          int step_count = 10;
          for (int i=0; i<step_count; i++) {
            int nx = step_size * i;
            next_x_vals.push_back(nx);
            next_y_vals.push_back(polyeval(coeffs, nx));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
