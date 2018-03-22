#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "cost.h"
#include "siso_pid.h"
#include "mimo_pid.h"
#include <chrono>
#include "telemetry.h"
#include "twiddle.h"
#include <sstream>
#include "Eigen/Dense"

/**
 * Checks if the SocketIO event has JSON data.
 * If there is data the JSON object in string format will be returned,
 * else the empty string "" will be returned.
 * */
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_last_of("]");
    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }
    return "";
}

template<typename T>
T stringTo(const std::string &str) {
    std::istringstream ss(str);
    T num;
    ss >> num;
    return num;
}

template<typename WS, typename T>
void send(WS &ws, T steering_angle, T throttle) {
    nlohmann::json msgJson;
    msgJson["steering_angle"] = steering_angle;
    msgJson["throttle"] = throttle;
    auto msg = "42[\"steer\"," + msgJson.dump() + "]";
    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main(int argc, char *argv[]) {

    using T = double;

    /* See if the user passed default gains */
    const T kp = argc > 1 ? stringTo<T>(argv[1]) : 0.6;
    const T ki = argc > 2 ? stringTo<T>(argv[2]) : 0.006;
    const T kd = argc > 3 ? stringTo<T>(argv[3]) : 1.5;
    const T default_throttle = argc > 4 ? stringTo<T>(argv[4]) : 0.3;

    uWS::Hub h;

    /* The number of control inputs and measurements are compile-time constants.
     * Specifically, if you are just designing a controller for the steering
     * angle, then you would use n_controls = 1. If you want to develop a controller
     * that will also adjust the throttle, then n_controls = 2. And so on...
     *
     * Similarly, n_measurements denotes the number of measurements whose
     * error sources will be used to influence the PID values. In our case,
     * we only have 1 error signal: the crosstrack error, so
     * n_measurements = 1; */
    const int n_controls = 1;
    const int n_measurements = 1;

    /* Create the initial PID gains and controller to be used. */
    using Control = Eigen::Matrix<T, n_controls, 1>;
    using Gain = Eigen::Matrix<T, n_controls, n_measurements>;
    const Gain k_p = Gain::Ones() * kp;
    const Gain k_i = Gain::Ones() * ki;
    const Gain k_d = Gain::Ones() * kd;
    using Parameters = MimoPID<T, n_controls, n_measurements>;
    Parameters pid(k_p, k_i, k_d);

    /* The first thing we need is a stable estimate of the sampling interval. This should be
     * constant, so we average this value over a few iterations. */
    const T dt = .015; // USING "FASTEST" SETTING IN SIMULATOR

    /* This is the error object that we will be updating with every new measurement. */
    using Error = Eigen::Matrix<T, n_measurements, 1>;
    Error error = Error::Zero();

    /* Execute this for every message we receive */
    h.onMessage(
            [&default_throttle,
                    &pid,
                    &dt,
                    &error](
                    uWS::WebSocket <uWS::SERVER> ws,
                    char *data, size_t length,
                    uWS::OpCode opCode) {

                /* "42" at the start of the message means there's a websocket message event.
                       The 4 signifies a websocket message
                       The 2 signifies a websocket event */
                if (length && length > 2 && data[0] == '4' && data[1] == '2') {
                    const auto s = hasData(std::string(data).substr(0, length));
                    if (s != "") {

                        const auto parsed_json = nlohmann::json::parse(s);
                        const std::string event = parsed_json[0].get<std::string>();
                        if (event == "telemetry") {

                            /* Create the telemetry event
                             * Note that parsed_json[1] is the data JSON object */
                            const T crosstrack_error = stringTo<T>(parsed_json[1]["cte"].get<std::string>());
                            const T speed = stringTo<T>(parsed_json[1]["speed"].get<std::string>());
                            const T steering_angle = stringTo<T>(
                                    parsed_json[1]["steering_angle"].get<std::string>());
                            const T throttle = stringTo<T>(parsed_json[1]["throttle"].get<std::string>());
                            const Telemetry<T> telemetry(crosstrack_error, speed, steering_angle, throttle, dt);

                            /* Show all of the values we are getting from telemetry */
                            const bool verbose = false;
                            if (verbose) {
                                std::cout << "crosstrack_error = " << crosstrack_error << std::endl;
                                std::cout << "speed            = " << speed << std::endl;
                                std::cout << "steering_angle   = " << steering_angle << std::endl;
                                std::cout << "throttle         = " << throttle << std::endl;
                                std::cout << "dt               = " << dt << std::endl;
                            } else if (false) {
                                std::cout << crosstrack_error << ","
                                          << speed << ","
                                          << steering_angle << ","
                                          << throttle << ","
                                          << dt << std::endl;
                            }

                            /* Get the control values from the pid controller */
                            error(0) = telemetry.crosstrack_error;
                            const Control control = pid.update(error, telemetry.dt);
                            const T new_throttle = n_controls == 1 ? default_throttle : control(1);

                            /* Make sure that the steering angle is in the range [-1,1] */
                            T new_steering_angle = control(0);
                            if (new_steering_angle < -1) {
                                new_steering_angle = -1;
                            } else if (new_steering_angle > 1) {
                                new_steering_angle = 1;
                            }

                            /* Now send our control commands over the web socket */
                            send(ws, new_steering_angle, new_throttle);
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
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket <uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket <uWS::SERVER> ws, int code, char *message, size_t length) {
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
