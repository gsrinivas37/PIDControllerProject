#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int twiddle_mode = 0; //Set this to 1 to tune parameters.
int twiddle_iterations = 1000;  //Number of iterations for twiddle tuning.
double max_init_error = 100000;

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

	PID pid;

	//Initial values for parameters.
	//	pid.Init(1,0.01,3);
	//	pid.InitTwiddle(0.5,0.005,1);

	//After tuning parameter with Twiddle. Seems to be working but the steering is too abrupt.
//	pid.Init(1.43,0.011,9.3);
//	pid.InitTwiddle(0.5,0.005,1);

	//Adjusting the parameters by a factor of 10 seems to fix the abrupt steering angle and make it smooth.
//	pid.Init(0.143,0.0011,0.93);

//	pid.Init(0.143,0,0);  //Trying only P controller
//	pid.Init(0.143,0,0.93);  //Trying P and D Controller.
	pid.Init(0.143,0.0011,0.93);  // PID Controller

	pid.setBestError(max_init_error);

	h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(std::string(data).substr(0, length));
			if (s != "") {
				auto j = json::parse(s);
				std::string event = j[0].get<std::string>();
				if (event == "telemetry") {
					// j[1] is the data JSON object
					double cte = std::stod(j[1]["cte"].get<std::string>());
					double speed = std::stod(j[1]["speed"].get<std::string>());
					double angle = std::stod(j[1]["steering_angle"].get<std::string>());

					pid.UpdateError(cte);
					double steer_value = pid.getSteeringAngle();
					if (steer_value>1){
						steer_value=1;
					}

					else if (steer_value<-1){
						steer_value=-1;
					}

					/*
					 * TODO: Calcuate steering value here, remember the steering value is
					 * [-1, 1].
					 * NOTE: Feel free to play around with the throttle and speed. Maybe use
					 * another PID controller to control the speed!
					 */

					// DEBUG
					std::cout << pid.getIterationCount()<<":CTE: " << cte << " Steering Value: " << steer_value << std::endl;

					if(twiddle_mode && pid.getIterationCount()==twiddle_iterations) {
						if(pid.getBestError()==max_init_error)
							pid.setBestError(pid.getAverageError());
						std::cout<<"Reseting the simulator and the average error is: "<<pid.getAverageError()<<std::endl;
						pid.printValues();
						pid.updateTwiddleParams(pid.getAverageError());
						std::string reset_msg = "42[\"reset\",{}]";
						ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
					}else{
						json msgJson;
						msgJson["steering_angle"] = steer_value;
						msgJson["throttle"] = 0.3;
						auto msg = "42[\"steer\"," + msgJson.dump() + "]";
						//std::cout << msg << std::endl;
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
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

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
		ws.close();
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
