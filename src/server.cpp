#include "server.h"

#include <vector>
#include <fstream>
#include <sstream>
#include "json.hpp"

#include "uWS/uWS.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

#include "utils.h"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

Server::Server(Planner &planner) : planner_(planner) {}

PlannerInput Server::ParseInput(nlohmann::json &json)
{
    PlannerInput input;

    // Main car's localization Data
    input.x_ = json[1]["x"];
    input.y_ = json[1]["y"];
    input.velocity_ = json[1]["speed"]; // this one doesn't work at some point doubles the values

    // Previous path data given to the Planner
    auto previous_path_x = json[1]["previous_path_x"];
    auto previous_path_y = json[1]["previous_path_y"];

    for(int i = 0; i < previous_path_x.size(); i++)
    {
        input.prev_x_.push_back(previous_path_x[i]);
        input.prev_y_.push_back(previous_path_y[i]);
    }

    // Previous path's end s and d values
    //input.end_s_ = j[1]["end_path_s"];
    //input.end_d_ = j[1]["end_path_d"];

    // Sensor Fusion Data, a list of all other cars on the same side of the road.
    auto sensor_fusion = json[1]["sensor_fusion"];
    for(int i = 0; i < sensor_fusion.size(); i++)
    {
        CarObservation observation;
        observation.id_ = sensor_fusion[i][0];
        observation.x_ = sensor_fusion[i][1];
        observation.y_ = sensor_fusion[i][2];
        observation.vx_ = sensor_fusion[i][3];
        observation.vy_ = sensor_fusion[i][4];
        observation.s_ = sensor_fusion[i][5];
        observation.d_ = sensor_fusion[i][6];

        input.sensor_fusion_.push_back(observation);
    }

    return input;
}

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string Server::hasData(string s)
{
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("}");
    if (found_null != string::npos)
    {
        return "";
    }
    else if (b1 != string::npos && b2 != string::npos)
    {
        return s.substr(b1, b2 - b1 + 2);
    }
    return "";
}

/***
 * Starts the server and does not return untill execution is over.
 * @return return code for "main" method.
 */
int Server::Run()
{
    uWS::Hub h;


    // The max s value before wrapping around the track back to 0
    //double max_s = 6945.554;

    h.onMessage([this](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
    {
        // "42" at the start of the message means there's a websocket message event.
        // The 4 signifies a websocket message
        // The 2 signifies a websocket event
        //auto sdata = string(data).substr(0, length);
        //cout << sdata << endl;
        if (length && length > 2 && data[0] == '4' && data[1] == '2')
        {
            auto s = hasData(data);

            if (s != "")
            {
                auto j = json::parse(s);

                string event = j[0].get<string>();

                if (event == "telemetry")
                {
                    // j[1] is the data JSON object

                    PlannerInput input = ParseInput(j);

                    // do planning
                    auto trajectory = planner_.GetNextTrajectory(input);

                    vector<double> next_x, next_y;
                    tie(next_x, next_y) = trajectory.ToVectors();

                    json msgJson;
                    // return planned path to simulator
                    msgJson["next_x"] = next_x;
                    msgJson["next_y"] = next_y;

                    auto msg = "42[\"control\","+ msgJson.dump()+"]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                }
            }
            else
            {
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
                       size_t, size_t)
    {
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

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req)
    {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length)
    {
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

