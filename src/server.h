#ifndef PATH_PLANNING_SERVER_H
#define PATH_PLANNING_SERVER_H

#include <iostream>
#include "planner.h"
#include "json.hpp"

/***
 * This class encapsulates connection to simulator.
 * It is a wrapper over uWS methods and consists mainly of the course starter code.
 *
 * The main functionality:
 * 1) get data from simulator
 * 2) repack it into more convenient structures
 * 3) pass to business logic modules
 * 4) get result from business logic and return it to simulator
 */
class Server
{
private:
    Planner &planner_;

private:
    // Checks if the SocketIO event has JSON data.
    // If there is data the JSON object in string format will be returned,
    // else the empty string "" will be returned.
    std::string hasData(std::string s);

public:
    Server(Planner &planner);

    /***
     * Starts the server and does not return until execution is over.
     * @return return code for "main" method.
     */
    int Run();

    /***
     * Parses input json into convenience class
     * @param json
     * @return
     */
    PlannerInput ParseInput(nlohmann::json &json);
};


#endif //PATH_PLANNING_SERVER_H
