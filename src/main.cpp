#include "planner.h"
#include "server.h"


#include <math.h>

using namespace std;

//Entry point
int main()
{
    //All the business logic is in the planner
    Planner planner;

    Server server(planner);

    return server.Run();
}
















































































