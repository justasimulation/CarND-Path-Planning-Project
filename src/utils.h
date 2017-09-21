#ifndef PATH_PLANNING_UTILS_H
#define PATH_PLANNING_UTILS_H

#include <vector>

#include "point.h"
#include "map.h"

using namespace std;

/***
 * Helpers.
 */
class Utils
{
public:
    static double CalcDistance(double x1, double y1, double x2, double y2);
    static double CalcDistance(double dx, double dy);
    static double CalcDistance(Point pt0, Point pt1);
    static double CalcAngle(Point pt0, Point pt1);
    static double CalcVelocity(Point pt0, Point pt1);

    static int GetClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);

    // Transform from Cartesian x,y coordinates to Frenet s,d coordinates
    static vector<double> Convert2Frenet(double x, double y, const Map &map);
    // Transform from Frenet s,d coordinates to Cartesian x,y
    static vector<double> ConvertToCartesian(double s, double d, const Map &map);

    static Map LoadMap();

    static double MileHour2MeterSecond(double mile_per_hour);

    static double GetLaneCenterD(int lane_num);
    static int    ConvertD2LaneNum(double d);

    static Point CreatePointByCartesian(double x, double y, const Map &map);
};


#endif //PATH_PLANNING_UTILS_H
