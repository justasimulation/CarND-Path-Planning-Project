#include "utils.h"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <memory>

#include "spline.h"
#include "math.h"
#include "config.h"

// For converting back and forth between radians and degrees.

double Utils::CalcDistance(double x1, double y1, double x2, double y2)
{
    return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

double Utils::CalcDistance(Point pt0, Point pt1)
{
    return CalcDistance(pt0.x_, pt0.y_, pt1.x_, pt1.y_);
}

double Utils::CalcDistance(double dx, double dy)
{
    return sqrt(dx * dx + dy * dy);
}

double Utils::CalcAngle(Point pt0, Point pt1)
{
    return atan2(pt1.y_ - pt0.y_, pt1.x_ - pt0.x_);
}

double Utils::CalcVelocity(Point pt0, Point pt1)
{
    return CalcDistance(pt0, pt1) / Config::kTimeStep;
}

//Finds closes map way point to given coordinates
//Just iterates over all points and find the closest one so it is linear over points number
int Utils::GetClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

    double closestLen = numeric_limits<double>::max(); //large number
    int closestWaypoint = 0;

    for(int i = 0; i < maps_x.size(); i++)
    {
        double map_x = maps_x[i];
        double map_y = maps_y[i];
        double dist = CalcDistance(x, y, map_x, map_y);
        if(dist < closestLen)
        {
            closestLen = dist;
            closestWaypoint = i;
        }

    }

    return closestWaypoint;

}

//Finds cross point of two lines: line (x1,y1)->(x2, y2) and a line perpendicular to it and passing
//through (x, y).
//Also returns true in case cross point lies between (x1, y1)->(x2, y2) and false otherwise
tuple<double, double, bool> FindCrossPoint(double x1, double y1, double x2, double y2, double x, double y)
{
    //vector from pt1 to pt2
    double dx = x2 - x1;
    double dy = y2 - y1;

    //line through pt1 and pt2
    double a1 = dy;
    double b1 = -dx;
    double c1 = -(a1 * x1 + b1 * y1);

    //line perpendicular to a1, b1, c1 and passing through x, y
    double a2 = dx;
    double b2 = dy;
    double c2 = -(a2 * x + b2 * y);

    //cross point of line1 and line2
    double x_cross = (c2 * b1 - c1 * b2) / (a1 * b2 - a2 * b1);
    double y_cross = (c2 * a1 - c1 * a2) / (b1 * a2 - b2 * a1);

    //vector from cross point to pt1
    double x_cross_1 = x1 - x_cross;
    double y_cross_1 = y1 - y_cross;

    //vector from cross point to pt2
    double x_cross_2 = x2 - x_cross;
    double y_cross_2 = y2 - y_cross;

    //norms of vectors
    double norm_cross_1 = sqrt(x_cross_1 * x_cross_1 + y_cross_1 * y_cross_1);
    double norm_cross_2 = sqrt(x_cross_2 * x_cross_2 + y_cross_2 * y_cross_2);

    //in case cross point is really close to pt1 or pt2
    if(fabs(norm_cross_1) < 1e-5 || fabs(norm_cross_2) < 1e-5)
    {
        return make_tuple(x_cross, y_cross, true);
    }
    else
    {
        //find cosine between cross vectors
        double cos_12 = (x_cross_1 * x_cross_2 + y_cross_1 * y_cross_2) / (norm_cross_1 * norm_cross_2);

        //in case cross vectors lie on the same line then an angle between them should be close to 180 degrees,
        //so cosine should be close to 1

        return make_tuple(x_cross, y_cross, fabs(cos_12 + 1) < 1e-2 ? true : false);
    }
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// 1. Find closest way point pt1 and its two adjacent waypoints pt0, pt2
// 2. Calculate distance from x,y to line segments pt0->pt1, pt1->pt2 and to the closest waypoint pt1
// 3. Choose miminum distance but only use distance to pt0->pt1 and pt1->pt2 in case cross point is located inside
//    those segments, otherwise use only distance to pt1.
// 4. Interpolate s value from the closest wapoint or use pt1.
vector<double> Utils::Convert2Frenet(double x, double y, const Map &map)
{
    //find closest waypoint and use it as a center point
    int points_num = map.GetSize();
    int closest_wp = GetClosestWaypoint(x, y, map.x_, map.y_);

    //find surrounding waypoints
    int prev_wp = closest_wp - 1 % points_num;
    int next_wp = closest_wp + 1 % points_num;

    // find cross points of each line prev->cur, cur->next and lines perpendicular to them and passing through x, y
    // also find out whether cross points line inside prev->cur, cur->next
    double cross_x0, cross_x1, cross_y0, cross_y1;
    bool is_cross0_inside, is_cross1_inside;
    tie(cross_x0, cross_y0, is_cross0_inside) = FindCrossPoint(map.x_[prev_wp], map.y_[prev_wp],
                                                               map.x_[closest_wp], map.y_[closest_wp], x, y);
    tie(cross_x1, cross_y1, is_cross1_inside) = FindCrossPoint(map.x_[closest_wp], map.y_[closest_wp],
                                                               map.x_[next_wp], map.y_[next_wp], x, y);

    double min_dist = numeric_limits<double>::max();

    double s = 0;

    //if first crosspoint is on the prev->cur line segment, then calc distance to it
    if(is_cross0_inside)
    {
        double d = CalcDistance(x, y, cross_x0, cross_y0);
        if(d < min_dist)
        {
            min_dist = d;
            double s_seg = map.s_[closest_wp] - map.s_[prev_wp];
            double dist_seg = CalcDistance(map.x_[closest_wp], map.y_[closest_wp], map.x_[prev_wp], map.y_[prev_wp]);
            double dist_cross = CalcDistance(map.x_[prev_wp], map.y_[prev_wp], cross_x0, cross_y0);

            s = map.s_[prev_wp] + s_seg * (dist_cross / dist_seg);
        }
    }

    //if second cross point is located on the cur->next line segment then calc distance to it
    if(is_cross1_inside)
    {
        double d = CalcDistance(x, y, cross_x1, cross_y1);
        if(d < min_dist)
        {
            min_dist = d;
            double s_seg = map.s_[next_wp] - map.s_[closest_wp];
            double dist_seg = CalcDistance(map.x_[closest_wp], map.y_[closest_wp], map.x_[next_wp], map.y_[next_wp]);
            double dist_cross = CalcDistance(map.x_[closest_wp], map.y_[closest_wp], cross_x1, cross_y1);

            s = map.s_[closest_wp] + s_seg * (dist_cross / dist_seg);
        }
    }

    //calc distance to waypoint itself
    double d = CalcDistance(x, y, map.x_[closest_wp], map.y_[closest_wp]);
    if(d < min_dist)
    {
        s = map.s_[closest_wp];
        min_dist = d;
    }

    //in case s is larger than circumference length
    s = map.CorrectS(s);


    return {s, min_dist};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Utils::ConvertToCartesian(double raw_s, double d, const Map &map)
{
    double s = map.CorrectS(raw_s);

    int prev_wp = -1;

    //find previous waypoint
    while(s > map.s_[prev_wp+1] && (prev_wp < (int)(map.s_.size()-1) ))
    {
        prev_wp++;
    }

    //find next waypoint
    int wp2 = (prev_wp+1)%map.x_.size();

    //find ratio of s - s[prev_wp] and s[next_wp]-s[prev_wp] for interpolation
    double r = (s-map.s_[prev_wp])/ Utils::CalcDistance(map.x_[prev_wp], map.y_[prev_wp], map.x_[wp2], map.y_[wp2]);

    //tangent vector between waypoints
    double dx = map.x_[wp2] - map.x_[prev_wp];
    double dy = map.y_[wp2] - map.y_[prev_wp];
    double dist = sqrt(dx * dx + dy * dy);

    //coordinates correspoinding to s
    double x0 = map.x_[prev_wp] + r * dx;
    double y0 = map.y_[prev_wp] + r * dy;

    //normilized tangent vector
    double n_dx = dx / dist;
    double n_dy = dy / dist;

    //line through prev->next
    double a_0 = n_dy;
    double b_0 = -n_dx;
    double c_0 = -(a_0 * x0 + b_0 * y0);

    //line perpendicular to prev->next and passing through s
    double a_1 = n_dx;
    double b_1 = n_dy;
    double c_1 = -(a_1 * x0 + b_1 * y0);

    //using d to calc coordinates lying at given distance
    //d = ax + by + c
    double y = (c_0 * a_1 - c_1 * a_0 - d * a_1) / (b_1 * a_0 - b_0 * a_1);
    double x = (c_0 * b_1 - c_1 * b_0 - d * b_1) / (b_0 * a_1 - b_1 * a_0);


    return {x,y};

}

//loads map data and interpolates it using spline for better results
Map Utils::LoadMap()
{
    Map map;

    // Load up map values for waypoint's x,y,s and d normalized normal vectors

    // Waypoint map to read from
    string map_file_ = "../data/highway_map.csv";
    // The max s value before wrapping around the track back to 0
    //map.max_s_ = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    vector<double> file_x;
    vector<double> file_y;

    string line;
    while (getline(in_map_, line))
    {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        file_x.push_back(x);
        file_y.push_back(y);
    }

    vector<double> x, y, s;

    double max_s = 0;

    int points_num = file_x.size();

    //insert last point as the first point for spline data to avoid edge anomalies
    x.push_back(file_x[points_num - 1]);
    y.push_back(file_y[points_num - 1]);
    max_s = -Utils::CalcDistance(file_x[0], file_y[0], x[0], y[0]);
    s.push_back(max_s);

    //copy loaded data
    for(int i = 0; i < points_num; i++)
    {
        x.push_back(file_x[i]);
        y.push_back(file_y[i]);
        max_s += Utils::CalcDistance(file_x[i], file_y[i], x[i], y[i]);
        s.push_back(max_s);
    }

    //insert first point as last point for spline data to avoid edge anomalies
    x.push_back(file_x[0]);
    y.push_back(file_y[0]);
    max_s += Utils::CalcDistance(file_x[0], file_y[0], x[0], y[0]);
    s.push_back(max_s);

    map.max_s_ = max_s;

    //create spline
    tk::spline x_spline;
    tk::spline y_spline;

    x_spline.set_points(s, x);
    y_spline.set_points(s, y);

    //interpolate points using configurable segment length
    double cur_s = 0;
    while(cur_s < map.max_s_)
    {
        map.s_.push_back(cur_s);
        map.x_.push_back(x_spline(cur_s));
        map.y_.push_back(y_spline(cur_s));

        cur_s += Config::kMapSegmentStep;
    }

    //add last point
    map.s_.push_back(map.max_s_);
    map.x_.push_back(map.x_[0]);
    map.y_.push_back(map.y_[0]);

    return map;
}

double Utils::MileHour2MeterSecond(double mile_per_hour)
{
    return mile_per_hour * 1609.34 / 3600;
}

double Utils::GetLaneCenterD(int lane_num)
{
    return (Config::kLaneWidth / 2.) + (lane_num * Config::kLaneWidth);
}


int Utils::ConvertD2LaneNum(double d)
{
    return (int)floor(d / Config::kLaneWidth);
}

Point Utils::CreatePointByCartesian(double x, double y, const Map &map)
{
    auto frenet = Utils::Convert2Frenet(x, y, map);
    return Point(x, y, frenet[0], frenet[1]);
}
