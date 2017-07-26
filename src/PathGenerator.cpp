//
// Created by Jose Rojas on 7/23/17.
//

#include "PathGenerator.h"

class PathGenerator::impl {
public:

    ~impl();
    int find_waypoint_floor(double s);

    Waypoints waypoints_;
    double s_max_;
    LoopingUniformCRSpline<Vector<2>>* spline_;
};

PathGenerator::PathGenerator() : pimpl(new impl()) {

}

PathGenerator::~PathGenerator() = default;

PathGenerator::PathGenerator(Waypoints waypoints, double s_max) : PathGenerator() {

    impl& im = *pimpl;

    im.waypoints_ = waypoints;
    im.s_max_ = s_max;

    std::vector<Vector<2>> xv;
    std::vector<Vector<1>> sv;

    for (int i = 0; i < waypoints.s.size(); i++) {
        Vector<2> x;
        x[0] =(float) (im.waypoints_.x[i] + im.waypoints_.dx[i] * 14.0f);
        x[1] =(float) (im.waypoints_.y[i] + im.waypoints_.dy[i] * 14.0f);

        Vector<1> s;
        s[0] = (float) im.waypoints_.s[i];

        xv.push_back(x);
        sv.push_back(s);
    }

    im.spline_ = new LoopingUniformCRSpline<Vector<2>>(xv);
}

PathPoints PathGenerator::generate_path(VehicleState state) {

    impl& im = *pimpl;

    //generate 50 points along this path in one second
    PathPoints retval;

    int prevWaypoint = im.find_waypoint_floor(state.s);
    int nextWaypoint = prevWaypoint + 1;

    if (nextWaypoint == prevWaypoint) {
        nextWaypoint++;
    }

    if (prevWaypoint < 0) {
        prevWaypoint += im.waypoints_.s.size();
    }

    if (nextWaypoint >= im.waypoints_.s.size()) {
        nextWaypoint -= im.waypoints_.s.size();
    }

    int waypointDiff = nextWaypoint - prevWaypoint;
    double s_diff = im.waypoints_.s[nextWaypoint] - im.waypoints_.s[prevWaypoint];

    if (s_diff < 0) {
        s_diff = im.s_max_ -  im.waypoints_.s[prevWaypoint];
        s_diff += im.waypoints_.s[nextWaypoint];
    }

    if (waypointDiff < 0) {
        waypointDiff += im.waypoints_.s.size();
    }

    double s_curr = state.s - im.waypoints_.s[prevWaypoint];
    double s_rate = s_diff / waypointDiff;

    double s_base = (s_curr / s_rate) + prevWaypoint;
    double s_end = (s_curr + 40) / s_rate + prevWaypoint;
    double s_path_rate = (s_end - s_base) / 100;

    for (int i = 1; i <= 100; i++) {
        Vector<2> v = im.spline_->getPosition(i * s_path_rate + s_base);
        double x = v[0];
        double y = v[1];
        retval.x.push_back(x);
        retval.y.push_back(y);
    }

    printf("Position: %f %f\n", state.x, state.y);
    printf("S: %f\n", state.s);
    printf("Closest Waypoint: %f %f\n", im.waypoints_.x[prevWaypoint], im.waypoints_.y[prevWaypoint]);
    printf("Next Waypoint: %f %f\n", im.waypoints_.x[nextWaypoint], im.waypoints_.y[nextWaypoint]);
    printf("Start: %f %f\n", retval.x[0], retval.y[0]);
    printf("End: %f %f\n\n", retval.x[retval.x.size() - 1], retval.y[retval.y.size() - 1]);

    return retval;
}

int PathGenerator::impl::find_waypoint_floor(double s) {
    int waypoint = 0;
    while (waypoint < waypoints_.s.size() -1 && waypoints_.s[waypoint+1] <= s) {
        waypoint++;
    }
    return waypoint;
}

PathGenerator::impl::~impl() {
    delete spline_;
}
