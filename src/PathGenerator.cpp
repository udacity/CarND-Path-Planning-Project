//
// Created by Jose Rojas on 7/23/17.
//

#include "PathGenerator.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Geometry"

std::vector<double> JMT(std::vector< double> start, std::vector <double> end, double T)
{
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */

    Eigen::MatrixXd A(3, 3);
    Eigen::VectorXd b(3);

    double t2 = pow(T, 2);
    double t3 = pow(T, 3);
    double t4 = pow(T, 4);
    double t5 = pow(T, 5);

    A << t3, t4, t5,
            3 * t2, 4 * t3, 5 * t4,
            6 * T, 12 * t2, 20 * t3;

    b << end[0] - (start[0] + start[1] * T + 0.5 * start[2] * t2),
            end[1] - (start[1] + start[2] * T),
            end[2] - start[2];

    Eigen::MatrixXd Ainv = A.inverse();

    Eigen::VectorXd x = Ainv * b;

    return {start[0],start[1], 0.5 * start[2], x(0), x(1), x(2)};

}

std::chrono::milliseconds currentTime() {
    return std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );
}

double JMTeval(std::vector<double> params, double T) {
    double t2 = T * T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;
    return params[0] + params[1] * T + params[2] * t2 + params[3] * t3 + params[4] * t4 + params[5] * t5;
}

double JMTSpeedEval(std::vector<double> params, double T) {
    double t2 = T;
    double t3 = t2 * T;
    double t4 = t3 * T;
    double t5 = t4 * T;
    return params[1] + 2 * params[2] * t2 + 3 * params[3] * t3 + 4 * params[4] * t4 + 5 * params[5] * t5;
}

double JMTAccelEval(std::vector<double> params, double T) {
    double t3 = T;
    double t4 = t3 * T;
    double t5 = t4 * T;
    return 2 * params[2] + 6 * params[3] * t3 + 12 * params[4] * t4 + 20 * params[5] * t5;
}

double JMTJerkEval(std::vector<double> params, double T) {
    double t4 = T;
    double t5 = t4 * T;
    return 6 * params[3] + 24 * params[4] * t4 + 60 * params[5] * t5;
}

class PathGenerator::impl {
public:

    typedef struct {
        std::vector<double> JMTparams;
        double cost;
    } JMTCurve;

    impl();
    ~impl();
    int find_waypoint_floor(Waypoints waypoints, double s);
    double find_target_t(Waypoints waypoints, double s);
    double find_closest_time(double s);
    std::vector<double> generate_constraint_JMT(VehicleState s, double max_velocity, double max_accel, double max_jerk);

    Waypoints waypoints_;
    double s_max_;
    LoopingUniformCRSpline<Vector<2>>* spline_;
    LoopingUniformCRSpline<Vector<1>>* spline_d_;
    std::vector<double> JMTparams;
    double T;
    int maxItems;
    double last_s;

    double last_time;
};

PathGenerator::PathGenerator() : pimpl(new impl()) {

}

PathGenerator::~PathGenerator() = default;

PathGenerator::PathGenerator(Waypoints waypoints, double s_max) : PathGenerator() {

    impl& im = *pimpl;
    im.s_max_ = s_max;
    im.maxItems = 100;

    std::vector<Vector<2>> xv;
    std::vector<Vector<1>> sv;

    for (int i = 0; i < waypoints.s.size(); i++) {
        Vector<2> x;
        x[0] =(float) (waypoints.x[i] + waypoints.dx[i] * 6.0f);
        x[1] =(float) (waypoints.y[i] + waypoints.dy[i] * 6.0f);

        Vector<1> s;
        s[0] = (float) waypoints.s[i];

        xv.push_back(x);
        sv.push_back(s);
    }

    LoopingUniformCRSpline<Vector<2>> spline(xv);

    // the original waypoints are not uniformly distributed and our cubic hermite spline requires uniformly distributed
    // points - regenerate a set of waypoints with a uniform 's' distribution

    Waypoints newWaypoints;
    double uniformInterval = 10.f;
    double s = 0;
    Eigen::Rotation2D<double> rotNormal(-90.f * M_PI / 180.f);
    xv.clear();
    sv.clear();

    while (s < s_max) {

        //find t parameter that matches the target 's'
        double t = im.find_target_t(waypoints, s);
        UniformCRSpline<Vector<2>>::InterpolatedPT tangent = spline.getTangent(t);
        Vector<2> v = tangent.position;
        Vector<2> tx = tangent.tangent;
        double x = v[0];
        double y = v[1];

        //rotate tangent by -90 degrees to produce normal
        Eigen::Vector2d tv;
        tv << tx[0], tx[1];
        Eigen::Vector2d normal = rotNormal * tv;
        normal.normalize();

        newWaypoints.x.push_back(x);
        newWaypoints.y.push_back(x);
        newWaypoints.s.push_back(s);
        newWaypoints.dx.push_back(normal[0]);
        newWaypoints.dy.push_back(normal[1]);

        xv.push_back(v);
        Vector<1> sx;
        sx[0] = (float) s;
        sv.push_back(sx);

        s += uniformInterval;
    }

    im.spline_ = new LoopingUniformCRSpline<Vector<2>>(xv);
    im.spline_d_ = new LoopingUniformCRSpline<Vector<1>>(sv);
    im.waypoints_ = newWaypoints;
    im.last_time = 0;
    im.T = 5.f;

}

PathPoints PathGenerator::generate_path(VehicleState state) {

    impl& im = *pimpl;

    //generate 50 points along this path in one second
    PathPoints retval;
    //auto jmtParams = im.generate_constraint_JMT(state, 20, 5, 10);

    //int prevWaypoint = im.find_waypoint_floor(im.waypoints_, state.s);
    //int nextWaypoint = im.find_waypoint_floor(im.waypoints_, JMTeval(jmtParams, im.T)) + 1;

    int prevWaypoint = im.find_waypoint_floor(im.waypoints_, state.remaining_path_x.size() == 0 ? state.s : state.end_s);
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

    int remItems = im.maxItems - state.remaining_path_x.size();
    double s_start = state.remaining_path_x.size() == 0 ? state.s : state.end_s;
    double s_curr = s_start - im.waypoints_.s[prevWaypoint];
    double s_rate = s_diff / waypointDiff;

    double s_base = (s_curr / s_rate) + prevWaypoint;
    //double s_end = JMTeval(jmtParams, im.T) / s_rate + prevWaypoint;
    double s_end = (s_curr + remItems * 25 / im.maxItems) / s_rate + prevWaypoint;
    //double s_path_rate = (s_end - s_base) / remItems;

    double s_path_rate = 0.025f;
    // add previous points
    int i = 0;
    for (; i < state.remaining_path_x.size() && i < 10; i++) {
        retval.x.push_back(state.remaining_path_x[i]);
        retval.y.push_back(state.remaining_path_y[i]);
    }

    double base = im.last_time;
    base -= (state.remaining_path_x.size() - i) * s_path_rate;

    // determine optimal s positions along JMT
    int j = 0;
    for (j = 0; i < im.maxItems; i++, j++) {
        //double s_delta_eval = JMTeval(jmtParams, i * 0.02);
        //double s_curr_delta = s_delta_eval - im.waypoints_.s[prevWaypoint];
        //double s = (s_curr_delta / s_rate) + prevWaypoint;
        //Vector<2> v = im.spline_->getPosition(s);
        Vector<2> v = im.spline_->getPosition(base + j * s_path_rate);
        double x = v[0];
        double y = v[1];
        retval.x.push_back(x);
        retval.y.push_back(y);
    }

#if 0
    printf("Position: %f %f\n", state.x, state.y);
    //printf("Speed, Accel: %f %f\n", JMTSpeedEval(jmtParams, 0), JMTAccelEval(jmtParams, 0));
    printf("Stated Speed: %f\n", state.speed);
    printf("Closest Waypoint: %f %f\n", im.waypoints_.x[prevWaypoint], im.waypoints_.y[prevWaypoint]);
    printf("Next Waypoint: %f %f\n", im.waypoints_.x[nextWaypoint], im.waypoints_.y[nextWaypoint]);
    printf("Start: %f %f\n", retval.x[0], retval.y[0]);
    printf("End: %f %f\n\n", retval.x[retval.x.size() - 1], retval.y[retval.y.size() - 1]);
#endif

    im.last_time += remItems * s_path_rate;
    //im.JMTparams = jmtParams;

    return retval;
}

int PathGenerator::impl::find_waypoint_floor(Waypoints waypoints, double s) {
    int waypoint = 0;
    while (waypoint < waypoints.s.size() -1 && waypoints.s[waypoint+1] <= s) {
        waypoint++;
    }
    return waypoint;
}

double PathGenerator::impl::find_closest_time(double s) {
    double t = 0;
    while (t < T) {
        if (JMTeval(JMTparams, t + .02) > s) {
            break;
        }
        t += 0.02;
    }
    return t;
}

double PathGenerator::impl::find_target_t(Waypoints waypoints, double s) {
    int prevWaypoint = find_waypoint_floor(waypoints, s);
    int nextWaypoint = prevWaypoint + 1;

    if (nextWaypoint == prevWaypoint) {
        nextWaypoint++;
    }

    if (prevWaypoint < 0) {
        prevWaypoint += waypoints.s.size();
    }

    if (nextWaypoint >= waypoints.s.size()) {
        nextWaypoint -= waypoints.s.size();
    }

    int waypointDiff = nextWaypoint - prevWaypoint;
    double s_diff = waypoints.s[nextWaypoint] - waypoints.s[prevWaypoint];

    if (s_diff < 0) {
        s_diff = s_max_ -  waypoints.s[prevWaypoint];
        s_diff += waypoints.s[nextWaypoint];
    }

    if (waypointDiff < 0) {
        waypointDiff += waypoints.s.size();
    }

    double s_curr = s - waypoints.s[prevWaypoint];
    double s_rate = s_diff / waypointDiff;

    double s_base = (s_curr / s_rate) + prevWaypoint;

    return s_base;
}

std::vector<double> PathGenerator::impl::generate_constraint_JMT(VehicleState state, double max_velocity, double max_accel, double max_jerk) {

    double deltaT = maxItems - state.remaining_path_x.size() * 0.02;
    double speed =  JMTSpeedEval(JMTparams, deltaT);
    double accel = JMTAccelEval(JMTparams, deltaT);

    // determine JMT using start and end state
    JMTCurve bestCurve;
    bestCurve.cost = 0;

    for (double s_dot = max_velocity / 4.f; s_dot < max_velocity; s_dot += 0.5f) {
        for (double delta_s = s_dot / 2; delta_s < s_dot * T; delta_s += 0.5f) {

            std::vector<double> start {state.s, speed, accel};
            std::vector<double> end {state.s + delta_s, s_dot, 0};

            std::vector<double> params = JMT(start, end, T);

            //does it violate acceleration
            bool failedConstraint = false;
            for (double t = 0; t < T && !failedConstraint; t += 0.02f) {
                if (fabs(JMTAccelEval(params, t)) > max_accel) {
                    failedConstraint = true;
                    break;
                }
            }

            //does it violate speed
            for (double t = 0; t < T && !failedConstraint; t += 0.02f) {
                if (JMTSpeedEval(params, t) > max_velocity) {
                    failedConstraint = true;
                    break;
                }
            }

            //does it violate jerk
            for (double t = 0; t < T && !failedConstraint; t += 0.02f) {
                if (fabs(JMTJerkEval(params, t)) > max_jerk) {
                    failedConstraint = true;
                    break;
                }
            }

            if (!failedConstraint) {
                //calculate cost
                double cost = 0;

                if (cost <= bestCurve.cost) {
                    bestCurve = {params, cost};
                }
            }
        }
    }

    if (bestCurve.JMTparams.size() == 0) {
        throw new std::exception();
    }

    return bestCurve.JMTparams;
}

PathGenerator::impl::impl() : JMTparams(6) {

}


PathGenerator::impl::~impl() {
    delete spline_;
}
