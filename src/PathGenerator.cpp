//
// Created by Jose Rojas on 7/23/17.
//

#include "PathGenerator.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU"
#include "Eigen-3.3/Eigen/Geometry"

typedef struct {
    int id;
    int lane;
    double start;
    double end;
} LaneBoundingBox;

typedef struct {
    std::vector<double> JMTparams;
    double cost;
} JMTCurve;

typedef std::vector<std::vector<LaneBoundingBox>> VecLaneBoundingBox;

static Eigen::Rotation2D<double> g_rotNormal(-90.f * M_PI / 180.f);

extern std::vector<double> getFrenet(double x, double y, double theta, std::vector<double> maps_x, std::vector<double> maps_y);

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

Eigen::Vector2d getNormalFromTangent(UniformCRSpline<Vector<2>>::InterpolatedPT& tangent, double t) {
    Vector<2> v = tangent.position;
    Vector<2> tx = tangent.tangent;
    double x = v[0];
    double y = v[1];

    //rotate tangent by -90 degrees to produce normal
    Eigen::Vector2d tv;
    tv << tx[0], tx[1];
    Eigen::Vector2d normal = g_rotNormal * tv;
    normal.normalize();

    return normal;
}

Eigen::Vector2d getNormalFromSpline(LoopingUniformCRSpline<Vector<2>>* spline, double t) {
    UniformCRSpline<Vector<2>>::InterpolatedPT tangent = spline->getTangent(t);
    return getNormalFromTangent(tangent, t);
}

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

JMTCurve permuteJMT(double pos, double speed, double accel, double max_s, double max_velocity, double max_accel, double max_jerk, double dT, double dt) {
    JMTCurve bestCurve;
    bestCurve.cost = 0;

    for (double s_dot = speed / 4.f; s_dot < max_velocity; s_dot += 0.5f) {
        for (double delta_s = s_dot * dt; pos + delta_s <= max_s; delta_s += 1.f) {

            std::vector<double> start {pos, speed, accel};
            std::vector<double> end {pos + delta_s, s_dot, 0};

            std::vector<double> params = JMT(start, end, dT);

            //does it violate acceleration
            bool failedConstraint = false;
            for (double t = 0; t < dT && !failedConstraint; t += dt) {
                if (fabs(JMTAccelEval(params, t)) > max_accel) {
                    failedConstraint = true;
                    break;
                }
            }

            //does it violate speed
            for (double t = 0; t < dT && !failedConstraint; t += dt) {
                if (JMTSpeedEval(params, t) > max_velocity) {
                    failedConstraint = true;
                    break;
                }
            }

            //does it violate jerk
            for (double t = 0; t < dT && !failedConstraint; t += dt) {
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

    return bestCurve;
}

std::vector<LaneBoundingBox> calculateLaneBoundingBoxForSensorVehicleState(SensorVehicleState state, double dt, double laneWidth) {
    //XXX assume cars do not switch lanes for now

    double direction = state.yaw > M_PI/4 ? 1.f : -1.f;
    double s1 = state.s;
    double s2 = state.s + (state.speed * dt) * direction;
    int lane = (int) (state.d / laneWidth);

    return {{state.id,lane,s1,s2}};
}

std::vector<LaneBoundingBox> calculateLaneBoundingBox(double d, double s, double speed, double dt, double laneWidth) {
    double s1 = s;
    double s2 = s + (speed * dt);
    int lane = (int) (d / laneWidth);
    return {{0,lane,s1,s2}};
}

bool does_collide(std::vector<LaneBoundingBox>& boxes1, std::vector<LaneBoundingBox>& boxes2) {
    for (LaneBoundingBox& box1 : boxes1) {
        for (LaneBoundingBox& box2 : boxes2) {
            return /*(box1.start >= box2.start && box1.start <= box2.end) || (box1.end >= box2.start && box1.end <= box2.end)||*/
                    (box2.start >= box1.start && box2.start <= box1.end) || (box2.end >= box1.start && box2.end <= box1.end);
        }
    }
    return false;
}

std::chrono::milliseconds currentTime() {
    return std::chrono::duration_cast< std::chrono::milliseconds >(
            std::chrono::system_clock::now().time_since_epoch()
    );
}


class PathGenerator::impl {
public:

    impl();
    ~impl();
    int find_waypoint_floor(Waypoints waypoints, double s);
    double find_target_t(Waypoints waypoints, double s);
    double find_closest_time(double s);
    std::vector<double> generate_constraint_JMT(VehicleState state, double max_velocity, double max_accel, double max_jerk);
    VecLaneBoundingBox check_collisions(const VehicleState& state, const std::vector<double>& params, double dt);

    Waypoints waypoints_;
    double s_max_;
    LoopingUniformCRSpline<Vector<2>>* spline_;
    std::vector<double> JMTparams;
    double dT;
    int maxItems;
    double dt;
    int prevItems;
    double maxToKeep;
    double timeToNextJMT;
};

PathGenerator::PathGenerator() : pimpl(new impl()) {

}

PathGenerator::~PathGenerator() = default;

PathGenerator::PathGenerator(Waypoints waypoints, double s_max) : PathGenerator() {

    impl& im = *pimpl;
    im.s_max_ = s_max;
    im.maxItems = 150;
    im.timeToNextJMT = 0;
    im.prevItems = 0;
    im.maxToKeep = 40;

    std::vector<Vector<2>> xv;
    std::vector<Vector<1>> sv;

    for (int i = 0; i < waypoints.s.size(); i++) {
        Vector<2> x;
        x[0] =(float) (waypoints.x[i]);
        x[1] =(float) (waypoints.y[i]);

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
    xv.clear();
    sv.clear();

    while (s < s_max) {

        //find t parameter that matches the target 's'
        double t = im.find_target_t(waypoints, s);
        UniformCRSpline<Vector<2>>::InterpolatedPT tangent = spline.getTangent(t);
        Eigen::Vector2d normal = getNormalFromTangent(tangent, t);

        Vector<2> v = tangent.position;
        double x = v[0];
        double y = v[1];
        newWaypoints.x.push_back(x);
        newWaypoints.y.push_back(y);
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
    im.waypoints_ = newWaypoints;
    im.dT = 3.f;
    im.dt = 0.02;

}

PathPoints PathGenerator::generate_path(VehicleState state) {

    impl& im = *pimpl;

    double usedT = im.find_closest_time(state.s);
    int used = (int) fmax(0, im.prevItems - (int) state.remaining_path_x.size());
    unsigned long toKeep = (unsigned long) fmin(im.maxToKeep, state.remaining_path_x.size());

    //reduce time
    im.timeToNextJMT -= used * im.dt;
    im.prevItems = (int) state.remaining_path_x.size();

    bool bSkip = im.timeToNextJMT > 0;

    // add previous points
    PathPoints retval;
    int i = 0;
    for (; i < state.remaining_path_x.size() && (bSkip || i < im.maxToKeep); i++) {
        retval.x.push_back(state.remaining_path_x[i]);
        retval.y.push_back(state.remaining_path_y[i]);
    }

    if (bSkip) {
        printf("skipped - time to cover %f\n", im.timeToNextJMT);
        return retval;
    }

    //generate points along this path in one second
    auto jmtParams = im.generate_constraint_JMT(state, 20, 10, 10);

    double remT = im.dT - toKeep * im.dt;
    double s_start = JMTeval(jmtParams, 0);
    double s_end = JMTeval(jmtParams, remT);
    int prevWaypoint = im.find_waypoint_floor(im.waypoints_, s_start);
    int nextWaypoint = im.find_waypoint_floor(im.waypoints_, s_end) + 1;

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

    double s_rate = s_diff / waypointDiff;

    double diff = 0, normalDiff;

    // determine optimal s positions along JMT
    int j = 0, lastKeep = i;
    for (j = 0; i < im.maxItems; i++, j++) {
        double s_delta_eval = JMTeval(jmtParams, j * im.dt);
        if (s_delta_eval > s_end + 0.0001) {
            throw new std::exception();
        }
        double s_curr_delta = s_delta_eval - im.waypoints_.s[prevWaypoint];
        double s = (s_curr_delta / s_rate) + prevWaypoint;
        Vector<2> v = im.spline_->getPosition(s);
        Eigen::Vector2d normal = getNormalFromSpline(im.spline_, s);
        double x = v[0] + normal[0] * 6.0f;
        double y = v[1] + normal[1] * 6.0f;
        retval.x.push_back(x);
        retval.y.push_back(y);
    }

    if (lastKeep > 1) {
        diff = sqrt(pow(retval.x[lastKeep] - retval.x[lastKeep - 1], 2) +
                    pow(retval.y[lastKeep] - retval.y[lastKeep - 1], 2));
        normalDiff = sqrt(pow(retval.x[lastKeep - 1] - retval.x[lastKeep - 2], 2) +
                          pow(retval.y[lastKeep - 1] - retval.y[lastKeep - 2], 2));
    }

    printf("Used: %ld\n", used);
    printf("S: %f, Estimated S: %f, diff: %f\n", state.s, JMTeval(im.JMTparams, usedT), state.s - JMTeval(im.JMTparams, usedT));
    printf("Diff between new/old JMT: %f, normal diff: %f\n", diff, normalDiff);

#if 0
    printf("Position: %f %f\n", state.x, state.y);
    //printf("Speed, Accel: %f %f\n", JMTSpeedEval(jmtParams, 0), JMTAccelEval(jmtParams, 0));
    printf("Stated Speed: %f\n", state.speed);
    printf("Closest Waypoint: %f %f\n", im.waypoints_.x[prevWaypoint], im.waypoints_.y[prevWaypoint]);
    printf("Next Waypoint: %f %f\n", im.waypoints_.x[nextWaypoint], im.waypoints_.y[nextWaypoint]);
    printf("Start: %f %f\n", retval.x[0], retval.y[0]);
    printf("End: %f %f\n\n", retval.x[retval.x.size() - 1], retval.y[retval.y.size() - 1]);
#endif

    im.timeToNextJMT = toKeep * im.dt;
    im.JMTparams = jmtParams;
    im.prevItems = im.maxItems;

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
    while (t < dT) {
        if (JMTeval(JMTparams, t + dt) > s) {
            break;
        }
        t += dt;
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

    // number of time steps that have been consumed
    // number of time steps that remain that should be kept
    unsigned long toKeep = (unsigned long) fmin(maxToKeep, state.remaining_path_x.size());
    // elapsed time to the new 'reference point', the origin of the new JMT for this update
    double deltaT = - timeToNextJMT + toKeep * dt;

    //use the previous JMT and final reference point to determine where the
    double pos =  state.remaining_path_x.size() == 0 ? state.s : JMTeval(JMTparams, deltaT);
    double speed =  JMTSpeedEval(JMTparams, deltaT);
    double accel = JMTAccelEval(JMTparams, deltaT);

    // determine JMT using start and end state

    // amount of time necessary to generate a full prediction horizon
    double remT = dT - toKeep * dt;
    if (!(remT > toKeep * dt)) {
        throw new std::exception();
    }

    double furthest_s = pos + max_velocity * remT;
    auto bestCurve = permuteJMT(pos, speed, accel, furthest_s, max_velocity, max_accel, max_jerk, remT, dt);

    if (bestCurve.JMTparams.size() == 0) {
        throw new std::exception();
    }

    //check for collisions
    auto collisions = check_collisions(state, bestCurve.JMTparams, remT);
    if (!collisions.empty()) {
        //there's a collision if we remain in this lane at the given final speed; slow down
        //find closest start distance
        double closest_s = furthest_s;
        for (auto boundingBoxes : collisions) {
            for (auto boundingBox : boundingBoxes) {
                if (boundingBox.lane == 1 && closest_s > boundingBox.start && boundingBox.start > pos) {
                    closest_s = boundingBox.start;
                }
            }
        }

        do {
            if (closest_s < pos + max_velocity * remT) {
                //determine new max position
                bestCurve = permuteJMT(pos, speed, accel, closest_s, max_velocity, max_accel, max_jerk, remT, dt);
            }
            closest_s += 0.5;
        } while (bestCurve.JMTparams.size() == 0 && max_velocity > 0);
    }

    if (bestCurve.JMTparams.size() == 0) {
        throw new std::exception();
    }

    return bestCurve.JMTparams;
}

VecLaneBoundingBox PathGenerator::impl::check_collisions(const VehicleState& state, const std::vector<double>& params, double dt) {
    VecLaneBoundingBox retval;
    std::vector<LaneBoundingBox> boundingBoxesCar = calculateLaneBoundingBox(state.d, JMTeval(params, 0), JMTSpeedEval(params, 0), dt, 4.0);
    for (SensorVehicleState s : state.sensor_state) {
        std::vector<LaneBoundingBox> boundingBoxes = calculateLaneBoundingBoxForSensorVehicleState(s, dt, 4.f);

        bool collide = does_collide(boundingBoxesCar, boundingBoxes);
        if (collide) {
            retval.push_back(boundingBoxes);
        }
    }
    return retval;
}

PathGenerator::impl::impl() : JMTparams(6) {

}


PathGenerator::impl::~impl() {
    delete spline_;
}
