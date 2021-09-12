#include <cstdio>
#include <iostream>

#include "helpers.h"
#include "spline.h"

void calcNextPoints(points &nextPoints, const egoVehicle &car,
                    const tk::spline &spline, const poseXY &reference) {
  // define the actual (x,y) points we will use for the planer
  // start with all of the previous path points from last time
  for (int i = 0; i < car.previous_path_x.size(); i++) {
    nextPoints.x.push_back(car.previous_path_x[i]);
    nextPoints.y.push_back(car.previous_path_y[i]);
  }

  // Fill up the rest of our path planner after filling it with previous
  // points, here we will always output 50 points
  const int numberOfOutputPoints = 50;
  double x_add_on = 0;
  double distancePerCycle = getTravelledDistance(controlSpeed);
  double delta = distancePerCycle / 1000;
  for (int i = 1; i <= numberOfOutputPoints - car.previous_path_x.size(); i++) {
    double target_dist = 0.0;
    while (target_dist < distancePerCycle) {
      double x1 = x_add_on;
      double y1 = spline(x1);
      x_add_on += delta;
      double y2 = spline(x_add_on);
      target_dist += distance(x1, y1, x_add_on, y2);
    }
    x_add_on -= delta;

    pointXY newPoint;
    newPoint.x = x_add_on;
    newPoint.y = spline(newPoint.x);

    // rotate back to normal after rotating it earlier
    auto newPointTrans = calcRotation(newPoint, reference.yaw);
    newPointTrans = calcTranslation(newPointTrans, reference.xy);

    nextPoints.x.push_back(newPointTrans.x);
    nextPoints.y.push_back(newPointTrans.y);
  }
}

tk::spline calcSpline(poseXY &reference, const egoVehicle &car,
                      const mapWaypoints &map) {
  // reference x,y,yaw state
  // either we will reference the starting point as where the car is or at the
  // previous paths end point
  reference.xy = car.xy;
  reference.yaw = deg2rad(car.yaw);

  // if previous size is almost empty, use the car as starting reference
  pointXY previousPoint;
  if (car.previous_path_x.size() < 2) {
    // use two points that make the path tangent to the car
    previousPoint = calcPreviousPoint(reference.xy, car.yaw);
  }
  // use the previous path's end point end point as starting reference
  else {
    // Redefine reference state as previous path end point
    reference.xy.x = car.previous_path_x[car.previous_path_x.size() - 1];
    reference.xy.y = car.previous_path_y[car.previous_path_x.size() - 1];

    previousPoint.x = car.previous_path_x[car.previous_path_x.size() - 2];
    previousPoint.y = car.previous_path_y[car.previous_path_x.size() - 2];
    reference.yaw = calcYaw(reference.xy, previousPoint);
  }

  // Use two points that make the path tangent to the previous path's end
  // point
  path anchorPoints;
  anchorPoints.xy.push_back(previousPoint);
  anchorPoints.xy.push_back(reference.xy);

  // In frenet add evenly 30m spaced  points ahead of the starting reference
  double startS = 30;
  double stepS = 30;
  double endS = 90;
  for (int offsetLong = startS; offsetLong <= endS; offsetLong += stepS) {
    auto next_wp =
        getXY(car.sd.s + offsetLong, controlOffsetLat, map.s, map.x, map.y);
    anchorPoints.xy.push_back(next_wp);
  }

  vector<double> x;
  vector<double> y;
  for (int i = 0; i < anchorPoints.xy.size(); i++) {
    // shift car reference angle to 0 degrees
    auto shift = calcTranslation(anchorPoints.xy[i], reference.xy, true);
    anchorPoints.xy[i] = calcRotation(shift, 0 - reference.yaw);
    x.push_back(anchorPoints.xy[i].x);
    y.push_back(anchorPoints.xy[i].y);
  }

  // create a spline
  tk::spline spline(x, y);

  return spline;
}

void control(const egoVehicle &car) {
  // make the control dynamic
  double factor = 1;
  if (targetSpeed > car.speed) {
    if (car.speed != 0) {
      factor = targetSpeed / car.speed;
    }
  } else {
    factor = car.speed / targetSpeed;
  }
  factor = std::max(factor, 1.0);
  factor = std::min(factor, 5.0);

  // rudimentary controlling of velocity
  if (controlSpeed < targetSpeed) {
    controlSpeed += velocityStep * factor;
  } else {
    controlSpeed -= velocityStep * factor;
  }

  // rudimentary control of target lateral offset
  if (controlOffsetLat < targetOffsetLat) {
    controlOffsetLat += offsetLatStep;
  } else {
    controlOffsetLat -= offsetLatStep;
  }
}

void calcLane(egoVehicle &car, vector<vector<double>> sensor_fusion) {
  // prepare objects
  vector<object> objList;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    object obj(sensor_fusion[i]);
    // calc velocity in miles per hour
    obj.v = distance(0, 0, obj.vx, obj.vy) * factorMilesPhToMperS;
    obj.currentlaneIndex = static_cast<laneIndex>(floor(obj.sd.d / laneWidth));
    // predict obj to future
    obj.predS = obj.sd.s + getTravelledDistance(obj.v, laneChangeDuration);

    // check if obj is within relevant distance
    bool isCurrentPosCritical =
        ((obj.sd.s > (car.sd.s - bufferDistanceBehindEgo)) &&
         (obj.sd.s < (car.sd.s + bufferDistanceBehindEgo)));
    bool isPredPosCritical =
        ((obj.predS > car.predS - bufferDistanceBehindEgo) &&
         (obj.predS < car.predS + bufferDistanceBehindEgo));
    bool isPathCritical = ((obj.sd.s > car.sd.s - bufferDistanceBehindEgo) &&
                           (obj.predS < car.predS + bufferDistanceBehindEgo));
    if (car.currentlaneIndex == obj.currentlaneIndex) {
      if (isPathCritical) {
        objList.push_back(obj);
      }
    } else {
      if (isCurrentPosCritical || isPredPosCritical) {
        objList.push_back(obj);
      }
    }

    // std::cout << obj.id << ";" << obj.sd.s << ";" << obj.sd.d << ";" << obj.v
    //           << ";";
  }

  // calc max speed per lane
  lane lanes[3];
  for (auto &obj : objList) {
    if (lanes[obj.currentlaneIndex].maxV > obj.v) {
      lanes[obj.currentlaneIndex].maxV = obj.v - decreaseObjectVelocity;
    }
  }

  std::cout << std::fixed << std::setprecision(1);
  for (auto &lane : lanes) {
    std::cout << lane.maxV << ";";
  }

  // determine which other lanes are available
  vector<laneIndex> possibleLanes;
  bool isLaneChangeDone = car.currentlaneIndex == targetLaneIndex;
  bool isLaneBlocked = lanes[car.currentlaneIndex].maxV < maxVelocity;
  if ((isLaneChangeDone) && (isLaneBlocked)) {
    switch (car.currentlaneIndex) {
      case left:
        possibleLanes.push_back(middle);
        break;
      case right:
        possibleLanes.push_back(middle);
        break;
      case middle:
        possibleLanes.push_back(left);
        possibleLanes.push_back(right);
        break;
      default:
        break;
    }
  }

  // check if new lane is better
  for (auto &possibilities : possibleLanes) {
    bool isNewLaneFree = lanes[possibilities].maxV == maxVelocity;
    bool isNewLaneFaster =
        lanes[targetLaneIndex].maxV < lanes[possibilities].maxV;
    if (isNewLaneFree && isNewLaneFaster) {
      targetLaneIndex = possibilities;
    }
  }

  // calc control values
  targetSpeed = lanes[car.currentlaneIndex].maxV;
  targetOffsetLat = getLaneDisplacement(targetLaneIndex);
}

void calc(points &nextPoints, egoVehicle &car, const mapWaypoints &map,
          vector<vector<double>> sensor_fusion) {
  // choose best lane
  calcLane(car, sensor_fusion);

  // rudimentary controlling long and lat
  control(car);

  // calc trajectory for best lane
  poseXY reference;
  auto spline = calcSpline(reference, car, map);

  // generate points out of trajectory
  calcNextPoints(nextPoints, car, spline, reference);

  // Output debug variables
  std::cout << controlSpeed << ";" << car.speed << ";" << targetSpeed << ";"
            << car.sd.s << ";" << car.sd.d << ";" << car.xy.x << ";" << car.xy.y
            << car.currentlaneIndex << ";" << targetLaneIndex << ";" << car.yaw;
  std::cout << std::endl;
}