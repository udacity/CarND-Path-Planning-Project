#include "Vehicle.h"

Vehicle::Vehicle() {}

Vehicle::Vehicle(int lane, float s, float v, float a, std::string state="CS")
{
    this->lane = lane;
    this->s = s;
    this->v = s;
    this->a = a;
    this->state = state;
    maxAcceleration = -1;
}

Vehicle::~Vehicle();