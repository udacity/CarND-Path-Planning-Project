#ifndef CONFIG_H_
#define CONFIG_H_

#define LANES 3 
#define SLOTS 401 // Odd numbers only
#define SPEED_LIMIT 22.0 //meters per second
#define SLOT_LENGTH 2.0 
#define SLOT_RAD SLOT_LENGTH/2 - 0.00001
#define MANEUVER_PENALTY 1

#define PREDICTOR_TIME_QUANT 0.5 // seconds
#define PREDICTOR_TIME_SLOTS 60

#endif
