#ifndef CONFIG_H_
#define CONFIG_H_

#define LANES  3
#define SLOTS 201 // Odd numbers only
#define SPEED_LIMIT 22.0 //meters per second
#define SLOT_LENGTH 2.0 
#define SLOT_RAD SLOT_LENGTH/2 - 0.00001

#define PREDICTOR_TIME_QUANT 0.5 // seconds
#define PREDICTOR_TIME_SLOTS 20

#endif
