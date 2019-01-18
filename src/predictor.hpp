#ifndef PREDICTOR_HPP_
#define PREDICTOR_HPP_

#include <vector>
#include "telemetry.hpp"
#include "belief.h"
#include "trigs.hpp"
#include "map.hpp"
#include "config.h"


using namespace std;
class Predictor {
  public:
    static Predictor& Instance(){
      static Predictor instance;
      return instance;
    }

    vector<vector<vector<Slot>>> update(Telemetry tl){
      vector<vector<vector<Slot>>> belief(PREDICTOR_TIME_SLOTS, vector<vector<Slot>>(3, vector<Slot>(LANES, create_empty_slot())));

      vector<SensorFusion> sf = tl.sensor_fusion;
      for(unsigned int i=0; i<sf.size(); i++) {
        double obs_speed = calc_speed(sf[i].vx, sf[i].vy);

        for(unsigned int t=0; t<PREDICTOR_TIME_SLOTS; t++){
          double diff = predict_diff(t, tl.s, tl.speed, sf[i].s, obs_speed);

          //Observable car is in visibility range
          if(diff>-SLOTS && diff < SLOTS){
            int direction = (diff<0)?-1:0;
            int slot = (SLOTS-1)/2 - (int) (diff+direction*SLOT_RAD) / SLOT_LENGTH; 
            int lane = (int) (sf[i].d / 4);
            if(lane>=0 && lane <LANES){
              // Occupy more than one slot
              for(int ds=-2; ds<4; ds++){
                int occupied_slot = slot + ds;
                if(is_slot_in_belief(occupied_slot)){
                  belief[t][to_belief_slot(occupied_slot)][lane] = create_occupied_slot(Trigs::max(obs_speed - 5, 0));
                }
              }

              // Create slow slots
              for(int ds=4; ds<10; ds++){
                int target_slot_idx = slot+ds;
                if(is_slot_in_belief(target_slot_idx)){
                  Slot target_slot = belief[t][to_belief_slot(target_slot_idx)][lane];
                  if(!target_slot.is_occupied) {
                    double target_speed = obs_speed - 5 + (ds-4)*2;
                    belief[t][to_belief_slot(target_slot_idx)][lane] = create_unoccupied_slot(Trigs::min(obs_speed, target_speed));
                  }
                }
              }
            }
          } 
        }
      }


      return belief;
    }


  private:
    Predictor() {} 
    ~Predictor() {}

    Predictor(Predictor const&); 
    Predictor& operator= (Predictor const&); 

    double predict_diff(unsigned int time_slot, double ego_s, double ego_speed, double obs_s, double obs_speed){
      double time = time_slot * PREDICTOR_TIME_QUANT;
      double ego_pos_s = ego_s + ego_speed * time;
      double obs_pos_s = obs_s + obs_speed * time;
      double diff = MapUtils::diff(ego_pos_s, obs_pos_s);
      return  diff;
    }

    double calc_speed(double vx, double vy){
      return sqrt(vx*vx + vy*vy);
    }

    bool is_slot_in_belief(int slot_num){
      //return slot_num >= 0 && slot_num < SLOTS;
      return slot_num > 198 && slot_num < 202;
    }

    int to_belief_slot(int slot_num) {
      return slot_num -199;
    }

    Slot create_empty_slot(){
      Slot empty_slot;
      empty_slot.speed = SPEED_LIMIT;
      empty_slot.is_occupied = false;
      return empty_slot;
    }

    Slot create_occupied_slot(double speed){
      Slot result = create_unoccupied_slot(speed);
      result.is_occupied = true;
      return result;
    }

    Slot create_unoccupied_slot(double speed){
      Slot result;
      result.is_occupied = false;
      result.speed = speed;
      return result;
    }
};
#endif
