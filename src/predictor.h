#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <vector>
#include "telemetry.h"
#include "map.h"
#include "belief.h"
#include "trigs.h"
#include "map.h"
#include "config.h"


using namespace std;
class Predictor {
  public:
    static Predictor& Instance(){
      static Predictor instance;
      return instance;
    }

    vector<vector<vector<Slot>>> update(Telemetry tl){
      vector<vector<vector<Slot>>> occupation_ts(PREDICTOR_TIME_SLOTS, vector<vector<Slot>>(SLOTS, vector<Slot>(LANES, create_empty_slot())));

      vector<SensorFusion> sf = tl.sensor_fusion;
      for(unsigned int i=0; i<sf.size(); i++) {
        double obs_speed = calc_speed(sf[i].vx, sf[i].vy);

        for(unsigned int t=0; t<PREDICTOR_TIME_SLOTS; t++){
          double diff = predict_diff(t, tl.s, tl.speed, sf[i].s, obs_speed);

          //Observable car is in visibility range
          if(diff>-1*SLOTS*SLOT_LENGTH/2 && diff < SLOTS*SLOT_LENGTH/2){
            int direction = (diff<0)?-1:0;
            int slot = (SLOTS-1)/2 - (int) (diff+direction*SLOT_RAD) / SLOT_LENGTH; 
            int lane = (int) (sf[i].d / 4);
            if(lane>=0 && lane <LANES){
              // Occupy more than one slot
              for(int ds=-6; ds<10; ds++){
                int occupied_slot = slot + ds;
                if(occupied_slot>=0 && occupied_slot<SLOTS){
                  occupation_ts[t][occupied_slot][lane] = create_occupied_slot(obs_speed - 5);
                }
              }
            }
          } 
        }
      }

     // cout<<"Keep speed"<<endl;
     // for(int t=0; t<PREDICTOR_TIME_SLOTS; t++) {
     //   char slot = occupation_ts[t][100][1].is_occupied?'X':'.';
     //   cout<<slot<<'|';
     // }
     // cout<<endl;

      return occupation_ts;
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
