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

    vector<vector<Slot>> update(Telemetry tl){
      Slot empty_slot;
      empty_slot.speed = SPEED_LIMIT;
      empty_slot.is_occupied = false;
      vector<vector<Slot>> speed_mx(SLOTS, vector<Slot>(LANES, empty_slot));

      vector<SensorFusion> sf = tl.sensor_fusion;
      for(unsigned int i=0; i<sf.size(); i++) {
        double diff = MapUtils::diff(tl.s+tl.speed*0.5, sf[i].s);
        double speed = sqrt(sf[i].vx*sf[i].vx + sf[i].vy*sf[i].vy);
        double new_diff = diff + speed*0.5;
        if(new_diff>-1*SLOTS*SLOT_LENGTH/2 && new_diff < SLOTS*SLOT_LENGTH/2){
          int direction = (new_diff<0)?-1:0;
          int slot = 20 - (int) (new_diff+direction*SLOT_RAD) / SLOT_LENGTH; 
          int lane = (int) (sf[i].d / 4);
          if(lane>=0 && lane <3){
            Slot sl_0;
            sl_0.is_occupied = true;
            sl_0.speed = speed - 2;
            speed_mx[slot][lane] = sl_0;
            if(slot <=20){
              if(!speed_mx[slot+1][lane].is_occupied){
                Slot sl_1;
                sl_1.is_occupied = false;
                sl_1.speed = Trigs::min(speed, SPEED_LIMIT);
                speed_mx[slot+1][lane] = sl_1;
              }
            }
          }
        } 
      }

    //  cout<<"-------"<<endl;
    //  for(unsigned int i =0; i<SLOTS; i++){
    //    for(unsigned int j =0; j<LANES; j++){
    //      cout << "|"<<speed_mx[i][j].speed;
    //    }
    //    cout<<"|"<<endl;
    //  }
      return speed_mx;
    }


  private:
    Predictor() {} 
    ~Predictor() {}

    Predictor(Predictor const&); 
    Predictor& operator= (Predictor const&); 
};
#endif
