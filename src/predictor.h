#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <vector>
#include "telemetry.h"
#include "map.h"
#include "belief.h"
#include "trigs.h"
#include "map.h"

#define LANES 3
#define SLOTS 41
#define SPEED_LIMIT 22.0 //meters per second
#define SLOT_RAD 4.99999


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
      vector<vector<char>> occupation_mx(SLOTS, vector<char>(LANES, '.'));

      vector<SensorFusion> sf = tl.sensor_fusion;
      for(unsigned int i=0; i<sf.size(); i++) {
        double diff = MapUtils::diff(tl.s, sf[i].s);
        double speed = sqrt(sf[i].vx*sf[i].vx + sf[i].vy*sf[i].vy);
        double new_diff = diff + speed;
        if(new_diff>-205 && new_diff < 205){
          int direction = (new_diff<0)?-1:0;
          int slot = 20 - (int) (new_diff+direction*SLOT_RAD) / 10; 
          int lane = (int) (sf[i].d / 4);
          if(lane>=0 && lane <4){
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
              if(!speed_mx[slot+2][lane].is_occupied){
                Slot sl_1;
                sl_1.is_occupied = false;
                sl_1.speed = Trigs::min(speed+2, SPEED_LIMIT);
                speed_mx[slot+2][lane] = sl_1;
              }
              if(!speed_mx[slot+3][lane].is_occupied){
                Slot sl_1;
                sl_1.is_occupied = false;
                sl_1.speed = Trigs::min(speed+4, SPEED_LIMIT);
                speed_mx[slot+3][lane] = sl_1;
              }
            }
            occupation_mx[slot][lane] = 'X';
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
