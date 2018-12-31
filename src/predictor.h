#ifndef PREDICTOR_H_
#define PREDICTOR_H_

#include <vector>
#include "telemetry.h"
#include "map.h"
#include "belief.h"
#include "trigs.h"

#define LANES 3
#define SLOTS 21
#define SPEED_LIMIT 22.0 //meters per second
#define SLOT_RAD 4.99999


using namespace std;
class Predictor {
  public:
    static Predictor& Instance(){
      static Predictor instance;
      return instance;
    }

    vector<vector<double>> update(Telemetry tl){
      vector<vector<double>> speed_mx(SLOTS, vector<double>(LANES, SPEED_LIMIT));
      vector<vector<char>> occupation_mx(SLOTS, vector<char>(LANES, '.'));

      vector<SensorFusion> sf = tl.sensor_fusion;
      for(unsigned int i=0; i<sf.size(); i++) {
        double diff = sf[i].s - tl.s;
        double speed = sqrt(sf[i].vx*sf[i].vx + sf[i].vy*sf[i].vy);
        if(diff>-105 && diff < 105){
          int direction = (diff<0)?-1:0;
          int slot = 10 - (int) (diff+direction*SLOT_RAD) / 10; 
          int lane = (int) (sf[i].d / 4);
          if(lane>=0 && lane <4){
            speed_mx[slot][lane] = speed-2;
            if(slot <=10){
              speed_mx[slot+1][lane] = Trigs::min(speed, speed_mx[slot+1][lane], SPEED_LIMIT);
              speed_mx[slot+2][lane] = Trigs::min(speed+5, speed_mx[slot+2][lane], SPEED_LIMIT);
            }
            occupation_mx[slot][lane] = 'X';
          }
        } 
      }

      cout<<"-------"<<endl;
      for(unsigned int i =0; i<SLOTS; i++){
        for(unsigned int j =0; j<LANES; j++){
          cout << "|"<<speed_mx[i][j];
        }
        cout<<"|"<<endl;
      }
      return speed_mx;
    }


  private:
    Predictor() {} 
    ~Predictor() {}

    Predictor(Predictor const&); 
    Predictor& operator= (Predictor const&); 
};
#endif
