#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_
//TODO: use from predictor
#define SLOT_RAD_1 4.99999

#include <vector>
#include "telemetry.h"
#include "map.h"
#include "trigs.h"
#include "belief.h"

using namespace std;

struct BPosition{
  unsigned int lane;
  double speed;
};

class BehaviorPlanner {
  public:
    static BehaviorPlanner& Instance(){
      static BehaviorPlanner instance;
      return instance;
    }

    BPosition next_position(vector<vector<Slot>> belief, Telemetry tl){
      unsigned int curr_lane = (int)tl.d/4;
      unsigned int next_row = 20-(int)(tl.speed + SLOT_RAD_1)/10;

      vector<vector<double>> dp(belief.size(), vector<double>(belief[0].size(), 0));

      for(unsigned int j=0; j<3; j++){
        dp[0][j] = belief[0][j].speed;
      }

      for(unsigned int i=1; i<dp.size(); i++){
        for(unsigned int j=0; j<3; j++){
          for(int k=-1; k<2; k++){
            int source_lane = j+k;
            if(source_lane>=0 && source_lane <3){
              if( k!=0 && 
                  !belief[i][source_lane].is_occupied && 
                  !belief[i-1][source_lane].is_occupied &&
                  !belief[i][j].is_occupied &&
                  !belief[i-1][j].is_occupied){
                dp[i][j] = Trigs::max(dp[i][j], dp[i-1][source_lane] + belief[i][j].speed);
              } else if(k==0){
                dp[i][j] = Trigs::max(dp[i][j], dp[i-1][source_lane] + belief[i][j].speed);
              }
            }
          }
        }
      }


      BPosition pos;

      pos.lane=curr_lane;
      for(int i=-1; i<2; i++){
        unsigned int target_lane = curr_lane + i;
        if(target_lane>=0 && target_lane<3){
          cout << dp[next_row][target_lane]<< " | ";
          if(i!=0){
            if (
                belief[next_row][target_lane].is_occupied || 
                belief[next_row+1][target_lane].is_occupied){
              continue;
            }
          }
          if(dp[next_row][pos.lane] < dp[next_row][target_lane]-0.5) {
            pos.lane = target_lane;
          }
        }
      }
      cout << endl;
      //pos.lane = 1;

      pos.speed = belief[next_row][pos.lane].speed;
      return pos;
    }

  private:
    BehaviorPlanner() {} 
    ~BehaviorPlanner() {}

    BehaviorPlanner(BehaviorPlanner const&); 
    BehaviorPlanner& operator= (BehaviorPlanner const&); 
};
#endif
