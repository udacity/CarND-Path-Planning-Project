#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <vector>
#include "telemetry.h"
#include "map.h"
#include "trigs.h"

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

    BPosition next_position(vector<vector<double>> belief, unsigned int curr_lane){

      vector<vector<double>> dp(belief.size(), vector<double>(belief[0].size(), 0));
      dp[0] = belief[0];
      for(unsigned int i=1; i<dp.size(); i++){
        for(unsigned int j=0; j<3; j++){
          for(int k=-1; k<2; k++){
            int source_lane = j+k;
            if(source_lane>=0 && source_lane <3){
              dp[i][j] = Trigs::max(dp[i][j], dp[i-1][source_lane] + belief[i][j]);
            }
          }
        }
      }


      BPosition pos;
      pos.lane=curr_lane;
      for(int i=-1; i<2; i++){
        unsigned int target_lane = curr_lane + i;
        if(target_lane>=0 && target_lane<3){
          if(dp[9][pos.lane] < dp[9][target_lane]) {
            pos.lane = target_lane;
          }
        }
      }

      pos.speed = belief[10][pos.lane];
      return pos;
    }

  private:
    BehaviorPlanner() {} 
    ~BehaviorPlanner() {}

    BehaviorPlanner(BehaviorPlanner const&); 
    BehaviorPlanner& operator= (BehaviorPlanner const&); 
};
#endif
