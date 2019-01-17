#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <iostream>
#include <vector>
#include "telemetry.h"
#include "map.h"
#include "trigs.h"
#include "belief.h"
#include "config.h"

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

    BPosition next_position(vector<vector<vector<Slot>>> belief, Telemetry tl){
      unsigned int curr_lane = (int)tl.d/4;
      unsigned int BEHAVIOR_SLOTS = 3;//SLOTS;

      vector<vector<vector<double>>> dp(PREDICTOR_TIME_SLOTS, vector<vector<double>>(BEHAVIOR_SLOTS, vector<double>(LANES, 0.0)));

      //Initialize last layer
      unsigned int last_layer = PREDICTOR_TIME_SLOTS - 1; 
      for(unsigned int i = 0; i < BEHAVIOR_SLOTS; i++){
        for(unsigned int j = 0; j < LANES; j++){
          if(belief[last_layer][i][j].is_occupied){
            dp[last_layer][i][j] = 0;
          }else {
            dp[last_layer][i][j] = belief[last_layer][i][j].speed;
          }
        }
      }

      for(int t=PREDICTOR_TIME_SLOTS-2; t>=0; t--){
        for(unsigned int s=0; s< BEHAVIOR_SLOTS; s++){
          for(unsigned int lane=0; lane<LANES; lane ++){
            if(belief[t][s][lane].is_occupied){
              dp[t][s][lane] = 0;
              continue;
            }

            // Update cost
            dp[t][s][lane] = dp[t+1][s][lane] + belief[t][s][lane].speed;

            // Check maneuver collision
            for(int dl=0; dl<2; dl++){
              int target_lane = lane+maneuver_priority[dl];
              if(is_valid_lane(target_lane)){
                if(!has_collision(belief, target_lane, s, t)){
                  double candidate = dp[t+1][s][target_lane] + belief[t][s][lane].speed -MANEUVER_PENALTY;
                  dp[t][s][lane] = Trigs::max(dp[t][s][lane], candidate);
                }
              }
            }

          }
        }
      }

      double lane_costs[LANES] = {0,0,0};
      for(int l = 0; l< LANES; l++) {
        lane_costs[l] = dp[6][1][l];

        //Check collision
        if(has_collision(belief, l, 1, 0)) {
          lane_costs[l] = 0.0;
        }
      }

      BPosition pos;
      pos.lane = curr_lane;
      double max_lane_cost =lane_costs[curr_lane];
      for(unsigned int dl=0; dl<3; dl++) {
        int target_lane = curr_lane + maneuver_priority[dl];
        if(is_valid_lane(target_lane)) {
          if(lane_costs[target_lane] > max_lane_cost){
            pos.lane = target_lane;
            max_lane_cost = lane_costs[target_lane];
          }
        }
      }

      pos.speed = target_speed(tl.d, pos.lane, belief[1][1][curr_lane].speed, belief[1][1][pos.lane].speed);

      return pos;
    }

  private:
    BehaviorPlanner() {} 
    ~BehaviorPlanner() {}

    BehaviorPlanner(BehaviorPlanner const&); 
    BehaviorPlanner& operator= (BehaviorPlanner const&); 

    int acceleration[3] = { -1, 0, 1};
    int maneuver_priority[3] = {-1, 1};

    bool is_valid_lane(int lane) {
      return lane>=0 && lane<LANES;
    }

    bool has_collision(vector<vector<vector<Slot>>> belief, int lane, int acc, int time_from){
      // Check predictable region
      if(time_from+6 > belief.size()){
        return true;
      }

      // Check target lane
      for(unsigned int t=time_from; t<time_from+6; t++){
        if(belief[t][acc][lane].is_occupied){
          return true;
        }
      }
      return false;
    }

    double target_speed(double d, int target_lane, double curr_lane_speed, double target_lane_speed){
      int target_center = target_lane * 4 + 2;
      double delta = fabs(d - target_center);
      double alpha = delta/6;
      double speed = alpha * curr_lane_speed + (1-alpha) * target_lane_speed;
      return speed;
    }
};
#endif
