#ifndef BEHAVIOR_H_
#define BEHAVIOR_H_

#include <iostream>
#include <vector>
#include "telemetry.h"
#include "map.h"
#include "trigs.h"
#include "belief.h"
#include "config.h"

#define _INF 100000

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
      cout<<"behavior start"<<endl;
      unsigned int curr_lane = (int)tl.d/4;
      unsigned int next_row = (SLOTS-1)/2-(int)(tl.speed + SLOT_RAD)/SLOT_LENGTH;

      vector<vector<vector<int>>> dp(PREDICTOR_TIME_SLOTS, vector<vector<int>>(SLOTS, vector<int>(LANES, 0)));

      //Initialize last layer
      unsigned int last_layer = PREDICTOR_TIME_SLOTS - 1; 
      for(unsigned int i = 0; i < SLOTS; i++){
        for(unsigned int j = 0; j < LANES; j++){
          if(belief[last_layer][i][j].is_occupied){
            dp[last_layer][i][j] = 0;
          }else {
            dp[last_layer][i][j] = 10;
          }
        }
      }

      for(int t=PREDICTOR_TIME_SLOTS-2; t>=0; t--){
        for(unsigned int s=0; s<SLOTS; s++){
          //cout<<"Slot "<<s<<endl;
          for(unsigned int lane=0; lane<LANES; lane ++){
            if(belief[t][s][lane].is_occupied){
              dp[t][s][lane] = 0;
              continue;
            }

            //Keep speed, deaccelerate, accelerate
            for(int acc=0; acc<3; acc++){
              int source_x = s + acceleration[acc];

              // Overshoot predictable space
              if(source_x<0 || source_x>=SLOTS){
                continue;
              }

              // Check possible collision
              //if(belief[t][source_x][lane].is_occupied || belief[t+1][source_x][lane].is_occupied){
              //continue;
              //}

              // Update cost
              //cout << "Update cost "<<t<<" "<<s<<" "<<lane<<endl;
              dp[t][s][lane] = Trigs::max(dp[t][s][lane], dp[t+1][source_x][lane]+1);
            }
          }
        }
      }

      BPosition pos;

      cout<<"Lanes: "<<endl;
      int maneuver[3] = {-1, -1, -1};
      for(int d_lane=-1; d_lane<2; d_lane++){
        int target_lane = curr_lane + d_lane;
        if(target_lane>=0 && target_lane < LANES){
          int lane_cost = dp[6][99][target_lane];
          for(int i=0; i<6; i++){
            if(belief[i][100][target_lane].is_occupied){
              lane_cost = 0;
            }
          }
          maneuver[target_lane] = lane_cost;
          cout<<lane_cost<<"; ";
        }
      }
      cout<<endl;

      pos.lane=curr_lane;

      int pos_lane_cost = -1;

      for(int target_lane=0; target_lane<3; target_lane ++){
        if(maneuver[target_lane] > pos_lane_cost){
          pos_lane_cost = maneuver[target_lane];
          pos.lane = target_lane;
        }
      }

      int keep_speed_cost = dp[1][100][curr_lane];
      int accelerate_cost = dp[1][99][curr_lane];
      int deaccelerate_cost = dp[1][101][curr_lane];
      cout<<accelerate_cost<<"; "<<keep_speed_cost<<"; "<<deaccelerate_cost<<endl;
      int max_cost = Trigs::max(keep_speed_cost, accelerate_cost, deaccelerate_cost);
      if(accelerate_cost==max_cost && max_cost>0){
        pos.speed = tl.speed+2;
      }else if(keep_speed_cost==max_cost && max_cost>0){
        pos.speed = tl.speed;
      }else if(deaccelerate_cost == max_cost) {
        pos.speed = tl.speed - 2;
      }

      pos.speed = Trigs::min(pos.speed, SPEED_LIMIT);

      return pos;
    }

  private:
    BehaviorPlanner() {} 
    ~BehaviorPlanner() {}

    BehaviorPlanner(BehaviorPlanner const&); 
    BehaviorPlanner& operator= (BehaviorPlanner const&); 

    int acceleration[3] = { -1, 0, 1};
};
#endif
