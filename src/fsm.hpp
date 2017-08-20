#ifndef FSM_H
#define FSM_H

#include <iostream>
#include <string>
#include "json.hpp"

using json = nlohmann::json;

// ego car state
struct ego_car{
    double x;
    double y;
    double s;
    double d;
    double yaw;
    double speed;
};

class FSM{
public:
    std::string current_state; // Keep lane, left or right
    int current_lane;
    int lanes_available; // totoal number of lanes
    double target_speed;
    float safety_coeff = 0.5;

    FSM(int current_lane_, double target_speed_, int lanes_available_);
    ~FSM();

    void UpdateState(json predictions, ego_car ego);
    float cost_for_action(json predictions, std::string action, ego_car ego);

};
#endif