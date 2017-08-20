#include "fsm.hpp"

using namespace std;

FSM::FSM(int current_lane_, double target_speed_, int lanes_available_){
    current_lane = current_lane_;
    target_speed = target_speed_;
    lanes_available = lanes_available_;
    current_state = "KL";
}

FSM::~FSM(){
}

void FSM::UpdateState(json predictions, ego_car ego){
  
  vector<string> possible_actions;
  if (current_state == "KL") {
    possible_actions = { "KL", "LCL", "LCR" };
  }
  if (current_state == "LCL") {
    possible_actions = { "KL" };
  }
  if (current_state == "LCR") {
    possible_actions = { "KL" };
  }
  
  vector<float> costs;
  for (auto const& a: possible_actions) costs.push_back(
      FSM::cost_for_action(predictions, a, ego));

  // DEBUG
  // cout << "Actions : ";
  // for (auto const& a: possible_actions) cout << a << ", " ;
  // cout << endl << "Costs: ";
  // for (auto const& c: costs) cout << c << ", " ;
  // cout << endl;
  
  int min_pos = distance(costs.begin(), min_element(costs.begin(), costs.end()));
  // cout << "min_pos " << min_pos << endl;
  current_state = possible_actions[min_pos];
  // cout << "current state: " << current_state << endl;
  if(current_state == "LCL") current_lane -= 1;
  if(current_state == "LCR") current_lane += 1;
}

float FSM::cost_for_action(json predictions, string action, ego_car ego){
  // calculate gaps (front & rear), in Frenet coordinates
  float front_gaps[3];
  float rear_gaps[3];
  for (int l=0; l<lanes_available; l++) {
    front_gaps[l] = numeric_limits<float>::max();
    rear_gaps[l] = numeric_limits<float>::max();
  }
  for (auto const& entry: predictions) {
    // int vehicle_id = entry[0];
    float gap = float(entry[5]) - ego.s;
    int vl = int(float((entry[6])) / 4); // vehicle lane

    // find the smallest gap for both front and rear in each lane
    if (gap > 0 && front_gaps[vl] > gap) {
      front_gaps[vl] = gap;
    }
    else if(gap <= 0 && rear_gaps[vl] > fabs(gap)){
      rear_gaps[vl] = fabs(gap);
    }
  }

  if (action == "KL") {
    int target_lane = current_lane;
    float front_gap_cost = target_speed / front_gaps[target_lane];
    return front_gap_cost;
  }

  if (action == "LCL") {
    int target_lane = current_lane - 1;
    if (target_lane < 0) return numeric_limits<float>::max();
    // total cost is the sum of front gap cost and the rear gap cost
    float front_gap_cost = target_speed / front_gaps[target_lane];
    float rear_gap_cost = target_speed / fabs(rear_gaps[target_lane]);
    float total_cost = front_gap_cost + rear_gap_cost * safety_coeff;
    // cout << "left cost" << front_gap_cost << endl;
    return total_cost;
  }
  if (action == "LCR") {
    int target_lane = current_lane + 1;
    // cout << "target lane: " << target_lane << endl;
    // cout << "lanes available: " << lanes_available - 1 << endl;
    if (target_lane > lanes_available - 1) return numeric_limits<float>::max();

    float front_gap_cost = target_speed / front_gaps[target_lane];
    float rear_gap_cost = target_speed / fabs(rear_gaps[target_lane]);
    float total_cost = front_gap_cost + rear_gap_cost * safety_coeff;
    // cout << "right cost" << front_gap_cost << endl;
    return total_cost;
  }
  return numeric_limits<float>::max();
}
