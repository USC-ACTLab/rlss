#pragma once

#include <vector>

namespace libSearch {


template <
  typename State,
  typename Action,
  typename Cost
  >
struct PlanResult
{
  std::vector<std::pair<State, Cost> > states;  // states and their gScore
  std::vector<std::pair<Action, Cost> > actions; // actions and their cost
  Cost cost; // actual cost of the result
  Cost fmin; // lower bound of the cost (for suboptimal solvers)
};


} // namespace libSearch
