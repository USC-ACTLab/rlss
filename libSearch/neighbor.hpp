#pragma once


namespace libSearch {

template <
  typename State,
  typename Action,
  typename Cost
  >
struct Neighbor
{
  Neighbor(
    const State& state,
    const Action& action,
    Cost cost)
    : state(state)
    , action(action)
    , cost(cost)
  {
  }

  State state;
  Action action;
  Cost cost;
};

} // namespace libSearch
