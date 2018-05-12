#pragma once
#include "occupancy_grid.h"
#include <boost/functional/hash.hpp>
#include "libSearch/a_star.hpp"

namespace discreteSearch {

struct State
{
  State(int x, int y, OG::direction direction)
    : x(x)
    , y(y)
    , direction(direction)
  {
  }

  bool operator==(const State& other) const
  {
    return std::tie(x, y, direction) == std::tie(other.x, other.y, direction);
  }

  friend std::ostream& operator<< ( std::ostream& os, const State& s)
  {
    return os << "(" << s.x << "," << s.y << "," << s.direction << ")";
  }

  int x;
  int y;
  OG::direction direction;
};

enum class Action
{
  Forward,
  RotateLeft,
  RotateRight,
};

std::ostream& operator<< ( std::ostream& os, const Action& a)
{
  switch(a)
  {
    case Action::Forward:
      os << "Forward";
      break;
    case Action::RotateLeft:
      os << "RotateLeft";
      break;
    case Action::RotateRight:
      os << "RotateRight";
      break;
  }
  return os;
}

class Environment
{
public:
  Environment(
    OG& og,
    const std::vector< std::pair<double, double> >& otherRobots,
    double robotRadius,
    const State& goal)
    : m_og(og)
    , m_otherRobots(otherRobots)
    , m_robotRadius(robotRadius)
    , m_goal(goal)
  {

    if (!stateValid(goal)) {
      std::cerr << "GOAL not valid!" << std::endl;
    }
  }

  int admissibleHeuristic(
    const State& s)
  {
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y);
  }

  bool isSolution(
    const State& s)
  {
    return s.x == m_goal.x && s.y == m_goal.y;
  }

  void getNeighbors(
    const State& s,
    std::vector<libSearch::Neighbor<State, Action, int> >& neighbors)
  {
    neighbors.clear();

    // forward actions
    if (   s.direction == OG::direction::NONE
        || s.direction == OG::direction::UP) {
      State up(s.x, s.y+1, OG::direction::UP);
      if (stateValid(up)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(up, Action::Forward, 1));
      }
    }
    if (   s.direction == OG::direction::NONE
        || s.direction == OG::direction::DOWN) {
      State down(s.x, s.y-1, OG::direction::DOWN);
      if (stateValid(down)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(down, Action::Forward, 1));
      }
    }
    if (   s.direction == OG::direction::NONE
        || s.direction == OG::direction::LEFT) {
      State left(s.x-1, s.y, OG::direction::LEFT);
      if (stateValid(left)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(left, Action::Forward, 1));
      }
    }
    if (   s.direction == OG::direction::NONE
        || s.direction == OG::direction::RIGHT) {
      State right(s.x+1, s.y, OG::direction::RIGHT);
      if (stateValid(right)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(right, Action::Forward, 1));
      }
    }
    // rotate actions
    if (s.direction == OG::direction::UP) {
      State rot1(s.x, s.y, OG::direction::LEFT);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, OG::direction::RIGHT);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
    }
    if (s.direction == OG::direction::DOWN) {
      State rot1(s.x, s.y, OG::direction::RIGHT);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, OG::direction::LEFT);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
    }
    if (s.direction == OG::direction::LEFT) {
      State rot1(s.x, s.y, OG::direction::DOWN);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, OG::direction::UP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
    }
    if (s.direction == OG::direction::RIGHT) {
      State rot1(s.x, s.y, OG::direction::UP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, OG::direction::DOWN);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
    }

  }

  void onExpandNode(
    const State& s,
    int fScore,
    int gScore)
  {
  }

  void onDiscover(
    const State& s,
    int fScore,
    int gScore)
  {
  }

private:
  bool stateValid(
    const State& s)
  {
    // check if occupied in grid
    OG::index idx(s.x, s.y);
    std::pair<double, double> coord = m_og.get_coordinates(idx);
    if (m_og.occupied(coord.first, coord.second, m_robotRadius)) {
      return false;
    }

    // check if occupied by another robot
    for (const auto& otherRobot : m_otherRobots) {
      // double distSq = pow(otherRobot.first - coord.first, 2) + pow(otherRobot.second - coord.second, 2);
      // if (distSq < pow(2 * m_robotRadius, 2)) {
      //   return false;
      // }

      if (   fabs(coord.first - otherRobot.first) <= 2 * m_robotRadius
          && fabs(coord.second - otherRobot.second) <= 2 * m_robotRadius) {
        return false;
      }
    }

    return true;

  }

private:
  OG& m_og;
  const std::vector< std::pair<double, double> >& m_otherRobots;
  double m_robotRadius;
  State m_goal;
};

} // namespace discreteSearch

namespace std
{
template<>
struct hash<discreteSearch::State> {
    size_t operator()(const discreteSearch::State& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, s.x);
      boost::hash_combine(seed, s.y);
      boost::hash_combine(seed, s.direction);
      return seed;
    }
};
}
