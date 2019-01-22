#pragma once
#include "OccupancyGrid3D.h"
#include <boost/functional/hash.hpp>
#include "libSearch/a_star.hpp"

using namespace ACT;

namespace discreteSearch {

enum Direction {
  NONE,
  XP, // x++
  XM, // x--
  YP, // y++
  YM, // y--
  ZP, // z++
  ZM  // z--
};

struct State
{
  State(int x, int y, int z, Direction direction)
    : x(x)
    , y(y)
    , z(z)
    , direction(direction)
  {
  }

  bool operator==(const State& other) const
  {
    return std::tie(x, y, z) == std::tie(other.x, other.y, other.z);
  }

  friend std::ostream& operator<< ( std::ostream& os, const State& s)
  {
    return os << "(" << s.x << "," << s.y << ","  << s.z << "," << s.direction << ")";
  }

  int x;
  int y;
  int z;
  Direction direction;
};

// We have a axis aligned frame where X points forward, Y points left
// left rotation means rotating around axis in right-handed way
enum class Action
{
  Forward,
  RotateRight,
  RotateLeft,
  RotateUp,
  RotateDown
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
    case Action::RotateUp:
      os << "RotateUp";
      break;
    case Action::RotateDown:
      os << "RotateDown";
      break;
  }
  return os;
}

template<typename T>
class Environment
{
public:
  Environment(
    OccupancyGrid3D<T>& og,
    T robotRadius,
    const State& goal)
    : m_og(og)
    , m_robotRadius(robotRadius)
    , m_goal(goal)
  {

    if (!stateValid(goal)) {
      std::cerr << "GOAL not valid! goal: " << goal.x << " " << goal.y
        << " " << goal.z  << std::endl;
    }
  }

  int admissibleHeuristic(
    const State& s)
  {
    return std::abs(s.x - m_goal.x) + std::abs(s.y - m_goal.y)
      + std::abs(s.z - m_goal.z);
  }

  bool isSolution(
    const State& s)
  {
    return s.x == m_goal.x && s.y == m_goal.y && s.z == m_goal.z;
  }

  void getNeighbors(
    const State& s,
    std::vector<libSearch::Neighbor<State, Action, int> >& neighbors)
  {
    neighbors.clear();

    // forward actions
    if (   s.direction == Direction::NONE
        || s.direction == Direction::XP) {
      State xp(s.x + 1, s.y, s.z, Direction::XP);
      if (stateValid(xp)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(xp, Action::Forward, 1));
      }
    }
    if (   s.direction == Direction::NONE
        || s.direction == Direction::XM) {
      State xm(s.x - 1, s.y, s.z, Direction::XM);
      if (stateValid(xm)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(xm, Action::Forward, 1));
      }
    }
    if (   s.direction == Direction::NONE
        || s.direction == Direction::YP) {
      State yp(s.x, s.y + 1, s.z, Direction::YP);
      if (stateValid(yp)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(yp, Action::Forward, 1));
      }
    }
    if (   s.direction == Direction::NONE
        || s.direction == Direction::YM) {
      State ym(s.x, s.y - 1, s.z, Direction::YM);
      if (stateValid(ym)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(ym, Action::Forward, 1));
      }
    }
    if (   s.direction == Direction::NONE
        || s.direction == Direction::ZP) {
      State zp(s.x, s.y, s.z + 1, Direction::ZP);
      if (stateValid(zp)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(zp, Action::Forward, 1));
      }
    }
    if (   s.direction == Direction::NONE
        || s.direction == Direction::ZM) {
      State zm(s.x, s.y, s.z - 1, Direction::ZM);
      if (stateValid(zm)) {
        neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(zm, Action::Forward, 1));
      }
    }

    // rotate actions
    if (s.direction == Direction::XP) {
      State rot1(s.x, s.y, s.z, Direction::YP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, s.z, Direction::YM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
      State rot3(s.x, s.y, s.z, Direction::ZP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot3, Action::RotateUp, 1));
      State rot4(s.x, s.y, s.z, Direction::ZM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot4, Action::RotateDown, 1));
    }

    if (s.direction == Direction::XM) {
      State rot1(s.x, s.y, s.z, Direction::YM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, s.z, Direction::YP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
      State rot3(s.x, s.y, s.z, Direction::ZP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot3, Action::RotateUp, 1));
      State rot4(s.x, s.y, s.z, Direction::ZM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot4, Action::RotateDown, 1));
    }

    if (s.direction == Direction::YP) {
      State rot1(s.x, s.y, s.z, Direction::XM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, s.z, Direction::XP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
      State rot3(s.x, s.y, s.z, Direction::ZP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot3, Action::RotateUp, 1));
      State rot4(s.x, s.y, s.z, Direction::ZM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot4, Action::RotateDown, 1));
    }

    if (s.direction == Direction::YM) {
      State rot1(s.x, s.y, s.z, Direction::XP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, s.z, Direction::XM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
      State rot3(s.x, s.y, s.z, Direction::ZP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot3, Action::RotateUp, 1));
      State rot4(s.x, s.y, s.z, Direction::ZM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot4, Action::RotateDown, 1));
    }

    if (s.direction == Direction::ZP) {
      State rot1(s.x, s.y, s.z, Direction::XM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, s.z, Direction::XP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
      State rot3(s.x, s.y, s.z, Direction::YP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot3, Action::RotateUp, 1));
      State rot4(s.x, s.y, s.z, Direction::YM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot4, Action::RotateDown, 1));
    }

    if (s.direction == Direction::ZM) {
      State rot1(s.x, s.y, s.z, Direction::XP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot1, Action::RotateLeft, 1));
      State rot2(s.x, s.y, s.z, Direction::XM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot2, Action::RotateRight, 1));
      State rot3(s.x, s.y, s.z, Direction::YP);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot3, Action::RotateUp, 1));
      State rot4(s.x, s.y, s.z, Direction::YM);
      neighbors.emplace_back(libSearch::Neighbor<State, Action, int>(rot4, Action::RotateDown, 1));
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
    typename OccupancyGrid3D<T>::Index idx(s.x, s.y, s.z);
    auto coord = m_og.getCoordinates(idx);
    try {
      if(m_og.isOccupied(coord(0), coord(1), coord(2), m_robotRadius)) {
        return false;
      }
    } catch(const typename OccupancyGrid3D<T>::OutOfBoundsException& exp) {
      return false;
    }

    return true;

  }

private:
  OccupancyGrid3D<T>& m_og;
  T m_robotRadius;
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
      boost::hash_combine(seed, s.z);
      boost::hash_combine(seed, s.direction);
      return seed;
    }
};
}
