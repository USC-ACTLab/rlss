#include "occupancy_grid.h"

OG::index::index(int a, int b): i(a), j(b), d(direction::NONE) {

}

OG::index::index(int a, int b, direction c): i(a), j(b), d(c) {

}


OG::OG(double step_size, double xm, double xM, double ym, double yM, vector<obstacle2D>& obstacles): ss(step_size), ss2(step_size/2), x_min(xm), y_min(ym), x_max(xM), y_max(yM) {

  const size_t numColumns = ceil((x_max - x_min) / ss);
  const size_t numRows = ceil((y_max - y_min) / ss);
  // initialize as empty
  grid.resize(numRows, vector<bool>(numColumns, false));

  const double sampleDist = 0.01;
  for(double y = y_min; y<=y_max; y+= sampleDist) {
    for(double x = x_min; x<=x_max; x+= sampleDist) {
      vectoreuc pt(2);
      pt[0] = x;
      pt[1] = y;
      bool occupied = false;
      for(int i=0; i<obstacles.size(); i++) {
        obstacle2D& obs = obstacles[i];
        if(obs.point_inside(pt)) {
            set_occupied(x, y);
            break;
        }
      }
    }
  }
}


OG::index OG::get_index(double x, double y) {
  int yidx = (y-y_min) / ss;
  int xidx = (x-x_min) / ss;

  return OG::index(yidx, xidx);
}


bool OG::idx_occupied(index& idx) {
  int r = grid.size();
  int c = grid[0].size();

  if(idx.i >= r || idx.i < 0 || idx.j >= c || idx.j <0) {
    return true;
  }

   return grid[idx.i][idx.j];
  // return grid[idx.i][idx.j] || ((idx.i + 1 < r) && grid[idx.i+1][idx.j])
  //     || ((idx.j + 1 < c) && grid[idx.i][idx.j+1]) || ((idx.i+1 < r && idx.j+1 < c) && grid[idx.i+1][idx.j+1])
  //     || ((idx.i-1 > -1) && grid[idx.i-1][idx.j]) || ((idx.j-1 > -1) &&grid[idx.i][idx.j-1])
  //     || ((idx.i-1 > -1 && idx.j-1 > -1) && grid[idx.i-1][idx.j-1]);
}

bool OG::occupied(double x, double y, double robot_radius)
{
  // check all neighbors within the specified radius

  index idxMin = get_index(x - robot_radius, y - robot_radius);
  index idxMax = get_index(x + robot_radius, y + robot_radius);

  for (int i = idxMin.i; i <= idxMax.i; ++i) {
    for (int j = idxMin.j; j <= idxMax.j; ++j) {
      index idx(i, j);
      if (idx_occupied(idx)) {
        return true;
      }
    }
  }
  return false;
}


pair<double, double> OG::get_coordinates(OG::index& idx) {
  double y = idx.i * ss + y_min + ss2;
  double x = idx.j * ss + x_min + ss2;

  return make_pair(x, y);
}

vector<OG::index> OG::neighbors(index& cur) {
  vector<OG::index> n;
  index up(cur.i+1, cur.j, OG::direction::UP);

  if(!idx_occupied(up)) {
    n.push_back(up);
  }

  index down(cur.i-1, cur.j, OG::direction::DOWN);
  if(!idx_occupied(down)) {
    n.push_back(down);
  }


  index left(cur.i, cur.j-1, OG::direction::LEFT);
  if(!idx_occupied(left)) {
    n.push_back(left);
  }


  index right(cur.i, cur.j+1, OG::direction::RIGHT);
  if(!idx_occupied(right)) {
    n.push_back(right);
  }

  return n;
}

bool OG::occupied(trajectory& traj, double robot_radius, double start_time, double end_time)
{
  if (end_time < 0) {
    end_time = traj.duration();
  }
  const double dt = 0.01;
  for(double t = start_time; t<=end_time; t+=dt) {
    vectoreuc pos = traj.eval(t);
    if(occupied(pos[0], pos[1], robot_radius)) {
      return true;
    }
  }
  return false;
}

bool OG::occupied(splx::BSpline& spl, double robot_radius, double start_time, double end_time) {
  if(end_time < 0) {
    end_time = spl.m_b;
  }
  const double dt = 0.01;

  for(double t = start_time; t<=end_time; t+=dt) {
    splx::Vec pos = spl.eval(t, 0);
    if(occupied(pos(0), pos(1), robot_radius)) {
      return true;
    }
  }
  return false;
}

size_t OG::max_i() const
{
  return grid.size();
}

size_t OG::max_j() const
{
  return grid[0].size();
}

void OG::set_occupied(double x, double y)
{
  index idx = get_index(x, y);
  grid[idx.i][idx.j] = true;
}

void OG::set_free(double x, double y)
{
  index idx = get_index(x, y);
  grid[idx.i][idx.j] = false;
}
