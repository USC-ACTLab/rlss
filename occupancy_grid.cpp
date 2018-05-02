#include "occupancy_grid.h"

OG::index::index(int a, int b): i(a), j(b), d(direction::NONE) {

}

OG::index::index(int a, int b, direction c): i(a), j(b), d(c) {

}


OG::OG(double step_size, double xm, double xM, double ym, double yM, vector<obstacle2D>& obstacles): ss(step_size), ss2(step_size/2), x_min(xm), y_min(ym), x_max(xM), y_max(yM) {
  for(double y = y_min + ss2; y<=y_max; y+=ss) {
    vector<bool> distRow;
    for(double x = x_min + ss2; x<=x_max; x+=ss) {
      vectoreuc pt(2);
      pt[0] = x;
      pt[1] = y;
      bool occupied = false;
      for(int i=0; i<obstacles.size(); i++) {
        obstacle2D& obs = obstacles[i];
        if(obs.point_inside(pt)) {
            occupied = true;
            break;
        }
      }
      distRow.push_back(occupied);
    }
    grid.push_back(distRow);
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

  return grid[idx.i][idx.j] || ((idx.i + 1 < r) && grid[idx.i+1][idx.j])
      || ((idx.j + 1 < c) && grid[idx.i][idx.j+1]) || ((idx.i+1 < r && idx.j+1 < c) && grid[idx.i+1][idx.j+1])
      || ((idx.i-1 > -1) && grid[idx.i-1][idx.j]) || ((idx.j-1 > -1) &&grid[idx.i][idx.j-1])
      || ((idx.i-1 > -1 && idx.j-1 > -1) && grid[idx.i-1][idx.j-1]);
}

bool OG::occupied(double x, double y) {
  index idx = get_index(x, y);
  return idx_occupied(idx);
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

bool OG::occupied(trajectory& traj) {
  double dt = 0.01;
  for(double t = 0; t<=traj.total_duration; t+=dt) {
    vectoreuc pos = traj.eval(t);
    if(occupied(pos[0], pos[1])) {
      return true;
    }
  }
  return false;
}
