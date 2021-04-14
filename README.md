# RLSS: Distributed Real-time Trajectory Replanning using Linear Spatial Separations
<p float="left">
<img src="https://github.com/usc-actlab/rlss/blob/master/gifs/rlss_1.gif?raw=true" width="300"/>
<img src="https://github.com/usc-actlab/rlss/blob/master/gifs/rlss_2.gif?raw=true" width="300"/>
<img src="https://github.com/usc-actlab/rlss/blob/master/gifs/rlss_3.gif?raw=true" width="300"/>
<img src="https://github.com/usc-actlab/rlss/blob/master/gifs/rlss_4.gif?raw=true" width="300"/>
</p>

RLSS is a real time trajectory replanning algorithm for multi robot teams.
RLSS combines A*-based discrete search with QP based trajectory optimization
where linear separations enforce safety.


RLSS,
* computes provably executable trajectories for differentially flat robots
* enforces safety as hard constraints
* can work in real-time (1-10 Hz)
* requires perfect sensing of only the positions of robots and obstacles
* does not depend on communication for safety

## Citing RLSS
Please use the following entry to cite RLSS:

```
@article{senbaslar2021rlss,
    author = {{{\c{S}}enba{\c{s}}lar}, Bask{\i}n and {H{\"o}nig}, Wolfgang and {Ayanian}, Nora},
    title = "{RLSS: Real-time Multi-Robot Trajectory Replanning using Linear Spatial Separations}",
    journal = {arXiv e-prints},
    keywords = {Computer Science - Robotics},
    year = 2021,
    month = mar,
    eid = {arXiv:2103.07588},
    pages = {arXiv:2103.07588},
    archivePrefix = {arXiv},
    eprint = {2103.07588},
    primaryClass = {cs.RO}
}
```

## Dependencies
RLSS depends on [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), [Boost](https://www.boost.org/).
It also depends on [ILOG CPLEX C++](https://www.ibm.com/products/ilog-cplex-optimization-studio), [GUROBI](https://www.gurobi.com/products/gurobi-optimizer/) as optimization libraries. 


## Building
```
git clone -b master https://github.com/usc-actlab/rlss/
git submodule update --init --recursive
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_RLSS_EXAMPLES=ON -DENABLE_RLSS_JSON_BUILDER=ON ..
make -j4
```

### If you are having problems linking Gurobi
```
cd $GUROBI_HOME/src/build
make
cd $GUROBI_HOME/lib
rm libgurobi_c++.a
ln -s ../src/build/libgurobi_c++.a  libgurobi_c++.a
```
    
### Targets 
* `make 2d_sim`: build 2D simulator example
* `make 3d_sim`: build 3D simulator example
* `make build_rlss_examples`: `2d_sim` and `3d_sim`
* `make build_rlss_tests`: build tests
* `make test`: run tests


## Running Simulator Examples & Visualization
* Set `base_path` to the rlss root folder in `examples/2d_config.json` and `examples/3d_config.json`
* For 2D visualization run:
    ```
    cd build
    ./2d_sim (outputs vis.json)
    python3 ../tools/vis/2d/run.py vis.json
    ```
* 3D visualizer requires [ROS](https://www.ros.org/) and [rviz](http://wiki.ros.org/rviz).
* For 3D visualizer run:
    ```
    (terminal A) cd build
    (terminal A) ./3d_sim (outputs vis.json)
    (terminal B) roscore
    (terminal C) rosrun rviz rviz
    (rviz) Add > MarkerArray (topic: visualization_marker_array)
    (rviz) Set background color to white
    (terminal A) python3 ../tools/vis/3d/run.py
    (terminal A) start vis.json (reads vis.json and displays the result in rviz)
    ```
### 3D Visualizer Commands
3D visualizer accepts several command line commands for visualization.
* `start <simulation_output>`: Starts the visualization for the simulation output file
* `stop`: Stops the current visualization
* `forward`: Run visualization forward
* `backward`: Run visualization backward
* `pause`: Pause visualization
* `step`: Move 1 step in visualization
* `unpause`: Unpause visualization
* `help`: List available commands
* `exit`: Exit visualizer

## ROS Integration

The ROS integration of RLSS is developed in a separate repo: [rlss_ros](https://github.com/usc-actlab/rlss_ros).
