# Multi Robot Trajectory Replanning

### To build the project

```
sudo apt install libeigen3-dev
mkdir build
cd build
cmake ..
make
```
### Start optimization
```
./main
```

To give a different configuration file
```
./main --cfg newconfig.json
```

If you do not use --cfg, default file path is ../config.json.

Check config.json for the sample config file.

Notice that in default config file "obstacles" key has value of "../test_data/obstacled/noobs". Since noobs should be an empty folder, pushing it into git was not possible. Create this folder by yourself if you have no obstacles.

Optimization results in a "res.json" file.

### Visualize the results

To install dependencies
```
sudo pip install bezier
sudo pip install numpy
```

Run visualization

```
cd visualization_tool
python launch.py ../build/res.json
./rviz.sh
```

The visualization tool accepts commands while simulation is running.
The commands could be sent by typing the command and pressing enter

```python
p #play/pause the visualization

t time_diff #Seek by time_diff seconds Ex: t 10 will seek forward by 10 seconds and t -5 will seek backward by 5 seconds 
```

## Profiling

### Install

```
sudo apt install google-perftools
```

Follow instructions for visualizer at https://github.com/google/pprof

### Run

```
LD_PRELOAD="/usr/lib/libprofiler.so" CPUPROFILE=prof.out ./main
```

### Visualize

```
~/go/bin/pprof -web prof.out
```



## NL-OPT issues

https://github.com/scipy/scipy/issues/7618
https://scicomp.stackexchange.com/questions/83/is-there-a-high-quality-nonlinear-programming-solver-for-python
https://www.jstage.jst.go.jp/article/jrsj/32/6/32_32_536/_pdf

## Test Cases

### Standard cases

* Obstacle appears for one (or multiple) robots
* One robot breaks down (suddenly stops on its trajectory)
* One robot gets moved externally (e.g., temporary control issue, wind gust etc.)
* Obstacle no longer present
* U-shape obstacle example

### Hard cases

* Moving obstacle
* New robot participates that was unaware of other robots (e.g, head-to-head collision)

## TODO

* try osqp instead of cvxgen for numerical stability?
* automatic test/movie generation?
* add soft margin to objective to improve numerical stability?
