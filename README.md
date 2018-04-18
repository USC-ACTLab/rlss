# Multi Robot Trajectory Replanning

### To build the project

```
sudo apt install libnlopt-dev;
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
sudo pip3 install shapely
sudo pip3 install descartes
```

Run visualization

```
python3 vis.py ../build/res.json
```

To save the images of each frame to images/ folder

```
python3 vis.py ../build/res.json save
```

Saving waits until frame number 300 (3 seconds). Until that time, zoom the simulation for better images.
