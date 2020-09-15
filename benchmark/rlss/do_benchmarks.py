import os
import json
import subprocess

_2d_solver = "/home/baskin/repos/rlss/cmake-build-release/cplex/2d_sim"
_3d_solver = "/home/baskin/repos/rlss/cmake-build-release/cplex/3d_sim"
result_checker = "/home/baskin/repos/rlss/tools/scripts/result_check.py"

strategies = ["hard", "soft", "hard-soft"]

for strategy in strategies:
    for setup_name in os.listdir(strategy):
        print(strategy, setup_name)

        cfgf = open("./configs/" + setup_name + ".json", "r")
        cfg = json.loads(cfgf.read())
        cfgf.close()
        
        cfg["optimizer"] = "rlss-" + strategy

        cfgf = open("config.json", "w")
        cfgf.write(json.dumps(cfg))
        cfgf.close()

        dimension = len(cfg["occupancy_grid_step_size"])

        if dimension == 2:
            solver = _2d_solver
        elif dimension == 3:
            solver = _3d_solver
        else:
            raise Exception("unknown dimension")

        
        subprocess.run([solver, "-c", "./config.json"], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
        subprocess.run(["mv", "vis.json", "all_stats.json", strategy + "/" + setup_name + "/"])

        if dimension == 3:
            p = subprocess.Popen("python3 " + result_checker + " " + strategy + "/" + setup_name + "/vis.json > late_stats", shell = True)
            os.waitpid(p.pid, 0)
            subprocess.run(["mv", "late_stats", strategy + "/" + setup_name + "/"])

        