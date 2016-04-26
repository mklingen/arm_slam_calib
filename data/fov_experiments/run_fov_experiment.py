from subprocess import Popen
import os
import numpy
import math

def flatten_params(d):
    to_return = [];
    for (key, value) in d.iteritems():
        to_return.append("_" + str(key) + ':=' + str(value));
    print to_return
    return to_return;
    
fx_base = 320;
fy_base = 240;
cx_base = 160;
cy_base = 120;


arm_slam_params = dict();
camera_params = dict();
camera_params['cx'] = cx_base;
camera_params['cy'] = cy_base;
camera_params['scene_file'] = "/home/mklingen/prdev/SceneGraphRendering/data/bedroom1.obj"
camera_params['data_directory'] = "/home/mklingen/prdev/SceneGraphRendering/data/"

multipliers = numpy.linspace(0.5, 3.0);

for m in multipliers:
    print "Running with multiplier " + str(m);
    camera_params['fx'] = fx_base * m;
    camera_params['fy'] = fy_base * m;
    fov =  round(180 * (2 * math.atan2(320/2, fx_base * m)) / 3.14159)
    dirName = './'+str(fov)
    print dirName;
    if not os.path.exists(dirName):
        os.makedirs(dirName);
    else:
        continue;
    
    
    camera_process = Popen(["rosrun", "scene_graph_ros", "scene_graph_ros_node"] + flatten_params(camera_params));
    calib_process = Popen(["roslaunch", "arm_slam_calib", "launch_mico_sim.launch"]);
    calib_process.wait();
    camera_process.kill();
    
    Popen('mv ~/.ros/*.txt ' + dirName, shell=True);
