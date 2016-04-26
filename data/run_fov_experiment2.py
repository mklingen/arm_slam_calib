from subprocess import Popen
import os
import numpy
import seaborn.apionly as sns
import math

def flatten_params(d):
    to_return = [];
    for (key, value) in d.iteritems():
        to_return.append("_" + str(key) + ':=' + str(value));
    print to_return
    return to_return;
    

arm_slam_params = dict();


fx_base = 320;
fy_base = 240;
cx_base = 160;
cy_base = 120;


num_experiments = 25
multipliers = numpy.linspace(0.5, 3.0);

for ex in xrange(0, num_experiments):
    for m in multipliers:
        print "Running experiment " + str(ex + 1) + "/" + str(num_experiments) + " fov: " + str(m);

        fov =  round(180 * (2 * math.atan2(320/2, fx_base * m)) / 3.14159)
        dirName = './fov_experiments/'+str(fov);
        postfix = '_' + str(ex + 1);

        print dirName;
        print postfix;
        if not os.path.exists(dirName):
            os.makedirs(dirName);
        
        arm_slam_params['num_dofs'] = 6;
        arm_slam_params['data_dir'] = dirName;
        arm_slam_params['data_postfix'] = postfix;
        arm_slam_params['seed'] = ex;
        arm_slam_params['fx'] = fx_base * m;
        arm_slam_params['fy'] = fy_base * m;
        arm_slam_params['cx'] = cx_base * m;
        arm_slam_params['cy'] = cy_base * m;
        arm_slam_params['trajectory_length'] = 200;

        calib_process = Popen(["rosrun", "arm_slam_calib", "sim_genrobot"] + flatten_params(arm_slam_params));
        calib_process.wait();
        
