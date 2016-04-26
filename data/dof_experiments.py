from subprocess import Popen
import os
import numpy
import seaborn.apionly as sns

def flatten_params(d):
    to_return = [];
    for (key, value) in d.iteritems():
        to_return.append("_" + str(key) + ':=' + str(value));
    print to_return
    return to_return;
    

arm_slam_params = dict();


dofs = xrange(4, 20)
num_experiments = 25

for ex in xrange(0, num_experiments):
    for d in dofs:
        print "Running experiment " + str(ex + 1) + "/" + str(num_experiments) + " num dofs: " + str(d);

        
        dirName = './dof_experiments/'+str(d);
        postfix = '_' + str(ex + 1);

        print dirName;
        print postfix;
        if not os.path.exists(dirName):
            os.makedirs(dirName);
        
        arm_slam_params['num_dofs'] = d;
        arm_slam_params['data_dir'] = dirName;
        arm_slam_params['data_postfix'] = postfix;
        arm_slam_params['seed'] = ex;

        calib_process = Popen(["rosrun", "arm_slam_calib", "sim_genrobot"] + flatten_params(arm_slam_params));
        calib_process.wait();
        
