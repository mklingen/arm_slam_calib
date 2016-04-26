import scipy;
import numpy as np;
import ss_plotting.make_plots as ss;
import matplotlib.pyplot as plt;
import os

datas = dict();
ts = [];
errs = [];

for dirname, dirnames, filenames in os.walk('.'):
    dirnames = sorted(dirnames)
    # print path to all subdirectories first.
    for subdirname in dirnames:
        print subdirname
        datas[subdirname] = np.loadtxt(os.path.join(dirname, subdirname) + "/extrinsic_error.txt")
        ts.append(float(subdirname))
        errs.append(50 * np.sqrt(np.dot(np.transpose(datas[subdirname][7:10]), datas[subdirname][7:10])));
        
    # print path to all filenames.
    #for filename in filenames:
    #    print(os.path.join(dirname, filename))


ss.plot(series=[(ts, errs)], series_colors=['blue'],
series_labels=[''], plot_xlabel="Camera field of view (degrees)", plot_ylabel="Final Extrinsic Error (cm)", savefile="./extrinsic_fov.pdf", savefile_size=(3, 3), y_grid=True,
line_styles=[''], plot_markers=['.']);

##print errs
##plt.plot(ts, errs, 'x')
##plt.show();
