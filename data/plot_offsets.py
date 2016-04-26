import scipy
import numpy
import matplotlib
import matplotlib.pyplot as plt
import ss_plotting.make_plots as ss;
import matplotlib.pyplot as plt;

def smooth(y, box_pts):
    box = numpy.ones(box_pts)/box_pts
    y_smooth = numpy.convolve(y, box, mode='same')
    return y_smooth

def myfunc(row):
    return numpy.sum(row) > 0.0001

print "Loading...";
data = numpy.loadtxt('./offsets.txt', delimiter=' ')
#data = data[numpy.array([myfunc(row) for row in data])]
m = numpy.max(numpy.abs(data));

t = xrange(1, 200);
ss.plot(series=[(t, data[1:200, 1]), 
                (t, data[1:200, 2])], 
                series_colors=['blue', 'red'], y_grid=True,
series_labels=['$q_2$', '$q_3$'], plot_xlabel="Frame", plot_ylabel="Offset (rad)", savefile="./kinematic_redundancy.pdf", savefile_size=(4, 3));

plt.plot(t, data[1:200, 1],  '-');
plt.plot(t, data[1:200, 2], '-');
plt.show();

f, axarr = plt.subplots(3, 2, sharex=True)


for i in xrange(0, 6):
    ax = axarr[i / 2, i % 2];
    ax.plot(data[:, i], '.', color=(0.6, 0.0, 0.0));
    ax.plot(data[:, i + 6], '.', color=(0.6, 0.6, 0.6));
    #ax.plot(smooth(data[:, i], 9), '-', color=(0, 0, 0));
    ax.set_ylabel('j ' + str(i + 1) + ' offset (rad)');
    ax.set_xlabel('frame')
    ax.set_ylim((-0.1, 0.1))
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.grid(True)
plt.show();
