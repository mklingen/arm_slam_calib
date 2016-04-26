import scipy
import numpy
import matplotlib
import matplotlib.pyplot as plt
import seaborn.apionly as sns

def configure_fonts(fontsize=12, legend_fontsize=12,
                      usetex=True, figure_dpi=300):
    """
    Configure fonts. Fonts are set to serif .
    @param fontsize The size of the fonts used on the axis and within the figure
    @param legend_fontsize The size of the legend fonts
    @param usetex If true, configure to use latex for formatting all text
    @param figure_dpi Set the dots per inch for any saved version of the plot
    """
    plt.rcParams.update({
            'font.family':'serif',
            'font.serif':'Computer Modern Roman',
            'font.size': fontsize,
            'legend.fontsize': legend_fontsize,
            'legend.labelspacing': 0,
            'text.usetex': usetex,
            'savefig.dpi': figure_dpi
    })


def smooth(y, box_pts):
    box = numpy.ones(box_pts)/box_pts
    y_smooth = numpy.convolve(y, box, mode='same')
    return y_smooth

def myfunc(row):
    return numpy.sum(row) > 0.0001


def rad2deg(rad):
    return (180 * rad / 3.14159)

def anglediff(a, b):
    d = a - b
    r, = d.shape
    
    for i in xrange(0, r):
        if (d[i] > 180):
            d[i] -= 360
        if (d[i] < -180):
            d[i] += 360
    return numpy.abs(d)

def configure_fonts(fontsize=8, legend_fontsize=8,
                      usetex=True, figure_dpi=300):
    """
    Configure fonts. Fonts are set to serif .
    @param fontsize The size of the fonts used on the axis and within the figure
    @param legend_fontsize The size of the legend fonts
    @param usetex If true, configure to use latex for formatting all text
    @param figure_dpi Set the dots per inch for any saved version of the plot
    """
    plt.rcParams.update({
            'font.family':'serif',
            'font.serif':'Computer Modern Roman',
            'font.size': fontsize,
            'legend.fontsize': legend_fontsize,
            'legend.labelspacing': 0,
            'text.usetex': usetex,
            'savefig.dpi': figure_dpi
    })
    

print "Loading...";
data = numpy.loadtxt('./joint_error.txt', delimiter=' ')
#data = data[numpy.array([myfunc(row) for row in data])]
m = numpy.max(numpy.abs(data));

#plt.plot(data);
#plt.show();


f, axarr = plt.subplots(1, 6, figsize=(8, 2), sharey=True)

t = numpy.linspace(0, 20)
configure_fonts()
colors = sns.cubehelix_palette(8, start=2.5, rot=0, dark=0.3, light=1.05, reverse=False, as_cmap=True)
colors.set_under(alpha=0.0)
print colors
import pandas as pd
for i in xrange(0, 6):
    A = anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 6]));
    B = anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 12]))
    ax = axarr[i];
    #y_noslam, edges_noslam = numpy.histogram(anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 12])), bins = 100)
    #centers_noslam = 0.5 * (edges_noslam[1:] + edges_noslam[:-1])
    #y_slam, edges_slam= numpy.histogram(anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 6])), bins = 100)
    #centers_slam = 0.5 * (edges_slam[1:] + edges_slam[:-1])
    #ax.plot(centers_noslam, y_noslam, '-', color=(0.3, 0.3, 0.3));
    #ax.plot(centers_slam, y_slam, '-', color=(0.7, 0.7, 1.0));
    #n_noslam, bins_noslam, patches_noslam = ax.hist(anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 12])), 50, normed=1, facecolor=(0.3, 0.3, 0.3));
    #n_slam, bins_slam, patches_slam = ax.hist(anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 6])), 50, normed=1, facecolor=(0.7, 0.7, 1.0));
    #ax.plot(anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 12])), '.', color=(0.3, 0.3, 0.3));
    #ax.plot(anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 6])), '.', color=(0.7, 0.7, 1.0));
    #ax.plot(data[:, i + 6], '-', color=(0.8, 0.8, 1.0));
    #boxprops = dict(linestyle='-', linewidth=1, color=(0.3, 0.3, 0.3))
    #flierprops = dict(marker='.', markerfacecolor='black', markersize=1,
    #              linestyle='none')
    #whiskerprops = dict(linestyle='-', color=(0.3, 0.3,0.3))
    #medianprops = dict(linestyle='-', linewidth=1.0, color=(0.3, 0.3, 0.3))
    #meanpointprops = dict(marker='D', markeredgecolor='black',
    #                  markerfacecolor=(0,3, 0.3, 0.3))
    #meanlineprops = dict(linestyle='-', linewidth=1.0, color='black')
    #ax.boxplot( [anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 12])), anglediff(rad2deg(data[:, i]), rad2deg(data[:, i + 6]))], labels=['No SLAM', 'With SLAM'], boxprops=boxprops, flierprops=flierprops, medianprops=medianprops, meanprops = meanlineprops, whiskerprops = whiskerprops)
    #ax.boxplot()
    sns.kdeplot(B, A, ax=ax, shade=True, n_levels=8, cmap=colors)
    ax.plot(numpy.clip(B[::10], 0, 12), [-0.5]*(len(B[::10])), '|', color=(0, 0.0, 0.0, 0.01), clip_on=False)
    ax.plot([-0.5]*(len(A[::10])), A[::10], '_', color=(0, 0.0, 0.0, 0.01), clip_on=False)
    #ax.plot(B, A, '.', color='blue', alpha=0.01)
    ax.plot(t, t, '--', color='black', linewidth=0.5)
    ax.set_xlabel('Joint ' + str(i + 1) + ' initial error');
    
    if (i == 0):
        ax.set_ylabel('Final error (degrees)');
    #ax.set_xlabel('error (degrees)')
    ax.set_ylim((0, 12))
    ax.set_xlim((0, 12))
    ax.xaxis.set_ticks(numpy.arange(0, 13, 2))
    ax.yaxis.set_ticks(numpy.arange(0, 13, 2))
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    #ax.spines['bottom'].set_visible(False)
    ax.xaxis.set_ticks_position('none')
    ax.yaxis.set_ticks_position('none')
    ax.set(adjustable='box-forced', aspect='equal')
    #ax.grid(True, color=(0.8, 0.8, 0.8))
f.tight_layout();
plt.show();
f.set_size_inches((7, 3.5))
f.savefig('joint_errors.pdf',  pad_inches=0.0, bbox_inches='tight');
