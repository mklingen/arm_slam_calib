import scipy;
import numpy as np;
import ss_plotting.make_plots as ss;
import matplotlib.pyplot as plt;
import seaborn.apionly as sns

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

 
data = np.loadtxt("./kinematic_error.txt", delimiter = ' ');
#plt.plot(data);
#plt.show();
rows = data.shape;
t = range(0, rows[0]);
errs = np.zeros((rows[0], 2))
t2 = np.linspace(0, 15)
for i in xrange(0, rows[0]):
    errs[i, 0] = np.sqrt(np.dot(data[i, 0:3] - data[i, 3:6], data[i, 0:3] - data[i, 3:6])) * 100
    errs[i, 1] = np.sqrt(np.dot(data[i, 0:3] - data[i, 6:9], data[i, 0:3] - data[i, 6:9])) * 100

errs[rows[0] - 1, 0] = 14
errs[rows[0] - 1, 1] = 14
colors = sns.cubehelix_palette(8, start=2.5, rot=0, dark=0.3, light=1.1, reverse=False, as_cmap=True)
colors.set_under(alpha=0.0)
#print np.mean(errs[:, 0]), np.mean(errs[:, 1])

#ss.plot(series=[(t, errs[:, 1]), 
#                (t, errs[:, 0])], 
#                series_colors=['red', 'blue'], y_grid=True,
#series_labels=['No SLAM', 'With SLAM'], plot_xlabel="Frame", plot_ylabel="Meters", savefile="./kinematic_error.pdf", savefile_size=(4, 3));
#plt.yscale('log')
boxprops = dict(linestyle='-', linewidth=1, color=(0.3, 0.3, 0.3))
flierprops = dict(marker='.', markerfacecolor='black', markersize=1,
              linestyle='none')
whiskerprops = dict(linestyle='-', color=(0.3, 0.3,0.3))
medianprops = dict(linestyle='-', linewidth=1.0, color=(0.3, 0.3, 0.3))
meanpointprops = dict(marker='D', markeredgecolor='black',
                  markerfacecolor=(0,3, 0.3, 0.3))
meanlineprops = dict(linestyle='-', linewidth=1.0, color='black')
f = plt.figure(figsize=(4, 4))
ax = plt.subplot(111)
kde = sns.kdeplot(errs[:,1], errs[:,0], ax=ax, shade=True, n_levels=8, cmap=colors, rug=True)
ax.plot(np.clip(errs[:, 1], 0, 14), [-0.5]*len(errs[:,1]), '|', color=(0, 0.0, 0.0, 0.005), clip_on=False)
ax.plot([-0.5]*len(errs[:,0]), errs[:, 0], '_', color=(0, 0.0, 0.0, 0.005), clip_on=False)
#sns.rugplot(errs[::10,1], color=(0.1, 0.1, 0.1, 0.05), height=0.05, ax=ax)
#sns.rugplot(errs[::10,0], color=(0.1, 0.1, 0.1, 0.05), height=0.05, vertical=True, ax=ax);
#ax.figure.colorbar();
#ax.boxplot([errs[:, 1], errs[:, 0]], labels=['No SLAM', 'With SLAM'], boxprops=boxprops, flierprops=flierprops, medianprops=medianprops, meanprops = meanlineprops, whiskerprops = 
#whiskerprops)
#ax.plot(errs[:,1], errs[:, 0], '.', color='black', alpha=0.01)
ax.plot(t2, t2, color='black', linestyle='--', linewidth=0.5)
ax.set_ylabel("Final Kinematic error (cm)")
ax.set_xlabel("Initial Kinematic error (cm)")
ax.set_xlim((-0.01, 0.15))
ax.set_ylim((-0.01, 0.15))
ax.spines['top'].set_visible(False)
ax.spines['right'].set_visible(False)
#cbar = f.colorbar(kde, ticks=[0, 0.5, 1], orientation='vertical')
#ax.spines['bottom'].set_visible(False)
ax.xaxis.set_ticks_position('none')
ax.yaxis.set_ticks_position('none')
ax.xaxis.set_ticks(np.arange(0, 15, 2))
ax.yaxis.set_ticks(np.arange(0, 15, 2))
configure_fonts();
plt.show();
f.set_size_inches((2, 2))
f.tight_layout();
f.savefig('kinematic_error.pdf', pad_inches=0.0, bbox_inches='tight');
