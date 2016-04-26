import os
import numpy
import matplotlib.pyplot as plt;
import ss_plotting.make_plots as ss;
import re
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
    

def grep(path, regex):
	regObj = re.compile(regex)
	res = []
	print path
	for root, dirs, fnames in os.walk(path):
		for fname in fnames:
			if (regObj.match(fname)):
				res.append(os.path.join(root, fname))
	return res;

fovs = range(20, 91)
xes = range(0, 71)
reproj_errors = [];
extr_errors = [];
joint_errors = [];
configure_fonts()
for d in fovs:
    reproj_files = grep('./fov_experiments/' + str(d) + '.0', 'reproj_error*');
    extr_files = grep('./fov_experiments/' + str(d) + '.0', 'extrinsic_error_');
    joint_files = grep('./fov_experiments/' + str(d) + '.0', 'joint_error_');
    datas = []
    datas_extr = []
    datas_joints = [];

    for file in reproj_files:
    	data = numpy.loadtxt(file)
    	datas.append(data)

    for file in extr_files:
    	data = numpy.loadtxt(file)
    	datas_extr.append(data)

    for file in joint_files:
        data = numpy.loadtxt(file)
        datas_joints.append(data)

    final_error = numpy.zeros((len(datas), 1))
    final_extr_error = numpy.zeros((len(datas_extr), 1));
    final_joint_error = numpy.zeros((len(datas_extr), 1));

    for l in range(0, len(datas)):
        data = datas[l]
    	final_error[l, :] = data[-1];
    	print data[-1]
    reproj_errors.append(final_error)

    for l in range(0, len(datas_extr)):
        data = datas_extr[l]
    	final_extr_error[l, :] = numpy.sqrt(data[8] ** 2 + data[9] ** 2 + data[10] ** 2) * 100
    extr_errors.append(final_extr_error);
    
    for l in range(0, len(datas_joints)):
        data = datas_joints[l]
        final_joint_error[l, :] = numpy.mean(numpy.abs(data[:, 0:6] - data[:, 6:(2 * 6)])[::10])* (180 / 3.14159);
    joint_errors.append(final_joint_error.flatten())

f1 = plt.figure();
sns.boxplot(data=reproj_errors, color=(0.5, 0.6, 1.0, 0), linewidth=0.5, saturation=0.1, fliersize=1);
#plt.ylim([0, 4])
plt.xticks(xes[::10], fovs[::10])
plt.xlabel('Field of View (degrees)')
plt.ylabel('Median Reprojection Error (pixels)')
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().xaxis.set_ticks_position('none')
plt.gca().yaxis.set_ticks_position('none')

f2 = plt.figure();
sns.boxplot(data=extr_errors, color=(0.5, 0.6, 1.0, 0), linewidth=0.5, saturation=0.1, fliersize=1);
#plt.xticks(xes, fovs)
plt.xlabel('Field of View (degrees)')
plt.ylabel('Extrinsic Error (cm)')
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().xaxis.set_ticks_position('none')
plt.gca().yaxis.set_ticks_position('none')

f3 = plt.figure();
#print joint_errors
sns.boxplot(data=joint_errors, color=(0.5, 0.6, 1.0, 0),linewidth=0.5, saturation=0.1, fliersize=1);
#plt.xticks(xes, fovs)
plt.xlabel('Field of View (degrees)')
plt.ylabel('Absolute Joint Angle Error (Degrees)')
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().xaxis.set_ticks_position('none')
plt.gca().yaxis.set_ticks_position('none')

#plt.bar(range(1, 20), reproj_erros)
#plt.bar(range(1, 20), extr_errors)
plt.show();

f1.tight_layout();
f1.set_size_inches((2.6, 2.6))
f1.savefig('fov_exp_reproj_error.pdf',  pad_inches=0.0, bbox_inches='tight');
f2.tight_layout();
f2.set_size_inches((2.6, 2.6))
f2.savefig('fov_exp_extrinsic_error.pdf',  pad_inches=0.0, bbox_inches='tight');

f3.tight_layout();
f3.set_size_inches((2.6, 2.6))
f3.savefig('fov_exp_joint_error.pdf',  pad_inches=0.0, bbox_inches='tight');
