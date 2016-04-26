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

dofs = range(1, 20)
xes = range(0, 19)
reproj_errors = [];
extr_errors = [];
joint_errors = [];
configure_fonts()
for d in dofs:
    reproj_files = grep('./dof_experiments/' + str(d), 'reproj_error*');
    extr_files = grep('./dof_experiments/' + str(d), 'extrinsic_error_');
    joint_files = grep('./dof_experiments/' + str(d), 'joint_error_');
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
    final_joint_error = numpy.zeros(((len(datas_extr) * d), 1))

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
        final_joint_error[l * d : l * d + d, :] = numpy.median(numpy.abs(data[:, 0:d] - data[:, d:(2 * d)]))* (180 / 3.14159);
    joint_errors.append(final_joint_error.flatten())

f1 = plt.figure();
sns.boxplot(data=reproj_errors, color=(0.5, 0.6, 1.0, 0), linewidth=0.5, saturation=0.1, fliersize=1);
plt.ylim([0, 8])
plt.xticks(xes, dofs)
plt.xlabel('\# Degrees of Freedom')
plt.ylabel('Median Reprojection Error (pixels)')
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().xaxis.set_ticks_position('none')
plt.gca().yaxis.set_ticks_position('none')

f2 = plt.figure();
sns.boxplot(data=extr_errors, color=(0.5, 0.6, 1.0, 0), linewidth=0.5, saturation=0.1, fliersize=1);
plt.xticks(xes, dofs)
plt.xlabel('\# Degrees of Freedom')
plt.ylabel('Extrinsic Error (cm)')
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().xaxis.set_ticks_position('none')
plt.gca().yaxis.set_ticks_position('none')

f3 = plt.figure();
#print joint_errors
sns.boxplot(data=joint_errors, color=(0.5, 0.6, 1.0, 0),linewidth=0.5, saturation=0.1, fliersize=1);
plt.xticks(xes, dofs)
plt.xlabel('\# Degrees of Freedom')
plt.ylabel('Median Abs Joint Angle Error (Degrees)')
plt.gca().spines['top'].set_visible(False)
plt.gca().spines['right'].set_visible(False)
plt.gca().xaxis.set_ticks_position('none')
plt.gca().yaxis.set_ticks_position('none')

#plt.bar(range(1, 20), reproj_erros)
#plt.bar(range(1, 20), extr_errors)
plt.show();

f1.tight_layout();
f1.set_size_inches((2.6, 2.6))
f1.savefig('dof_exp_reproj_error.pdf',  pad_inches=0.0, bbox_inches='tight');
f2.tight_layout();
f2.set_size_inches((2.6, 2.6))
f2.savefig('dof_exp_extrinsic_error.pdf',  pad_inches=0.0, bbox_inches='tight');

f3.tight_layout();
f3.set_size_inches((2.6, 2.6))
f3.savefig('dof_exp_joint_error.pdf',  pad_inches=0.0, bbox_inches='tight');
