import scipy;
import numpy as np;
import ss_plotting.make_plots as ss;
import matplotlib.pyplot as plt;

 
data = np.loadtxt("./extrinsic_errors.txt", delimiter = ' ');
plt.plot(data[:, 0:6] - data[:, 7:13]);
plt.show();
rows = data.shape;
# x y z x y z w x' y' z' w' x' y' z'
# 0 1 2 3 4 5 6 7  8  9  10 11 12 13
t = range(0, rows[0]);
ss.plot(series=[(t, -(data[:, 0] - data[:, 7])), (t, -(data[:, 1] - data[:, 8])), (t, -(data[:, 2] - data[:, 9]))], series_colors=['red', 'green', 'blue'],
series_labels=['X Error', 'Y Error', 'Z Error'], plot_xlabel="Iteration", plot_ylabel="Meters", savefile="./extrinsic_error.pdf", savefile_size=(3, 3), y_grid=True);
#plt.yscale('log')
plt.show();

ss.plot(series=[(t, -(data[:, 3] - data[:, 10])), (t, -(data[:, 4] - data[:, 11])), (t, -(data[:, 5] - data[:, 12])), (t, -(data[:, 6] - data[:, 13]))], series_colors=['red', 'green', 'blue', 'black'],
series_labels=['QX Error', 'QY Error', 'QZ Error', 'QW Error'], plot_xlabel="Iteration", plot_ylabel="Radians", savefile="./extrinsic_rot_error.pdf", savefile_size=(3, 3), y_grid=True);
#plt.yscale('log')
plt.show();
