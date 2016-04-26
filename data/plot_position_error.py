import scipy;
import numpy as np;
import ss_plotting.make_plots as ss;
import matplotlib.pyplot as plt;

 
data = np.loadtxt("./position_error.txt", delimiter = ' ');
plt.plot(data);
plt.show();
rows = data.shape;
t = range(0, rows[0]);
ss.plot(series=[(t, data[:, 0] - data[:, 3]), (t, data[:, 1] - data[:, 4]), (t, data[:, 2] - data[:, 5])], series_colors=['red', 'green', 'blue'], y_grid=True,
series_labels=['X Error', 'Y Error', 'Z Error'], plot_xlabel="Frame", plot_ylabel="Meters", savefile="./position_error.pdf", savefile_size=(3, 3));
#plt.yscale('log')
plt.show();
