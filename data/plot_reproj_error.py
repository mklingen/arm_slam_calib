import scipy;
import numpy as np;
import ss_plotting.make_plots as ss;
import matplotlib.pyplot as plt;

 
data = np.loadtxt("./reproj_error.txt", delimiter = ' ');
plt.plot(data);
plt.show();
rows = data.shape;
t = range(0, rows[0]);
ss.plot(series=[(t, data)], series_colors=['blue'], y_grid=True, 
series_labels=[''], 
plot_xlabel="Iterations", plot_ylabel=" Median Reprojection Error (Pixels)",
 savefile="./reproj_error.pdf", savefile_size=(2, 2));
#plt.yscale('log')
plt.show();
