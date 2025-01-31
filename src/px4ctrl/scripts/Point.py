import numpy as np
import matplotlib.pyplot as plt
from pylab import *
from scipy import interpolate

# from test import E

if __name__ == "__main__":
    filename1 = '/home/uav/twf_ws/src/test_wifi/scripts/E3.csv'
    M1 = np.loadtxt(filename1,delimiter=',').reshape(-1, 1)
    filename2 = '/home/uav/twf_ws/src/test_wifi/scripts/X3.csv'
    M2= np.loadtxt(filename2,delimiter=',').reshape(-1, 1)
    filename3 = '/home/uav/twf_ws/src/test_wifi/scripts/Y3.csv'
    M3 = np.loadtxt(filename3,delimiter=',').reshape(-1, 1)
    
    location = np.hstack((M2, M3))

    x_new, y_new = np.meshgrid(np.linspace(-2.8,2.8,200), np.linspace(-2.8,2.8,200)) 

    print(x_new.shape)
    print(x_new)
    
    data = interpolate.griddata(location, M1, (x_new, y_new), method='linear')
    # data = interpolate.griddata(location, M1, (x_new, y_new), method='linear')

    e_new = data[:, :, 0]

    print(e_new.max(), e_new.min())
    im =  plt.imshow(e_new,origin='lower',extent=[-2.8,2.8,-2.8,2.8],cmap=mpl.cm.jet)
    plt.colorbar(im)

    plt.figure()
    plt.scatter(M2, M3, s=20,  c=M1, cmap=mpl.cm.jet)
    # im1 = pl.scatter(M2, M3, M1,origin='lower',extent=[-2.8,2.8,-2.8,2.8],cmap=mpl.cm.jet)
    plt.show()
    # pl.colorbar(im1)
