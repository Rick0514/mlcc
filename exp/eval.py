import numpy as np
import rospkg
import matplotlib.pyplot as plt

from pytransform3d import rotations as pr
import pytransform3d.transformations as pytr

dir_name = '/scene3/'

# read ros packge path
rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('mlcc')
print(pkg_dir)

ext_fn = pkg_dir + dir_name + 'ext.txt'
# read the first line of ext_fn
lidars = open(ext_fn, 'r').readline().strip().split()

# read data of ext.txt and ref_optims.json
exts = np.loadtxt(ext_fn, skiprows=2)
ref_fn = pkg_dir + dir_name + 'ref_optim.json'
refs = np.loadtxt(ref_fn)

# compare ext.txt and ref_optims.json
r, c = exts.shape
logs = []
for i in range(r):
    Text = pytr.transform_from_pq(exts[i])
    Tref = pytr.transform_from_pq(refs[i])

    diff = pytr.invert_transform(Text) @ Tref
    log = pytr.exponential_coordinates_from_transform(diff)
    logs.append(np.linalg.norm(log))

# function to add value labels
def addlabels(x,y):
    for i in range(len(x)):
        plt.text(i, y[i], '{:.4f}'.format(y[i]), ha = 'center')

plt.figure()
# draw bar plot, x axis is lidar topic from idx 1, 2..., y is the norm of log
x = lidars
plt.bar(x, logs, width=0.5, align='center')
addlabels(x, logs)
plt.xlim(-1, len(logs))
plt.grid(True)
plt.title('norm of exponential coordinates')
plt.show()
