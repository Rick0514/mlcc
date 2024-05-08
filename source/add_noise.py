import numpy as np
import rospkg

from pytransform3d import rotations as pr
import pytransform3d.transformations as pytr

to_rad = np.pi / 180.0

# read ros packge path
rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('mlcc')
print(pkg_dir)

data_dir = pkg_dir + '/scene3/'
print(data_dir)

gt_fn = data_dir + 'ext.txt'
exts = np.loadtxt(gt_fn, skiprows=1)
print(exts)

# read the first line of gt.txt
two_line = []
with open(gt_fn, 'r') as f:
    for i in range(2):
        two_line.append(f.readline())

# angle noise and trans noise
noise_std = [3, 0.04]

# add noise to pose ext, each row is a pose: xyzw xyz
rows, cols = exts.shape
wfn = open(data_dir + 'ref.json', 'w')
wfn.writelines(two_line)

for i in range(rows):
    e = exts[i]
    q = e[:4]
    t = e[4:]

    ag = pr.axis_angle_from_quaternion(q)
    ag += np.random.normal(0, noise_std[0] * to_rad, 1)

    nq = pr.quaternion_from_axis_angle(ag)
    # add noise to translation
    nt = t + np.random.normal(0, noise_std[1], 3)

    line = '{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}\n'.format(nq[1],
            nq[2], nq[3], nq[0], nt[0], nt[1], nt[2])
    wfn.write(line)

wfn.close()