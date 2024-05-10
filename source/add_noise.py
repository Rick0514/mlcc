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
exts = np.loadtxt(gt_fn, skiprows=2)
print(exts)

# read the first line of gt.txt
two_line = []
with open(gt_fn, 'r') as f:
    for i in range(2):
        two_line.append(f.readline())

# angle noise and trans noise
noise_std = [5, 0.1]

# add noise to pose ext, each row is a pose: xyzw xyz
rows, cols = exts.shape
wfn = open(data_dir + 'ref.json', 'w')
wfn.writelines(two_line)

for i in range(rows):
    # xyz wxyz
    e = exts[i]
    q = e[3:]
    t = e[:3]

    ag = pr.axis_angle_from_quaternion(q)
    print(ag)
    ag[-1] += np.random.normal(0, noise_std[0] * to_rad, 1)
    print(ag)

    nq = pr.quaternion_from_axis_angle(ag)
    # add noise to translation
    nt = t + np.random.normal(0, noise_std[1], 3)

    # xyz wxyz
    line = '{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n'.format(nt[0], nt[1], nt[2],
            nq[0], nq[1], nq[2], nq[3])
    wfn.write(line)

wfn.close()