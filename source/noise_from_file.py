import numpy as np
import rospkg, rosbag

from pytransform3d import rotations as pr
import pytransform3d.transformations as pytr

to_rad = np.pi / 180.0

dir_name = '/scene3/'
bag_name = 'fivelids.bag'
odom_name = 'odom'

# read ros packge path
rospack = rospkg.RosPack()
pkg_dir = rospack.get_path('mlcc')
print(pkg_dir)

data_dir = pkg_dir + dir_name
print(data_dir)

gt_fn = data_dir + 'ext.txt'
exts = np.loadtxt(gt_fn, skiprows=2)
print(exts)

# read all odom from the bag
bag_fn = data_dir + bag_name
odom_f = open(data_dir + 'odom.txt', 'w')
bag = rosbag.Bag(bag_fn)
for topic, msg, t in bag.read_messages(topics=[odom_name]):
    # save odom to odom.txt, each pose is xyz wxyz
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    line = '{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n'.format(p.x, p.y, p.z, q.w, q.x, q.y, q.z)
    odom_f.write(line)
odom_f.close()

# read the noise.txt file, first ne lines mean number of extrinsic poses
# then next np lines mean the noise of each pose
noise_fn = data_dir + 'noise.txt'
noise = np.loadtxt(noise_fn)

# read the first line of gt.txt
two_line = []
with open(gt_fn, 'r') as f:
    for i in range(2):
        two_line.append(f.readline())

# add noise to pose ext, each row is a pose: xyzw xyz
rows, cols = exts.shape
wfn = open(data_dir + 'ref.json', 'w')
wfn.writelines(two_line)

for i in range(rows):
    # xyz wxyz
    e = exts[i]
    q = e[3:]
    t = e[:3]
    ns = noise[i]

    ag = pr.axis_angle_from_quaternion(q)
    ag[-1] += ns[0]

    nq = pr.quaternion_from_axis_angle(ag)
    # add noise to translation
    nt = t + ns[1:]

    # xyz wxyz
    line = '{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n'.format(nt[0], nt[1], nt[2],
            nq[0], nq[1], nq[2], nq[3])
    wfn.write(line)

wfn.close()

# add noise to odom
odom = np.loadtxt(data_dir + 'odom.txt')
pose_f = open(data_dir + 'pose.json', 'w')
for i in range(odom.shape[0]):
    # xyz wxyz
    e = odom[i]
    q = e[3:]
    t = e[:3]
    ns = noise[rows + i, :]

    ag = pr.axis_angle_from_quaternion(q)
    ag[-1] += ns[0]

    nq = pr.quaternion_from_axis_angle(ag)
    # add noise to translation
    nt = t + ns[1:]

    # xyz wxyz
    line = '{:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f} {:.6f}\n'.format(nt[0], nt[1], nt[2],
            nq[0], nq[1], nq[2], nq[3])
    pose_f.write(line)
pose_f.close()