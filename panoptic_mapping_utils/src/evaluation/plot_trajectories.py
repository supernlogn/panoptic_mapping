import json
import numpy as np
from mpl_toolkits import mplot3d  # pylint: disable=W0611
from matplotlib import pyplot as plt


def getGroundTruthTrajectory(file_path):
    ret1 = []
    ret2 = []
    ret3 = []
    with open(file_path, 'r') as fr:
        lines = fr.readlines()
        for line in lines:
            d = json.loads(line)
            p = d['original']['translation']
            p2 = d['noisy']['translation']
            t = float(d['time'])
            ret1.append(p)
            ret2.append(p2)
            ret3.append(t)
    return np.array(ret1), np.array(ret2), np.array(ret3)


def getOptimizedTrajectory(file_path):
    ret = []
    ret2 = []
    with open(file_path, 'r') as fr:
        num_points = int(fr.readline())
        for _ in range(num_points):
            t0 = fr.readline()
            t1 = fr.readline()
            t2 = fr.readline()
            t3 = fr.readline()
            fr.readline()  # index
            time = float(fr.readline())
            x = float(t0[t0.rfind(' '):])
            y = float(t1[t1.rfind(' '):])
            z = float(t2[t2.rfind(' '):])
            w = float(t3[t3.rfind(' '):])
            ret.append([x, y, z, w])
            ret2.append(time)
    return np.array(ret), np.array(ret2)


def main():
    gt_tr, wd_tr, _ = getGroundTruthTrajectory(
        "/home/ioannis/catkin_ws/src/panoptic_mapping/"
        "panoptic_mapping_utils/generated_path.txt")
    op_tr, _ = getOptimizedTrajectory("/home/ioannis/datasets/trajectory.in")
    print(len(gt_tr), len(op_tr))
    _ = plt.figure()
    ax = plt.axes(projection='3d')
    ax.plot3D(wd_tr[:, 0], wd_tr[:, 1], wd_tr[:, 2], 'gray')
    ax.plot3D(gt_tr[:, 0], gt_tr[:, 1], gt_tr[:, 2], 'green')
    ax.plot3D(op_tr[:, 0], op_tr[:, 1], op_tr[:, 2], 'red')
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    plt.show()


if __name__ == "__main__":
    main()
