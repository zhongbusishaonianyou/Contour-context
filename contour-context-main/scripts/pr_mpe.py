import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import matplotlib
import math
from scipy.spatial import KDTree
from matplotlib.font_manager import FontProperties
font = FontProperties(fname='/usr/share/fonts/truetype/msttcorefonts/Times_New_Roman.ttf', size=20)
from functools import reduce

thres_dist = 5.0
num_candidates = 300

def get_gt_sens_poses(fpath_gt_sens_poses):
    """

    :return: 2d matrix, each row is a 12 dim elements
    """

    with open(fpath_gt_sens_poses, "r") as fp:
        lines = fp.readlines()

    res = []
    for line in lines:
        assert len(line.strip().split()) == 13
        res.append([eval(x) for x in line.strip().split()[1:]])

    return np.vstack(res)


def get_maxf1_idx(data):
    max_f1 = 0
    idx = -1
    max_pt = None
    for d in data:
        cur = 2 * d[0] * d[1] / (d[0] + d[1]) if (d[0] + d[1]) > 0 else 0

        if max_f1 < cur:
            max_f1 = cur
            idx = d[2]
            max_pt = d
    print("Max f1 point: ", max_pt)
    return max_f1, idx


class SimpleRMSE:
    def __init__(self):
        self.sum_sqs = 0
        self.sum_abs = 0
        self.cnt_sqs = 0

    def add_one_error(self, err_vec):
        self.cnt_sqs += 1
        tmp = 0
        for i in err_vec:
            tmp += i ** 2
        self.sum_sqs += tmp
        self.sum_abs += math.sqrt(tmp)

    def get_rmse(self):
        if self.cnt_sqs:
            return math.sqrt(self.sum_sqs / self.cnt_sqs)
        else:
            return -1

    def get_mean(self):
        if self.cnt_sqs:
            return self.sum_abs / self.cnt_sqs
        else:
            return -1


def get_points_ours2(fp_gt_sens_poses, fp_outcome):
    
    plots_data = []

    print(fp_gt_sens_poses)
    print(fp_outcome)
    pr_points = []

    gt_pose = get_gt_sens_poses(fp_gt_sens_poses)  # the sensor poses must be ordered by time/creation/acquisition
    gt_positive = np.zeros(gt_pose.shape[0])
    gt_points = gt_pose[:, [3, 7, 11]]
    tree = KDTree(gt_points)
    N=0
    for i in range(gt_pose.shape[0]):
        near_points = tree.query_ball_point(gt_points[i, :], thres_dist)
        for j in near_points:
            if j < i - num_candidates:
                gt_positive[i] = 1
                N=N+1
                break
    print('the number of revisit events:',N)
    with open(fp_outcome, "r") as f1:
        lines = f1.readlines()
        est = []
        index=-1;
        loop_results=np.zeros((len(lines), 4))
        for line in lines:
            index   += 1     
            line_info = line.strip().split()
            assert len(line_info) > 3

            pairing = line_info[1].split('-')
            idx_curr = int(pairing[0])

            est_line = [eval(line_info[2]), 0, 0, idx_curr]
            if pairing[1] != 'x':
                idx_best = int(pairing[1])
                if np.linalg.norm(gt_pose[idx_curr].reshape(3, 4)[:, 3] -
                                  gt_pose[idx_best].reshape(3, 4)[:, 3]) < thres_dist:
                    est_line[1] = 1
            else :
                idx_best = -1    
                # 3. if the overall is P
            est_line[2] = gt_positive[idx_curr]
            loop_results[index,:]=(idx_curr,idx_best, line_info[2],est_line[1]); 
            est.append(est_line)
            

        orig_est = est

        est = np.vstack(est)
        est = est[(-est[:, 0]).argsort()]  # sort by correlation, larger better
        
        TP = 0
        FP = 0
        
        for i in range(est.shape[0]):
            if est[i, 1]:
                TP += 1
            else:
                FP += 1
            
            pr_points.append([TP / N, TP / (TP + FP), est[i, 3]])
        

        points = np.vstack(pr_points)[:, 0:2]
        points = points[points[:, 0].argsort()]
        plots_data.append(points)
        
        th0 = []
        PRE = []
        REC= []
        F1_score=[]
        min_similarity = loop_results[:, 2].min()
        max_similarity = loop_results[:, 2].max()

        for i in np.arange(max_similarity - (max_similarity-min_similarity) * 1.0 /100,min_similarity,  -(max_similarity-min_similarity) * 1.0 /100):
            TP = 0 
            P = 0
            for j in range(0, loop_results.shape[0]):
                if loop_results[j][2] >= i:
                    P = P+1
                    if loop_results[j][3] == 1: 
                       TP = TP+1
            
            Recall = TP * 1.0 / N
            Precision = TP * 1.0 / P
            th0.append(i)
            REC.append(Recall)
            PRE.append(Precision)
            F1_score.append(2*Recall*Precision/(Recall+Precision))
        print('F1 max score:',max(F1_score))  

        # get max F1
        max_f1, f1_pose_idx = get_maxf1_idx(pr_points)
        print("Max F1 score: %f @%d " % (max_f1, int(f1_pose_idx)))

        # calc rmse for scores above max f1 sim
        sim_thres = eval(lines[int(f1_pose_idx)].split()[2])
        print("sim thres for Max F1 score: %f" % sim_thres)

        sr_trans = SimpleRMSE()
        sr_rot = SimpleRMSE()
        for i, line in enumerate(lines):
            line_info = line.strip().split()
            assert len(line_info) > 5

            # if current is TP
            if eval(line_info[2]) >= sim_thres and orig_est[i][1] == 1 and orig_est[i][2] == 1:
                sr_trans.add_one_error([eval(line_info[3]), eval(line_info[4])])
                sr_rot.add_one_error([eval(line_info[5]), ])

        print("TP count: ", sr_rot.cnt_sqs)
        print("Rot mean err: ", sr_rot.get_mean() / np.pi * 180)
        print("Rot rmse    : ", sr_rot.get_rmse() / np.pi * 180)
        print("Trans mean err: ", sr_trans.get_mean())
        print("Trans rmse    :  ", sr_trans.get_rmse())
    
    return plots_data,gt_pose,loop_results,sim_thres,gt_positive,REC,PRE

def main(fp_gt_sens_poses, fp_outcome):
    
    data_res,gt_poses ,loop_results,sim_thres,gt,REC,PRE=  get_points_ours2(fp_gt_sens_poses, fp_outcome)
    
    x_cord = -gt_poses[:,7]
    z_cord = gt_poses[:,3]

    plt.figure(1) 

    plt.xlabel('Recall', color='b', fontproperties=font, fontstyle='normal')
    plt.ylabel('Precision',color='b',fontproperties=font, fontstyle='normal')
    plt.tick_params(labelsize=24)
    plt.plot(REC,PRE, "r-o",label = "contour context", linewidth=3.0)
    plt.legend(loc="lower left", prop=font)
    new_ticks=np.linspace(0,1,11)
    plt.xticks(new_ticks,fontproperties=font)
    plt.yticks(new_ticks,fontproperties=font)
    plt.title('Precision-Recall Curve', fontproperties=font)

    ax=plt.gca()
    ax.yaxis.grid(color='black', linestyle='-', linewidth=1,alpha=0.1)
    ax.xaxis.grid(color='black', linestyle='-', linewidth=1,alpha=0.1)

    plt.figure(2)
    plt.title('Trajectory truth and results',fontproperties=font)
    plt.xlabel('x', fontproperties=font, fontstyle='normal')
    plt.ylabel('z',fontproperties=font, fontstyle='normal')
    plt.tick_params(labelsize=18)
    plt.xticks(fontproperties=font)
    plt.yticks(fontproperties=font)
    plt.plot(x_cord, z_cord,  "k", linewidth=1.5)
    
    for i in range(len(loop_results[:,0])):
      if gt[int(loop_results[i][0])]:
        index = int(int(loop_results[i][0]))
        plt.scatter(x_cord[index], z_cord[index], c="g",alpha=0.2)
      if loop_results[i][2] >= sim_thres and gt[int(loop_results[i][0])] == 1:
        index = int(loop_results[i][0])
        plt.scatter(x_cord[index], z_cord[index], c="r")
      if loop_results[i][2] > =sim_thres and gt[int(loop_results[i][0])] == 0:
        index = int(loop_results[i][0])
        plt.scatter(x_cord[index], z_cord[index], c="b")

    plt.show()

if __name__ == "__main__":
    file_gt_sens_poses = "../sample_data/ts-sens_pose-kitti05.txt"
    file_outcome = "../results/outcome_txt/outcome-kitti05.txt"
    main(file_gt_sens_poses, file_outcome)
