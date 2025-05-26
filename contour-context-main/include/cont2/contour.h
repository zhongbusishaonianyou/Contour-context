//
// Created by lewis on 5/5/22.
//

/*
 * Slice actually...
 * */

#ifndef CONT2_CONTOUR_H
#define CONT2_CONTOUR_H

#include <utility>
#include <vector>
#include <iostream>

#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <opencv2/core/types.hpp>

#include "tools/algos.h"

typedef Eigen::Matrix<float, 2, 1> V2F;
typedef Eigen::Matrix<float, 2, 2> M2F;
typedef Eigen::Matrix<double, 2, 1> V2D;
typedef Eigen::Matrix<double, 2, 2> M2D;

struct ContourViewStatConfig
{
  int16_t min_cell_cov = 4;
  float point_sigma = 1.0;    // have nothing to do with resolution: on pixel only
  float com_bias_thres = 0.5; // com dist from geometric center
  //  int half_strip_num_ = 4;
};

// The configuration for checking the similarity of two contours
struct ContourSimThresConfig
{
  float ta_cell_cnt ,tp_cell_cnt ;
  float tp_eigval;
  float ta_h_bar; 
  float ta_rcom,tp_rcom ;
};

struct RunningStatRecorder
{
  int16_t cell_cnt_{}; //number of cells in this contour
  V2D cell_pos_sum_;  // sum of cell positions
  M2D cell_pos_tss_;  // cell_pos_sum_*cell_pos_sum_.transpose() 
  float cell_vol3_{}; // sum of height
  V2D cell_vol3_torq_; // height * cell_pos_sum_

  RunningStatRecorder()
  {
    cell_pos_sum_.setZero();
    cell_pos_tss_.setZero();
    cell_vol3_torq_.setZero();
  }

  void runningStatsF(float curr_row, float curr_col, float height)
  { // a more accurate one with continuous coordinate
    DCHECK_GE(curr_row, -0.5f);
    DCHECK_GE(curr_col, -0.5f);
    cell_cnt_ += 1;
    V2D v_rc(curr_row, curr_col);
    cell_pos_sum_ += v_rc;
    cell_pos_tss_ += v_rc * v_rc.transpose();
    cell_vol3_ += height;
    cell_vol3_torq_ += height * v_rc;
  }

  static RunningStatRecorder addContourStat(const RunningStatRecorder &rec1, const RunningStatRecorder &rec2)
  {
    RunningStatRecorder res;
    res.cell_cnt_ = rec1.cell_cnt_ + rec2.cell_cnt_;
    res.cell_pos_sum_ = rec1.cell_pos_sum_ + rec2.cell_pos_sum_;
    res.cell_pos_tss_ = rec1.cell_pos_tss_ + rec2.cell_pos_tss_;
    res.cell_vol3_ = rec1.cell_vol3_ + rec2.cell_vol3_;
    res.cell_vol3_torq_ = rec1.cell_vol3_torq_ + rec2.cell_vol3_torq_;
    return res;
  }
};

struct ContourView
{
  // Coordinate definition:
  //  row as x, col as y, center of pixel(0,0) as origin.

  int16_t level_;
  int16_t poi_[2]; // a point in full bev coordinate belonging to this contour/slice.

  int16_t cell_cnt_{};
  V2F pos_mean_;
  M2F pos_cov_;
  V2F eig_vals_;
  M2F eig_vecs_;  // gaussian ellipsoid axes. if ecc_feat_==false, this is meaningless
  float eccen_{}; // 0: circle
  float vol3_mean_{};
  V2F com_;               // center of mass
  bool ecc_feat_ = false; // eccentricity large enough (with enough cell count)
  bool com_feat_ = false; // com not at fitted geometric center

  explicit ContourView(int16_t level, int16_t poi_r, int16_t poi_c) : level_(level)
  {
    DCHECK_GE(poi_r, 0);
    DCHECK_GE(poi_c, 0);
    poi_[0] = poi_r;
    poi_[1] = poi_c;
  };

  ContourView(const ContourView &obj) = default;

  // TO-DO: 2. calculate statistics from running data (including feature hypothesis)
  void calcStatVals(const RunningStatRecorder &rec, const ContourViewStatConfig &cfg)
  {
    cell_cnt_ = rec.cell_cnt_;
    pos_mean_ = rec.cell_pos_sum_.cast<float>() / cell_cnt_;

    vol3_mean_ = rec.cell_vol3_ / cell_cnt_;
    com_ = rec.cell_vol3_torq_.cast<float>() / rec.cell_vol3_;
    // eccentricity:
    if (cell_cnt_ < cfg.min_cell_cov)
    {
      pos_cov_ = M2F::Identity() * cfg.point_sigma * cfg.point_sigma;
      eig_vals_ = V2F(cfg.point_sigma, cfg.point_sigma);
      eig_vecs_.setIdentity();
      ecc_feat_ = false;
      com_feat_ = false;
      
    }
    else
    {
      pos_cov_ = (rec.cell_pos_tss_.cast<float>() - pos_mean_ * pos_mean_.transpose() * cell_cnt_) / (cell_cnt_ - 1); // simplified, verified
      
      Eigen::SelfAdjointEigenSolver<M2F> es(pos_cov_.template selfadjointView<Eigen::Upper>());
      eig_vals_ = es.eigenvalues();       // increasing order
      
      if (eig_vals_(0) < cfg.point_sigma) // determine if eccentricity feat using another function
        eig_vals_(0) = cfg.point_sigma;
      if (eig_vals_(1) < cfg.point_sigma)
        eig_vals_(1) = cfg.point_sigma;
      eccen_ = std::sqrt(eig_vals_(1) * eig_vals_(1) - eig_vals_(0) * eig_vals_(0)) / eig_vals_(1);
      eig_vecs_ = es.eigenvectors();

      ecc_feat_ = eccentricitySalient(cfg);

      // vol/weight of mountain:
      com_feat_ = centerOfMassSalient(cfg);

    }
  }

  inline bool eccentricitySalient(const ContourViewStatConfig &cfg) const
  {
    return cell_cnt_ > 5 && diff_perc<float>(eig_vals_(0), eig_vals_(1), 0.2f) && eig_vals_(1) > 2.5f;
  }

  // TODO: should have sth to do with total area
  inline bool centerOfMassSalient(const ContourViewStatConfig &cfg) const
  {
    return (com_ - pos_mean_).norm() > cfg.com_bias_thres;
  }

  // TODO
  bool orietSalient(const ContourViewStatConfig &cfg) const
  {
    return false;
  }
  
  /**
   * @description: check the similarity of two Anchor CA
   * @param cont_src: source contour
   * @param cont_tgt: target contour
   * @param simthres: similarity threshold configuration
   * @return: true if the two contours are similar enough
   */

  static bool checkSim(const ContourView &cont_src, const ContourView &cont_tgt, const ContourSimThresConfig &simthres)
  {

    bool ret = false;
    // 1. area, 2.3. eig, 4. com;
    //第一个特征：联通区域数量相似度对比（利用论文公式（4）和差绝对值）
    if (diff_perc<float>(cont_src.cell_cnt_, cont_tgt.cell_cnt_, simthres.tp_cell_cnt)  && 
        diff_delt<float>(cont_src.cell_cnt_, cont_tgt.cell_cnt_, simthres.ta_cell_cnt))
    {
#if HUMAN_READABLE
      printf("\tCell cnt not pass.\n");
#endif
      return ret;
    }
    //第二个特征：次轴长度相似度对比（利用论文公式（4）） 
    if (std::max(cont_src.eig_vals_(1), cont_tgt.eig_vals_(1)) > 2.0 &&
        diff_perc<float>(std::sqrt(cont_src.eig_vals_(1)), std::sqrt(cont_tgt.eig_vals_(1)), simthres.tp_eigval))
    {
#if HUMAN_READABLE
      printf("\tBig eigval not pass.\n");
#endif
      return ret;
    }
    //第三个特征：主轴长度相似度对比（利用论文公式（4））
    if (std::max(cont_src.eig_vals_(0), cont_tgt.eig_vals_(0)) > 2.0 &&
        diff_perc<float>(std::sqrt(cont_src.eig_vals_(0)), std::sqrt(cont_tgt.eig_vals_(0)), simthres.tp_eigval))
    {
#if HUMAN_READABLE
      printf("\tSmall eigval not pass.\n");
#endif
      return ret;
    }
    //第四个特征：平均高度相似度对比（利用论文公式（4））
    if (std::max(cont_src.cell_cnt_, cont_tgt.cell_cnt_) > 15 &&
        diff_delt<float>(cont_src.vol3_mean_, cont_tgt.vol3_mean_, simthres.ta_h_bar))
    {
#if HUMAN_READABLE
      printf("\tAvg height not pass.\n");
#endif
      return ret;
    }
    //第五个特征：几何中心和质心的距离范数
    const float com_r1 = (cont_src.com_ - cont_src.pos_mean_).norm();
    const float com_r2 = (cont_tgt.com_ - cont_tgt.pos_mean_).norm();
    if (diff_delt<float>(com_r1, com_r2, simthres.ta_rcom) && diff_perc<float>(com_r1, com_r2, simthres.tp_rcom))
    {
#if HUMAN_READABLE
      printf("\tCom radius not pass.\n");
#endif
      return ret;
    }

    ret = true;
    return ret;
  }

  // TODO: 4. add two contours. Only statistical parts are useful

  // Add contour results (**NOT** accurate statistics!)
  // procedure: revert to stat recorder and merge
  static ContourView addContourRes(const ContourView &cont1, const ContourView &cont2, const ContourViewStatConfig &cfg)
  {
    CHECK_EQ(cont1.level_, cont2.level_);
    RunningStatRecorder media;
    media.cell_cnt_ = cont1.cell_cnt_ + cont2.cell_cnt_;
    media.cell_pos_sum_ = (cont1.cell_cnt_ * cont1.pos_mean_ + cont2.cell_cnt_ * cont2.pos_mean_).cast<double>();
    media.cell_vol3_ = cont1.cell_cnt_ * cont1.vol3_mean_ + cont2.cell_cnt_ * cont2.vol3_mean_;
    media.cell_vol3_torq_ = 
         (cont1.com_ * (cont1.cell_cnt_ * cont1.vol3_mean_) + cont2.com_ * (cont2.cell_cnt_ * cont2.vol3_mean_)).cast<double>();
    media.cell_pos_tss_ =
        (cont1.pos_cov_ * (cont1.cell_cnt_ - 1) + cont1.cell_cnt_ * cont1.pos_mean_ * cont1.pos_mean_.transpose()
         + cont2.pos_cov_ * (cont2.cell_cnt_ - 1) + cont2.cell_cnt_ * cont2.pos_mean_ * cont2.pos_mean_.transpose()).cast<double>();

    ContourView res(cont1.level_, cont1.poi_[0], cont1.poi_[1]);
    res.calcStatVals(media, cfg);

    return res;
  }

  // getter setter
  //  int getArea() const {
  //    return cell_cnt_;
  //  }

  //  void addChildren(std::shared_ptr<ContourView> &chd) {
  //    children_.push_back(chd);
  //  }

  // auxiliary functions
  // 1. get the position of all contour pixels
  std::vector<std::vector<int>> getContPixelPos() const
  {
    return {};
  }

  //协方差矩阵重构
  inline M2F getManualCov() const
  {
    return eig_vecs_ * eig_vals_.asDiagonal() * eig_vecs_.transpose();
  }
};

#endif // CONT2_CONTOUR_H
