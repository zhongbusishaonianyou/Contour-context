//
// Created by lewis on 5/5/22.
//

#ifndef CONT2_CONTOUR_MNG_H
#define CONT2_CONTOUR_MNG_H

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <bitset>
#include <set>
#include <map>
#include <string>
#include "cont2/contour.h"
#include "tools/algos.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

//// For SURF:
// #include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"

#include <utility>

#include "tools/bm_util.h"
#include "tools/algos.h"

using KeyFloatType = float; // retrieval key's float number type
// using RetrievalKey = Eigen::Matrix<KeyFloatType, 5, 1>;
// size_t sz 表示模板参数 ,它接受一个 具体的整数值 sz 是一个 编译时常量，且必须是非负整数（因为 size_t 是无符号的）。
template <size_t sz>
struct ArrayAsKey
{
  enum
  {
    SizeAtCompileTime = sz
  };
  //  static constexpr size_t SizeAtCompileTime = sz;  // undefined reference when linking
  KeyFloatType array[sz]{};

  KeyFloatType *data()
  {
    return array;
  }
  // 提供了两种访问数组元素的方式（）[]
  KeyFloatType &operator()(size_t i) { return array[i]; }

  const KeyFloatType &operator()(size_t i) const { return array[i]; }

  KeyFloatType &operator[](size_t i) { return array[i]; }

  const KeyFloatType &operator[](size_t i) const { return array[i]; }
  // 减法运算符重载
  ArrayAsKey<sz> operator-(ArrayAsKey<sz> const &obj) const
  {
    ArrayAsKey<sz> res;
    for (int i = 0; i < sz; i++)
      res.array[i] = array[i] - obj.array[i];
    return res;
  }

  void setZero()
  {
    std::fill(array, array + SizeAtCompileTime, KeyFloatType(0));
  }
  // 返回固定数组大小
  size_t size() const
  {
    return sz;
  }

  KeyFloatType sum() const
  {
    KeyFloatType ret(0);
    for (const auto &dat : array)
      ret += dat;
    return ret;
  }

  KeyFloatType squaredNorm() const
  {
    KeyFloatType ret(0);
    for (const auto &dat : array)
      ret += dat * dat;
    return ret;
  }
};

const int RET_KEY_DIM = 10;
using RetrievalKey = ArrayAsKey<RET_KEY_DIM>;

struct ContourManagerConfig
{
  std::vector<float> lv_grads_; 

  float reso_row_ ,reso_col_ ;
  int n_row_ , n_col_ ;

  float lidar_height_; // ground assumption
  float blind_sq_;

  int min_cont_key_cnt_ ;  // minimal the cell count to calculate a valid key around an anchor contour
  int min_cont_cell_cnt_; // the minimal cell cnt to consider creating a contour

  int piv_firsts_;       // the top x contours to be treated as anchor CAs
  int dist_firsts_;     // the top x contours to be treated as peripheral CAs
  float roi_radius_; // RoI radius around the center of anchor
};

const int16_t BITS_PER_LAYER = 64;
// the layers for generating the dist key and forming the constellation
const int8_t DIST_BIN_LAYERS[] = {1, 2, 3, 4};
// weights for each layer when calculating a normalized "used area percentage"
const float LAYER_AREA_WEIGHTS[] = {0.3, 0.3, 0.3, 0.1}; 
const int16_t NUM_BIN_KEY_LAYER = sizeof(DIST_BIN_LAYERS) / sizeof(int8_t);

// scores for checks
// i: int, typically for counts. f: float, just represented in int.
union ScoreConstellSim
{
  enum
  {
    SizeAtCompileTime = 3
  };
  int data[SizeAtCompileTime]{};
  struct
  {
    int i_ovlp_sum;
    int i_ovlp_max_one;
    int i_in_ang_rng;
  };

  inline const int &overall() const
  {
    // Manually select a thres in the check as the score for the overall check to pass.
    return i_in_ang_rng;
  }

  inline int cnt() const
  {
    return i_in_ang_rng;
  }

  void print() const
  {
    printf("%d, %d, %d;", i_ovlp_sum, i_ovlp_max_one, i_in_ang_rng);
  }

  bool strictSmaller(const ScoreConstellSim &b) const
  {
    for (int i = 0; i < SizeAtCompileTime; i++)
    {
      if (data[i] >= b.data[i])
        return false;
    }
    return true;
  }
};

union ScorePairwiseSim
{
  enum
  {
    SizeAtCompileTime = 2
  };
  int data[SizeAtCompileTime]{};
  struct
  {
    int i_indiv_sim;
    int i_orie_sim;
    //    int f_area_perc;  // the area score is a weighted sum of used area percentage of all concerning levels, normalized to int(100)
  };

  inline const int &overall() const
  {
    // Manually select a thres in the check as the min requirement for the overall check to pass.
    return i_orie_sim;
    //    return f_area_perc;  // which has the final say?
  }

  inline int cnt() const
  {
    return i_orie_sim;
  }

  void print() const
  {
    printf("%d, %d;", i_indiv_sim, i_orie_sim);
  }

  bool strictSmaller(const ScorePairwiseSim &b) const
  {
    for (int i = 0; i < SizeAtCompileTime; i++)
    {
      if (data[i] >= b.data[i])
        return false;
    }
    return true;
  }
};

union ScorePostProc
{
  enum
  {
    SizeAtCompileTime = 3
  };
  float data[SizeAtCompileTime]{};

  struct
  {
    float correlation;
    float area_perc;
    float neg_est_dist; // 2D trans distance. Record as the negated distance (since the larger the better)
  };

  inline const float &overall() const
  {
    return correlation;
  }

  //  inline int cnt() const {
  //    return 0;
  //  }

  void print() const
  {
    printf("%6f, %6f%%, %6fm;", correlation, 100 * area_perc, neg_est_dist);
  }

  bool strictSmaller(const ScorePostProc &b) const
  {
    for (int i = 0; i < SizeAtCompileTime; i++)
    {
      if (data[i] >= b.data[i])
        return false;
    }
    return true;
  }
};

union ConstellationPair
{ // given a pair of ContourManager, this records the seq of 2 "matched" contours at a certain level
  struct
  {
    int8_t level;
    int8_t seq_src;
    int8_t seq_tgt;
  };
  int data[1]{};

  ConstellationPair(int8_t l, int8_t s, int8_t t) : level(l), seq_src(s), seq_tgt(t) {}

  bool operator<(const ConstellationPair &a) const
  {
    return level < a.level || (level == a.level && seq_src < a.seq_src) ||
           (level == a.level && seq_src == a.seq_src && seq_tgt < a.seq_tgt);
  }

  bool operator==(const ConstellationPair &a) const
  {
    return level == a.level && seq_src == a.seq_src && seq_tgt == a.seq_tgt; // why not just compare data[0]?
  }
};

//! binary constellation identity
struct BCI
{ // binary constellation identity
  //! a point/star of the constellation seen from an anchor contour
  union RelativePoint
  {
    struct
    {
      int8_t level;
      int8_t seq;
      int16_t bit_pos;
      float r;
      float theta;
    };
    int data[3]{};

    RelativePoint(int8_t l, int8_t a, int16_t b, float f1, float f2) : level(l), seq(a), bit_pos(b), r(f1), theta(f2) {}

    //    RelativePoint() = default;
  };

  //! potential pairs from 2 constellations that passed the dist check
  union DistSimPair
  {
    struct
    {
      float orie_diff; // the diff of star-anchor orientation
      int8_t seq_src;  // source sequence of the neighbor of the anchor
      int8_t seq_tgt;
      int8_t level; // the level at which the pairing occurs
    };
    int data[2]{};

    DistSimPair(int8_t l, int8_t s, int8_t t, float o) : level(l), seq_src(s), seq_tgt(t), orie_diff(o) {}
  };

  // Four member variable
  std::bitset<BITS_PER_LAYER * NUM_BIN_KEY_LAYER> dist_bin_;
  //  std::map<u_int16_t, std::vector<RelativePoint>> dist_bit_neighbors_;  // {bit position in the bit vector: [neighbours point info, ...]}
  std::vector<RelativePoint> nei_pts_;
  std::vector<uint16_t> nei_idx_segs_; // index in the `nei_pts_`, [seg[i], seg[i+1]) is a segment with the same dist bit set.
  int8_t piv_seq_, level_;             // level and seq of the anchor/pivot

  explicit BCI(int8_t seq, int8_t lev) : dist_bin_(0), piv_seq_(seq), level_(lev) {}

  /**
   * * @brief checkConstellSim
   * * @param src: 候选闭环帧的BCI"
   * * @param tgt: 查询帧的BCI
   * * @param lb: 预设的阈值
   * * @param constell_res: 输出参数，可能的匹配对
   */
  static ScoreConstellSim checkConstellSim(const BCI &src, const BCI &tgt, const ScoreConstellSim &lb,
                                           std::vector<ConstellationPair> &constell_res)
  {
    DCHECK_EQ(src.level_, tgt.level_);
    std::bitset<BITS_PER_LAYER * NUM_BIN_KEY_LAYER> and1, and2, and3;
    // dist_bin_为二值距离向量，通过与操作进行相似度计算
    and1 = src.dist_bin_ & tgt.dist_bin_;
    and2 = (src.dist_bin_ << 1) & tgt.dist_bin_;
    and3 = (src.dist_bin_ >> 1) & tgt.dist_bin_;
    int ovlp1 = and1.count(), ovlp2 = and2.count(), ovlp3 = and3.count();
    int ovlp_sum = ovlp1 + ovlp2 + ovlp3;
    int max_one = std::max(ovlp1, std::max(ovlp2, ovlp3));

    ScoreConstellSim ret;

    ret.i_ovlp_sum = ovlp_sum;
    ret.i_ovlp_max_one = max_one;

    // the anchors are assumed to be matched
    if (ovlp_sum >= lb.i_ovlp_sum && max_one >= lb.i_ovlp_max_one)
    {
      // check the angular for constellation
      std::vector<DistSimPair> potential_pairs;

      int16_t p11 = 0, p12;
      // 寻找相近的anchor和外围CA之间的距离，因为这些CA很可能是匹配的
      for (int16_t p2 = 0; p2 < tgt.nei_idx_segs_.size() - 1; p2++)
      {
        while (p11 < src.nei_idx_segs_.size() - 1 &&
               src.nei_pts_[src.nei_idx_segs_[p11]].bit_pos < tgt.nei_pts_[tgt.nei_idx_segs_[p2]].bit_pos - 1)
        {
          p11++;
        }

        p12 = p11;

        while (p12 < src.nei_idx_segs_.size() - 1 &&
               src.nei_pts_[src.nei_idx_segs_[p12]].bit_pos <= tgt.nei_pts_[tgt.nei_idx_segs_[p2]].bit_pos + 1)
        {
          p12++;
        }
        // 保存匹配的CA对potential_pairs
        for (int i = tgt.nei_idx_segs_[p2]; i < tgt.nei_idx_segs_[p2 + 1]; i++)
        {
          for (int j = src.nei_idx_segs_[p11]; j < src.nei_idx_segs_[p12]; j++)
          {
            const BCI::RelativePoint &rp1 = src.nei_pts_[j], &rp2 = tgt.nei_pts_[i];

            DCHECK_EQ(rp1.level, rp2.level);
            DCHECK_LE(std::abs(rp1.bit_pos - rp2.bit_pos), 1);
            potential_pairs.emplace_back(rp1.level, rp1.seq, rp2.seq, rp2.theta - rp1.theta);
          }
        }
      }

      // 角度限制范围
      for (auto &x : potential_pairs)
        clampAng<float>(x.orie_diff);

      std::sort(potential_pairs.begin(), potential_pairs.end(), [&](const DistSimPair &a, const DistSimPair &b)
                { return a.orie_diff < b.orie_diff; });
      // p1-p2是一个滑动窗口，满足角度小于angular_range的序列中寻找最长子序列，因为角度差小于angular_range的CA对很可能是匹配的
      const float angular_range = M_PI / 16; // 0.2 rad, 11 deg
      int longest_in_range_beg = 0, longest_in_range = 1, pot_sz = potential_pairs.size(), p1 = 0, p2 = 0;
      while (p1 < pot_sz)
      {
        if (potential_pairs[p2 % pot_sz].orie_diff - potential_pairs[p1].orie_diff + 2 * M_PI * int(p2 / pot_sz) > angular_range)
          p1++;
        else
        {
          if (p2 - p1 + 1 > longest_in_range)
          {
            longest_in_range = p2 - p1 + 1;
            longest_in_range_beg = p1;
          }
          p2++;
        }
      }

      ret.i_in_ang_rng = longest_in_range;
      // the min number of pairs in range that assumed to be the true delta theta
      if (longest_in_range < lb.i_in_ang_rng)
        return ret;

      constell_res.clear();
      constell_res.reserve(longest_in_range + 1);

      // 保存满足角度条件的 constell_res
      for (int i = longest_in_range_beg; i < longest_in_range + longest_in_range_beg; i++)
      {
        constell_res.emplace_back(potential_pairs[i % pot_sz].level, potential_pairs[i % pot_sz].seq_src,
                                  potential_pairs[i % pot_sz].seq_tgt);
      }
      constell_res.emplace_back(src.level_, src.piv_seq_, tgt.piv_seq_); // the pivots are also a pair.

#if HUMAN_READABLE
      // the sort is for human readability
      std::sort(constell_res.begin(), constell_res.end(), [&](const ConstellationPair &a, const ConstellationPair &b)
                { return a.level < b.level || (a.level == b.level) && a.seq_src < b.seq_src; });
#endif

      return ret;
    }
    else
    {
      return ret; // ret code -1: not passing dist binary check
    }
  }
};

//! 2.5D continuous(float) pixel
union Pixelf
{
  struct
  {
    float row_f;
    float col_f;
    float elev;
  };
  int data[3]{};

  Pixelf(float r, float c, float e) : row_f(r), col_f(c), elev(e) {}

  Pixelf()
  {
    row_f = -1;
    col_f = -1;
    elev = -1;
  }

  bool operator<(const Pixelf &b) const
  {
    return row_f < b.row_f;
  }
};

//! manage the collection of contours in a scan
class ContourManager
{
  // configuration
  const ContourManagerConfig cfg_;
  const ContourViewStatConfig view_stat_cfg_;
  const float VAL_ABS_INF_ = 1e3;

  // property
  float x_max_, x_min_, y_max_, y_min_; // for points in the sensor frame, not in the bev frame
  std::string str_id_;
  int int_id_;

  // data
  std::vector<std::vector<std::shared_ptr<ContourView>>> cont_views_;
  std::vector<std::vector<float>> cont_perc_;         // the area percentage of the contour in its layer
  std::vector<int> layer_cell_cnt_;                   // total number of cells in each layer/level
  std::vector<std::vector<RetrievalKey>> layer_keys_; // the key of each layer
  std::vector<std::vector<BCI>> layer_key_bcis_;      // NOTE: No validity check on bci. Check key before using corresponding bci!

  cv::Mat1f bev_;
  std::vector<std::pair<int, Pixelf>> bev_pixfs_; // float row col height, and the hash generated from discrete row column.
  float max_bin_val_ = -VAL_ABS_INF_, min_bin_val_ = VAL_ABS_INF_;

protected:
  template <typename PointType>

  std::pair<int, int> hashPointToImage(const PointType &pt) const
  {
    std::pair<int, int> res{-1, -1};
    float padding = 1e-2;
    if (pt.x < x_min_ + padding || pt.x > x_max_ - padding || pt.y < y_min_ + padding || pt.y > y_max_ - padding ||
        (pt.y * pt.y + pt.x * pt.x) < cfg_.blind_sq_)
    {
      //      std::cout << pt.x << "\t" << pt.y << std::endl;
      return res;
    }
    res.first = int(std::floor(pt.x / cfg_.reso_row_)) + cfg_.n_row_ / 2;
    res.second = int(std::floor(pt.y / cfg_.reso_col_)) + cfg_.n_col_ / 2;

    DCHECK(res.first >= 0 && res.first < cfg_.n_row_);
    DCHECK(res.second >= 0 && res.second < cfg_.n_col_);

    return res;
  }

  V2F pointToContRowCol(const V2F &p_in_l) const
  {
    V2F continuous_rc(p_in_l.x() / cfg_.reso_row_ + cfg_.n_row_ / 2 - 0.5f,
                      p_in_l.y() / cfg_.reso_col_ + cfg_.n_col_ / 2 - 0.5f);
    return continuous_rc;
  }

  void makeContourRecursiveHelper(const cv::Rect &cc_roi, const cv::Mat1b &cc_mask, int level,
                                  const std::shared_ptr<ContourView> &parent);

public:
  explicit ContourManager(const ContourManagerConfig &config, int int_id) : cfg_(config), int_id_(int_id)
  {
    CHECK(cfg_.n_col_ % 2 == 0);
    CHECK(cfg_.n_row_ % 2 == 0);
    DCHECK(!cfg_.lv_grads_.empty());

    x_min_ = -(cfg_.n_row_ / 2) * cfg_.reso_row_;
    x_max_ = -x_min_;
    y_min_ = -(cfg_.n_col_ / 2) * cfg_.reso_col_;
    y_max_ = -y_min_;

    bev_ = cv::Mat::ones(cfg_.n_row_, cfg_.n_col_, CV_32F) * (-VAL_ABS_INF_);
    //    std::cout << bev_ << std::endl;
    //    pillar_pos2f_.clear();
    //    bev_pixfs_.reserve(int(cfg_.n_col_ * cfg_.n_row_ * 0.05));
    cont_views_.resize(cfg_.lv_grads_.size());
    cont_perc_.resize(cfg_.lv_grads_.size());

    layer_cell_cnt_.resize(cfg_.lv_grads_.size());
    layer_keys_.resize(cfg_.lv_grads_.size());
    layer_key_bcis_.resize(cfg_.lv_grads_.size());
  }
  //BEV为迪卡尔坐标系下的图像，行列数为n_row_和n_col_，每个像素点的值为最大高度
  template <typename PointType>
  void makeBEV(typename pcl::PointCloud<PointType>::ConstPtr &ptr_gapc, std::string str_id = "")
  {
    CHECK(ptr_gapc);
    CHECK_GT(ptr_gapc->size(), 10);

    TicToc clk;
    std::map<int, Pixelf> tmp_pillars;

    for (const auto &pt : ptr_gapc->points)
    {
      // 返回点所在的行列索引，原点在图像中心
      std::pair<int, int> rc = hashPointToImage<PointType>(pt);
      if (rc.first > 0)
      {
        float height = cfg_.lidar_height_ + pt.z;
        // 每个像素取最大高度（150×150）
        if (bev_(rc.first, rc.second) < height)
        {
          bev_(rc.first, rc.second) = height;
          // 所有坐标改为正值（即相加一半的有效距离）
          V2F coor_f = pointToContRowCol(V2F(pt.x, pt.y)); // same coord as row and col
          tmp_pillars[rc.first * cfg_.n_col_ + rc.second] = Pixelf(coor_f.x(), coor_f.y(), height);
        }
        max_bin_val_ = max_bin_val_ < height ? height : max_bin_val_;
        min_bin_val_ = min_bin_val_ > height ? height : min_bin_val_;
      }
    }
    bev_pixfs_.clear();
    bev_pixfs_.insert(bev_pixfs_.begin(), tmp_pillars.begin(), tmp_pillars.end());
    std::cout << "Time makebev: " << clk.toc() << std::endl;

    printf("Max/Min bin height: %f %f\n", max_bin_val_, min_bin_val_);
    if (!str_id.empty())
      str_id_ = std::move(str_id);
    else
      str_id_ = std::to_string(ptr_gapc->header.stamp);

    //    printf("Continuous Pos size: %lu\n", pillar_pos2f_.size());
    printf("Continuous Pos size: %lu\n", bev_pixfs_.size());

#if SAVE_MID_FILE
    cv::Mat mask, view;
    inRange(bev_, cv::Scalar::all(0), cv::Scalar::all(max_bin_val_), mask);
    //    inRange(bev_, cv::Scalar::all(0), cv::Scalar::all(5.0), mask);  // NOTE: set to a fixed max height for uniformity
    //    CHECK_GT(5.0, cfg_.lv_grads_.back());
    normalize(bev_, view, 0, 255, cv::NORM_MINMAX, -1, mask);
    std::string dir = std::string(PJSRCDIR) + "/results/bev_img/";
    cv::imwrite(dir + "cart_context-" + str_id_ + ".png", view);
#endif
  }

  void clearImage()
  {
    bev_.release();
  }

  void resumeImage()
  {
    bev_ = cv::Mat::ones(cfg_.n_row_, cfg_.n_col_, CV_32F) * (-VAL_ABS_INF_);
    for (const auto &pillar : bev_pixfs_)
    {
      int rr = pillar.first / cfg_.n_col_;
      int cc = pillar.first % cfg_.n_col_;
      DCHECK_LT(rr, cfg_.n_row_);
      DCHECK_LT(cc, cfg_.n_col_);
      bev_(rr, cc) = pillar.second.elev;
    }
  }

  cv::Mat1f getBevImage() const
  {
    if (bev_.empty())
    {
      cv::Mat1f tmp = cv::Mat::ones(cfg_.n_row_, cfg_.n_col_, CV_32F) * (-VAL_ABS_INF_);
      for (const auto &pillar : bev_pixfs_)
      {
        int rr = pillar.first / cfg_.n_col_;
        int cc = pillar.first % cfg_.n_col_;
        DCHECK_LT(rr, cfg_.n_row_);
        DCHECK_LT(cc, cfg_.n_col_);
        tmp(rr, cc) = pillar.second.elev;
      }
      return tmp;
    }
    else
      return bev_.clone();
  }

  void makeContoursRecurs()
  {
    cv::Rect full_bev_roi(0, 0, bev_.cols, bev_.rows);

    TicToc clk;
    // 分析所有联通区域提取轮廓，每个联通区域特征按照层级保存到cont_views_中
    makeContourRecursiveHelper(full_bev_roi, cv::Mat1b(1, 1), 0, nullptr);
    std::cout << "Time makecontour: " << clk.toc() << std::endl;
    // 每层的contour按cell数量排序
    for (int ll = 0; ll < cont_views_.size(); ll++)
    {
      std::sort(cont_views_[ll].begin(), cont_views_[ll].end(),
                [&](const std::shared_ptr<ContourView> &p1, const std::shared_ptr<ContourView> &p2) -> bool
                {
                  return p1->cell_cnt_ > p2->cell_cnt_;
                });
      layer_cell_cnt_[ll] = 0;
      for (int j = 0; j < cont_views_[ll].size(); j++)
      {
        layer_cell_cnt_[ll] += cont_views_[ll][j]->cell_cnt_;
      }
      // 每个联通区域占据该层所有cell的比例
      cont_perc_[ll].reserve(cont_views_[ll].size());

      for (int j = 0; j < cont_views_[ll].size(); j++)
      {
        cont_perc_[ll].push_back(cont_views_[ll][j]->cell_cnt_ * 1.0f / layer_cell_cnt_[ll]);
      }
    }

    // 创建检索键(10维向量3+7)
    const int roi_radius_padded = std::ceil(cfg_.roi_radius_ + 1);
    for (int ll = 0; ll < cfg_.lv_grads_.size(); ll++)
    {
      // 检索键第三个数值相关变量
      int accumulate_cell_cnt = 0;
      // 每次前6个轮廓作为锚点anchor
      for (int seq = 0; seq < cfg_.piv_firsts_; seq++)
      {
        RetrievalKey key;
        key.setZero();
        // 为锚点anchor创建BCI距离向量，对应论文公式（5）
        BCI bci(seq, ll);

        if (cont_views_[ll].size() > seq)
          accumulate_cell_cnt += cont_views_[ll][seq]->cell_cnt_;
        // anchor联通区域需要足够大 >9
        if (cont_views_[ll].size() > seq && cont_views_[ll][seq]->cell_cnt_ >= cfg_.min_cont_key_cnt_)
        {
          // 计算ROI边界索引，半径为10的圆
          V2F v_cen = cont_views_[ll][seq]->pos_mean_.cast<float>();
          int r_cen = int(v_cen.x()), c_cen = int(v_cen.y());
          int r_min = std::max(0, r_cen - roi_radius_padded),
              r_max = std::min(cfg_.n_row_ - 1, r_cen + roi_radius_padded);
          int c_min = std::max(0, c_cen - roi_radius_padded),
              c_max = std::min(cfg_.n_col_ - 1, c_cen + roi_radius_padded);

          int num_bins = RET_KEY_DIM - 3;
          KeyFloatType bin_len = cfg_.roi_radius_ / num_bins;
          std::vector<KeyFloatType> ring_bins(num_bins, 0);

          // 为了取消离散化误差每个bin继续细分
          int div_per_bin = 5;
          std::vector<KeyFloatType> discrete_divs(div_per_bin * num_bins, 0);
          // div_len=10/35
          KeyFloatType div_len = cfg_.roi_radius_ / (num_bins * div_per_bin);
          int cnt_point = 0;

          // 遍历ROI的所有CELL
          for (int rr = r_min; rr <= r_max; rr++)
          {
            for (int cc = c_min; cc <= c_max; cc++)
            {

              if (bev_(rr, cc) < cfg_.lv_grads_[DIST_BIN_LAYERS[0]]) // NOTE: new key
                continue;

              int q_hash = rr * cfg_.n_col_ + cc;
              std::pair<int, Pixelf> sear_res = search_vec<Pixelf>(bev_pixfs_, 0, bev_pixfs_.size() - 1, q_hash);
              DCHECK_EQ(sear_res.first, q_hash);
              KeyFloatType dist = (V2F(sear_res.second.row_f, sear_res.second.col_f) - v_cen).norm();

              if (dist < cfg_.roi_radius_ - 1e-2 && bev_(rr, cc) > cfg_.lv_grads_[DIST_BIN_LAYERS[0]])
              {

                int higher_cnt = 0;
                for (int ele = DIST_BIN_LAYERS[0]; ele < cfg_.lv_grads_.size(); ele++)
                  if (bev_(rr, cc) > cfg_.lv_grads_[ele])
                    higher_cnt++;

                cnt_point++;
                for (int div_idx = 0; div_idx < num_bins * div_per_bin; div_idx++)
                  discrete_divs[div_idx] +=
                      higher_cnt * gaussPDF<KeyFloatType>(div_idx * div_len + 0.5 * div_len, dist, 1.0);
              }
            }
          }

          for (int b = 0; b < num_bins; b++)
          {
            for (int d = 0; d < div_per_bin; d++)
            {
              ring_bins[b] += discrete_divs[b * div_per_bin + d];
            }
            ring_bins[b] *= bin_len / std::sqrt(cnt_point);
          }

          key(0) =
              std::sqrt(cont_views_[ll][seq]->eig_vals_(1) * cont_views_[ll][seq]->cell_cnt_); // max eigen value * cnt
          key(1) =
              std::sqrt(cont_views_[ll][seq]->eig_vals_(0) * cont_views_[ll][seq]->cell_cnt_); // min eigen value * cnt

          key(2) = std::sqrt(accumulate_cell_cnt);

          DCHECK_EQ(num_bins + 3, RET_KEY_DIM);
          for (int nb = 0; nb < num_bins; nb++)
          {
            key(3 + nb) = ring_bins[nb];
          }

          for (int bl = 0; bl < NUM_BIN_KEY_LAYER; bl++)
          {
            int bit_offset = bl * BITS_PER_LAYER;
            // the top 10 contours to be treated as peripheral CAs
            for (int j = 0; j < std::min(cfg_.dist_firsts_, (int)cont_views_[DIST_BIN_LAYERS[bl]].size()); j++)
            {
              // ll != DIST_BIN_LAYERS[bl]不在一层的CA也构建？ 我认为应当是同层
              if (ll == DIST_BIN_LAYERS[bl] || j != seq)
              {

                V2F vec_cc = cont_views_[DIST_BIN_LAYERS[bl]][j]->pos_mean_ - cont_views_[ll][seq]->pos_mean_;
                float tmp_dist = vec_cc.norm();

                if (tmp_dist > (BITS_PER_LAYER - 1) * 1.01 + 5.43 - 1e-3 || tmp_dist <= 5.43)
                  continue;

                float tmp_orie = std::atan2(vec_cc.y(), vec_cc.x());
                // 5.43数据含义？是为了避免更近距离的点可能为anchor点?
                int dist_idx = std::min(std::floor((tmp_dist - 5.43) / 1.01), BITS_PER_LAYER - 1.0) + bit_offset;

                DCHECK_LT(dist_idx, BITS_PER_LAYER * NUM_BIN_KEY_LAYER);

                bci.dist_bin_.set(dist_idx, true);
                bci.nei_pts_.emplace_back(DIST_BIN_LAYERS[bl], j, dist_idx, tmp_dist, tmp_orie);
              }
            }
          }

          if (!bci.nei_pts_.empty())
          { // 由近到远排序
            std::sort(bci.nei_pts_.begin(), bci.nei_pts_.end(), [&](const BCI::RelativePoint &p1, const BCI::RelativePoint &p2)
                      { return p1.bit_pos < p2.bit_pos; });

            bci.nei_idx_segs_.emplace_back(0);
            // nei_idx_segs_记录的是不同位的contour对的位置索引，nei_pts_中的dist_idx可能是相同的
            for (int p1 = 0; p1 < bci.nei_pts_.size(); p1++)
            {
              if (bci.nei_pts_[bci.nei_idx_segs_.back()].bit_pos != bci.nei_pts_[p1].bit_pos)
                bci.nei_idx_segs_.emplace_back(p1);
            }
            bci.nei_idx_segs_.emplace_back(bci.nei_pts_.size());
            DCHECK_EQ(bci.nei_idx_segs_.size(), bci.dist_bin_.count() + 1);
          }
        }
        //        if(key.sum()!=0)
        layer_key_bcis_[ll].emplace_back(bci); // no validity check on bci. check key before use bci!
        layer_keys_[ll].emplace_back(key);     // even invalid keys are recorded.

        //      printf("Key: l%d s%d: ", ll, seq);
        //       for (float &ki: key.array)
        //        printf("%8.4f ", ki);
        //      printf("\n");
      }
    }

    for (int ll = 0; ll < cfg_.lv_grads_.size(); ll++)
    {
      DCHECK_EQ(layer_keys_[ll].size(), cfg_.piv_firsts_);
      DCHECK_EQ(layer_key_bcis_[ll].size(), cfg_.piv_firsts_);
    }

#if SAVE_MID_FILE
    // save statistics of this scan:
    std::string fpath = std::string(PJSRCDIR) + "/results/contours_orig-" + str_id_ + ".txt";
    saveContours(fpath, cont_views_);
#endif

    int cnt = 0;
    printf("Manager data sizes:\n");

    for (const auto &itms : cont_views_)
      for (const auto &itm : itms)
        cnt++;
    printf("cont_views_: %d\n", cnt);

    cnt = 0;
    for (const auto &itms : layer_keys_)
      for (const auto &itm : itms)
        cnt++;
    printf("layer_keys_: %d\n", cnt);

    cnt = 0;
    for (const auto &itms : layer_key_bcis_)
      for (const auto &itm : itms)
        cnt++;
    printf("layer_key_bcis_: %d\n", cnt);

    cnt = 0;
    for (const auto &itms : bev_pixfs_)
      cnt++;
    printf("bev_pixfs_: %d\n", cnt);
  }

  // save accumulated contours to a file that is readable to the python script
  void saveAccumulatedContours(int top_n) const
  {
    std::vector<std::vector<std::shared_ptr<ContourView>>> new_cont_views;
    new_cont_views.resize(cont_views_.size());
    for (int ll = 0; ll < cfg_.lv_grads_.size(); ll++)
    {
      for (int i = 0; i < std::min(top_n, (int)cont_views_[ll].size()); i++)
      {
        if (i == 0)
          new_cont_views[ll].emplace_back(std::make_shared<ContourView>(*cont_views_[ll][i]));
        else
        {
          new_cont_views[ll].emplace_back(std::make_shared<ContourView>(
              ContourView::addContourRes(*new_cont_views[ll].back(), *cont_views_[ll][i], view_stat_cfg_)));
        }
      }
    }
    std::string fpath = std::string(PJSRCDIR) + "/results/contours_accu-" + str_id_ + ".txt";
    saveContours(fpath, new_cont_views);
  }

  // experimental: show dists from one contour to several others
  void expShowDists(int level, int pivot, int top_n)
  {
    CHECK_LT(level, cfg_.lv_grads_.size());
    CHECK_LT(pivot, cont_views_[level].size());
    printf("Level %d, pivot No.%d distances:\n", level, pivot);
    std::vector<std::pair<int, float>> dists;
    for (int i = 0; i < std::min(top_n, (int)cont_views_[level].size()); i++)
      if (i != pivot)
        dists.emplace_back(i, (cont_views_[level][i]->pos_mean_ - cont_views_[level][pivot]->pos_mean_).norm());

    std::sort(dists.begin(), dists.end(), [&](const std::pair<int, float> &a, const std::pair<int, float> &b)
              { return a.second < b.second; });
    for (const auto &pr : dists)
    {
      printf("%2d: %7.4f, ", pr.first, pr.second);
    }
    printf("\n");
  }

  // experimental: show dists from one contour to several others
  void expShowBearing(int level, int pivot, int top_n)
  {
    CHECK_LT(level, cfg_.lv_grads_.size());
    CHECK_LT(pivot, cont_views_[level].size());
    printf("Level %d, pivot No.%d orientations:\n", level, pivot);
    std::vector<std::pair<int, float>> bearings;
    bool first_set = false;
    V2F vec0(0, 0);
    for (int i = 0; i < std::min(top_n, (int)cont_views_[level].size()); i++)
    {
      if (i != pivot)
      {
        if (!first_set)
        {
          bearings.emplace_back(i, 0);
          first_set = true;
          vec0 = (cont_views_[level][i]->pos_mean_ - cont_views_[level][pivot]->pos_mean_).normalized();
        }
        else
        {
          V2F vec1 = (cont_views_[level][i]->pos_mean_ - cont_views_[level][pivot]->pos_mean_).normalized();
          float ang = std::atan2(vec0.x() * vec1.y() - vec0.y() * vec1.x(), vec0.dot(vec1));
          bearings.emplace_back(i, ang);
        }
      }
    }

    std::sort(bearings.begin(), bearings.end(), [&](const std::pair<int, float> &a, const std::pair<int, float> &b)
              { return a.second < b.second; });
    for (const auto &pr : bearings)
    {
      printf("%2d: %7.4f, ", pr.first, pr.second);
    }
    printf("\n");
  }

  // util functions
  // 1. save all contours' statistical data into a text file
  static void saveContours(
      const std::string &fpath, const std::vector<std::vector<std::shared_ptr<ContourView>>> &cont_views);

  // 2. save a layer of contours to image
  void saveContourImage(const std::string &fpath, int level) const;

  cv::Mat getContourImage(int level) const
  {
    cv::Mat mask;
    cv::threshold(getBevImage(), mask, cfg_.lv_grads_[level], 123,
                  cv::THRESH_TOZERO); // mask is same type and dimension as bev_

    cv::Mat normalized_layer, mask_u8;
    cv::normalize(mask, normalized_layer, 0, 255, cv::NORM_MINMAX, CV_8U); // dtype=-1 (default): same type as input
    return normalized_layer;
  }

  // TODO: get retrieval key of a scan
  const std::vector<RetrievalKey> &getLevRetrievalKey(int level) const
  {
    DCHECK_GE(level, 0);
    DCHECK_GT(cont_views_.size(), level);
    return layer_keys_[level];
  }

  const RetrievalKey &getRetrievalKey(int level, int seq) const
  {
    DCHECK_GE(level, 0);
    DCHECK_GT(cont_views_.size(), level);
    DCHECK_LT(seq, layer_keys_[level].size());
    return layer_keys_[level][seq];
  }

  // get contour
  inline const std::vector<std::shared_ptr<ContourView>> &getLevContours(int level) const
  {
    DCHECK_GE(level, 0);
    DCHECK_GT(cont_views_.size(), level);
    return cont_views_[level];
  }

  inline int getLevTotalPix(int level) const
  {
    DCHECK_GE(level, 0);
    DCHECK_GT(layer_cell_cnt_.size(), level);
    return layer_cell_cnt_[level];
  }

  // get bci
  const std::vector<BCI> &getLevBCI(int level) const
  {
    DCHECK_GE(level, 0);
    DCHECK_GT(cont_views_.size(), level);
    return layer_key_bcis_[level];
  }

  const BCI &getBCI(int level, int seq) const
  {
    DCHECK_GE(level, 0);
    DCHECK_GT(cont_views_.size(), level);
    DCHECK_LT(seq, layer_key_bcis_[level].size());
    return layer_key_bcis_[level][seq];
  }

  inline std::string getStrID() const
  {
    return str_id_;
  }

  inline int getIntID() const
  {
    return int_id_;
  }

  inline const ContourManagerConfig &getConfig() const
  {
    return cfg_;
  }

  inline const float &getAreaPerc(const int8_t &lev, const int8_t &seq) const
  {
    return cont_perc_[lev][seq];
  }

  // TODO: check if contours in two scans can be accepted as from the same heatmap, and return the transform
  // TODO: when retrieval key contains the combination, we should only look into that combination.
  // T_tgt = T_delta * T_src
  static std::pair<Eigen::Isometry2d, bool> calcScanCorresp(const ContourManager &src, const ContourManager &tgt);

  // T_tgt = T_delta * T_src
  /**
   * * @brief Check the similarity of each pair of stars(contours) in the constellation.
   * * @param src 候选闭环帧contour.
   * * @param 查询帧contour.
   * * @param cstl_in 输入的经过角度限制检查的星座对.
   * * @param lb 给定配置参数.
   * * @param cont_sim 给定配置参数.
   * * @param cstl_out 经过主轴方向相似度检查的星座对.
   * * @param area_perc 输出参数;The area percentage of each matched constellation pair.
   */

  static ScorePairwiseSim checkConstellCorrespSim(const ContourManager &src, const ContourManager &tgt,
                                                  const std::vector<ConstellationPair> &cstl_in,
                                                  const ScorePairwiseSim &lb,
                                                  const ContourSimThresConfig &cont_sim,
                                                  std::vector<ConstellationPair> &cstl_out,
                                                  std::vector<float> &area_perc)
  {
    // cross level consensus (CLC)
    // The rough constellation should have been established.
    DCHECK_EQ(src.cont_views_.size(), tgt.cont_views_.size());

    ScorePairwiseSim ret;

    std::map<int, std::pair<float, float>> lev_frac; // {lev:[src, tgt], }

    cstl_out.clear();
    area_perc.clear();
    // 1. check individual sim
#if HUMAN_READABLE
    printf("check individual sim of the constellation:\n");
#endif
    for (auto pr : cstl_in)
    {
      bool curr_success = false;
      if (checkContPairSim(src, tgt, pr, cont_sim))
      {
        cstl_out.push_back(pr);
        auto &it = lev_frac[pr.level];
        it.first += src.cont_perc_[pr.level][pr.seq_src];
        it.second += tgt.cont_perc_[pr.level][pr.seq_tgt];
        curr_success = true;
      }
#if HUMAN_READABLE
      printf("%d@lev %d, %d-%d\n", int(curr_success), pr.level, pr.seq_src, pr.seq_tgt);
#endif
    }

#if HUMAN_READABLE
    for (const auto &rec : lev_frac)
    {
      printf("matched percent lev: %d, %.3f/%.3f\n", rec.first, rec.second.first, rec.second.second);
    }
#endif

    ret.i_indiv_sim = cstl_out.size();

    if (ret.i_indiv_sim < lb.i_indiv_sim)
      return ret;

    // 2. check orientation
    // 2.1 get major axis direction
    // 选择距离最远的两个轮廓中心点之间的方向作为主轴
    V2F shaft_src(0, 0), shaft_tgt(0, 0);
    for (int i = 1; i < std::min((int)cstl_out.size(), 10); i++)
    {
      for (int j = 0; j < i; j++)
      {
        V2F curr_shaft = src.cont_views_[cstl_out[i].level][cstl_out[i].seq_src]->pos_mean_ -
                         src.cont_views_[cstl_out[j].level][cstl_out[j].seq_src]->pos_mean_;
        if (curr_shaft.norm() > shaft_src.norm())
        {
          shaft_src = curr_shaft.normalized();
          shaft_tgt = (tgt.cont_views_[cstl_out[i].level][cstl_out[i].seq_tgt]->pos_mean_ -
                       tgt.cont_views_[cstl_out[j].level][cstl_out[j].seq_tgt]->pos_mean_)
                          .normalized();
        }
      }
    }
    // 通过比较两个轮廓的主轴方向来验证匹配的合理性
    int num_sim = cstl_out.size();
    for (int i = 0; i < num_sim;)
    {
      const auto &sc1 = src.cont_views_[cstl_out[i].level][cstl_out[i].seq_src],
                 &tc1 = tgt.cont_views_[cstl_out[i].level][cstl_out[i].seq_tgt];
      if (sc1->ecc_feat_ && tc1->ecc_feat_)
      {
        float theta_s = std::acos(shaft_src.transpose() * sc1->eig_vecs_.col(1)); // acos: [0,pi)
        float theta_t = std::acos(shaft_tgt.transpose() * tc1->eig_vecs_.col(1));
        if (diff_delt<float>(theta_s, theta_t, M_PI / 6) && diff_delt<float>(M_PI - theta_s, theta_t, M_PI / 6))
        {
          // 将不一致的匹配交换到数组末尾
          std::swap(cstl_out[i], cstl_out[num_sim - 1]);
          num_sim--;
          continue;
        }
      }
      i++;
    }
    // 删除末尾的不一致匹配
    cstl_out.erase(cstl_out.begin() + num_sim, cstl_out.end()); // sure to reduce size, without default constructor
    ret.i_orie_sim = cstl_out.size();
    if (ret.i_orie_sim < lb.i_orie_sim)
      return ret; // TODO: use cross level consensus to find more possible matched pairs

#if HUMAN_READABLE
    std::sort(cstl_out.begin(), cstl_out.end()); // human readability
    printf("Found matched pairs:\n");
    for (const auto &i : cstl_out)
    {
      printf("\tlev %d, src:tgt  %d: %d\n", i.level, i.seq_src, i.seq_tgt);
    }
#endif

    // get the percentage of each of the nodes in the constellation
    area_perc.reserve(cstl_out.size());
    for (const auto &i : cstl_out)
    {
      area_perc.push_back(0.5f * (src.cont_perc_[i.level][i.seq_src] + tgt.cont_perc_[i.level][i.seq_tgt]));
    }

    return ret;
  }

  template <typename Iter>
  static Eigen::Isometry2d getTFFromConstell(const ContourManager &src, const ContourManager &tgt,
                                             Iter cstl_beg, Iter cstl_end)
  { // no const, just don't modify in the code
    int num_elem = cstl_end - cstl_beg;
    CHECK_GT(num_elem, 2);
    Eigen::Matrix<double, 2, Eigen::Dynamic> pointset1; // src
    Eigen::Matrix<double, 2, Eigen::Dynamic> pointset2; // tgt
    pointset1.resize(2, num_elem);
    pointset2.resize(2, num_elem);
    // 利用Umeyama算法计算变换矩阵（利用有效配对的位置中心）
    for (int i = 0; i < num_elem; i++)
    {
      pointset1.col(i) =
          src.cont_views_[(cstl_beg + i)->level][(cstl_beg + i)->seq_src]->pos_mean_.template cast<double>();
      pointset2.col(i) =
          tgt.cont_views_[(cstl_beg + i)->level][(cstl_beg + i)->seq_tgt]->pos_mean_.template cast<double>();
    }

    Eigen::Matrix3d T_delta = Eigen::umeyama(pointset1, pointset2, false);
#if HUMAN_READABLE
    std::cout << "Transform matrix:\n"
              << T_delta << std::endl;
#endif
    Eigen::Isometry2d ret;
    ret.setIdentity();
    ret.rotate(std::atan2(T_delta(1, 0), T_delta(0, 0)));
    ret.pretranslate(T_delta.block<2, 1>(0, 2));

    return ret;
  }
  // inline适用于短小、频繁调用的函数
  inline static bool checkContPairSim(const ContourManager &src, const ContourManager &tgt,const ConstellationPair &cstl,
                                      const ContourSimThresConfig &cont_sim)
  {
    return ContourView::checkSim(*src.cont_views_[cstl.level][cstl.seq_src],  *tgt.cont_views_[cstl.level][cstl.seq_tgt], cont_sim);
  }

  static void saveMatchedPairImg(const std::string &fpath, const ContourManager &cm1, const ContourManager &cm2)
  {
    ContourManagerConfig config = cm2.getConfig();

    DCHECK_EQ(config.n_row_, cm1.getConfig().n_row_);
    DCHECK_EQ(config.n_col_, cm1.getConfig().n_col_);
    DCHECK_EQ(cm2.getConfig().n_row_, cm1.getConfig().n_row_);
    DCHECK_EQ(cm2.getConfig().n_col_, cm1.getConfig().n_col_);

    cv::Mat output(config.n_row_ * 2 + 1, (config.n_col_ + 1) * config.lv_grads_.size(), CV_8U);

    output.setTo(255);

    for (int i = 0; i < config.lv_grads_.size(); i++)
    {

      cm1.getContourImage(i).copyTo(output(cv::Rect(i * config.n_col_ + i, 0, config.n_col_, config.n_row_)));
      cm2.getContourImage(i).copyTo(
          output(cv::Rect(i * config.n_col_ + i, config.n_row_ + 1, config.n_col_, config.n_row_)));
    }
    cv::imwrite(fpath, output);
  }
};

#endif
