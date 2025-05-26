//
// Created by lewis on 5/5/22.
//

#include "cont2/contour_mng.h"

void ContourManager::saveContours(const std::string &fpath,
                                  const std::vector<std::vector<std::shared_ptr<ContourView>>> &cont_views)
{
  // 0:level, 1:cell_cnt, 2:pos_mean, 4:pos_cov, 8:eig_vals, eig_vecs(10), 14:eccen, 15:vol3_mean, 16:com, 18,19:..
  // Note that recording data as strings has accuracy loss
  //    std::string fpath = sav_dir + "/contours_" + str_id_ + ".txt";
  std::fstream res_file(fpath, std::ios::out);

  if (res_file.rdstate() != std::ifstream::goodbit)
  {
    std::cerr << "Error opening " << fpath << std::endl;
    return;
  }
  printf("Writing results to file \"%s\" ...", fpath.c_str());
  res_file << "\nDATA_START\n";
  for (const auto &layer : cont_views)
  {
    for (const auto &cont : layer)
    {
      res_file << cont->level_ << '\t';
      res_file << cont->cell_cnt_ << '\t';

      res_file << cont->pos_mean_.x() << '\t' << cont->pos_mean_.y() << '\t';
      for (int i = 0; i < 4; i++)
        res_file << cont->pos_cov_.data()[i] << '\t';

      res_file << cont->eig_vals_.x() << '\t' << cont->eig_vals_.y() << '\t';
      for (int i = 0; i < 4; i++)
        res_file << cont->eig_vecs_.data()[i] << '\t';

      res_file << cont->eccen_ << '\t';
      res_file << cont->vol3_mean_ << '\t';
      res_file << cont->com_.x() << '\t' << cont->com_.y() << '\t';

      res_file << int(cont->ecc_feat_) << '\t';
      res_file << int(cont->com_feat_) << '\t';

      res_file << '\n';
    }
  }
  res_file << "DATA_END\n";
  res_file.close();
  printf("Writing results finished.\n");
}


void ContourManager::saveContourImage(const std::string &fpath, int level) const
{
  CHECK(!bev_.empty());
  cv::imwrite(fpath, getContourImage(level));
}

void ContourManager::makeContourRecursiveHelper(const cv::Rect &cc_roi, const cv::Mat1b &cc_mask, int level,
                                                const std::shared_ptr<ContourView> &parent)
{
  DCHECK(bool(level) == bool(parent));

  if (level >= cfg_.lv_grads_.size())
    return;

  float h_min = cfg_.lv_grads_[level], h_max = VAL_ABS_INF_;

  cv::Mat1f bev_roi = bev_(cc_roi), thres_roi;
  cv::threshold(bev_roi, thres_roi, h_min, 255, cv::THRESH_BINARY);

  cv::Mat1b bin_bev_roi;
  thres_roi.convertTo(bin_bev_roi, CV_8U);

  if (level)

    cv::bitwise_and(bin_bev_roi, cc_mask, bin_bev_roi);

  if (level < cfg_.lv_grads_.size() - 1)

    h_max = cfg_.lv_grads_[level - 1];

  // 2. calculate connected blobs
  cv::Mat1i labels, stats;
  cv::Mat centroids;
  cv::connectedComponentsWithStats(bin_bev_roi, labels, stats, centroids, 8, CV_32S);

  // 3. create contours for each connected component
  // https://stackoverflow.com/questions/37745274/opencv-find-perimeter-of-a-connected-component/48618464#48618464
  for (int n = 1; n < stats.rows; n++)
  {
    // 联通CELL数量必须大于阈值3
    if (stats(n, 4) < cfg_.min_cont_cell_cnt_)
      continue;

    // Rectangle around the connected component
    //  Rect: col0, row0, n_col, n_row
    cv::Rect rect_g(stats(n, 0) + cc_roi.x, stats(n, 1) + cc_roi.y, stats(n, 2), stats(n, 3)); // global: on bev
    cv::Rect rect_l(stats(n, 0), stats(n, 1), stats(n, 2), stats(n, 3));                       // local: relative to bev_roi

    cv::Mat1b mask_n = labels(rect_l) == n;

    RunningStatRecorder tmp_rec;
    int poi_r = -1, poi_c = -1; // the point(r,c) coordinate on the global bev for the contour
    // 利用Mask提取区域内的有效CELL
    for (int i = 0; i < rect_l.height; i++)
      for (int j = 0; j < rect_l.width; j++)
        if (mask_n(i, j))
        {
          poi_r = i + rect_g.y;
          poi_c = j + rect_g.x;

          int q_hash = poi_r * cfg_.n_col_ + poi_c;
          std::pair<int, Pixelf> sear_res = search_vec<Pixelf>(bev_pixfs_, 0, (int)bev_pixfs_.size() - 1, q_hash);
          DCHECK_EQ(sear_res.first, q_hash);
          // 记录需要利用的5个特性信息
          tmp_rec.runningStatsF(sear_res.second.row_f, sear_res.second.col_f, bev_(poi_r, poi_c));
        }

    std::shared_ptr<ContourView> ptr_tmp_cv(new ContourView(level, poi_r, poi_c));
    ptr_tmp_cv->calcStatVals(tmp_rec, view_stat_cfg_);
    
    DCHECK(ptr_tmp_cv->cell_cnt_ == stats(n, 4));

    cont_views_[level].emplace_back(ptr_tmp_cv); 

    //printf("contour ROI: %d, %d, level: %d\n", mask_n.rows, mask_n.cols, level);
    makeContourRecursiveHelper(rect_g, mask_n, level + 1, ptr_tmp_cv);
  }
}
