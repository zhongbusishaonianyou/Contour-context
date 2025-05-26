/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2025-05-10 21:30:41
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2025-05-26 13:01:17
 * @FilePath: /contour_context/src/contour-context-main/include/tools/config_handler.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
//
// Created by lewis on 2/2/23.
//

#ifndef CONT2_CONFIG_HANDLER_H
#define CONT2_CONFIG_HANDLER_H

#include <vector>
#include <iostream>
#include <glog/logging.h>
#include <opencv2/core.hpp>

// load and print loaded config
struct yamlLoader
{ // assume no list/sequence in the middle
  cv::FileStorage fs;

  explicit yamlLoader(const std::string &config_fpath)
  {
    fs.open(config_fpath, cv::FileStorage::READ);
  }

  void read(const std::string &config_fpath)
  {
    if (fs.isOpened())
      fs.release();
    fs.open(config_fpath, cv::FileStorage::READ);
  }

  void close()
  {
    fs.release();
  }

  template <typename T>
  void loadOneConfig(const std::vector<std::string> &keys, T &container) const
  {
    // assume no list/sequence in the middle
    CHECK(!keys.empty());
    cv::FileNode fn;
    for (int i = 0; i < keys.size(); i++)
    {
      fn = i ? fn[keys[i]] : fs[keys[0]];
      std::cout << "\"" << keys[i] << "\"->";
    }
    if (fn.isNone())
    {
      printf(": [!] Cannot find the specified config parameter!\n");
      return;
    }
    container = (T)fn;
    std::cout << ": " << container << std::endl;
  }

  template <typename T>
  void loadSeqConfig(const std::vector<std::string> &keys, std::vector<T> &container) const
  {
    // assume no list/sequence in the middle
    CHECK(!keys.empty());
    cv::FileNode fn;
    for (int i = 0; i < keys.size(); i++)
    {
      fn = i ? fn[keys[i]] : fs[keys[0]];
      std::cout << "\"" << keys[i] << "\"->";
    }
    if (fn.isNone())
    {
      printf(": [!] Cannot find the specified config parameter!\n");
      return;
    }

    CHECK_EQ(fn.type(), cv::FileNode::SEQ);
    container.clear();
    cv::FileNodeIterator it = fn.begin(), it_end = fn.end(); // Go through the node
    for (; it != it_end; ++it)
      container.emplace_back((T)(*it));

    std::cout << ": ";
    for (const auto &dat : container)
    {
      std::cout << dat << ", ";
    }
    std::cout << std::endl;
  }
};

#endif // CONT2_CONFIG_HANDLER_H
