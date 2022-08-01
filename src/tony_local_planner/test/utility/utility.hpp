#ifndef GOAL_SENDER_HPP_
#define GOAL_SENDER_HPP_

#include <algorithm>
#include <iterator>
#include <vector>

#include <boost/iterator/zip_iterator.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/algorithm/max_element.hpp>
#include <boost/range/algorithm/min_element.hpp>
#include <boost/range/combine.hpp>
#include <boost/tuple/tuple.hpp>

#include <boost/filesystem.hpp>
#include <boost/iterator/transform_iterator.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "tony_local_planner/tf_wrapper.h"

namespace bag_util {

template <typename ContentType>
struct BagContent {
  explicit BagContent(rosbag::Bag const &t_bag) noexcept : content_{extract_bag_content(t_bag)} {}

  static auto extract_bag_content(rosbag::Bag const &t_bag) noexcept {
    std::vector<ContentType> ret_val;
    rosbag::View view(t_bag);
    ret_val.reserve(std::distance(view.begin(), view.end()));  // view.size()

    std::transform(view.begin(), view.end(), std::back_inserter(ret_val),
                   [](auto const &t_content) { return *t_content.template instantiate<ContentType>(); });

    return ret_val;
  }

  inline auto get() const noexcept { return this->content_; }

 private:
  std::vector<ContentType> content_;
};

template <typename ContentType>
struct BagPlayer {
  using FilePath = boost::filesystem::path;

  explicit BagPlayer(std::vector<FilePath> const &t_bag_dir) noexcept : bag_contents_{get_bag_contents(t_bag_dir)} {}

  static auto get_bag_contents(std::vector<FilePath> const &t_bag_dir) noexcept {
    std::vector<BagContent<ContentType>> ret_val;
    ret_val.reserve(t_bag_dir.size());

    std::transform(t_bag_dir.begin(), t_bag_dir.end(), std::back_inserter(ret_val),
                   [](auto const &t_input) { return BagContent<ContentType>{rosbag::Bag{t_input.string()}}; });

    return ret_val;
  }

  auto begin() const noexcept { return this->bag_contents_.begin(); }
  auto end() const noexcept { return this->bag_contents_.end(); }
  auto size() const noexcept { return this->bag_contents_.size(); }

 private:
  std::vector<BagContent<ContentType>> bag_contents_;
};

inline auto get_test_bag_files(std::string const &t_test_file_dir) noexcept {
  using boost::make_iterator_range;
  using boost::filesystem::directory_iterator;
  using boost::filesystem::path;

  std::vector<path> ret_val;
  for (auto const &file : make_iterator_range(directory_iterator(t_test_file_dir), {})) {
    if (file.path().extension() == path(".bag")) {
      ret_val.push_back(file.path());
    }
  }

  return ret_val;
}

}  // namespace bag_util

namespace path_info {

inline double find_min_dist_between_pathpoint(bag_util::BagContent<nav_msgs::Path> const &t_plan) noexcept {
  using namespace boost::adaptors;
  using boost::make_iterator_range;
  using boost::make_zip_iterator;

  auto const content = t_plan.get();

  auto const min_dist_in_path = [](auto const &t_nav_path) {
    auto const compute_dist = [](auto const &t_pose_pair) {
      geometry_msgs::PoseStamped first, second;
      boost::tie(first, second) = t_pose_pair;

      auto const dist = std::sqrt(std::pow(first.pose.position.x - second.pose.position.x, 2) +
                                  std::pow(first.pose.position.y - second.pose.position.y, 2));

      return dist >= 1e-3 ? dist : 1e6;  // non zero
    };

    auto const path  = t_nav_path.poses;
    auto const begin = make_zip_iterator(boost::make_tuple(path.begin(), std::next(path.begin())));
    auto const end   = make_zip_iterator(boost::make_tuple(std::prev(path.end()), path.end()));
    auto const dist_ = make_iterator_range(begin, end) | transformed(std::ref(compute_dist));
    return *boost::min_element(dist_);
  };

  return *boost::min_element(content | transformed(std::ref(min_dist_in_path)));
}

inline double find_max_dist_path(bag_util::BagContent<nav_msgs::Path> const &t_plan) noexcept {
  using namespace boost::adaptors;

  auto const get_dist = [](auto const t_path) {
    auto const start = t_path.poses.front();
    auto const end   = t_path.poses.back();

    return std::sqrt(std::pow(start.pose.position.x - end.pose.position.x, 2) +
                     std::pow(start.pose.position.y - end.pose.position.y, 2));
  };

  return *boost::max_element(t_plan.get() | transformed(std::ref(get_dist)));
}

}  // namespace path_info

#endif