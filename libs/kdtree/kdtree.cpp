#include "kdtree.hpp"

#include <algorithm>
#include <numeric>
#include <utility>

namespace kdtree {
/** @brief 构造并建树。 */
KDTree::KDTree(const std::vector<std::array<float, 3>>& points) {
  build(points);
}

KDTree::KDTree(KDTree&& other) noexcept {
  points_.swap(other.points_);
  indices_.swap(other.indices_);
  root_.swap(other.root_);
}

KDTree& KDTree::operator=(KDTree&& other) noexcept {
  if (this != &other) {
    KDTree moved(std::move(other));
    points_.swap(moved.points_);
    indices_.swap(moved.indices_);
    root_.swap(moved.root_);
  }
  return *this;
}

/** @brief 构建新索引，成功后替换当前点云和树。 */
void KDTree::build(const std::vector<std::array<float, 3>>& points) {
  KDTree rebuilt;
  rebuilt.points_ = points;
  rebuilt.indices_.resize(points.size());
  std::iota(rebuilt.indices_.begin(), rebuilt.indices_.end(), 0);
  rebuilt.root_ = rebuilt.buildRecursive(0, points.size(), 0);

  points_.swap(rebuilt.points_);
  indices_.swap(rebuilt.indices_);
  root_.swap(rebuilt.root_);
}

/** @brief 递归构建下标区间 [begin, end) 对应的子树。 */
std::unique_ptr<KDTree::Node> KDTree::buildRecursive(std::size_t begin,
                                                     std::size_t end,
                                                     std::size_t axis) {
  if (begin >= end) {
    return nullptr;
  }
  const std::size_t middle = begin + (end - begin) / 2;
  // 只需让 middle 就位，O(n) 即可，不必整体排序；
  // 坐标相同时按下标比较，保证划分结果确定。
  std::nth_element(indices_.begin() + begin, indices_.begin() + middle,
                   indices_.begin() + end,
                   [this, axis](std::size_t lhs, std::size_t rhs) {
                     const float lhs_coordinate = points_[lhs][axis];
                     const float rhs_coordinate = points_[rhs][axis];
                     if (lhs_coordinate != rhs_coordinate) {
                       return lhs_coordinate < rhs_coordinate;
                     }
                     return lhs < rhs;
                   });

  auto node = std::make_unique<Node>(indices_[middle], axis);
  const std::size_t next_axis = (axis + 1) % 3;
  node->left = buildRecursive(begin, middle, next_axis);
  node->right = buildRecursive(middle + 1, end, next_axis);
  return node;
}

/** @brief 递归搜索 node 子树，把更近的点写回 best。 */
void KDTree::nearestRecursive(const Node* node,
                              const std::array<float, 3>& query_point,
                              NearestResult& best) const noexcept {
  if (node == nullptr) {
    return;
  }
  const auto& current_point = points_[node->point_index];
  const float current_distance_squared =
      squaredDistance(query_point, current_point);
  if (current_distance_squared < best.distance_squared) {
    best.point_index = node->point_index;
    best.distance_squared = current_distance_squared;
  }
  const std::size_t axis = node->axis;
  const float axis_distance = query_point[axis] - current_point[axis];
  const Node* near_child = nullptr;
  const Node* far_child = nullptr;
  if (axis_distance < 0) {
    near_child = node->left.get();
    far_child = node->right.get();
  } else {
    near_child = node->right.get();
    far_child = node->left.get();
  }
  nearestRecursive(near_child, query_point, best);
  const float plane_distance_squared = axis_distance * axis_distance;
  if (plane_distance_squared < best.distance_squared) {
    nearestRecursive(far_child, query_point, best);
  }
}

/** @brief 计算两点欧氏距离的平方。 */
float KDTree::squaredDistance(const std::array<float, 3>& lhs,
                              const std::array<float, 3>& rhs) noexcept {
  const float dx = lhs[0] - rhs[0];
  const float dy = lhs[1] - rhs[1];
  const float dz = lhs[2] - rhs[2];
  return dx * dx + dy * dy + dz * dz;
}

/** @brief 最近邻查询入口。 */
std::optional<KDTree::NearestResult> KDTree::nearest(
    const std::array<float, 3>& query) const noexcept {
  if (!root_) {
    return std::nullopt;
  }
  NearestResult best{root_->point_index,
                     squaredDistance(query, points_[root_->point_index])};
  nearestRecursive(root_.get(), query, best);
  return best;
}

bool KDTree::empty() const noexcept { return points_.empty(); }

std::size_t KDTree::size() const noexcept { return points_.size(); }

}  // namespace kdtree
