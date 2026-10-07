#pragma once

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

namespace kdtree {
/**
 * @brief 三维点云的 kd-tree 索引，用于最近邻搜索。
 *
 * x/y/z 轴轮流作分裂轴；建树时重排的是下标副本，不改动 points_ 本身。
 * 保存点云副本；不可拷贝，可以移动。
 * 多线程只读查询需要保证没有线程同时调用 build() 或移动该对象。
 */
class KDTree {
 public:
  /** @brief 最近邻查询结果。 */
  struct NearestResult {
    /** 命中点在 points_ 中的下标。 */
    std::size_t point_index{};
    /** 到查询点的欧氏距离平方。 */
    float distance_squared{};
  };

  /** @brief 构造空树，需再调用 build() 才能查询。 */
  KDTree() = default;
  ~KDTree() = default;

  KDTree(const KDTree&) = delete;
  KDTree& operator=(const KDTree&) = delete;
  KDTree(KDTree&& other) noexcept;
  KDTree& operator=(KDTree&& other) noexcept;

  /**
   * @brief 用点云构造并建树。
   */
  explicit KDTree(const std::vector<std::array<float, 3>>& points);

  /**
   * @brief 重建索引，丢弃原有的树。
   * @param points 新的点云，按值拷贝保存。
   * @note 如果分配内存失败，原有点云和索引保持有效。
   */
  void build(const std::vector<std::array<float, 3>>& points);

  /**
   * @brief 查询最近的点。
   * @return 最近点的下标和距离平方；空树返回 std::nullopt。
   */
  [[nodiscard]] std::optional<NearestResult> nearest(
      const std::array<float, 3>& query_point) const noexcept;

  /** @return 树是否为空。 */
  [[nodiscard]] bool empty() const noexcept;

  /** @return 点的个数。 */
  [[nodiscard]] std::size_t size() const noexcept;

 private:
  /** @brief kd-tree 节点。 */
  struct Node {
    /** 分裂点在 points_ 中的下标。 */
    std::size_t point_index{};
    /** 分裂轴：0 = x，1 = y，2 = z。 */
    std::size_t axis{};
    /** 分裂轴上坐标较小的一侧。 */
    std::unique_ptr<Node> left;
    /** 分裂轴上坐标较大的一侧。 */
    std::unique_ptr<Node> right;
    Node(std::size_t index, std::size_t split_axis)
        : point_index(index), axis(split_axis) {}
  };

  /** 点云副本，下标与 indices_、Node::point_index 对应。 */
  std::vector<std::array<float, 3>> points_;
  /** 下标数组，buildRecursive() 用 nth_element 在原位重排它。 */
  std::vector<std::size_t> indices_;
  /** 树根；空树时为 nullptr。 */
  std::unique_ptr<Node> root_;

  /**
   * @brief 递归构建下标区间 [begin, end) 对应的子树。
   * @return 子树根节点；区间为空时返回 nullptr。
   */
  [[nodiscard]] std::unique_ptr<Node> buildRecursive(std::size_t begin,
                                                     std::size_t end,
                                                     std::size_t axis);

  /**
   * @brief 递归搜索 node 子树，把更近的点写回 best。
   * @param best 输入输出参数：进入时为当前候选。
   */
  void nearestRecursive(const Node* node,
                        const std::array<float, 3>& query_point,
                        NearestResult& best) const noexcept;

  /**
   * @brief 计算两点欧氏距离的平方。
   * @return dx² + dy² + dz²，省去开方以便直接比较。
   */
  [[nodiscard]] static float squaredDistance(
      const std::array<float, 3>& lhs,
      const std::array<float, 3>& rhs) noexcept;
};
}  // namespace kdtree
