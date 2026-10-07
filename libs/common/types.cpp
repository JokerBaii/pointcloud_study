#include <Eigen/Core>
#include <vector>
namespace PointCloud {
using Point3D = Eigen::Vector3d;
using PointCloud = std::vector<Point3D>;
using Vec3D = Eigen::Vector3d;
using Mat3D = Eigen::Matrix3d;
using Mat4D = Eigen::Matrix4d;
struct PointNormal {
  Point3D point = Point3D::Zero();
  Vec3D normal = Vec3D::Zero();
};
using PointNormalCloud = std::vector<PointNormal>;
}  // namespace PointCloud