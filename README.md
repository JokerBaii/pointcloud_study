# Bunny Reconstruction — 模块化 C++ 点云基础工程

目标：把 Stanford Bunny 的多视角局部扫描配准、融合为完整点云，并估计法向；若安装 PCL，可继续做 Poisson Surface Reconstruction（泊松表面重建）输出 Mesh。

## 工程结构

```text
pointcloud_bunny_reconstruction_modular/
├── libs/
│   ├── common/          # 基础类型
│   ├── io/              # PLY 读写
│   ├── kdtree/          # KD-Tree / KNN
│   ├── filters/         # Voxel Grid
│   ├── registration/    # SVD rigid transform + ICP
│   ├── features/        # Normal Estimation
│   └── reconstruction/  # 可选 PCL Poisson
├── app/
│   └── main.cpp
├── tests/
├── data/
└── CMakeLists.txt
```

每个算法目录都是独立 CMake Target；`app` 只负责把库串起来。

## 算法主线

```text
多个局部扫描
    ↓
Voxel Grid
    ↓
ICP Registration
    ↓
求出每帧 source -> world 的 T
    ↓
Transform + Fusion
    ↓
完整 Bunny Point Cloud
    ↓
Normal Estimation
    ↓
Poisson（可选）
    ↓
Triangle Mesh
```

核心基础算法是自己实现的：

- KD-Tree 最近邻与 KNN
- Voxel Grid
- SVD 刚体变换
- Point-to-Point ICP
- PCA/协方差法向估计

Poisson 本身涉及八叉树、隐式函数、Poisson 方程、稀疏系统与等值面提取，所以这里把它作为 PCL 可选后端，不把主线学习目标淹没掉。

## Fedora 依赖

基础版本：

```fish
sudo dnf install clang cmake ninja-build eigen3-devel
```

需要输出 Mesh 时再装：

```fish
sudo dnf install pcl-devel
```

## 编译

```fish
cmake -S . -B build \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Debug \
    -DCMAKE_CXX_COMPILER=clang++

cmake --build build
```

完全不使用 PCL：

```fish
cmake -S . -B build \
    -G Ninja \
    -DPOINTCLOUD_WITH_PCL=OFF

cmake --build build
```

## 测试

```fish
ctest --test-dir build --output-on-failure
```

有三个最小测试：KD-Tree、SVD rigid transform、synthetic ICP。

## 两帧兔子

```fish
./build/app/bunny_reconstruct \
    data/bun000.ply \
    data/bun045.ply \
    --voxel 0.002 \
    --max-dist 0.03
```

默认输出：

```text
bunny_merged.ply
bunny_normals.ply
```

## 多帧兔子

```fish
./build/app/bunny_reconstruct \
    data/bun000.ply \
    data/bun045.ply \
    data/bun090.ply \
    data/bun180.ply \
    --voxel 0.002 \
    --max-dist 0.03
```

普通 ICP 是 Local Registration（局部配准），初始姿态差太大时会失败。因此项目支持 manifest 给每帧一个粗初值：

```text
path tx ty tz roll_deg pitch_deg yaw_deg
```

例如：

```text
bun000.ply 0 0 0 0 0 0
bun045.ply 0.01 0 0 0 0 -40
bun090.ply 0.02 0 0 0 0 -85
```

上面的数值只是格式示例，不是 Stanford Bunny 官方真值。

运行：

```fish
./build/app/bunny_reconstruct \
    --manifest data/bunny_manifest.txt \
    --voxel 0.002 \
    --max-dist 0.03
```

以后实现 FPFH + RANSAC 后，manifest 的粗初值就可以由粗配准自动提供。

## 输出 Mesh

如果 CMake 找到了 PCL：

```fish
./build/app/bunny_reconstruct \
    --manifest data/bunny_manifest.txt \
    --voxel 0.002 \
    --max-dist 0.03 \
    --normal-k 24 \
    --mesh bunny_mesh.ply \
    --poisson-depth 9
```

## 推荐阅读代码顺序

```text
1. libs/common/.../types.hpp
2. libs/registration/src/rigid_transform.cpp
3. libs/kdtree/src/kdtree.cpp
4. libs/registration/src/icp.cpp
5. libs/filters/src/voxel_grid.cpp
6. libs/features/src/normal_estimation.cpp
7. libs/io/src/ply_io.cpp
8. app/main.cpp
9. libs/reconstruction/src/poisson.cpp
```

### ICP 只记这个循环

```text
当前 source
    ↓
KD-Tree 找 target 最近邻
    ↓
得到 correspondence
    ↓
SVD 求 ΔR、Δt
    ↓
source <- ΔT source
    ↓
重新找 correspondence
```

变换累计：

```text
T_total <- ΔT * T_total
```

### Normal Estimation 只记这个过程

```text
一个点
 ↓
KNN 找局部邻居
 ↓
局部质心
 ↓
Covariance Matrix
 ↓
Eigen Decomposition
 ↓
最小特征值对应特征向量
 ↓
Normal
```

## 下一步升级

建议按顺序继续加：

```text
Correspondence rejection
→ Trimmed ICP
→ Point-to-Plane ICP
→ FPFH
→ RANSAC coarse registration
→ Multi-view Pose Graph
→ Loop Closure
```

这个仓库的定位不是“兔子专用程序”，而是一个小型 3D Geometry / Point Cloud Library；Bunny 只是第一个完整验证对象。
