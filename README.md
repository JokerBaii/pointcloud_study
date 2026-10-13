# study_PCL — Stanford Bunny 点云重建练习

这是一个 **C++20 / Eigen / CMake** 的小型点云几何算法项目。目录完全按**一个功能一个文件夹**组织：头文件与实现文件就放在同一模块内，根目录 `main.cpp` 调用模块。

**注意**：仓库名里的 `PCL` 是你原有项目命名；基础算法**不依赖 PCL**。只有最终 Poisson 三角网格重建可选调用 PCL。项目与 ROS、具体雷达型号无关。

## 目录

```text
study_PCL/
├── data/
│   ├── demo_target.ply               # 自带的合成测试点云（不是 Bunny）
│   ├── demo_source.ply               # 自带的合成测试点云（不是 Bunny）
│   └── bunny_manifest.example.txt    # 多帧配准初始位姿格式示例
├── libs/
│   ├── common/types.hpp              # Eigen 点、点云、点+法向等类型
│   ├── kdtree/
│   │   ├── kdtree.hpp
│   │   ├── kdtree.cpp
│   │   └── CMakeLists.txt
│   ├── icp/
│   │   ├── rigid_transform.hpp
│   │   ├── rigid_transform.cpp       # 点到点 SVD 刚体估计
│   │   ├── icp.hpp
│   │   ├── icp.cpp                   # KD-Tree + SVD 的 ICP
│   │   └── CMakeLists.txt
│   ├── features/normal_estimation.*   # PCA + KNN 法向估计
│   ├── filters/voxel_grid.*          # 体素网格质心降采样
│   ├── io/ply_io.*                   # ASCII / binary little endian PLY
│   └── reconstruction/poisson.*      # 可选 PCL Poisson 后端
├── tests/                             # CTest 自包含测试
├── main.cpp
├── CMakeLists.txt
├── .clangd
└── README.md
```

## 1. 安装并编译（Fedora，fish shell）

```fish
sudo dnf install clang cmake ninja-build eigen3-devel

cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_COMPILER=clang++ \
  -DPOINTCLOUD_WITH_PCL=OFF

cmake --build build -j
ctest --test-dir build --output-on-failure
```

Arch / CachyOS：安装 `clang cmake ninja eigen` 后使用相同的 CMake 命令。

## 2. 无需下载数据，先验证能跑

```fish
./build/bunny_reconstruct \
  data/demo_target.ply data/demo_source.ply \
  --voxel 0.002 --max-dist 0.015 --normal-k 20
```

应该输出：

```text
bunny_merged.ply      # 两帧拼好的点云（只有顶点）
bunny_normals.ply     # 每个点还有 nx ny nz 法向
```

这两个演示文件是**非对称椭球形状的合成点云**，不是 Stanford Bunny；它们只是供你立刻检查完整流程（读取→ICP→融合→法向）是否运行正常。

### 本地点云预览

如果已有 `data/bunny/reconstruction/bun_zipper.ply`，可以直接读取它的顶点，降采样并估计法向：

```sh
./build/bunny_reconstruct data/bunny/reconstruction/bun_zipper.ply \
  --voxel 0.002 --normal-k 24 \
  --output out/bunny_cloud.ply --normals out/bunny_normals.ply \
  --pcd out/bunny_cloud.pcd
pcl_viewer out/bunny_cloud.pcd -ps 2
```

输出目录会自动创建。PLY 保留 double 坐标；`--pcd` 额外输出符合 [PCD 格式](https://pointclouds.org/documentation/tutorials/pcd_file_format.html)的 float32 XYZ 点云，供 `pcl_viewer` 预览。拖动鼠标旋转，滚轮缩放，按 `h` 查看操作说明。

在 VS Code 的 **Terminal → Run Task** 中选择 `Point cloud: view Bunny`，即可依次配置、编译、处理并打开查看器。`Point cloud: run ICP demo` 会运行上面的两帧合成数据配准。查看器需要安装 PCL 命令行工具。

这里读取的是已有重建模型的顶点，单文件运行不执行多帧 ICP；两帧配准请使用演示数据或下文的局部扫描。

## 3. 换成真正的 Stanford Bunny

把 Stanford Bunny 数据集的两个**局部扫描**放到 `data/`，例如：

```text
data/bun000.ply
data/bun045.ply
```

运行：

```fish
./build/bunny_reconstruct data/bun000.ply data/bun045.ply \
  --voxel 0.002 --max-dist 0.03
```

其中 `--voxel`、`--max-dist` 的数值**与点云原始 XYZ 单位相同**，不是必然以米为单位。先检查点云尺度，再根据点间距和初始偏差设定参数。项目不随压缩包再分发 Stanford Bunny 原始数据。

**ICP（Iterative Closest Point，迭代最近点）是局部优化，不负责发现任意两帧的全局相对位姿。** 如果原始扫描不是大致对齐状态，应提供粗初始位姿，否则可能失败或错误收敛。

## 4. 多帧扫描使用 manifest

`data/bunny_manifest.example.txt` 示范如下格式：

```text
# 文件路径 相对世界系的粗平移和姿态，角度单位是 degree
# path tx ty tz roll pitch yaw
bun000.ply 0 0 0 0 0 0
bun045.ply 0 0 0 0 0 0
```

**示例零位姿只是格式示例，不是 Stanford Bunny 提供的实际相机位姿。** 请按照实际数据的初始估计修改。每个路径相对 manifest 文件所在目录解析，第二帧起由 ICP 细化初始位姿。

```fish
cp data/bunny_manifest.example.txt data/bunny_manifest.txt
# 手工修改文件名和初始位姿后：
./build/bunny_reconstruct --manifest data/bunny_manifest.txt \
  --voxel 0.002 --max-dist 0.03
```

若你的数据**已经配准到同一坐标系**，可设置 `--skip-icp`，直接根据 manifest 中的位姿变换后合并。

## 5. 得到真正的三角网格 Mesh

点云融合与表面重建是**不同阶段**：点云 PLY 只有 `vertex`；三角网格 PLY 同时有 `vertex` 和 `face`。

安装 PCL 并开启后端：

```fish
sudo dnf install pcl-devel

cmake -S . -B build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DPOINTCLOUD_WITH_PCL=ON
cmake --build build -j
```

```fish
./build/bunny_reconstruct --manifest data/bunny_manifest.txt \
  --voxel 0.002 --max-dist 0.03 \
  --normal-k 24 --mesh bunny_mesh.ply --poisson-depth 9
```

流程：多帧点云 → ICP → 体素融合 → PCA 法向 → Poisson → 三角面网格。

**限制**：

- Poisson 使用 PCL 后端，不是本项目自实现的泊松方程求解器。
- 法向一致化目前采用“指向全局质心外”的简单启发式，对凹面、薄壁结构**不保证正确**；真实 Bunny 应检查法向方向，严重错误时需要更可靠的法向一致化。
- 逐帧 scan-to-map ICP 没有全局优化、闭环约束、FPFH/RANSAC 粗配准；多视角结果可能有漂移，也可能对不齐。
- 本示例 PLY 输入只提取 `vertex` 的 XYZ；不会将原有网格的 `face` 当作输入几何重建。

## 6. 按顺序读代码（推荐）

| 顺序 | 文件 | 只关心一个问题 |
|---|---|---|
| 1 | `libs/common/types.hpp` | 一个 3D 点在内存里怎么表示？ |
| 2 | `libs/icp/rigid_transform.cpp` | 给定对应点，如何用 SVD 算 R、t？ |
| 3 | `libs/kdtree/kdtree.cpp` | 怎样快速找最近邻？ |
| 4 | `libs/icp/icp.cpp` | 为什么要循环“找对应→求变换”？ |
| 5 | `libs/filters/voxel_grid.cpp` | 怎样减少重复点？ |
| 6 | `libs/features/normal_estimation.cpp` | 如何用局部 PCA 得到法向？ |
| 7 | `main.cpp` | 如何把库组装成一条完整流水线？ |

用 `ctest --test-dir build --output-on-failure` 检验你修改后有没有把核心算法写坏。代码注释重点放在几何公式和变换方向上，而不是重复变量名。
