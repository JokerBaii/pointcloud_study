# 点云练习数据：Stanford Bunny

官方数据页：https://graphics.stanford.edu/data/3Dscanrep/

下载地址：https://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz

已下载到 `data/stanford-bunny/`。官方压缩包约 4.9 MB，包含 10 次扫描、
完整重建模型和多个简化模型。本机已用系统 PCL 成功读取完整模型，
并转换成二进制 PCD，共 35,947 个点。

## 直接开始

在项目根目录 `/home/jokerbai/code` 运行：

```bash
pcl_viewer data/stanford-bunny/bunny.pcd
```

第一项练习：体素降采样。本机已实测从 35,947 点减少到 7,836 点。

```bash
pcl_voxel_grid data/stanford-bunny/bunny.pcd \
  data/stanford-bunny/bunny-downsampled.pcd -leaf 0.003,0.003,0.003
pcl_viewer data/stanford-bunny/bunny-downsampled.pcd
```

程序中可以使用：

```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
    new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile("data/stanford-bunny/bunny.pcd", *cloud);
```

该路径相对于程序运行目录；运行时先进入项目根目录。

## 文件选择

| 文件 | 内容 | 练习建议 |
| --- | --- | --- |
| `data/stanford-bunny/bunny.pcd` | 完整模型，35,947 点 | 读写、可视化、降采样、离群点过滤、法线估计 |
| `data/stanford-bunny/bunny/reconstruction/bun_zipper_res3.ply` | 简化模型，1,889 点 | 快速试验算法参数 |
| `data/stanford-bunny/bunny/data/bun000.ply` | 单次扫描，40,256 点 | 与其他扫描练习配准 |
| `data/stanford-bunny/bunny/data/bun045.ply` | 另一角度扫描，40,097 点 | 与 `bun000.ply` 配准，结合官方 `.conf` 位姿文件 |

先从完整模型的降采样和法线估计开始，再做配准。
ICP 入门可以先对同一云施加小幅已知变换，再尝试恢复；真实扫描之间
需要关注重叠范围和初始位姿，不能默认未经初始化就会正确收敛。

原始 PLY 包含三角形面。转换成 PCD 时保留顶点，丢弃面拓扑，
PCL 关于 `face/vertex_indices` 的提示不影响点云读取。

如果需要重新转换：

```bash
pcl_ply2pcd -format 1 \
  data/stanford-bunny/bunny/reconstruction/bun_zipper.ply \
  data/stanford-bunny/bunny.pcd
```

这些基础练习使用 CPU，显示点云使用本机 AMD 显卡的 OpenGL。
数据目录已被 `.gitignore` 忽略，这份说明可以跟随代码提交。

下载包的本地 SHA-256（用于识别这次下载，不是官方签名）：
`a5720bd96d158df403d153381b8411a727a1d73cff2f33dc9b212d6f75455b84`
