# 开发日志

## 2026-08-18：v1.4.2 导出文本换行符统一

- 所有由插件写入或复制的文本文件统一使用 UTF-8、LF（`\n`）及单一文件末尾换行；覆盖 XML、Xacro、URDF、YAML、RViz、launch、trans、LICENSE 和包元数据。
- 模板复制后只规范化已知文本类型，STL、DAE 等二进制网格不会被读取或改写；新增自动化校验，检查 UTF-8、CRLF/孤立 CR 与末尾换行。

## 2026-08-13：独立碰撞网格导出

本次在不影响原版 Fusion 脚本的前提下，新增 `fusion2urdf_collision` 部署方式及
`collision_` 命名约定。

- 名称以 `collision_` 开头的 body 会单独导出为 `meshes/collision/` 下的 STL，
  并在 Xacro 中生成对应的 `<collision>` 标签。
- 未定义碰撞 body 的 link 保持兼容：collision 自动复用 visual mesh；导出结束
  对话框会列出这些 link。
- 碰撞 body 不参与质量、质心和惯量计算；结束对话框同时显示每个 link 的质量、
  纳入物理计算的 body 数和被排除的碰撞 body 数，便于核验。
- 仅导出从 `base_link` 沿关节 parent → child 方向可达的机器人树；不连通的螺丝、
  夹具和参考组件不会写入 URDF 或生成 mesh。
- link 内的静态嵌套组件（如电机、减速器）会递归纳入 visual mesh、质量与惯量，
  但不能作为独立运动 link 使用。
- STL 导出改用导出后自动删除的临时 occurrence，不再重命名源组件或在设计树中
  留下 `old_component`。

验证：使用同一 Fusion 关节姿态比较新旧 Xacro，质量、质心、惯量和关节参数一致；
仅保留浮点运算末位差异。

## 2026-08-13：集成 Pinocchio 与 EAIK URDF 转换

- 在导出完成后，直接于 `urdf/` 生成 `<robot>_pin.urdf` 和
  `<robot>_eaik.urdf`，不再需要单独运行转换脚本。
- Pinocchio URDF 保留完整连杆树；EAIK URDF 自动提取唯一六轴串联链，并将关节
  依序命名为 `joint_1` 至 `joint_6`。
- 转换会内联材料，移除 xacro include、Gazebo 与 transmission 标签，并将所有
  visual/collision mesh 改为相对文件路径。
- EAIK 自动识别失败不会影响原始 Xacro 或 Pinocchio URDF 的生成；最终 Fusion
  对话框会给出两种 URDF 的生成状态。

## 2026-08-13：v1.3.0 Fusion 虚拟末端坐标系

- 删除导出前自动添加 `tool0`/`tcp` 的对话框和写入逻辑；末端位姿必须在 Fusion
  设计中显式定义，避免手工输入坐标与 CAD 脱节。
- 根装配中名称严格为 `tool0` 或 `tcp`、且不含任何 BRep body 的组件，会被识别为
  虚拟 link。草图与 Joint Origin 可保留，用于定义坐标系位置和方向。
- 虚拟 link 必须通过 Rigid Joint（URDF fixed joint）连接到机器人树；Xacro、
  Pinocchio 与 EAIK URDF 均保留该 joint 的 Fusion 位姿。
- 虚拟 link 不生成质量、惯量、visual、collision 或 STL；结束对话框会显示其父
  link、fixed joint 及无物理/网格属性状态。EAIK 仍只计数六个可动关节。

## 2026-08-13：v1.4.0 通用下游输出 Profile

- 独立 URDF 的可动关节按从根 link 出发的运动学树稳定编号为 `joint_1`、
  `joint_2`……；空末端 frame 的固定关节使用语义名称，例如
  `tool0_fixed_joint`，不再成为易混淆的 `joint_7`。
- EAIK Profile 仅在末端 fixed frame 变换与最后可动关节旋转可交换时，才将它折叠
  到最后关节的 origin；实现使用齐次变换组合，并检测轴旋转与垂直偏移。不能等价
  折叠时明确不生成 EAIK 文件，避免输出错误 TCP。
- 新增通用 ROS 2 / MoveIt Profile：生成以根组件名自动派生名称的独立 ROS 2 描述包，
  包含 `base_footprint`、`base_link → tool0` SRDF 规划链、仅含可动关节的
  ros2_control 模板、控制器/初始位姿配置和 ROS 2 package URI mesh。
- 新增 `model_manifest.yaml`，记录导出版本、CAD 文档、link/joint/fixed frame 以及
  输出 URDF 与 mesh 的 SHA256；增加无 Fusion 依赖的稳定命名、EAIK 折叠/拒绝和
  MoveIt Profile 回归测试。
- 公共导出层会归一化可动关节轴并拒绝零轴；碰撞过滤后的普通 link 惯量必须通过
  对称正定性检查。避免继续依赖不说明来源的占位运动参数。
- 传统 ROS Xacro 所需的 `effort`/`velocity` 改为有来源说明的插件默认值，并在
  manifest 中标识其不来自 Fusion 的电机/驱动数据。

## 2026-08-14：v1.4.1 扁平化 ROS 2 输出

- 取消 `ros2/<robot>_description/` 嵌套描述包及重复复制的 mesh；MoveIt Xacro、SRDF、
  控制器与初始位姿配置直接写入外层导出包的 `urdf/`、`config/` 目录，并复用唯一的
  `meshes/`。
- 外层 `package.xml`、`CMakeLists.txt` 由 ROS 2 Profile 直接写为 `ament_cmake`
  描述包元数据。重新导出时，仅自动清理由旧版导出器生成且可识别的嵌套 ROS 2 包，
  不触碰其他用户目录。
