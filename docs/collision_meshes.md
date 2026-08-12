# Collision Mesh 导出设计

## 目标

导出器支持轻量的碰撞几何体，而不降低连杆 visual 网格的质量。Fusion 中名称以
`collision_` 开头的 body 被识别为仅碰撞几何体。

## 建模约定

将碰撞 body 与连杆的普通 body 放在同一个顶层连杆组件中；它们可以直接位于该
组件下，也可以位于其静态子组件中。例如 `arm_link` 可包含 `arm_housing`、
`motor` 子组件和 `collision_box`。

- 普通 body 合并导出为 `meshes/<link>.stl`，供 `<visual>` 使用。
- `collision_` body 单独导出为
  `meshes/collision/<link>__<body>__<n>.stl`。
- 生成的 Xacro 为每个碰撞 body 写入一个 `<collision>`；其 link 原点和 STL
  缩放比例与 visual 一致。
- 连杆没有 `collision_` body 时保持旧行为：collision 复用 visual STL。

前缀匹配不区分大小写。不要依赖 Fusion 的可见性小眼睛：隐藏 body 仍会按名称
规则导出。每个连杆至少应有一个不以 `collision_` 开头的 body；只有碰撞体的
连杆会报错，因为它没有 visual 网格和物理质量。

仅导出从 `base_link` 沿 Fusion joint 的 parent → child 方向可达的连杆。与该树
不连通的顶层组件（例如螺丝、治具或参考件）不会参与惯量计算，也不会生成 STL。
已导出连杆内部的静态子组件（例如电机或减速器）则会被递归纳入该连杆的 visual
网格、质量和惯量；这不表示嵌套组件可作为独立运动 link 使用。

## 物理属性

碰撞 body 不参与导出的质量、质心或惯量计算，无须在 Fusion 中改材料或设为零
密度。导出器会以高精度读取 occurrence 装配坐标系下每个普通 body 的属性，累加
世界坐标系惯量，再围绕合并质心进行平行轴转换。

## 坐标与临时几何体

STL 导出以包含源 body 副本的临时 root occurrence 为目标。这样 visual 和
collision 走相同的 occurrence 导出路径，避免组件局部坐标和装配坐标不一致。
临时 occurrence 在 `finally` 中删除，因此不会重命名源组件为 `old_component`，
也不会在导出结束后留下副本。

## 手动验证

选用一个经过关节旋转、平移的连杆，并创建明显的 `collision_box`。导出后检查
Xacro 中的两个网格路径是否存在；在 RViz 或 Gazebo 开启碰撞显示，确认碰撞形状
与目标连杆重合。仅新增碰撞 body 前后，质量与惯量应保持不变。

Fusion 的结束对话框会显示每个导出连杆的质量，并列出没有 `collision_` body、
因此使用 visual STL 作为 collision 的连杆。该质量清单不含 collision body，
并同时显示纳入计算的普通 body 数量和排除的 collision body 数量，可用于核验
碰撞体没有污染物理属性。
