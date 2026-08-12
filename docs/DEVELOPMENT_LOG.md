# 开发日志

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
