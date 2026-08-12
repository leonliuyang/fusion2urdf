# Repository Guidelines

## 项目结构与模块

`URDF_Exporter/` 是 Fusion 360 脚本目录：`URDF_Exporter.py` 为入口，`core/` 负责从 Fusion API 提取关节和连杆并写出 URDF/Xacro，`utils/` 放置文件、STL 导出和 XML 辅助函数。`package/` 是导出时复制的 ROS/catkin 包模板。`Example/Basic_Robot.f3d` 和 `Example/Basic_Robot_description/` 分别是示例 CAD 模型及其预期导出包（URDF、网格和 launch 文件）。

## 构建、测试与本地开发

脚本必须在 Fusion 360 中运行；Windows 安装副本可用：

```powershell
Copy-Item .\URDF_Exporter "$env:APPDATA\Autodesk\Autodesk Fusion 360\API\Scripts\" -Recurse
```

在 Fusion 的 **Add-Ins** 中运行 `fusion2urdf`，用示例或小型装配体手动验证导出。验证生成的 ROS 包时，在 catkin 工作区执行 `catkin_make`，随后运行 `roslaunch <robot_name>_description display.launch`；需要仿真时运行 `gazebo.launch`。项目当前没有可运行的单元测试或覆盖率门槛，因此改动须附带相应的手动验证说明。

## 编码风格与命名

沿用现有 Python 风格：四空格缩进、函数/变量使用 `snake_case`、类使用 `PascalCase`，常量仅在确有必要时用 `UPPER_CASE`。保持 Fusion API 调用与 XML 生成逻辑分开；新增写出逻辑优先放入 `core/Write.py`，共享操作放入 `utils/utils.py`。不要无关地重排旧代码；修改路径或复制逻辑时保持 Windows 和 macOS 兼容，并使用 Python 3.12 支持的标准库。

## 测试准则

对导出器改动，至少验证：`base_link`、Rigid/Revolute/Slider 关节、STL 网格路径，以及生成包能通过 `catkin_make`。对 XML 输出变更，检查 `Example/Basic_Robot_description/urdf/` 中的 Xacro 是否仍可由 ROS 启动。测试脚本命名为 `test_<feature>.py`，避免依赖真实 Fusion 会话的测试应可独立执行。

## 提交与拉取请求

近期历史使用简短的祈使式主题，例如 `Fix compatibility with Python 3.12`、`Update Joint.py`。每个提交聚焦一项变更；标题说明对象和结果。拉取请求应说明问题、实现和验证的 Fusion/ROS 版本；如导出结果改变，附上生成的 Xacro 差异或 RViz/Gazebo 截图，并关联相关 issue。

## Fusion 与安全提示

运行导出器会修改 Fusion 设计，先备份模型。组件应直接包含 bodies，避免嵌套组件；把基础组件命名为 `base_link`，并在提交前避免加入生成的网格或本地 ROS 构建产物，除非它们是有意更新的示例资产。
