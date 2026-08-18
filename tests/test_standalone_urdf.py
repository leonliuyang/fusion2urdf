"""Regression tests for standalone URDF output profiles.

Run without Fusion with: ``uv run --with pytest pytest tests``.
"""

import importlib
import importlib.util
import sys
import types
from pathlib import Path
from xml.etree import ElementTree


REPOSITORY_ROOT = Path(__file__).parents[1]
MODULE_FILE = REPOSITORY_ROOT / 'URDF_Exporter' / 'core' / 'StandaloneURDF.py'
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))
SPEC = importlib.util.spec_from_file_location('standalone_urdf', MODULE_FILE)
STANDALONE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(STANDALONE)


def _load_legacy_exporter(monkeypatch):
    """加载传统写出模块，并提供不需要 Fusion 会话的最小 adsk 桩。"""
    adsk_module = types.ModuleType('adsk')
    adsk_core_module = types.ModuleType('adsk.core')
    adsk_fusion_module = types.ModuleType('adsk.fusion')
    adsk_module.core = adsk_core_module
    adsk_module.fusion = adsk_fusion_module
    monkeypatch.setitem(sys.modules, 'adsk', adsk_module)
    monkeypatch.setitem(sys.modules, 'adsk.core', adsk_core_module)
    monkeypatch.setitem(sys.modules, 'adsk.fusion', adsk_fusion_module)
    return (
        importlib.import_module('URDF_Exporter.core.Write'),
        importlib.import_module('URDF_Exporter.utils.utils'),
    )


def test_exported_text_files_use_utf8_lf_and_leave_mesh_bytes_unchanged(tmp_path, monkeypatch):
    """最小导出样例覆盖 XML、Xacro、URDF、YAML、RViz 与二进制网格。"""
    writer, utils = _load_legacy_exporter(monkeypatch)
    package_template = tmp_path / 'template'
    (package_template / 'launch').mkdir(parents=True)
    (package_template / 'meshes').mkdir()
    (package_template / 'CMakeLists.txt').write_bytes(b'project(fusion2urdf)\r\n')
    (package_template / 'LICENSE').write_bytes(b'license\r\n')
    (package_template / 'package.xml').write_bytes(b'<package>\r\n</package>\r\n')
    (package_template / 'launch' / 'urdf.rviz').write_bytes(b'Panels:\r\n')
    stl_bytes = b'\x00STL\r\n\xff\n'
    dae_bytes = b'\x00DAE\r\n\xfe\n'
    (package_template / 'meshes' / 'sample.stl').write_bytes(stl_bytes)
    (package_template / 'meshes' / 'sample.dae').write_bytes(dae_bytes)

    export_directory = tmp_path / 'sample_description'
    export_directory.mkdir()
    joints = {}
    inertial = {
        'base_link': {
            'center_of_mass': [0, 0, 0],
            'mass': 1.0,
            'inertia': [1, 1, 1, 0, 0, 0],
        }
    }
    writer.write_urdf(joints, {}, inertial, 'sample_description', 'sample', str(export_directory))
    writer.write_materials_xacro(joints, {}, inertial, 'sample_description', 'sample', str(export_directory))
    writer.write_transmissions_xacro(joints, {}, inertial, 'sample_description', 'sample', str(export_directory))
    writer.write_gazebo_xacro(joints, {}, inertial, 'sample_description', 'sample', str(export_directory))
    writer.write_display_launch('sample_description', 'sample', str(export_directory))
    writer.write_gazebo_launch('sample_description', 'sample', str(export_directory))
    writer.write_control_launch('sample_description', 'sample', str(export_directory), joints)
    writer.write_yaml('sample_description', 'sample', str(export_directory), joints)
    STANDALONE._write_xml(ElementTree.Element('robot'), export_directory / 'urdf' / 'sample.urdf')
    utils.copy_package(str(export_directory), str(package_template))
    utils.update_cmakelists(str(export_directory), 'sample_description')
    utils.update_package_xml(str(export_directory), 'sample_description')

    expected_text_files = [
        export_directory / 'package.xml',
        export_directory / 'urdf' / 'sample.xacro',
        export_directory / 'urdf' / 'sample.urdf',
        export_directory / 'launch' / 'controller.yaml',
        export_directory / 'launch' / 'urdf.rviz',
    ]
    assert not utils.validate_exported_text_files(str(export_directory))
    for file_name in expected_text_files:
        content = file_name.read_bytes()
        assert b'\r' not in content
        assert content.endswith(b'\n')
        assert not content.endswith(b'\n\n')
        content.decode('utf-8')
    assert (export_directory / 'meshes' / 'sample.stl').read_bytes() == stl_bytes
    assert (export_directory / 'meshes' / 'sample.dae').read_bytes() == dae_bytes


def _robot_with_tool0(offset='0.02 0 0'):
    robot = ElementTree.Element('robot')
    for link_name in ['base_link', 'L7_1', 'tool0']:
        ElementTree.SubElement(robot, 'link', {'name': link_name})
    joint = ElementTree.SubElement(robot, 'joint', {'name': 'fusion_axis', 'type': 'revolute'})
    ElementTree.SubElement(joint, 'parent', {'link': 'base_link'})
    ElementTree.SubElement(joint, 'child', {'link': 'L7_1'})
    ElementTree.SubElement(joint, 'origin', {'xyz': '1 0 0', 'rpy': '0 0 0'})
    ElementTree.SubElement(joint, 'axis', {'xyz': '1 0 0'})
    fixed = ElementTree.SubElement(robot, 'joint', {'name': 'rigid_15', 'type': 'fixed'})
    ElementTree.SubElement(fixed, 'parent', {'link': 'L7_1'})
    ElementTree.SubElement(fixed, 'child', {'link': 'tool0'})
    ElementTree.SubElement(fixed, 'origin', {'xyz': offset, 'rpy': '0 0 0'})
    return robot


def test_normalize_joint_names_uses_kinematic_order_and_semantic_fixed_name():
    robot = _robot_with_tool0()
    STANDALONE._normalize_joint_names(robot)
    joints = robot.findall('joint')
    assert joints[0].get('name') == 'joint_1'
    assert joints[1].get('name') == 'tool0_fixed_joint'


def test_eaik_folds_collinear_tool0_offset():
    robot = _robot_with_tool0()
    tip, info = STANDALONE._fold_virtual_end_frame_for_eaik(robot, 'L7_1')
    joint = robot.find('joint')
    assert tip == 'tool0'
    assert info['from'] == 'L7_1'
    assert joint.find('child').get('link') == 'tool0'
    assert joint.find('origin').get('xyz') == '1.02 0 0'
    assert not any(link.get('name') == 'L7_1' for link in robot.findall('link'))


def test_eaik_rejects_non_collinear_tool0_offset():
    robot = _robot_with_tool0('0 0.02 0')
    try:
        STANDALONE._fold_virtual_end_frame_for_eaik(robot, 'L7_1')
    except ValueError as exception:
        assert 'does not commute' in str(exception)
    else:
        raise AssertionError('A non-collinear end-frame offset must be rejected.')


def test_moveit_profile_contains_base_footprint_tool0_and_six_controlled_joints(tmp_path):
    robot = ElementTree.Element('robot', {'name': 'sample'})
    link_names = ['base_link'] + ['L{}_1'.format(index) for index in range(2, 8)] + ['tool0']
    for link_name in link_names:
        ElementTree.SubElement(robot, 'link', {'name': link_name})
    parent = 'base_link'
    for index, child in enumerate(link_names[1:7], start=1):
        joint = ElementTree.SubElement(robot, 'joint', {
            'name': 'joint_{}'.format(index), 'type': 'revolute'})
        ElementTree.SubElement(joint, 'parent', {'link': parent})
        ElementTree.SubElement(joint, 'child', {'link': child})
        ElementTree.SubElement(joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        ElementTree.SubElement(joint, 'axis', {'xyz': '0 0 1'})
        parent = child
    tool_joint = ElementTree.SubElement(robot, 'joint', {
        'name': 'tool0_fixed_joint', 'type': 'fixed'})
    ElementTree.SubElement(tool_joint, 'parent', {'link': 'L7_1'})
    ElementTree.SubElement(tool_joint, 'child', {'link': 'tool0'})
    ElementTree.SubElement(tool_joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
    (tmp_path / 'urdf').mkdir()

    files, tip = STANDALONE._write_moveit_profile(str(tmp_path), 'Sample Arm', robot)
    moveit_robot = ElementTree.parse(files[0]).getroot()
    control = moveit_robot.find('ros2_control')

    assert tip == 'tool0'
    assert any(link.get('name') == 'base_footprint' for link in moveit_robot.findall('link'))
    assert control is not None
    assert [joint.get('name') for joint in control.findall('joint')] == [
        'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    srdf = ElementTree.parse(files[1]).getroot()
    chain = srdf.find('./group/chain')
    assert chain.get('base_link') == 'base_link'
    assert chain.get('tip_link') == 'tool0'
    assert Path(files[0]).parent == tmp_path / 'urdf'
    assert not (tmp_path / 'ros2').exists()


def test_moveit_profile_removes_only_the_legacy_generated_nested_package(tmp_path):
    legacy = tmp_path / 'ros2' / 'sample_arm_description'
    legacy.mkdir(parents=True)
    (legacy / 'package.xml').write_text(
        '<description>Generated ROS 2 robot description package.</description>',
        encoding='utf-8')
    custom_directory = tmp_path / 'ros2' / 'custom_profile'
    custom_directory.mkdir()

    STANDALONE._remove_legacy_nested_ros2_profile(str(tmp_path), 'sample_arm_description')

    assert not legacy.exists()
    assert custom_directory.exists()


def test_complete_profiles_keep_pinocchio_tool0_and_fold_eaik_tool0(tmp_path):
    package_directory = tmp_path / 'sample_arm_description'
    urdf_directory = package_directory / 'urdf'
    urdf_directory.mkdir(parents=True)
    robot = ElementTree.Element('robot', {'name': 'Sample Arm'})
    link_names = ['base_link'] + ['L{}_1'.format(index) for index in range(2, 8)] + ['tool0']
    for link_name in link_names:
        ElementTree.SubElement(robot, 'link', {'name': link_name})
    parent = 'base_link'
    for index, child in enumerate(link_names[1:7], start=1):
        joint = ElementTree.SubElement(robot, 'joint', {
            'name': 'fusion_joint_{}'.format(index), 'type': 'revolute'})
        ElementTree.SubElement(joint, 'parent', {'link': parent})
        ElementTree.SubElement(joint, 'child', {'link': child})
        ElementTree.SubElement(joint, 'origin', {'xyz': '0.1 0 0', 'rpy': '0 0 0'})
        ElementTree.SubElement(joint, 'axis', {'xyz': '1 0 0'})
        parent = child
    tool_joint = ElementTree.SubElement(robot, 'joint', {
        'name': 'fusion_rigid', 'type': 'fixed'})
    ElementTree.SubElement(tool_joint, 'parent', {'link': 'L7_1'})
    ElementTree.SubElement(tool_joint, 'child', {'link': 'tool0'})
    ElementTree.SubElement(tool_joint, 'origin', {'xyz': '0.02 0 0', 'rpy': '0 0 0'})
    STANDALONE._write_xml(robot, urdf_directory / 'Sample Arm.xacro')

    result = STANDALONE.generate_standalone_urdfs(
        str(package_directory), 'sample_arm_description', 'Sample Arm', '1.4.0', 'Sample Arm')
    pin = ElementTree.parse(urdf_directory / 'Sample Arm_pin.urdf').getroot()
    eaik = ElementTree.parse(urdf_directory / 'Sample Arm_eaik.urdf').getroot()

    assert any(joint.get('name') == 'tool0_fixed_joint' for joint in pin.findall('joint'))
    assert len([joint for joint in eaik.findall('joint') if joint.get('type') != 'fixed']) == 6
    final_joint = [joint for joint in eaik.findall('joint') if joint.get('name') == 'joint_6'][0]
    assert final_joint.find('child').get('link') == 'tool0'
    assert not any(link.get('name') == 'L7_1' for link in eaik.findall('link'))
    manifest = package_directory / 'model_manifest.yaml'
    assert manifest.is_file()
    manifest_text = manifest.read_text(encoding='utf-8')
    assert 'plugin_version: "1.4.0"' in manifest_text
    assert 'tool0_fixed_joint' in manifest_text
    assert 'effort_velocity_source:' in manifest_text
    assert any('ROS 2 / MoveIt' in line for line in result)
