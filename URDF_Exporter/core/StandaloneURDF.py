# -*- coding: utf-8 -*-
"""Create standalone URDF files for Pinocchio and EAIK consumers."""

import copy
import os
from xml.etree import ElementTree as ElementTree


XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'
VIRTUAL_LINK_NAMES = ('tool0', 'tcp')


def _read_xml(file_name):
    with open(file_name, 'rb') as file_handle:
        return ElementTree.fromstring(file_handle.read().decode('utf-8', errors='replace'))


def _write_xml(robot, file_name):
    if hasattr(ElementTree, 'indent'):
        ElementTree.indent(robot, space='  ')
    ElementTree.ElementTree(robot).write(
        file_name, encoding='utf-8', xml_declaration=True)


def _strip_xacro_includes(robot):
    include_tag = '{{{}}}include'.format(XACRO_NAMESPACE)
    for child in list(robot):
        if child.tag == include_tag:
            robot.remove(child)


def _append_materials(robot, package_dir):
    materials_file = os.path.join(package_dir, 'urdf', 'materials.xacro')
    if not os.path.isfile(materials_file):
        return

    materials_robot = _read_xml(materials_file)
    for child in reversed(list(materials_robot)):
        if child.tag == 'material':
            robot.insert(0, copy.deepcopy(child))


def _remove_ros_control_tags(robot):
    for child in list(robot):
        if child.tag in ['gazebo', 'transmission']:
            robot.remove(child)


def _rewrite_mesh_paths(robot, package_dir, package_name, output_file):
    package_prefix = 'package://{}/'.format(package_name)
    output_directory = os.path.dirname(os.path.abspath(output_file))
    for mesh in robot.iter('mesh'):
        filename = mesh.get('filename')
        if not filename or not filename.startswith(package_prefix):
            continue

        mesh_file = os.path.join(package_dir, filename[len(package_prefix):])
        mesh.set('filename', os.path.relpath(mesh_file, output_directory).replace('\\', '/'))


def _normalize_joint_names(robot):
    for index, joint in enumerate(robot.findall('joint'), start=1):
        joint.set('name', 'joint_{}'.format(index))


def _build_standalone_urdf(package_dir, package_name, xacro_file, output_file):
    robot = _read_xml(xacro_file)
    _strip_xacro_includes(robot)
    _append_materials(robot, package_dir)
    _remove_ros_control_tags(robot)
    _rewrite_mesh_paths(robot, package_dir, package_name, output_file)
    _normalize_joint_names(robot)
    return robot


def _joint_parent_child(joint):
    parent = joint.find('parent')
    child = joint.find('child')
    return (
        parent.get('link') if parent is not None else None,
        child.get('link') if child is not None else None,
    )


def _infer_six_axis_tip(robot):
    joints_by_parent = {}
    child_links = set()
    for joint in robot.findall('joint'):
        parent_name, child_name = _joint_parent_child(joint)
        if parent_name and child_name:
            joints_by_parent.setdefault(parent_name, []).append(joint)
            child_links.add(child_name)

    roots = [
        link.get('name') for link in robot.findall('link')
        if link.get('name') and link.get('name') not in child_links
    ]
    if len(roots) != 1:
        raise ValueError('无法自动识别 EAIK 链：URDF 必须只有一个根 link。')

    current = roots[0]
    movable_joint_count = 0
    while movable_joint_count < 6:
        child_joints = joints_by_parent.get(current, [])
        if len(child_joints) != 1:
            raise ValueError(
                '无法自动识别 EAIK 六轴串联链：第 {} 个可动关节处不是唯一子链。'.format(
                    movable_joint_count + 1))
        joint = child_joints[0]
        _, current = _joint_parent_child(joint)
        if joint.get('type') != 'fixed':
            movable_joint_count += 1
    return current


def _keep_only_chain(robot, tip_link):
    links = {
        link.get('name'): link for link in robot.findall('link')
        if link.get('name')
    }
    if tip_link not in links:
        raise ValueError('EAIK 末端 link 不存在：{}。'.format(tip_link))

    parent_by_child = {}
    joint_by_child = {}
    for joint in robot.findall('joint'):
        parent_name, child_name = _joint_parent_child(joint)
        if parent_name and child_name:
            parent_by_child[child_name] = parent_name
            joint_by_child[child_name] = joint.get('name')

    kept_links = {tip_link}
    kept_joint_names = set()
    current = tip_link
    while current in parent_by_child:
        kept_joint_names.add(joint_by_child[current])
        current = parent_by_child[current]
        kept_links.add(current)

    for child in list(robot):
        if child.tag == 'link' and child.get('name') not in kept_links:
            robot.remove(child)
        elif child.tag == 'joint' and child.get('name') not in kept_joint_names:
            robot.remove(child)


def _virtual_tip_after(robot, mechanical_tip):
    """Return the last explicit tool0/tcp frame after the six-axis tip."""
    joints_by_parent = {}
    for joint in robot.findall('joint'):
        parent_name, child_name = _joint_parent_child(joint)
        if parent_name and child_name:
            joints_by_parent.setdefault(parent_name, []).append(joint)

    current = mechanical_tip
    while True:
        child_joints = joints_by_parent.get(current, [])
        if len(child_joints) != 1:
            break
        joint = child_joints[0]
        _, child_name = _joint_parent_child(joint)
        if joint.get('type') != 'fixed' or child_name not in VIRTUAL_LINK_NAMES:
            break
        current = child_name
    return current


def _describe_robot(robot):
    links = robot.findall('link')
    joints = robot.findall('joint')
    movable_joints = [joint for joint in joints if joint.get('type') != 'fixed']
    return '{} links, {} joints ({} movable)'.format(
        len(links), len(joints), len(movable_joints))


def generate_standalone_urdfs(package_dir, package_name, robot_name):
    """Generate Pinocchio and EAIK URDFs without breaking the main export.

    Returns dialog-ready status lines. The original Xacro is never modified.
    Pinocchio output is the full robot tree. EAIK output is generated only when
    a single six-axis serial chain can be inferred automatically.
    """
    xacro_file = os.path.join(package_dir, 'urdf', '{}.xacro'.format(robot_name))
    pin_file = os.path.join(package_dir, 'urdf', '{}_pin.urdf'.format(robot_name))
    eaik_file = os.path.join(package_dir, 'urdf', '{}_eaik.urdf'.format(robot_name))
    status_lines = ['', 'Standalone URDF conversion:']

    try:
        pin_robot = _build_standalone_urdf(
            package_dir, package_name, xacro_file, pin_file)
        _write_xml(pin_robot, pin_file)
        status_lines.append('- Pinocchio: {} ({})'.format(
            os.path.basename(pin_file), _describe_robot(pin_robot)))
    except Exception as exception:
        status_lines.append('- Pinocchio: failed ({})'.format(exception))
        status_lines.append('- EAIK: skipped because Pinocchio conversion failed.')
        return status_lines

    try:
        eaik_robot = copy.deepcopy(pin_robot)
        mechanical_tip = _infer_six_axis_tip(eaik_robot)
        tip_link = _virtual_tip_after(eaik_robot, mechanical_tip)
        _keep_only_chain(eaik_robot, tip_link)
        _write_xml(eaik_robot, eaik_file)
        status_lines.append('- EAIK: {} ({}, tip={})'.format(
            os.path.basename(eaik_file), _describe_robot(eaik_robot), tip_link))
    except Exception as exception:
        status_lines.append('- EAIK: not generated ({})'.format(exception))

    return status_lines
