# -*- coding: utf-8 -*-
"""Create standalone URDF files for Pinocchio and EAIK consumers."""

import copy
import hashlib
import math
import os
import shutil
from datetime import datetime, timezone
from xml.etree import ElementTree as ElementTree


XACRO_NAMESPACE = 'http://www.ros.org/wiki/xacro'
VIRTUAL_LINK_NAMES = ('tool0', 'tcp')
EAIK_FOLD_TOLERANCE = 1e-8


def _read_xml(file_name):
    with open(file_name, 'rb') as file_handle:
        return ElementTree.fromstring(file_handle.read().decode('utf-8', errors='replace'))


def _write_xml(robot, file_name):
    if hasattr(ElementTree, 'indent'):
        ElementTree.indent(robot, space='  ')
    xml_content = ElementTree.tostring(
        robot, encoding='unicode', xml_declaration=True)
    _write_utf8_lf(file_name, xml_content)


def _write_utf8_lf(file_name, content):
    """以 UTF-8 和固定 LF 写出文本，且仅保留一个文件末尾换行。"""
    normalized = content.replace('\r\n', '\n').replace('\r', '\n').rstrip('\n') + '\n'
    with open(file_name, 'w', encoding='utf-8', newline='\n') as file_handle:
        file_handle.write(normalized)


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


def _ordered_joints(robot):
    """Return joints in deterministic root-to-tip kinematic order."""
    joints = list(robot.findall('joint'))
    children = set()
    joints_by_parent = {}
    for joint in joints:
        parent_name, child_name = _joint_parent_child(joint)
        if parent_name and child_name:
            children.add(child_name)
            joints_by_parent.setdefault(parent_name, []).append(joint)

    link_names = [link.get('name') for link in robot.findall('link') if link.get('name')]
    roots = sorted(name for name in link_names if name not in children)
    ordered = []
    visited = set()
    pending = list(roots)
    while pending:
        parent_name = pending.pop(0)
        child_joints = sorted(
            joints_by_parent.get(parent_name, []),
            key=lambda joint: (_joint_parent_child(joint)[1], joint.get('name', '')))
        for joint in child_joints:
            marker = id(joint)
            if marker in visited:
                continue
            visited.add(marker)
            ordered.append(joint)
            pending.append(_joint_parent_child(joint)[1])

    ordered.extend(sorted(
        (joint for joint in joints if id(joint) not in visited),
        key=lambda joint: (joint.get('name', ''), _joint_parent_child(joint))))
    return ordered


def _normalize_joint_names(robot):
    """Name movable joints by kinematic order and fixed frames semantically."""
    movable_index = 1
    used_names = set()
    for joint in _ordered_joints(robot):
        _, child_name = _joint_parent_child(joint)
        if joint.get('type') != 'fixed':
            name = 'joint_{}'.format(movable_index)
            movable_index += 1
        elif child_name in VIRTUAL_LINK_NAMES:
            name = '{}_fixed_joint'.format(child_name)
        else:
            name = joint.get('name', 'fixed_joint')
        if name in used_names:
            suffix = 2
            while '{}_{}'.format(name, suffix) in used_names:
                suffix += 1
            name = '{}_{}'.format(name, suffix)
        used_names.add(name)
        joint.set('name', name)


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


def _parse_vector(value, field_name):
    try:
        values = [float(number) for number in (value or '0 0 0').split()]
    except ValueError:
        values = []
    if len(values) != 3:
        raise ValueError('{} must contain three numbers.'.format(field_name))
    return values


def _matrix_from_origin(origin):
    xyz = _parse_vector(origin.get('xyz') if origin is not None else None, 'origin xyz')
    roll, pitch, yaw = _parse_vector(
        origin.get('rpy') if origin is not None else None, 'origin rpy')
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matrix_multiply(left, right):
    return [[sum(left[row][index] * right[index][column] for index in range(4))
             for column in range(4)] for row in range(4)]


def _origin_from_matrix(matrix):
    pitch = math.asin(max(-1.0, min(1.0, -matrix[2][0])))
    if abs(math.cos(pitch)) > 1e-10:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return (
        ' '.join('{:.12g}'.format(matrix[index][3]) for index in range(3)),
        ' '.join('{:.12g}'.format(value) for value in [roll, pitch, yaw]),
    )


def _rotation_vector(matrix, vector):
    return [sum(matrix[row][column] * vector[column] for column in range(3))
            for row in range(3)]


def _cross(left, right):
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def _norm(vector):
    return math.sqrt(sum(value * value for value in vector))


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


def _virtual_fixed_chain(robot, mechanical_tip):
    """Return the contiguous explicit virtual fixed joints after a chain tip."""
    joints_by_parent = {}
    for joint in robot.findall('joint'):
        parent_name, child_name = _joint_parent_child(joint)
        if parent_name and child_name:
            joints_by_parent.setdefault(parent_name, []).append(joint)

    chain = []
    current = mechanical_tip
    while True:
        child_joints = joints_by_parent.get(current, [])
        if len(child_joints) != 1:
            break
        joint = child_joints[0]
        _, child_name = _joint_parent_child(joint)
        if joint.get('type') != 'fixed' or child_name not in VIRTUAL_LINK_NAMES:
            break
        chain.append(joint)
        current = child_name
    return chain


def _fold_virtual_end_frame_for_eaik(robot, mechanical_tip):
    """Fold a commutative fixed virtual-frame chain into the final movable joint.

    EAIK 1.2.2 builds its terminal frame from the child link of the final
    actuated joint and does not apply a following fixed chain.  Replacing the
    final child with the virtual tip is only equivalent when the combined fixed
    transform commutes with that joint's rotation.  Otherwise this function
    deliberately raises rather than writing a silently incorrect EAIK model.
    """
    fixed_chain = _virtual_fixed_chain(robot, mechanical_tip)
    if not fixed_chain:
        return mechanical_tip, None

    final_tip = _joint_parent_child(fixed_chain[-1])[1]
    movable_joints = [
        joint for joint in robot.findall('joint')
        if _joint_parent_child(joint)[1] == mechanical_tip
        and joint.get('type') != 'fixed'
    ]
    if len(movable_joints) != 1:
        raise ValueError(
            'EAIK end-frame folding requires exactly one final movable joint ending at {}.'.format(
                mechanical_tip))
    final_joint = movable_joints[0]
    axis_element = final_joint.find('axis')
    axis = _parse_vector(
        axis_element.get('xyz') if axis_element is not None else None,
        'final joint axis')
    axis_length = _norm(axis)
    if axis_length <= EAIK_FOLD_TOLERANCE:
        raise ValueError('EAIK end-frame folding cannot use a zero final joint axis.')
    axis = [value / axis_length for value in axis]

    fixed_transform = [[1.0 if row == column else 0.0 for column in range(4)]
                       for row in range(4)]
    for joint in fixed_chain:
        fixed_transform = _matrix_multiply(
            fixed_transform, _matrix_from_origin(joint.find('origin')))

    translation = [fixed_transform[index][3] for index in range(3)]
    rotated_axis = _rotation_vector(fixed_transform, axis)
    axis_rotation_error = _norm([
        rotated_axis[index] - axis[index] for index in range(3)])
    axis_offset_error = _norm(_cross(translation, axis))
    if axis_rotation_error > EAIK_FOLD_TOLERANCE or axis_offset_error > EAIK_FOLD_TOLERANCE:
        raise ValueError(
            'EAIK cannot fold the fixed end-frame chain {} -> {}: its transform '
            'does not commute with the final joint rotation (axis rotation error {:.3g}, '
            'perpendicular offset {:.3g} m). Use an EAIK H/P model or an importer '
            'that evaluates terminal fixed joints.'.format(
                mechanical_tip, final_tip, axis_rotation_error, axis_offset_error))

    final_origin = _matrix_from_origin(final_joint.find('origin'))
    folded_origin = _matrix_multiply(final_origin, fixed_transform)
    xyz, rpy = _origin_from_matrix(folded_origin)
    origin = final_joint.find('origin')
    if origin is None:
        origin = ElementTree.SubElement(final_joint, 'origin')
    origin.set('xyz', xyz)
    origin.set('rpy', rpy)
    final_joint.find('child').set('link', final_tip)

    removable_links = {mechanical_tip}
    for joint in fixed_chain[:-1]:
        removable_links.add(_joint_parent_child(joint)[1])
    for joint in fixed_chain:
        robot.remove(joint)
    for link in list(robot.findall('link')):
        if link.get('name') in removable_links:
            robot.remove(link)

    return final_tip, {
        'from': mechanical_tip,
        'to': final_tip,
        'xyz': xyz,
        'rpy': rpy,
    }


def _describe_robot(robot):
    links = robot.findall('link')
    joints = robot.findall('joint')
    movable_joints = [joint for joint in joints if joint.get('type') != 'fixed']
    return '{} links, {} joints ({} movable)'.format(
        len(links), len(joints), len(movable_joints))


def _sha256(file_name):
    digest = hashlib.sha256()
    with open(file_name, 'rb') as file_handle:
        for block in iter(lambda: file_handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _yaml_value(value):
    return '"{}"'.format(str(value).replace('"', '\\"'))


def _write_model_manifest(package_dir, pin_robot, output_files, plugin_version,
                          document_name):
    """Write traceable export metadata without changing the model files."""
    manifest_file = os.path.join(package_dir, 'model_manifest.yaml')
    lines = [
        'format_version: 1',
        'exported_at_utc: {}'.format(_yaml_value(
            datetime.now(timezone.utc).replace(microsecond=0).isoformat())),
        'plugin_version: {}'.format(_yaml_value(plugin_version or 'unknown')),
        'fusion_document: {}'.format(_yaml_value(document_name or 'unknown')),
        'joint_limit_metadata:',
        '  effort_velocity_source: {}'.format(_yaml_value(
            'plugin defaults; Fusion CAD does not provide actuator ratings')),
        'links:',
    ]
    for link in pin_robot.findall('link'):
        if link.get('name'):
            lines.append('  - {}'.format(_yaml_value(link.get('name'))))
    lines.append('joints:')
    for joint in _ordered_joints(pin_robot):
        parent_name, child_name = _joint_parent_child(joint)
        lines.extend([
            '  - name: {}'.format(_yaml_value(joint.get('name'))),
            '    type: {}'.format(_yaml_value(joint.get('type'))),
            '    parent: {}'.format(_yaml_value(parent_name)),
            '    child: {}'.format(_yaml_value(child_name)),
        ])
    lines.append('fixed_frames:')
    for joint in _ordered_joints(pin_robot):
        if joint.get('type') == 'fixed':
            parent_name, child_name = _joint_parent_child(joint)
            lines.append('  - parent: {}'.format(_yaml_value(parent_name)))
            lines.append('    child: {}'.format(_yaml_value(child_name)))
            lines.append('    joint: {}'.format(_yaml_value(joint.get('name'))))
    lines.append('model_files:')
    for file_name in sorted(output_files):
        if os.path.isfile(file_name):
            lines.append('  - path: {}'.format(_yaml_value(
                os.path.relpath(file_name, package_dir).replace('\\', '/'))))
            lines.append('    sha256: {}'.format(_yaml_value(_sha256(file_name))))
    lines.append('mesh_files:')
    mesh_directory = os.path.join(package_dir, 'meshes')
    if os.path.isdir(mesh_directory):
        mesh_files = []
        for directory, _, file_names in os.walk(mesh_directory):
            for file_name in file_names:
                mesh_files.append(os.path.join(directory, file_name))
        for file_name in sorted(mesh_files):
            lines.append('  - path: {}'.format(_yaml_value(
                os.path.relpath(file_name, package_dir).replace('\\', '/'))))
            lines.append('    sha256: {}'.format(_yaml_value(_sha256(file_name))))
    _write_utf8_lf(manifest_file, '\n'.join(lines))
    return manifest_file


def _ros2_identifier(value):
    """Return a conservative ROS 2 package/robot identifier from a CAD name."""
    identifier = ''.join(
        character.lower() if character.isalnum() else '_'
        for character in value).strip('_')
    return identifier or 'fusion_robot'


def _rewrite_moveit_mesh_paths(robot, ros_package_name):
    for mesh in robot.iter('mesh'):
        filename = mesh.get('filename')
        if not filename:
            continue
        normalized = filename.replace('\\', '/')
        marker = '/meshes/'
        if marker in normalized:
            relative_mesh = normalized.split(marker, 1)[1]
        else:
            relative_mesh = os.path.basename(normalized)
        mesh.set('filename', 'package://{}/meshes/{}'.format(
            ros_package_name, relative_mesh))


def _append_base_footprint(robot):
    existing_links = {link.get('name') for link in robot.findall('link')}
    if 'base_link' not in existing_links:
        raise ValueError('MoveIt profile requires a base_link.')
    if 'base_footprint' not in existing_links:
        robot.insert(0, ElementTree.Element('link', {'name': 'base_footprint'}))
    if not any(joint.get('name') == 'base_fixed_joint'
               for joint in robot.findall('joint')):
        joint = ElementTree.SubElement(robot, 'joint', {
            'name': 'base_fixed_joint', 'type': 'fixed'})
        ElementTree.SubElement(joint, 'parent', {'link': 'base_footprint'})
        ElementTree.SubElement(joint, 'child', {'link': 'base_link'})
        ElementTree.SubElement(joint, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})


def _append_ros2_control(robot, robot_id):
    ElementTree.register_namespace('xacro', XACRO_NAMESPACE)
    arg_tag = '{{{}}}arg'.format(XACRO_NAMESPACE)
    robot.insert(0, ElementTree.Element(arg_tag, {
        'name': 'hardware_plugin', 'default': 'mock_components/GenericSystem'}))
    robot.insert(1, ElementTree.Element(arg_tag, {
        'name': 'initial_positions_file', 'default': ''}))

    control = ElementTree.SubElement(robot, 'ros2_control', {
        'name': '{}_system'.format(robot_id), 'type': 'system'})
    hardware = ElementTree.SubElement(control, 'hardware')
    ElementTree.SubElement(hardware, 'plugin').text = '$(arg hardware_plugin)'
    ElementTree.SubElement(hardware, 'param', {
        'name': 'initial_positions_file'}).text = '$(arg initial_positions_file)'
    for joint in _ordered_joints(robot):
        if joint.get('type') == 'fixed':
            continue
        control_joint = ElementTree.SubElement(control, 'joint', {
            'name': joint.get('name')})
        ElementTree.SubElement(control_joint, 'command_interface', {'name': 'position'})
        ElementTree.SubElement(control_joint, 'state_interface', {'name': 'position'})
        ElementTree.SubElement(control_joint, 'state_interface', {'name': 'velocity'})


def _write_moveit_srdf(file_name, robot_id, joints, chain_tip, physical_links):
    robot = ElementTree.Element('robot', {'name': robot_id})
    group = ElementTree.SubElement(robot, 'group', {'name': 'arm'})
    ElementTree.SubElement(group, 'chain', {'base_link': 'base_link', 'tip_link': chain_tip})
    ElementTree.SubElement(robot, 'group_state', {'name': 'home', 'group': 'arm'})
    for joint in joints:
        parent_name, child_name = _joint_parent_child(joint)
        if parent_name in physical_links and child_name in physical_links:
            ElementTree.SubElement(robot, 'disable_collisions', {
                'link1': parent_name, 'link2': child_name, 'reason': 'Adjacent'})
    _write_xml(robot, file_name)


def _write_ros2_controller_config(file_name, movable_joints):
    lines = [
        'controller_manager:',
        '  ros__parameters:',
        '    update_rate: 100',
        '    joint_state_broadcaster:',
        '      type: joint_state_broadcaster/JointStateBroadcaster',
        '    arm_controller:',
        '      type: joint_trajectory_controller/JointTrajectoryController',
        'arm_controller:',
        '  ros__parameters:',
        '    joints:',
    ]
    lines.extend('      - {}'.format(joint.get('name')) for joint in movable_joints)
    lines.extend([
        '    command_interfaces:',
        '      - position',
        '    state_interfaces:',
        '      - position',
        '      - velocity',
    ])
    _write_utf8_lf(file_name, '\n'.join(lines))


def _write_initial_positions(file_name, movable_joints):
    lines = ['initial_positions:']
    lines.extend('  {}: 0.0'.format(joint.get('name')) for joint in movable_joints)
    _write_utf8_lf(file_name, '\n'.join(lines))


def _write_ros2_package_metadata(package_directory, ros_package_name):
    package_xml = os.path.join(package_directory, 'package.xml')
    cmake_lists = os.path.join(package_directory, 'CMakeLists.txt')
    _write_utf8_lf(package_xml, '''<?xml version="1.0"?>
<package format="3">
  <name>{}</name>
  <version>0.1.0</version>
  <description>Generated ROS 2 robot description package.</description>
  <maintainer email="noreply@example.com">Fusion2URDF</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros2_control</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>joint_trajectory_controller</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>'''.format(ros_package_name))
    _write_utf8_lf(cmake_lists, '''cmake_minimum_required(VERSION 3.8)
project({})

find_package(ament_cmake REQUIRED)
install(DIRECTORY config meshes urdf DESTINATION share/${{PROJECT_NAME}})
ament_package()'''.format(ros_package_name))
    return [package_xml, cmake_lists]


def _remove_legacy_nested_ros2_profile(package_dir, ros_package_name):
    """Remove only the old, exporter-generated nested ROS 2 package."""
    legacy_directory = os.path.join(package_dir, 'ros2', ros_package_name)
    metadata_file = os.path.join(legacy_directory, 'package.xml')
    if not os.path.isfile(metadata_file):
        return
    with open(metadata_file, 'r', encoding='utf-8', errors='replace') as file_handle:
        metadata = file_handle.read()
    if '<description>Generated ROS 2 robot description package.</description>' not in metadata:
        return
    shutil.rmtree(legacy_directory)
    legacy_parent = os.path.dirname(legacy_directory)
    if os.path.isdir(legacy_parent) and not os.listdir(legacy_parent):
        os.rmdir(legacy_parent)


def _write_moveit_profile(package_dir, robot_name, pin_robot):
    """Create a generic ROS 2 / MoveIt model profile from the full URDF."""
    robot_id = _ros2_identifier(robot_name)
    ros_package_name = '{}_description'.format(robot_id)
    profile_robot = copy.deepcopy(pin_robot)
    profile_robot.set('name', robot_id)
    _rewrite_moveit_mesh_paths(profile_robot, ros_package_name)
    _append_base_footprint(profile_robot)
    _append_ros2_control(profile_robot, robot_id)

    mechanical_tip = _infer_six_axis_tip(profile_robot)
    chain_tip = _virtual_tip_after(profile_robot, mechanical_tip)
    _remove_legacy_nested_ros2_profile(package_dir, ros_package_name)
    urdf_directory = os.path.join(package_dir, 'urdf')
    config_directory = os.path.join(package_dir, 'config')
    mesh_directory = os.path.join(package_dir, 'meshes')
    for directory in [urdf_directory, config_directory, mesh_directory]:
        if not os.path.isdir(directory):
            os.makedirs(directory)
    moveit_file = os.path.join(urdf_directory, '{}.urdf.xacro'.format(robot_id))
    srdf_file = os.path.join(config_directory, '{}.srdf'.format(robot_id))
    controller_file = os.path.join(config_directory, '{}_ros2_controllers.yaml'.format(robot_id))
    initial_positions_file = os.path.join(
        config_directory, '{}_initial_positions.yaml'.format(robot_id))
    _write_xml(profile_robot, moveit_file)

    movable_joints = [
        joint for joint in _ordered_joints(profile_robot)
        if joint.get('type') != 'fixed'
    ]
    physical_links = {
        link.get('name') for link in pin_robot.findall('link')
        if link.get('name')
    }
    _write_moveit_srdf(srdf_file, robot_id, movable_joints, chain_tip, physical_links)
    _write_ros2_controller_config(controller_file, movable_joints)
    _write_initial_positions(initial_positions_file, movable_joints)
    metadata_files = _write_ros2_package_metadata(
        package_dir, ros_package_name)
    return ([moveit_file, srdf_file, controller_file, initial_positions_file] +
            metadata_files), chain_tip


def generate_standalone_urdfs(package_dir, package_name, robot_name,
                               plugin_version=None, document_name=None):
    """Generate Pinocchio and EAIK URDFs without breaking the main export.

    Returns dialog-ready status lines. The original Xacro is never modified.
    Pinocchio output is the full robot tree. EAIK output is generated only when
    a single six-axis serial chain can be inferred automatically.
    """
    xacro_file = os.path.join(package_dir, 'urdf', '{}.xacro'.format(robot_name))
    pin_file = os.path.join(package_dir, 'urdf', '{}_pin.urdf'.format(robot_name))
    eaik_file = os.path.join(package_dir, 'urdf', '{}_eaik.urdf'.format(robot_name))
    status_lines = ['', 'Standalone URDF conversion:']
    if os.path.isfile(eaik_file):
        os.remove(eaik_file)

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

    profile_files = []
    try:
        profile_files, moveit_tip = _write_moveit_profile(
            package_dir, robot_name, pin_robot)
        status_lines.append('- ROS 2 / MoveIt: {} (tip={}).'.format(
            os.path.basename(profile_files[0]), moveit_tip))
    except Exception as exception:
        status_lines.append('- ROS 2 / MoveIt: not generated ({})'.format(exception))

    try:
        eaik_robot = copy.deepcopy(pin_robot)
        mechanical_tip = _infer_six_axis_tip(eaik_robot)
        tip_link, fold_info = _fold_virtual_end_frame_for_eaik(
            eaik_robot, mechanical_tip)
        _keep_only_chain(eaik_robot, tip_link)
        _write_xml(eaik_robot, eaik_file)
        status_lines.append('- EAIK: {} ({}, tip={})'.format(
            os.path.basename(eaik_file), _describe_robot(eaik_robot), tip_link))
        if fold_info:
            status_lines.append(
                '- EAIK terminal fixed frame folded: {} -> {} (xyz={}, rpy={}).'.format(
                    fold_info['from'], fold_info['to'], fold_info['xyz'], fold_info['rpy']))
    except Exception as exception:
        status_lines.append('- EAIK: not generated ({})'.format(exception))

    manifest_file = _write_model_manifest(
        package_dir, pin_robot, [xacro_file, pin_file, eaik_file] + profile_files, plugin_version,
        document_name)
    status_lines.append('- Manifest: {}.'.format(os.path.basename(manifest_file)))

    return status_lines
