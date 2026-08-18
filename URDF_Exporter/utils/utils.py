# -*- coding: utf-8 -*-
"""
Created on Sun May 12 19:15:34 2019

@author: syuntoku
"""

import adsk, adsk.core, adsk.fusion
import math
import os.path, re
from xml.etree import ElementTree
from xml.dom import minidom
import shutil  # Replaced distutils with shutil

COLLISION_PREFIX = 'collision_'
VIRTUAL_LINK_NAMES = ('tool0', 'tcp')
TEXT_EXPORT_EXTENSIONS = frozenset({
    '.cfg', '.cmake', '.gazebo', '.ini', '.json', '.launch', '.rviz', '.srdf',
    '.trans', '.txt', '.urdf', '.xacro', '.xml', '.yaml', '.yml',
})
TEXT_EXPORT_FILENAMES = frozenset({'CMakeLists.txt', 'LICENSE'})


def is_export_text_file(file_name):
    """判断导出目录中的文件是否属于插件生成的文本文件。"""
    return (os.path.basename(file_name) in TEXT_EXPORT_FILENAMES or
            os.path.splitext(file_name)[1].lower() in TEXT_EXPORT_EXTENSIONS)


def open_export_text(file_name, mode='w'):
    """以固定的 UTF-8 与 LF 规则打开导出文本，绝不依赖系统默认值。"""
    return open(file_name, mode=mode, encoding='utf-8', newline='\n')


def write_export_text(file_name, content):
    """写入 UTF-8/LF 文本，并确保文件结尾恰好保留一个 LF。"""
    normalized = content.replace('\r\n', '\n').replace('\r', '\n').rstrip('\n') + '\n'
    with open_export_text(file_name) as file_handle:
        file_handle.write(normalized)


def normalize_exported_text_files(directory):
    """规范化复制模板后的文本；STL、DAE 等非文本文件完全不读取或改写。"""
    for root, _, file_names in os.walk(directory):
        for file_name in file_names:
            path = os.path.join(root, file_name)
            if not is_export_text_file(path):
                continue
            with open(path, 'r', encoding='utf-8', newline=None) as file_handle:
                content = file_handle.read()
            write_export_text(path, content)


def validate_exported_text_files(directory):
    """返回导出文本的编码、换行符及末尾换行校验错误列表。"""
    errors = []
    for root, _, file_names in os.walk(directory):
        for file_name in file_names:
            path = os.path.join(root, file_name)
            if not is_export_text_file(path):
                continue
            with open(path, 'rb') as file_handle:
                content = file_handle.read()
            try:
                content.decode('utf-8')
            except UnicodeDecodeError:
                errors.append('{} 不是 UTF-8 文本。'.format(path))
                continue
            if b'\r' in content:
                errors.append('{} 包含 CRLF 或孤立 CR。'.format(path))
            if not content.endswith(b'\n') or content.endswith(b'\n\n'):
                errors.append('{} 文件末尾不是恰好一个 LF。'.format(path))
    return errors


def is_collision_body(body):
    """Return whether a Fusion body is reserved for collision geometry."""
    return body.name.lower().startswith(COLLISION_PREFIX)


def normalize_axis(axis, context):
    """Return a unit axis or fail before writing an invalid URDF."""
    length = math.sqrt(sum(value * value for value in axis))
    if length <= 1e-12:
        raise ValueError('{} has a zero-length axis.'.format(context))
    return [round(value / length, 12) for value in axis]


def is_positive_definite_inertia(inertia_tensor):
    """Check Sylvester's criterion for a symmetric 3×3 inertia tensor."""
    if len(inertia_tensor) != 6 or not all(math.isfinite(value) for value in inertia_tensor):
        return False
    ixx, iyy, izz, ixy, iyz, ixz = inertia_tensor
    scale = max(abs(ixx), abs(iyy), abs(izz), 1e-16)
    epsilon = scale * 1e-12
    leading_minor_2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz))
    return ixx > epsilon and leading_minor_2 > epsilon ** 2 and determinant > epsilon ** 3


def occurrence_link_name(occurrence):
    """Return the link name used by the generated URDF for an occurrence."""
    if occurrence.component.name == 'base_link':
        return 'base_link'
    component_name = occurrence.component.name.lower()
    if component_name in VIRTUAL_LINK_NAMES:
        return component_name
    return re.sub('[ :()]', '_', occurrence.name)


def is_virtual_link_occurrence(occurrence):
    """Return whether an occurrence represents a reserved virtual end frame."""
    return occurrence.component.name.lower() in VIRTUAL_LINK_NAMES


def virtual_link_info(joints_dict, inertial_dict):
    """Validate explicit Fusion virtual links and return dialog details."""
    details = []
    for link_name, properties in inertial_dict.items():
        if not properties.get('is_virtual'):
            continue
        matching_joints = [
            (joint_name, joint) for joint_name, joint in joints_dict.items()
            if joint['child'] == link_name
        ]
        if len(matching_joints) != 1:
            return [], 'Virtual link {} must have exactly one parent joint.'.format(link_name)
        joint_name, joint = matching_joints[0]
        if joint['type'] != 'fixed':
            return [], 'Virtual link {} must be connected using a Rigid (fixed) Fusion joint.'.format(
                link_name)
        details.append({
            'name': link_name,
            'parent': joint['parent'],
            'joint': joint_name,
        })
    return details, None


def occurrence_bodies(occurrence):
    """Return body proxies from an occurrence and all of its descendants.

    A robot link can contain nested static components, such as a motor,
    gearbox, or fasteners. Their bodies belong to the link's visual geometry
    and physical properties even though they are not direct bodies of the
    top-level link occurrence.
    """
    bodies = []
    pending_occurrences = [occurrence]
    while pending_occurrences:
        current_occurrence = pending_occurrences.pop()
        for i in range(current_occurrence.bRepBodies.count):
            bodies.append(current_occurrence.bRepBodies.item(i))
        for i in range(current_occurrence.childOccurrences.count):
            pending_occurrences.append(current_occurrence.childOccurrences.item(i))
    return bodies


def _mesh_stem(link_name, body_name, index):
    """Create a deterministic, filesystem-safe collision mesh filename stem."""
    safe_body_name = re.sub('[^A-Za-z0-9_.-]', '_', body_name)
    return '{}__{}__{}'.format(link_name, safe_body_name, index)


def prepare_mesh_exports(root, link_names=None):
    """Create temporary root occurrences for visual and collision STL export.

    The copies preserve the exporter's existing occurrence-based coordinate
    behavior, while leaving the source components and their names untouched.
    The caller must pass the returned occurrences to ``cleanup_mesh_exports``.
    """
    visual_exports = []
    collision_exports = []
    collision_meshes = {}
    all_occurrences = root.occurrences
    link_names = set(link_names) if link_names else None

    # Snapshot the collection because new temporary occurrences are appended to it.
    source_occurrences = [all_occurrences.item(i) for i in range(all_occurrences.count)]
    for occurrence in source_occurrences:
        bodies = occurrence_bodies(occurrence)
        if not bodies:
            continue

        link_name = occurrence_link_name(occurrence)
        if link_names is not None and link_name not in link_names:
            continue
        visual_bodies = []
        collision_bodies = []
        for body in bodies:
            if is_collision_body(body):
                collision_bodies.append(body)
            else:
                visual_bodies.append(body)

        if visual_bodies:
            visual_occurrence = all_occurrences.addNewComponent(adsk.core.Matrix3D.create())
            visual_occurrence.component.name = link_name
            for body in visual_bodies:
                body.copyToComponent(visual_occurrence)
            visual_exports.append((link_name, visual_occurrence))

        for index, body in enumerate(collision_bodies, start=1):
            mesh_stem = _mesh_stem(link_name, body.name, index)
            collision_occurrence = all_occurrences.addNewComponent(adsk.core.Matrix3D.create())
            collision_occurrence.component.name = mesh_stem
            body.copyToComponent(collision_occurrence)
            collision_exports.append((mesh_stem, collision_occurrence))
            collision_meshes.setdefault(link_name, []).append(
                'collision/{}.stl'.format(mesh_stem))

    return visual_exports, collision_exports, collision_meshes


def export_summary(link_names, inertial_dict, collision_meshes,
                   conversion_status=None, virtual_links=None):
    """Create the final Fusion dialog text for collision and mass verification."""
    lines = ['Successfully created URDF files.', '', 'Exported link masses:']
    for link_name in link_names:
        link_properties = inertial_dict[link_name]
        if link_properties.get('is_virtual'):
            continue
        lines.append('- {}: {:.6f} kg ({} physical bodies; {} collision bodies excluded)'.format(
            link_name,
            link_properties['mass'],
            link_properties['physical_body_count'],
            link_properties['collision_body_count']))

    fallback_links = [
        link_name for link_name in link_names
        if not inertial_dict[link_name].get('is_virtual')
        and not collision_meshes.get(link_name)
    ]
    lines.extend(['', 'Collision meshes:'])
    if fallback_links:
        lines.append('The following links have no collision_ body; their visual mesh is used for collision:')
        lines.extend('- {}'.format(link_name) for link_name in fallback_links)
    else:
        lines.append('All exported links use dedicated collision_ bodies.')

    lines.extend([
        '',
        'Bodies whose names start with collision_ are excluded from the masses above and from inertia calculations.'
    ])
    if virtual_links:
        lines.extend(['', 'Virtual end frames from Fusion:'])
        for virtual_link in virtual_links:
            lines.append(
                '- {name}: parent={parent}, fixed joint={joint}; no mass, inertia, visual, collision, or STL.'.format(
                    **virtual_link))
    if conversion_status:
        lines.extend(conversion_status)
    return '\n'.join(lines)


def cleanup_mesh_exports(visual_exports, collision_exports):
    """Delete temporary occurrences created by ``prepare_mesh_exports``."""
    for _, occurrence in visual_exports + collision_exports:
        if occurrence and occurrence.isValid:
            occurrence.deleteMe()


def _export_occurrence_stl(export_manager, occurrence, file_name):
    stl_export_options = export_manager.createSTLExportOptions(occurrence, file_name)
    stl_export_options.sendToPrintUtility = False
    stl_export_options.isBinaryFormat = True
    stl_export_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
    export_manager.execute(stl_export_options)


def export_stl(design, save_dir, visual_exports, collision_exports):
    """
    export stl files into "save_dir/"
    
    Parameters
    ----------
    design: adsk.fusion.Design.cast(product)
    save_dir: str
        directory path to save
    visual_exports: list of ``(link_name, occurrence)`` pairs
    collision_exports: list of ``(mesh_stem, occurrence)`` pairs
    """
          
    # create a single exportManager instance
    exportMgr = design.exportManager
    mesh_dir = save_dir + '/meshes'
    collision_dir = mesh_dir + '/collision'
    for directory in [mesh_dir, collision_dir]:
        try: os.mkdir(directory)
        except: pass

    for link_name, occurrence in visual_exports:
        try:
            print(link_name)
            _export_occurrence_stl(exportMgr, occurrence, mesh_dir + '/' + link_name)
        except:
            print('Component ' + link_name + ' has something wrong.')

    for mesh_stem, occurrence in collision_exports:
        try:
            print(mesh_stem)
            _export_occurrence_stl(exportMgr, occurrence, collision_dir + '/' + mesh_stem)
        except:
            print('Collision mesh ' + mesh_stem + ' has something wrong.')


def file_dialog(ui):     
    """
    display the dialog to save the file
    """
    # Set styles of folder dialog.
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Fusion Folder Dialog' 
    
    # Show folder dialog
    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    convert the moment of the inertia about the world coordinate into 
    that about center of mass coordinate

    Parameters
    ----------
    moment of inertia about the world coordinate:  [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    
    Returns
    ----------
    moment of inertia about center of mass : [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [y**2 + z**2, x**2 + z**2, x**2 + y**2,
                         -x*y, -y*z, -x*z]
    return [i - mass*t for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """
    Return a pretty-printed XML string for the Element.
    
    Parameters
    ----------
    elem : xml.etree.ElementTree.Element
    
    Returns
    ----------
    pretified xml : str
    """
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    try:
        # Check if the target directory exists, if not, create it
        if not os.path.exists(save_dir + '/launch'):
            os.mkdir(save_dir + '/launch')
        if not os.path.exists(save_dir + '/urdf'):
            os.mkdir(save_dir + '/urdf')
        
        # Check if the package directory exists and copy it
        if os.path.exists(package_dir):
            shutil.copytree(package_dir, save_dir, dirs_exist_ok=True)  # dirs_exist_ok=True allows overwriting
            normalize_exported_text_files(save_dir)
        else:
            print(f"Package directory '{package_dir}' does not exist.")
        
    except Exception as e:
        print(f"Error copying package: {e}")


def update_cmakelists(save_dir, package_name):
    file_name = save_dir + '/CMakeLists.txt'
    with open(file_name, 'r', encoding='utf-8', newline=None) as file_handle:
        lines = file_handle.readlines()
    lines = [
        "project(" + package_name + ")\n"
        if 'project(fusion2urdf)' in line else line
        for line in lines
    ]
    write_export_text(file_name, ''.join(lines))


def update_package_xml(save_dir, package_name):
    file_name = save_dir + '/package.xml'
    with open(file_name, 'r', encoding='utf-8', newline=None) as file_handle:
        lines = file_handle.readlines()
    lines = [
        "  <name>" + package_name + "</name>\n" if '<name>' in line else
        "<description>The " + package_name + " package</description>\n"
        if '<description>' in line else line
        for line in lines
    ]
    write_export_text(file_name, ''.join(lines))
