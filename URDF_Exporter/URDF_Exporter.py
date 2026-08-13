#Author-syuntoku14
#Description-Generate URDF file from Fusion 360

import adsk, adsk.core, adsk.fusion, traceback
import os
import sys
from .utils import utils
from .core import Link, Joint, StandaloneURDF, Write

PLUGIN_VERSION = '1.4.0'

"""
# length unit is 'cm' and inertial unit is 'kg/cm^2'
# If there is no 'body' in the root component, maybe the corrdinates are wrong.
"""

# joint effort: 100
# joint velocity: 100
# supports "Revolute", "Rigid" and "Slider" joint types

# I'm not sure how prismatic joint acts if there is no limit in fusion model

def run(context):
    ui = None
    success_msg = 'Successfully create URDF file'
    msg = success_msg
    
    try:
        # --------------------
        # initialize
        app = adsk.core.Application.get()
        ui = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Fusion2URDF'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        root = design.rootComponent  # root component 

        # set the names        
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'
        save_dir = utils.file_dialog(ui)
        if save_dir == False:
            ui.messageBox('Fusion2URDF was canceled', title)
            return 0

        save_dir = save_dir + '/' + package_name
        try: os.mkdir(save_dir)
        except: pass     

        package_dir = os.path.abspath(os.path.dirname(__file__)) + '/package/'
        
        # --------------------
        # set dictionaries
        
        # Generate the Fusion joint list, then keep only the directed tree
        # reachable from base_link. Loose hardware is not part of the URDF.
        joints_dict, msg = Joint.make_joints_dict(root, msg)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        joints_dict, link_names = Joint.connected_joints_and_links(joints_dict)

        # Generate inertial_dict
        inertial_dict, msg = Link.make_inertial_dict(root, msg, link_names)
        if msg != success_msg:
            ui.messageBox(msg, title)
            return 0
        elif not 'base_link' in inertial_dict:
            msg = 'There is no base_link. Please set base_link and run again.'
            ui.messageBox(msg, title)
            return 0

        virtual_links, virtual_link_error = utils.virtual_link_info(
            joints_dict, inertial_dict)
        if virtual_link_error:
            ui.messageBox(virtual_link_error, title)
            return 0
        
        links_xyz_dict = {}
        
        # --------------------
        # Generate URDF and STL files. Temporary occurrences preserve the
        # coordinate behavior of the legacy exporter without renaming or
        # retaining copies of the user's source components.
        visual_exports = []
        collision_exports = []
        conversion_status = []
        try:
            visual_exports, collision_exports, collision_meshes = \
                utils.prepare_mesh_exports(root, link_names)

            Write.write_urdf(joints_dict, links_xyz_dict, inertial_dict,
                             package_name, robot_name, save_dir, collision_meshes)
            Write.write_materials_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_transmissions_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_gazebo_xacro(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
            Write.write_display_launch(package_name, robot_name, save_dir)
            Write.write_gazebo_launch(package_name, robot_name, save_dir)
            Write.write_control_launch(package_name, robot_name, save_dir, joints_dict)
            Write.write_yaml(package_name, robot_name, save_dir, joints_dict)

            # Copy the catkin package template before placing mesh files in it.
            utils.copy_package(save_dir, package_dir)
            utils.update_cmakelists(save_dir, package_name)
            utils.update_package_xml(save_dir, package_name)
            utils.export_stl(design, save_dir, visual_exports, collision_exports)
            conversion_status = StandaloneURDF.generate_standalone_urdfs(
                save_dir, package_name, robot_name, PLUGIN_VERSION, root.name)
        finally:
            utils.cleanup_mesh_exports(visual_exports, collision_exports)
        
        ui.messageBox(utils.export_summary(
            link_names, inertial_dict, collision_meshes, conversion_status,
            virtual_links), title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
