# -*- coding: utf-8 -*-
"""
Created on Sun May 12 20:11:28 2019

@author: syuntoku
"""

import adsk, re
from xml.etree.ElementTree import Element, SubElement
from ..utils import utils

class Link:

    def __init__(self, name, xyz, center_of_mass, repo, mass, inertia_tensor,
                 collision_meshes=None, is_virtual=False):
        """
        Parameters
        ----------
        name: str
            name of the link
        xyz: [x, y, z]
            coordinate for the visual and collision
        center_of_mass: [x, y, z]
            coordinate for the center of mass
        link_xml: str
            generated xml describing about the link
        repo: str
            the name of the repository to save the xml file
        mass: float
            mass of the link
        inertia_tensor: [ixx, iyy, izz, ixy, iyz, ixz]
            tensor of the inertia
        """
        self.name = name
        # xyz for visual
        self.xyz = [-_ for _ in xyz]  # reverse the sign of xyz
        # xyz for center of mass
        self.center_of_mass = center_of_mass
        self.link_xml = None
        self.repo = repo
        self.mass = mass
        self.inertia_tensor = inertia_tensor
        self.collision_meshes = collision_meshes or []
        self.is_virtual = is_virtual
        
    def make_link_xml(self):
        """
        Generate the link_xml and hold it by self.link_xml
        """
        
        link = Element('link')
        link.attrib = {'name':self.name}
        if self.is_virtual:
            self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])
            return
        
        #inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz':' '.join([str(_) for _ in self.center_of_mass]), 'rpy':'0 0 0'}       
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value':str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = \
            {'ixx':str(self.inertia_tensor[0]), 'iyy':str(self.inertia_tensor[1]),\
            'izz':str(self.inertia_tensor[2]), 'ixy':str(self.inertia_tensor[3]),\
            'iyz':str(self.inertia_tensor[4]), 'ixz':str(self.inertia_tensor[5])}        
        
        # visual
        visual = SubElement(link, 'visual')
        origin_v = SubElement(visual, 'origin')
        origin_v.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.attrib = {'filename':'package://' + self.repo + self.name + '.stl','scale':'0.001 0.001 0.001'}
        material = SubElement(visual, 'material')
        material.attrib = {'name':'silver'}
        
        # Collision meshes are optional.  If none were modelled explicitly,
        # retain the legacy behavior of reusing the visual mesh.
        collision_meshes = self.collision_meshes or [self.name + '.stl']
        for collision_mesh in collision_meshes:
            collision = SubElement(link, 'collision')
            origin_c = SubElement(collision, 'origin')
            origin_c.attrib = {'xyz':' '.join([str(_) for _ in self.xyz]), 'rpy':'0 0 0'}
            geometry_c = SubElement(collision, 'geometry')
            mesh_c = SubElement(geometry_c, 'mesh')
            mesh_c.attrib = {
                'filename':'package://' + self.repo + collision_mesh,
                'scale':'0.001 0.001 0.001'
            }

        # print("\n".join(utils.prettify(link).split("\n")[1:]))
        self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])


def make_inertial_dict(root, msg, link_names=None):
    """      
    Parameters
    ----------
    root: adsk.fusion.Design.cast(product)
        Root component
    msg: str
        Tell the status
        
    Returns
    ----------
    inertial_dict: {name:{mass, inertia, center_of_mass}}
    
    msg: str
        Tell the status
    """
    # Get component properties. Collision bodies are deliberately excluded:
    # they describe simplified contact geometry, not physical material.
    allOccs = root.occurrences
    link_names = set(link_names) if link_names else None
    inertial_dict = {}
    
    for occs in allOccs:
        # Skip the root component.
        link_name = utils.occurrence_link_name(occs)
        if link_names is not None and link_name not in link_names:
            continue

        occs_dict = {}
        occs_dict['name'] = link_name

        all_bodies = utils.occurrence_bodies(occs)
        if utils.is_virtual_link_occurrence(occs):
            if all_bodies:
                msg = ('Virtual link {} must not contain BRep bodies. Use a sketch and '
                       'Joint Origin only, or rename the physical component.').format(occs.name)
                return {}, msg
            occs_dict['is_virtual'] = True
            inertial_dict[link_name] = occs_dict
            continue

        physical_bodies = []
        collision_body_count = 0
        for body in all_bodies:
            if not utils.is_collision_body(body):
                physical_bodies.append(body)
            else:
                collision_body_count += 1

        if not physical_bodies:
            msg = ('{} has no non-collision body. Each link must contain at '
                   'least one body whose name does not start with "{}".').format(
                       occs.name, utils.COLLISION_PREFIX)
            return {}, msg

        # Individual body properties are expressed in the root assembly
        # context because occurrence body collections return body proxies. Sum
        # their world moments, then use the existing parallel-axis conversion once.
        mass = 0.0
        weighted_center = [0.0, 0.0, 0.0]
        moment_inertia_world = [0.0] * 6
        for body in physical_bodies:
            prop = body.getPhysicalProperties(
                adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
            body_mass = prop.mass
            body_center = [_ / 100.0 for _ in prop.centerOfMass.asArray()]
            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            body_moment = [_ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz]]

            mass += body_mass
            weighted_center = [total + body_mass * coordinate for total, coordinate
                               in zip(weighted_center, body_center)]
            moment_inertia_world = [total + value for total, value
                                    in zip(moment_inertia_world, body_moment)]

        if mass <= 0.0:
            msg = '{} has zero physical mass after collision filtering.'.format(occs.name)
            return {}, msg

        center_of_mass = [coordinate / mass for coordinate in weighted_center]
        occs_dict['mass'] = mass
        occs_dict['center_of_mass'] = center_of_mass
        occs_dict['physical_body_count'] = len(physical_bodies)
        occs_dict['collision_body_count'] = collision_body_count
        occs_dict['inertia'] = utils.origin2center_of_mass(
            moment_inertia_world, center_of_mass, mass)
        if not utils.is_positive_definite_inertia(occs_dict['inertia']):
            msg = ('{} has a non-positive-definite inertia tensor after collision '
                   'filtering. Check the physical bodies and materials.').format(occs.name)
            return {}, msg
        
        if link_name == 'base_link':
            inertial_dict['base_link'] = occs_dict
        else:
            inertial_dict[link_name] = occs_dict

    if link_names is not None:
        missing_links = sorted(link_names.difference(inertial_dict))
        if missing_links:
            msg = 'Could not find Fusion occurrences for: {}.'.format(
                ', '.join(missing_links))
            return {}, msg

    return inertial_dict, msg
