"""
URDF validation utilities for the Physical AI Book
"""
import xml.etree.ElementTree as ET
from typing import Dict, List, Tuple, Optional
import logging

logger = logging.getLogger(__name__)

class URDFValidator:
    """
    Utility class for validating URDF models
    """

    def __init__(self):
        """
        Initialize the URDF validator
        """
        logger.info("Initializing URDF Validator")

    def validate_urdf_syntax(self, urdf_content: str) -> Tuple[bool, List[str]]:
        """
        Validate the basic XML syntax of a URDF file

        Args:
            urdf_content: String content of the URDF file

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        try:
            # Parse the XML to check basic syntax
            root = ET.fromstring(urdf_content)
        except ET.ParseError as e:
            errors.append(f"XML syntax error: {str(e)}")
            return False, errors
        except Exception as e:
            errors.append(f"Error parsing URDF: {str(e)}")
            return False, errors

        # Check if root element is robot
        if root.tag != 'robot':
            errors.append("Root element must be 'robot'")
            return False, errors

        # Check robot name attribute
        if 'name' not in root.attrib:
            errors.append("Robot element must have a 'name' attribute")
            return False, errors

        return True, errors

    def validate_robot_structure(self, urdf_content: str) -> Tuple[bool, List[str]]:
        """
        Validate the basic structure of a robot in URDF

        Args:
            urdf_content: String content of the URDF file

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        errors = []

        try:
            root = ET.fromstring(urdf_content)
        except Exception:
            # If syntax is invalid, structure validation doesn't make sense
            return False, ["URDF syntax is invalid, cannot validate structure"]

        # Find all links and joints
        links = root.findall('link')
        joints = root.findall('joint')

        if len(links) == 0:
            errors.append("URDF must contain at least one link")
        else:
            # Check that each link has required elements
            for link in links:
                if 'name' not in link.attrib:
                    errors.append(f"Link element missing 'name' attribute")

        if len(joints) == 0:
            errors.append("URDF should contain at least one joint to connect links")
        else:
            # Check that each joint has required elements
            for joint in joints:
                if 'name' not in joint.attrib:
                    errors.append(f"Joint element missing 'name' attribute")
                if 'type' not in joint.attrib:
                    errors.append(f"Joint element missing 'type' attribute")
                if len(joint.findall('parent')) == 0:
                    errors.append(f"Joint missing 'parent' element")
                if len(joint.findall('child')) == 0:
                    errors.append(f"Joint missing 'child' element")

        # Check for a proper kinematic chain
        if len(links) > 0 and len(joints) == 0:
            errors.append("Multiple links without joints to connect them")

        return len(errors) == 0, errors

    def validate_urdf_model(self, urdf_content: str) -> Tuple[bool, List[str]]:
        """
        Validate a complete URDF model

        Args:
            urdf_content: String content of the URDF file

        Returns:
            Tuple of (is_valid, list_of_errors)
        """
        all_errors = []

        # Validate syntax first
        syntax_valid, syntax_errors = self.validate_urdf_syntax(urdf_content)
        all_errors.extend(syntax_errors)

        if not syntax_valid:
            return False, all_errors

        # Validate structure
        structure_valid, structure_errors = self.validate_robot_structure(urdf_content)
        all_errors.extend(structure_errors)

        return structure_valid, all_errors

    def get_model_info(self, urdf_content: str) -> Optional[Dict[str, any]]:
        """
        Extract information about the URDF model

        Args:
            urdf_content: String content of the URDF file

        Returns:
            Dictionary with model information or None if invalid
        """
        try:
            root = ET.fromstring(urdf_content)
        except Exception:
            return None

        # Count links and joints
        links = root.findall('link')
        joints = root.findall('joint')
        materials = root.findall('material')
        transmissions = root.findall('transmission')

        # Find sensors (gazebo tags with sensor elements)
        sensors = []
        for gazebo_elem in root.findall('gazebo'):
            for sensor_elem in gazebo_elem.findall('.//sensor'):
                sensors.append(sensor_elem)

        return {
            'name': root.attrib.get('name', ''),
            'link_count': len(links),
            'joint_count': len(joints),
            'material_count': len(materials),
            'transmission_count': len(transmissions),
            'sensor_count': len(sensors),
            'links': [link.attrib.get('name', '') for link in links],
            'joints': [joint.attrib.get('name', '') for joint in joints]
        }