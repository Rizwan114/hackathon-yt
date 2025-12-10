#!/usr/bin/env python3

"""
Validation script for URDF models
This script validates URDF models using the URDFValidator class
"""

import sys
import os
from pathlib import Path

# Add the urdf_models module to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../python/urdf_models'))

from urdf_validator import URDFValidator


def validate_urdf_model(urdf_path):
    """
    Validate a URDF model file

    Args:
        urdf_path: Path to the URDF file to validate

    Returns:
        Tuple of (is_valid, list_of_errors)
    """
    # Read the URDF file
    try:
        with open(urdf_path, 'r', encoding='utf-8') as f:
            urdf_content = f.read()
    except Exception as e:
        return False, [f"Error reading URDF file: {e}"]

    # Validate the URDF
    validator = URDFValidator()
    is_valid, errors = validator.validate_urdf_model(urdf_content)

    return is_valid, errors


def main():
    """
    Main function to validate the humanoid URDF model
    """
    urdf_path = os.path.join(os.path.dirname(__file__), 'humanoid_model.urdf')

    print(f"Validating URDF model: {urdf_path}")

    is_valid, errors = validate_urdf_model(urdf_path)

    if is_valid:
        print("✅ URDF model is valid!")

        # Get model info
        validator = URDFValidator()
        model_info = validator.get_model_info(open(urdf_path, 'r', encoding='utf-8').read())

        if model_info:
            print(f"Model name: {model_info['name']}")
            print(f"Links: {model_info['link_count']}")
            print(f"Joints: {model_info['joint_count']}")
            print(f"Sensors: {model_info['sensor_count']}")

        return 0
    else:
        print("❌ URDF model has validation errors:")
        for error in errors:
            print(f"  - {error}")
        return 1


if __name__ == '__main__':
    sys.exit(main())