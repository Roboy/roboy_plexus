# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

import os

from colcon_core.dependency_descriptor import DependencyDescriptor
from colcon_core.location import get_relative_package_index_path
from colcon_core.package_augmentation \
    import PackageAugmentationExtensionPoint
from colcon_core.plugin_system import satisfies_version


class ColconIndexPackageAugmentation(PackageAugmentationExtensionPoint):
    """
    Augment installed packages with information from a colcon index.

    Only packages of the `installed` type are considered.
    """

    # This extension adds dependency information for installed packages more
    # efficiently than the Python extension, so it should have a higher
    # higher priority than it.
    PRIORITY = 120

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')

    def augment_package(  # noqa: D102
        self, desc, *, additional_argument_names=None
    ):
        if desc.type != 'installed':
            return

        marker_file = desc.path / get_relative_package_index_path() / desc.name
        if not marker_file.is_file():
            return

        with marker_file.open() as f:
            raw_deps = f.read().split(os.pathsep)
        desc.type = 'installed.colcon'
        desc.dependencies['run'].update(
            DependencyDescriptor(dep) for dep in raw_deps)
