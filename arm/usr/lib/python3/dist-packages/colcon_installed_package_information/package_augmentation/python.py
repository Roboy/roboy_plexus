# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

import sysconfig

from colcon_core.package_augmentation \
    import PackageAugmentationExtensionPoint
from colcon_core.package_augmentation.python \
    import create_dependency_descriptor
from colcon_core.plugin_system import satisfies_version
from pkg_resources import Environment
from pkg_resources import Requirement


class InstalledPythonPackageAugmentation(PackageAugmentationExtensionPoint):
    """
    Augment installed packages with Python distribution information.

    Only packages of the `installed` type are considered.
    """

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageAugmentationExtensionPoint.EXTENSION_POINT_VERSION,
            '^1.0')

    def augment_packages(  # noqa: D102
        self, descs, *, additional_argument_names=None
    ):
        descs = {d for d in descs if d.type == 'installed'}
        if not descs:
            return

        environments = {}
        for desc in descs:
            key = Requirement.parse(desc.name).key
            for lib_dir in _enumerate_python_dirs(str(desc.path)):
                if lib_dir not in environments:
                    environments[lib_dir] = Environment([lib_dir])
                dist = next(iter(environments[lib_dir][key]), None)
                if dist:
                    break
            else:
                continue

            if dist.version and not desc.metadata.get('version'):
                desc.metadata['version'] = dist.version
            desc.type = 'installed.python'
            desc.dependencies['run'].update(
                create_dependency_descriptor(str(req))
                for req in dist.requires())


def _enumerate_python_dirs(prefix):
    get_path_vars = {'base': prefix, 'platbase': prefix}
    yield sysconfig.get_path('purelib', vars=get_path_vars)
    yield sysconfig.get_path('platlib', vars=get_path_vars)
