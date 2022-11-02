# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

from pathlib import Path

from colcon_core.package_descriptor import PackageDescriptor
from colcon_core.package_discovery import PackageDiscoveryExtensionPoint
from colcon_core.plugin_system import satisfies_version
from colcon_core.prefix_path import get_chained_prefix_path
from colcon_core.shell import find_installed_packages
from colcon_installed_package_information.package_discovery import logger


class PrefixPathPackageDiscovery(PackageDiscoveryExtensionPoint):
    """Discover packages in chained prefix paths."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageDiscoveryExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def has_parameters(self, *, args):  # noqa: D102
        return False

    def discover(self, *, args, identification_extensions):  # noqa: D102
        descs = set()

        for priority, prefix_path in enumerate(get_chained_prefix_path()):
            packages = find_installed_packages(Path(prefix_path))
            if packages is None:
                continue
            num_packages = len(packages)
            logger.debug('Found {num_packages} installed packages in '
                         '{prefix_path}'.format_map(locals()))
            for pkg, path in (packages or {}).items():
                desc = PackageDescriptor(path)
                desc.name = pkg
                desc.type = 'installed'
                desc.metadata['override_priority'] = priority
                descs.add(desc)

        return descs
