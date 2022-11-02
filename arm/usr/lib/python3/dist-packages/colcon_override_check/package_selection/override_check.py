# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

import argparse
from pathlib import Path
import sys

from colcon_core.location import get_relative_package_index_path
from colcon_core.package_selection import logger
from colcon_core.package_selection import PackageSelectionExtensionPoint
from colcon_core.plugin_system import satisfies_version
from colcon_installed_package_information.package_augmentation \
    import augment_packages
from colcon_installed_package_information.package_discovery \
    import discover_packages
from colcon_installed_package_information.package_identification \
    import get_package_identification_extensions


if sys.version_info < (3, 8):
    # TODO(sloretz) remove when minimum supported Python version is 3.8
    # https://stackoverflow.com/a/41153081
    class _ExtendAction(argparse.Action):
        """Add argparse action to extend a list."""

        def __call__(self, parser, namespace, values, option_string=None):
            """Extend the list with new arguments."""
            items = getattr(namespace, self.dest) or []
            items.extend(values)
            setattr(namespace, self.dest, items)


class OverrideCheckPackageSelection(PackageSelectionExtensionPoint):
    """Check for potential problems when overriding installed packages."""

    # Because this is effectively observing the selected packages, this
    # extension should run after other package selection extensions
    PRIORITY = 10

    TARGET_VERBS = ('build',)

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(
            PackageSelectionExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        if sys.version_info < (3, 8):
            # TODO(sloretz) remove when minimum supported Python version is 3.8
            parser.register('action', 'extend', _ExtendAction)

        parser.add_argument(
            '--allow-overriding',
            action='extend',
            default=[],
            metavar='PKG_NAME',
            nargs='+',
            help='Allow building packages that exist in underlay workspaces')

    def check_parameters(self, args, pkg_names):  # noqa: D102
        for pkg_name in args.allow_overriding or []:
            if pkg_name not in pkg_names:
                logger.warning(
                    "ignoring unknown package '{pkg_name}' in "
                    '--allow-overriding'.format_map(locals()))

    def select_packages(self, args, decorators):  # noqa: D102
        verb_name = getattr(args, 'verb_name', None)
        if verb_name not in OverrideCheckPackageSelection.TARGET_VERBS:
            return

        # Enumerate the names of packages selected to be built.
        selected_names = {d.descriptor.name for d in decorators if d.selected}

        # Enumerate the underlay packages.
        identification_extensions = get_package_identification_extensions()
        underlay_descriptors = discover_packages(
            args, identification_extensions)
        augment_packages(underlay_descriptors)
        if not underlay_descriptors:
            return

        # Enumerate workspace packages already built (if applicable).
        building_or_built = set(selected_names)
        if hasattr(args, 'install_base'):
            install_base = Path(args.install_base).resolve()

            # Drop any enumerated underlay packages in our install base.
            # This could happen when a workspace install base is activated and
            # then built again.
            rebuilding = [
                d for d in underlay_descriptors if d.path == install_base]
            underlay_descriptors.difference_update(rebuilding)
            if not underlay_descriptors:
                return
            building_or_built.update(d.name for d in rebuilding)

            # Explicitly enumerate which workspace packages have already been
            # built/installed in case the install base hasn't already been
            # activated.
            index_path = install_base / get_relative_package_index_path()
            if index_path.is_dir():
                building_or_built.update(p.name for p in index_path.iterdir())

        # Enumerate dependencies from underlay packages to selected packages.
        reverse_dependencies = {}
        for d in underlay_descriptors:
            # If an underlay package is being built or has already been built,
            # its dependencies are irrelevant because it is already, or will
            # be, overridden itself.
            if d.name in building_or_built:
                continue

            # Consider only runtime dependencies from underlay packages to
            # selected packages.
            deps = {
                d.name for d in d.get_dependencies(categories=('run',))
                if d.name in selected_names}

            for dep in deps:
                reverse_dependencies.setdefault(dep, set())
                reverse_dependencies[dep].add(d.name)

        # Compute the list of paths where potentially overridden packages are
        # installed to.
        underlay_paths = {}
        for d in underlay_descriptors:
            underlay_paths.setdefault(d.name, [])
            underlay_paths[d.name].append(str(d.path))

        # Finally, determine what packages are overriding underlay packages
        overriding_names = selected_names.intersection(underlay_paths)

        # Of those, ignore specifically allowed overrides
        overriding_names.difference_update(args.allow_overriding)

        # Of those, Ignore overrides which are (effectively) leaf packages
        overriding_names.intersection_update(
            name for name, deps in reverse_dependencies.items()
            if deps)

        override_messages = {}
        for overriding_package in overriding_names:
            override_messages[overriding_package] = (
                "'{overriding_package}' is in: ".format_map(locals()) +
                ', '.join(underlay_paths[overriding_package]))

        if override_messages:
            override_msg = (
                'Some selected packages are already built in one or more'
                ' underlay workspaces:'
                '\n\t' +
                '\n\t'.join(override_messages.values()) +
                '\nIf a package in a merged underlay workspace is overridden'
                ' and it installs headers, then all packages in the overlay'
                ' must sort their include directories by workspace order.'
                ' Failure to do so may result in build failures or undefined'
                ' behavior at run time.'
                '\nIf the overridden package is used by another package'
                ' in any underlay, then the overriding package in the'
                ' overlay must be API and ABI compatible or undefined'
                ' behavior at run time may occur.'
                '\n\nIf you understand the risks and want to override a'
                ' package anyways, add the following to the command'
                ' line:'
                '\n\t--allow-overriding ' +
                ' '.join(sorted(override_messages.keys())))

            logger.warn(
                override_msg + '\n\nThis may be promoted to an error in a'
                ' future release of colcon-override-check.')
