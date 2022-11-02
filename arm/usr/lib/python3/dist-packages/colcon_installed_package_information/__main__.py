# Copyright 2022 Scott K Logan
# Licensed under the Apache License, Version 2.0

import logging

from colcon_core.logging import colcon_logger
from colcon_installed_package_information.package_augmentation \
    import augment_packages
from colcon_installed_package_information.package_discovery \
    import discover_packages
from colcon_installed_package_information.package_identification \
    import get_package_identification_extensions


def main():
    """
    PEP 338 entry point used for testing this module.

    This function will enumerate all of the installed packages and print them
    to the console in a similar way to the `colcon list` command. Because this
    functionality provides only limited utility outside of debugging, it is
    not exposed as a full colcon verb.
    """
    colcon_logger.setLevel(logging.WARNING)

    # Enumerate the underlay packages.
    identification_extensions = get_package_identification_extensions()
    descriptors = discover_packages(None, identification_extensions)
    augment_packages(descriptors)

    colcon_logger.debug('Found {} packages'.format(len(descriptors)))

    lines = []
    for pkg in sorted(descriptors, key=lambda pkg: pkg.name):
        lines.append(pkg.name + '\t' + str(pkg.path) + '\t(%s)' % pkg.type)

    for line in lines:
        print(line)


if __name__ == '__main__':
    main()
