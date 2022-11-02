# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

from colcon_core.logging import colcon_logger
from colcon_core.package_discovery import \
    add_package_discovery_arguments as add_package_discovery_arguments_impl
from colcon_core.package_discovery import \
    discover_packages as discover_packages_impl
from colcon_core.plugin_system import instantiate_extensions
from colcon_core.plugin_system import order_extensions_by_priority

logger = colcon_logger.getChild(__name__)


def get_package_discovery_extensions():
    """
    Get the available package discovery extensions.

    The extensions are ordered by their priority and entry point name.

    :rtype: OrderedDict
    """
    extensions = instantiate_extensions(__name__)
    for name, extension in extensions.items():
        extension.PACKAGE_DISCOVERY_NAME = name
    return order_extensions_by_priority(extensions)


def add_package_discovery_arguments(parser, *, extensions=None):
    """
    Add the command line arguments for the package discovery extensions.

    :param parser: The argument parser
    :param extensions: The package discovery extensions to use, if `None` is
      passed use the extensions provided by
      :function:`get_package_discovery_extensions`
    """
    if extensions is None:
        extensions = get_package_discovery_extensions()
    add_package_discovery_arguments_impl(parser, extensions=extensions)


def discover_packages(
    args, identification_extensions, *, discovery_extensions=None
):
    """
    Discover and identify packages.

    All discovery extensions which report to have parameters are being used to
    discover packages.
    If none report to have parameters all discovery extensions are being used
    but only the one with a default value should discover packages.
    Each discovery extension uses the passed identification extensions to check
    each potential location for the existence of a package.

    :param args: The parsed command line arguments
    :param identification_extensions: The package identification extensions to
      pass to each invocation of
      :function:`PackageDiscoveryExtensionPoint.discover`
    :param discovery_extensions: The package discovery extensions to use, if
      `None` is passed use the extensions provided by
      :function:`get_package_discovery_extensions`
    :returns: set of
      :py:class:`colcon_core.package_descriptor.PackageDescriptor`
    :rtype: set
    """
    if discovery_extensions is None:
        discovery_extensions = get_package_discovery_extensions()
    return discover_packages_impl(
        args, identification_extensions,
        discovery_extensions=discovery_extensions)
