# Copyright 2022 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0

from colcon_core.package_augmentation \
    import augment_packages as augment_packages_impl
from colcon_core.plugin_system import instantiate_extensions
from colcon_core.plugin_system import order_extensions_by_priority


def get_package_augmentation_extensions():
    """
    Get the available package augmentation extensions.

    The extensions are ordered by their priority and entry point name.

    :rtype: OrderedDict
    """
    extensions = instantiate_extensions(__name__)
    for name, extension in extensions.items():
        extension.PACKAGE_AUGMENTATION_NAME = name
    return order_extensions_by_priority(extensions)


def augment_packages(
    descs, *, additional_argument_names=None, augmentation_extensions=None
):
    """
    Augment package descriptors with additional information.

    :param descs: the packages
    :type descs: set of
      :py:class:`colcon_core.package_descriptor.PackageDescriptor`
    """
    if augmentation_extensions is None:
        augmentation_extensions = get_package_augmentation_extensions()

    augment_packages_impl(
        descs, additional_argument_names=additional_argument_names,
        augmentation_extensions=augmentation_extensions)
