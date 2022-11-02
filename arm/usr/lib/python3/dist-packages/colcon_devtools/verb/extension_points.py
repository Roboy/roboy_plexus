# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from colcon_core.entry_point import EXTENSION_POINT_GROUP_NAME
from colcon_core.entry_point import get_entry_points
from colcon_core.entry_point import load_entry_point
from colcon_core.plugin_system import get_first_line_doc
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


class ExtensionPointsVerb(VerbExtensionPoint):
    """List extension points."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        parser.add_argument(
            '--all', '-a',
            action='store_true',
            default=False,
            help='Also show extension points which failed to be imported. '
                 '(prefixed with `- `)')
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            default=False,
            help='Show more information for each extension point')

    def main(self, *, context):  # noqa: D102
        colcon_extension_points = get_entry_points(EXTENSION_POINT_GROUP_NAME)
        for name in sorted(colcon_extension_points.keys()):
            # skip "private" extension points
            if name.startswith('_'):
                continue
            self._print_extension_point(
                context.args, name, colcon_extension_points[name])

    def _print_extension_point(self, args, name, entry_point):
        exception = None
        try:
            extension_point = load_entry_point(entry_point)
        except Exception as e:  # noqa: B902
            # catch exceptions raised when loading entry point
            if not args.all:
                # skip entry points which failed to load
                return
            exception = e
            extension_point = None

        prefix = '' if exception is None else '- '
        print(prefix + name + ':', get_first_line_doc(extension_point))

        if args.verbose:
            print(prefix, ' ', 'module_name:', entry_point.module_name)
            if entry_point.attrs:
                print(prefix, ' ', 'attributes:', '.'.join(entry_point.attrs))
            if hasattr(extension_point, 'EXTENSION_POINT_VERSION'):
                print(
                    prefix, ' ', 'version:',
                    extension_point.EXTENSION_POINT_VERSION)

        if exception:
            print(prefix, ' ', 'reason:', str(exception))
