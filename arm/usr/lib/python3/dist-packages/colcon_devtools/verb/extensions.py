# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

from contextlib import suppress

from colcon_core.entry_point import get_all_entry_points
from colcon_core.entry_point import load_entry_point
from colcon_core.plugin_system import get_first_line_doc
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint


class ExtensionsVerb(VerbExtensionPoint):
    """List extensions."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def add_arguments(self, *, parser):  # noqa: D102
        all_entry_points = get_all_entry_points()
        parser.add_argument(
            'group_name',
            nargs='?',
            choices=sorted(all_entry_points.keys()),
            metavar='GROUP_NAME',
            help='Only show the extensions in a specific group')
        parser.add_argument(
            '--all', '-a',
            action='store_true',
            default=False,
            help='Also show extensions which failed to load or are '
                 'incompatible. (prefixed with `- `)')
        parser.add_argument(
            '--verbose', '-v',
            action='store_true',
            default=False,
            help='Show more information for each extension')

    def main(self, *, context):  # noqa: D102
        all_entry_points = get_all_entry_points()
        for group_name in sorted(all_entry_points.keys()):
            if context.args.group_name is not None:
                if group_name != context.args.group_name:
                    continue
            print(group_name)
            group = all_entry_points[group_name]
            for entry_point_name in sorted(group.keys()):
                (dist, entry_point) = group[entry_point_name]
                self._print_entry_point(context.args, dist, entry_point)

    def _print_entry_point(self, args, dist, entry_point):
        exception = None
        try:
            extension = load_entry_point(entry_point)
        except Exception as e:  # noqa: B902
            # catch exceptions raised when loading entry point
            if not args.all:
                # skip entry points which failed to load
                return
            exception = e
            extension = None
        else:
            try:
                extension()
            except Exception as e:  # noqa: B902
                # catch exceptions raised in extension constructor
                if not args.all:
                    # skip extensions which failed to be instantiated
                    return
                exception = e

        prefix = ' ' if exception is None else '-'
        print(prefix, entry_point.name + ':', get_first_line_doc(extension))

        if args.verbose:
            print(prefix, ' ', 'module_name:', entry_point.module_name)
            if entry_point.attrs:
                print(prefix, ' ', 'attributes:', '.'.join(entry_point.attrs))
            print(prefix, ' ', 'distribution:', repr(dist))
            with suppress(AttributeError):
                print(prefix, ' ', 'priority:', extension.PRIORITY)

        if exception:
            print(prefix, ' ', 'reason:', str(exception))
