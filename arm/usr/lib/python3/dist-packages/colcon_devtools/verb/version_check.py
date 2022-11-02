# Copyright 2016-2018 Dirk Thomas
# Licensed under the Apache License, Version 2.0

import json
import sys
import urllib
import urllib.request

from colcon_core.entry_point import EXTENSION_POINT_GROUP_NAME
from colcon_core.entry_point import get_all_entry_points
from colcon_core.entry_point import get_entry_points
from colcon_core.plugin_system import satisfies_version
from colcon_core.verb import VerbExtensionPoint
from pkg_resources import parse_version


class VersionCheckVerb(VerbExtensionPoint):
    """Compare local package versions with PyPI."""

    def __init__(self):  # noqa: D107
        super().__init__()
        satisfies_version(VerbExtensionPoint.EXTENSION_POINT_VERSION, '^1.0')

    def main(self, *, context):  # noqa: D102
        distributions = set()
        all_entry_points = get_all_entry_points()
        for group in all_entry_points.values():
            for dist, _ in group.values():
                distributions.add(dist)

        # also consider extension points which don't have any extensions
        colcon_extension_points = get_entry_points(EXTENSION_POINT_GROUP_NAME)
        for entry_point in colcon_extension_points.values():
            distributions.add(entry_point.dist)

        base_url = 'https://pypi.org/pypi/{project}/json'
        for dist in sorted(distributions, key=lambda d: d.project_name):
            url = base_url.format(project=dist.project_name)

            try:
                response = urllib.request.urlopen(url)
            except urllib.error.HTTPError as e:
                if e.code == 404:
                    print(
                        '{dist.project_name}: could not find package on PyPI'
                        .format_map(locals()), file=sys.stderr)
                else:
                    print(
                        '{dist.project_name}: Server could not fulfill the '
                        'request: {e.reason}'
                        .format_map(locals()), file=sys.stderr)
                continue
            except urllib.error.URLError:
                print(
                    '{dist.project_name}: Failed to reach server'
                    .format_map(locals()), file=sys.stderr)
                continue

            try:
                data = json.load(response)
            except json.decoder.JSONDecodeError:
                print(
                    '{dist.project_name}: could not parse PyPI response'
                    .format_map(locals()), file=sys.stderr)
                continue

            try:
                latest_version = data['info']['version']
            except KeyError:
                print(
                    '{dist.project_name}: could not determine version'
                    .format_map(locals()))
                continue

            if parse_version(latest_version) == parse_version(dist.version):
                print(
                    '{dist.project_name} {dist.version}: up-to-date'
                    .format_map(locals()))
                continue
            if parse_version(dist.version) < parse_version(latest_version):
                print(
                    '{dist.project_name} {dist.version}: newer version '
                    'available ({latest_version})'
                    .format_map(locals()))
                continue
            print(
                '{dist.project_name} {dist.version}: local version is newer '
                'than latest release ({latest_version})'
                .format_map(locals()))
            continue
