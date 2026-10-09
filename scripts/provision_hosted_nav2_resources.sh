#!/bin/bash
# SPDX-License-Identifier: Apache-2.0
# Extract the official resource-only package, without installing its dependencies.
set -euo pipefail

validate_prefix() {
    local prefix=$1
    local markers=("$prefix"/share/ament_index/resource_index/packages/*)
    if [[ "$prefix" != /* || ! -f "$prefix/share/nav2_bringup/package.xml" ||
          ! -f "$prefix/share/nav2_bringup/launch/localization_launch.py" ||
          ! -f "$prefix/share/nav2_bringup/rviz/nav2_default_view.rviz" ||
          ${#markers[@]} != 1 || ${markers[0]##*/} != nav2_bringup ||
          ! -f ${markers[0]} || -e "$prefix/lib" || -e "$prefix/bin" ]]; then
        printf 'Invalid Nav2 resource prefix: %s\n' "$prefix" >&2
        return 2
    fi
}

if [[ ${1:-} == --validate && $# == 2 ]]; then
    validate_prefix "$2"
    exit
fi
[[ $# == 0 ]] || { printf 'Usage: %s [--validate PREFIX]\n' "$0" >&2; exit 2; }
staging=$(mktemp -d "${RUNNER_TEMP:-${TMPDIR:-/tmp}}/factory-nav2-resources.XXXXXX")
cd "$staging"
apt-get download ros-humble-nav2-bringup
packages=(ros-humble-nav2-bringup_*.deb)
[[ ${#packages[@]} == 1 && -f ${packages[0]} ]] || exit 2
package=${packages[0]}
[[ $(dpkg-deb --field "$package" Package) == ros-humble-nav2-bringup ]] || exit 2
dpkg-deb --field "$package" Package Version Architecture
sha256sum "$package"
dpkg-deb --extract "$package" "$staging/resources"
prefix="$staging/resources/opt/ros/humble"
validate_prefix "$prefix"
printf 'Official Nav2 resource prefix: %s\n' "$prefix"
if [[ -n ${GITHUB_ENV:-} ]]; then
    printf 'FACTORY_AMR_NAV2_RESOURCE_PREFIX=%s\n' "$prefix" >> "$GITHUB_ENV"
fi
