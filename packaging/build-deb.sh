#!/usr/bin/env bash
# Build on Debian/Ubuntu after cargo build --release -p zerocad-gui --locked.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
tag="${1:?usage: build-deb.sh vVERSION [output-directory]}"
version="${tag#v}"
version="${version/-/\~}"
dpkg --validate-version "$version"
arch="$(dpkg --print-architecture)"
out="${2:-$root/dist}"
mkdir -p "$out"
out="$(cd "$out" && pwd)"
stage="$(mktemp -d)"
trap 'rm -rf "$stage"' EXIT
pkg="$stage/package"
install -Dm755 "$root/target/release/zerocad-gui" "$pkg/usr/bin/ZeroCAD"
ln -s ZeroCAD "$pkg/usr/bin/zerocad"
install -Dm644 "$root/packaging/ZeroCAD.desktop" "$pkg/usr/share/applications/ZeroCAD.desktop"
install -Dm644 "$root/packaging/icon.svg" "$pkg/usr/share/icons/hicolor/scalable/apps/zerocad.svg"
for size in 32 48 64 128 256; do
  mkdir -p "$pkg/usr/share/icons/hicolor/${size}x${size}/apps"
  rsvg-convert -w "$size" -h "$size" "$root/packaging/icon.svg" \
    -o "$pkg/usr/share/icons/hicolor/${size}x${size}/apps/zerocad.png"
done
for file in README.md changelog.md LICENSE-MIT LICENSE-APACHE THIRD_PARTY_NOTICES; do
  install -Dm644 "$root/$file" "$pkg/usr/share/doc/zerocad/$file"
done
desktop-file-validate "$pkg/usr/share/applications/ZeroCAD.desktop"
# Resolve linked library requirements from the binary; winit/wgpu also load
# display and Vulkan libraries at runtime, so list those explicitly below.
mkdir -p "$stage/debian" "$pkg/DEBIAN"
printf 'Source: zerocad\nSection: graphics\nPriority: optional\nMaintainer: ZeroCAD contributors <noreply@github.com>\n' > "$stage/debian/control"
deps="$(cd "$stage" && dpkg-shlibdeps -O -e"$pkg/usr/bin/ZeroCAD" | sed -n 's/^shlibs:Depends=//p')"
test -n "$deps"
cat > "$pkg/DEBIAN/control" <<EOF
Package: zerocad
Version: $version
Architecture: $arch
Maintainer: ZeroCAD contributors <noreply@github.com>
Section: graphics
Priority: optional
Installed-Size: $(du -sk "$pkg/usr" | cut -f1)
Depends: $deps, libvulkan1, libxkbcommon0, libwayland-client0, libx11-6, libxcursor1, libxi6, libxrandr2
Recommends: mesa-vulkan-drivers
Homepage: https://github.com/CadeWardC/ZeroCAD
Description: Parametric CAD modeling application
 Native desktop solid modeling with sketches, feature history and assemblies.
EOF
artifact="$out/zerocad_${version}_${arch}.deb"
dpkg-deb --root-owner-group --build "$pkg" "$artifact"
dpkg-deb --info "$artifact"
dpkg-deb --contents "$artifact"
