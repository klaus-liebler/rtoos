#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
#
# Build QEMU with the STM32H563 SoC and the "fcu" machine (Factory Control Unit), the
# STM32G431 SoC with the "stm32g431" machine and the STM32H743 SoC with the "stm32h743" machine
# (bare chips).
#
# Run in an MSYS2 UCRT64 environment (cmd.exe, from the repository root):
#   set MSYSTEM=UCRT64& C:\opt\msys64\usr\bin\bash -l emulation/qemu/build-qemu.sh
# The source tree is unpacked and built in build/qemu of this repository.
#
# Result: a self-contained installation (qemu-system-arm.exe + required DLLs) in
# $PREFIX (default C:\opt\qemu-stm32h5).
#
# Required MSYS2 packages:
#   make diffutils patch mingw-w64-ucrt-x86_64-{gcc,meson,ninja,python,pkgconf,glib2,pixman,dtc,zlib}
set -euo pipefail

QEMU_VERSION=11.1.2
# QEMU release signing key (Michael Roth), as published on qemu.org
QEMU_KEY_FPR=CEACC9E15534EBABB82D3FA03353C9CEF108B584

HERE=$(cygpath -u "$(cd "$(dirname "$0")" && pwd)")
PREFIX=$(cygpath -u "${PREFIX:-C:/opt/qemu-stm32h5}")
WORK=$(cygpath -u "${WORK:-$HERE/../../build/qemu}")
TARBALL=qemu-${QEMU_VERSION}.tar.xz

mkdir -p "$WORK"
cd "$WORK"

# ---- download and verify --------------------------------------------------------
# Downloads go to a .part file and are renamed only when complete: an interrupted download (the
# connection reset) must not stay behind as the tarball, which the next run would take as present.
fetch() {
    [ -f "$1" ] && return 0
    curl -fsSL --retry 5 --retry-all-errors -o "$1.part" "https://download.qemu.org/$1"
    mv "$1.part" "$1"
}
fetch "$TARBALL"
fetch "$TARBALL.sig"
export GNUPGHOME="$WORK/gnupg"
mkdir -p "$GNUPGHOME" && chmod 700 "$GNUPGHOME"
if ! gpg --batch --list-keys "$QEMU_KEY_FPR" >/dev/null 2>&1; then
    # gpg --import exits with 2 when it cannot start gpg-agent (MSYS2), although the public key is
    # imported - only whether the key is there counts
    curl -fsS "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x$QEMU_KEY_FPR" | gpg --batch --import || true
    gpg --batch --list-keys "$QEMU_KEY_FPR" >/dev/null 2>&1 || { echo "error: key $QEMU_KEY_FPR not imported" >&2; exit 1; }
fi
if ! gpg --batch --status-fd 1 --verify "$TARBALL.sig" "$TARBALL" 2>/dev/null | grep -q "VALIDSIG $QEMU_KEY_FPR"; then
    # a damaged download is removed, the next run fetches it again
    rm -f "$TARBALL" "$TARBALL.sig"
    echo "error: $TARBALL is not signed by $QEMU_KEY_FPR (removed; run again to download it anew)" >&2
    exit 1
fi
echo "signature OK ($QEMU_KEY_FPR)"

# ---- source tree: upstream + our models + build integration ---------------------
rm -rf src
mkdir src
# The excluded directories are not needed for an arm-softmmu build on Windows and
# contain symlinks, which Windows cannot create without developer mode.
tar -xf "$TARBALL" -C src --strip-components=1 \
    --exclude="qemu-$QEMU_VERSION/roms" \
    --exclude="qemu-$QEMU_VERSION/tests/lcitool/libvirt-ci" \
    --exclude="qemu-$QEMU_VERSION/rust/hw/char/pl011/build.rs" \
    --exclude="qemu-$QEMU_VERSION/rust/bindings/*-sys/build.rs" \
    --exclude="qemu-$QEMU_VERSION/subprojects/libvduse" \
    --exclude="qemu-$QEMU_VERSION/subprojects/libvhost-user"
cp -r "$HERE/hw" "$HERE/include" src/
for p in "$HERE"/patches/*.patch; do
    patch -d src -p1 --forward < "$p"
done

# ---- configure and build ----------------------------------------------------------
mkdir -p src/build
cd src/build
../configure \
    --prefix="$PREFIX" \
    --target-list=arm-softmmu \
    --without-default-features \
    --enable-fdt=system \
    --enable-trace-backends=log \
    --disable-docs \
    --disable-werror
make -j"$(nproc)"
make install

# ---- make the installation self-contained (copy the MinGW runtime DLLs) ----------
for exe in "$PREFIX"/qemu-system-arm.exe; do
    ldd "$exe" | awk '/\/ucrt64\// { print $3 }' | sort -u | while read -r dll; do
        cp -u "$dll" "$PREFIX/"
    done
done
"$PREFIX/qemu-system-arm.exe" -M help | grep -E "^(fcu|stm32g431) "
echo "installed to $PREFIX"
