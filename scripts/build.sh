#!/usr/bin/env bash
# Builds, installs and tests the ROS-free ALICA packages with plain CMake.
# Each package is installed into ./install so the next one can find_package() it.
#
# Usage: scripts/build.sh [--clean] [--no-test] [package ...]
# TEST_JOBS sets how many tests run in parallel (default: number of CPUs).
# TEST_EXCLUDE is a regular expression of test names to skip.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$ROOT/build"
INSTALL="$ROOT/install"

PACKAGES=(
  alica_solver_interface
  alica_engine
  alica_dynamic_loading
  alica_standard_library
  alica_simple_solver
  alica_dummy_proxy
  alica_test_utility
  alica_tests
  supplementary/autodiff
  supplementary/constraintsolver
  supplementary/alica_turtlesim
)

clean=0
test=1
selected=()
for arg in "$@"; do
  case "$arg" in
    --clean) clean=1 ;;
    --no-test) test=0 ;;
    -h|--help) sed -n 2,7p "$0"; exit 0 ;;
    *) selected+=("$arg") ;;
  esac
done
if [ ${#selected[@]} -gt 0 ]; then PACKAGES=("${selected[@]}"); fi

if [ "$clean" = 1 ]; then rm -rf "$BUILD" "$INSTALL"; fi

if command -v nproc >/dev/null; then JOBS=$(nproc); else JOBS=$(sysctl -n hw.ncpu); fi
TEST_JOBS="${TEST_JOBS:-$JOBS}"
TEST_ARGS=(-j "$TEST_JOBS")
if [ -n "${TEST_EXCLUDE:-}" ]; then TEST_ARGS+=(-E "$TEST_EXCLUDE"); fi
GENERATOR=(-G "Unix Makefiles")
if command -v ninja >/dev/null; then GENERATOR=(-G Ninja); fi

PREFIX_PATH="$INSTALL"
if command -v brew >/dev/null; then PREFIX_PATH="$INSTALL;$(brew --prefix)"; fi

failed=()
for pkg in "${PACKAGES[@]}"; do
  name="$(basename "$pkg")"
  echo "==> $name"
  cmake -S "$ROOT/$pkg" -B "$BUILD/$name" "${GENERATOR[@]}" \
    -DCMAKE_INSTALL_PREFIX="$INSTALL" \
    -DCMAKE_PREFIX_PATH="$PREFIX_PATH" \
    -DBUILD_TESTING="$([ "$test" = 1 ] && echo ON || echo OFF)"
  cmake --build "$BUILD/$name" -j "$JOBS"
  cmake --install "$BUILD/$name"
  if [ "$test" = 1 ] && [ -f "$BUILD/$name/CTestTestfile.cmake" ]; then
    ctest --test-dir "$BUILD/$name" --output-on-failure "${TEST_ARGS[@]}" || failed+=("$name")
  fi
done

if [ ${#failed[@]} -gt 0 ]; then
  echo "Tests failed in: ${failed[*]}" >&2
  exit 1
fi
echo "All packages built."
