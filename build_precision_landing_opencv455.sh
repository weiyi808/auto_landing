#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${1:-/home/wxh/nb_code/autofly_ws}"
OPENCV_PREFIX="/usr/local/opencv-455-cuda"
OPENCV_CMAKE="${OPENCV_PREFIX}/lib/cmake/opencv4"
PACKAGE_BUILD="${WORKSPACE}/build/precision_landing"

if [[ ! -f "${OPENCV_CMAKE}/OpenCVConfig.cmake" ]]; then
  echo "ERROR: OpenCV 4.5.5 config not found: ${OPENCV_CMAKE}/OpenCVConfig.cmake" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/src/precision_landing/package.xml" ]]; then
  echo "ERROR: precision_landing package not found under ${WORKSPACE}/src" >&2
  exit 1
fi

set +u
source /opt/ros/noetic/setup.bash
set -u
cd "${WORKSPACE}"

# Remove only this package's generated CMake cache, never source or other packages.
rm -rf "${PACKAGE_BUILD}"

export OpenCV_DIR="${OPENCV_CMAKE}"
export LD_LIBRARY_PATH="${OPENCV_PREFIX}/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"

catkin_make --pkg precision_landing \
  -DCMAKE_BUILD_TYPE=Release \
  -DOpenCV_DIR="${OPENCV_CMAKE}"

VISION_BIN="${WORKSPACE}/devel/lib/precision_landing/precision_landing_debug_overlay"
CONTROL_BIN="${WORKSPACE}/devel/lib/precision_landing/precision_landing_node"
for binary in "${VISION_BIN}" "${CONTROL_BIN}"; do
  if [[ ! -x "${binary}" ]]; then
    echo "ERROR: expected executable was not built: ${binary}" >&2
    exit 1
  fi
done

LINKS="$(ldd "${VISION_BIN}")"
echo "${LINKS}" | awk '/opencv/ {print}'
if echo "${LINKS}" | awk '/libopencv_.*\.so\.4\.2/ {found=1} END {exit !found}'; then
  echo "ERROR: OpenCV 4.2 was linked. Build rejected." >&2
  exit 1
fi
if ! echo "${LINKS}" | awk \
    '/\/usr\/local\/opencv-455-cuda\/lib\/libopencv_.*\.so\.405/ {found=1} END {exit !found}'; then
  echo "ERROR: OpenCV 4.5.5 libraries were not resolved from ${OPENCV_PREFIX}." >&2
  exit 1
fi

echo "SUCCESS: precision_landing compiled with OpenCV 4.5.5 only."
echo "Run: source ${WORKSPACE}/devel/setup.bash"
echo "Then: roslaunch precision_landing d455_precision_landing.launch enable_flight:=false"
