# AprilTag Landing Board Joint-PnP Design

## Objective

The landing target is always the physical center of ID 0. Single-tag and
multi-tag observations use the same board estimator instead of independently
estimating tag poses and averaging extrapolated centers.

## Geometry

The board frame has its origin at ID 0, +X right on the page, and +Y down. The
OpenCV detector returns the generated artwork's canonical corners in page order
bottom-right, bottom-left, top-left, top-right. Each image corner is therefore
paired directly with a known point in the common board frame.

Detected black-square sides are 68, 28.8, and 14.4 mm for the outer, middle,
and center tags. A `board_scale` parameter multiplies both tag sizes and center
offsets for uniformly resized prints.

## Estimation and validation

All valid visible corners are passed to one OpenCV 4.5.5
`solvePnP(..., SOLVEPNP_ITERATIVE)`. Its translation is the camera-frame ID 0
position. Non-finite poses, non-positive depth, and estimates exceeding the
configured RMS reprojection threshold are rejected and are not published to
the controller.

The debug image displays detected IDs, final ID 0 target, tag count, RMS,
camera/FRD errors, guide and actual velocity, and the direction arrow.

## Safety

Camera output is converted only once using
`body_frd = (-camera_y, camera_x, camera_z)` and published as MAVROS
`FRAME_BODY_NED`. The default `enable_flight=false` gate remains authoritative.
The package does not arm, change mode, or call LAND.

## Verification

Geometry tests anchor physical black-square sizes and canonical corner order.
Synthetic single-tag and tilted multi-tag tests verify recovery of the same ID
0 origin. Jetson verification requires all tests passing, OpenCV `.so.405`
linkage without 4.2, a live debug stream, and zero BODY_NED velocity while the
flight gate is disabled.
