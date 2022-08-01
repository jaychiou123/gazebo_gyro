define laser ranger
(
  sensor
  (
    size [ 0.06 0.15 0.03 ]
    range [ 0.0 10.0]
    fov 360.0
    samples 1081
  )

  # generic model properties
  color "black"
)

define omnidir_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  localization "gps"

  size [ 1.20 0.60 1.80 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "red"

  # kinematics
  drive "omni"

  # spawn sensors
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)
