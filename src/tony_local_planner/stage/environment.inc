define floorplan model
(
  # sombre, sensible, artistic
  color "gray30"

  # most maps will need a bounding box
  boundary 1

  gui_nose 0
  gui_grid 0
  gui_outline 0
  gripper_return 0
  fiducial_return 0
  ranger_return 1
)

resolution 0.05
interval_sim 50  # simulation timestep in milliseconds, THIS MUST MATCH THE SAMPLING RATE OF LOCAL_PLANNER!  
speedup 14

window
(
  size [ 1280.0 640.0 ]
  center [ 0.0 0.0 ]
  rotate [ 0.0 0.0 ]
  scale 5 #bigger scale run faster

  show_data 1
  show_clock 1
)