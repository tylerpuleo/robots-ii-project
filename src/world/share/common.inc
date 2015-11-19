window (
  scale 10
  size [800 800]
)

define floorplan model (
  boundary 0
  color "gray30"
  fiducial_return 0
  gripper_return 0
  gui_grid 0
  gui_move 0
  gui_nose 0
  gui_outline 0
  laser_return 1
)

define sicklms ranger (
  sensor (
    fov 180.0
    samples 180
  )
  model (
    size [ 0.25 0.25 0.1 ]
    color "black"
  )
)

define robot position (
  acceleration_bounds [-10 10 0 0 0 0 -90 90]
  color "red"
  drive "diff"
  gui_nose 1
  gui_outline 1
  size [1 1 0.5]
  velocity_bounds [-5 5 0 0 0 0 -90 90]

  sicklms (
    pose [0.25 0 0 0]
  )
)

define pedestrian position (
  acceleration_bounds [-10 10 -10 10 -10 10 -90 90]
  color "blue"
  drive "omni"
  gui_nose 1
  gui_outline 1
  size [0.5 0.5 1]
  velocity_bounds [-1 1 -1 1 -1 1 -90 90] 
)

resolution 0.02 
interval_sim 100
