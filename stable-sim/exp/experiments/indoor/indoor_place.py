# Indoor place: a placeholder. Nothing runs here yet, and the simulator doesn't use this file.
#
# The simulator's world is an open field with a few boxes (drone_sim/world.py). The first real
# flights will most likely be indoors, so this folder is for a room to fly in.
#
# What a room needs that the field doesn't have
#   - walls and a ceiling. The drone can only hit the floor and the boxes today (collide in
#     drone_sim/hardware/drone.py), and it can climb for ever
#   - furniture and doorways to fly around and through
#   - the room drawn in the 3D view (drone_sim/view.py only draws the floor grid, boxes and rings)
#
# What changes indoors on the real drone, and is not modelled yet
#   - no wind, but the drone's own downwash bounces off the floor, walls and ceiling and pushes
#     it around, most of all in corners
#   - ground effect: extra lift close to the floor, so the hover throttle changes in the last
#     few centimetres. Close to the ceiling it gets sucked upward
#   - the radio and the video usually work better at room range, but bodies and walls block them
#   - there is far less room for drift: at 0.1 m/s it reaches a wall in well under a minute
#
# Ideas for when this is built
#   - a room described the same way as the boxes in world.py: a size, and a list of things in it
#   - a setting to pick the place: the field (as now) or a room
#   - flight tests for it: hover in the middle of the room, fly through a doorway, land on a table
#   - modeled after lattea
