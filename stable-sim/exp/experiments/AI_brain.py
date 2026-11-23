# AI brain: a placeholder. Nothing runs here yet, and the simulator doesn't use this file.
#
# This is for the AI box in software-plan.png:
#   - a small vision network built from scratch
#   - gestures: take off, land, move
#   - person tracking: follow me
#
# Where it sits. The AI never talks to the drone directly:
#
#     video + flight data -> AI -> stick command -> Safety Layer -> Radio Bridge -> drone
#
# So the tilt limit, the video-loss landing and DISARM still apply, and the plan's rules hold:
# the pilot always overrides, and if the AI is lost the Safety Layer hovers or lands.
#
# What the AI gets, and nothing more (same as the real laptop):
#   - the video frames from the XIAO camera, late by video_delay_ms
#   - flight data: lean angles, battery voltage, armed
#   - no height, speed or position
#
# What it must return each frame: {"roll", "pitch", "yaw", "thr"}, the same as pilot.py does for
# the keyboard. command_center.py is where a second source of commands would be plugged in.
#
# What has to be built first
#   - pictures: the simulator's video is a camera position, not an image. The camera view has to
#     be rendered to pixels before a vision network can see anything
#   - a person in the world to track and to make gestures (world.py only has boxes)
#   - Safety Layer: an "AI lost" case next to video lost and link lost, and a key that hands
#     control to the AI and takes it back
#   - a way to score a flight without a window (flight_tests.py already judges with the true
#     position, which is what training needs)
#
# Related: fly_brain.py, the other placeholder in this folder.
