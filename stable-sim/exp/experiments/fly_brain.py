# Fly brain mode: a placeholder. Nothing runs here yet, and the simulator doesn't use this file.
#
# The idea: put a real fruit fly brain (the mapped wiring, not a hand-made circuit) where the plan's
# AI box is, and train it in the simulator before it ever flies the real drone.
#
#     video + flight data -> fly brain -> stick command -> Safety Layer -> Radio Bridge -> drone
#
# The brain's commands go through the Safety Layer like the keyboard's do, so the tilt limit, the
# video-loss landing and DISARM all still apply.
#
# Where the brain comes from (links written from memory: check them before relying on them)
#   - FlyWire: the wiring of a whole adult fruit fly brain, about 140,000 neurons and 50 million
#     synapses, with a predicted transmitter for each neuron. Download from codex.flywire.ai
#   - A ready-made spiking model built on that wiring (Shiu and others, 2024):
#     github.com/philshiu/Drosophila_brain_model
#   - The visual motion part only, already set up for training (Lappalainen and others, 2024):
#     github.com/TuragaLab/flyvis. Much smaller, and the likely first thing to try
#   - The brain stops at the neck. Wings are driven from the nerve cord, which is a separate map
#     (MANC, at neuprint.janelia.org), so the brain's output still has to be turned into commands
#
# What has to be decided or built first
#   - eyes: the simulator's video is a camera position, not a picture. The brain needs pixels, so
#     the camera view has to be rendered and sampled onto the fly's eye
#   - output: which neurons are read, and how their firing becomes roll, pitch, yaw and throttle
#   - senses: the drone only reports lean angles and battery. A fly also has halteres (a gyro,
#     which we have) and no height sensor (same as us)
#   - what is trained: the wiring is fixed by the map. The trainable parts are the synapse
#     strengths and the way in and out
#
# What the simulator already gives training
#   - it runs without a window: Sim and CommandCenter, the way tests/test_sim.py flies it
#   - every build and weather setting can be changed per flight (hardware/parts.py), and the random seed
#     sets the build errors, so each training flight can be a slightly different drone
#   - the flight tests (flight_tests.py) judge with the true position, which is what a training
#     score needs
#
# What the simulator still needs for training
#   - a rendered camera picture (see eyes)
#   - a reset / step / score wrapper around Sim and CommandCenter
#   - speed: the physics is plain Python at 4 kHz and runs about 10 times faster than real time on
#     one laptop core, before any picture is rendered. Training needs many thousands of flights, so
#     that is probably too slow as it stands
