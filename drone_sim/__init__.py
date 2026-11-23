# The name of this stable simulator. How names work (they don't, much) is in NAMES.md.
GADGET = 1                                    # goes up by one for every new stable simulator
NAME = "Euphrosyne Mirth Particle Windmill"   # must not describe what it does
VER = "1.0"                                   # goes up for fixes on top of this gadget
VERSION = f"Sky Gadget No. {GADGET}: {NAME}" + ("" if VER == "1.0" else f" ver {VER}") + " (name not final)"
