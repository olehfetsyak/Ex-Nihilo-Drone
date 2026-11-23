# How the simulator is named

Every stable simulator is a numbered Sky Gadget with a grand name that explains nothing.

This one is **Sky Gadget No. 1: Euphrosyne Mirth Particle Windmill (name not final)**.

It is a keyboard-flown drone simulator. It has no particles and no windmill.

## Where Euphrosyne comes from

We decided on a space theme, looked through the exceptional asteroids, and chose 31 Euphrosyne, one of the largest in the asteroid belt. It seemed fitting. That is the only part of the name with a reason.

## The rules

1. **Every stable simulator gets the next number.** A simulator earns one only when every test passes: the headless suite, the settings sweep, the stress runs and the live window run.
2. **The name is one exceptional asteroid, then three important-sounding words.** The asteroid keeps the space theme. The three words must not describe a flight simulator. If someone could guess what it does from the name, pick again.
3. **"(name not final)" is always on the end.** The name never changes anyway.
4. **A fix adds a ver.** Bug fixes on top of a gadget keep its number and name: Sky Gadget No. 1: Euphrosyne Mirth Particle Windmill ver 1.1 (name not final).
5. **Work in progress has no number.** Until it is stable it is just "the next gadget".
6. **The drone keeps its own number.** "Drone v.0.0" is the hardware. The gadget is the simulator.

## Short forms

- In conversation: "Gadget 1", or "the Windmill".
- Git tag: `sky-gadget-1`, or `sky-gadget-1-ver-1.1` for a fix.

## Where the name shows

- the window title and the H panel
- `python run_sim.py --version`

To change it, edit `GADGET`, `NAME` and `VER` in `drone_sim/__init__.py`.
