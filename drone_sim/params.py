# All the settings in one place: the build (hardware/parts.py) plus the laptop side and the weather.
from .hardware.parts import CHIPS, CHOICES, PARTS, REGULATORS
from .hardware.parts import LIMITS as PART_LIMITS

LAPTOP = {
    # keyboard
    "kb_strength": 0.6,      # how far a held key pushes the stick, 0 to 1
    "hover_throttle": 0.6,   # the laptop's guess at the throttle that hovers (at 4.0 V). Measure it!
    # safety layer
    "safety_max_tilt": 30.0,   # degrees tilt limit. With no speed sensor, this is also the speed limit
    "safety_hover_s": 5.0,     # video lost: hover this long waiting for the pilot, then land
}

# steady wind and typical gust strength in m/s, and the direction the wind blows toward
WEATHER = {"wind": 0.0, "gust": 0.0, "wind_dir": 90}

LIMITS = {**PART_LIMITS,
          "kb_strength": (0.1, 1), "hover_throttle": (0.1, 0.95),
          "safety_max_tilt": (1, 80), "safety_hover_s": (0, 60),
          "wind": (0, 20), "gust": (0, 10), "wind_dir": (-360, 360)}


def default_params():
    return {**PARTS, **LAPTOP, **WEATHER}
