"""Blind Spot Assist bars: each side's blindspotmodeld level (white -> amber -> red), a red glow on a detection and an amber
glow for the car's own blind spot radar."""
import raylib_nb as rl

from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app

# comma 3X sizes in pixels; the comma four scales them by SMALL_SCALE
BAR_W, BAR_MARGIN_X = 30, 22
SMALL_SCALE, SMALL_MARGIN_X = 1 / 3, 6
BAR_HEIGHT_FRAC, BAR_CENTER_FRAC = 0.46, 0.515  # of the onroad view height
THRESHOLD, RED_AT = 0.4, 0.8                     # level: detection threshold (blindspotmodeld), full red

TRACK = rl.Color(0, 0, 0, 110)
WHITE, AMBER, RED = (235, 235, 235), (255, 176, 32), (240, 56, 56)


def _lerp(a, b, t):
  return tuple(int(x + (y - x) * t) for x, y in zip(a, b, strict=True))


def _fill_color(level: float, alpha: int) -> rl.Color:
  c = _lerp(WHITE, AMBER, level / THRESHOLD) if level < THRESHOLD else _lerp(AMBER, RED, min(1.0, (level - THRESHOLD) / (RED_AT - THRESHOLD)))
  return rl.Color(*c, alpha)


class BlindSpotBars:
  def __init__(self):
    f = lambda tc=0.1: FirstOrderFilter(0.0, tc, 1 / gui_app.target_fps)  # noqa: E731
    self._alpha = f(0.3)
    # per side: level, model detection, car radar
    self._sides = {side: (f(), f(), f()) for side in ('left', 'right')}

  def update(self) -> None:
    sm = ui_state.sm
    live = ui_state.blind_spot_assist and sm.alive['blindSpotAssist'] and sm.valid['blindSpotAssist']
    self._alpha.update(float(live))
    bsa, cs = sm['blindSpotAssist'], sm['carState']
    for side, (level, detected, radar) in self._sides.items():
      if live:
        level.update(getattr(bsa, f'{side}Level'))
      detected.update(float(live and getattr(bsa, f'{side}Detected')))
      radar.update(float(getattr(cs, f'{side}Blindspot')))

  def render(self, rect: rl.Rectangle) -> None:
    if not ui_state.blind_spot_assist or self._alpha.x < 0.02:
      return
    s, margin = (1.0, BAR_MARGIN_X) if gui_app.big_ui() else (SMALL_SCALE, SMALL_MARGIN_X)
    w, h = BAR_W * s, rect.height * BAR_HEIGHT_FRAC
    y = rect.y + rect.height * BAR_CENTER_FRAC - h / 2
    for side, x in (('left', rect.x + margin), ('right', rect.x + rect.width - margin - w)):
      self._draw_bar(rl.Rectangle(x, y, w, h), s, *(flt.x for flt in self._sides[side]))

  def _draw_bar(self, r: rl.Rectangle, s: float, level: float, detected: float, radar: float) -> None:
    a = self._alpha.x
    # glows: amber for the car's radar (outside), red for the model's detection (inside)
    for on, color, sizes in ((radar, AMBER, ((20, 45), (13, 75))), (detected, RED, ((14, 40), (8, 70)))):
      if on > 0.01:
        for grow, ga in sizes:
          g = grow * s
          rl.draw_rectangle_rounded(rl.Rectangle(r.x - g, r.y - g, r.width + 2 * g, r.height + 2 * g), 1.0, 12, rl.Color(*color, int(ga * a * on)))
    rl.draw_rectangle_rounded(r, 1.0, 12, rl.Color(TRACK.r, TRACK.g, TRACK.b, int(TRACK.a * a)))
    fill_h = r.height * min(max(level, 0.0), 1.0)
    if fill_h > 1:
      rl.draw_rectangle_rounded(rl.Rectangle(r.x, r.y + r.height - fill_h, r.width, fill_h), 1.0, 12, _fill_color(level, int(235 * a)))
