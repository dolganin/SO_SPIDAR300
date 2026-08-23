from __future__ import annotations

import numpy as np

try:
    import carb
    import omni
    from isaaclab.devices import DeviceBase
except Exception:
    DeviceBase = object

from .. import kinematics as K
from .gait import TripodGait

_LIN_SPEED = 1.0
_ANG_SPEED = 1.0
_STEP = 0.008

_PAIR_RU = {"front": "передняя (у камеры)", "middle": "средняя", "rear": "задняя"}

HELP = """
=================== SO-SPIDAR300 TELEOP ===================
  Ориентация: ПЕРЕД = сторона камеры. W идёт вперёд, к уступу.
  Каждое движение — симметричная пара клавиш:
  ВЕРХНЯЯ = вперёд / вверх / шире, НИЖНЯЯ = наоборот.

  Походка (удерживать):
    W / S ............. вперёд / назад
    A / D ............. влево / вправо (стрейф)
    Q / E ............. поворот влево / вправо

  Корпус (опорные ноги перемещают корпус; можно удерживать):
    стрелка UP/DOWN ... корпус вперёд / назад
    PgUp / PgDn ....... корпус выше / ниже

  Пара ног — выбор: 1 передняя (у камеры) | 2 средняя | 3 задняя
    U / J ............. стопы вперёд / назад
    I / K ............. стопы выше / ниже (поднять / прижать)
    O / L ............. стопы шире / уже

    R ................. сброс в нейтральную стойку

  (G, H, F, T не используются — это хоткеи Isaac Sim:
   H прячет выбранный прим, F наводит на него камеру.)

  Как залезть на уступ:
    1. W — подойти почти вплотную, PgDn — слегка присесть.
    2. «1» -> I (поднять переднюю пару), U (вытянуть стопы
       вперёд над уступом), K (опустить их на уступ).
    3. Стрелка UP — корпус вперёд (опорные ноги тянут его
       на уступ) вперемешку с PgUp — корпус выше, над кромкой.
    4. «2» -> I, U, K — переставить среднюю пару на уступ,
       снова UP / PgUp.
    5. Корпус на уступе: R, затем W — дойти до маркера.
===========================================================
"""

class KeyboardLeggedController(DeviceBase):
    def __init__(self, gait: TripodGait | None = None, control_dt: float = 1.0 / 30.0):
        self.dt = control_dt
        self.gait = gait or TripodGait()
        self._pairs = list(K.PAIRS)
        self._selected = 0
        self._body = np.zeros(2)
        self._cart = {p: np.zeros(3) for p in self._pairs}
        self._pressed: set[str] = set()
        self._additional_callbacks: dict = {}

        if DeviceBase is not object:
            self._appwindow = omni.appwindow.get_default_app_window()
            self._input = carb.input.acquire_input_interface()
            self._keyboard = self._appwindow.get_keyboard()
            self._sub = self._input.subscribe_to_keyboard_events(
                self._keyboard, self._on_keyboard_event
            )

    def reset(self) -> None:
        self._pressed.clear()
        self.gait.phase = 0.0
        self._body[:] = 0.0
        for p in self._pairs:
            self._cart[p][:] = 0.0

    def add_callback(self, key: str, func) -> None:
        self._additional_callbacks[key] = func

    def _foot_delta(self, leg: K.LegDef) -> np.ndarray:
        cart = self._cart[leg.pair]
        sgn = 1.0 if leg.side == "left" else -1.0
        return np.array([
            cart[0] - self._body[0],
            sgn * cart[1],
            cart[2] - self._body[1],
        ])

    @staticmethod
    def _solve(leg: K.LegDef, foot0: np.ndarray, delta: np.ndarray, fallback) -> tuple:
        try:
            return K.ik_leg_body(leg, foot0 + delta)
        except ValueError:
            pass
        lo, hi, best = 0.0, 1.0, None
        for _ in range(8):
            mid = 0.5 * (lo + hi)
            try:
                best = K.ik_leg_body(leg, foot0 + mid * delta)
                lo = mid
            except ValueError:
                hi = mid
        return best if best is not None else tuple(fallback)

    def _state_feasible(self) -> bool:
        for leg in K.LEGS:
            delta = self._foot_delta(leg)
            if not np.any(delta):
                continue
            try:
                angles = K.ik_leg_body(leg, K.nominal_foot_body(leg) + delta)
            except ValueError:
                return False
            if not all(K.within_limits(j, v) for j, v in zip(K.JOINT_ORDER, angles)):
                return False
        return True

    def _nudge(self, arr: np.ndarray, idx: int, sign: float) -> None:
        old = arr[idx]
        arr[idx] = old + sign * _STEP
        if not self._state_feasible():
            arr[idx] = old
            print("[teleop] предел досягаемости/лимита сустава — движение остановлено")

    def advance(self) -> np.ndarray:
        vx, vy, wz = self._command_from_keys()
        targets = self.gait.step(self.dt, vx=vx, vy=vy, wz=wz)
        for li, leg in enumerate(K.LEGS):
            delta = self._foot_delta(leg)
            if not np.any(delta):
                continue
            i = li * 3
            foot0 = K.fk_leg_body(leg, targets[i], targets[i + 1], targets[i + 2])
            targets[i:i + 3] = self._solve(leg, foot0, delta, targets[i:i + 3])
        for k in range(6):
            for j, jn in enumerate(K.JOINT_ORDER):
                targets[3 * k + j] = K.clamp(jn, targets[3 * k + j])
        return targets

    def _command_from_keys(self) -> tuple[float, float, float]:
        vx = (("W" in self._pressed) - ("S" in self._pressed)) * _LIN_SPEED
        vy = (("A" in self._pressed) - ("D" in self._pressed)) * _LIN_SPEED
        wz = (("Q" in self._pressed) - ("E" in self._pressed)) * _ANG_SPEED
        return vx, vy, wz

    def _on_keyboard_event(self, event, *args, **kwargs):
        key = event.input if isinstance(event.input, str) else event.input.name
        if event.type == carb.input.KeyboardEventType.KEY_PRESS:
            self._pressed.add(key)
            self._on_press(key)
            if key in self._additional_callbacks:
                self._additional_callbacks[key]()
        elif event.type == carb.input.KeyboardEventType.KEY_REPEAT:
            self._on_press(key, repeat=True)
        elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
            self._pressed.discard(key)
        return True

    def _on_press(self, key: str, repeat: bool = False) -> None:
        if key in ("KEY_1", "KEY_2", "KEY_3", "NUMPAD_1", "NUMPAD_2", "NUMPAD_3", "1", "2", "3"):
            self._selected = int(key[-1]) - 1
            pair = self._pairs[self._selected]
            print(f"[teleop] выбрана пара: {_PAIR_RU[pair]}")
            return
        if key == "R":
            if not repeat:
                self.reset()
                print("[teleop] сброс в нейтральную стойку")
            return
        body = {"UP": (0, +1), "DOWN": (0, -1), "PAGE_UP": (1, +1), "PAGE_DOWN": (1, -1)}
        if key in body:
            idx, sgn = body[key]
            self._nudge(self._body, idx, sgn)
            return
        cart = {
            "U": (0, +1), "J": (0, -1),
            "O": (1, +1), "L": (1, -1),
            "I": (2, +1), "K": (2, -1),
        }
        if key in cart:
            idx, sgn = cart[key]
            self._nudge(self._cart[self._pairs[self._selected]], idx, sgn)
