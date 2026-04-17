"""Abstracção de um botão mecânico ligado a um GPIO da Raspberry Pi.

Expõe três eventos ao consumidor (tipicamente a máquina de estados):

    on_pressed()   - botão foi premido (após debounce)
    on_released()  - botão foi largado (após debounce)
    on_held()      - botão esteve premido >= hold_time_ms sem ser largado

O "clique curto" não é emitido aqui: o consumidor sabe que
    pressed -> released sem on_held pelo meio == clique curto
    pressed -> on_held (fica premido) == long press

Suporta dois backends:

* "gpiozero" - real, para a Pi.
* "sim"      - para bancada / testes: ouve um tópico ROS
               ``std_msgs/Bool`` (True=premido, False=largado).

O backend é escolhido por parâmetro (``backend``); se "gpiozero" falhar a
importar (ex. dev em PC), faz-se fallback automático para "sim" com aviso.
"""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import rospy


class GpioButton:
    """Botão com eventos pressed / released / held."""

    def __init__(
        self,
        backend: str = "gpiozero",
        pin: int = 17,
        active_low: bool = True,
        debounce_ms: int = 30,
        hold_time_ms: int = 5000,
        sim_topic: str = "/initializer/sim_button",
        on_pressed: Optional[Callable[[], None]] = None,
        on_released: Optional[Callable[[], None]] = None,
        on_held: Optional[Callable[[], None]] = None,
    ) -> None:
        self._on_pressed = on_pressed
        self._on_released = on_released
        self._on_held = on_held

        self._hold_time_s = hold_time_ms / 1000.0
        self._debounce_s = debounce_ms / 1000.0

        self._hold_timer: Optional[threading.Timer] = None
        self._lock = threading.Lock()
        self._is_pressed = False
        # Instante (monotonico) da ultima transicao ACEITE. Usado para
        # descartar eventos que cheguem antes de debounce_s (bounce mecanico
        # ou ruido no GPIO). Aplica-se a press E release, em ambos os backends.
        self._last_edge_time: float = 0.0
        self._debounced_count: int = 0

        self._backend_name = backend
        self._button = None
        self._sim_sub = None

        if backend == "gpiozero":
            try:
                self._setup_gpiozero(pin=pin, active_low=active_low)
            except Exception as exc:  # pragma: no cover - hardware-specific
                rospy.logwarn(
                    "[fdpo_initializer] gpiozero indisponivel (%s). A usar backend 'sim'.",
                    exc,
                )
                self._backend_name = "sim"
                self._setup_sim(sim_topic)
        elif backend == "sim":
            self._setup_sim(sim_topic)
        else:
            raise ValueError(f"GpioButton: backend desconhecido '{backend}'")

    # ------------------------------------------------------------------ setup

    def _setup_gpiozero(self, pin: int, active_low: bool) -> None:
        """Configura o backend real via gpiozero."""
        from gpiozero import Button  # import local para permitir o fallback

        # gpiozero: pull_up=True => botão entre pino e GND (activo LOW).
        pull_up = active_low
        self._button = Button(
            pin=pin,
            pull_up=pull_up,
            bounce_time=self._debounce_s,
            hold_time=self._hold_time_s,
            hold_repeat=False,
        )
        self._button.when_pressed = self._handle_pressed
        self._button.when_released = self._handle_released
        # Nota: usamos o nosso próprio timer (abaixo) para "hold" em vez do
        # gpiozero.when_held porque queremos o mesmo comportamento no backend
        # "sim" e controlo explícito do cancel-on-release.

    def _setup_sim(self, topic: str) -> None:
        """Configura o backend de simulação (tópico ROS)."""
        from std_msgs.msg import Bool

        rospy.loginfo(
            "[fdpo_initializer] GpioButton em modo 'sim': publica %s (std_msgs/Bool) para simular o botao.",
            topic,
        )

        def _cb(msg: Bool) -> None:
            if msg.data:
                self._handle_pressed()
            else:
                self._handle_released()

        self._sim_sub = rospy.Subscriber(topic, Bool, _cb, queue_size=10)

    # ------------------------------------------------------------- callbacks

    def _handle_pressed(self) -> None:
        with self._lock:
            if self._is_pressed:
                return
            if self._is_bouncing_locked():
                self._debounced_count += 1
                rospy.logdebug(
                    "[fdpo_initializer] Press ignorado por debounce (%.1f ms < %.1f ms)",
                    self._since_last_edge_locked() * 1000.0,
                    self._debounce_s * 1000.0,
                )
                return
            self._is_pressed = True
            self._mark_edge_locked()
            self._start_hold_timer()
        if self._on_pressed is not None:
            try:
                self._on_pressed()
            except Exception:
                rospy.logexception("[fdpo_initializer] on_pressed falhou")

    def _handle_released(self) -> None:
        with self._lock:
            if not self._is_pressed:
                return
            if self._is_bouncing_locked():
                self._debounced_count += 1
                rospy.logdebug(
                    "[fdpo_initializer] Release ignorado por debounce (%.1f ms < %.1f ms)",
                    self._since_last_edge_locked() * 1000.0,
                    self._debounce_s * 1000.0,
                )
                return
            self._is_pressed = False
            self._mark_edge_locked()
            self._cancel_hold_timer()
        if self._on_released is not None:
            try:
                self._on_released()
            except Exception:
                rospy.logexception("[fdpo_initializer] on_released falhou")

    def _handle_held(self) -> None:
        with self._lock:
            if not self._is_pressed:
                return
        if self._on_held is not None:
            try:
                self._on_held()
            except Exception:
                rospy.logexception("[fdpo_initializer] on_held falhou")

    # ---------------------------------------------- helpers (assume lock held)

    def _since_last_edge_locked(self) -> float:
        return time.monotonic() - self._last_edge_time

    def _is_bouncing_locked(self) -> bool:
        if self._debounce_s <= 0.0 or self._last_edge_time == 0.0:
            return False
        return self._since_last_edge_locked() < self._debounce_s

    def _mark_edge_locked(self) -> None:
        self._last_edge_time = time.monotonic()

    def _start_hold_timer(self) -> None:
        self._cancel_hold_timer()
        self._hold_timer = threading.Timer(self._hold_time_s, self._handle_held)
        self._hold_timer.daemon = True
        self._hold_timer.start()

    def _cancel_hold_timer(self) -> None:
        if self._hold_timer is not None:
            self._hold_timer.cancel()
            self._hold_timer = None

    # ------------------------------------------------------------------ misc

    @property
    def backend(self) -> str:
        return self._backend_name

    @property
    def debounced_count(self) -> int:
        """Numero acumulado de transicoes descartadas por debounce."""
        with self._lock:
            return self._debounced_count

    def close(self) -> None:
        self._cancel_hold_timer()
        if self._button is not None:
            try:
                self._button.close()
            except Exception:
                pass
        if self._sim_sub is not None:
            try:
                self._sim_sub.unregister()
            except Exception:
                pass
