"""Máquina de estados (simplificada) do ``initializer_node``.

Comportamento
-------------

* Em ``IDLE``, qualquer clique curto (release sem long press) arranca
  imediatamente o **modo único** configurado no YAML (``mode``).
* Cliques recebidos enquanto o stack está ``RUNNING`` ou ``STOPPING``
  são ignorados.
* ``Long press`` (``hold_time_ms``) em ``RUNNING`` faz reset: encerra o
  filho (SIGINT → SIGTERM → SIGKILL) e volta a ``IDLE``.
* ``Long press`` em ``IDLE`` é ignorado (stack não está no ar).

Estados
-------

* ``IDLE``     - pronto; sem stack no ar.
* ``RUNNING``  - stack em execução.
* ``STOPPING`` - a encerrar o stack (transiente).

Eventos
-------

* ``BUTTON_PRESSED``, ``BUTTON_RELEASED`` (com flag interna ``was_held``)
* ``BUTTON_HELD`` (disparado pelo :class:`GpioButton` após ``hold_time_ms``)
* ``STACK_EXITED`` (via callback do :class:`StackSupervisor`)
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Optional

import rospy


# Estados --------------------------------------------------------------------

S_IDLE = "IDLE"
S_RUNNING = "RUNNING"
S_STOPPING = "STOPPING"


@dataclass
class ModeCommand:
    """Argumentos a passar a ``StackSupervisor.start``."""

    launch_pkg: str
    launch_file: str
    args: Dict[str, str] = field(default_factory=dict)


class InitializerStateMachine:
    def __init__(
        self,
        mode: ModeCommand,
        start_stack: Callable[[ModeCommand], bool],
        stop_stack: Callable[[], bool],
        on_state_change: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> None:
        self._mode = mode
        self._start_stack = start_stack
        self._stop_stack = stop_stack
        self._on_state_change = on_state_change

        self._lock = threading.RLock()
        self._state = S_IDLE
        self._was_held_current_press = False

        self._notify_state()

    # ------------------------------------------------------------------ api

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    # ---------------------------------------------------------- button events

    def on_button_pressed(self) -> None:
        with self._lock:
            self._was_held_current_press = False

    def on_button_released(self) -> None:
        with self._lock:
            was_held = self._was_held_current_press
            self._was_held_current_press = False

            if was_held:
                # Long press: já tratado em on_button_held; o release fecha o ciclo.
                return

            if self._state != S_IDLE:
                rospy.loginfo(
                    "[fdpo_initializer] Clique curto ignorado (estado=%s)",
                    self._state,
                )
                return

            mode = self._mode

        # Fora do lock: arrancar o stack pode demorar.
        rospy.loginfo(
            "[fdpo_initializer] Clique detectado -> a arrancar modo unico (%s/%s)",
            mode.launch_pkg,
            mode.launch_file,
        )
        ok = False
        try:
            ok = self._start_stack(mode)
        except Exception:
            rospy.logexception("[fdpo_initializer] start_stack falhou")

        with self._lock:
            if ok:
                self._set_state(S_RUNNING)
            else:
                rospy.logerr("[fdpo_initializer] Arranque falhou; a ficar em IDLE")
                self._set_state(S_IDLE)

    def on_button_held(self) -> None:
        """Dispara quando o botão fica premido ``hold_time_ms``."""
        with self._lock:
            self._was_held_current_press = True

            if self._state != S_RUNNING:
                rospy.logwarn(
                    "[fdpo_initializer] Long press em %s ignorado", self._state
                )
                return

            rospy.loginfo("[fdpo_initializer] Long press detectado: a parar stack")
            self._set_state(S_STOPPING)
            # stop_stack é bloqueante; o STACK_EXITED chega via callback.

        try:
            self._stop_stack()
        except Exception:
            rospy.logexception("[fdpo_initializer] stop_stack falhou")

    # ----------------------------------------------------- supervisor events

    def on_stack_exited(self, return_code: int) -> None:
        with self._lock:
            if self._state == S_STOPPING:
                rospy.loginfo(
                    "[fdpo_initializer] Stack parado (rc=%s). Pronto para novo arranque.",
                    return_code,
                )
            elif self._state == S_RUNNING:
                rospy.logwarn(
                    "[fdpo_initializer] Stack saiu inesperadamente (rc=%s). A voltar a IDLE.",
                    return_code,
                )
            else:
                rospy.logwarn(
                    "[fdpo_initializer] STACK_EXITED em estado %s (rc=%s)",
                    self._state,
                    return_code,
                )
            self._set_state(S_IDLE)

    # ---------------------------------------------------------- helpers

    def _set_state(self, new_state: str) -> None:
        if new_state != self._state:
            rospy.loginfo(
                "[fdpo_initializer] Estado: %s -> %s", self._state, new_state
            )
            self._state = new_state
        self._notify_state()

    def _notify_state(self) -> None:
        if self._on_state_change is None:
            return
        payload = {"state": self._state}
        try:
            self._on_state_change(self._state, payload)
        except Exception:
            rospy.logexception("[fdpo_initializer] on_state_change falhou")

    # --------------------------------------------------------------- cleanup

    def shutdown(self) -> None:
        return
