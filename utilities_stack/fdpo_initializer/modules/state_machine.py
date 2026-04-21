"""Máquina de estados do ``initializer_node``.

Converte eventos vindos do :class:`GpioButton` e do :class:`StackSupervisor`
em acções concretas: contar cliques durante a janela de inactividade,
escolher o modo correspondente e arrancar o ``roslaunch`` do stack; ou,
com um ``long press``, parar e relançar o stack.

Estados
-------

* ``IDLE``     - pronto; sem cliques acumulados, sem stack no ar.
* ``COUNTING`` - pelo menos 1 clique curto recebido; a aguardar
                 ``inactivity_ms`` sem novo clique para confirmar o modo.
* ``RUNNING``  - stack em execução. Long press => reset.
* ``STOPPING`` - a encerrar o stack (transiente).

Eventos
-------

* ``BUTTON_PRESSED``, ``BUTTON_RELEASED`` (com flag interna ``was_held``)
* ``BUTTON_HELD`` (disparado pelo :class:`GpioButton` após hold_time_ms)
* ``INACTIVITY_TIMEOUT`` (timer interno)
* ``STACK_EXITED`` (via callback do :class:`StackSupervisor`)
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import rospy


# Estados --------------------------------------------------------------------

S_IDLE = "IDLE"
S_COUNTING = "COUNTING"
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
        click_to_mode: Dict[int, ModeCommand],
        inactivity_ms: int,
        start_stack: Callable[[ModeCommand], bool],
        stop_stack: Callable[[], bool],
        on_state_change: Optional[Callable[[str, Dict[str, Any]], None]] = None,
    ) -> None:
        self._click_to_mode = click_to_mode
        self._inactivity_s = inactivity_ms / 1000.0
        self._start_stack = start_stack
        self._stop_stack = stop_stack
        self._on_state_change = on_state_change

        self._lock = threading.RLock()
        self._state = S_IDLE
        self._click_count = 0
        self._was_held_current_press = False
        self._inactivity_timer: Optional[threading.Timer] = None

        self._notify_state()

    # ------------------------------------------------------------------ api

    @property
    def state(self) -> str:
        with self._lock:
            return self._state

    @property
    def click_count(self) -> int:
        with self._lock:
            return self._click_count

    # ---------------------------------------------------------- button events

    def on_button_pressed(self) -> None:
        with self._lock:
            self._was_held_current_press = False
            # Qualquer press cancela a janela de inactividade; ela volta a
            # arrancar quando houver release curto em COUNTING.
            self._cancel_inactivity_timer()

    def on_button_released(self) -> None:
        with self._lock:
            was_held = self._was_held_current_press
            self._was_held_current_press = False

            if was_held:
                # Já tratado em on_button_held; o release apenas fecha o ciclo.
                return

            # Clique curto.
            if self._state == S_IDLE:
                self._click_count = 1
                self._set_state(S_COUNTING)
                self._start_inactivity_timer()
            elif self._state == S_COUNTING:
                self._click_count += 1
                self._start_inactivity_timer()
                self._notify_state()  # actualiza contador visível
            elif self._state == S_RUNNING:
                rospy.loginfo(
                    "[fdpo_initializer] Clique curto ignorado (stack em execucao)"
                )
            elif self._state == S_STOPPING:
                rospy.loginfo(
                    "[fdpo_initializer] Clique curto ignorado (stack a parar)"
                )

    def on_button_held(self) -> None:
        """Dispara quando o botão fica premido ``hold_time_ms``."""
        with self._lock:
            self._was_held_current_press = True

            if self._state == S_IDLE:
                rospy.logwarn(
                    "[fdpo_initializer] Long press em IDLE ignorado (stack nao esta no ar)"
                )
                return

            if self._state == S_COUNTING:
                rospy.logwarn(
                    "[fdpo_initializer] Long press durante contagem: a cancelar cliques"
                )
                self._cancel_counting()
                return

            if self._state == S_RUNNING:
                rospy.loginfo(
                    "[fdpo_initializer] Long press detectado: a reiniciar stack"
                )
                self._set_state(S_STOPPING)
                # stop_stack é bloqueante; o STACK_EXITED chega via callback.
                # Libertar lock para não bloquear outros callbacks.
        # Fora do lock:
        if self.state == S_STOPPING:
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
            self._click_count = 0
            self._set_state(S_IDLE)

    # ---------------------------------------------------------- timer events

    def _on_inactivity_timeout(self) -> None:
        mode: Optional[ModeCommand] = None
        with self._lock:
            if self._state != S_COUNTING:
                return

            count = self._click_count
            mode = self._click_to_mode.get(count)
            if mode is None:
                rospy.logwarn(
                    "[fdpo_initializer] %d cliques nao mapeados para nenhum modo; a voltar a IDLE",
                    count,
                )
                self._click_count = 0
                self._set_state(S_IDLE)
                return

            rospy.loginfo(
                "[fdpo_initializer] %d cliques -> modo '%s %s %s'",
                count,
                mode.launch_pkg,
                mode.launch_file,
                mode.args,
            )

        # Fora do lock porque start_stack pode demorar.
        ok = False
        try:
            ok = self._start_stack(mode)
        except Exception:
            rospy.logexception("[fdpo_initializer] start_stack falhou")

        with self._lock:
            if ok:
                self._set_state(S_RUNNING)
            else:
                rospy.logerr("[fdpo_initializer] Arranque falhou; a voltar a IDLE")
                self._click_count = 0
                self._set_state(S_IDLE)

    # ---------------------------------------------------------- helpers

    def _start_inactivity_timer(self) -> None:
        self._cancel_inactivity_timer()
        self._inactivity_timer = threading.Timer(
            self._inactivity_s, self._on_inactivity_timeout
        )
        self._inactivity_timer.daemon = True
        self._inactivity_timer.start()

    def _cancel_inactivity_timer(self) -> None:
        if self._inactivity_timer is not None:
            self._inactivity_timer.cancel()
            self._inactivity_timer = None

    def _cancel_counting(self) -> None:
        self._cancel_inactivity_timer()
        self._click_count = 0
        self._set_state(S_IDLE)

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
        payload = {"state": self._state, "click_count": self._click_count}
        try:
            self._on_state_change(self._state, payload)
        except Exception:
            rospy.logexception("[fdpo_initializer] on_state_change falhou")

    # --------------------------------------------------------------- cleanup

    def shutdown(self) -> None:
        with self._lock:
            self._cancel_inactivity_timer()
