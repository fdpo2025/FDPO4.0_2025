#!/usr/bin/env python3
"""Nó ``initializer_node``: ponto de entrada do sistema FDPO na Pi4.

Responsabilidades
-----------------
1. Ler o **modo único** (YAML) a executar quando o botão for premido.
2. Escutar um switch ligado a um GPIO através do :class:`GpioButton`.
3. Correr a :class:`InitializerStateMachine` para, no primeiro clique
   curto detectado, pedir ao :class:`StackSupervisor` para arrancar o
   ``roslaunch`` do stack FDPO.
4. Em long press (``hold_time_ms``) com o stack no ar, reiniciar o stack:
   encerrar o filho (SIGINT → SIGTERM → SIGKILL à process group) e voltar
   a IDLE para novo arranque.
5. Publicar o estado actual em ``/initializer/status`` para depuração.

Identidade do robô
------------------
O ``robot_id`` desta Pi vem da variável de ambiente ``FDPO_ROBOT_ID``.
É substituída nos argumentos de ``roslaunch`` sempre que aparecer o token
``$(env FDPO_ROBOT_ID)`` no YAML, para ficar explícito no log.
"""

from __future__ import annotations

import json
import os
import re
import sys
from typing import Any, Dict

import rospy
from std_msgs.msg import String

# Permitir ``from modules import ...`` quando corrido directamente via
# ``rosrun`` (o script vive em scripts/ e os módulos em ../modules/).
_CURRENT_DIR = os.path.dirname(os.path.abspath(__file__))
_PARENT_DIR = os.path.dirname(_CURRENT_DIR)
if _PARENT_DIR not in sys.path:
    sys.path.insert(0, _PARENT_DIR)

try:
    from modules.gpio_button import GpioButton
    from modules.stack_supervisor import StackSupervisor
    from modules.state_machine import (
        InitializerStateMachine,
        ModeCommand,
    )
except ImportError as exc:  # pragma: no cover
    print(f"[fdpo_initializer] Falha a importar modulos: {exc}", file=sys.stderr)
    sys.exit(1)


_ENV_TOKEN_RE = re.compile(r"\$\(env\s+([A-Z_][A-Z0-9_]*)\)")


def _expand_env(value: str) -> str:
    """Substitui ``$(env VAR)`` em strings pelos valores do environment."""

    def _sub(match: "re.Match[str]") -> str:
        var = match.group(1)
        val = os.environ.get(var)
        if val is None:
            raise RuntimeError(
                f"Variavel de ambiente '{var}' referida em parametros mas nao definida"
            )
        return val

    return _ENV_TOKEN_RE.sub(_sub, value)


def _parse_mode(raw: Any) -> ModeCommand:
    """Converte o YAML ``mode`` num :class:`ModeCommand`."""
    if not isinstance(raw, dict):
        raise ValueError("mode deve ser um dicionario")

    pkg = raw.get("launch_pkg")
    launch = raw.get("launch_file")
    if not pkg or not launch:
        raise ValueError("mode precisa de 'launch_pkg' e 'launch_file'")

    raw_args = raw.get("args", {}) or {}
    if not isinstance(raw_args, dict):
        raise ValueError("mode.args deve ser um dicionario")

    expanded_args: Dict[str, str] = {}
    for k, v in raw_args.items():
        expanded_args[str(k)] = _expand_env(str(v))

    return ModeCommand(
        launch_pkg=str(pkg), launch_file=str(launch), args=expanded_args
    )


class InitializerNode:
    def __init__(self) -> None:
        rospy.init_node("initializer_node")

        # ---- Parâmetros gerais --------------------------------------------
        self.backend = rospy.get_param("~backend", "gpiozero")
        self.gpio_pin = int(rospy.get_param("~gpio_pin", 17))
        self.active_low = bool(rospy.get_param("~active_low", True))
        self.debounce_ms = int(rospy.get_param("~debounce_ms", 30))
        self.hold_time_ms = int(rospy.get_param("~long_press_ms", 5000))
        self.sim_topic = rospy.get_param("~sim_topic", "/initializer/sim_button")
        self.status_topic = rospy.get_param("~status_topic", "/initializer/status")
        self.source_setup = rospy.get_param("~source_setup", "")
        self.sigint_timeout_s = float(rospy.get_param("~sigint_timeout_s", 15.0))
        self.sigterm_timeout_s = float(rospy.get_param("~sigterm_timeout_s", 5.0))

        # ---- Modo único ---------------------------------------------------
        raw_mode = rospy.get_param("~mode", None)
        if raw_mode is None:
            rospy.logfatal(
                "[fdpo_initializer] Parametro ~mode em falta. Abortar."
            )
            sys.exit(2)

        try:
            self.mode = _parse_mode(raw_mode)
        except Exception as exc:
            rospy.logfatal("[fdpo_initializer] mode invalido: %s", exc)
            sys.exit(2)

        rospy.loginfo(
            "[fdpo_initializer] Modo unico: %s/%s args=%s",
            self.mode.launch_pkg,
            self.mode.launch_file,
            self.mode.args,
        )
        rospy.loginfo(
            "[fdpo_initializer] FDPO_ROBOT_ID=%s",
            os.environ.get("FDPO_ROBOT_ID", "(nao definido)"),
        )

        # ---- Publisher de estado ------------------------------------------
        self.status_pub = rospy.Publisher(
            self.status_topic, String, queue_size=1, latch=True
        )

        # ---- Supervisor ----------------------------------------------------
        self.supervisor = StackSupervisor(
            sigint_timeout_s=self.sigint_timeout_s,
            sigterm_timeout_s=self.sigterm_timeout_s,
            source_setup=self.source_setup or None,
        )

        # ---- State machine -------------------------------------------------
        self.fsm = InitializerStateMachine(
            mode=self.mode,
            start_stack=self._start_stack,
            stop_stack=self._stop_stack,
            on_state_change=self._publish_status,
        )

        # ---- Botão ---------------------------------------------------------
        self.button = GpioButton(
            backend=self.backend,
            pin=self.gpio_pin,
            active_low=self.active_low,
            debounce_ms=self.debounce_ms,
            hold_time_ms=self.hold_time_ms,
            sim_topic=self.sim_topic,
            on_pressed=self.fsm.on_button_pressed,
            on_released=self.fsm.on_button_released,
            on_held=self.fsm.on_button_held,
        )
        rospy.loginfo(
            "[fdpo_initializer] Botao pronto (backend=%s, pin=%s, active_low=%s, hold=%dms)",
            self.button.backend,
            self.gpio_pin,
            self.active_low,
            self.hold_time_ms,
        )

        rospy.on_shutdown(self._on_shutdown)

    # ------------------------------------------------------------- bindings

    def _start_stack(self, mode: ModeCommand) -> bool:
        return self.supervisor.start(
            launch_pkg=mode.launch_pkg,
            launch_file=mode.launch_file,
            args=mode.args,
            on_exit=self._on_stack_exit,
        )

    def _stop_stack(self) -> bool:
        return self.supervisor.stop()

    def _on_stack_exit(self, return_code: int) -> None:
        self.fsm.on_stack_exited(return_code)

    def _publish_status(self, state: str, payload: Dict[str, Any]) -> None:
        try:
            msg = String()
            msg.data = json.dumps(payload, ensure_ascii=False)
            self.status_pub.publish(msg)
        except Exception:
            rospy.logdebug_throttle(5.0, "[fdpo_initializer] Falha a publicar status")

    # ---------------------------------------------------------------- spin

    def spin(self) -> None:
        rospy.spin()

    def _on_shutdown(self) -> None:
        rospy.loginfo("[fdpo_initializer] Shutdown: a encerrar stack filho se existir")
        try:
            self.supervisor.stop()
        except Exception:
            rospy.logexception("[fdpo_initializer] Erro ao parar stack em shutdown")
        try:
            self.button.close()
        except Exception:
            pass
        try:
            self.fsm.shutdown()
        except Exception:
            pass


def main() -> None:
    node = InitializerNode()
    node.spin()


if __name__ == "__main__":
    main()
