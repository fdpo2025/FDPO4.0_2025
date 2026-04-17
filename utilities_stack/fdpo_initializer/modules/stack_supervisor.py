"""Supervisor de um processo ``roslaunch`` filho.

O supervisor arranca o stack principal (``wake_up_fdpo*.launch``) como
**processo filho numa process group separada** e oferece ``start``, ``stop``
e ``is_running``. O ``stop`` envia SIGINT à *process group* inteira (como
Ctrl+C), faz escalada para SIGTERM e por fim SIGKILL se o processo não
terminar dentro dos timeouts configurados.

Motivação: um único ``SIGINT`` ao grupo garante que todos os nós arrancados
pelo ``roslaunch`` filho saem de forma coerente (incluindo drivers que
fecham a série do LiDAR / Pico), sem precisar de ``rosnode kill`` em massa.
"""

from __future__ import annotations

import os
import shlex
import signal
import subprocess
import threading
import time
from typing import Dict, List, Optional

import rospy


class StackSupervisor:
    """Lança/termina um ``roslaunch`` filho de forma robusta."""

    def __init__(
        self,
        sigint_timeout_s: float = 15.0,
        sigterm_timeout_s: float = 5.0,
        source_setup: Optional[str] = None,
    ) -> None:
        """
        Args:
            sigint_timeout_s: quanto tempo esperar após SIGINT antes de SIGTERM.
            sigterm_timeout_s: quanto tempo esperar após SIGTERM antes de SIGKILL.
            source_setup: caminho para ``devel/setup.bash`` a dar source antes
                do ``roslaunch``. Se None, assume-se que o ambiente já está
                configurado (tipicamente pelo systemd/launch pai).
        """
        self._sigint_timeout_s = sigint_timeout_s
        self._sigterm_timeout_s = sigterm_timeout_s
        self._source_setup = source_setup

        self._proc: Optional[subprocess.Popen] = None
        self._lock = threading.Lock()
        self._watch_thread: Optional[threading.Thread] = None
        self._on_exit_cb = None

    # ---------------------------------------------------------------- public

    def start(
        self,
        launch_pkg: str,
        launch_file: str,
        args: Optional[Dict[str, str]] = None,
        on_exit: Optional[callable] = None,
    ) -> bool:
        """Arranca o roslaunch filho. Retorna True se foi arrancado agora."""
        with self._lock:
            if self._proc is not None and self._proc.poll() is None:
                rospy.logwarn(
                    "[fdpo_initializer] start() chamado mas ja ha um stack filho (pid=%s)",
                    self._proc.pid,
                )
                return False

            cmd_str = self._build_command(launch_pkg, launch_file, args or {})
            rospy.loginfo("[fdpo_initializer] A arrancar stack: %s", cmd_str)

            try:
                self._proc = subprocess.Popen(
                    ["bash", "-lc", cmd_str],
                    preexec_fn=os.setsid,  # nova process group para SIGINT ao grupo
                    stdout=None,
                    stderr=None,
                )
            except Exception:
                rospy.logexception("[fdpo_initializer] Falha a arrancar roslaunch filho")
                self._proc = None
                return False

            self._on_exit_cb = on_exit
            self._watch_thread = threading.Thread(
                target=self._watch_process, name="stack_watch", daemon=True
            )
            self._watch_thread.start()
            return True

    def stop(self) -> bool:
        """Termina o filho (SIGINT→SIGTERM→SIGKILL) à process group."""
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                return False

        pgid = self._safe_pgid(proc)
        if pgid is None:
            rospy.logwarn("[fdpo_initializer] Nao consegui obter pgid; a usar pid")

        rospy.loginfo(
            "[fdpo_initializer] A parar stack (pid=%s, pgid=%s)...", proc.pid, pgid
        )

        # 1) SIGINT -> deixa o roslaunch fazer shutdown coordenado
        self._signal(proc, pgid, signal.SIGINT)
        if self._wait(proc, self._sigint_timeout_s):
            rospy.loginfo("[fdpo_initializer] Stack terminado via SIGINT")
            return True

        # 2) SIGTERM
        rospy.logwarn(
            "[fdpo_initializer] Stack nao reagiu a SIGINT em %.1fs; a enviar SIGTERM",
            self._sigint_timeout_s,
        )
        self._signal(proc, pgid, signal.SIGTERM)
        if self._wait(proc, self._sigterm_timeout_s):
            rospy.loginfo("[fdpo_initializer] Stack terminado via SIGTERM")
            return True

        # 3) SIGKILL (último recurso)
        rospy.logerr(
            "[fdpo_initializer] Stack nao reagiu a SIGTERM em %.1fs; SIGKILL",
            self._sigterm_timeout_s,
        )
        self._signal(proc, pgid, signal.SIGKILL)
        self._wait(proc, 2.0)
        return True

    def is_running(self) -> bool:
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    @property
    def pid(self) -> Optional[int]:
        with self._lock:
            return self._proc.pid if self._proc is not None else None

    # --------------------------------------------------------------- helpers

    def _build_command(
        self, launch_pkg: str, launch_file: str, args: Dict[str, str]
    ) -> str:
        """Monta a linha de comando bash que arranca o roslaunch."""
        parts: List[str] = []
        if self._source_setup:
            parts.append(f"source {shlex.quote(self._source_setup)}")
        roslaunch_cmd = ["roslaunch", launch_pkg, launch_file]
        for key, value in args.items():
            roslaunch_cmd.append(f"{key}:={value}")
        parts.append(" ".join(shlex.quote(tok) for tok in roslaunch_cmd))
        return " && ".join(parts)

    @staticmethod
    def _safe_pgid(proc: subprocess.Popen) -> Optional[int]:
        try:
            return os.getpgid(proc.pid)
        except (ProcessLookupError, PermissionError):
            return None

    @staticmethod
    def _signal(proc: subprocess.Popen, pgid: Optional[int], sig: int) -> None:
        try:
            if pgid is not None:
                os.killpg(pgid, sig)
            else:
                proc.send_signal(sig)
        except ProcessLookupError:
            pass
        except Exception:
            rospy.logexception(
                "[fdpo_initializer] Falha ao enviar sinal %s ao stack", sig
            )

    @staticmethod
    def _wait(proc: subprocess.Popen, timeout_s: float) -> bool:
        """Espera pelo fim do processo; True se terminou dentro do timeout."""
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if proc.poll() is not None:
                return True
            time.sleep(0.1)
        return proc.poll() is not None

    def _watch_process(self) -> None:
        """Corre num thread: espera o filho sair e notifica via callback."""
        proc = self._proc
        if proc is None:
            return
        try:
            rc = proc.wait()
        except Exception:
            rc = -1
        rospy.loginfo(
            "[fdpo_initializer] roslaunch filho (pid=%s) terminou com rc=%s",
            proc.pid,
            rc,
        )
        cb = self._on_exit_cb
        with self._lock:
            # Se este ainda é o proc "actual", limpa.
            if self._proc is proc:
                self._proc = None
                self._on_exit_cb = None
        if cb is not None:
            try:
                cb(rc)
            except Exception:
                rospy.logexception("[fdpo_initializer] on_exit callback falhou")
