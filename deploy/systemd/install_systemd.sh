#!/usr/bin/env bash
# Instala as systemd units do stack FDPO em /etc/systemd/system.
#
# Uso:
#   sudo ./install_systemd.sh

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UNITS=(fdpo-roscore.service fdpo-bootstrap.service)

if [[ $EUID -ne 0 ]]; then
    echo "[fdpo] install_systemd.sh precisa de root (sudo)." >&2
    exit 1
fi

echo "[fdpo] A verificar /etc/fdpo/env..."
if [[ ! -f /etc/fdpo/env ]]; then
    echo "[fdpo] AVISO: /etc/fdpo/env nao existe."
    echo "[fdpo]        Criar com 'FDPO_ROBOT_ID=<id>' antes de enable das units."
    echo "[fdpo]        Exemplo:"
    echo "[fdpo]          sudo install -d -m 0755 /etc/fdpo"
    echo "[fdpo]          sudo cp ${SCRIPT_DIR}/fdpo-env /etc/fdpo/env"
    echo "[fdpo]          sudo \$EDITOR /etc/fdpo/env"
fi

echo "[fdpo] A copiar units para /etc/systemd/system..."
for unit in "${UNITS[@]}"; do
    install -m 0644 "${SCRIPT_DIR}/${unit}" "/etc/systemd/system/${unit}"
    echo "[fdpo]   ${unit}"
done

echo "[fdpo] systemctl daemon-reload..."
systemctl daemon-reload

echo "[fdpo] A activar units (enable)..."
for unit in "${UNITS[@]}"; do
    systemctl enable "${unit}"
done

cat <<'EOS'

[fdpo] Units instaladas e activadas.

Arrancar agora (sem reboot):
  sudo systemctl start fdpo-roscore.service
  sudo systemctl start fdpo-bootstrap.service

Ver o estado:
  systemctl status fdpo-bootstrap.service
  journalctl -u fdpo-bootstrap.service -f

Desactivar em caso de problema:
  sudo systemctl disable --now fdpo-bootstrap.service fdpo-roscore.service
EOS
