#!/bin/bash

###############################################################################
# Script para desligar ROS em ambos os robôs via SSH
# 
# Uso:
#   ./kill_ros_both.sh [opções]
#
# IPs estão hardcoded no script:
#   Robô 1: 10.242.255.166
#   Robô 2: 10.242.202.243
#
# Exemplo:
#   ./kill_ros_both.sh
#   ./kill_ros_both.sh --pass1 "senha1" --pass2 "senha2"
#
###############################################################################

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuração fixa dos robôs
ROBOT1_HOST="10.242.255.166"
ROBOT2_HOST="10.242.202.243"

# Valores configuráveis
SSH_USER="user"
ROBOT1_PASSWORD=""
ROBOT2_PASSWORD=""
USE_SSHPASS=false

# Função de ajuda
show_help() {
    echo "Uso: $0 [opções]"
    echo ""
    echo "IPs estão hardcoded:"
    echo "  Robô 1: ${ROBOT1_HOST}"
    echo "  Robô 2: ${ROBOT2_HOST}"
    echo ""
    echo "Opções:"
    echo "  --user USER            Usuário SSH (padrão: user)"
    echo "  --pass1 PASSWORD       Senha SSH do robô 1 (requer sshpass)"
    echo "  --pass2 PASSWORD       Senha SSH do robô 2 (requer sshpass)"
    echo "  -h, --help             Mostra esta ajuda"
    echo ""
    echo "NOTA: Usar senhas não é recomendado por segurança."
    echo "      Configure chaves SSH para autenticação sem senha:"
    echo "      ssh-keygen -t rsa -b 4096"
    echo "      ssh-copy-id ${SSH_USER}@${ROBOT1_HOST}"
    echo "      ssh-copy-id ${SSH_USER}@${ROBOT2_HOST}"
    echo ""
    echo "Exemplos:"
    echo "  # Uso básico (com chaves SSH):"
    echo "  $0"
    echo ""
    echo "  # Com senhas:"
    echo "  $0 --pass1 'senha1' --pass2 'senha2'"
    echo ""
    echo "  # Apenas robô 1 precisa de senha:"
    echo "  $0 --pass1 'senha1'"
}

# Parse de argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        --user)
            SSH_USER="$2"
            shift 2
            ;;
        --pass1)
            ROBOT1_PASSWORD="$2"
            USE_SSHPASS=true
            shift 2
            ;;
        --pass2)
            ROBOT2_PASSWORD="$2"
            USE_SSHPASS=true
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}Erro: Opção desconhecida '$1'${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Verificar se sshpass está instalado quando necessário
if [ "$USE_SSHPASS" = true ]; then
    if ! command -v sshpass &> /dev/null; then
        echo -e "${RED}Erro: sshpass não está instalado${NC}"
        echo "Instale com: sudo apt-get install sshpass"
        echo ""
        echo -e "${YELLOW}Recomendação: Configure chaves SSH em vez de usar senhas:${NC}"
        echo "  ssh-keygen -t rsa -b 4096"
        echo "  ssh-copy-id ${SSH_USER}@${ROBOT1_HOST}"
        echo "  ssh-copy-id ${SSH_USER}@${ROBOT2_HOST}"
        exit 1
    fi
    echo -e "${YELLOW}⚠ Aviso: Usando senhas via sshpass não é recomendado por segurança${NC}"
    echo -e "${YELLOW}   Configure chaves SSH para autenticação sem senha${NC}"
    echo ""
fi

# Função para executar comando via SSH
ssh_exec() {
    local host=$1
    local cmd=$2
    local password="$3"
    
    if [[ -n "$password" ]]; then
        # Usar sshpass quando senha fornecida
        sshpass -p "$password" ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${SSH_USER}@${host}" "$cmd"
    else
        # SSH normal (usa chaves ou pede senha interativamente)
        ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${SSH_USER}@${host}" "$cmd"
    fi
}

# Função para matar processos ROS
kill_ros_processes() {
    local host=$1
    local password="$2"
    local robot_num=$3
    
    echo -e "${YELLOW}Desligando ROS no robô ${robot_num} (${host})...${NC}"
    
    # Comando para matar todos os processos ROS
    local cmd="pkill -f roslaunch; pkill -f rosmaster; pkill -f roscore; pkill -f rosout; pkill -f rosnode; echo 'Processos ROS terminados'"
    
    ssh_exec "$host" "$cmd" "$password" > /dev/null 2>&1
}
# Main execution
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Desligar ROS em Ambos os Robôs${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Configuração:${NC}"
echo -e "  Robô 1: ${ROBOT1_HOST}"
echo -e "  Robô 2: ${ROBOT2_HOST}"
echo -e "  Usuário SSH: ${SSH_USER}"
echo ""

# Desligar ROS no robô 1
kill_ros_processes "$ROBOT1_HOST" "$ROBOT1_PASSWORD" "1"
echo ""

# Desligar ROS no robô 2
kill_ros_processes "$ROBOT2_HOST" "$ROBOT2_PASSWORD" "2"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Processo concluído!${NC}"
echo -e "${GREEN}========================================${NC}"




