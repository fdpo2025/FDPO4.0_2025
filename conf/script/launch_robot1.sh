#!/bin/bash

###############################################################################
# Script para lançar ROS e publicar tópicos no primeiro robô via SSH
# 
# Uso:
#   ./launch_robot1.sh [opções]
#
# IP e caminho estão hardcoded no script:
#   Robô 1: 10.242.255.166
#
# Exemplo:
#   ./launch_robot1.sh
#   ./launch_robot1.sh --pass "senha"
#
###############################################################################

# Cores para output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Configuração fixa do robô
ROBOT_HOST="10.242.255.166"
PLANNED_PATHS_DATA="31 16 9 11 3 11 4 7 22 29 28 36 28 27 34 31 16 9 10 2 10 11 12 26 27 29 37"

# Valores configuráveis
SSH_USER="user"
ROBOT_PASSWORD=""
LAUNCH_FILE="conf wake_up_fdpo.launch"
WORKSPACE_PATH="~/catkin_ws_fdpo"
KILL_EXISTING=false
USE_SSHPASS=false

# Função de ajuda
show_help() {
    echo "Uso: $0 [opções]"
    echo ""
    echo "IP e caminho estão hardcoded:"
    echo "  Robô: ${ROBOT_HOST}"
    echo ""
    echo "Opções:"
    echo "  --user USER            Usuário SSH (padrão: user)"
    echo "  --pass PASSWORD        Senha SSH (requer sshpass)"
    echo "  --launch FILE          Arquivo launch (padrão: 'conf wake_up_fdpo.launch')"
    echo "  --workspace PATH       Caminho do workspace ROS (padrão: ~/catkin_ws_fdpo)"
    echo "  --kill                 Mata processos ROS existentes antes de iniciar"
    echo "  -h, --help             Mostra esta ajuda"
    echo ""
    echo "NOTA: Usar senhas não é recomendado por segurança."
    echo "      Configure chaves SSH para autenticação sem senha:"
    echo "      ssh-keygen -t rsa -b 4096"
    echo "      ssh-copy-id ${SSH_USER}@${ROBOT_HOST}"
    echo ""
    echo "Exemplos:"
    echo "  # Uso básico (com chaves SSH):"
    echo "  $0"
    echo ""
    echo "  # Com senha:"
    echo "  $0 --pass 'senha'"
    echo ""
    echo "  # Matar processos existentes antes de iniciar:"
    echo "  $0 --kill"
}

# Parse de argumentos
while [[ $# -gt 0 ]]; do
    case $1 in
        --user)
            SSH_USER="$2"
            shift 2
            ;;
        --pass)
            ROBOT_PASSWORD="$2"
            USE_SSHPASS=true
            shift 2
            ;;
        --launch)
            LAUNCH_FILE="$2"
            shift 2
            ;;
        --workspace)
            WORKSPACE_PATH="$2"
            shift 2
            ;;
        --kill)
            KILL_EXISTING=true
            shift
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
        echo "  ssh-copy-id ${SSH_USER}@${ROBOT_HOST}"
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

# Função para verificar conectividade SSH
check_ssh_connection() {
    local host=$1
    local password="$2"
    echo -e "${YELLOW}Verificando conexão SSH com ${host}...${NC}"
    if ssh_exec "$host" "echo 'OK'" "$password" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ Conexão SSH OK com ${host}${NC}"
        return 0
    else
        echo -e "${RED}✗ Falha na conexão SSH com ${host}${NC}"
        if [[ -z "$password" ]]; then
            echo -e "${YELLOW}  Dica: Se precisar de senha, use --pass${NC}"
        fi
        return 1
    fi
}

# Função para matar processos ROS existentes
kill_ros_processes() {
    local host=$1
    local password="$2"
    echo -e "${YELLOW}Matando processos ROS existentes em ${host}...${NC}"
    ssh_exec "$host" "pkill -f roslaunch; pkill -f rosmaster; pkill -f roscore" "$password" || true
    sleep 2
}

# Função para lançar ROS no robô
launch_ros() {
    local host=$1
    local password="$2"
    echo -e "${YELLOW}Lançando ROS no robô (${host})...${NC}"
    
    # Comando para executar roslaunch em background
    # Usar setsid e redirecionar tudo para garantir que SSH retorne imediatamente
    local cmd="cd ${WORKSPACE_PATH} && source devel/setup.bash && (setsid roslaunch ${LAUNCH_FILE} > /tmp/roslaunch_robot1.log 2>&1 < /dev/null &) && sleep 0.5 && echo 'OK'"
    
    # Executar com timeout para evitar que trave
    if [[ -n "$password" ]]; then
        timeout 10 sshpass -p "$password" ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${SSH_USER}@${host}" "$cmd" > /dev/null 2>&1
    else
        timeout 10 ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${SSH_USER}@${host}" "$cmd" > /dev/null 2>&1
    fi
    
    if [ $? -eq 0 ] || [ $? -eq 124 ]; then  # 124 é o código de saída do timeout (que é OK aqui)
        echo -e "${GREEN}✓ ROS launch iniciado no robô${NC}"
        sleep 2  # Dar tempo para o ROS iniciar
        return 0
    else
        echo -e "${RED}✗ Falha ao lançar ROS no robô${NC}"
        return 1
    fi
}

# Função para publicar no tópico /planned_paths
publish_planned_paths() {
    local host=$1
    local password="$2"
    local path_data="$3"
    
    if [[ -z "$path_data" ]]; then
        echo -e "${YELLOW}Nenhum dado fornecido para /planned_paths, pulando publicação${NC}"
        return 0
    fi
    
    echo -e "${YELLOW}Publicando em /planned_paths no robô (${host})...${NC}"
    echo -e "${YELLOW}  Caminho: [${path_data}]${NC}"
    
    # Converter espaços em vírgulas para o formato ROS
    local path_data_commas=$(echo "$path_data" | tr ' ' ',')
    
    # Comando para publicar no tópico usando formato YAML correto
    local cmd="cd ${WORKSPACE_PATH} && source devel/setup.bash && rostopic pub -1 /planned_paths std_msgs/Int32MultiArray \"{data: [${path_data_commas}]}\""
    
    ssh_exec "$host" "$cmd" "$password"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Publicação realizada no robô${NC}"
    else
        echo -e "${RED}✗ Falha ao publicar no robô${NC}"
        return 1
    fi
}

# Main execution
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Lançamento de Robô ROS${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Configuração:${NC}"
echo -e "  Robô: ${ROBOT_HOST}"
echo -e "  Usuário SSH: ${SSH_USER}"
echo ""

# Verificar conexão SSH
echo -e "${YELLOW}Verificando conexão SSH...${NC}"
if ! check_ssh_connection "$ROBOT_HOST" "$ROBOT_PASSWORD"; then
    exit 1
fi
echo ""

# Matar processos existentes se solicitado
if [ "$KILL_EXISTING" = true ]; then
    kill_ros_processes "$ROBOT_HOST" "$ROBOT_PASSWORD"
    echo ""
fi

# Lançar ROS no robô
launch_ros "$ROBOT_HOST" "$ROBOT_PASSWORD"
if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# Publicar no robô
publish_planned_paths "$ROBOT_HOST" "$ROBOT_PASSWORD" "$PLANNED_PATHS_DATA"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Processo concluído!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Para ver os logs: ssh ${SSH_USER}@${ROBOT_HOST} 'tail -f /tmp/roslaunch_robot1.log'"
echo ""
echo "Para matar os processos:"
echo "  ssh ${SSH_USER}@${ROBOT_HOST} 'pkill -f roslaunch'"

