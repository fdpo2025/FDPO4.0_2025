#!/bin/bash

###############################################################################
# Script para lançar ROS e publicar tópicos em dois robôs via SSH
# 
# Uso:
#   ./launch_dual_robots.sh [opções]
#
# IPs e caminhos estão hardcoded no script:
#   Robô 1: 10.242.255.166
#   Robô 2: 10.242.202.243
#
# Exemplo:
#   ./launch_dual_robots.sh
#   ./launch_dual_robots.sh --pass1 "senha1" --pass2 "senha2"
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
PLANNED_PATHS_DATA_R1="16 9 11 3 11 4 7 22 29 28 36 28 27 34 31 16 9 10 2 10 11 12 26 27 29 37"
PLANNED_PATHS_DATA_R2="31 16 9 1 9 11 4 7 30 38 30 27 34 31 8 0 8 11 12 26 27 35"

# Valores configuráveis
SSH_USER="user"
ROBOT1_PASSWORD=""
ROBOT2_PASSWORD=""
LAUNCH_FILE="conf wake_up_fdpo.launch"
DELAY_SECONDS=0
WORKSPACE_PATH="~/catkin_ws_fdpo"
KILL_EXISTING=false
USE_SSHPASS=false

# Função de ajuda
show_help() {
    echo "Uso: $0 [opções]"
    echo ""
    echo "IPs e caminhos estão hardcoded:"
    echo "  Robô 1: ${ROBOT1_HOST}"
    echo "  Robô 2: ${ROBOT2_HOST}"
    echo ""
    echo "Opções:"
    echo "  --user USER            Usuário SSH (padrão: user)"
    echo "  --pass1 PASSWORD       Senha SSH para o primeiro robô (requer sshpass)"
    echo "  --pass2 PASSWORD       Senha SSH para o segundo robô (requer sshpass)"
    echo "  --launch FILE          Arquivo launch (padrão: 'conf wake_up_fdpo.launch')"
    echo "  --delay SECONDS        Delay em segundos para o segundo robô (padrão: 0)"
    echo "  --workspace PATH       Caminho do workspace ROS (padrão: ~/catkin_ws_fdpo)"
    echo "  --kill                 Mata processos ROS existentes antes de iniciar"
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
    echo "  # Com delay customizado:"
    echo "  $0 --delay 5"
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
        --launch)
            LAUNCH_FILE="$2"
            shift 2
            ;;
        --delay)
            DELAY_SECONDS="$2"
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
            echo -e "${YELLOW}  Dica: Se precisar de senha, use --pass1 ou --pass2${NC}"
        fi
        return 1
    fi
}

# Função para matar processos ROS existentes
kill_ros_processes() {
    local host=$1
    local password="$2"
    echo -e "${YELLOW}Killing processos ROS existentes em ${host}...${NC}"
    ssh_exec "$host" "pkill -f roslaunch; pkill -f rosmaster; pkill -f roscore" "$password" || true
    sleep 2
}

# Função para lançar ROS no robô
launch_ros() {
    local host=$1
    local robot_num=$2
    local password="$3"
    echo -e "${YELLOW}Launching ROS no robot ${robot_num} (${host})...${NC}"
    
    # Comando para executar roslaunch em background
    # Usar setsid e redirecionar tudo para garantir que SSH retorne imediatamente
    local cmd="cd ${WORKSPACE_PATH} && source devel/setup.bash && (setsid roslaunch ${LAUNCH_FILE} > /tmp/roslaunch_robot${robot_num}.log 2>&1 < /dev/null &) && sleep 0.5 && echo 'OK'"
    
    # Executar com timeout para evitar que trave
    if [[ -n "$password" ]]; then
        timeout 10 sshpass -p "$password" ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${SSH_USER}@${host}" "$cmd" > /dev/null 2>&1
    else
        timeout 10 ssh -o ConnectTimeout=5 -o StrictHostKeyChecking=no "${SSH_USER}@${host}" "$cmd" > /dev/null 2>&1
    fi
    
    if [ $? -eq 0 ] || [ $? -eq 124 ]; then  # 124 é o código de saída do timeout (que é OK aqui)
        echo -e "${GREEN} ROS launch iniciado no robô ${robot_num}${NC}"
        sleep 2  # Dar tempo para o ROS iniciar
        return 0
    else
        echo -e "${RED}✗ Falha ao lançar ROS no robô ${robot_num}${NC}"
        return 1
    fi
}

# Função para publicar no tópico /planned_paths
publish_planned_paths() {
    local host=$1
    local robot_num=$2
    local password="$3"
    local path_data="$4"
    
    if [[ -z "$path_data" ]]; then
        echo -e "${YELLOW}Nenhum dado fornecido para /planned_paths no robô ${robot_num}, pulando publicação${NC}"
        return 0
    fi
    
    echo -e "${YELLOW}Publicando em /planned_paths no robô ${robot_num} (${host})...${NC}"
    echo -e "${YELLOW}  Caminho: [${path_data}]${NC}"
    
    # Converter espaços em vírgulas para o formato ROS
    local path_data_commas=$(echo "$path_data" | tr ' ' ',')
    
    # Comando para publicar no tópico usando formato YAML correto
    local cmd="cd ${WORKSPACE_PATH} && source devel/setup.bash && rostopic pub -1 /planned_paths std_msgs/Int32MultiArray \"{data: [${path_data_commas}]}\""
    
    ssh_exec "$host" "$cmd" "$password"
    
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Publicação realizada no robô ${robot_num}${NC}"
    else
        echo -e "${RED}✗ Falha ao publicar no robô ${robot_num}${NC}"
        return 1
    fi
}

# Main execution
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Lançamento Dual de Robôs ROS${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo -e "${YELLOW}Configuração:${NC}"
echo -e "  Robô 1: ${ROBOT1_HOST}"
echo -e "  Robô 2: ${ROBOT2_HOST}"
echo -e "  Usuário SSH: ${SSH_USER}"
echo ""

# Verificar conexões SSH
echo -e "${YELLOW}Verificando conexões SSH...${NC}"
if ! check_ssh_connection "$ROBOT1_HOST" "$ROBOT1_PASSWORD"; then
    exit 1
fi
if ! check_ssh_connection "$ROBOT2_HOST" "$ROBOT2_PASSWORD"; then
    exit 1
fi
echo ""

# Matar processos existentes se solicitado
if [ "$KILL_EXISTING" = true ]; then
    kill_ros_processes "$ROBOT1_HOST" "$ROBOT1_PASSWORD"
    kill_ros_processes "$ROBOT2_HOST" "$ROBOT2_PASSWORD"
    echo ""
fi

# Lançar ROS no primeiro robô
launch_ros "$ROBOT1_HOST" 1 "$ROBOT1_PASSWORD"
if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# Lançar ROS no segundo robô
launch_ros "$ROBOT2_HOST" 2 "$ROBOT2_PASSWORD"
if [ $? -ne 0 ]; then
    exit 1
fi
echo ""

# Publicar no primeiro robô (sem delay)
publish_planned_paths "$ROBOT1_HOST" 1 "$ROBOT1_PASSWORD" "$PLANNED_PATHS_DATA_R1"
echo ""

# Aguardar delay antes de publicar no segundo robô
if [ "$DELAY_SECONDS" -gt 0 ]; then
    echo -e "${YELLOW}Aguardando ${DELAY_SECONDS} segundos antes de publicar no robô 2...${NC}"
    sleep "$DELAY_SECONDS"
fi

# Publicar no segundo robô (com delay)
publish_planned_paths "$ROBOT2_HOST" 2 "$ROBOT2_PASSWORD" "$PLANNED_PATHS_DATA_R2"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Processo concluído!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Para ver os logs do robô 1: ssh ${SSH_USER}@${ROBOT1_HOST} 'tail -f /tmp/roslaunch_robot1.log'"
echo "Para ver os logs do robô 2: ssh ${SSH_USER}@${ROBOT2_HOST} 'tail -f /tmp/roslaunch_robot2.log'"
echo ""
echo "Para matar os processos:"
echo "  ssh ${SSH_USER}@${ROBOT1_HOST} 'pkill -f roslaunch'"
echo "  ssh ${SSH_USER}@${ROBOT2_HOST} 'pkill -f roslaunch'"

