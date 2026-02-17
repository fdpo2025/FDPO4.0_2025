#!/bin/bash

###############################################################################
# Script para publicar caminhos planejados em dois robôs via SSH
# Assume que o ROS já está rodando em ambos os robôs
# 
# Uso:
#   ./publish_paths_both.sh [opções]
#
# IPs e caminhos estão hardcoded no script:
#   Robô 1: 10.242.255.166
#   Robô 2: 10.242.202.243
#
# Exemplo:
#   ./publish_paths_both.sh
#   ./publish_paths_both.sh --pass1 "senha1" --pass2 "senha2"
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
WORKSPACE_PATH="~/catkin_ws_fdpo"
DELAY_SECONDS=0
USE_SSHPASS=false

# Função de ajuda
show_help() {
    echo "Uso: $0 [opções]"
    echo ""
    echo "Este script publica caminhos planejados em ambos os robôs."
    echo "Assume que o ROS já está rodando em ambos os robôs."
    echo ""
    echo "IPs e caminhos estão hardcoded:"
    echo "  Robô 1: ${ROBOT1_HOST}"
    echo "  Robô 2: ${ROBOT2_HOST}"
    echo ""
    echo "Opções:"
    echo "  --user USER            Usuário SSH (padrão: user)"
    echo "  --pass1 PASSWORD       Senha SSH para o primeiro robô (requer sshpass)"
    echo "  --pass2 PASSWORD       Senha SSH para o segundo robô (requer sshpass)"
    echo "  --delay SECONDS        Delay em segundos para o segundo robô (padrão: 0)"
    echo "  --workspace PATH       Caminho do workspace ROS (padrão: ~/catkin_ws_fdpo)"
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
    echo "  $0 --delay 3"
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
        --delay)
            DELAY_SECONDS="$2"
            shift 2
            ;;
        --workspace)
            WORKSPACE_PATH="$2"
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

# Função para verificar se ROS está rodando
check_ros_running() {
    local host=$1
    local password="$2"
    local robot_num=$3
    
    echo -e "${YELLOW}Verificando se ROS está rodando no robô ${robot_num} (${host})...${NC}"
    
    # Verificar se rosmaster está rodando
    local cmd="pgrep -f rosmaster > /dev/null 2>&1"
    if ssh_exec "$host" "$cmd" "$password" > /dev/null 2>&1; then
        echo -e "${GREEN}✓ ROS está rodando no robô ${robot_num}${NC}"
        return 0
    else
        echo -e "${RED}✗ ROS não está rodando no robô ${robot_num}${NC}"
        echo -e "${YELLOW}  Execute primeiro o roslaunch no robô antes de usar este script${NC}"
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
echo -e "${GREEN}Publicação de Caminhos Planejados${NC}"
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

# Verificar se ROS está rodando
echo -e "${YELLOW}Verificando se ROS está rodando...${NC}"
if ! check_ros_running "$ROBOT1_HOST" "$ROBOT1_PASSWORD" 1; then
    exit 1
fi
if ! check_ros_running "$ROBOT2_HOST" "$ROBOT2_PASSWORD" 2; then
    exit 1
fi
echo ""

# Publicar no primeiro robô
publish_planned_paths "$ROBOT1_HOST" 1 "$ROBOT1_PASSWORD" "$PLANNED_PATHS_DATA_R1"
echo ""

# Aguardar delay antes de publicar no segundo robô
if [ "$DELAY_SECONDS" -gt 0 ]; then
    echo -e "${YELLOW}Aguardando ${DELAY_SECONDS} segundos antes de publicar no robô 2...${NC}"
    sleep "$DELAY_SECONDS"
fi

# Publicar no segundo robô
publish_planned_paths "$ROBOT2_HOST" 2 "$ROBOT2_PASSWORD" "$PLANNED_PATHS_DATA_R2"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Processo concluído!${NC}"
echo -e "${GREEN}========================================${NC}"


