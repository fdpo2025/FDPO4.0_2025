# ControllerPoint message
# Representa um ponto de controlo com coordenadas e parâmetros de navegação

int32 node_id
float64 x                    # Coordenada x
float64 y                    # Coordenada y
float64 line_switch_ratio    # Ratio da linha para mudar para próxima (0.75 = 75%)
float64 vel_lin_nom          # Velocidade linear nominal para esta linha
bool backwards               # Se deve ir para trás neste ponto
bool pick_box                # Se é warehouse de pick (true) ou drop (false)
bool is_warehouse            # Se o ponto final é uma warehouse
bool is_process_warehouse    # Se é warehouse de process (pick/drop em nós de processo)
