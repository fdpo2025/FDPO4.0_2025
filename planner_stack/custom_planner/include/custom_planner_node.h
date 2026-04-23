#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt8.h>

#include <pi_pico_driver/RadioNetworkTable.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

/*
 * Novo modelo de sincronização (sem mensagens de rádio extra):
 *
 *   Nomenclatura:
 *     - MSG_Y    na timeline do produtor X → "mensagem para o robô Y".
 *     - WAIT_X   na timeline do consumidor Y → "espero por mensagem vinda do robô X".
 *     - O emparelhamento é o par ordenado (produtor X, consumidor Y).
 *
 *   Libertação:
 *     - O i-ésimo WAIT_X em Y liberta quando o produtor X já tiver atravessado
 *       o i-ésimo MSG_Y na sua própria timeline.
 *     - Cada robô publica my_timeline_index (cursor monotónico, uint8) via Pico→rádio.
 *       Os restantes robôs contam localmente quantos MSG_<meu_id> o produtor X atravessou.
 *
 *   Validação estática (arranque de missão):
 *     - Para cada par (X, Y): #MSG_Y emitidos por X ≥ #WAIT_X consumidos por Y,
 *       caso contrário a missão é abortada (prevenção de deadlock).
 *
 *   Parsing:
 *     - WAIT_X   ou  W_X            (sufixo _N obrigatório; 0 ≤ X < num_robots)
 *     - MSG_Y                       (0 ≤ Y < num_robots; MSG_ALL aceite mas ignorado para ticks)
 *     - <color>_W_X / <color>_WAIT_X (pause no pick, espera produtor X)
 *     - "WAIT"/"W" sem sufixo são rejeitados com ROS_ERROR.
 */

class CustomPlannerNode {
 public:
  explicit CustomPlannerNode(ros::NodeHandle& nh);

 private:
  enum MissionState {
    STATE_IDLE = 0,
    STATE_NAVIGATING = 1,
    STATE_WAITING = 2
  };

  struct MissionTask {
    /** Tokens após expansão. Valores:
     *   - inteiro >= 0         -> nó físico na timeline de navegação
     *   - -1000 - Y (Y 0..255) -> MSG_Y (mensagem para o robô Y)
     *   - -2000 - X (X 0..255) -> WAIT_X (espera por mensagem do robô X)
     *   - -3000                -> WAIT_FINAL (bloqueio local permanente)
     * "WAIT"/"W" sem sufixo já não existem; são rejeitados no parsing.
     */
    std::vector<int> timeline_tokens;
    std::vector<int> nav_nodes;
    std::vector<uint32_t> token_required_consumed_count;
    /** plan_handler ignora o 1º nó se for warehouse; ajuste no consumed_count. */
    uint8_t plan_handler_skips_first_nav_waypoint = 0;
    /** Índice inicial desta task na timeline global (concatenação de todas as legs). */
    uint32_t task_start_global_index = 0;
  };

  /** Timeline pré-computada para cada robô da missão ativa (índice = robot_id). */
  struct PeerTimeline {
    std::vector<int> timeline_tokens;                // mesmo esquema que MissionTask
    std::vector<uint8_t> expected_cp_at_index;       // último nó físico visto até i (ou 0)
    std::vector<uint8_t> has_expected_cp_at_index;   // bool: 1 se expected_cp_at_index[i] é válido
    /** Para o produtor, mapa destinatário Y -> posições na sua timeline onde emite MSG_Y. */
    std::map<int, std::vector<uint32_t>> msg_positions_by_target;
    /** Para o waiter, mapa produtor X -> nº total de WAIT_X (usado na validação estática). */
    std::map<int, uint32_t> wait_counts_by_producer;
  };

  void onMissionColorSequence(const std_msgs::String::ConstPtr& msg);
  void onRobotIdentity(const std_msgs::Int32::ConstPtr& msg);
  void applyRobotIdentity(int new_id);
  void onThisCurrentPose(const std_msgs::UInt32::ConstPtr& msg);
  void onNavRoutePauseRequest(const std_msgs::Bool::ConstPtr& msg);
  void onNavPlanWaypointConsumed(const std_msgs::UInt32MultiArray::ConstPtr& msg);
  void onNetworkTable(const pi_pico_driver::RadioNetworkTable::ConstPtr& msg);

  bool loadMissions();
  std::string selectMangaKey(const std::string& color_sequence) const;
  std::vector<int> resolveMissionLeg(const XmlRpc::XmlRpcValue& leg, const std::string& color_sequence,
                                     std::vector<uint32_t>* leg_pause_out,
                                     bool collapse_duplicates = true) const;
  int resolveIndexedColorNode(const std::string& token, const std::string& color_sequence) const;
  bool parseColorWith900Suffix(const std::string& token, std::string& color_token_out, bool& wait_at_pick_out,
                               int& wait_producer_out, bool& use_900_approach_out, bool& end_on_900_out) const;
  /** MSG_Y -> escreve Y (destinatário) em target_out. MSG_ALL -> target_out=255 (informativo). */
  bool parseMessageToken(const XmlRpc::XmlRpcValue& item, int& target_out) const;
  /** WAIT_X (obrigatório sufixo). Retorna true e escreve X (produtor) em producer_out. */
  bool parseWaitToken(const XmlRpc::XmlRpcValue& item, int& producer_out) const;
  bool tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const;
  void appendWarehousePickupTraversal(int approach_source_shelf_node, int target_shelf_node,
                                      std::vector<int>& out, bool wait_at_pick, int wait_producer) const;
  void collapseConsecutiveDuplicateNodes(std::vector<int>& nodes) const;

  void startMission(const std::string& color_sequence);
  void publishMissionRoute(const std::vector<int>& nav_nodes);
  void publishActiveTaskDebugLocked();
  void processTimelineLocked();
  bool loadNextTaskLocked();
  void setState(MissionState new_state);

  bool validatePath(const std::vector<int>& path) const;
  void loadValidNodeIds();
  void publishSpawnPose(const std::string& manga_key);
  bool tryReadDouble(const XmlRpc::XmlRpcValue& v, double& out) const;

  /** Must match is_warehouse_coordinate in plan_handler_node.cpp (first-node skip rule). */
  static bool isWarehouseCoordinate(int node_id);

  /** Parses all robots' missions for the given manga + color sequence into peer_timelines_.
   *  Populates msg_positions_by_target and wait_counts_by_producer. Returns false em erro fatal. */
  bool buildPeerTimelinesLocked(const std::string& manga_key, const std::string& color_sequence);
  /** Validação estática: para cada par (produtor X, consumidor Y), verifica que
   *  #MSG_Y em X >= #WAIT_X em Y. Previne deadlocks. Retorna false se detetar problema. */
  bool validateChannelsLocked() const;
  /** Publica my_timeline_index_ (clamped 0..255). */
  void publishMyTimelineIndexLocked();
  /** Liberta WAIT_X pendente se ticks_from_producer_[X] >= consumed_from_producer_[X] + 1. */
  void tryReleaseWaitingLocked();
  /** Recalcula ticks_from_producer_ a partir de observed_timeline_index_ e peer_timelines_. */
  void recomputeTicksLocked();

  ros::NodeHandle nh_;
  ros::Subscriber mission_color_sub_;
  ros::Subscriber robot_identity_sub_;
  ros::Subscriber network_table_sub_;
  ros::Subscriber this_pose_sub_;
  ros::Subscriber nav_plan_waypoint_consumed_sub_;
  ros::Subscriber nav_route_pause_request_sub_;
  ros::Publisher planned_paths_pub_;
  ros::Publisher nav_pause_after_wp_index_pub_;
  ros::Publisher mission_state_pub_;
  ros::Publisher wt_send_pub_;
  ros::Publisher my_timeline_index_pub_;
  ros::Publisher timeline_tokens_pub_;
  ros::Publisher timeline_cursor_pub_;
  ros::Publisher active_task_nav_nodes_pub_;
  ros::Publisher last_executed_token_pub_;
  ros::Publisher spawn_pose_pub_;

  std::string color_sequence_topic_;
  std::string this_current_pose_topic_;
  std::string network_table_topic_;
  std::string my_timeline_index_topic_;
  int num_robots_ = 2;

  XmlRpc::XmlRpcValue missions_root_;
  std::vector<int> valid_node_ids_;
  std::unordered_map<int, int> color_input_node_by_index_;
  std::deque<MissionTask> pending_tasks_;
  MissionTask active_task_;
  bool has_active_task_ = false;
  size_t timeline_cursor_ = 0;
  int32_t last_executed_token_index_ = -1;
  int32_t last_executed_token_value_ = 0;
  uint32_t consumed_nav_nodes_count_ = 0;
  uint32_t active_nav_plan_seq_ = 0;
  bool active_nav_plan_seq_valid_ = false;
  bool mission_route_published_ = false;
  std::string last_color_sequence_;
  std::string pending_color_sequence_;
  int robot_id_ = -1;
  MissionState state_ = STATE_IDLE;
  mutable std::mutex mtx_;
  bool debug_verbose_ = false;

  /** Timelines completas de todos os robôs para a manga ativa. */
  std::vector<PeerTimeline> peer_timelines_;
  /** Último timeline_index observado (monotónico) por robô. */
  std::vector<uint32_t> observed_timeline_index_;
  /** Nº de MSG_<meu_id> já atravessados pelo produtor X, por cada X. */
  std::map<int, uint32_t> ticks_from_producer_;
  /** Nº de WAIT_X já consumidos localmente, por cada produtor X. */
  std::map<int, uint32_t> consumed_from_producer_;
  /** Produtor em que estamos atualmente a aguardar (-1 = não parado). */
  int waiting_on_producer_ = -1;
  /** Latch de WAIT_FINAL: true => bloqueio local sem auto-release. */
  bool waiting_final_latch_ = false;
  /** Cursor publicado da timeline local (0..N). */
  uint32_t my_timeline_index_ = 0;
};

