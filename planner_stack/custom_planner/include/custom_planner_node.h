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
 *   - Cada produtor de um canal N contribui para um contador global ticks[N] ao atravessar
 *     um token "MSG_N" na sua timeline. Isto é detectado pelos restantes robôs observando o
 *     timeline_index que cada Pico publica em RadioNetworkTable (broadcast no slot TDMA).
 *   - Cada waiter possui tokens "WAIT_N" ou "<algo>_W_N". A i-ésima ocorrência só liberta
 *     quando ticks[N] >= i.
 *   - Tokens "WAIT" / "_W" sem sufixo "_N" são rejeitados durante o parsing.
 *   - my_timeline_index é o cursor monotónico da nossa timeline, propagado via Pico→rádio.
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
     *   - inteiro >= 0      -> nó físico na timeline de navegação
     *   - -1000 - N (N 0..255) -> MSG_N (produtor)
     *   - -2000 - N (N 0..255) -> WAIT_N (waiter)
     * Leading "W" (-1 puro) já não existe; é sempre sufixado.
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
    std::map<int, std::vector<uint32_t>> msg_positions;   // MSG_N -> posições (índices de timeline) onde produz
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
                               int& wait_channel_out, bool& use_900_approach_out, bool& end_on_900_out) const;
  /** MSG_N, MSG_ALL -> devolve N em msg_channel_out (255 para MSG_ALL, mas não produz canal).
   *  MSG_ALL continua suportado como "aviso broadcast" mas NÃO incrementa ticks em nenhum canal;
   *  é deprecado no novo esquema e deve ser evitado. */
  bool parseMessageToken(const XmlRpc::XmlRpcValue& item, int& msg_channel_out) const;
  /** WAIT_N (obrigatório sufixo). Retorna true e escreve channel se reconhecido. */
  bool parseWaitToken(const XmlRpc::XmlRpcValue& item, int& channel_out) const;
  bool tryReadInt(const XmlRpc::XmlRpcValue& item, int& value) const;
  void appendWarehousePickupTraversal(int approach_source_shelf_node, int target_shelf_node,
                                      std::vector<int>& out, bool wait_at_pick, int wait_channel) const;
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
   *  Populates msg_positions. Returns false on fatal parse error. */
  bool buildPeerTimelinesLocked(const std::string& manga_key, const std::string& color_sequence);
  /** Static validation: for every channel N, count WAIT_N tokens across all waiters and ensure
   *  producers emit at least that many MSG_N. Prevents deadlocks from bad missions.yaml.
   *  Logs errors (does not throw); returns false if any issue found. */
  bool validateChannelsLocked() const;
  /** Publish current my_timeline_index_ (clamped 0..255). */
  void publishMyTimelineIndexLocked();
  /** Release any pending WAIT_N if ticks_on_channel_[N] reached the needed count. Expects mtx_. */
  void tryReleaseWaitingLocked();
  /** Recompute ticks_on_channel_ from observed_timeline_index_ and peer_timelines_. Expects mtx_. */
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
  /** Contagem derivada de MSG_N efetivamente produzidos (sobre todos os produtores). */
  std::map<int, uint32_t> ticks_on_channel_;
  /** Nº de WAIT_N já consumidos por nós (waiter local). */
  std::map<int, uint32_t> consumed_on_channel_;
  /** Canal em que estamos atualmente parados (-1 = não parado por canal). */
  int waiting_on_channel_ = -1;
  /** Cursor publicado da timeline local (0..N). */
  uint32_t my_timeline_index_ = 0;
};

