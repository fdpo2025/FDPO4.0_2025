#include "beacon_detector/beacon_detector_node.h"

// ---------------------------------------------------------------------------------------
//                               BeaconDetector Class 
// ---------------------------------------------------------------------------------------

BeaconDetector::BeaconDetector(ros::NodeHandle& nh, ros::NodeHandle& nh_priv) : nh(nh), nh_priv(nh_priv) {

	tf_buffer = new tf2_ros::Buffer();
	tf_listener = new tf2_ros::TransformListener(*tf_buffer);

    target_frame = "base_link";

    // Ler parâmetros do namespace privado
    nh_priv.param("max_match_dist", maxMatchDist, 0.2);
    nh_priv.param<std::string>("input_topic_type", input_topic_type, "laser_scan");

    ROS_INFO("BeaconDetector input_topic_type parameter: '%s'", input_topic_type.c_str());

    // Subscribe usando NodeHandle público (tópicos globais com "/" para absoluto)
    if (input_topic_type == "point_cloud") {
        // Usa a point cloud já compensada pelo mdistortion_compensator_node
        pointCloudSub = nh.subscribe("/laser_scan_point_cloud", 10, &BeaconDetector::processPointCloud, this);
        ROS_INFO("BeaconDetector subscribed to compensated PointCloud topic: /laser_scan_point_cloud");
    } else {
        sensorDataSub = nh.subscribe("/base_scan", 10, &BeaconDetector::processSensorData, this);
        ROS_INFO("BeaconDetector subscribed to LaserScan topic: /base_scan");
    }

    // Publicar usando NodeHandle público (tópicos globais com "/" para absoluto)
    beaconEstimation_pub = nh.advertise<localizer::BeaconMatch>("/beacon_estimation", 1);
    markers_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/dbscan_markers", 1, true);  // latched
    beacons_map_pub_  = nh.advertise<visualization_msgs::MarkerArray>("/beacons_map_markers", 1, true);  // latched 
    
    loadBeaconsFromParams();
    publishBeaconsInMap();

    ROS_INFO("BeaconDetector instance created.");

}

BeaconDetector::~BeaconDetector() {

    delete tf_listener;
	delete tf_buffer;

}

double BeaconDetector::normalizeAngle(double theta) {

    while(theta > M_PI) theta -= 2.0 * M_PI;
    while (theta <= -M_PI) theta += 2.0*M_PI;

    return theta;
}

void BeaconDetector::pointCloud2XY(const sensor_msgs::PointCloud2& cloud, std::vector<Point>& out)
{
    out.clear();

    sensor_msgs::PointCloud2ConstIterator<float> iterX(cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterY(cloud, "y");

    while ((iterX != iterX.end()) && (iterY != iterY.end())) {

        Point point(*iterX, *iterY);
        out.push_back(point);

        ++iterX;
        ++iterY;

    }
}

void BeaconDetector::dataClustering(std::vector<Point>& dataPoints) {

    clusters.clear();

    double eps;
    nh.param<double>("eps", eps, 0.07);
    
    int minPoints;
    nh.param<int>("minPoints", minPoints, 3);

    DBSCAN dbscan(dataPoints, eps, minPoints);
    dbscan.clusteringAlgorithm();

    clustersCentroids_robotFrame.clear();

    for(auto& cluster: dbscan.clusters) {

        clusters.push_back(cluster);
        clustersCentroids_robotFrame.push_back(cluster.centroid);

    }

    #ifdef RVIZ_VISUALIZATION
    publishClusters(dbscan.clusters);
    #endif

}

void BeaconDetector::loadBeaconsFromParams() {

    beacons_globalFrame.clear();

    XmlRpc::XmlRpcValue beaconsParams;

    if(nh.getParam("/beacon_detector_node/beacons", beaconsParams)) {

        for(int beacon_id = 0; beacon_id < static_cast<int>(beaconsParams.size()); beacon_id++) {

            Beacon beacon_temp;
            
            beacon_temp.name = static_cast<std::string>(beaconsParams[beacon_id]["name"]);
            beacon_temp.pose.x =  static_cast<double>(beaconsParams[beacon_id]["x"]);
            beacon_temp.pose.y =  static_cast<double>(beaconsParams[beacon_id]["y"]);
            beacon_temp.radius =  static_cast<double>(beaconsParams[beacon_id]["radius"]);

            beacons_globalFrame.push_back(beacon_temp);
        }

    }

    ROS_INFO("Loaded %zu beacons.", beacons_globalFrame.size());

}

void BeaconDetector::updateRobotFrameBeacons(const ros::Time& stamp) {

    beacons_robotFrame.clear();
    geometry_msgs::TransformStamped T_robot_map;

    try {
        // tenta no timestamp da medição
        T_robot_map = tf_buffer->lookupTransform("base_link", "map", stamp, ros::Duration(0.2));
    } catch (const tf2::ExtrapolationException& ex) {
        ROS_WARN_THROTTLE(1.0,
          "TF extrapolation (base_link<-map) em t=%.3f; a usar latest. %s",
          stamp.toSec(), ex.what());
        try {
            // fallback: última disponível
            T_robot_map = tf_buffer->lookupTransform("base_link", "map",
                                                     ros::Time(0));
        } catch (const tf2::TransformException& ex2) {
            ROS_WARN_THROTTLE(1.0, "Falha também no latest TF: %s", ex2.what());
            return;
        }
    } catch (const tf2::TransformException& ex) {
        // No arranque o frame "map" pode ainda não existir (EKF não publicou); não avisar
        const std::string msg = ex.what();
        if (msg.find("does not exist") == std::string::npos) {
            ROS_WARN_THROTTLE(1.0, "TF lookup falhou (base_link<-map): %s", ex.what());
        }
        return;
    }

    for(auto& beacon: beacons_globalFrame) {

        geometry_msgs::PointStamped p_mapFrame, p_robotFrame;
        p_mapFrame.header.frame_id = "map";
        p_mapFrame.header.stamp = stamp;

        p_mapFrame.point.x = beacon.pose.x;
        p_mapFrame.point.y = beacon.pose.y;
        p_mapFrame.point.z = 0.0;

        tf2::doTransform(p_mapFrame, p_robotFrame, T_robot_map);

        Beacon beacon_temp;
        beacon_temp.name = beacon.name;
        beacon_temp.pose.x = p_robotFrame.point.x;
        beacon_temp.pose.y = p_robotFrame.point.y;
        beacon_temp.radius = beacon.radius;

        beacons_robotFrame.push_back(beacon_temp);
    }

}

// Neste momento esta função depende da ordem no vetor dos beacons, portanto dps mudar para o Hungarian
void BeaconDetector::matchBeaconsToClusters() {

    beaconToCluster.clear();
    beacons_measured.clear();

    std::vector<bool> used(clustersCentroids_robotFrame.size(), false);

    double maxMatchDist2 = maxMatchDist * maxMatchDist;  

    for(int beacon_index = 0; beacon_index <  static_cast<int>(beacons_robotFrame.size()); ++beacon_index) {

        double bestDist2 = maxMatchDist2;
        int bestMatch = -1;

        for(int cluster_index = 0; cluster_index < static_cast<int>(clustersCentroids_robotFrame.size()); ++cluster_index) {
            
            if (used[cluster_index]) continue;

            double dx = beacons_robotFrame[beacon_index].pose.x - clustersCentroids_robotFrame[cluster_index].x;
            double dy = beacons_robotFrame[beacon_index].pose.y - clustersCentroids_robotFrame[cluster_index].y;

            double dist2 = dx*dx + dy*dy;

            if(dist2 < bestDist2) {

                bestDist2 = dist2;
                bestMatch = cluster_index;

            }

        } 
        
        if(bestMatch != -1) {

            used[bestMatch] = true;
            
            beaconToCluster[beacons_robotFrame[beacon_index].name] = bestMatch;

            Beacon beacon_temp;
            beacon_temp.name = beacons_robotFrame[beacon_index].name;
            beacon_temp.pose = clusters[bestMatch].centroid;
            beacon_temp.radius = beacons_robotFrame[beacon_index].radius;
            beacons_measured[beacons_robotFrame[beacon_index].name] = beacon_temp;
        
        }
    }

}

double BeaconDetector::computeCentroidOffset(const std::string& beacon_name, const double& beacon_radius) {

    auto it = beaconToCluster.find(beacon_name);
    if (it == beaconToCluster.end()) return 0.0;  // without association
    if (beacon_radius <= 0.0) return 0.0;        

    const Cluster& cluster_temp = clusters[it->second];
    if (cluster_temp.points.size() < 2) return 0.0;

    Pose point_c = cluster_temp.centroid;
    double aref = std::atan2(point_c.y, point_c.x);

    // to choose the extreme points 
    int i_min = 0, i_max = 0;
    double a_min = normalizeAngle(std::atan2(cluster_temp.points[0].y, cluster_temp.points[0].x) - aref);
    double a_max = a_min;

    for (int i = 1; i < (int)cluster_temp.points.size(); ++i) {
        double a = normalizeAngle(std::atan2(cluster_temp.points[i].y, cluster_temp.points[i].x) - aref);
        if (a < a_min) { a_min = a; i_min = i; }
        if (a > a_max) { a_max = a; i_max = i; }
    }
    Pose point_1 = cluster_temp.points[i_min];
    Pose point_n = cluster_temp.points[i_max];

    double L = std::hypot(point_1.x - point_n.x, point_1.y - point_n.y);
    double s = L/beacon_radius * 0.5;
    if(s > 1.0) s = 1.0;
    else if(s < -1.0) s = -1.0;

    double alfa = std::asin(s);

    double dist_c1 = std::hypot(point_c.x - point_1.x, point_c.y - point_1.y);
    double dist_cn = std::hypot(point_c.x - point_n.x, point_c.y - point_n.y);
    double dist_c_average = (dist_c1 + dist_cn) * 0.5;

    double cos_alfa = std::cos(alfa), cos_alfa_2 = cos_alfa * cos_alfa;

    double rad = beacon_radius*beacon_radius * cos_alfa_2 
               - (beacon_radius*beacon_radius - dist_c_average*dist_c_average);
    if (rad < 0.0) rad = 0.0; 
    
    double offset = beacon_radius * cos_alfa 
                  + std::sqrt(rad);

    return offset;
}


void BeaconDetector::beaconsCentroidCompensation() {

    for(auto& bn_to_beacon : beacons_measured) {

        Beacon& beacon = bn_to_beacon.second;

        const double r = std::hypot(beacon.pose.x, beacon.pose.y);
        double offset = computeCentroidOffset(beacon.name, beacon.radius);

        if (offset != 0.0 && r > 1e-6) {
            beacon.pose.x += offset * (beacon.pose.x / r);
            beacon.pose.y += offset * (beacon.pose.y / r);
        }

    }
 
}

void BeaconDetector::publishBeaconsEstimation(const std_msgs::Header& header) {

    if(beaconToCluster.empty()) return;

    localizer::BeaconMatch beacons_aux;

    beacons_aux.header = header;          
    beacons_aux.header.frame_id = target_frame; 

    for(const auto& beacon: beacons_robotFrame) {

        Cluster cluster_aux;

        auto it = beaconToCluster.find(beacon.name);
        if (it == beaconToCluster.end()) continue;   // no match -> end cycle          
        int cluster_index = it->second; 

        cluster_aux = clusters[cluster_index];
        int numPoints = static_cast<int>(cluster_aux.points.size());

        localizer::Cluster cluster_msg;
        cluster_msg.beacon_match_name = beacon.name;
        cluster_msg.num_points = numPoints;
        
        auto it_meas = beacons_measured.find(beacon.name);
        if (it_meas != beacons_measured.end()) {
            cluster_msg.centroid.x = it_meas->second.pose.x;  // centro corrigido
            cluster_msg.centroid.y = it_meas->second.pose.y;
        } else {
            cluster_msg.centroid.x = cluster_aux.centroid.x;  // fallback: centróide bruto
            cluster_msg.centroid.y = cluster_aux.centroid.y;
        }

        for(int point_id = 0; point_id < numPoints; ++point_id) {

            localizer::Pose point_aux; 
            point_aux.x = cluster_aux.points[point_id].x;
            point_aux.y = cluster_aux.points[point_id].y;

            cluster_msg.points.push_back(point_aux);

        }

        beacons_aux.clusters.push_back(cluster_msg);

    }

    beaconEstimation_pub.publish(beacons_aux);

}

void BeaconDetector::processSensorData(const sensor_msgs::LaserScan::ConstPtr& scan) {

    laser_geometry::LaserProjection projector;

    projector.projectLaser(*scan, pointCloud, -1.0);

    try {

        sensor_msgs::PointCloud2 pointCloud_targetFrame;
        tf_buffer->transform(pointCloud, pointCloud_targetFrame, target_frame, ros::Duration(1.0));
        pointCloud = std::move(pointCloud_targetFrame);

    } catch (const tf2::TransformException& ex) {

        ROS_WARN_THROTTLE(1.0, "Failed to project laser data: %s", ex.what());
        return;
    }

    std::vector<Point> dataPoints;
    pointCloud2XY(pointCloud, dataPoints);
    dataClustering(dataPoints);
    updateRobotFrameBeacons(scan->header.stamp);
    matchBeaconsToClusters();
    beaconsCentroidCompensation();
    publishBeaconsEstimation(scan->header);

    #ifdef RVIZ_VISUALIZATION
        publishBeaconAssociations();
    #endif

}

void BeaconDetector::processPointCloud(const sensor_msgs::PointCloud::ConstPtr& cloud) {

    // Convert PointCloud (v1) to PointCloud2
    sensor_msgs::convertPointCloudToPointCloud2(*cloud, pointCloud);

    try {

        sensor_msgs::PointCloud2 pointCloud_targetFrame;
        tf_buffer->transform(pointCloud, pointCloud_targetFrame, target_frame, ros::Duration(1.0));
        pointCloud = std::move(pointCloud_targetFrame);

    } catch (const tf2::TransformException& ex) {

        ROS_WARN_THROTTLE(1.0, "Failed to transform point cloud data: %s", ex.what());
        return;
    }

    std::vector<Point> dataPoints;
    pointCloud2XY(pointCloud, dataPoints);
    dataClustering(dataPoints);
    updateRobotFrameBeacons(cloud->header.stamp);
    matchBeaconsToClusters();
    beaconsCentroidCompensation();
    publishBeaconsEstimation(cloud->header);

    #ifdef RVIZ_VISUALIZATION
        publishBeaconAssociations();
    #endif

}


// RViz Visualization ------------------------------------------------------------

namespace {

    inline void colorFromId(std_msgs::ColorRGBA& c, std::size_t id) {
        static const float palette[][3] = {
            {0.90f,0.10f,0.10f}, {0.10f,0.60f,0.95f}, {0.10f,0.75f,0.20f},
            {0.95f,0.65f,0.10f}, {0.60f,0.10f,0.80f}, {0.10f,0.85f,0.80f},
            {0.85f,0.30f,0.10f}, {0.55f,0.55f,0.10f}
    };
        const auto& rgb = palette[id % (sizeof(palette)/sizeof(palette[0]))];
        c.r = rgb[0]; c.g = rgb[1]; c.b = rgb[2]; c.a = 1.0f;
    }
} 

void BeaconDetector::publishClusters(const std::vector<Cluster>& clusters) {
  visualization_msgs::MarkerArray arr;

  {
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(clear);
  }

  const ros::Time now = ros::Time::now();

  for (std::size_t i = 0; i < clusters.size(); ++i) {
    const auto& cl = clusters[i];

    // 1) Pontos do cluster
    visualization_msgs::Marker pts;
    pts.header.frame_id = target_frame;
    pts.header.stamp = now;
    pts.ns = "dbscan_points";
    pts.id = static_cast<int>(i);
    pts.type = visualization_msgs::Marker::POINTS;
    pts.action = visualization_msgs::Marker::ADD;
    pts.pose.orientation.w = 1.0;
    pts.scale.x = 0.05; // diâmetro dos pontos (m)
    pts.scale.y = 0.05;
    colorFromId(pts.color, i);
    pts.lifetime = ros::Duration(0.2);

    pts.points.reserve(cl.points.size());
    for (auto& p : cl.points) {
      geometry_msgs::Point gp;
      gp.x = p.x; gp.y = p.y; gp.z = 0.0;
      pts.points.push_back(gp);
    }
    arr.markers.push_back(pts);

    // 2) Centroide (esfera branca)
    visualization_msgs::Marker centroid;
    centroid.header = pts.header;
    centroid.ns = "dbscan_centroid";
    centroid.id = static_cast<int>(i);
    centroid.type = visualization_msgs::Marker::SPHERE;
    centroid.action = visualization_msgs::Marker::ADD;
    centroid.pose.position.x = cl.centroid.x;
    centroid.pose.position.y = cl.centroid.y;
    centroid.pose.position.z = 0.0;
    centroid.pose.orientation.w = 1.0;
    centroid.scale.x = centroid.scale.y = centroid.scale.z = 0.08;
    centroid.color.r = 1.0; centroid.color.g = 1.0; centroid.color.b = 1.0; centroid.color.a = 1.0;
    centroid.lifetime = ros::Duration(0.2);
    arr.markers.push_back(centroid);

    // 3) Etiqueta do cluster
    visualization_msgs::Marker label;
    label.header = pts.header;
    label.ns = "dbscan_label";
    label.id = static_cast<int>(i);
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::Marker::ADD;
    label.pose.position.x = cl.centroid.x;
    label.pose.position.y = cl.centroid.y;
    label.pose.position.z = 0.25;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.18;
    colorFromId(label.color, i);
    label.lifetime = ros::Duration(0.2);
    label.text = std::to_string(i);
    arr.markers.push_back(label);
  }

  markers_pub_.publish(arr);
}

void BeaconDetector::publishBeaconAssociations() {

    visualization_msgs::MarkerArray arr;
    const ros::Time now = ros::Time::now();

    // 1) Beacons previstos (no frame do robô)
    for (std::size_t i = 0; i < beacons_robotFrame.size(); ++i) {
        const auto& b = beacons_robotFrame[i];

        // 1a) Esfera do beacon
        visualization_msgs::Marker m;
        m.header.frame_id = target_frame;
        m.header.stamp    = now;
        m.ns   = "beacons_pred";
        m.id   = static_cast<int>(i);
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = b.pose.x;
        m.pose.position.y = b.pose.y;
        m.pose.position.z = 0.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.075;   // tamanho da esfera
        m.color.r = 1.0; m.color.g = 0.85; m.color.b = 0.10; m.color.a = 1.0; // “dourado”
        m.lifetime = ros::Duration(0.2);
        arr.markers.push_back(m);

        // 1b) Label com nome (+ dist se houver match)
        visualization_msgs::Marker label;
        label.header = m.header;
        label.ns = "beacon_name";
        label.id = static_cast<int>(i);
        label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::Marker::ADD;
        label.pose.position.x = b.pose.x;
        label.pose.position.y = b.pose.y;
        label.pose.position.z = 0.25;
        label.pose.orientation.w = 1.0;
        label.scale.z = 0.18;
        label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0; label.color.a = 1.0;
        label.lifetime = ros::Duration(0.2);

        // 2) Se existir associação, desenha linha e mostra distância no label
        auto it = beaconToCluster.find(b.name);
        if (it != beaconToCluster.end()) {
        const int cidx = it->second;
        if (cidx >= 0 && static_cast<std::size_t>(cidx) < clustersCentroids_robotFrame.size()) {
            const auto& c = clustersCentroids_robotFrame[cidx];

            // 2a) Linha beacon -> centróide
            visualization_msgs::Marker line;
            line.header = m.header;
            line.ns = "beacon_assoc";
            line.id = static_cast<int>(i);
            line.type = visualization_msgs::Marker::LINE_LIST;
            line.action = visualization_msgs::Marker::ADD;
            line.scale.x = 0.03; // espessura
            line.color.r = 0.10; line.color.g = 0.90; line.color.b = 0.20; line.color.a = 1.0; // verde
            line.lifetime = ros::Duration(0.2);

            geometry_msgs::Point p1, p2;
            p1.x = b.pose.x; p1.y = b.pose.y; p1.z = 0.0;
            p2.x = c.x;      p2.y = c.y;      p2.z = 0.0;
            line.points.push_back(p1);
            line.points.push_back(p2);
            arr.markers.push_back(line);

            const double d = std::hypot(c.x - b.pose.x, c.y - b.pose.y);
            label.text = b.name + "  (" + std::to_string(d).substr(0,4) + " m)";
        } else {
            label.text = b.name + "  (sem idx)";
        }
        } else {
        // sem match → label em vermelho claro
        label.text = b.name + "  (sem match)";
        label.color.r = 1.0; label.color.g = 0.4; label.color.b = 0.4; label.color.a = 1.0;
        }

        arr.markers.push_back(label);
    }

    // Publica (sem DELETEALL aqui — já o fizeste nos clusters)
    markers_pub_.publish(arr);
}

void BeaconDetector::publishBeaconsInMap() {
  visualization_msgs::MarkerArray arr;

  // limpa anteriores deste tópico (sem afetar os clusters noutro tópico)
  {
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(clear);
  }

  const ros::Time now = ros::Time::now();

  for (std::size_t i = 0; i < beacons_globalFrame.size(); ++i) {
    const auto& b = beacons_globalFrame[i];

    // esfera no frame MAP (fixa no mundo)
    visualization_msgs::Marker m;
    m.header.frame_id = "map";
    m.header.stamp    = now;
    m.ns   = "beacons_map";
    m.id   = static_cast<int>(i);
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.pose.position.x = b.pose.x;
    m.pose.position.y = b.pose.y;
    m.pose.position.z = 0.0;
    m.scale.x = m.scale.y = m.scale.z = 0.0075;           // tamanho da esfera
    m.color.r = 0.10; m.color.g = 0.85; m.color.b = 1.0; // ciano para distinguir dos “dourados”
    m.color.a = 1.0;
    m.lifetime = ros::Duration(0.0); // infinito
    arr.markers.push_back(m);

    // label com o nome
    visualization_msgs::Marker label = m;
    label.ns   = "beacons_map_label";
    label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    label.pose.position.z = 0.25;
    label.scale.z = 0.05;
    label.color.r = 1.0; label.color.g = 1.0; label.color.b = 1.0;
    label.text = b.name;
    arr.markers.push_back(label);
  }

  beacons_map_pub_.publish(arr);
}
