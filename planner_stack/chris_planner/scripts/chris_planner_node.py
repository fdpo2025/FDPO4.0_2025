#!/usr/bin/env python3

import rospy
import sys
import os
from std_msgs.msg import String, Int32MultiArray
import yaml

# Add the modules directory to the path
import rospkg
rospack = rospkg.RosPack()
chris_planner_path = rospack.get_path('chris_planner')

# Add modules to Python path
# In development: modules are in source tree
# After installation: modules are in lib/python3/dist-packages/chris_planner/
modules_path = os.path.join(chris_planner_path, 'modules')
if os.path.exists(modules_path):
    # Development mode: add parent directory so we can import modules
    sys.path.insert(0, os.path.dirname(modules_path))
else:
    # Installed mode: modules should be in the Python path already
    # But we can also try to add the package path
    pass

# Import chris_planner modules
try:
    import modules.planner as planner_module
    import modules.factory as factory_module
    import modules.yaml_utils as yaml_utils
except ImportError:
    # Fallback: try importing from chris_planner package
    import chris_planner.modules.planner as planner_module
    import chris_planner.modules.factory as factory_module
    import chris_planner.modules.yaml_utils as yaml_utils

class ChrisPlannerNode:
    def __init__(self):
        rospy.init_node('chris_planner_node', anonymous=False)
        
        # Get package path (works both in development and after installation)
        self.package_path = rospack.get_path('chris_planner')
        
        # Load configuration files
        # Files are installed to share/chris_planner/files/ after catkin_make install
        graph_file = os.path.join(self.package_path, 'files', 'inputs', 'graph.yaml')
        factory_components_file = os.path.join(self.package_path, 'files', 'inputs', 'factory_components.yaml')
        
        if not os.path.exists(graph_file):
            rospy.logerr(f"Graph file not found at: {graph_file}")
            raise FileNotFoundError(f"Graph file not found: {graph_file}")
        if not os.path.exists(factory_components_file):
            rospy.logerr(f"Factory components file not found at: {factory_components_file}")
            raise FileNotFoundError(f"Factory components file not found: {factory_components_file}")
        
        rospy.loginfo(f"Loading graph from: {graph_file}")
        rospy.loginfo(f"Loading factory components from: {factory_components_file}")
        
        # Load YAML files
        graph_dict = yaml_utils.load_file(graph_file)
        factory_components_dict = yaml_utils.load_file(factory_components_file)
        
        # Get planning method from parameter (default: "closest")
        planning_method = rospy.get_param('~planning_method', 'astar')
        rospy.loginfo(f"Using planning method: {planning_method}")
        
        # Initialize planner
        self.planner = planner_module.Planner(graph_dict, factory_components_dict, method=planning_method)
        rospy.loginfo("Chris planner initialized successfully")
        
        # ROS subscribers and publishers
        self.color_seq_sub = rospy.Subscriber('/color_sequence', String, self.color_sequence_callback, queue_size=1)
        self.planned_paths_pub = rospy.Publisher('/planned_paths', Int32MultiArray, queue_size=100, latch=True)
        
        # State
        self.running = False
        
        rospy.loginfo("Chris planner node ready, waiting for /color_sequence messages...")
    
    def color_sequence_to_boxtypes(self, color_seq):
        """
        Convert color sequence string (e.g., "RGBB") to boxtypes list.
        R -> 0 (TYPE_A), G -> 1 (TYPE_B), B -> 2 (TYPE_C)
        """
        color_map = {'R': 0, 'G': 1, 'B': 2}
        boxtypes = []
        for color in color_seq:
            if color.upper() in color_map:
                boxtypes.append(color_map[color.upper()])
            else:
                rospy.logwarn(f"Unknown color '{color}', skipping")
        return boxtypes
    
    def color_sequence_callback(self, msg):
        if self.running:
            rospy.logwarn("Planner is already running, ignoring new request")
            return
        
        self.running = True
        color_seq = msg.data
        rospy.loginfo(f"Received color sequence: {color_seq}")
        
        try:
            # Convert color sequence to boxtypes
            boxtypes = self.color_sequence_to_boxtypes(color_seq)
            rospy.loginfo(f"Converted to boxtypes: {boxtypes}")
            
            if len(boxtypes) == 0:
                rospy.logwarn("No valid boxtypes found in color sequence")
                self.running = False
                return
            
            # Plan path using chris_planner
            high_level_path, low_level_paths, low_level_paths_compact, total_cost = self.planner.plan_initial(boxtypes)
            
            rospy.loginfo(f"Planning completed. High-level path: {high_level_path}")
            rospy.loginfo(f"Total cost: {total_cost}")
            
            # Convert low_level_paths_compact to a single flat list
            # The planner returns a list of paths, we need to merge them
            final_path = self.planner.convert_paths2path(low_level_paths_compact)
            
            rospy.loginfo(f"Final path (length={len(final_path)}): {final_path}")
            
            # Publish the path
            path_msg = Int32MultiArray()
            path_msg.data = final_path
            self.planned_paths_pub.publish(path_msg)
            rospy.loginfo(f"Published path with {len(final_path)} nodes to /planned_paths")
            
        except Exception as e:
            rospy.logerr(f"Error during planning: {str(e)}")
            import traceback
            rospy.logerr(traceback.format_exc())
        finally:
            self.running = False
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ChrisPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

