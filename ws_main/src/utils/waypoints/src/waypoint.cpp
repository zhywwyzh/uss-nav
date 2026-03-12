#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
// #define HARD_Z 1.0
using namespace std;
using namespace visualization_msgs;
class PathRecorder {
public:
  PathRecorder() : server_(new interactive_markers::InteractiveMarkerServer("wayPts")) {

    sub_ = nh_.subscribe("/initialpose", 10, &PathRecorder::poseCallback, this);
    ros::NodeHandle nh("~");
    nh.getParam("drone_num", drone_num);
    nh.getParam("hard_z", hard_z);
    ROS_ERROR_ONCE("===========For view point file, Drone number is %d ===============", drone_num);
    ROS_ERROR_ONCE("===========For view point file, hard_z is %f ===============", hard_z);
  }
  ~PathRecorder() {
    std::string filepath = ros::package::getPath("exploration_manager") + "/config/viewpoint.txt";
    // assert(path_.size() >= 2 * drone_num && "path size is less than 2*drone number");  // 不允许重复
    writePathToFile(filepath, drone_num);
  }

  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {

    geometry_msgs::Point position = msg->pose.pose.position;
    position.z = hard_z;
    path_.push_back(position);
    createInteractiveMarker(position, path_.size() - 1);
    ROS_INFO("Recorded position: x=%f, y=%f, z=%f", position.x, position.y, position.z);
  }
  void makeNodeControl(InteractiveMarker& msg, bool movable = true) {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.orientation_mode = InteractiveMarkerControl::VIEW_FACING;
    if (movable) {
      control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    } else {
      control.interaction_mode = InteractiveMarkerControl::NONE;
    }
    control.independent_marker_orientation = true;

    Marker marker;

    marker.type = Marker::CYLINDER;
    marker.scale.x = msg.scale;
    marker.scale.y = msg.scale;
    marker.scale.z = msg.scale;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    control.markers.push_back(marker);
    marker.type = Marker::TEXT_VIEW_FACING;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    marker.text = msg.name;
    marker.pose.position.z += msg.scale;
    control.markers.push_back(marker);

    msg.controls.push_back(control);

    return;
  }

  void createInteractiveMarker(const geometry_msgs::Point& position, int index) {

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "world";
    int_marker.pose.position = position;
    int_marker.scale = 1.0;
    int_marker.name = std::to_string(index);
    int_marker.description = std::to_string(index);

    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    makeNodeControl(int_marker, true);

    // Add the interactive marker to the server
    server_->insert(int_marker, boost::bind(&PathRecorder::processFeedback, this, _1));
    server_->applyChanges();
  }

  // Callback function to handle interactive marker feedback
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr& feedback) {
    int index =
        std::stoi(feedback->marker_name.substr(0));  // Extract the index from the marker name
    path_[index] = feedback->pose.position;
    path_[index].z = hard_z;
    // ROS_INFO("Marker %d moved to: x=%f, y=%f, z=%f", index, feedback->pose.position.x,
    // feedback->pose.position.y, feedback->pose.position.z);
  }

  // Function to print the recorded path coordinates
  void printPath() {
    ROS_WARN("Path coordinates:");
    for (size_t i = 0; i < path_.size(); ++i) {
      ROS_INFO("Point %lu: x=%f, y=%f, z=%f", i, path_[i].x, path_[i].y, path_[i].z);
    }
    // print each segment length
    if (path_.size() > 1) {
      for (size_t i = 0; i < path_.size() - 1; i++) {
        double segment_length = calculateDistance(path_[i], path_[i + 1]);
        ROS_INFO("Segment %d length: %f", i, segment_length);
      }
    }
  }
  // Function to calculate the total length of the path
  double calculateTotalPathLength() {
    double total_length = 0.0;
    for (size_t i = 1; i < path_.size(); ++i) {
      total_length += calculateDistance(path_[i - 1], path_[i]);
    }
    return total_length;
  }

  // Function to calculate the distance between two points
  double calculateDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return std::sqrt(
        std::pow(p2.x - p1.x, 2) + std::pow(p2.y - p1.y, 2) + std::pow(p2.z - p1.z, 2));
  }

  // Function to write the path coordinates to a file
  void writePathToFile(const std::string& filename, int Num) {
    if(!((path_.size() %  Num)==0&& path_.size() >= 2 * Num ))
    {
        std::cout << "\033[1;31m xxxx Error!path size is less than 2*drone number and should divide by drone number xxxx \033[1;0m" << endl;
        std::cout << "\033[1;31m xxxx Error!path size is less than 2*drone number and should divide by drone number xxxx \033[1;0m" << endl;
    }
    // assert((path_.size() %  Num)==0&& path_.size() >= 2 * Num && "path size is less than 2*drone number and should divide by drone number");  // 不允许重复
    std::ofstream outfile(filename);
    int piece_num = path_.size() / Num;
    if (outfile.is_open()) {

      for (size_t i = 0; i < path_.size(); ++i) {
        outfile << i/piece_num << "," << path_[i].x << "," << path_[i].y << "," << path_[i].z << ","
                << 0 << "," << 0 << "\n";
      }

      outfile.close();
     
    } else {
      ROS_ERROR("Failed to open file %s for writing", filename.c_str());
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  std::vector<geometry_msgs::Point> path_;
  int drone_num{ 1 };
  double hard_z{ 1.0 };
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_recorder");
  PathRecorder recorder;

  ros::Rate rate(20);
  int count = 0;
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    if (count == 40) {
      //   ROS_INFO("Recording position");
      recorder.printPath();
      count = 0;
    }
    count++;
  }

  return 0;
}
