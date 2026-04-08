//
// Created by gwq on 25-2-27.
//
# ifndef SKELETON_GENERATION_DATA_STRUCTURE_HPP
# define SKELETON_GENERATION_DATA_STRUCTURE_HPP

#include <ros/ros.h>
#include <unordered_map>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <time.h>
#include <scene_graph/EncodeMask.h>

#define INFO_MSG(str)        do {std::cout << str << std::endl; } while(false)
#define INFO_MSG_RED(str)    do {std::cout << "\033[31m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_GREEN(str)  do {std::cout << "\033[32m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_YELLOW(str) do {std::cout << "\033[33m" << str << "\033[0m" << std::endl; } while(false)
#define INFO_MSG_BLUE(str)   do {std::cout << "\033[34m" << str << "\033[0m" << std::endl; } while(false)

class SkeletonGenerator;
class ObjectFactory;
struct ObjectNode;
struct ObjectEdge;

class Vertex;
class Facet;
class Polyhedron;
class PolyhedronFtr;
class Edge;
class PolyhedronCluster;

typedef std::shared_ptr<SkeletonGenerator> SkeletonGeneratorPtr;
typedef std::shared_ptr<ObjectNode>        ObjectNodePtr;

typedef std::shared_ptr<Vertex>        VertexPtr;
typedef std::shared_ptr<Facet>         FacetPtr;
typedef std::shared_ptr<Polyhedron>    PolyHedronPtr;
typedef std::shared_ptr<PolyhedronFtr> PolyhedronFtrPtr;
typedef std::shared_ptr<Edge>          EdgePtr;

/**
 * @class ColorGenerator
 * @brief A class to generate vibrant, highly distinguishable colors from integer IDs.
 */
class ColorGenerator {
public:
  /**
   * @brief Generates a color from an integer ID.
   * @param id The integer object ID.
   * @return Eigen::Vector3d An (r, g, b) vector with components in the range [0, 1].
   */
  static Eigen::Vector3d getColorById(int id) {
    // Use the golden ratio to distribute hues for maximum visual separation
    const double golden_ratio_conjugate = 0.61803398875;
    double hue = fmod(id * golden_ratio_conjugate, 1.0);

    // Set high saturation and value for vibrancy
    HSV hsv;
    hsv.h = hue * 360.0; // Hue in degrees [0, 360]
    hsv.s = 0.9;         // Saturation in [0, 1]
    hsv.v = 0.95;        // Value in [0, 1]

    return hsvToRgb(hsv);
  }

private:
  // Helper structure for HSV color
  struct HSV {
    double h, s, v; // h:[0, 360], s:[0, 1], v:[0, 1]
  };

  // Internal helper function to convert HSV to RGB
  static Eigen::Vector3d hsvToRgb(HSV hsv) {
    Eigen::Vector3d rgb;
    double c = hsv.v * hsv.s;
    double h_prime = fmod(hsv.h / 60.0, 6);
    double x = c * (1 - fabs(fmod(h_prime, 2) - 1));
    double m = hsv.v - c;

    if (0 <= h_prime && h_prime < 1) {
      rgb << c, x, 0;
    } else if (1 <= h_prime && h_prime < 2) {
      rgb << x, c, 0;
    } else if (2 <= h_prime && h_prime < 3) {
      rgb << 0, c, x;
    } else if (3 <= h_prime && h_prime < 4) {
      rgb << 0, x, c;
    } else if (4 <= h_prime && h_prime < 5) {
      rgb << x, 0, c;
    } else if (5 <= h_prime && h_prime < 6) {
      rgb << c, 0, x;
    } else {
      rgb << 0, 0, 0;
    }

    rgb.array() += m;
    return rgb;
  }
};


struct ObjectEdge {
  enum EdgeType {
    UNKNOWN = 0,
    WITH_SKELETON = 1,
    WITH_OBJECT = 2
  } father_type{UNKNOWN};
  std::string edge_description;
  PolyHedronPtr polyhedron_father{nullptr};
  ObjectNodePtr object_father{nullptr};
  std::vector<ObjectNodePtr> object_child;
};

struct ObjectNode {
  typedef std::shared_ptr<ObjectNode> Ptr;
  int id;
  std::string label;
  double conf;
  Eigen::VectorXd label_feature;
  Eigen::Vector3d pos;
  Eigen::Vector3d color;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obb_corners, obb_axis;

  // skeleton edge
  ObjectEdge edge;

  // filters (上一次检测的时间)
  bool is_alive{true};
  ros::Time last_detection_time;
  unsigned int detection_count{0};

  ObjectNode(){
    pos   = Eigen::Vector3d::Zero();
    label = "None";
    id    = -1;
    conf  = 0.0f;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    obb_corners.reset(new pcl::PointCloud<pcl::PointXYZ>);
    obb_axis.reset(new pcl::PointCloud<pcl::PointXYZ>);
    label_feature = Eigen::VectorXd::Zero(512);
  }
  bool isWellDetected(int threshold) const {
    return detection_count >= threshold;
  }
};

struct ProcessedCLoudInput {
  cv::Mat         depth_img;
  cv::Mat         rgb_img;
  cv::Mat         mask;
  Eigen::Vector3d pos;
  Eigen::Matrix4d tf;
  std::string     label;
  double          conf;
  Eigen::Vector3d pt_color;
  Eigen::VectorXd label_feature;
  ProcessedCLoudInput (const cv::Mat& depth_in, const cv::Mat& rgb_in, const cv::Mat& mask_in,
                       const Eigen::Matrix4d& tf_in, const std::string& label_in, const Eigen::VectorXd & label_feature_in,
                       const double & conf_in, const Eigen::Vector3d& pt_color_in) {
    rgb_img   = rgb_in.clone();
    depth_img = depth_in.clone();
    mask      = mask_in.clone();
    tf        = tf_in;
    pos       = tf_in.block<3, 1>(0, 3);
    label     = label_in;
    conf      = conf_in;
    pt_color  = pt_color_in;
    if (label_feature_in.size() != 512) {
      INFO_MSG_RED("*** ERROR: label_feature_in.size() != 512, program dumped out ***");
      exit(1);
    }
    label_feature = label_feature_in;
  };
};

class Vertex{
public:
  enum VertexType{
    BLACK   = 0,           // 障碍物采样点
    WHITE   = 1,           // 无障碍物采样丢按
    GRAY    = 2,           // 用于进行上下界检查
    RUBBISH = 3            // 弃用的采样点
  };
  bool                   is_visited_;
  bool                   is_critical_;
  VertexType             type_;
  Eigen::Vector3d        position_, collision_polyhedron_index_;
  Eigen::Vector3d        direction_in_unit_sphere_;
  int                    dir_sample_buffer_index_;
  std::vector<VertexPtr> connected_vertices_;
  double                 distance_to_center_;
  Vertex(Eigen::Vector3d position, Eigen::Vector3d direction_in_unit_sphere, VertexType type){
    position_ = position;
    direction_in_unit_sphere_ = direction_in_unit_sphere;
    type_ = type;
    is_visited_ = false;
    is_critical_ = false;
  }
  Vertex() {}
  ~Vertex() {}
};

class Facet{
public:
  int index_;
  Eigen::Vector3d         out_unit_normal_;    // 外法线
  Eigen::Vector3d         center_;             // 面中心 （顶点的平均坐标）
  Eigen::Vector4d         plane_equation_;     // ax + by + cz + d = 0 （平面方程）
  std::vector<VertexPtr>  vertices_;           // 面的顶点
  std::vector<FacetPtr>   neighbor_facets_;    // 相邻面的指针
  PolyHedronPtr           master_polyhedron_;  // 所属的多面体
  bool                    frontier_processed_;
  bool                    is_linked_;
  bool                    is_visited_;         // used by split frontier
  Facet(const std::vector<VertexPtr> &vertices, const PolyHedronPtr &master_polyhedron){
    vertices_ = vertices;
    master_polyhedron_ = master_polyhedron;
    center_ = (vertices_.at(0)->position_ + vertices_.at(1)->position_ + vertices_.at(2)->position_) / 3.0;
    // Compute plane equation: ax + by + cz + d = 0
    const Eigen::Vector3d v1 = vertices.at(1)->position_ - vertices.at(0)->position_;
    const Eigen::Vector3d v2 = vertices.at(2)->position_ - vertices.at(0)->position_;
    Eigen::Vector3d normal = v1.cross(v2);
    normal.normalize();
    const double a = normal(0); const double b = normal(1); const double c = normal(2);
    Eigen::Vector3d abc(a, b, c);
    const double d = -normal.dot(vertices.at(0)->position_);
    plane_equation_ = Eigen::Vector4d(a, b, c, d);
    frontier_processed_ = false;
    is_linked_   = false;
    is_visited_  = false;
  }
  Facet() {}
  ~Facet() {}
};

class Polyhedron{
private:
  struct Vector3dHash {
    std::size_t operator()(const Eigen::Vector3d& vector) const {
      std::size_t h1 = std::hash<double>()(vector.x());
      std::size_t h2 = std::hash<double>()(vector.y());
      std::size_t h3 = std::hash<double>()(vector.z());
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };
public:
  typedef std::shared_ptr<Polyhedron> Ptr;
  Eigen::Vector3d               center_, origin_center_;      // origin center 可以当做多面体的index进行哈希查询
  bool                          is_gate_, is_rollbacked_;
  bool                          can_reach_;
  double                        radius_;
  std::vector<VertexPtr>        black_vertices_, white_vertices_, gray_vertices_;
  std::vector<FacetPtr>         facets_;
  std::vector<PolyhedronFtrPtr> ftrs_;
  std::vector<PolyHedronPtr>    connected_nodes_;  // nodes 包括 gates; polyhedron 不包括 gates
  std::vector<Edge>             edges_;
  PolyhedronFtrPtr              parent_ftr_;
  std::unordered_map<Eigen::Vector3d, bool, Vector3dHash> candidate_rollback_;

  Eigen::Vector3d box_min_, box_max_;

  // objects tree structure
  std::map<int, ObjectNodePtr> objs_;

  // area related
  int area_id_{-1};

  // temp data
  double temp_distance_to_nxt_poly_;  // 到下一个多面体中心的Astar距离

  Polyhedron(const Eigen::Vector3d center, const PolyhedronFtrPtr &parent_ftr, const bool is_gate = false){
    center_         = center;
    origin_center_  = center;
    parent_ftr_     = parent_ftr;
    is_gate_        = is_gate;
    can_reach_      = false;
    radius_         = 0.0;
    black_vertices_.clear();
    white_vertices_.clear();
    connected_nodes_.clear();
    edges_.clear();
    candidate_rollback_.clear();
    facets_.clear();
    ftrs_.clear();
    temp_distance_to_nxt_poly_ = 0.0;
    box_max_ = Eigen::Vector3d(-99999.0, -99999.0, -99999.0);
    box_min_ = Eigen::Vector3d(99999.0, 99999.0, 99999.0);
  }
  Polyhedron() {}
  ~Polyhedron() {}
};

class PolyhedronFtr{
public:
  int index;
  // Average of the facets center
  Eigen::Vector3d avg_center_;
  // Average of the facets outwards_unit_normal;
  Eigen::Vector3d out_unit_normal_;
  // The projection of avg_coord along outwards_normal,
  // which is on one of the facets owned by the frontier
  Eigen::Vector3d proj_center_;
  double cos_theta_; // frontier平均向外法向量与投影面法向量之间的cos值
  double area_size_; // 面积大小
  // Position of the node generate by the frontier
  Eigen::Vector3d next_node_pos_;
  // valid means this frontier is clear at the time of processing
  bool valid_;   // 只有frontier有效（true）时，才会向外扩展新node
  bool deleted_;
  PolyHedronPtr master_polyhedron;    // 由哪个node生成的
  PolyHedronPtr gate_;                // GATE！
  FacetPtr proj_facet_;
  std::vector<FacetPtr>  facets_;
  std::vector<VertexPtr> vertices_;

  // temp data

  PolyhedronFtr(std::vector<FacetPtr> facets, PolyHedronPtr master){
    auto addSingleFacetAreaSize = [] (FacetPtr facet) -> double{
        Eigen::Vector3d v1 = facet->vertices_.at(1)->position_ - facet->vertices_.at(0)->position_;
        Eigen::Vector3d v2 = facet->vertices_.at(2)->position_ - facet->vertices_.at(0)->position_;
        return 0.5 * v1.cross(v2).norm();
    };
    facets_                    = facets;
    master_polyhedron          = master;
    int num_facet              = facets_.size();
    area_size_                 = 0.0;
    Eigen::Vector3d coord_sum  = Eigen::Vector3d::Zero();
    Eigen::Vector3d normal_sum = Eigen::Vector3d::Zero();

    for (int i = 0; i < num_facet; i++) {
      coord_sum  += facets_.at(i)->center_;
      normal_sum += facets_.at(i)->out_unit_normal_;
      area_size_ += addSingleFacetAreaSize(facets_.at(i));
    }
    avg_center_           = coord_sum / num_facet;
    out_unit_normal_ = normal_sum / num_facet;

    // Set default value
    valid_   = false;
    deleted_ = false;
    gate_    = nullptr;
  }
  PolyhedronFtr(){}
  ~PolyhedronFtr() = default;
};

class Edge{
public:
  PolyHedronPtr poly_nxt_;
  double length_;
  double weight_;
  bool is_force_connected_{false};
  std::vector<Eigen::Vector3d> path_;
  Edge(PolyHedronPtr poly_nxt, double length): poly_nxt_(poly_nxt), length_(length) { path_.clear(); }
  Edge(PolyHedronPtr poly_nxt, std::vector<Eigen::Vector3d> path, bool do_path_reverse): poly_nxt_(poly_nxt){
    if (do_path_reverse){
      path_ = path;
      std::reverse(path_.begin(), path_.end());
    }
    length_ = 0.0;
    for (int i = 1; i < path_.size(); i++)
      length_ += (path_.at(i) - path_.at(i-1)).norm();
  }
  void forceConnect(){is_force_connected_ = true;};
  ~Edge() {}
};

class PolyhedronCluster {
public:
  typedef std::shared_ptr<PolyhedronCluster> Ptr;
  PolyhedronCluster() {
    polys_.clear();
    objects_.clear();
    room_label_ = room_description_ = "";
    id_      = 0;
    color_   = Eigen::Vector3d(1.0, 0.0, 0.0);
    box_min_ = Eigen::Vector3d(99999.0, 99999.0, 99999.0);
    box_max_ = Eigen::Vector3d(-99999.0, -99999.0, -99999.0);
    center_  = Eigen::Vector3d::Zero();
    num_ftrs_= 0;
  }
  std::vector<PolyHedronPtr>   polys_;
  std::vector<ObjectNodePtr> objects_;
  std::string     room_label_, room_description_;
  Eigen::Vector3d box_min_, box_max_;
  Eigen::Vector3d center_;
  Eigen::Vector3d color_;
  unsigned int    id_;
  int             num_ftrs_;
  int             last_obj_num_{0};
  std::map<int, bool> nbr_area_;

  void addPoly(PolyHedronPtr& poly, bool change_poly_mount){
    polys_ .push_back(poly);
    box_max_ = Eigen::Vector3d(std::max(box_max_.x(), poly->box_max_.x()), std::max(box_max_.y(), poly->box_max_.y()), std::max(box_max_.z(), poly->box_max_.z()));
    box_min_ = Eigen::Vector3d(std::min(box_min_.x(), poly->box_min_.x()), std::min(box_min_.y(), poly->box_min_.y()), std::min(box_min_.z(), poly->box_min_.z()));
    center_  = center_ + (poly->center_ - center_) / polys_.size();
    if (change_poly_mount) poly->area_id_ = id_;
  };
  void addObject(ObjectNodePtr obj) {
    if (obj->isWellDetected(5)) {
      objects_.push_back(obj);
    }
  }
  void clearObjs() {
    last_obj_num_ = objects_.size();
    objects_.clear();
  }

  void resetClusterWithPolys(std::vector<PolyHedronPtr>& polys, bool reset_semantics=true) {
    box_min_ = Eigen::Vector3d(99999.0, 99999.0, 99999.0);
    box_max_ = Eigen::Vector3d(-99999.0, -99999.0, -99999.0);
    center_  = Eigen::Vector3d::Zero();
    num_ftrs_     = 0;
    if (reset_semantics){
      room_label_   = room_description_ = "Unknown";
    }
    last_obj_num_ = objects_.size();
    nbr_area_.clear();
    objects_.clear();
    polys_.clear();
    for (auto& poly : polys) addPoly(poly, true);
  }
};

#endif //SKELETON_GENERATION_DATA_STRUCTURE_HPP