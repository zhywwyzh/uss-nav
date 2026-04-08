#include "../include/scene_graph/scene_graph_map_io.h"

#include "../include/scene_graph/scene_graph.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <ctime>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <pcl/io/pcd_io.h>

namespace {

using nlohmann::json;

double sanitizeFiniteNumber(const double value, const double fallback = 0.0) {
    return std::isfinite(value) ? value : fallback;
}

double jsonNumberOrDefault(const json& value, const double fallback = 0.0) {
    return value.is_number() ? value.get<double>() : fallback;
}

// JSON 与 Eigen/PCL 数据之间的轻量转换工具，供保存/加载流程复用。
json vec3ToJson(const Eigen::Vector3d& vec) {
    return json::array({
        sanitizeFiniteNumber(vec.x()),
        sanitizeFiniteNumber(vec.y()),
        sanitizeFiniteNumber(vec.z())
    });
}

json vec4ToJson(const Eigen::Vector4d& vec) {
    return json::array({
        sanitizeFiniteNumber(vec.x()),
        sanitizeFiniteNumber(vec.y()),
        sanitizeFiniteNumber(vec.z()),
        sanitizeFiniteNumber(vec.w())
    });
}

json vectorXdToJson(const Eigen::VectorXd& vec) {
    json data = json::array();
    for (int i = 0; i < vec.size(); ++i) data.push_back(sanitizeFiniteNumber(vec(i)));
    return data;
}

Eigen::Vector3d jsonToVec3(const json& data, const Eigen::Vector3d& default_value = Eigen::Vector3d::Zero()) {
    if (!data.is_array() || data.size() != 3) return default_value;
    return Eigen::Vector3d(
        jsonNumberOrDefault(data.at(0), default_value.x()),
        jsonNumberOrDefault(data.at(1), default_value.y()),
        jsonNumberOrDefault(data.at(2), default_value.z()));
}

Eigen::Vector4d jsonToVec4(const json& data, const Eigen::Vector4d& default_value = Eigen::Vector4d::Zero()) {
    if (!data.is_array() || data.size() != 4) return default_value;
    return Eigen::Vector4d(
        jsonNumberOrDefault(data.at(0), default_value.x()),
        jsonNumberOrDefault(data.at(1), default_value.y()),
        jsonNumberOrDefault(data.at(2), default_value.z()),
        jsonNumberOrDefault(data.at(3), default_value.w()));
}

Eigen::VectorXd jsonToVectorXd(const json& data, int default_size = 512) {
    if (!data.is_array()) return Eigen::VectorXd::Zero(default_size);
    Eigen::VectorXd vec(data.size());
    for (int i = 0; i < data.size(); ++i) vec(i) = jsonNumberOrDefault(data.at(i), 0.0);
    return vec;
}

std::string nowAsCompactString() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_now;
#if defined(_WIN32)
    localtime_s(&tm_now, &now);
#else
    localtime_r(&now, &tm_now);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y%m%d_%H%M%S");
    return oss.str();
}

std::string nowAsReadableString() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_now;
#if defined(_WIN32)
    localtime_s(&tm_now, &now);
#else
    localtime_r(&now, &tm_now);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

std::string sanitizePathSegment(const std::string& raw_segment) {
    std::string sanitized;
    sanitized.reserve(raw_segment.size());
    for (const char ch : raw_segment) {
        if ((ch >= 'a' && ch <= 'z') ||
            (ch >= 'A' && ch <= 'Z') ||
            (ch >= '0' && ch <= '9') ||
            ch == '_' || ch == '-') {
            sanitized.push_back(ch);
        } else {
            sanitized.push_back('_');
        }
    }

    while (!sanitized.empty() && sanitized.front() == '_') sanitized.erase(sanitized.begin());
    while (!sanitized.empty() && sanitized.back() == '_') sanitized.pop_back();
    return sanitized;
}

bool normalizeRelativeSavePath(const std::string& raw_path,
                               std::string& normalized_path,
                               const bool allow_empty_default = false) {
    normalized_path.clear();
    if (raw_path.empty()) {
        if (allow_empty_default) {
            normalized_path = "snapshot_" + nowAsCompactString();
            return true;
        }
        return false;
    }

    if (raw_path.front() == '/') return false;

    std::stringstream ss(raw_path);
    std::string token;
    while (std::getline(ss, token, '/')) {
        if (token.empty() || token == ".") continue;
        if (token == "..") return false;

        std::string sanitized = sanitizePathSegment(token);
        if (sanitized.empty()) sanitized = "segment";
        if (!normalized_path.empty()) normalized_path += "/";
        normalized_path += sanitized;
    }

    if (normalized_path.empty() && allow_empty_default) {
        normalized_path = "snapshot_" + nowAsCompactString();
        return true;
    }
    return !normalized_path.empty();
}

bool directoryExists(const std::string& path) {
    struct stat info {};
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool createDirectoryRecursive(const std::string& path) {
    if (path.empty()) return false;
    if (directoryExists(path)) return true;

    std::string normalized = path;
    while (!normalized.empty() && normalized.back() == '/') normalized.pop_back();
    if (normalized.empty()) return false;

    std::string current;
    if (normalized.front() == '/') current = "/";

    std::stringstream ss(normalized);
    std::string token;
    while (std::getline(ss, token, '/')) {
        if (token.empty()) continue;
        if (!current.empty() && current.back() != '/') current += "/";
        current += token;
        if (directoryExists(current)) continue;
        if (mkdir(current.c_str(), 0755) != 0 && errno != EEXIST) return false;
    }
    return directoryExists(normalized);
}

bool writeJsonFile(const std::string& path, const json& data) {
    std::ofstream out(path);
    if (!out.is_open()) return false;
    out << data.dump(2) << std::endl;
    return true;
}

bool readJsonFile(const std::string& path, json& data) {
    std::ifstream in(path);
    if (!in.is_open()) return false;
    try {
        in >> data;
    } catch (const std::exception&) {
        return false;
    }
    return true;
}

template <typename T>
int touchPtrAndGetId(const std::shared_ptr<T>& ptr,
                     std::unordered_map<const T*, int>& id_map,
                     std::vector<std::shared_ptr<T>>& ordered_ptrs) {
    if (ptr == nullptr) return -1;
    const T* raw_ptr = ptr.get();
    auto it = id_map.find(raw_ptr);
    if (it != id_map.end()) return it->second;

    const int new_id = static_cast<int>(ordered_ptrs.size());
    id_map[raw_ptr] = new_id;
    ordered_ptrs.push_back(ptr);
    return new_id;
}

using PolyOriginKey = std::array<double, 3>;

struct PolyOriginKeyHash {
    std::size_t operator()(const PolyOriginKey& key) const {
        std::size_t h1 = std::hash<double>()(key[0]);
        std::size_t h2 = std::hash<double>()(key[1]);
        std::size_t h3 = std::hash<double>()(key[2]);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

struct PolySnapshotSelection {
    std::vector<PolyHedronPtr> polys;
    std::unordered_map<const Polyhedron*, PolyHedronPtr> canonical_poly_map;
    int skipped_orphan_gate_count{0};
    int merged_duplicate_gate_count{0};
};

PolyOriginKey makePolyOriginKey(const PolyHedronPtr& poly) {
    return {poly->origin_center_.x(), poly->origin_center_.y(), poly->origin_center_.z()};
}

bool shouldSerializePolyhedron(const PolyHedronPtr& poly) {
    if (poly == nullptr) return false;
    if (!poly->is_gate_) return true;

    // 未接入拓扑图的 gate 多为 frontier 处理失败时留下的临时对象，不应进入快照。
    return !poly->connected_nodes_.empty() || !poly->edges_.empty();
}

int polySerializationScore(const PolyHedronPtr& poly) {
    if (poly == nullptr) return -1;

    int score = 0;
    score += static_cast<int>(poly->connected_nodes_.size()) * 1000;
    score += static_cast<int>(poly->edges_.size()) * 1000;
    score += static_cast<int>(poly->facets_.size()) * 100;
    score += static_cast<int>(poly->ftrs_.size()) * 100;
    score += static_cast<int>(poly->objs_.size()) * 10;
    if (poly->parent_ftr_ != nullptr) score += 5;
    if (!poly->is_rollbacked_) score += 1;
    return score;
}

PolyHedronPtr resolveCanonicalPoly(const PolyHedronPtr& poly,
                                   const std::unordered_map<const Polyhedron*, PolyHedronPtr>& canonical_poly_map) {
    if (poly == nullptr) return nullptr;
    auto it = canonical_poly_map.find(poly.get());
    return it != canonical_poly_map.end() ? it->second : nullptr;
}

int getCanonicalPolyId(const PolyHedronPtr& poly,
                       const std::unordered_map<const Polyhedron*, PolyHedronPtr>& canonical_poly_map,
                       const std::unordered_map<const Polyhedron*, int>& poly_id_map) {
    PolyHedronPtr canonical_poly = resolveCanonicalPoly(poly, canonical_poly_map);
    if (canonical_poly == nullptr) return -1;
    auto it = poly_id_map.find(canonical_poly.get());
    return it != poly_id_map.end() ? it->second : -1;
}

// 从骨架生成器可达的入口出发，筛出可持久化的 polyhedron，并对重复 gate 做归一化。
PolySnapshotSelection collectAllReachablePolys(const SceneGraph& scene_graph) {
    std::vector<PolyHedronPtr> seed_polys;
    scene_graph.skeleton_gen_->getAllPolys(seed_polys);

    PolySnapshotSelection selection;
    std::vector<PolyHedronPtr> all_polys;
    std::unordered_set<const Polyhedron*> visited;
    std::vector<PolyHedronPtr> queue = seed_polys;

    while (!queue.empty()) {
        PolyHedronPtr cur_poly = queue.back();
        queue.pop_back();
        if (cur_poly == nullptr) continue;
        if (!visited.insert(cur_poly.get()).second) continue;
        if (!shouldSerializePolyhedron(cur_poly)) {
            if (cur_poly->is_gate_) ++selection.skipped_orphan_gate_count;
            continue;
        }

        all_polys.push_back(cur_poly);
        for (const auto& nbr_poly : cur_poly->connected_nodes_) {
            if (nbr_poly != nullptr) queue.push_back(nbr_poly);
        }
        for (const auto& edge : cur_poly->edges_) {
            if (edge.poly_nxt_ != nullptr) queue.push_back(edge.poly_nxt_);
        }
        for (const auto& ftr : cur_poly->ftrs_) {
            if (ftr != nullptr && ftr->gate_ != nullptr) queue.push_back(ftr->gate_);
        }
    }

    auto poly_less = [](const PolyHedronPtr& lhs, const PolyHedronPtr& rhs) {
        if (lhs == nullptr || rhs == nullptr) return lhs.get() < rhs.get();
        if (lhs->origin_center_.x() != rhs->origin_center_.x()) return lhs->origin_center_.x() < rhs->origin_center_.x();
        if (lhs->origin_center_.y() != rhs->origin_center_.y()) return lhs->origin_center_.y() < rhs->origin_center_.y();
        if (lhs->origin_center_.z() != rhs->origin_center_.z()) return lhs->origin_center_.z() < rhs->origin_center_.z();
        if (lhs->center_.x() != rhs->center_.x()) return lhs->center_.x() < rhs->center_.x();
        if (lhs->center_.y() != rhs->center_.y()) return lhs->center_.y() < rhs->center_.y();
        if (lhs->center_.z() != rhs->center_.z()) return lhs->center_.z() < rhs->center_.z();
        return lhs.get() < rhs.get();
    };
    std::sort(all_polys.begin(), all_polys.end(), poly_less);

    std::unordered_map<PolyOriginKey, std::vector<PolyHedronPtr>, PolyOriginKeyHash> gate_groups;
    for (const auto& poly : all_polys) {
        if (poly == nullptr) continue;
        if (!poly->is_gate_) {
            selection.polys.push_back(poly);
            selection.canonical_poly_map[poly.get()] = poly;
            continue;
        }
        gate_groups[makePolyOriginKey(poly)].push_back(poly);
    }

    for (auto& gate_group : gate_groups) {
        auto& group = gate_group.second;
        auto best_it = std::max_element(group.begin(), group.end(), [](const PolyHedronPtr& lhs, const PolyHedronPtr& rhs) {
            const int lhs_score = polySerializationScore(lhs);
            const int rhs_score = polySerializationScore(rhs);
            if (lhs_score != rhs_score) return lhs_score < rhs_score;
            return lhs.get() > rhs.get();
        });
        PolyHedronPtr canonical_gate = *best_it;
        selection.polys.push_back(canonical_gate);
        for (const auto& gate : group) selection.canonical_poly_map[gate.get()] = canonical_gate;
        selection.merged_duplicate_gate_count += static_cast<int>(group.size()) - 1;
    }

    std::sort(selection.polys.begin(), selection.polys.end(), poly_less);
    return selection;
}

template <typename PointT>
bool loadPointCloudFile(const std::string& path, const typename pcl::PointCloud<PointT>::Ptr& cloud) {
    return pcl::io::loadPCDFile(path, *cloud) == 0;
}

std::vector<int> jsonToIntVector(const json& data) {
    std::vector<int> result;
    if (!data.is_array()) return result;
    result.reserve(data.size());
    for (const auto& item : data) result.push_back(item.get<int>());
    return result;
}

std::string joinPath(const std::string& lhs, const std::string& rhs) {
    if (lhs.empty()) return rhs;
    if (rhs.empty()) return lhs;
    if (lhs.back() == '/') return lhs + rhs;
    return lhs + "/" + rhs;
}

}  // namespace

SceneGraphMapIO::SceneGraphMapIO(SceneGraph& scene_graph)
    : scene_graph_(scene_graph) {}

bool SceneGraph::saveMap(const std::string& save_name) {
    SceneGraphMapIO map_io(*this);
    return map_io.save(save_name);
}

bool SceneGraph::loadMap(const std::string& save_name) {
    SceneGraphMapIO map_io(*this);
    return map_io.load(save_name);
}

bool SceneGraphMapIO::save(const std::string& save_name) {
    INFO_MSG_BLUE("\n[SceneGraphMapIO] ================= Save Scene Graph =================");

    // 1) 规范化相对保存路径并准备快照目录。
    std::string final_save_name;
    if (!normalizeRelativeSavePath(save_name, final_save_name, true)) {
        INFO_MSG_RED("[SceneGraphMapIO] Save aborted: invalid relative save path.");
        return false;
    }
    const std::string saved_data_dir = scene_graph_.this_package_path_ + "/saved_data";
    const std::string save_dir = joinPath(saved_data_dir, final_save_name);
    const std::string object_dir = joinPath(save_dir, "objects");

    if (!createDirectoryRecursive(saved_data_dir) ||
        !createDirectoryRecursive(save_dir) ||
        !createDirectoryRecursive(object_dir)) {
        INFO_MSG_RED("[SceneGraphMapIO] Save aborted: failed to prepare output directory.");
        return false;
    }

    // 2) 锁住骨架和目标对象模块，保证导出期间读到的是一致状态。
    std::unique_lock<SkeletonGenerator> skeleton_lock(*scene_graph_.skeleton_gen_);
    std::unique_lock<ObjectFactory> object_lock(*scene_graph_.object_factory_);

    INFO_MSG_YELLOW("[SceneGraphMapIO] Snapshot path : " << save_dir);
    INFO_MSG_YELLOW("[SceneGraphMapIO] Collecting runtime snapshot ...");

    // 先为跨对象引用建立稳定 ID，避免 JSON 中直接保存裸指针关系。
    const PolySnapshotSelection poly_selection = collectAllReachablePolys(scene_graph_);
    const std::vector<PolyHedronPtr>& polys = poly_selection.polys;
    auto& object_map = scene_graph_.object_factory_->object_map_;
    auto& area_map = scene_graph_.skeleton_gen_->area_handler_->area_map_;

    std::unordered_map<const Polyhedron*, int> poly_id_map;
    for (std::size_t i = 0; i < polys.size(); ++i) poly_id_map[polys[i].get()] = static_cast<int>(i);

    std::unordered_map<const Vertex*, int> vertex_id_map;
    std::vector<VertexPtr> vertices;
    std::unordered_map<const Facet*, int> facet_id_map;
    std::vector<FacetPtr> facets;
    std::unordered_map<const PolyhedronFtr*, int> frontier_id_map;
    std::vector<PolyhedronFtrPtr> frontiers;

    auto touch_vertex = [&](const VertexPtr& vertex) { return touchPtrAndGetId(vertex, vertex_id_map, vertices); };
    auto touch_facet = [&](const FacetPtr& facet) { return touchPtrAndGetId(facet, facet_id_map, facets); };
    auto touch_frontier = [&](const PolyhedronFtrPtr& frontier) { return touchPtrAndGetId(frontier, frontier_id_map, frontiers); };

    // 3) 从 polyhedron 出发展开依赖，把顶点、面片、frontier 等关联元素收集齐。
    for (const auto& poly : polys) {
        for (const auto& vertex : poly->black_vertices_) touch_vertex(vertex);
        for (const auto& vertex : poly->white_vertices_) touch_vertex(vertex);
        for (const auto& vertex : poly->gray_vertices_) touch_vertex(vertex);
        for (const auto& facet : poly->facets_) {
            touch_facet(facet);
            for (const auto& vertex : facet->vertices_) touch_vertex(vertex);
            for (const auto& neighbor : facet->neighbor_facets_) touch_facet(neighbor);
        }
        for (const auto& frontier : poly->ftrs_) {
            touch_frontier(frontier);
            for (const auto& facet : frontier->facets_) {
                touch_facet(facet);
                for (const auto& vertex : facet->vertices_) touch_vertex(vertex);
            }
            for (const auto& vertex : frontier->vertices_) touch_vertex(vertex);
            if (frontier->proj_facet_ != nullptr) touch_facet(frontier->proj_facet_);
        }
    }

    // 按模块拆分写入场景图状态，便于后续按对象类型重建。
    json root;
    root["format_version"] = 1;
    root["saved_at"] = nowAsReadableString();
    root["package_path"] = scene_graph_.this_package_path_;
    root["save_name"] = final_save_name;
    root["scene_graph"] = {
        {"target_cmd", scene_graph_.target_cmd_string_},
        {"prior_knowledge", scene_graph_.prior_knowledge_string_},
        {"history_visited_area_ids", scene_graph_.history_visited_area_ids_},
        {"cur_prompt_id", scene_graph_.cur_prompt_id_},
        {"wait_recv_id", scene_graph_.wait_recv_id_},
        {"need_area_prediction", scene_graph_.need_area_prediction_},
        {"current_poly_id", getCanonicalPolyId(scene_graph_.cur_poly_, poly_selection.canonical_poly_map, poly_id_map)}
    };
    root["flags"] = {
        {"skeleton_ready", scene_graph_.skeleton_gen_->ready()},
        {"object_factory_running", scene_graph_.object_factory_->ok()}
    };
    root["counters"] = {
        {"poly_count", polys.size()},
        {"vertex_count", vertices.size()},
        {"facet_count", facets.size()},
        {"frontier_count", frontiers.size()},
        {"area_count", area_map.size()},
        {"object_count", object_map.size()},
        {"skipped_orphan_gate_count", poly_selection.skipped_orphan_gate_count},
        {"merged_duplicate_gate_count", poly_selection.merged_duplicate_gate_count}
    };

    // 4) 按类别写入几何、拓扑、区域和 SceneGraph 自身状态。
    for (const auto& vertex : vertices) {
        json vertex_json;
        vertex_json["id"] = vertex_id_map.at(vertex.get());
        vertex_json["position"] = vec3ToJson(vertex->position_);
        vertex_json["collision_polyhedron_index"] = vec3ToJson(vertex->collision_polyhedron_index_);
        vertex_json["direction_in_unit_sphere"] = vec3ToJson(vertex->direction_in_unit_sphere_);
        vertex_json["is_visited"] = vertex->is_visited_;
        vertex_json["is_critical"] = vertex->is_critical_;
        vertex_json["type"] = static_cast<int>(vertex->type_);
        vertex_json["dir_sample_buffer_index"] = vertex->dir_sample_buffer_index_;
        vertex_json["distance_to_center"] = vertex->distance_to_center_;
        vertex_json["connected_vertex_ids"] = json::array();
        for (const auto& connected_vertex : vertex->connected_vertices_) {
            vertex_json["connected_vertex_ids"].push_back(connected_vertex != nullptr ? vertex_id_map.at(connected_vertex.get()) : -1);
        }
        root["vertices"].push_back(vertex_json);
    }

    for (const auto& facet : facets) {
        json facet_json;
        facet_json["id"] = facet_id_map.at(facet.get());
        facet_json["index"] = facet->index_;
        facet_json["center"] = vec3ToJson(facet->center_);
        facet_json["out_unit_normal"] = vec3ToJson(facet->out_unit_normal_);
        facet_json["plane_equation"] = vec4ToJson(facet->plane_equation_);
        facet_json["frontier_processed"] = facet->frontier_processed_;
        facet_json["is_linked"] = facet->is_linked_;
        facet_json["is_visited"] = facet->is_visited_;
        facet_json["master_poly_id"] = getCanonicalPolyId(facet->master_polyhedron_, poly_selection.canonical_poly_map, poly_id_map);
        facet_json["vertex_ids"] = json::array();
        facet_json["neighbor_facet_ids"] = json::array();
        for (const auto& vertex : facet->vertices_) facet_json["vertex_ids"].push_back(vertex_id_map.at(vertex.get()));
        for (const auto& neighbor : facet->neighbor_facets_) facet_json["neighbor_facet_ids"].push_back(facet_id_map.at(neighbor.get()));
        root["facets"].push_back(facet_json);
    }

    for (const auto& frontier : frontiers) {
        json frontier_json;
        frontier_json["id"] = frontier_id_map.at(frontier.get());
        frontier_json["index"] = frontier->index;
        frontier_json["avg_center"] = vec3ToJson(frontier->avg_center_);
        frontier_json["out_unit_normal"] = vec3ToJson(frontier->out_unit_normal_);
        frontier_json["proj_center"] = vec3ToJson(frontier->proj_center_);
        frontier_json["next_node_pos"] = vec3ToJson(frontier->next_node_pos_);
        frontier_json["cos_theta"] = frontier->cos_theta_;
        frontier_json["area_size"] = frontier->area_size_;
        frontier_json["valid"] = frontier->valid_;
        frontier_json["deleted"] = frontier->deleted_;
        frontier_json["master_poly_id"] = getCanonicalPolyId(frontier->master_polyhedron, poly_selection.canonical_poly_map, poly_id_map);
        frontier_json["gate_poly_id"] = getCanonicalPolyId(frontier->gate_, poly_selection.canonical_poly_map, poly_id_map);
        frontier_json["proj_facet_id"] = frontier->proj_facet_ != nullptr ? facet_id_map.at(frontier->proj_facet_.get()) : -1;
        frontier_json["facet_ids"] = json::array();
        frontier_json["vertex_ids"] = json::array();
        for (const auto& facet : frontier->facets_) frontier_json["facet_ids"].push_back(facet_id_map.at(facet.get()));
        for (const auto& vertex : frontier->vertices_) frontier_json["vertex_ids"].push_back(vertex_id_map.at(vertex.get()));
        root["frontiers"].push_back(frontier_json);
    }

    for (const auto& poly : polys) {
        json poly_json;
        poly_json["id"] = poly_id_map.at(poly.get());
        poly_json["center"] = vec3ToJson(poly->center_);
        poly_json["origin_center"] = vec3ToJson(poly->origin_center_);
        poly_json["is_gate"] = poly->is_gate_;
        poly_json["is_rollbacked"] = poly->is_rollbacked_;
        poly_json["can_reach"] = poly->can_reach_;
        poly_json["radius"] = poly->radius_;
        poly_json["box_min"] = vec3ToJson(poly->box_min_);
        poly_json["box_max"] = vec3ToJson(poly->box_max_);
        poly_json["area_id"] = poly->area_id_;
        poly_json["temp_distance_to_nxt_poly"] = poly->temp_distance_to_nxt_poly_;
        poly_json["parent_frontier_id"] = poly->parent_ftr_ != nullptr && frontier_id_map.count(poly->parent_ftr_.get()) > 0 ? frontier_id_map.at(poly->parent_ftr_.get()) : -1;
        poly_json["black_vertex_ids"] = json::array();
        poly_json["white_vertex_ids"] = json::array();
        poly_json["gray_vertex_ids"] = json::array();
        poly_json["facet_ids"] = json::array();
        poly_json["frontier_ids"] = json::array();
        poly_json["connected_node_ids"] = json::array();
        poly_json["object_ids"] = json::array();
        poly_json["candidate_rollback"] = json::array();
        poly_json["edges"] = json::array();

        for (const auto& vertex : poly->black_vertices_) poly_json["black_vertex_ids"].push_back(vertex_id_map.at(vertex.get()));
        for (const auto& vertex : poly->white_vertices_) poly_json["white_vertex_ids"].push_back(vertex_id_map.at(vertex.get()));
        for (const auto& vertex : poly->gray_vertices_) poly_json["gray_vertex_ids"].push_back(vertex_id_map.at(vertex.get()));
        for (const auto& facet : poly->facets_) poly_json["facet_ids"].push_back(facet_id_map.at(facet.get()));
        for (const auto& frontier : poly->ftrs_) poly_json["frontier_ids"].push_back(frontier_id_map.at(frontier.get()));
        for (const auto& connected_node : poly->connected_nodes_) {
            poly_json["connected_node_ids"].push_back(getCanonicalPolyId(connected_node, poly_selection.canonical_poly_map, poly_id_map));
        }
        for (const auto& obj_pair : poly->objs_) poly_json["object_ids"].push_back(obj_pair.first);
        for (const auto& rollback_pair : poly->candidate_rollback_) {
            poly_json["candidate_rollback"].push_back({{"poly_origin_center", vec3ToJson(rollback_pair.first)}, {"value", rollback_pair.second}});
        }
        for (const auto& edge : poly->edges_) {
            json edge_json;
            edge_json["dst_poly_id"] = getCanonicalPolyId(edge.poly_nxt_, poly_selection.canonical_poly_map, poly_id_map);
            edge_json["length"] = edge.length_;
            edge_json["weight"] = edge.weight_;
            edge_json["is_force_connected"] = edge.is_force_connected_;
            edge_json["path"] = json::array();
            for (const auto& path_point : edge.path_) edge_json["path"].push_back(vec3ToJson(path_point));
            poly_json["edges"].push_back(edge_json);
        }
        root["polyhedrons"].push_back(poly_json);
    }

    for (const auto& area_pair : area_map) {
        const auto& area = area_pair.second;
        json area_json;
        area_json["id"] = area->id_;
        area_json["room_label"] = area->room_label_;
        area_json["room_description"] = area->room_description_;
        area_json["box_min"] = vec3ToJson(area->box_min_);
        area_json["box_max"] = vec3ToJson(area->box_max_);
        area_json["center"] = vec3ToJson(area->center_);
        area_json["color"] = vec3ToJson(area->color_);
        area_json["num_ftrs"] = area->num_ftrs_;
        area_json["last_obj_num"] = area->last_obj_num_;
        area_json["poly_ids"] = json::array();
        area_json["object_ids"] = json::array();
        area_json["neighbor_area_ids"] = json::array();
        for (const auto& poly : area->polys_) area_json["poly_ids"].push_back(getCanonicalPolyId(poly, poly_selection.canonical_poly_map, poly_id_map));
        for (const auto& obj : area->objects_) area_json["object_ids"].push_back(obj != nullptr ? obj->id : -1);
        for (const auto& nbr : area->nbr_area_) area_json["neighbor_area_ids"].push_back(nbr.first);
        root["areas"].push_back(area_json);
    }

    root["areas_need_predict"] = json::array();
    for (const auto& area_pair : scene_graph_.skeleton_gen_->area_handler_->areas_need_predict_) {
        root["areas_need_predict"].push_back({{"area_id", area_pair.first}, {"value", area_pair.second}});
    }
    root["areas_need_delete"] = json::array();
    for (const auto& area_pair : scene_graph_.skeleton_gen_->area_handler_->areas_need_delete_) {
        root["areas_need_delete"].push_back({{"area_id", area_pair.first}, {"value", area_pair.second}});
    }

    // 目标检测对象除了元数据外，还会把点云附件落盘到独立文件。
    // 5) 单独导出 object 的点云附件，并在 JSON 中记录相对路径。
    int saved_cloud_num = 0;
    for (const auto& obj_pair : object_map) {
        const auto& obj = obj_pair.second;
        json obj_json;
        obj_json["id"] = obj->id;
        obj_json["label"] = obj->label;
        obj_json["conf"] = obj->conf;
        obj_json["pos"] = vec3ToJson(obj->pos);
        obj_json["color"] = vec3ToJson(obj->color);
        obj_json["label_feature"] = vectorXdToJson(obj->label_feature);
        obj_json["is_alive"] = obj->is_alive;
        obj_json["last_detection_time_sec"] = obj->last_detection_time.toSec();
        obj_json["detection_count"] = obj->detection_count;
        obj_json["need_more_detection"] = scene_graph_.object_factory_->object_map_needMoreDetection_.find(obj->id) != scene_graph_.object_factory_->object_map_needMoreDetection_.end();
        obj_json["cloud_size"] = obj->cloud != nullptr ? obj->cloud->size() : 0;
        obj_json["obb_corners_size"] = obj->obb_corners != nullptr ? obj->obb_corners->size() : 0;
        obj_json["obb_axis_size"] = obj->obb_axis != nullptr ? obj->obb_axis->size() : 0;
        obj_json["edge"] = {
            {"father_type", static_cast<int>(obj->edge.father_type)},
            {"edge_description", obj->edge.edge_description},
            {"father_poly_id", getCanonicalPolyId(obj->edge.polyhedron_father, poly_selection.canonical_poly_map, poly_id_map)},
            {"father_object_id", obj->edge.object_father != nullptr ? obj->edge.object_father->id : -1},
            {"child_object_ids", json::array()}
        };
        for (const auto& child : obj->edge.object_child) obj_json["edge"]["child_object_ids"].push_back(child != nullptr ? child->id : -1);

        const std::string obj_prefix = "object_" + std::to_string(obj->id);
        const std::string cloud_rel_path = "objects/" + obj_prefix + "_cloud.pcd";
        const std::string obb_rel_path = "objects/" + obj_prefix + "_obb_corners.pcd";
        const std::string axis_rel_path = "objects/" + obj_prefix + "_obb_axis.pcd";
        obj_json["files"] = {{"cloud", ""}, {"obb_corners", ""}, {"obb_axis", ""}};

        if (obj->cloud != nullptr && !obj->cloud->empty()) {
            if (pcl::io::savePCDFileBinary(joinPath(object_dir, obj_prefix + "_cloud.pcd"), *obj->cloud) == 0) {
                obj_json["files"]["cloud"] = cloud_rel_path;
                ++saved_cloud_num;
            } else {
                INFO_MSG_YELLOW("[SceneGraphMapIO] Skip object cloud save, id = " << obj->id);
            }
        }
        if (obj->obb_corners != nullptr && !obj->obb_corners->empty()) {
            if (pcl::io::savePCDFileBinary(joinPath(object_dir, obj_prefix + "_obb_corners.pcd"), *obj->obb_corners) == 0) {
                obj_json["files"]["obb_corners"] = obb_rel_path;
            } else {
                INFO_MSG_YELLOW("[SceneGraphMapIO] Skip object OBB corners save, id = " << obj->id);
            }
        }
        if (obj->obb_axis != nullptr && !obj->obb_axis->empty()) {
            if (pcl::io::savePCDFileBinary(joinPath(object_dir, obj_prefix + "_obb_axis.pcd"), *obj->obb_axis) == 0) {
                obj_json["files"]["obb_axis"] = axis_rel_path;
            } else {
                INFO_MSG_YELLOW("[SceneGraphMapIO] Skip object OBB axis save, id = " << obj->id);
            }
        }
        root["objects"].push_back(obj_json);
    }

    json manifest;
    manifest["format_version"] = 1;
    manifest["saved_at"] = root["saved_at"];
    manifest["save_name"] = final_save_name;
    manifest["scene_graph_file"] = "scene_graph.json";
    manifest["object_dir"] = "objects";
    manifest["summary"] = {
        {"poly_count", polys.size()},
        {"area_count", area_map.size()},
        {"object_count", object_map.size()},
        {"saved_cloud_num", saved_cloud_num},
        {"skipped_orphan_gate_count", poly_selection.skipped_orphan_gate_count},
        {"merged_duplicate_gate_count", poly_selection.merged_duplicate_gate_count}
    };

    // 6) 写出主 JSON 和 manifest，作为一次可回放的快照。
    const bool scene_graph_ok = writeJsonFile(joinPath(save_dir, "scene_graph.json"), root);
    const bool manifest_ok = writeJsonFile(joinPath(save_dir, "manifest.json"), manifest);
    if (!scene_graph_ok || !manifest_ok) {
        INFO_MSG_RED("[SceneGraphMapIO] Save failed: metadata file write error.");
        return false;
    }

    INFO_MSG_GREEN("[SceneGraphMapIO] Save completed.");
    INFO_MSG_GREEN("[SceneGraphMapIO] Summary | poly: " << polys.size()
                   << ", vertex: " << vertices.size()
                   << ", facet: " << facets.size()
                   << ", frontier: " << frontiers.size()
                   << ", area: " << area_map.size()
                   << ", object: " << object_map.size()
                   << ", skipped orphan gate: " << poly_selection.skipped_orphan_gate_count
                   << ", merged duplicate gate: " << poly_selection.merged_duplicate_gate_count);
    INFO_MSG_BLUE("[SceneGraphMapIO] ====================================================\n\n");
    return true;
}

bool SceneGraphMapIO::load(const std::string& save_name) {
    std::string final_save_name;
    if (!normalizeRelativeSavePath(save_name, final_save_name, false)) {
        INFO_MSG_RED("[SceneGraphMapIO] Load failed: save path must be a valid relative path.");
        return false;
    }

    INFO_MSG_BLUE("\n[SceneGraphMapIO] ================= Load Scene Graph =================");

    // 1) 根据相对保存路径定位快照目录，并读取入口 manifest。
    const std::string saved_data_dir = joinPath(scene_graph_.this_package_path_, "saved_data");
    const std::string save_dir = joinPath(saved_data_dir, final_save_name);
    const std::string manifest_path = joinPath(save_dir, "manifest.json");

    json manifest;
    if (!readJsonFile(manifest_path, manifest)) {
        INFO_MSG_RED("[SceneGraphMapIO] Load failed: cannot read manifest file: " << manifest_path);
        return false;
    }
    if (manifest.value("format_version", -1) != 1) {
        INFO_MSG_RED("[SceneGraphMapIO] Load failed: unsupported format_version = " << manifest.value("format_version", -1));
        return false;
    }

    // 2) 校验快照格式，解析主 scene graph 文件。
    const std::string scene_graph_file = manifest.value("scene_graph_file", std::string("scene_graph.json"));
    const std::string object_dir_name = manifest.value("object_dir", std::string("objects"));
    const std::string scene_graph_path = joinPath(save_dir, scene_graph_file);
    const std::string object_dir = joinPath(save_dir, object_dir_name);

    json root;
    if (!readJsonFile(scene_graph_path, root)) {
        INFO_MSG_RED("[SceneGraphMapIO] Load failed: cannot read scene graph file: " << scene_graph_path);
        return false;
    }
    if (root.value("format_version", -1) != 1) {
        INFO_MSG_RED("[SceneGraphMapIO] Load failed: scene graph format mismatch.");
        return false;
    }

    // 3) 锁住运行时模块，防止加载过程中被并发访问。
    std::unique_lock<SkeletonGenerator> skeleton_lock(*scene_graph_.skeleton_gen_);
    std::unique_lock<ObjectFactory> object_lock(*scene_graph_.object_factory_);

    INFO_MSG_YELLOW("[SceneGraphMapIO] Load path : " << save_dir);
    INFO_MSG_YELLOW("[SceneGraphMapIO] Resetting runtime state ...");

    // 清空旧运行时状态，确保后续完全由快照内容重建。
    scene_graph_.skeleton_gen_->resetForMapLoad();
    scene_graph_.skeleton_gen_->area_handler_->resetForMapLoad();
    scene_graph_.object_factory_->resetForMapLoad();
    
    scene_graph_.cur_poly_ = nullptr;
    scene_graph_.history_visited_area_ids_.clear();
    scene_graph_.llm_ans_str_poll_.clear();
    scene_graph_.llm_prompts_.clear();
    scene_graph_.llm_ans_promises_.clear();
    scene_graph_.cur_prompt_id_ = 0;
    scene_graph_.wait_recv_id_ = 0;
    scene_graph_.need_area_prediction_ = false;

    std::map<int, VertexPtr> vertex_lookup;
    std::map<int, FacetPtr> facet_lookup;
    std::map<int, PolyhedronFtrPtr> frontier_lookup;
    std::map<int, PolyHedronPtr> poly_lookup;
    std::map<int, PolyhedronCluster::Ptr> area_lookup;
    std::map<int, ObjectNode::Ptr> object_lookup;
    std::map<int, bool> object_need_more_detection;

    INFO_MSG_YELLOW("[SceneGraphMapIO] Creating runtime objects ...");

    // 第一阶段仅创建对象本体并恢复标量字段，关系引用留到下一阶段统一连接。
    // 4) 先按 ID 重建各类运行时对象，并恢复它们的基础属性。
    for (const auto& vertex_json : root.value("vertices", json::array())) {
        const int id = vertex_json.at("id").get<int>();
        VertexPtr vertex = std::make_shared<Vertex>();
        vertex->position_ = jsonToVec3(vertex_json.value("position", json::array()));
        vertex->collision_polyhedron_index_ = jsonToVec3(vertex_json.value("collision_polyhedron_index", json::array()), Eigen::Vector3d(-9999.0, -9999.0, -9999.0));
        vertex->direction_in_unit_sphere_ = jsonToVec3(vertex_json.value("direction_in_unit_sphere", json::array()));
        vertex->is_visited_ = vertex_json.value("is_visited", false);
        vertex->is_critical_ = vertex_json.value("is_critical", false);
        vertex->type_ = static_cast<Vertex::VertexType>(vertex_json.value("type", 0));
        vertex->dir_sample_buffer_index_ = vertex_json.value("dir_sample_buffer_index", -1);
        vertex->distance_to_center_ = vertex_json.value("distance_to_center", 0.0);
        vertex->connected_vertices_.clear();
        vertex_lookup[id] = vertex;
    }

    for (const auto& facet_json : root.value("facets", json::array())) {
        const int id = facet_json.at("id").get<int>();
        FacetPtr facet = std::make_shared<Facet>();
        facet->index_ = facet_json.value("index", -1);
        facet->center_ = jsonToVec3(facet_json.value("center", json::array()));
        facet->out_unit_normal_ = jsonToVec3(facet_json.value("out_unit_normal", json::array()));
        facet->plane_equation_ = jsonToVec4(facet_json.value("plane_equation", json::array()));
        facet->frontier_processed_ = facet_json.value("frontier_processed", false);
        facet->is_linked_ = facet_json.value("is_linked", false);
        facet->is_visited_ = facet_json.value("is_visited", false);
        facet->vertices_.clear();
        facet->neighbor_facets_.clear();
        facet->master_polyhedron_ = nullptr;
        facet_lookup[id] = facet;
    }

    for (const auto& frontier_json : root.value("frontiers", json::array())) {
        const int id = frontier_json.at("id").get<int>();
        PolyhedronFtrPtr frontier = std::make_shared<PolyhedronFtr>();
        frontier->index = frontier_json.value("index", -1);
        frontier->avg_center_ = jsonToVec3(frontier_json.value("avg_center", json::array()));
        frontier->out_unit_normal_ = jsonToVec3(frontier_json.value("out_unit_normal", json::array()));
        frontier->proj_center_ = jsonToVec3(frontier_json.value("proj_center", json::array()));
        frontier->next_node_pos_ = jsonToVec3(frontier_json.value("next_node_pos", json::array()));
        frontier->cos_theta_ = frontier_json.value("cos_theta", 0.0);
        frontier->area_size_ = frontier_json.value("area_size", 0.0);
        frontier->valid_ = frontier_json.value("valid", false);
        frontier->deleted_ = frontier_json.value("deleted", false);
        frontier->master_polyhedron = nullptr;
        frontier->gate_ = nullptr;
        frontier->proj_facet_ = nullptr;
        frontier->facets_.clear();
        frontier->vertices_.clear();
        frontier_lookup[id] = frontier;
    }

    for (const auto& poly_json : root.value("polyhedrons", json::array())) {
        const int id = poly_json.at("id").get<int>();
        PolyHedronPtr poly = std::make_shared<Polyhedron>();
        poly->center_ = jsonToVec3(poly_json.value("center", json::array()));
        poly->origin_center_ = jsonToVec3(poly_json.value("origin_center", json::array()));
        poly->is_gate_ = poly_json.value("is_gate", false);
        poly->is_rollbacked_ = poly_json.value("is_rollbacked", false);
        poly->can_reach_ = poly_json.value("can_reach", false);
        poly->radius_ = poly_json.value("radius", 0.0);
        poly->box_min_ = jsonToVec3(poly_json.value("box_min", json::array()), Eigen::Vector3d(99999.0, 99999.0, 99999.0));
        poly->box_max_ = jsonToVec3(poly_json.value("box_max", json::array()), Eigen::Vector3d(-99999.0, -99999.0, -99999.0));
        poly->area_id_ = poly_json.value("area_id", -1);
        poly->temp_distance_to_nxt_poly_ = poly_json.value("temp_distance_to_nxt_poly", 0.0);
        poly->black_vertices_.clear();
        poly->white_vertices_.clear();
        poly->gray_vertices_.clear();
        poly->facets_.clear();
        poly->ftrs_.clear();
        poly->connected_nodes_.clear();
        poly->edges_.clear();
        poly->objs_.clear();
        poly->candidate_rollback_.clear();
        poly->parent_ftr_ = nullptr;
        poly_lookup[id] = poly;
    }

    for (const auto& area_json : root.value("areas", json::array())) {
        const int id = area_json.at("id").get<int>();
        PolyhedronCluster::Ptr area = std::make_shared<PolyhedronCluster>();
        area->id_ = id;
        area->room_label_ = area_json.value("room_label", std::string(""));
        area->room_description_ = area_json.value("room_description", std::string(""));
        area->box_min_ = jsonToVec3(area_json.value("box_min", json::array()), Eigen::Vector3d(99999.0, 99999.0, 99999.0));
        area->box_max_ = jsonToVec3(area_json.value("box_max", json::array()), Eigen::Vector3d(-99999.0, -99999.0, -99999.0));
        area->center_ = jsonToVec3(area_json.value("center", json::array()));
        area->color_ = jsonToVec3(area_json.value("color", json::array()), ColorGenerator::getColorById(id));
        area->num_ftrs_ = area_json.value("num_ftrs", 0);
        area->last_obj_num_ = area_json.value("last_obj_num", 0);
        area->polys_.clear();
        area->objects_.clear();
        area->nbr_area_.clear();
        area_lookup[id] = area;
    }

    for (const auto& obj_json : root.value("objects", json::array())) {
        const int id = obj_json.at("id").get<int>();
        ObjectNode::Ptr obj = std::make_shared<ObjectNode>();
        obj->id = id;
        obj->label = obj_json.value("label", std::string("None"));
        obj->conf = obj_json.value("conf", 0.0);
        obj->pos = jsonToVec3(obj_json.value("pos", json::array()));
        obj->color = jsonToVec3(obj_json.value("color", json::array()));
        obj->label_feature = jsonToVectorXd(obj_json.value("label_feature", json::array()));
        obj->is_alive = obj_json.value("is_alive", true);
        obj->last_detection_time = ros::Time(obj_json.value("last_detection_time_sec", 0.0));
        obj->detection_count = obj_json.value("detection_count", 0u);
        object_need_more_detection[id] = obj_json.value("need_more_detection", false);

        const json files_json = obj_json.value("files", json::object());
        const std::string cloud_rel = files_json.value("cloud", std::string(""));
        const std::string corners_rel = files_json.value("obb_corners", std::string(""));
        const std::string axis_rel = files_json.value("obb_axis", std::string(""));
        if (!cloud_rel.empty()) loadPointCloudFile<pcl::PointXYZRGB>(joinPath(save_dir, cloud_rel), obj->cloud);
        if (!corners_rel.empty()) loadPointCloudFile<pcl::PointXYZ>(joinPath(save_dir, corners_rel), obj->obb_corners);
        if (!axis_rel.empty()) loadPointCloudFile<pcl::PointXYZ>(joinPath(save_dir, axis_rel), obj->obb_axis);
        object_lookup[id] = obj;
    }

    INFO_MSG_YELLOW("[SceneGraphMapIO] Resolving runtime relations ...");

    // 第二阶段按保存时记录的 ID 恢复拓扑关系、归属关系和图边。
    // 5) 再补回对象之间的引用关系，恢复完整场景拓扑。
    for (const auto& vertex_json : root.value("vertices", json::array())) {
        VertexPtr vertex = vertex_lookup.at(vertex_json.at("id").get<int>());
        for (const int connected_id : jsonToIntVector(vertex_json.value("connected_vertex_ids", json::array()))) {
            auto it = vertex_lookup.find(connected_id);
            if (it != vertex_lookup.end()) vertex->connected_vertices_.push_back(it->second);
        }
    }

    for (const auto& facet_json : root.value("facets", json::array())) {
        FacetPtr facet = facet_lookup.at(facet_json.at("id").get<int>());
        for (const int vertex_id : jsonToIntVector(facet_json.value("vertex_ids", json::array()))) {
            auto it = vertex_lookup.find(vertex_id);
            if (it != vertex_lookup.end()) facet->vertices_.push_back(it->second);
        }
        for (const int neighbor_id : jsonToIntVector(facet_json.value("neighbor_facet_ids", json::array()))) {
            auto it = facet_lookup.find(neighbor_id);
            if (it != facet_lookup.end()) facet->neighbor_facets_.push_back(it->second);
        }
        const int master_poly_id = facet_json.value("master_poly_id", -1);
        if (poly_lookup.find(master_poly_id) != poly_lookup.end()) facet->master_polyhedron_ = poly_lookup.at(master_poly_id);
    }

    for (const auto& frontier_json : root.value("frontiers", json::array())) {
        PolyhedronFtrPtr frontier = frontier_lookup.at(frontier_json.at("id").get<int>());
        const int master_poly_id = frontier_json.value("master_poly_id", -1);
        const int gate_poly_id = frontier_json.value("gate_poly_id", -1);
        const int proj_facet_id = frontier_json.value("proj_facet_id", -1);
        if (poly_lookup.find(master_poly_id) != poly_lookup.end()) frontier->master_polyhedron = poly_lookup.at(master_poly_id);
        if (poly_lookup.find(gate_poly_id) != poly_lookup.end()) frontier->gate_ = poly_lookup.at(gate_poly_id);
        if (facet_lookup.find(proj_facet_id) != facet_lookup.end()) frontier->proj_facet_ = facet_lookup.at(proj_facet_id);
        for (const int facet_id : jsonToIntVector(frontier_json.value("facet_ids", json::array()))) {
            auto it = facet_lookup.find(facet_id);
            if (it != facet_lookup.end()) frontier->facets_.push_back(it->second);
        }
        for (const int vertex_id : jsonToIntVector(frontier_json.value("vertex_ids", json::array()))) {
            auto it = vertex_lookup.find(vertex_id);
            if (it != vertex_lookup.end()) frontier->vertices_.push_back(it->second);
        }
    }

    for (const auto& poly_json : root.value("polyhedrons", json::array())) {
        PolyHedronPtr poly = poly_lookup.at(poly_json.at("id").get<int>());
        for (const int vertex_id : jsonToIntVector(poly_json.value("black_vertex_ids", json::array()))) {
            auto it = vertex_lookup.find(vertex_id);
            if (it != vertex_lookup.end()) poly->black_vertices_.push_back(it->second);
        }
        for (const int vertex_id : jsonToIntVector(poly_json.value("white_vertex_ids", json::array()))) {
            auto it = vertex_lookup.find(vertex_id);
            if (it != vertex_lookup.end()) poly->white_vertices_.push_back(it->second);
        }
        for (const int vertex_id : jsonToIntVector(poly_json.value("gray_vertex_ids", json::array()))) {
            auto it = vertex_lookup.find(vertex_id);
            if (it != vertex_lookup.end()) poly->gray_vertices_.push_back(it->second);
        }
        for (const int facet_id : jsonToIntVector(poly_json.value("facet_ids", json::array()))) {
            auto it = facet_lookup.find(facet_id);
            if (it != facet_lookup.end()) poly->facets_.push_back(it->second);
        }
        for (const int frontier_id : jsonToIntVector(poly_json.value("frontier_ids", json::array()))) {
            auto it = frontier_lookup.find(frontier_id);
            if (it != frontier_lookup.end()) poly->ftrs_.push_back(it->second);
        }
        for (const int connected_id : jsonToIntVector(poly_json.value("connected_node_ids", json::array()))) {
            auto it = poly_lookup.find(connected_id);
            if (it != poly_lookup.end()) poly->connected_nodes_.push_back(it->second);
        }
        for (const int object_id : jsonToIntVector(poly_json.value("object_ids", json::array()))) {
            auto it = object_lookup.find(object_id);
            if (it != object_lookup.end()) poly->objs_[object_id] = it->second;
        }
        for (const auto& rollback_json : poly_json.value("candidate_rollback", json::array())) {
            poly->candidate_rollback_[jsonToVec3(rollback_json.value("poly_origin_center", json::array()))] = rollback_json.value("value", false);
        }
        const int parent_frontier_id = poly_json.value("parent_frontier_id", -1);
        if (frontier_lookup.find(parent_frontier_id) != frontier_lookup.end()) poly->parent_ftr_ = frontier_lookup.at(parent_frontier_id);
        for (const auto& edge_json : poly_json.value("edges", json::array())) {
            const int dst_poly_id = edge_json.value("dst_poly_id", -1);
            auto it = poly_lookup.find(dst_poly_id);
            if (it == poly_lookup.end()) continue;
            Edge edge(it->second, edge_json.value("length", 0.0));
            edge.weight_ = edge_json.value("weight", 0.0);
            edge.is_force_connected_ = edge_json.value("is_force_connected", false);
            edge.path_.clear();
            for (const auto& path_point : edge_json.value("path", json::array())) edge.path_.push_back(jsonToVec3(path_point));
            poly->edges_.push_back(edge);
        }
    }

    for (const auto& area_json : root.value("areas", json::array())) {
        PolyhedronCluster::Ptr area = area_lookup.at(area_json.at("id").get<int>());
        for (const int poly_id : jsonToIntVector(area_json.value("poly_ids", json::array()))) {
            auto it = poly_lookup.find(poly_id);
            if (it != poly_lookup.end()) area->polys_.push_back(it->second);
        }
        for (const int object_id : jsonToIntVector(area_json.value("object_ids", json::array()))) {
            auto it = object_lookup.find(object_id);
            if (it != object_lookup.end()) area->objects_.push_back(it->second);
        }
        for (const int neighbor_area_id : jsonToIntVector(area_json.value("neighbor_area_ids", json::array()))) {
            area->nbr_area_[neighbor_area_id] = true;
        }
    }

    for (const auto& obj_json : root.value("objects", json::array())) {
        ObjectNode::Ptr obj = object_lookup.at(obj_json.at("id").get<int>());
        const json edge_json = obj_json.value("edge", json::object());
        obj->edge.father_type = static_cast<ObjectEdge::EdgeType>(edge_json.value("father_type", 0));
        obj->edge.edge_description = edge_json.value("edge_description", std::string(""));
        const int father_poly_id = edge_json.value("father_poly_id", -1);
        const int father_object_id = edge_json.value("father_object_id", -1);
        if (poly_lookup.find(father_poly_id) != poly_lookup.end()) obj->edge.polyhedron_father = poly_lookup.at(father_poly_id);
        if (object_lookup.find(father_object_id) != object_lookup.end()) obj->edge.object_father = object_lookup.at(father_object_id);
        obj->edge.object_child.clear();
        for (const int child_id : jsonToIntVector(edge_json.value("child_object_ids", json::array()))) {
            auto it = object_lookup.find(child_id);
            if (it != object_lookup.end()) obj->edge.object_child.push_back(it->second);
        }
    }

    INFO_MSG_YELLOW("[SceneGraphMapIO] Rebuilding runtime indices ...");

    // 最后重新注册到各运行时管理器，并恢复 SceneGraph 级别的上下文状态。
    // 6) 把重建后的对象重新挂回管理器，并补齐全局上下文与缓存状态。
    for (const auto& area_pair : area_lookup) {
        if (!scene_graph_.skeleton_gen_->area_handler_->registerLoadedArea(area_pair.second)) return false;
    }
    for (const auto& poly_pair : poly_lookup) {
        if (!scene_graph_.skeleton_gen_->registerLoadedPolyhedron(poly_pair.second)) return false;
    }
    for (const auto& object_pair : object_lookup) {
        if (!scene_graph_.object_factory_->registerLoadedObject(object_pair.second, object_need_more_detection[object_pair.first])) return false;
    }

    scene_graph_.skeleton_gen_->area_handler_->areas_need_predict_.clear();
    for (const auto& item : root.value("areas_need_predict", json::array())) {
        scene_graph_.skeleton_gen_->area_handler_->areas_need_predict_[item.value("area_id", -1)] = item.value("value", false);
    }
    scene_graph_.skeleton_gen_->area_handler_->areas_need_delete_.clear();
    for (const auto& item : root.value("areas_need_delete", json::array())) {
        scene_graph_.skeleton_gen_->area_handler_->areas_need_delete_[item.value("area_id", -1)] = item.value("value", false);
    }

    const json scene_json = root.value("scene_graph", json::object());
    scene_graph_.setTargetAndPriorKnowledge(scene_json.value("target_cmd", std::string("")),
                                            scene_json.value("prior_knowledge", std::string("")));
    scene_graph_.history_visited_area_ids_ = jsonToIntVector(scene_json.value("history_visited_area_ids", json::array()));
    scene_graph_.cur_prompt_id_ = scene_json.value("cur_prompt_id", 0u);
    scene_graph_.wait_recv_id_ = scene_json.value("wait_recv_id", 0);
    scene_graph_.need_area_prediction_ = scene_json.value("need_area_prediction", false) ||
                                         !scene_graph_.skeleton_gen_->area_handler_->areas_need_predict_.empty();
    const int current_poly_id = scene_json.value("current_poly_id", -1);
    if (poly_lookup.find(current_poly_id) != poly_lookup.end()) scene_graph_.cur_poly_ = poly_lookup.at(current_poly_id);

    scene_graph_.skeleton_gen_->area_handler_->finishMapLoad();
    scene_graph_.skeleton_gen_->finishMapLoad();
    scene_graph_.object_factory_->finishMapLoad();

    INFO_MSG_GREEN("[SceneGraphMapIO] Load completed.");
    INFO_MSG_GREEN("[SceneGraphMapIO] Summary | poly: " << poly_lookup.size()
                   << ", vertex: " << vertex_lookup.size()
                   << ", facet: " << facet_lookup.size()
                   << ", frontier: " << frontier_lookup.size()
                   << ", area: " << area_lookup.size()
                   << ", object: " << object_lookup.size());
    INFO_MSG_BLUE("[SceneGraphMapIO] ====================================================\n");
    return true;
}
