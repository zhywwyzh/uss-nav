//
// Created by gwq on 8/14/25.
//

#ifndef SCENE_GRAPH_H
#define SCENE_GRAPH_H

#include <ros/ros.h>
#include <ros/package.h>
#include "../include/scene_graph/data_structure.h"
#include "../include/scene_graph/skeleton_generation.h"
#include "../include/scene_graph/object_factory.h"
#include "../include/scene_graph/skeleton_cluster.h"
#include "nlohmann/json.hpp"
#include <fstream>

#include <visualization_msgs/MarkerArray.h>
#include "scene_graph/PromptMsg.h"

class SceneGraphMapIO;

struct SceneGraph_Data {
std::vector<AreaHandler::Ptr> area;
};

struct VLASwarmPromptResult {
    bool valid{false};
    bool success{false};
    std::string error;
    std::string detail;
    nlohmann::json payload;
};

class SceneGraph {
public:
    typedef std::shared_ptr<SceneGraph> Ptr;
    SceneGraph(ros::NodeHandle& nh, ego_planner::MapInterface::Ptr& map_interface) {
        nh_ = nh;
        map_interface_      = map_interface;
        scene_graph_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/scene_graph/vis", 2);
        prompt_pub_         = nh_.advertise<scene_graph::PromptMsg>("/scene_graph/prompt", 2);
        llm_ans_sub_        = nh_.subscribe("/scene_graph/llm_ans", 2, &SceneGraph::llmAnsCallback, this, ros::TransportHints().tcpNoDelay());
        skeleton_gen_       = std::make_shared<SkeletonGenerator>(nh, map_interface);
        object_factory_     = std::make_unique<ObjectFactory>(nh, skeleton_gen_);
        this_package_path_  = ros::package::getPath("scene_graph");
        // 拓扑点不可达检测/修复/标记 参数(可被 YAML/launch 覆盖)
        nh_.param("topo_block/enable",                  topo_block_enable_,             true);
        nh_.param("topo_block/repair_radius",           topo_repair_radius_,            0.5);
        nh_.param("topo_block/repair_vis_mode",         topo_repair_vis_mode_,           0);
        nh_.param("topo_block/repair_vis_sphere_radius", topo_repair_vis_sphere_radius_, 2.0);
        nh_.param("topo_block/hits_thresh",             topo_block_hits_thresh_,        2);
        nh_.param("topo_block/ttl",                     topo_block_ttl_,                8.0);
        nh_.param("topo_block/revalidate_on_fail",      topo_block_revalidate_on_fail_, true);
        nh_.param("topo_block/max_iter",                topo_block_max_iter_,            4);
        nh_.param("topo_block/repair_insert_node",     topo_repair_insert_node_,        false);
        INFO_MSG("SceneGraph initialized, package path: " << this_package_path_);
    };
    ~SceneGraph() = default;
    // submodules //
    SkeletonGenerator::Ptr  skeleton_gen_;
    ObjectFactory::UPtr     object_factory_;
    PolyHedronPtr           cur_poly_;
    std::vector<int>        history_visited_area_ids_;

    std::string target_cmd_string_;
    std::string prior_knowledge_string_;

    void setTargetAndPriorKnowledge(const std::string& target_cmd_str, const std::string& prior_knowledge_str);

    // current state interface
    void mountCurPoly(const Eigen::Vector3d pos, const double yaw);
    PolyHedronPtr getCurPoly() {return cur_poly_;};
    int    getRepairVisMode()        const { return topo_repair_vis_mode_; }
    double getRepairVisSphereRadius() const { return topo_repair_vis_sphere_radius_; }

    // scene graph operations //
    bool initSceneGraph(const Eigen::Vector3d &cur_pos, double yaw);
    void updateSceneGraph(const Eigen::Vector3d &cur_pos, const double &yaw, bool &new_topo);
    void updateObjectToSceneGraph();
    // 场景图更新冻结控制：冻结后 updateSceneGraph 不再增量更新拓扑，但 loadMap 全量加载不受影响
    void freezeUpdate()   { scene_graph_update_frozen_ = true; }
    void unfreezeUpdate() { scene_graph_update_frozen_ = false; }
    bool isUpdateFrozen() const { return scene_graph_update_frozen_; }
    bool getPathToObjectWithId(const int &id, std::vector<Eigen::Vector3d> &path, Eigen::Vector3d & aim_pos, double &aim_yaw);

    // 返航辅助: fly-origin 挂载点查询/注册 (转发给 object_factory_)
    // fly-origin 是一种虚拟 object, 用于在 topo 图上持久化绑定 "返航起点" 所在的稳定 polyhedron,
    // 避免每次返航都依赖脆弱的实时 mountCurTopoPoint 吸附
    // 查询是否已有 fly-origin 绑定, 未找到返回 nullptr
    ObjectNode::Ptr findFlyOrigin() { return object_factory_->findFlyOrigin(); }
    // 在指定 polyhedron 上挂载/更新 fly-origin
    bool registerFlyOriginAtPoly(const Eigen::Vector3d& pos, const PolyHedronPtr& poly) {
        return object_factory_->registerFlyOriginAtPoly(pos, poly);
    }
    // 发布 fly-origin 状态日志到 /if_hold_origin
    void publishFlyOriginStatus(bool found) { object_factory_->publishFlyOriginStatus(found); }

    // 拓扑点不可达: 检测 / 修复 / 标记 / 恢复 //
    // C0: 把不可达点投影到最近的 (在local map内 且 inflate-free) 点;
    //     toward 为前进参考点(下一个路径点/目标), 投影结果趋向该方向且排除往回方向; 失败返回 false
    bool projectToInflateFree(const Eigen::Vector3d &p, const Eigen::Vector3d &toward, Eigen::Vector3d &p_out);
    // 球交会中间点: probe和toward各自画半径为sphere_radius的可见性球, 在交会区域搜索inflate-free且双向isVisible的中间点
    bool findIntersectionMidpoint(const Eigen::Vector3d &probe, const Eigen::Vector3d &toward,
                                  double sphere_radius, Eigen::Vector3d &mid_out);
    // 按 center 在上次 A* 路径多面体中查找并标记 nav_blocked_(force=true 时立即置位, 否则按去抖累计)
    void markPolyhedronBlocked(const Eigen::Vector3d &center, bool force = false);
    // 策略A: 遍历被标记节点, 超过 TTL 的重新校验 occupancy, 已空闲则清除标记
    void revalidateBlocked();
    // 策略B: 清空全部不可达标记(A* 全失败时调用)
    void clearAllBlocked();
    // 将修复点插入拓扑图: 创建新多面体+注册kdtree+连接可见邻居; 旧节点永久封锁不恢复
    void insertReplacementNode(const Eigen::Vector3d &old_center, const Eigen::Vector3d &new_center);
    // 某点是否处于"在local map内 且 inflate占据"(仅此情形判为坏点; 越界点不算)
    bool isInflateBlocked(const Eigen::Vector3d &p);

    // LLM interface //
    std::map<unsigned int, std::string> llm_ans_str_poll_;
    std::map<unsigned int, scene_graph::PromptMsg> llm_prompts_;
    std::future<std::string> sendPrompt(unsigned int prompt_id, unsigned char prompt_type, std::string prompt_str,
                                        const std::chrono::seconds &timeout, int max_retries);
    int wait_recv_id_;
    bool hasPromptAnswer(unsigned int prompt_id);
    void clearPromptData(unsigned int prompt_id);

    template<typename T>
    bool waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout);

    // prompt generation //
    bool allRoomPredictionPromptGen(std::string &prompt_str);
    bool singleRoomPredictionPromptGen(const int room_id, nlohmann::json &prompt_json);
    bool newAreaPredictionPromptGen(std::string &prompt_str);
    bool chooseAreaToGoPromptGen(std::string &prompt_str);
    bool chooseTerminateObjIdPromptGen(std::string &prompt_str);
    bool DFDemoPromptGen(std::string &prompt_str);
    void sendSceneGraphJson(std::string &scene_graph_json_str);
    bool vlaSwarmPromptGen(unsigned char prompt_type, const std::string &command,
                           uint32_t task_session_id, uint32_t observation_batch_id,
                           const nlohmann::json &semantic_context,
                           std::string &prompt_str) const;

    // result handle //
    void handleRoomPredictionResult(unsigned int prompt_id);
    int handelExplorationResult(unsigned int prompt_id);
    int handelTerminateObjIdResult(unsigned int prompt_id);
    int handelDFDemoResult(unsigned int prompt_id);
    VLASwarmPromptResult parseVlaSwarmPromptResult(unsigned int prompt_id,
                                                   unsigned char expected_prompt_type);

    // data operations //
    unsigned int getCurPromptIdAndPlusOne(){std::lock_guard<std::mutex> lock(mutex_); return cur_prompt_id_++; }
    unsigned int getCurPromptId(){return cur_prompt_id_;}
    int getAreaFromPoly(const PolyHedronPtr& poly){return poly->area_id_;}
    bool needAreaPrediction(){ return !skeleton_gen_->area_handler_->areas_need_predict_.empty();}
    bool saveMap(const std::string& save_name = "");
    bool loadMap(const std::string& save_name);

    // visualization //
    void refreshLoadedMapVisualization();
    void visualizeSceneGraph();

private:
    friend class SceneGraphMapIO;
    ros::NodeHandle        nh_;
    ros::Publisher         scene_graph_pub_;
    std::mutex             mutex_;

    // 拓扑点不可达 相关 //
    ego_planner::MapInterface::Ptr map_interface_;       // 占据/可达查询接口
    std::vector<PolyHedronPtr>     last_poly_path_;      // 上次 getPathToObjectWithId 的多面体序列(供按 center 标记)
    std::vector<PolyHedronPtr>     blocked_list_;        // 当前被标记不可达的多面体(供 TTL 重校验/清除)
    bool   topo_block_enable_               = true;
    double topo_repair_radius_              = 0.5;
    int    topo_repair_vis_mode_            = 0;    // 0=isVisible拦截, 1=关闭isVisible, 2=isVisible+球交会中间点
    double topo_repair_vis_sphere_radius_   = 2.0;  // 模式2专用球半径
    int    topo_block_hits_thresh_          = 2;
    double topo_block_ttl_                = 8.0;
    bool   topo_block_revalidate_on_fail_ = true;
    int    topo_block_max_iter_            = 4;
    bool   topo_repair_insert_node_       = false;   // 修复点插入拓扑图: true=丢弃旧节点+生成新节点并连接; false=标记+TTL恢复

    // LLM interface //
    std::string            this_package_path_;
    ros::Publisher         prompt_pub_;
    ros::Subscriber        llm_ans_sub_;
    unsigned int           cur_prompt_id_ = 0;
    bool                   need_area_prediction_ = false;
    bool                   scene_graph_update_frozen_ = false;

    std::map<unsigned int, std::promise<std::string>> llm_ans_promises_;
    void llmAnsCallback(const scene_graph::PromptMsg::ConstPtr& msg);
};

/**
 * @brief 等待一个 std::future，在等待期间通过调用 ros::spinOnce() 来处理回调。
 * @tparam T future 的返回类型。
 * @param future 要等待的 future 对象。
 * @param timeout 等待的超时时间。
 * @return 如果在超时时间内成功收到结果，则返回 true；否则返回 false。
 */
template<typename T>
bool SceneGraph::waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout)
{
    ros::Time start_time = ros::Time::now();
    while (ros::ok())
    {
        if (ros::Time::now() - start_time > timeout)
        {
            return false; // 超时失败
        }
        // 2. 检查 future 是否就绪 (使用0秒等待实现非阻塞检查)
        auto status = future.wait_for(std::chrono::seconds(0));
        if (status == std::future_status::ready)
        {
            return true; // 成功
        }
        ros::spinOnce();
        ros::WallDuration(0.01).sleep(); // 休眠10毫秒
    }

    return false;
}

#endif //SCENE_GRAPH_H
