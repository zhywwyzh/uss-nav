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

struct SceneGraph_Data {
std::vector<AreaHandler::Ptr> area;
};

class SceneGraph {
public:
    typedef std::shared_ptr<SceneGraph> Ptr;
    SceneGraph(ros::NodeHandle& nh, ego_planner::MapInterface::Ptr& map_interface) {
        nh_ = nh;
        scene_graph_pub_    = nh_.advertise<visualization_msgs::MarkerArray>("/scene_graph/vis", 2);
        prompt_pub_         = nh_.advertise<scene_graph::PromptMsg>("/scene_graph/prompt", 2);
        llm_ans_sub_        = nh_.subscribe("/scene_graph/llm_ans", 2, &SceneGraph::llmAnsCallback, this, ros::TransportHints().tcpNoDelay());
        skeleton_gen_       = std::make_shared<SkeletonGenerator>(nh, map_interface);
        object_factory_     = std::make_unique<ObjectFactory>(nh, skeleton_gen_);
        this_package_path_  = ros::package::getPath("scene_graph");
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

    // scene graph operations //
    bool initSceneGraph(const Eigen::Vector3d &cur_pos, double yaw);
    void updateSceneGraph(const Eigen::Vector3d &cur_pos, const double &yaw, bool &new_topo);
    void updateObjectToSceneGraph();
    bool getPathToObjectWithId(const int &id, std::vector<Eigen::Vector3d> &path, Eigen::Vector3d & aim_pos, double &aim_yaw);

    // LLM interface //
    std::map<unsigned int, std::string> llm_ans_str_poll_;
    std::map<unsigned int, scene_graph::PromptMsg> llm_prompts_;
    std::future<std::string> sendPrompt(unsigned int prompt_id, unsigned char prompt_type, std::string prompt_str,
                                        const std::chrono::seconds &timeout, int max_retries);
    int wait_recv_id_;

    template<typename T>
    bool waitForFutureWithSpinOnce(std::future<T>& future, const ros::Duration& timeout);

    // prompt generation //
    bool allRoomPredictionPromptGen(std::string &prompt_str);
    bool singleRoomPredictionPromptGen(const int room_id, nlohmann::json &prompt_json);
    bool newAreaPredictionPromptGen(std::string &prompt_str);
    bool chooseAreaToGoPromptGen(std::string &prompt_str);
    bool chooseTerminateObjIdPromptGen(std::string &prompt_str);
    bool DFDemoPromptGen(std::string &prompt_str);

    // result handle //
    void handleRoomPredictionResult(unsigned int prompt_id);
    int handelExplorationResult(unsigned int prompt_id);
    int handelTerminateObjIdResult(unsigned int prompt_id);
    int handelDFDemoResult(unsigned int prompt_id);

    // data operations //
    unsigned int getCurPromptIdAndPlusOne(){std::lock_guard<std::mutex> lock(mutex_); return cur_prompt_id_++; }
    unsigned int getCurPromptId(){return cur_prompt_id_;}
    int getAreaFromPoly(const PolyHedronPtr& poly){return poly->area_id_;}
    bool needAreaPrediction(){ return !skeleton_gen_->area_handler_->areas_need_predict_.empty();}

    // visualization //
    void visualizeSceneGraph();

private:
    ros::NodeHandle        nh_;
    ros::Publisher         scene_graph_pub_;
    std::mutex             mutex_;

    // LLM interface //
    std::string            this_package_path_;
    ros::Publisher         prompt_pub_;
    ros::Subscriber        llm_ans_sub_;
    unsigned int           cur_prompt_id_ = 0;
    bool                   need_area_prediction_ = false;

    std::map<unsigned int, std::promise<std::string>> llm_ans_promises_;
    void llmAnsCallback(const scene_graph::PromptMsg::ConstPtr& msg);
    void eraseLLMData(unsigned int prompt_id);
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
