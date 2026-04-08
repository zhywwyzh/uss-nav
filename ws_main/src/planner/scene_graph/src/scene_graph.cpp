//
// Created by gwq on 8/14/25.
//
#include "../include/scene_graph/scene_graph.h"
#include "quadrotor_msgs/Instruction.h"
#include "scene_graph/PromptMsg.h"
#include <json_fwd.hpp>
#include <map>
#include <ros/console.h>
#include <scene_graph/data_structure.h>
#include <scene_graph/skeleton_cluster.h>
#include <scene_graph/skeleton_generation.h>
#include <string>
#include <unordered_map>
#include <vector>
#include <zmqpp/message.hpp>

bool SceneGraph::initSceneGraph(const Eigen::Vector3d &cur_pos, double yaw) {
    if (skeleton_gen_->expandSkeleton(cur_pos, yaw)) {
        std::vector<PolyHedronPtr> poly_without_gate;
        for (auto poly : skeleton_gen_->cur_iter_polys_)
            if(!poly->is_gate_) poly_without_gate.push_back(poly);
        skeleton_gen_->area_handler_->incrementalUpdateAreas(poly_without_gate);
        skeleton_gen_->visualizePolyBelongsToArea();
        updateObjectToSceneGraph();
        INFO_MSG("Init SceneGraph Success!");
        return true;
    }else {
        ROS_ERROR("Init SceneGraph Error, Please Reboot!");
        exit(1);
    }
    return false;
}

void SceneGraph::updateSceneGraph(const Eigen::Vector3d &cur_pos, const double &yaw, bool &new_topo) {

    if (skeleton_gen_->doDenseCheckAndExpand(cur_pos, yaw)) {
        new_topo = true;
        std::vector<PolyHedronPtr> poly_without_gate;
        for (auto poly : skeleton_gen_->cur_iter_polys_)
            if(!poly->is_gate_) poly_without_gate.push_back(poly);
        skeleton_gen_->area_handler_->incrementalUpdateAreas(poly_without_gate);
        skeleton_gen_->visualizePolyBelongsToArea();
        updateObjectToSceneGraph();
        visualizeSceneGraph();
    }else {
        new_topo = false;
        updateObjectToSceneGraph();
    }
}

void SceneGraph::updateObjectToSceneGraph() {
    ros::Time t1 = ros::Time::now();
    object_factory_->lock();

    // todo [gwq] 物体挂载更新可以改成增量式的，但是我懒了，先这样吧 :D
    for (auto& area : skeleton_gen_->area_handler_->area_map_)
        area.second->clearObjs();

    for (auto obj : this->object_factory_->object_map_) {
        if (obj.second->edge.polyhedron_father == nullptr) continue;
        int area_id = obj.second->edge.polyhedron_father->area_id_;
        if (skeleton_gen_->area_handler_->area_map_.find(area_id) != skeleton_gen_->area_handler_->area_map_.end()) {
            skeleton_gen_->area_handler_->area_map_[area_id]->addObject(obj.second);
        }
    }
    
    need_area_prediction_ = false;
    for (auto& area : skeleton_gen_->area_handler_->area_map_) {
        int cur_obj_num = area.second->objects_.size();
        if ((area.second->last_obj_num_ >= 15 && abs(cur_obj_num - area.second->last_obj_num_) > 10) ||
            (area.second->last_obj_num_ >= 8 && area.second->last_obj_num_ < 15 && abs(cur_obj_num - area.second->last_obj_num_) > 3) ||
            (area.second->last_obj_num_ < 8 && area.second->last_obj_num_ != cur_obj_num)) {
            skeleton_gen_->area_handler_->areas_need_predict_[area.first] = true;
            need_area_prediction_ = true;
            INFO_MSG_YELLOW("[SceneGraph] [Update Obj] | Area " << area.first << " need predict!");
        }
    }
    object_factory_->unlock();
    // INFO_MSG("[SceneGraph] | Update object to scene graph time: " << (ros::Time::now() - t1).toSec() * 1000 << " ms");
}

// remember mount current poly cbefore call this function!
bool SceneGraph::getPathToObjectWithId(const int &id, std::vector<Eigen::Vector3d> &path, Eigen::Vector3d & aim_pos, double &aim_yaw) {

    if (!skeleton_gen_->ready()) {
        INFO_MSG_RED("[SceneGraph] | [Query Object Info] Scene Graph has not been initialized yet!");
        return false;
    }

    auto obj_map = object_factory_->object_map_;
    if (obj_map.find(id) == obj_map.end()) {
        INFO_MSG_RED("[SceneGraph] | [Query Object Info] Invalid object id (%d) for path searching.");
        return false;
    }
    ObjectNode::Ptr obj = obj_map.find(id)->second;

    if (cur_poly_ == nullptr || obj->edge.polyhedron_father == nullptr) {
        if (cur_poly_ == nullptr)
            INFO_MSG_RED("[SceneGraph] | [Query Object Info] UAV poly father is null, skip searching !");
        if (obj->edge.polyhedron_father == nullptr)
            INFO_MSG_RED("[SceneGraph] | [Query Object Info] Object poly father is null, skip searching !");
        return false;
    }

    double dis = skeleton_gen_->astarSearch(cur_poly_, obj->edge.polyhedron_father, path);
    aim_pos = obj->edge.polyhedron_father->center_;

    // 计算末端目标yaw
    Eigen::Vector3d dxy = obj->edge.polyhedron_father->center_ - obj->pos;
    double aim_direction_ = atan2(dxy(1), dxy(0)) + M_PI;
    if (aim_direction_ > M_PI)
        aim_direction_ -= 2 * M_PI;
    if (aim_direction_ < -M_PI)
        aim_direction_ += 2 * M_PI;
    aim_yaw = aim_direction_;
    return true;
}

bool SceneGraph::singleRoomPredictionPromptGen(const int room_id, nlohmann::json &prompt_json) {
    if (skeleton_gen_->area_handler_->area_map_.find(room_id) == skeleton_gen_->area_handler_->area_map_.end()) {
        ROS_ERROR("[SceneGraph] | Invalid room id (%d) for room prediction prompt generation.", room_id);
        return false;
    }
    auto double2String = [](double num) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << num;
        return ss.str();
    };

    auto& area = skeleton_gen_->area_handler_->area_map_[room_id];
    nlohmann::json new_area;
    new_area["id"] = std::to_string(area->id_);
    new_area["dimensions"] = {
        {"width", double2String(area->box_max_.x() - area->box_min_.x())},
        {"height", double2String((area->box_max_.y() - area->box_min_.y()))},
        {"unit", "meters"}
    };
    for (const auto& obj_raw : area->objects_) {
        nlohmann::json new_obj;
        std::string obj_info_str = obj_raw->label + ",";
        obj_info_str += double2String(obj_raw->pos.x()) + ",";
        obj_info_str += double2String(obj_raw->pos.y()) ;
        new_obj["info"] = obj_info_str;
        new_area["objects"].push_back(new_obj);
    }
    prompt_json["areas"].push_back(new_area);
    return true;
}

bool SceneGraph::newAreaPredictionPromptGen(std::string &prompt_str) {
    nlohmann::json data;
    for (const auto& area_id : skeleton_gen_->area_handler_->areas_need_predict_) {
        if (skeleton_gen_->area_handler_->area_map_.find(area_id.first) != skeleton_gen_->area_handler_->area_map_.end())
            singleRoomPredictionPromptGen(area_id.first, data);
        else
            skeleton_gen_->area_handler_->areas_need_predict_.erase(area_id.first);
    }
    skeleton_gen_->area_handler_->areas_need_predict_.clear();

    std::ofstream json_f_out(this_package_path_ + "/prompts_out/new_area_prediction.json");
    if (!json_f_out.is_open()) {
        INFO_MSG_RED("[SceneGraph] | Failed to open new area prediction prompt output file ...");
        return false;
    }else {
        json_f_out << data.dump(2) << std::endl;
        json_f_out.close();
    }
    prompt_str = data.dump();
    return true;
}

bool SceneGraph::allRoomPredictionPromptGen(std::string &prompt_str) {

    nlohmann::json data;
    // add data!
    for (auto& area : skeleton_gen_->area_handler_->area_map_) {
        singleRoomPredictionPromptGen(area.first, data);
    }
    std::ofstream json_f_out(this_package_path_ + "/prompts_out/room_prediction.json");
    if (!json_f_out.is_open()) {
        INFO_MSG_RED("[SceneGraph] | Failed to open room prediction prompt output file ...");
        return false;
    }else {
        json_f_out << data.dump() << std::endl;
        json_f_out.close();
    }
    prompt_str = data.dump();
    return false;
}

bool SceneGraph::chooseAreaToGoPromptGen(std::string &prompt_str) {
    auto double2String = [](double num) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << num;
        return ss.str();
    };
    nlohmann::json data;
    if (skeleton_gen_->area_handler_->area_map_.empty()) {
        prompt_str = "No room found!";
        return false;
    }

    for (auto& area_iter : skeleton_gen_->area_handler_->area_map_) {
        // if (area_iter.second->num_ftrs_ == 0 ) continue;
        auto area = area_iter.second;
        nlohmann::json area_json;
        area_json["id"]             = std::to_string(area->id_);
        area_json["label"]          = area->room_label_;
        area_json["description"]    = area->room_description_;
        area_json["unknownFrontierNum"] = area->num_ftrs_;
        area_json["neighborAreas"]  = {};
        for (const auto& neighbor_id : area->nbr_area_)
            area_json["neighborAreas"].push_back(std::to_string(neighbor_id.first));

        // size`: (Array of Floats) The dimensions `[length, width]` of the area in meters, from a top-down perspective.
        area_json["size"] = {double2String(area->box_max_.x() - area->box_min_.x()), double2String(area->box_max_.y() - area->box_min_.y())};
        data["areas"].push_back(area_json);
    }

    // 在data中加入历史area访问信息，格式为data["visitHistory"] = {}
    data["visitHistory"] = {};
    std::vector<int> tmp_history;
    for (auto& id : history_visited_area_ids_) {
        if (skeleton_gen_->area_handler_->area_map_.find(id) != skeleton_gen_->area_handler_->area_map_.end() &&
            skeleton_gen_->area_handler_->areas_need_delete_.find(id) == skeleton_gen_->area_handler_->areas_need_delete_.end()) {
            tmp_history.push_back(id);
            data["visitHistory"].push_back(std::to_string(id));
        }
    }
    history_visited_area_ids_ = tmp_history;
    skeleton_gen_->area_handler_->areas_need_delete_.clear();

    data["CurAreaId"] = std::to_string(cur_poly_->area_id_);
    data["Target"]         = target_cmd_string_;
    // data["PriorKnowledge"] = {};
    // data["PriorKnowledge"].push_back(prior_knowledge_string_);
    prompt_str = data.dump();
    return true;
}

bool SceneGraph::chooseTerminateObjIdPromptGen(std::string &prompt_str){
    auto double2String = [](double num) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << num;
        return ss.str();
    };
    
    nlohmann::json data;
    // get current area
    int cur_area_id = cur_poly_->area_id_;
    INFO_MSG("[SceneGraph] | Current area id for terminate object choosing: " << cur_area_id);
    if (skeleton_gen_->area_handler_->area_map_.find(cur_area_id) == skeleton_gen_->area_handler_->area_map_.end()) {
        ROS_ERROR("[SceneGraph] | Invalid current area id (%d) for terminate object choosing prompt generation.", cur_area_id);
        return false;
    }
    auto area = skeleton_gen_->area_handler_->area_map_[cur_area_id];
    for (const auto& obj_raw : area->objects_) {
        nlohmann::json obj;
        obj["id"]    =  std::to_string(obj_raw->id);
        obj["label"] =  obj_raw->label;
        obj["pos"]   = { double2String(obj_raw->pos.x()), double2String(obj_raw->pos.y()) };
        data["objects"].push_back(obj);
    }
    data["Target"] = target_cmd_string_;
    prompt_str = data.dump();
    return true;
}

bool SceneGraph::DFDemoPromptGen(std::string &prompt_str) {
    auto double2String = [](double num) {
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << num;
        return ss.str();
    };
    if (skeleton_gen_->area_handler_->area_map_.empty()) {
        prompt_str = "No room found!";
        return false;
    }

    nlohmann::json data;
    auto pushAreaInfoIntoArray = [&data, &double2String](const PolyhedronCluster::Ptr& area) {
        nlohmann::json area_json;
        area_json["id"] = std::to_string(area->id_);
        area_json["Area"] = area->room_label_.empty() ? "unknown" : area->room_label_;
        area_json["objects"] = nlohmann::json::array();

        for (auto& obj_raw : area->objects_) {
            nlohmann::json obj_json;
            obj_json["id"] = std::to_string(obj_raw->id);
            obj_json["label"] = obj_raw->label;
            obj_json["pos"] = { double2String(obj_raw->pos.x()), double2String(obj_raw->pos.y()) };
            area_json["objects"].push_back(obj_json);
        }

        data["areas"].push_back(area_json);
    };

    data["areas"] = nlohmann::json::array();
    for (auto& area_iter : skeleton_gen_->area_handler_->area_map_) {
        skeleton_gen_->area_handler_->areas_need_predict_[area_iter.first] = false;
        pushAreaInfoIntoArray(area_iter.second);
    }

    data["Target"] = target_cmd_string_;
    prompt_str = data.dump();
    return true;
}

void SceneGraph::sendSceneGraphJson(std::string &scene_graph_json_str){
    scene_graph::PromptMsg msg;
    msg.header.stamp = ros::Time::now();
    msg.prompt_type = scene_graph::PromptMsg::PROMPT_TYPE_SCENE_GRAPH_JSON;
    msg.prompt      = scene_graph_json_str;
    msg.option      = scene_graph::PromptMsg::SEND_PROMPT;
    msg.prompt_id   = -1;
    prompt_pub_.publish(msg);
    INFO_MSG_CYAN("[SceneGraph] | Published scene graph json to CoPaw, json size: " << scene_graph_json_str.size() << " bytes\n");
}

int SceneGraph::handelDFDemoResult(unsigned int prompt_id){
    if (llm_ans_str_poll_.find(prompt_id) == llm_ans_str_poll_.end()) {
        ROS_WARN("[SceneGraph] | No answer for prompt (id: %u) found. Ignoring result.", prompt_id);
        return -1;
    }
    if (llm_prompts_[prompt_id].prompt_type != scene_graph::PromptMsg::PROMPT_TYPE_DF_DEMO) {
        ROS_WARN("[SceneGraph] | Prompt (id: %u) is not df demo prompt. Ignoring result.", prompt_id);
        return -1;
    }
    nlohmann::json data;
    try{
        data = nlohmann::json::parse(llm_ans_str_poll_[prompt_id]);
        string res_str = data["Result"]["id"].front();
        int res = std::stoi(res_str);
        INFO_MSG_GREEN("[SceneGraph]: choose DF demo object id [" << res_str << "]");
        return res;
    }catch(const std::exception& e){
         ROS_ERROR("[SceneGraph] | Failed to parse df demo id answer: %s", e.what());
        return -1;
    }
}


int SceneGraph::handelTerminateObjIdResult(unsigned int prompt_id){
    if (llm_ans_str_poll_.find(prompt_id) == llm_ans_str_poll_.end()) {
        ROS_WARN("[SceneGraph] | No answer for prompt (id: %u) found. Ignoring result.", prompt_id);
        return -1;
    }
    if (llm_prompts_[prompt_id].prompt_type != scene_graph::PromptMsg::PROMPT_TYPE_TERMINATE_OBJ_ID) {
        ROS_WARN("[SceneGraph] | Prompt (id: %u) is not terminate id choose prompt. Ignoring result.", prompt_id);
        return -1;
    }
    nlohmann::json data;
    try{
        data = nlohmann::json::parse(llm_ans_str_poll_[prompt_id]);
        string res_str = data["Result"]["id"].front();
        int res = std::stoi(res_str);
        INFO_MSG_GREEN("[SceneGraph]: choose terminate object id [" << res_str << "]");
        return res;
    }catch(const std::exception& e){
        ROS_ERROR("[SceneGraph] | Failed to parse terminate object id answer: %s", e.what());
        return -1;
    }
}

int SceneGraph::handelExplorationResult(unsigned int prompt_id) {
    if (llm_ans_str_poll_.find(prompt_id) == llm_ans_str_poll_.end()) {
        ROS_WARN("[SceneGraph] | No answer for prompt (id: %u) found. Ignoring result.", prompt_id);
        return -1;
    }
    if (llm_prompts_[prompt_id].prompt_type != scene_graph::PromptMsg::PROMPT_TYPE_EXPL_PREDICTION) {
        ROS_WARN("[SceneGraph] | Prompt (id: %u) is not explore prompt. Ignoring result.", prompt_id);
        return -1;
    }
    nlohmann::json data;
    int res = -1;
    try {
        data = nlohmann::json::parse(llm_ans_str_poll_[prompt_id]);
        string res_str = data["Result"].front();
        string reason_str = data["Reason"].front();
        res = std::stoi(res_str);
        INFO_MSG_GREEN("[SceneGraph]: choose area [" << res_str << "]");
        INFO_MSG_GREEN("[SceneGraph]: area choose reason : \n" << reason_str);
        return res;
    }catch (const std::exception& e) {
        ROS_ERROR("[SceneGraph] | Failed to parse exploration answer: %s", e.what());
        return -1;
    }
}

void SceneGraph::handleRoomPredictionResult(const unsigned int prompt_id) {
    if (llm_ans_str_poll_.find(prompt_id) == llm_ans_str_poll_.end()) {
        ROS_WARN("[SceneGraph] | No answer for prompt (id: %u) found. Ignoring result.", prompt_id);
        return;
    }
    if (llm_prompts_[prompt_id].prompt_type != scene_graph::PromptMsg::PROMPT_TYPE_ROOM_PREDICTION) {
        ROS_WARN("[SceneGraph] | Prompt (id: %u) is not a room prediction prompt. Ignoring result.", prompt_id);
        return;
    }
    nlohmann::json data;
    try {
        data = nlohmann::json::parse(llm_ans_str_poll_[prompt_id]);
    }catch (const std::exception& e) {
        ROS_ERROR("[SceneGraph] | Failed to parse room prediction answer: %s", e.what());
        return;
    }

    INFO_MSG("** >>> room prediction answer <<< **");

    for (const auto& area_raw : data["results"]) {
        std::string area_id_str = area_raw["id"];
        int area_id = std::stoi(area_id_str);
        INFO_MSG("   * Analyze room prediction result [area id: " << area_id << "]");

        auto it = skeleton_gen_->area_handler_->area_map_.find(area_id);
        if (it != skeleton_gen_->area_handler_->area_map_.end()) {
            try{
                auto area = it->second;
                std::string area_label_str = area_raw["areaType"];
                std::string area_description_str = area_raw["description"];
                INFO_MSG("   * recv predict res [area id: " << area_id_str << "] label: " << area_label_str << " description: " << area_description_str);

                area->room_label_ = area_label_str;
                area->room_description_ = area_description_str;
            }catch (const std::exception& e) {
                ROS_ERROR("[SceneGraph] | Failed to parse room prediction result: %s", e.what());
            }
        }else {
            ROS_ERROR("[SceneGraph] | Invalid area id (%d) for room prediction result.", area_id);
        }
    }
    visualizeSceneGraph();
}

std::future<std::string> SceneGraph::sendPrompt(unsigned int prompt_id, unsigned char prompt_type,
                                                std::string prompt_str, const std::chrono::seconds& timeout, int max_retries)
{
    // 1. 创建最终要返回给调用者的 promise 和 future
    wait_recv_id_ = prompt_id;
    auto final_promise = std::make_shared<std::promise<std::string>>();
    std::future<std::string> final_future = final_promise->get_future();

    // 2. 创建一个后台线程来执行所有耗时操作
    std::thread worker_thread([this, prompt_id, prompt_type, prompt_str, timeout, max_retries, final_promise]() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (llm_ans_promises_.find(prompt_id) != llm_ans_promises_.end()) {
                ROS_WARN("[SceneGraph] | Prompt (id: %u) is already in progress. Ignoring new request.", prompt_id);
                // 用异常告知调用者请求未被处理
                final_promise->set_exception(std::make_exception_ptr(
                    std::runtime_error("Prompt ID " + std::to_string(prompt_id) + " is already in progress.")));
                return;
            }
        }

        if (llm_ans_str_poll_.find(prompt_id) != llm_ans_str_poll_.end())
            llm_ans_str_poll_.erase(prompt_id);

        for (int attempt = 1; attempt <= max_retries; ++attempt) {
            std::promise<std::string> attempt_promise;
            std::future<std::string> attempt_future = attempt_promise.get_future();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                llm_ans_promises_[prompt_id] = std::move(attempt_promise);
            }

            ROS_INFO("[SceneGraph] | Sending prompt (id: %u, attempt: %d/%d)...", prompt_id, attempt, max_retries);
            scene_graph::PromptMsg prompt_msg;
            prompt_msg.header.stamp = ros::Time::now();
            prompt_msg.prompt_id = prompt_id;
            prompt_msg.prompt_type = prompt_type;
            prompt_msg.prompt = prompt_str;
            prompt_msg.option = scene_graph::PromptMsg::SEND_PROMPT;
            llm_prompts_[prompt_id] = prompt_msg;
            prompt_pub_.publish(prompt_msg);

            auto status = attempt_future.wait_for(timeout);
            if (status == std::future_status::ready) {
                try {
                    std::lock_guard<std::mutex> lock(mutex_);
                    std::string answer_str = attempt_future.get();
                    final_promise->set_value(answer_str);            // 将最终结果放入final_promise
                    llm_ans_str_poll_[prompt_id] = answer_str;
                    INFO_MSG_GREEN("[SceneGraph] | Success! Received answer for prompt (id: " << prompt_id << ")");
                    // INFO_MSG_GREEN("[SceneGraph] | Answer: \n" << answer_str);
                    return; // 任务成功，退出线程
                } catch (const std::exception& e) {
                    final_promise->set_exception(std::current_exception());
                    ROS_ERROR("[SceneGraph] | Exception caught while getting future value: %s", e.what());
                    return; // 出现异常，退出线程
                }
            }

            ROS_WARN("[SceneGraph] | Timeout for prompt (id: %u, attempt: %d/%d).", prompt_id, attempt, max_retries);
            {
                // 清理本次失败的promise，为下次重试或最终失败做准备
                std::lock_guard<std::mutex> lock(mutex_);
                llm_ans_promises_.erase(prompt_id);
                llm_prompts_.erase(prompt_id);
            }
        }

        // 5. 所有重试都失败了
        ROS_ERROR("[SceneGraph] | All %d attempts failed for prompt (id: %u).", max_retries, prompt_id);
        final_promise->set_exception(std::make_exception_ptr(
            std::runtime_error("Request failed after " + std::to_string(max_retries) + " attempts.")));
    });
    worker_thread.detach();
    return final_future;
}

void SceneGraph::eraseLLMData(unsigned int prompt_id) {
    llm_ans_promises_.erase(prompt_id);
    llm_ans_str_poll_.erase(prompt_id);
    llm_prompts_.erase(prompt_id);
}

void SceneGraph::llmAnsCallback(const scene_graph::PromptMsg::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (llm_ans_promises_.find(msg->prompt_id) != llm_ans_promises_.end() && msg->option == scene_graph::PromptMsg::SEND_ANSWER) {
        INFO_MSG_GREEN("[SceneGraph] | *** Callback Recv llm answer (id: " << msg->prompt_id << ")");
        llm_ans_promises_[msg->prompt_id].set_value(msg->answer);
    }
}

void SceneGraph::mountCurPoly(const Eigen::Vector3d pos, const double yaw) {
    if (!skeleton_gen_->ready()) {
        INFO_MSG_RED("[SceneGraph] | Skeleton generator is not ready. Ignoring mount request.");
        return;
    }
    cur_poly_ = skeleton_gen_->mountCurTopoPoint(pos, true);
    if (cur_poly_ == nullptr) {
        INFO_MSG_RED("[SceneGraph] | mount cur_poly_ => nullptr, failed!!!");
        return;
    }
}

void SceneGraph::setTargetAndPriorKnowledge(const std::string &target_cmd_str, const std::string &prior_knowledge_str) {
    target_cmd_string_      = target_cmd_str;
    prior_knowledge_string_ = prior_knowledge_str;

    if (target_cmd_string_.empty())
        target_cmd_string_ = "No Command, Please Directly retrun -100";
    if (prior_knowledge_string_.empty())
        prior_knowledge_string_ = "";
}

void SceneGraph::refreshLoadedMapVisualization() {
    skeleton_gen_->refreshLoadedMapVisualization();
    object_factory_->visualizeResult(true);
    skeleton_gen_->area_handler_->visualizeClusters();
    visualizeSceneGraph();
}

void SceneGraph::visualizeSceneGraph() {

    if (!skeleton_gen_->ready() || !object_factory_->ok()) return;

    // parameters
    double top_level_vis_height  = 13.0;
    double room_level_vis_height = 7.0;
    double obj_level_vis_height  = 2.0;
    // data preparation
    Eigen::Vector3d top_vertex = Eigen::Vector3d::Zero();
    std::map<int, Eigen::Vector3d> room_level_colors;
    std::map<int, Eigen::Vector3d> room_level_vertices;
    std::map<int, std::vector<Eigen::Vector3d>> obj_level_vertices;

    skeleton_gen_->lock();

    for (const auto& area : skeleton_gen_->area_handler_->area_map_) {
        top_vertex += area.second->center_;
        room_level_vertices[area.second->id_] = (area.second->center_);
        room_level_colors[area.second->id_]   = (area.second->color_);
    }
    top_vertex /= skeleton_gen_->area_handler_->area_map_.size();
    skeleton_gen_->unlock();

    for (const auto& area_iter : skeleton_gen_->area_handler_->area_map_) {
        auto area = area_iter.second;
        for (const auto& obj : area->objects_) {
            obj_level_vertices[area->id_].push_back(obj->pos);
        }
    }

    // visualize
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_top_vertex;
    marker_top_vertex.header.frame_id = "world";
    marker_top_vertex.header.stamp = ros::Time::now();
    marker_top_vertex.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker_top_vertex);
    marker_top_vertex.ns = "scene_graph_top_level";
    marker_top_vertex.id = 0;
    marker_top_vertex.type = visualization_msgs::Marker::SPHERE;
    marker_top_vertex.action = visualization_msgs::Marker::ADD;
    marker_top_vertex.color.r = 1.0; marker_top_vertex.color.g = 1.0; marker_top_vertex.color.b = 1.0; marker_top_vertex.color.a = 1.0;
    marker_top_vertex.pose.position.x = top_vertex.x(); marker_top_vertex.pose.position.y = top_vertex.y(); marker_top_vertex.pose.position.z = top_level_vis_height;
    marker_top_vertex.pose.orientation.w = 1.0;
    marker_top_vertex.scale.x = marker_top_vertex.scale.y = marker_top_vertex.scale.z = 0.3;
    marker_array.markers.push_back(marker_top_vertex);

    visualization_msgs::Marker marker_room_level;
    marker_room_level.header.frame_id = "world";
    marker_room_level.header.stamp = marker_top_vertex.header.stamp;
    marker_room_level.ns = "scene_graph_room_level";
    marker_room_level.id = 0;
    marker_room_level.type = visualization_msgs::Marker::SPHERE_LIST;
    marker_room_level.action = visualization_msgs::Marker::ADD;
    marker_room_level.pose.orientation.w = 1.0;
    marker_room_level.scale.x = marker_room_level.scale.y = marker_room_level.scale.z = 0.3;
    for (const auto& area : skeleton_gen_->area_handler_->area_map_) {
        geometry_msgs::Point point;
        std_msgs::ColorRGBA color;
        int area_id = area.first;
        color.r = room_level_colors[area_id].x(); color.g = room_level_colors[area_id].y(); color.b = room_level_colors[area_id].z(); color.a = 1.0;
        point.x = room_level_vertices[area_id].x(); point.y = room_level_vertices[area_id].y(); point.z = room_level_vis_height;
        marker_room_level.points.push_back(point);
        marker_room_level.colors.push_back(color);
    }
    marker_array.markers.push_back(marker_room_level);

    visualization_msgs::Marker marker_room_top_edge;
    marker_room_top_edge.header.frame_id = "world";
    marker_room_top_edge.header.stamp = marker_top_vertex.header.stamp;
    marker_room_top_edge.ns     = "scene_graph_room_top_edge";
    marker_room_top_edge.type   = visualization_msgs::Marker::LINE_LIST;
    marker_room_top_edge.action = visualization_msgs::Marker::ADD;
    marker_room_top_edge.pose.orientation.w = 1.0;
    marker_room_top_edge.scale.x = marker_room_top_edge.scale.y = marker_room_top_edge.scale.z = 0.05;

    for (const auto& area : skeleton_gen_->area_handler_->area_map_) {
        geometry_msgs::Point point1, point2;
        std_msgs::ColorRGBA color;
        color.r = room_level_colors[area.first].x(); color.g = room_level_colors[area.first].y(); color.b = room_level_colors[area.first].z(); color.a = 1.0;
        point1.x = top_vertex.x(); point1.y = top_vertex.y(); point1.z = top_level_vis_height;
        point2.x = room_level_vertices[area.first].x(); point2.y = room_level_vertices[area.first].y(); point2.z = room_level_vis_height;
        marker_room_top_edge.points.push_back(point1);
        marker_room_top_edge.points.push_back(point2);
        marker_room_top_edge.colors.push_back(color);
        marker_room_top_edge.colors.push_back(color);
    }
    marker_array.markers.push_back(marker_room_top_edge);

    visualization_msgs::Marker marker_room_room_edge;
    marker_room_room_edge.header.frame_id = "world";
    marker_room_room_edge.header.stamp = marker_top_vertex.header.stamp;
    marker_room_room_edge.ns     = "scene_graph_room_room_edge";
    marker_room_room_edge.type   = visualization_msgs::Marker::LINE_LIST;
    marker_room_room_edge.action = visualization_msgs::Marker::ADD;
    marker_room_room_edge.pose.orientation.w = 1.0;
    marker_room_room_edge.scale.x = marker_room_room_edge.scale.y = marker_room_room_edge.scale.z = 0.05;
    marker_room_room_edge.color.r = 1.0; marker_room_room_edge.color.g = 1.0; marker_room_room_edge.color.b = 1.0; marker_room_room_edge.color.a = 1.0;
    std::map<std::pair<int, int>, bool> edge_exist;
    for (const auto& area : skeleton_gen_->area_handler_->area_map_) {
        for (const auto& edge : area.second->nbr_area_) {
            if (edge_exist.find(std::make_pair(min(area.first, edge.first), max(area.first, edge.first))) != edge_exist.end()) continue;
            if (skeleton_gen_->area_handler_->area_map_.find(edge.first) == skeleton_gen_->area_handler_->area_map_.end() &&
                skeleton_gen_->area_handler_->area_map_.find(edge.first) == skeleton_gen_->area_handler_->area_map_.end()) continue;

            edge_exist[std::make_pair(min(area.first, edge.first), max(area.first, edge.first))] = true;
            geometry_msgs::Point point1, point2;
            point1.x = room_level_vertices[area.first].x(); point1.y = room_level_vertices[area.first].y(); point1.z = room_level_vis_height;
            point2.x = room_level_vertices[edge.first].x(); point2.y = room_level_vertices[edge.first].y(); point2.z = room_level_vis_height;
            marker_room_room_edge.points.push_back(point1);
            marker_room_room_edge.points.push_back(point2);
        }
    }
    marker_array.markers.push_back(marker_room_room_edge);

    visualization_msgs::Marker marker_obj_level;
    marker_obj_level.header.frame_id = "world";
    marker_obj_level.header.stamp = marker_top_vertex.header.stamp;
    marker_obj_level.ns = "scene_graph_obj_level";
    marker_obj_level.id = 0;
    marker_obj_level.type = visualization_msgs::Marker::LINE_LIST;
    marker_obj_level.action = visualization_msgs::Marker::ADD;
    marker_obj_level.pose.orientation.w = 1.0;
    marker_obj_level.scale.x = marker_obj_level.scale.y = marker_obj_level.scale.z = 0.02;

    for (const auto& area_objs : obj_level_vertices) {
        int area_id = area_objs.first;
        geometry_msgs::Point point1, point2;
        point2.x = room_level_vertices[area_id].x(); point2.y = room_level_vertices[area_id].y(); point2.z = room_level_vis_height;
        for (const auto& obj_pos : area_objs.second) {
            std_msgs::ColorRGBA color;
            color.r = room_level_colors[area_id].x(); color.g = room_level_colors[area_id].y(); color.b = room_level_colors[area_id].z(); color.a = 1.0;
            point1.x = obj_pos.x(); point1.y = obj_pos.y(); point1.z = obj_pos.z();
            marker_obj_level.points.push_back(point1);
            marker_obj_level.points.push_back(point2);
            marker_obj_level.colors.push_back(color);
            marker_obj_level.colors.push_back(color);
        }
    }
    marker_array.markers.push_back(marker_obj_level);

    visualization_msgs::Marker marker_room_label_text;
    marker_room_label_text.header.frame_id = "world";
    marker_room_label_text.header.stamp = marker_top_vertex.header.stamp;
    marker_room_label_text.ns = "scene_graph_room_label_text";
    marker_room_label_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker_room_label_text.action = visualization_msgs::Marker::ADD;
    marker_room_label_text.pose.orientation.w = 1.0;
    marker_room_label_text.scale.x = marker_room_label_text.scale.y = marker_room_label_text.scale.z = 0.5;
    for (const auto& area : skeleton_gen_->area_handler_->area_map_) {
        std_msgs::ColorRGBA color;
        geometry_msgs::Point point;
        color.r = room_level_colors[area.first].x(); color.g = room_level_colors[area.first].y(); color.b = room_level_colors[area.first].z(); color.a = 1.0;
        point.x = room_level_vertices[area.first].x(); point.y = room_level_vertices[area.first].y(); point.z = room_level_vis_height + 0.5;
        marker_room_label_text.id = area.first;
        marker_room_label_text.pose.position = point;
        std::string label_text = "Area[" + std::to_string(area.first) + "]" + area.second->room_label_;
        marker_room_label_text.text = label_text;
        marker_room_label_text.color = color;
        marker_array.markers.push_back(marker_room_label_text);
    }
    scene_graph_pub_.publish(marker_array);
    ros::Duration(0.01).sleep();
}
