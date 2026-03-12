//
// Created by gwq on 9/2/25.
//
#include <../include/active_perception/viewpoint_handler.h>

void ViewpointsHandler::calCurrentAreaCostMatrix(const PolyhedronCluster::Ptr &cluster, const Eigen::Vector3d &cur_pos,
                                            const double &cur_yaw, const Eigen::Vector3d & cur_vel, Eigen::MatrixXd &cost_mat) {
    ros::Time start_time = ros::Time::now();
    std::vector<Viewpoint::Ptr> vp_array;
    for (const auto& p : vps_map_) {
        if (scene_graph_->getAreaFromPoly(p.second->topo_father_) == cluster->id_)
            vp_array.push_back(p.second);
    }
    vps_candidate_ = vp_array;
    cur_yaw_ = cur_yaw;
    cur_vel_ = cur_vel;
    cur_pos_ = cur_pos;

    // matrix initialization
    unsigned int dim = vp_array.size();
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(dim + 2, dim + 2);
    mat.block(1, 0, dim, 1).setConstant(99999.0);      // vp -> cur_state max 不走回头路
    mat.block(dim + 1, 1, 1, dim).setConstant(99999.0);// home -> vp max      不从家离开
    mat(dim + 1, 0) = 0.0;                             // home -> cur_state free 免费传送门
    mat(0, dim + 1) = 99999.0;                         // 不允许直接回家

    // step1 vp<->vp cost
    for (int i = 0; i < dim; i++) {
        for (int j = i + 1; j < dim; j++) {
            mat(i + 1, j + 1) = mat(j + 1, i + 1) = vp_array[i]->costs_[vp_array[j]->id_];
        }
    }
    // step2 vp->cur_state cost
    for (int i = 0; i < dim; i++) {
        mat(0, i + 1) = calCurStateCost(cur_pos, cur_yaw, cur_vel, scene_graph_->getCurPoly(), vp_array[i]);
        mat(i + 1, dim + 1) = vp_array[i]->cost_to_home_;
    }
    cost_mat = mat;
    INFO_MSG_GREEN("[VP] | Current Area VP TSP matrix time : " << (ros::Time::now() - start_time).toSec() *1e3<< " ms");
    // INFO_MSG_GREEN("[VP] | Current Area VP TSP matrix:\n");
    // INFO_MSG_GREEN(mat);
}

double ViewpointsHandler::calCurStateCost(const Eigen::Vector3d cur_pos, const double cur_yaw,
                                          const Eigen::Vector3d &cur_vel, PolyHedronPtr cur_poly, Viewpoint::Ptr &vp) {
    double dis = -1.0f;
    std::vector<Eigen::Vector3d> path;
    if (vp->topo_father_ == nullptr) return 9999.0f;
    if (vp->topo_father_ == cur_poly || map_interface_->isVisible(vp->pos_, cur_pos, 0.0)) {
        if (!ego_planner::ViewNode::searchPath(cur_pos, vp->pos_, path, dis)) {
            path.clear();
            path.push_back(cur_pos), path.push_back(vp->pos_);
            dis = (cur_pos - vp->pos_).norm();
        }
    }else {
        getPathWithTopo(vp->topo_father_, vp->pos_, cur_poly, cur_pos, path, dis);
    }

    if (dis > 0.0f) {
        double yaw_diff  = fabs(vp->yaw_ - cur_yaw); yaw_diff = min(yaw_diff, 2.0 * M_PI - yaw_diff);
        double yaw_cost  = yaw_diff / ego_planner::ViewNode::yd_;
        double path_cost = dis / ego_planner::ViewNode::vm_;

        Eigen::Vector3d dir_2_vp = (vp->pos_ - cur_pos).normalized();
        if (cur_vel.norm() > 1e-3) {
            Eigen::Vector3d vidr = cur_vel.normalized();
            double diff          = acos(vidr.dot(dir_2_vp));
            path_cost           += ego_planner::ViewNode::w_dir_ * diff;
        }else {
            Eigen::Vector3d vdir(cos(cur_yaw), sin(cur_yaw), 0.0);
            double diff          = acos(vdir.dot(dir_2_vp));
            path_cost           += ego_planner::ViewNode::w_dir_ * diff;
        }
        double cost = max(path_cost, yaw_cost);
        return cost;
    }else
        return 9999.0f;
}

void ViewpointsHandler::updateCostMatrix(const PolyhedronCluster::Ptr& cluster) {
    ros::Time t1 = ros::Time::now();
    std::vector<Viewpoint::Ptr> vp_array;
    for (const auto& p : vps_map_) {
        if (scene_graph_->getAreaFromPoly(p.second->topo_father_) == cluster->id_)
            vp_array.push_back(p.second);
    }

    auto calculateCost = [this](const Viewpoint::Ptr& vp1, const Viewpoint::Ptr& vp2) {
        vector<Eigen::Vector3d> path;
        double dis            = -1.0f;
        bool   search_success = false;
        if (vp1->costs_.find(vp2->id_) != vp1->costs_.end()) return;

        if (vp1->topo_father_ == nullptr ||  vp2->topo_father_ == nullptr) {
            dis = 9999;
            path.push_back(vp1->pos_), path.push_back(vp2->pos_);
        }
        else if (vp1->topo_father_ == vp2->topo_father_) {
            search_success = ego_planner::ViewNode::searchPath(vp1->pos_, vp2->pos_, path, dis);
            if (dis < 0.0f || !search_success) {
                path.push_back(vp1->pos_), path.push_back(vp2->pos_);
                dis = (vp1->pos_ - vp2->pos_).norm();
            }
        }
        if (dis < 0.0f && map_interface_->isVisible(vp1->pos_, vp2->pos_, 0.0) /* && (vp1->pos_ - vp2->pos_).norm() < this->far_dis_thres_*/) {
            search_success = ego_planner::ViewNode::searchPath(vp1->pos_, vp2->pos_, path, dis);
        }
        if (dis < 0.0f || !search_success) {
            getPathWithTopo(vp1->topo_father_, vp1->pos_, vp2->topo_father_, vp2->pos_, path, dis);
        }

        double path_cost = dis / ego_planner::ViewNode::vm_;
        double yaw_diff  = fabs(vp1->yaw_ - vp2->yaw_); yaw_diff = min(yaw_diff, 2.0 * M_PI - yaw_diff);
        double yaw_cost  = yaw_diff / ego_planner::ViewNode::yd_;
        double cost      = (vp1->pos_ - vp2->pos_).norm() / ego_planner::ViewNode::vm_;

        vp1->costs_[vp2->id_] = cost;
        vp2->costs_[vp1->id_] = cost;
    };

    for (int i = 0; i < vp_array.size(); i++) {
        for (int j = i + 1; j < vp_array.size(); j++) {
            calculateCost(vp_array[i], vp_array[j]);
        }
    }

    INFO_MSG_GREEN("[VP] | Update cost matrix time: " << (ros::Time::now() - t1).toSec() *1e3<< " ms");
}

void ViewpointsHandler::getTopoPathToZeropoint(const Viewpoint::Ptr &vp) {
    vp->cost_to_home_ = -1.0f;

    if (vp->topo_father_ == nullptr)
        vp->cost_to_home_ = 9999.0f;
    else {
        std::vector<Eigen::Vector3d> path;
        vp->cost_to_home_ = scene_graph_->skeleton_gen_->astarSearch(vp->pos_, Eigen::Vector3d(0, 0, 0), path, true) / ego_planner::ViewNode::vm_;
        if (vp->cost_to_home_ < 0.0) vp->cost_to_home_ = (vp->pos_ - Eigen::Vector3d(0, 0, 0)).norm() / ego_planner::ViewNode::vm_;
    }
}

void ViewpointsHandler::getPathWithTopo(const PolyHedronPtr &start_poly, const Eigen::Vector3d &start_pose,
                                        const PolyHedronPtr &end_poly, const Eigen::Vector3d &end_pose,
                                        vector<Eigen::Vector3d> &path, double &dis) {
    path.clear();
    dis = scene_graph_->skeleton_gen_->astarSearch(start_poly, end_poly, path);
    if ((path.front() - start_pose).norm() > 0.1) path.insert(path.begin(), start_pose), dis += (start_pose - path.front()).norm();
    if ((path.back() - end_pose).norm() > 0.1) path.push_back(end_pose), dis += (end_pose - path.back()).norm();
}

void ViewpointsHandler::updateVPWellObserved(Eigen::Vector3d cur_pos, double yaw, double range) {
    VP_KDTree::PointVector vps_in_range;
    // ros::Time t1 = ros::Time::now();
    if (getViewpointInRange(cur_pos, range, vps_in_range)) {
        for (const auto & vp : vps_in_range) {
            if (abs(vp.vp_->yaw_ - yaw) < 30 / 180.0 * M_PI) {
                deleteViewpoint(vp.vp_->id_);
                INFO_MSG_YELLOW("[VP] | Delete viewpoint id: " << vp.vp_->id_ << " pos: " << vp.vp_->pos_.transpose());
            }
        }
    }
    // INFO_MSG_GREEN("[VP] | Update VP well observed time: " << (ros::Time::now() - t1).toSec() *1e3<< " ms");
}

void ViewpointsHandler::addViewpoint(Viewpoint vp, PolyHedronPtr cur_poly) {

    Viewpoint::Ptr vp_ptr = std::make_shared<Viewpoint>(vp);
    VP_KDTree::PointVector new_vps;
    new_vps.emplace_back(vp_ptr);

    // if (cur_poly == nullptr) std::cout << "12312312312321" << std::endl;

    // ros::Time t1 = ros::Time::now();
    if (kdtree_inited_) {
        // todo [gwq] 添加Viwpoint时候需要有一些逻辑判断
        bool need_add = true;
        VP_KDTree::PointVector vp_in_range;
        getViewpointInRange(vp.pos_, 2.0, vp_in_range);
        for (const auto p : vp_in_range) {
            if (abs(p.vp_->yaw_ - vp.yaw_) < 40 / 180.0 * M_PI) {
                need_add = false;
                break;
            }
        }
        if (need_add) {
            vp_ptr->id_          = cur_vp_id_++;
            vp_ptr->topo_father_ = cur_poly;
            getTopoPathToZeropoint(vp_ptr);

            vps_kdtree_->Add_Points(new_vps, false);
            vps_map_[vp_ptr->id_] = vp_ptr;
            INFO_MSG_GREEN("[VP] | Add viewpoint id: " << vp_ptr->id_ << " pos: " << vp_ptr->pos_.transpose());
            // INFO_MSG_GREEN("[VP] | Add viewpoint time: " << (ros::Time::now() - t1).toSec() *1e3<< " ms");
        }
    }
    if (vps_map_.empty() || !kdtree_inited_) {
        INFO_MSG_GREEN("[VP] | Start init KD-tree ");
        vps_kdtree_          = std::make_shared<VP_KDTree>();
        vp_ptr->id_          = cur_vp_id_ ++;
        vp_ptr->topo_father_ = cur_poly;
        getTopoPathToZeropoint(vp_ptr);
        vps_kdtree_->Build(new_vps);
        vps_map_[vp_ptr->id_] = vp_ptr;
        kdtree_inited_ = true;
        INFO_MSG_GREEN("[VP] | KD-tree inited ..... !");
    }

}

bool ViewpointsHandler::deleteViewpoint(unsigned int vp_id) {
    VP_KDTree::PointVector delete_vps;
    if (kdtree_inited_) {
        if (vps_kdtree_->size() == 1) {
            kdtree_inited_ = false;
            return true;
        }
        Eigen::Vector3d vp_pos = vps_map_[vp_id]->pos_;
        VP_KDTree::PointVector vps_nearest;
        if (getViewpointNearest(vp_pos, 1, vps_nearest)) {
            if ((vp_pos - vps_nearest.at(0).vp_->pos_).norm() > 0.1) return false;
            INFO_MSG_YELLOW("[VP] | Delete viewpoint id: " << vp_id << " pos: " << vp_pos.transpose());
            delete_vps.push_back(vps_nearest.at(0));
            vps_kdtree_->Delete_Points(delete_vps);
            vps_map_.erase(vp_id);
            return true;
        }else
            return false;
    } else {
        INFO_MSG_RED("[VP] | KD-tree not inited yet, skip delete viewpoint");
        return false;
    }
}


bool ViewpointsHandler::getViewpointInRange(const Eigen::Vector3d cur_pos, const double range,
                                            VP_KDTree::PointVector &vps_in_range) {
    if (kdtree_inited_) {
        auto sort_cmp = [cur_pos](const ftr_finder::ikdTree_VPType & p1,
                                 const ftr_finder::ikdTree_VPType & p2 ){
            return (p1.vp_->pos_ - cur_pos).norm() < (p2.vp_->pos_ - cur_pos).norm();
        };
        ftr_finder::ikdTree_VPType query_point(cur_pos);
        vps_kdtree_->Radius_Search(query_point, range, vps_in_range);
        if (vps_in_range.empty()) return false;
        std::sort(vps_in_range.begin(), vps_in_range.end(), sort_cmp);
        return true;
    }else {
        INFO_MSG_RED("[VP] | KD-tree not inited yet, skip search in range");
        return false;
    }
}

bool ViewpointsHandler::getViewpointNearest(Eigen::Vector3d cur_pos, int num, VP_KDTree::PointVector &vps_nearest) {
    if (kdtree_inited_) {
        auto sort_cmp = [cur_pos](const ftr_finder::ikdTree_VPType & p1,
                                 const ftr_finder::ikdTree_VPType & p2 ){
            return (p1.vp_->pos_ - cur_pos).norm() < (p2.vp_->pos_ - cur_pos).norm();
        };
        ftr_finder::ikdTree_VPType query_point(cur_pos);
        std::vector<float> dist_results;
        float max_dist;
        vps_kdtree_->Nearest_Search(query_point, num, vps_nearest, dist_results, max_dist);
        if (vps_nearest.empty()) return false;
        std::sort(vps_nearest.begin(), vps_nearest.end(), sort_cmp);
        return true;
    }else {
        INFO_MSG_RED("[VP] | KD-tree not inited yet, skip search nearest");
        return false;
    }
}



