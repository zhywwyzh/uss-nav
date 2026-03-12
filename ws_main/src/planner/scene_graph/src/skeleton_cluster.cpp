//
// Created by gwq on 8/11/25.
//
#include "../include/scene_graph/skeleton_cluster.h"

void SpectralCluster::calculate(std::vector<PolyHedronPtr> &polys_without_gate,
                                std::vector<PolyhedronCluster> &clusters) {
    Eigen::MatrixXd W, ED, D, L, U;
    ros::Time t1 = ros::Time::now();

    calSimilarityMatrix(W, ED, polys_without_gate);
    calDegreeMatrix(W, D);
    calLaplacianMatrix(W, D, L);
    calLaplacianEigen(L, U);

    // K-Means 算法
    srand(time(nullptr));
    std::vector<int> cluster_assignments = kmeans(U, static_cast<int>(k_), 100);
    clusters.resize(k_);
    for (int i = 0; i < polys_without_gate.size(); i++)
        clusters[cluster_assignments[i]].addPoly(polys_without_gate[i], false);

    visualizeClusters(clusters);
}

void SpectralCluster::calSimilarityMatrix(Eigen::MatrixXd &W, Eigen::MatrixXd &ED,
                                          std::vector<PolyHedronPtr> polys_without_gate) {
    int poly_num = static_cast<int>(polys_without_gate.size());
    auto & polys = polys_without_gate;
    W = Eigen::MatrixXd::Zero(poly_num, poly_num);
    ED= Eigen::MatrixXd::Zero(poly_num, poly_num);

    std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus> poly_index_map;

    // 1. 计算每个点的局部 sigma
    int K = 7; // K近邻参数
    std::vector<double> local_sigmas(poly_num);
    for (int i = 0; i < poly_num; ++i) {
        std::vector<double> dists_to_i;
        for (int j = 0; j < poly_num; ++j) {
            if (i == j) continue;
            dists_to_i.push_back((polys[i]->center_ - polys[j]->center_).norm());
        }
        std::sort(dists_to_i.begin(), dists_to_i.end());
        if (dists_to_i.size() >= K) {
            local_sigmas[i] = dists_to_i[K - 1]; // 到第K个邻居的距离
        } else if (!dists_to_i.empty()) {
            local_sigmas[i] = dists_to_i.back(); // 如果点数不足K，取最远的
        } else {
            local_sigmas[i] = 1.0; // 孤立点
        }
    }

    // 2. 计算相似度矩阵 W
    for (int i = 0; i < polys.size(); i++) {
        poly_index_map[polys[i]->center_] = i;
    }
    for (int i = 0; i < polys.size(); i++) {
        for (const auto& nbr_poly : polys[i]->edges_) {
            if (nbr_poly.is_force_connected_) continue;
            int j = poly_index_map[nbr_poly.poly_nxt_->center_];
            if (abs(W(i, j)) < 1e-5 && polys[i] != polys[j]) {
                // TODO [gwq] 根据节点所包含的空间信息设计相似度计算函数
                double eula_dis = (polys[i]->center_ - polys[j]->center_).norm();
                double sigma_product = local_sigmas[i] * local_sigmas[j];
                if (sigma_product > 1e-6)
                    W(i, j) = W(j, i) = exp(-eula_dis * eula_dis / (2 * 0.1));  // 欧氏距离越大，相似度越低
            }
        }
        for (int j = i + 1; j < polys.size(); j++) {
            double dis = (polys[i]->center_, polys[j]->center_).norm();
            ED(i, j) = ED(j, i) = dis;
        }
    }
    INFO_MSG_GREEN("W Matrix: \n" << W);
}

void SpectralCluster::calDegreeMatrix(Eigen::MatrixXd &W, Eigen::MatrixXd &D) {
    const int n = static_cast<int>(W.rows());
    D = Eigen::MatrixXd::Zero(n, n);
    for (int i = 0; i < n; i++) {
        double sum = 0;
        for (int j = 0; j < n; j++) {
            sum += W(i, j);
        }
        D(i, i) = sum;
    }
}

void SpectralCluster::calLaplacianMatrix(Eigen::MatrixXd &W, Eigen::MatrixXd &D, Eigen::MatrixXd &L) {
    const int N = static_cast<int>(W.rows());
    Eigen::MatrixXd D_n = D;
    for(int i = 0; i < N; i++){
        if(abs(D_n(i,i)) < 1e-10) continue;
        D_n(i,i) = 1.0f / std::sqrt(D_n(i,i));
    }
    L = Eigen::MatrixXd::Identity(N, N) - D_n * W * D_n;
}

void SpectralCluster::calLaplacianEigen(Eigen::MatrixXd &L, Eigen::MatrixXd &U) {
    const int n = static_cast<int>(L.rows());
    if (n == 0) return;

    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(L);
    if (es.info() != Eigen::Success) {
        // 处理错误
        return;
    }
    Eigen::VectorXd eigen_values = es.eigenvalues(); // 已排序
    Eigen::MatrixXd eigen_vectors = es.eigenvectors(); // 与特征值对应

    /* 使用特征向量间隙算法计算 k */
    k_ = 0;
    double max_gap = 0.0;
    // 特征值数量小于2时，无法计算间隙，默认为1个簇
    if (n < 2) {
        k_ = n;
    } else {
        for(int i = 0; i < n - 1; i++){
            double gap = eigen_values(i+1) - eigen_values(i);
            if(gap > max_gap){
                max_gap = gap;
                k_ = i + 1;
            }
        }
    }
    if (k_ == 0 && n > 0) k_ = 1; // 至少保证有1个簇

    k_ = 5;

    INFO_MSG("   =======================================================");
    INFO_MSG("   [SpectralCluster] Eigen Gap algorithm suggests k = "<< k_);
    INFO_MSG("   =======================================================");

    // 直接选取前 k_ 个特征向量
    U = eigen_vectors.leftCols(k_);
    // 归一化 U 的行
    for (int i = 0; i < n; i++) {
        double norm = U.row(i).norm();
        if (norm > 1e-6) {
            U.row(i).normalize();
        } else {
            // 如果范数接近于0，则保持为0向量
            U.row(i) = Eigen::VectorXd::Zero(k_).transpose();
        }
    }
}

void SpectralCluster::visualizeClusters(const std::vector<PolyhedronCluster> &clusters) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker, marker2;
    marker.header.frame_id = marker2.header.frame_id = "world";
    marker.header.stamp = marker2.header.stamp = ros::Time::now();
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker.ns = "clusters_sphere";
    marker_array.markers.push_back(marker);
    marker.ns = "clusters_text";
    marker_array.markers.push_back(marker);

    int marker_index = 0;
    for (int i = 0; i < clusters.size(); i++) {
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = (rand() % 256) / 255.0;
        marker.color.g = (rand() % 256) / 255.0;
        marker.color.b = (rand() % 256) / 255.0;
        for (const auto& poly : clusters[i].polys_) {
            marker.ns = "clusters_text";
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.id = marker_index++;
            marker.pose.position.x = poly->center_.x() + 0.5;
            marker.pose.position.y = poly->center_.y() + 0.5 ;
            marker.pose.position.z = poly->center_.z() + 0.5;
            marker.text = std::to_string(i);
            marker_array.markers.push_back(marker);
        }

        // marker.ns = "clusters_sphere";
        // marker.type = visualization_msgs::Marker::SPHERE_LIST;
        // marker.id = i;
        // for (const auto& poly : clusters[i].cluster_) {
        //     geometry_msgs::Point p;
        //     p.x = poly->center_.x(), p.y = poly->center_.y(), p.z = poly->center_.z();
        //     marker.points.push_back(p);
        // }
        // marker_array.markers.push_back(marker);
    }
    cluster_vis_pub_.publish(marker_array);
}


// K-Means 算法的简单实现
// centroids: 初始质心
// points: N x k 的矩阵，每一行是一个点
// max_iter: 最大迭代次数
// 返回值: 一个包含 N 个元素的一维向量，每个元素是对应点的簇索引
std::vector<int> SpectralCluster::kmeans(const Eigen::MatrixXd& points, int k, int max_iter) {
    int num_points = points.rows();
    int num_dims = points.cols();

    // 随机初始化质心
    Eigen::MatrixXd centroids(k, num_dims);
    for (int i = 0; i < k; ++i) {
        centroids.row(i) = points.row(rand() % num_points);
    }

    std::vector<int> assignments(num_points);
    for (int iter = 0; iter < max_iter; ++iter) {
        bool changed = false;
        // 分配步骤：将每个点分配给最近的质心
        for (int i = 0; i < num_points; ++i) {
            double min_dist = -1.0;
            int best_cluster = 0;
            for (int j = 0; j < k; ++j) {
                double dist = (points.row(i) - centroids.row(j)).squaredNorm();
                if (min_dist < 0 || dist < min_dist) {
                    min_dist = dist;
                    best_cluster = j;
                }
            }
            if (assignments[i] != best_cluster) {
                assignments[i] = best_cluster;
                changed = true;
            }
        }

        // 如果没有点的分配发生变化，则收敛
        if (!changed) break;

        // 更新步骤：重新计算质心
        centroids.setZero();
        std::vector<int> counts(k, 0);
        for (int i = 0; i < num_points; ++i) {
            centroids.row(assignments[i]) += points.row(i);
            counts[assignments[i]]++;
        }
        for (int j = 0; j < k; ++j) {
            if (counts[j] > 0) {
                centroids.row(j) /= counts[j];
            } else { // 如果一个簇变空了，重新随机初始化
                centroids.row(j) = points.row(rand() % num_points);
            }
        }
    }
    return assignments;
}

void AreaHandler::getCurAreas(std::vector<PolyhedronCluster::Ptr>& clusters) {
    clusters.clear();
    for (auto& area : area_map_) clusters.push_back(area.second);
}

int AreaHandler::getAreaFromPoly(const PolyHedronPtr &poly) {
    if (poly == nullptr) {
        INFO_MSG_RED("[CommDetection]: [getAreaFromPoly] failed, poly == nullptr");
        return -1;
    }
    // todo [gwq] 可能需要判斷當前poly是否存在
    return poly_cluster_map_[poly->center_];
}

void AreaHandler::communityDetection(vector<PolyHedronPtr> &polys_all, std::unique_ptr<CPMVertexPartition>& partition_res, double resolution = 0.02) {
    auto check_igraph_error = [](igraph_error_t err, const std::string& function_name) {
        if (err != IGRAPH_SUCCESS) {
            // 如果函数返回的不是 IGRAPH_SUCCESS，就打印错误信息并终止程序
            std::cerr << "Fatal IGraph Error in function '" << function_name
                      << "': " << igraph_strerror(err) << std::endl;
            exit(1); // 立即退出，防止程序带着无效状态继续运行
        }
    };

    std::vector<PolyHedronPtr> polys;
    std::unordered_map<Eigen::Vector3d, int, Vector3dHash_SpecClus> poly_index_map;
    std::map<int, PolyHedronPtr> index_poly_map;

    int node_num = 0;
    for (auto & i : polys_all) {
        if (!i->is_gate_) {
            index_poly_map[node_num]   = i;
            poly_index_map[i->center_] = node_num++;
            polys.push_back(i);
        }else {
            INFO_MSG_RED("[CommDetection]: [communityDetection] gate poly ignored, poly_center = " << i->center_.transpose());
        }
    }
    // 2. ⭐ Grapg construction ⭐ //
    std::vector<igraph_integer_t> ig_edges_data;
    std::vector<double>           ig_weights_data;
    for (int i = 0; i < polys.size(); i++) {
        if (polys[i]->edges_.empty()) {
            INFO_MSG_RED("[CommDetection] | Error: Poly has no edges !!!");
            continue;
        }
        for (auto& edge : polys[i]->edges_) {
            if (poly_index_map.find(edge.poly_nxt_->center_) == poly_index_map.end()) continue;
            int j = poly_index_map[edge.poly_nxt_->center_];
            if (i >= j || polys[i] == polys[j]) continue;
            ig_edges_data.push_back(i);
            ig_edges_data.push_back(j);

            double weight = 1e-9;
            if (!edge.is_force_connected_) {
                // double eula_dis = (polys[i]->center_ - polys[j]->center_).norm();
                // weight   = std::exp(-1.0 * eula_dis * eula_dis / (2.0 * 2.0));
                weight = 1.0;
            }
            ig_weights_data.push_back(weight);
            // INFO_MSG("   * Edge[" << ig_edges_data.size() / 2 << "] node (" << i << "->" << j << "), weight=" << weight);
        }
    }
    INFO_MSG("[CommDetection] : poly num = " << polys.size() << ", edge num = " << ig_weights_data.size());

    igraph_t            ig_graph;
    igraph_vector_int_t ig_edges;
    igraph_vector_t     ig_weights;
    igraph_error_t      ig_err;

    igraph_vector_int_init(&ig_edges, ig_edges_data.size());
    for (size_t i = 0; i < ig_edges_data.size(); i++)
        VECTOR(ig_edges)[i] = ig_edges_data[i];

    igraph_set_attribute_table(&igraph_cattribute_table);
    igraph_empty(&ig_graph, 0, IGRAPH_UNDIRECTED);
    igraph_add_vertices(&ig_graph, node_num, nullptr);
    igraph_add_edges(&ig_graph, &ig_edges, nullptr);
    igraph_vector_int_destroy(&ig_edges);
    INFO_MSG_GREEN("[CommDetection] | graph construction start...(" << node_num << " nodes)");
    INFO_MSG("   | graph weights size Input : " << ig_weights_data.size());
    INFO_MSG("   | graph nodes size Output  : " << igraph_vcount(&ig_graph));
    INFO_MSG("   | graph edges size Output  : " << igraph_ecount(&ig_graph));

    igraph_attribute_combination_t comb;
    igraph_attribute_combination(&comb,
                             "weight", IGRAPH_ATTRIBUTE_COMBINE_SUM,
                             "",       IGRAPH_ATTRIBUTE_COMBINE_IGNORE,
                             IGRAPH_NO_MORE_ATTRIBUTES);
    // attach 'weight' to each edge !
    for (int i = 0; i < igraph_ecount(&ig_graph); i++) {
        // INFO_MSG("add weight: " <<i );
        ig_err = igraph_cattribute_EAN_set(&ig_graph, "weight", i, ig_weights_data[i]);
        check_igraph_error(ig_err, "igraph_cattribute_EAN_set");
    }
    // INFO_MSG_GREEN("[CommDetection] | Communities detection start ...");
    Graph* cd_graph = Graph::GraphFromEdgeWeights(&ig_graph, ig_weights_data);
    // Graph* cd_graph = new Graph(&ig_graph);
    // CPMVertexPartition partition(cd_graph, resolution /* resolution */);
    partition_res = std::make_unique<CPMVertexPartition>(cd_graph, resolution);
    Optimiser opt; opt.set_rng_seed(666);
    std::vector<bool> is_member_fixed(partition_res->get_graph()->vcount(), false);
    for (int i = 0; i < 10; i++) {
        double diff = opt.optimise_partition(partition_res.get(), is_member_fixed);
        // INFO_MSG("   | diff: " << diff);
        if (diff < 1e-4) break;
    }
    // INFO_MSG_GREEN("   | ** number of communities: " << partition_res->n_communities() << " **");
}

void AreaHandler::findCurAreaNbrs(int cur_area_id) {
    for (auto& area : area_map_) {
        if (area.second->nbr_area_.find(cur_area_id) != area.second->nbr_area_.end())
            area.second->nbr_area_.erase(cur_area_id);
    }

    if (area_map_.find(cur_area_id) != area_map_.end()) {
        PolyhedronCluster::Ptr cur_area = area_map_[cur_area_id];
        cur_area->nbr_area_.clear();
        for (const auto& poly : cur_area->polys_) {
            for (const auto& edge : poly->edges_) {
                if (edge.poly_nxt_->is_gate_ || edge.poly_nxt_->area_id_ ==  -1 ) continue;
                if (edge.poly_nxt_->area_id_ != cur_area_id) {
                    cur_area->nbr_area_[edge.poly_nxt_->area_id_] = true;
                    area_map_[edge.poly_nxt_->area_id_]->nbr_area_[cur_area_id] = true;
                }
            }
        }
        for (const auto& area_id : cur_area->nbr_area_) {
            INFO_MSG("   * Area [" << cur_area_id << "]'s nbr_area_ = " << area_id.first);
        }
    }
}

void AreaHandler::incrementalUpdateAreas(vector<PolyHedronPtr>& new_polys) {
    Eigen::Vector3d new_poly_center = Eigen::Vector3d::Zero();
    for (auto& poly : new_polys) {
        if (poly->is_gate_) {
            INFO_MSG_YELLOW("[skeletonClusterIncrementalUpdateAreas]: A gate, skip it");
            continue;
        }
        new_poly_center += poly->center_;
    }
    new_poly_center /= static_cast<double>(new_polys.size());

    INFO_MSG_GREEN("\n[AreaHandler] | Incremental Update Areas...");
    INFO_MSG_GREEN("              | old area num: " << area_map_.size());
    INFO_MSG_GREEN("              | new poly num: " << new_polys.size());

    if (new_polys.empty()) {
        ROS_ERROR("!!!!!!!!!! New Polyhedrons is Empty !!!!!!!!!!");
        return ;
    }

    if (!area_map_.empty()) {
        // ... [前面部分代码保持不变] ...
        // calculate old areas which connected by 'new_polys'
        map<int, bool> areas_to_update;
        for (const auto& poly : new_polys) {
            for (const auto& edge : poly->edges_) {
                if (edge.poly_nxt_->area_id_ != -1 && area_map_.find(edge.poly_nxt_->area_id_) != area_map_.end()) {
                    std::cout << "nxt_poly->area_id_ = " << edge.poly_nxt_->area_id_<< std::endl;
                    areas_to_update[edge.poly_nxt_->area_id_] = true;
                }
            }
        }
        if (areas_to_update.empty()) {
            ROS_ERROR("!!!!!!!!!! Area Found No Connection with OLD Areas !!!!!!!!!!");
            exit(1);
        }

        // [Log统一] 打印需要更新的区域信息
        for (const auto& area : areas_to_update) {
            INFO_MSG_GREEN("              | * Area [" << area.first << ", pos " << area_map_[area.first]->center_.transpose() << "] need update.");
        }

        // communityDetection
        int old_area_num_need_update = areas_to_update.size();
        vector<PolyHedronPtr> candidate_polys;
        std::unique_ptr<CPMVertexPartition> partition;
        for (auto& area : areas_to_update) {
            for (auto& poly : area_map_[area.first]->polys_)
                candidate_polys.push_back(poly);
        }
        candidate_polys.insert(candidate_polys.end(), new_polys.begin(), new_polys.end());

        // 为多面体指针创建到索引的反向映射，方便后续查找
        std::map<PolyHedronPtr, size_t> poly_to_idx_map;
        for(size_t i = 0; i < candidate_polys.size(); ++i) {
            poly_to_idx_map[candidate_polys[i]] = i;
        }

        communityDetection(candidate_polys, partition, 0.018);

        // ======================= 新增：预合并逻辑开始 =======================
        int n_communities = partition->n_communities();
        INFO_MSG_GREEN("   | Communities detection done, split into [" << n_communities << "] communities.");
        std::vector<std::vector<PolyHedronPtr>> new_poly_members;
        vector<vector<size_t>>                  partition_raw_members(n_communities);

        // 1. 构建多面体到社区ID的快速查找映射
        std::vector<int> poly_idx_to_community_id(candidate_polys.size(), -1);
        for (int i = 0; i < candidate_polys.size(); ++i) {
            int comm_id = partition->membership(i);
            if (comm_id < n_communities && comm_id >= 0) {
                partition_raw_members[comm_id].push_back(i);
                poly_idx_to_community_id[i] = comm_id;
            }else {
                ROS_ERROR("COMMUNITY ID ERROR! comm_id = %d, n_communities = %d, i = %d", comm_id, n_communities, i);
                ROS_ERROR_STREAM("poly_pos: " << candidate_polys[i]->center_.transpose());
                exit(1);
            }
        }

        // [Log统一] 打印每个社区的成员数量
        for (int i = 0; i< partition_raw_members.size(); i++) {
            INFO_MSG_GREEN("   | -- Initial Community [" << i << "] has " << partition_raw_members[i].size() << " members.");
        }

        if (n_communities > 1) {

            // 2. 统计每对社区间的连接边界上的多面体数量
            //    键是排过序的社区ID对，值是边界上多面体的索引集合
            std::map<std::pair<int, int>, std::set<size_t>> border_polys_count;
            for (size_t i = 0; i < candidate_polys.size(); ++i) {
                int community1_id = poly_idx_to_community_id[i];
                for (const auto& edge : candidate_polys[i]->edges_) {
                    auto it = poly_to_idx_map.find(edge.poly_nxt_);
                    if (it != poly_to_idx_map.end()) { // 确保邻居在候选集内
                        size_t neighbor_idx = it->second;
                        int community2_id = poly_idx_to_community_id[neighbor_idx];
                        if (community1_id != community2_id) {
                            int c1 = std::min(community1_id, community2_id);
                            int c2 = std::max(community1_id, community2_id);
                            border_polys_count[{c1, c2}].insert(i);
                            border_polys_count[{c1, c2}].insert(neighbor_idx);
                            // [Log统一] 打印详细的边界多面体位置，可用于调试
                            // INFO_MSG_GREEN("   | ---- Border poly contact: " << candidate_polys[i]->center_.transpose() << " <-> " << candidate_polys[neighbor_idx]->center_.transpose());
                        }
                    }
                }
            }

            // [Log统一] 打印社区间的边界多面体统计
            for (const auto& border : border_polys_count) {
                INFO_MSG_GREEN("   | -- Border between Community [" << std::get<0>(border.first) << "] and [" << std::get<1>(border.first) << "] has "
                                  << floor(static_cast<double>(border.second.size()) / 2.0) << " poly pairs.");
            }

            // 3. 并查集 (DSU) 用于管理合并
            struct DSU {
                std::vector<int> parent;
                DSU(int n) { parent.resize(n); std::iota(parent.begin(), parent.end(), 0); }
                int find(int i) { return parent[i] == i ? i : parent[i] = find(parent[i]); }
                void unite(int i, int j) { parent[find(i)] = find(j); }
            };
            DSU dsu(n_communities);

            // 4. 根据连接强度（边界多面体数量）决定是否合并
            const int MERGE_THRESHOLD = 4;
            for (const auto& pair : border_polys_count) {
                if (pair.second.size() > MERGE_THRESHOLD) {
                    dsu.unite(pair.first.first, pair.first.second);
                    INFO_MSG_GREEN("   | -- Merging community " << pair.first.first << " and " << pair.first.second
                                   << " (connection count: " << pair.second.size() << " > " << MERGE_THRESHOLD << ")");
                }
            }

            // 5. 生成最终合并后的社区
            std::map<int, vector<PolyHedronPtr>> final_communities_map;
            for (int i = 0; i < n_communities; ++i) {
                int root = dsu.find(i);
                for (const auto& member_idx : partition_raw_members[i]) {
                    final_communities_map[root].push_back(candidate_polys[member_idx]);
                }
            }
            // 用合并后的结果替换原始的社区成员
            new_poly_members.clear();
            for(const auto& pair : final_communities_map) {
                new_poly_members.push_back(pair.second);
            }
        } else {
            // 如果只有一个社区，则直接使用
            new_poly_members.resize(1);
            vector<size_t> members = partition_raw_members[0];
            for (auto& member : members) {
                new_poly_members[0].push_back(candidate_polys[member]);
            }
        }
        // ======================= 新增：预合并逻辑结束 =======================

        std::map<int, Eigen::Vector3d> old_area_centers, new_area_centers;

        // 使用合并前的旧区域中心
        for (const auto& area_id : areas_to_update)
            old_area_centers[area_id.first] = area_map_[area_id.first]->center_;

        // 使用合并后的新社区来计算中心
        for (int i = 0; i < new_poly_members.size(); i++) {
            Eigen::Vector3d center(0, 0, 0);
            for (const auto& poly : new_poly_members[i]) {
                center += poly->center_;
            }
            center /= static_cast<double>(new_poly_members[i].size());
            new_area_centers[i] = center;
        }

        std::map<int, int> match_res = Hungarian::HungarianMatcher::matchMeans(old_area_centers, new_area_centers);   // new to old

        INFO_MSG_GREEN("   | ** number of OLD Areas need update : " << old_area_num_need_update);
        INFO_MSG_GREEN("   | ** number of NEW Areas after merge : "<< new_poly_members.size());

        for (const auto& match : match_res)
            INFO_MSG_GREEN("   | ** match_res (old [" << match.second << "] -> new [" << match.first << "])");


        // Construct new areas or update old areas
        std::map<int, bool> area_need_find_nbrs;
        if (new_poly_members.size() == old_area_num_need_update) {
            INFO_MSG_YELLOW("   | ** same number of area, update directly ...");
            for (int i = 0; i < new_poly_members.size(); i++) {
                int match_id = match_res[i];
                PolyhedronCluster::Ptr old_area = area_map_[match_id];
                old_area->resetClusterWithPolys(new_poly_members[i]);
                areas_need_predict_[old_area->id_] = true;
                area_need_find_nbrs[old_area->id_] = true;
            }
        }else if (new_poly_members.size() > old_area_num_need_update) {
            INFO_MSG_YELLOW("   | ** need new areas, create new areas ...");
            for (int i = 0; i < new_poly_members.size(); i++) {
                auto it = match_res.find(i);
                if (it != match_res.end()) {
                    INFO_MSG_YELLOW("   | ** area [" << it->second << "] already exist, update it ...");
                    int match_id = it->second;
                    PolyhedronCluster::Ptr old_area = area_map_[match_id];
                    old_area->resetClusterWithPolys(new_poly_members[i]);
                    areas_need_predict_[old_area->id_] = true;
                    area_need_find_nbrs[old_area->id_] = true;
                }else {
                    INFO_MSG_YELLOW("   | ** area [" << max_area_id_<< "] not exist, create new area ...");
                    PolyhedronCluster::Ptr new_area = std::make_shared<PolyhedronCluster>();
                    new_area->id_ = max_area_id_ ++;
                    new_area->color_ = ColorGenerator::getColorById(new_area->id_);
                    new_area->resetClusterWithPolys(new_poly_members[i]);
                    area_map_[static_cast<unsigned int>(new_area->id_)] = new_area;
                    areas_need_predict_[new_area->id_] = true;
                    area_need_find_nbrs[new_area->id_] = true;
                }
            }
        }else {
            INFO_MSG_YELLOW("   | ** UPDATE matched areas, but need DELETE some old areas ...");
            std::map<int, bool> matched_areas;
            for (const auto& area : areas_to_update) matched_areas [area.first] = false;
            for (int i = 0; i < new_poly_members.size(); i++) {
                int match_id = match_res[i];
                matched_areas[match_id] = true;
                PolyhedronCluster::Ptr old_area = area_map_[match_id];
                old_area->resetClusterWithPolys(new_poly_members[i]);
                areas_need_predict_[old_area->id_] = true;
                area_need_find_nbrs[old_area->id_] = true;
            }
            /* delete some areas not matched */
            for (const auto& area_id : matched_areas) {
                if (!area_id.second) {
                    INFO_MSG_YELLOW("   | ** DELETE area [" << area_id.first << "] ...");
                    area_map_.erase(area_id.first);          // todo [gwq] 应该在外部接口中统一处理删除逻辑
                    areas_need_delete_[area_id.first] = true;
                }
            }
        }
        for (const auto& area_id : area_need_find_nbrs) {
            findCurAreaNbrs(area_id.first);
        }
    }else {
        // ... [首次初始化的代码保持不变] ...
        PolyhedronCluster::Ptr new_area = std::make_shared<PolyhedronCluster>();
        new_area->id_ = max_area_id_ ++;
        for (auto& poly : new_polys) new_area->addPoly(poly, true);
        area_map_[static_cast<unsigned int>(new_area->id_)] = new_area;
        areas_need_predict_[new_area->id_] = true;
        INFO_MSG_GREEN("[AreaHandler] | First Area Initialization...");
    }
    visualizeClusters();
}


void AreaHandler::visualizeClusters() {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker_text;
    marker_text.header.frame_id = "world";
    marker_text.header.stamp = ros::Time::now();
    marker_text.action = visualization_msgs::Marker::DELETEALL;
    marker_text.ns = "clusters_sphere";
    marker_array.markers.push_back(marker_text);
    // marker_text.ns = "clusters_text";
    // marker_array.markers.push_back(marker_text);
    marker_text.ns = "clusters_box";
    marker_array.markers.push_back(marker_text);
    // 随机生成颜色
    // std::vector<Eigen::Vector3d> colors(cur_clusters_.size());
    // for (int i = 0; i < cur_clusters_.size(); i++) {
    //     double hue = (double)i / cur_clusters_.size();
    //     double saturation = 1.0;
    //     double value = 1.0;
    //     double r = value * std::max(0.0, std::min(1.0, std::cos(hue * 2.0 * M_PI) * 0.5 + 0.5));
    //     double g = value * std::max(0.0, std::min(1.0, std::sin(hue * 2.0 * M_PI) * 0.5 + 0.5));
    //     double b = value * std::max(0.0, std::min(1.0, std::sin(hue * 2.0 * M_PI + M_PI / 3.0) * 0.5 + 0.5));
    //     cur_clusters_[i]->color_ = colors[i] = Eigen::Vector3d(r, g, b);
    // }

    // int marker_index = 0;
    // for (int i = 0; i < clusters.size(); i++) {
    //     marker_text.action = visualization_msgs::Marker::ADD;
    //     marker_text.scale.x = marker_text.scale.y = marker_text.scale.z = 1.0;
    //     marker_text.color.a = 1.0;
    //
    //     // 在HSV上采样颜色，保证颜色的鲜艳
    //     double hue = static_cast<double>(clusters.size());
    //     double saturation = 1.0;
    //     double value = 1.0;
    //     marker_text.color.r = colors[i].x();
    //     marker_text.color.g = colors[i].y();
    //     marker_text.color.b = colors[i].z();
    //     for (const auto& poly : clusters[i].cluster_) {
    //         marker_text.ns = "clusters_text";
    //         marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    //         marker_text.id = marker_index++;
    //         marker_text.pose.position.x = poly->center_.x() + 0.5;
    //         marker_text.pose.position.y = poly->center_.y() + 0.5;
    //         marker_text.pose.position.z = poly->center_.z() + 0.5;
    //         marker_text.text = std::to_string(i);
    //         marker_array.markers.push_back(marker_text);
    //     }
    // }

    visualization_msgs::Marker marker_sphere;
    marker_sphere.header.frame_id = "world";
    marker_sphere.header.stamp = marker_text.header.stamp;
    marker_sphere.action  = visualization_msgs::Marker::ADD;
    marker_sphere.ns      = "clusters_sphere";
    marker_sphere.type    = visualization_msgs::Marker::SPHERE_LIST;
    marker_sphere.scale.x = marker_sphere.scale.y = marker_sphere.scale.z = 0.25;
    marker_sphere.color.a = 1.0;
    // for (int i = 0; i < cur_clusters_.size(); i++) {
    //     marker_sphere.points.clear();
    //     marker_sphere.id = i;
    //     marker_sphere.color.r = colors[i].x();
    //     marker_sphere.color.g = colors[i].y();
    //     marker_sphere.color.b = colors[i].z();
    //
    //     for (auto& poly : cur_clusters_[i]->polys_) {
    //         geometry_msgs::Point p;
    //         p.x = poly->center_.x(), p.y = poly->center_.y(), p.z = poly->center_.z();
    //         marker_sphere.points.push_back(p);
    //     }
    //     marker_array.markers.push_back(marker_sphere);
    // }
    //
    // for (int i = 0; i < cur_clusters_.size(); i++) {
    //     visualization_msgs::Marker marker_box;
    //     drawBoundingBox(marker_box, cur_clusters_[i]->box_min_, cur_clusters_[i]->box_max_, i, colors[i], 0.04);
    //     marker_array.markers.push_back(marker_box);
    // }

    // std::vector<Eigen::Vector3d> colors(area_map_.size());
    // int i = 0;
    // for (const auto& area : area_map_) {
    //     double hue = (double)i / area_map_.size();
    //     double value = 1.0;
    //     double r = value * std::max(0.0, std::min(1.0, std::cos(hue * 2.0 * M_PI) * 0.5 + 0.5));
    //     double g = value * std::max(0.0, std::min(1.0, std::sin(hue * 2.0 * M_PI) * 0.5 + 0.5));
    //     double b = value * std::max(0.0, std::min(1.0, std::sin(hue * 2.0 * M_PI + M_PI / 3.0) * 0.5 + 0.5));
    //     area.second->color_ = colors[i] = Eigen::Vector3d(r, g, b);
    //     i++;
    // }

    for (const auto& area : area_map_) {
        marker_sphere.points.clear();
        marker_sphere.id = area.second->id_;
        marker_sphere.color.r = area.second->color_.x();
        marker_sphere.color.g = area.second->color_.y();
        marker_sphere.color.b = area.second->color_.z();
        for (auto& poly : area.second->polys_) {
            geometry_msgs::Point p;
            p.x = poly->center_.x(), p.y = poly->center_.y(), p.z = poly->center_.z();
            marker_sphere.points.push_back(p);
        }
        marker_array.markers.push_back(marker_sphere);
    }
    for (const auto& area : area_map_) {
        visualization_msgs::Marker marker_box;
        drawBoundingBox(marker_box, area.second->box_min_, area.second->box_max_, area.second->id_, area.second->color_, 0.04);
        marker_array.markers.push_back(marker_box);
    }

    cluster_vis_pub_.publish(marker_array);
    ros::Duration(0.001).sleep();
}

void AreaHandler::visualizeEdgeWeights(const std::vector<PolyHedronPtr> &polys, const std::vector<igraph_integer_t> &edges_data, const std::vector<double> &edge_weights) {
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "edge_weights";
    marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(marker);

    for (int i = 0; i < edge_weights.size(); i++) {
        int i_poly = edges_data[i*2];
        int j_poly = edges_data[i*2 + 1];
        double weight = edge_weights[i];
        if (weight < 1e-5) weight = 0;
        marker.action = visualization_msgs::Marker::ADD;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.id = i;
        marker.pose.position.x = (polys[i_poly]->center_ + polys[j_poly]->center_).x() / 2.0;
        marker.pose.position.y = (polys[i_poly]->center_ + polys[j_poly]->center_).y() / 2.0;
        marker.pose.position.z = (polys[i_poly]->center_ + polys[j_poly]->center_).z() / 2.0;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << weight;
        marker.text = oss.str();

        marker.scale.x = marker.scale.y = marker.scale.z = 0.3;
        marker.color.a = 1.0; marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0;
        marker_array.markers.push_back(marker);
    }
    edge_weight_vis_pub_.publish(marker_array);
    ros::Duration(0.001).sleep();
}

void AreaHandler::drawBoundingBox(visualization_msgs::Marker& marker, const Eigen::Vector3d& min, const Eigen::Vector3d& max,
                                      int id, const Eigen::Vector3d &color = {1.0f, 0.0f, 0.0f}, float line_width = 0.02) {
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "clusters_box";
    marker.id = id; // 如果需要多个框，应使用不同的ID
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;

    // 设置线宽
    marker.scale.x = line_width;

    // 设置颜色 (r, g, b, a)
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.color.a = 1.0; // 不透明

    // 定义长方体的8个顶点
    std::vector<geometry_msgs::Point> vertices;
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(min.x(), min.y(), min.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(max.x(), min.y(), min.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(max.x(), max.y(), min.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(min.x(), max.y(), min.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(min.x(), min.y(), max.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(max.x(), min.y(), max.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(max.x(), max.y(), max.z())));
    vertices.push_back(eigenToGeoPt(Eigen::Vector3d(min.x(), max.y(), max.z())));
    auto addLine = [](visualization_msgs::Marker& m, const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
        m.points.push_back(p1); m.points.push_back(p2);
    };
    // 定义12条边 (每条边由两个点组成)
    // 前面四条边
    addLine(marker, vertices[0], vertices[1]);
    addLine(marker, vertices[1], vertices[2]);
    addLine(marker, vertices[2], vertices[3]);
    addLine(marker, vertices[3], vertices[0]);

    // 后面四条边
    addLine(marker, vertices[4], vertices[5]);
    addLine(marker, vertices[5], vertices[6]);
    addLine(marker, vertices[6], vertices[7]);
    addLine(marker, vertices[7], vertices[4]);

    // 连接前后的四条边
    addLine(marker, vertices[0], vertices[4]);
    addLine(marker, vertices[1], vertices[5]);
    addLine(marker, vertices[2], vertices[6]);
    addLine(marker, vertices[3], vertices[7]);
}

geometry_msgs::Point AreaHandler::eigenToGeoPt(const Eigen::Vector3d &pt) {
    geometry_msgs::Point geo_pt;
    geo_pt.x = pt.x(); geo_pt.y = pt.y(); geo_pt.z = pt.z();
    return geo_pt;
}