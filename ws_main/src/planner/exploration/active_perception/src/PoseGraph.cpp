#include <active_perception/PoseGraph.h>
/**
 * @brief Add bi-direction edge to the poseedge
 * 
 * @param inx_s edge start vertex inx
 * @param inx_e edge end vertex inx
 * @param dis euler distance
 */

void PoseGraph::addPoseEdge(int inx_s, int inx_e, double dis){
    addVertex(inx_s);
    addVertex(inx_e);
    pose_edge[inx_s].insert(Edge(inx_e, dis));
    pose_edge[inx_e].insert(Edge(inx_s, dis));
}

/**
 * @brief Please run renewKdtree() first!!!
 * Get the potential connectted vertex
 * (close in euler dis and far in sequential dis)
 * 
 * @param inx2 the query inx of self posegraph
 * @param res_set result map<inx,euler-distance>
 * @param connect_eulerdis_range close euler distance threshold
 * @param connect_seqendis_range far sequential distance threshold
 */
void PoseGraph::getPotenConnectSetSelf(int inx2, map<int, double>& res_set, double connect_eulerdis_range, double connect_seqendis_range){
    res_set.clear();
    pcl::PointXYZI cur = key_pose_list->points[inx2];
    Vector3d cur_p(cur.x, cur.y, cur.z);

    std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
    if ( kdtree_keypose->radiusSearch(cur, connect_eulerdis_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
            int inx1 = pointIdxRadiusSearch[k];
            pcl::PointXYZI endp = key_pose_list->points[inx1];
            Vector3d end_p(endp.x, endp.y, endp.z);

            double graph_dis = calSeqentialDis(inx1, inx2);
            if(graph_dis > connect_seqendis_range + 1e-3){
                res_set.insert({inx1, (cur_p-end_p).norm()});
            }

        }
    }
}

void PoseGraph::encodePosegraphData(quadrotor_msgs::MultiPoseGraph &msg) {
  // pack key nodes
  for (auto & iter : *key_pose_list){
    geometry_msgs::Point point_tmp;
    point_tmp.x = iter.x; point_tmp.y = iter.y; point_tmp.z = iter.z;
    msg.key_pose_list_xyz.push_back(point_tmp);
    msg.key_pose_list_intensity.push_back(iter.intensity);
  }
  // pack edges
  std_msgs::UInt16MultiArray p_end_tmp;
  std_msgs::Float32MultiArray weight_tmp;
  for (int i = 0; i < key_pose_list->size(); ++i) {
    p_end_tmp.data.clear();
    weight_tmp.data.clear();
    for (auto & edge : pose_edge[i]){
      p_end_tmp.data.push_back(edge.v_inx);
      weight_tmp.data.push_back(edge.weight);
    }
    msg.pose_edge_p_end.push_back(p_end_tmp);
    msg.pose_edge_weight.push_back(weight_tmp);
  }
}

void PoseGraph::getNearTopoNodesInRange(const Eigen::Vector3d &cur_pose, const double radius,
                                        map<double, int> &nearby_v_set) {
  nearby_v_set.clear();
  map<int, double> nearby_v_set_temp;
  pcl::PointXYZI cur;
  cur.x = cur_pose.x();
  cur.y = cur_pose.y();
  cur.z = cur_pose.z();
  getPotenConnectSetB(cur, nearby_v_set_temp, radius, 0.0);
  for (auto pair : nearby_v_set_temp)
  {
    pcl::PointXYZI  near_p = getCor(pair.first);
    Eigen::Vector3d near_pos(near_p.x, near_p.y, near_p.z);
    nearby_v_set.insert({pair.second, pair.first});
  }
}

/**
 * @brief Please run renewKdtree() first!!!
 * Get the potential connectted vertex
 * (close in euler dis and far in sequential dis)
 * 
 * @param cur the query point
 * @param res_set result map<inx,euler-distance>
 * @param connect_eulerdis_range close euler distance threshold
 * @param connect_seqendis_range far sequential distance threshold
 */
void PoseGraph::getPotenConnectSetB(pcl::PointXYZI cur, map<int, double>& res_set, double connect_eulerdis_range, double connect_seqendis_range){
    // cout<<"getPotenConnectSetB"<<endl;
    res_set.clear();
    if(kdtree_keypose->getInputCloud() == nullptr) return;
    Vector3d cur_p(cur.x, cur.y, cur.z);

    std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
    // cout<<"size: "<<kdtree_keypose->getInputCloud()->size()<<endl;
    if ( kdtree_keypose->radiusSearch(cur, connect_eulerdis_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
            int inx1 = pointIdxRadiusSearch[k];
            pcl::PointXYZI endp = key_pose_list->points[inx1];
            Vector3d end_p(endp.x, endp.y, endp.z);
            res_set.insert({inx1, (cur_p-end_p).norm()});
        }
    }
//    cout << "getPotenConnectSetB size: " << res_set.size() << " | " << kdtree_keypose->getInputCloud()->size() << endl;
}

double PoseGraph::calSeqentialDis(int inx1, int inx2){
    double dis_sum = 0;
    for(int i = min(inx1,inx2); i < max(inx1, inx2); i++){
        pcl::PointXYZI p1 = key_pose_list->points[i];
        pcl::PointXYZI p2 = key_pose_list->points[i+1];
        dis_sum += sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) +(p1.z-p2.z)*(p1.z-p2.z));
    }
    return dis_sum;
}

void PoseGraph::renewKdtree(int inx_thr){
    kdtree_keypose.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    cloud = key_pose_list->makeShared();
    cout<<key_pose_list->size()<<endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudnew;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    cout<<inx_thr<<", "<<cloud->size()<<endl;
    assert(inx_thr < cloud->size());
    for(size_t i = inx_thr; i < cloud->size(); i++){
        cloudnew->push_back(cloud->points[i]);
    }
    kdtree_keypose->setInputCloud(cloudnew);
}

void PoseGraph::deepCopy(PoseGraph::Ptr pg_tar, int copy_size){
    pg_tar->clear();
    ROS_ASSERT(getSize() >= copy_size);
    for(int i = 0; i < copy_size; i++){
        pg_tar->push_back(this->getCor(i));
    }
    pg_tar->pose_edge = this->pose_edge;
}

/**
 * @brief Copy the current posegraph to the target posegraph
 * @param pg_tar copy target
 * @param copy_size copy size of current posegraph
 */
void PoseGraph::deepCopy(PoseGraph& pg_tar, int copy_size){
    pg_tar.clear();
    // cout<<"copy size: "<<copy_size<<endl;
    ROS_ASSERT(getSize() >= copy_size);
    for(int i = 0; i < copy_size; i++){
        pg_tar.push_back(this->getCor(i));
    }
    pg_tar.pose_edge = this->pose_edge;
}


void PoseGraph::dispPoseEdges(){
    cout<<"PoseEdges: "<<endl;
    for(auto it = pose_edge.begin(); it != pose_edge.end(); it++){
        for(auto edge:it->second){
            cout<<it->first<<"-"<<edge.v_inx<<"("<<edge.weight<<"), ";
        }
    }
    cout<<endl;
}

int MultiPoseGraph::getEdgeSize(int ps, int pe){
    // map<int, set<Edge> > res;
    auto res = table.find(make_pair(ps, pe));
    if(res == table.end()) return 0;
    int s = 0;
    for(auto it = res->second.begin(); it!=res->second.end(); it++){
        s += it->second.size();
    }
    return s;
}

void MultiPoseGraph::clear(){
    posegraphes.clear();
    table.clear();
}

void MultiPoseGraph::dispEdgeTable(){
    int i = 0;
    for(auto it = table.begin(); it != table.end(); it++){
        PoseEdge& edges = it->second;
        for(auto iter = edges.begin(); iter!=edges.end(); iter++){
            set<Edge>& edge_set = iter->second;
            for(auto edge:edge_set){
                i++;
                cout<<"(<"<<it->first.first<<"->"<<it->first.second<<">:"<<iter->first<<"->"<<edge.v_inx<<"="<<edge.weight<<"); ";
            }
        }
    }
    cout<<endl;
    cout<<"Edge Num: "<<i<<endl;
}

void MultiPoseGraph::encodeMultiPoseGraphData(quadrotor_msgs::MultiPoseGraph &msg, const int pg_id) {
  ros::Time t1 = ros::Time::now();
  INFO_MSG_GREEN("[PoseGraph] Pack MultiPoseGraph data...");
  auto posegraph = posegraphes[pg_id];
  // pack key nodes
  for (auto & iter : *posegraph.key_pose_list){
    geometry_msgs::Point point_tmp;
    point_tmp.x = iter.x; point_tmp.y = iter.y; point_tmp.z = iter.z;
    msg.key_pose_list_xyz.push_back(point_tmp);
    msg.key_pose_list_intensity.push_back(iter.intensity);
  }
  // pack edges
  std_msgs::UInt16MultiArray p_end_tmp;
  std_msgs::Float32MultiArray weight_tmp;
  for (int i = 0; i < posegraph.key_pose_list->size(); ++i) {
    p_end_tmp.data.clear();
    weight_tmp.data.clear();
    for (auto & edge : posegraph.pose_edge[i]){
      p_end_tmp.data.push_back(edge.v_inx);
      weight_tmp.data.push_back(edge.weight);
    }
    msg.pose_edge_p_end.push_back(p_end_tmp);
    msg.pose_edge_weight.push_back(weight_tmp);
  }
  INFO_MSG_GREEN("[PoseGraph] Pack MultiPoseGraph data Done. time:" << (ros::Time::now() - t1).toSec() << "s");
}

void MultiPoseGraph::decodeMultiPoseGraphData(const quadrotor_msgs::MultiPoseGraph &msg, const int pose_graph_id) {
  ros::Time t1 = ros::Time::now();
  INFO_MSG_GREEN("[PoseGraph] Unpack MultiPoseGraph data...");
  PoseGraph posegraph_new;
  // unpack keyppoint
  for (int i = 0; i < msg.key_pose_list_xyz.size(); ++i){
    pcl::PointXYZI p;
    p.x = msg.key_pose_list_xyz[i].x; p.y = msg.key_pose_list_xyz[i].y; p.z = msg.key_pose_list_xyz[i].z;
    p.intensity = msg.key_pose_list_intensity[i];
    posegraph_new.key_pose_list->push_back(p);
    posegraph_new.addVertex(i);
  }
  // unpack edges
  for (int i = 0; i < msg.pose_edge_p_end.size(); ++i){
    for (int j = 0; j < msg.pose_edge_p_end[i].data.size(); ++j){
      posegraph_new.pose_edge[i].insert(Edge(msg.pose_edge_p_end[i].data[j], msg.pose_edge_weight[i].data[j]));
    }
  }
  posegraphes[pose_graph_id].clear();
  posegraphes[pose_graph_id] = posegraph_new;
  INFO_MSG_GREEN("[PoseGraph] Unpack MultiPoseGraph data Done. time:" << (ros::Time::now() - t1).toSec() << "s");
}

bool MultiPoseGraph::contain(pair<int, int> p, int inx) const{
    auto it = table.find(p);
    return it != table.end() && it->second.find(inx) != it->second.end();
}

bool MultiPoseGraph::containPg(int inx) const{
    auto it = posegraphes.find(inx);
    return it != posegraphes.end();
}

void MultiPoseGraph::addVertex(pair<int, int> p, int inx){
    auto it = table.find(p);
    if(it == table.end()){
        map<int, set<Edge>> ma;
        table[p] = ma;
    }
    else if(it->second.find(inx) == it->second.end())
    {   
      set<Edge> edge_list;
      table[p][inx] = edge_list;
    }
}

void MultiPoseGraph::addEdge(int ps, int pe, int inx_s, int inx_e, double dis){
    addVertex(make_pair(ps, pe), inx_s);
    addVertex(make_pair(pe, ps), inx_e);

    table[make_pair(ps, pe)][inx_s].insert(Edge(inx_e, dis));
    table[make_pair(pe, ps)][inx_e].insert(Edge(inx_s, dis));
}

void MultiPoseGraph::getEdge(int ps, int pe, map<int, set<Edge> >& res){
    res.clear();
    if(table.find(make_pair(ps, pe)) != table.end()){
        res = table[make_pair(ps, pe)];
    }
}

void MultiPoseGraph::getEdge(int ps, int pe, int vs, set<Edge>& res){
    res.clear();
    if(table.find(make_pair(ps, pe)) != table.end()){
        map<int, set<Edge> >& edge_map = table[make_pair(ps, pe)];
        auto it = edge_map.find(vs);
        if(it != edge_map.end()){
            res = it->second;
        }
    }
}


int MultiPoseGraph::getPoseGraphSize(int pg_id){
    auto it = posegraphes.find(pg_id);
    assert(it != posegraphes.end());
    return it->second.key_pose_list->size();
}

double MultiPoseGraph::calHeris(pair<int, int> cur, pair<int, int> aim){
    pcl::PointXYZI p1 = posegraphes[cur.first].getCor(cur.second);
    pcl::PointXYZI p2 = posegraphes[aim.first].getCor(aim.second);
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) +(p1.z-p2.z)*(p1.z-p2.z));
}

void MultiPoseGraph::getNeighbor(pair<int, int> p, multimap<double, pair<int,int>>& neighbor){
    neighbor.clear();
    int cur_pgid = p.first;
    int cur_vinx = p.second;
    // self edges
    // if(cur_vinx > 0){
    //     neighbor.insert({posegraphes[cur_pgid].calSeqentialDis(cur_vinx, cur_vinx-1), pair<int, int>(cur_pgid, cur_vinx-1)});
    // }
    // if(cur_vinx < getPoseGraphSize(cur_pgid)-1){
    //     neighbor.insert({posegraphes[cur_pgid].calSeqentialDis(cur_vinx, cur_vinx+1), pair<int, int>(cur_pgid, cur_vinx+1)});
    // }
    set<Edge> self_edges;
    posegraphes[cur_pgid].getEdge(cur_vinx, self_edges);
    for(auto edge:self_edges){
        neighbor.insert({edge.weight, pair<int, int>(cur_pgid, edge.v_inx)});
    }
    // edges between posegraphes
    for(auto it = posegraphes.begin(); it != posegraphes.end(); it++){
        if(it->first == cur_pgid) continue;
        set<Edge> edge_set;
        getEdge(cur_pgid, it->first, cur_vinx, edge_set);
        for(auto edge:edge_set){
            neighbor.insert({edge.weight, pair<int, int>(it->first, edge.v_inx)});    
        }
    }
}

double MultiPoseGraph::calGraphPath(pair<int, int> cur, pair<int, int> aim, vector<Vector3d>& path){
    vector<Vector4d> path4;
    double dis = calGraphPath(cur, aim, path4);
    for(auto p:path4){
        path.emplace_back(p(0),p(1),p(2));
    }
    return dis;
}

double MultiPoseGraph::calGraphPath(pair<int, int> cur, pair<int, int> aim, vector<Vector4d>& path){
    // cout<<"In MultiPoseGraph::calGraphPath"<<endl;
    // cout<<"keypose: "<<endl;
    // for(int i = 0; i < posegraphes[0].getSize(); i++){
    //     pcl::PointXYZI p_pcl = posegraphes[0].getCor(i);
    //     cout<<p_pcl.x<<", "<<p_pcl.y<<", "<<p_pcl.z<<"-"<<p_pcl.intensity<< endl;
    // }
    // cout<<"aim: ("<<aim.first<<", "<<aim.second<<")"<<endl;
    path.clear();
    if(cur == aim){
        pcl::PointXYZI p_pcl = posegraphes[aim.first].getCor(aim.second);
        path.emplace_back(p_pcl.x, p_pcl.y, p_pcl.z, p_pcl.intensity);
        return 0.0;
    }

    vector<int> size_table;
    int i = 0;
    for(auto it = posegraphes.begin(); it != posegraphes.end(); it++, i++){
        // cout<<"PG_"<<it->first<<endl;
        // it->second.dispPoseEdges();
        if(i == 0)
            size_table.push_back(0);
        else
            size_table.push_back(getPoseGraphSize(i-1)+size_table[i-1]);
    }
    size_table.push_back(getPoseGraphSize(i-1)+size_table[i-1]);
    // cout<<"size_table: ";
    // for(size_t i = 0; i < size_table.size(); i++){
    //     cout<<size_table[i]<<endl;
    // }
    // dispEdgeTable();


    auto pair2list = [size_table](pair<int, int> p) -> int{return size_table[p.first]+p.second;};

    vector<bool> is_in_close_list;
    vector<double> g_list;
    vector<pair<int, int>> parent_list;

    is_in_close_list.resize(size_table.back());
    g_list.resize(size_table.back());
    parent_list.resize(size_table.back());
    for(size_t i = 0; i < is_in_close_list.size(); i++){
        is_in_close_list[i] = false;
        g_list[i] = 9999999;
        parent_list[i] = pair<int, int>(-1,-1);
    }

    multimap<double, pair<int,int>> open_list;
    open_list.insert({calHeris(cur, aim), cur});
    g_list[pair2list(cur)] = 0;

    // loop
    bool success_flag = false;
    while(!open_list.empty()){
        multimap<double, pair<int,int>>::iterator it;
        it = open_list.begin();
        pair<int,int> inx_now = it->second;
        open_list.erase(it);
        is_in_close_list[pair2list(inx_now)] = true;
        // cout<<"current Node: <"<<inx_now.first<<","<<inx_now.second<<">"<<endl;
        // get aim
        if(inx_now == aim){
            success_flag = true;
            break;
        }

        multimap<double, pair<int,int>> neighbor;
        getNeighbor(inx_now, neighbor);
        // cout<<"Neighbor: "<<endl;
        for(multimap<double, pair<int,int>>::iterator it = neighbor.begin(); it != neighbor.end(); it++){
            pair<int,int> nei_inx = it->second;
            // cout<<"<"<<nei_inx.first<<","<<nei_inx.second<<">: ";

            if(is_in_close_list[pair2list(nei_inx)]){
                // cout<<"in close list"<<endl;
                continue;
            }
            if(g_list[pair2list(nei_inx)] == 9999999){
                g_list[pair2list(nei_inx)] = g_list[pair2list(nei_inx)] + it->first;
                open_list.insert({g_list[pair2list(nei_inx)]+calHeris(aim, nei_inx), nei_inx});
                parent_list[pair2list(nei_inx)] = inx_now;
                // cout<<"add in openlist"<<endl;
            }
            else if(g_list[pair2list(nei_inx)] > g_list[pair2list(nei_inx)] + it->first){
                double old_f = g_list[pair2list(nei_inx)] + calHeris(aim, nei_inx);
                auto it = open_list.find(old_f);
                bool deleteit = false;
                if(it != open_list.end()){
                    if(it->second == nei_inx){
                        open_list.erase(it);
                        deleteit = true;
                    }
                }
                if(!deleteit){
                    assert(false);
                }
                g_list[pair2list(nei_inx)] = g_list[pair2list(inx_now)] + it->first;
                open_list.insert({g_list[pair2list(nei_inx)]+calHeris(aim, nei_inx), nei_inx});
                parent_list[pair2list(nei_inx)] = inx_now;
                // cout<<"update in openlist"<<endl;
            }
        }
    }

    if(success_flag){
        // INFO_MSG("Graph A* search success");
        double dis_sum = 0.0;
        pair<int,int> inx_now = aim;

        // for(size_t i = 0; i < parent_list.size(); i++){
        //     cout<<parent_list[i].first<<"," <<parent_list[i].second <<endl;
        // }

        // path.push_back(aim_svp_p);
        while(inx_now != pair<int,int>(-1,-1)){
            // cout<<"inx: "<<inx_now.second<<endl;
            pcl::PointXYZI p_pcl = posegraphes[inx_now.first].getCor(inx_now.second);
            path.emplace_back(p_pcl.x, p_pcl.y, p_pcl.z, p_pcl.intensity);
            // cout<<path.back().transpose()<<" - ";
            if(path.size()>=2){
                dis_sum += (path.back().block(0,0,3,1) - path[path.size()-2].block(0,0,3,1) ).norm();
                // cout<<(path.back().block(0,0,3,1) - path[path.size()-2].block(0,0,3,1) ).norm()<<endl;
            }
            inx_now = parent_list[pair2list(inx_now)];
        }
        // path.push_back(drone_p);
        // dis_sum += (path.back().block(0,0,3,1) - path[path.size()-2].block(0,0,3,1)).norm();
        reverse(path.begin(), path.end());
        // INFO_MSG("Dis: "<<dis_sum);
        return dis_sum;
    }
    else
    {
        ROS_ERROR_THROTTLE(1.0,"Graph A* search ERROR");
//        throw "Graph A* search ERROR"; // todo [gwq] throw 会强制杀死程序，暂时删除
        return -1;
    }
}