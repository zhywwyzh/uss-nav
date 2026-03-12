// #include "path_searching/oc_a_star.h"

// namespace oc_a_star
// {

//     using namespace std;
//     using namespace Eigen;

//     AStar::~AStar()
//     {
//         for (int i = 0; i < POOL_SIZE_(0); i++)
//             for (int j = 0; j < POOL_SIZE_(1); j++)
//                 for (int k = 0; k < POOL_SIZE_(2); k++)
//                     delete GridNodeMap_[i][j][k];
//     }

//     void AStar::initAstar(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
//     {
//         POOL_SIZE_ = pool_size;
//         CENTER_IDX_ = pool_size / 2;

//         GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
//         for (int i = 0; i < POOL_SIZE_(0); i++)
//         {
//             GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
//             for (int j = 0; j < POOL_SIZE_(1); j++)
//             {
//                 GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
//                 for (int k = 0; k < POOL_SIZE_(2); k++)
//                 {
//                     GridNodeMap_[i][j][k] = new GridNode;
//                 }
//             }
//         }

//         grid_map_ = occ_map;
//     }

//     double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
//     {
//         double dx = abs(node1->index(0) - node2->index(0));
//         double dy = abs(node1->index(1) - node2->index(1));
//         double dz = abs(node1->index(2) - node2->index(2));

//         double h = 0.0;
//         int diag = min(min(dx, dy), dz);
//         dx -= diag;
//         dy -= diag;
//         dz -= diag;

//         if (dx == 0)
//         {
//             h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
//         }
//         if (dy == 0)
//         {
//             h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
//         }
//         if (dz == 0)
//         {
//             h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
//         }
//         return h;
//     }

//     double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
//     {
//         double dx = abs(node1->index(0) - node2->index(0));
//         double dy = abs(node1->index(1) - node2->index(1));
//         double dz = abs(node1->index(2) - node2->index(2));

//         return dx + dy + dz;
//     }

//     double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
//     {
//         return (node2->index - node1->index).norm();
//     }

//     vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
//     {
//         vector<GridNodePtr> path;
//         path.push_back(current);
//         cout << Index2Coord(current->index).transpose() << " \t\t " << getDistance(Index2Coord(current->index)) << " fScore=" << current->fScore << endl;

//         while (current->cameFrom != NULL)
//         {
//             current = current->cameFrom;
//             path.push_back(current);
//             cout << Index2Coord(current->index).transpose() << " \t\t  " << getDistance(Index2Coord(current->index)) << " fScore=" << current->fScore << endl;
//         }

//         return path;
//     }

//     bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
//     {
//         if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
//             return false;

//         return true; // debug

//         if (checkOccupancy(Index2Coord(start_idx)))
//         {
//             // ROS_WARN("Start point is insdide an obstacle.");
//             std::vector<Eigen::Vector3i> free_idx;
//             for (int x = -1; x <= 1; ++x)
//                 for (int y = -1; y <= 1; ++y)
//                     for (int z = -1; z <= 1; ++z)
//                     {
//                         Eigen::Vector3i chk_pt = start_idx + Eigen::Vector3i(x, y, z);
//                         if (chk_pt(0) >= 0 && chk_pt(0) < POOL_SIZE_(0) && chk_pt(1) >= 0 && chk_pt(1) < POOL_SIZE_(1) && chk_pt(2) >= 0 && chk_pt(2) < POOL_SIZE_(2))
//                             if (!checkOccupancy(Index2Coord(chk_pt)))
//                                 free_idx.push_back(chk_pt);
//                     }

//             double min_dist = 999999;
//             Eigen::Vector3d best_sp;
//             for (auto it : free_idx)
//             {
//                 double dist = (Index2Coord(it) - start_pt).norm();
//                 if (dist < min_dist)
//                 {
//                     min_dist = dist;
//                     best_sp = Index2Coord(it);
//                 }
//             }

//             if (min_dist > 0)
//                 return false;

//             start_pt = best_sp;
//         }

//         if (checkOccupancy(Index2Coord(end_idx)))
//         {
//             // ROS_WARN("End point is insdide an obstacle.");
//             std::vector<Eigen::Vector3i> free_idx;
//             for (int x = -1; x <= 1; ++x)
//                 for (int y = -1; y <= 1; ++y)
//                     for (int z = -1; z <= 1; ++z)
//                     {
//                         Eigen::Vector3i chk_pt = end_idx + Eigen::Vector3i(x, y, z);
//                         if (chk_pt(0) >= 0 && chk_pt(0) < POOL_SIZE_(0) && chk_pt(1) >= 0 && chk_pt(1) < POOL_SIZE_(1) && chk_pt(2) >= 0 && chk_pt(2) < POOL_SIZE_(2))
//                             if (!checkOccupancy(Index2Coord(chk_pt)))
//                                 free_idx.push_back(chk_pt);
//                     }

//             double min_dist = 999999;
//             Eigen::Vector3d best_sp;
//             for (auto it : free_idx)
//             {
//                 double dist = (Index2Coord(it) - end_pt).norm();
//                 if (dist < min_dist)
//                 {
//                     min_dist = dist;
//                     best_sp = Index2Coord(it);
//                 }
//             }

//             if (min_dist > 999998)
//                 return false;

//             end_pt = best_sp;
//         }

//         if (Coord2Index(start_pt, start_idx) && Coord2Index(end_pt, end_idx))
//         {
//             return true;
//         }

//         return false;
//     }

//     ASTAR_RET AStar::AstarSearch(const double radius,
//                                  const double min_dist,
//                                  Eigen::Vector3d start_pt,
//                                  Eigen::Vector3d end_pt,
//                                  std::vector<Eigen::Vector3d> &histroy)
//     {
//         ros::Time t0 = ros::Time::now();
//         if ((start_pt - end_pt).squaredNorm() < 1e-6) // <1mm
//         {
//             ROS_ERROR("start_pt=end_pt=%f, %f, %f, return!", end_pt(0), end_pt(1), end_pt(2));
//             return ASTAR_RET::INIT_ERR;
//         }

//         ros::Time time_1 = ros::Time::now();
//         if (++rounds_ == std::numeric_limits<unsigned int>::max())
//             for (int i = 0; i < POOL_SIZE_(0); i++)
//                 for (int j = 0; j < POOL_SIZE_(1); j++)
//                     for (int k = 0; k < POOL_SIZE_(2); k++)
//                         GridNodeMap_[i][j][k]->rounds = 0; // clear state

//         step_size_ = grid_map_->getResolution();
//         inv_step_size_ = 1 / step_size_;
//         center_ = (start_pt + end_pt) / 2;

//         Vector3i start_idx, end_idx;
//         if (!ConvertToIndexAndAdjustStartEndPoints(start_pt, end_pt, start_idx, end_idx))
//         {
//             ROS_ERROR("Unable to handle the initial or end point, force return!");
//             return ASTAR_RET::INIT_ERR;
//         }

//         // if (getDistance(Index2Coord(start_idx)) + 1e-5 >= GRID_MAP_OUTOFREGION_FLAG || getDistance(Index2Coord(end_idx)) + 1e-5 >= GRID_MAP_OUTOFREGION_FLAG)
//         // {
//         //     ROS_ERROR("Start or end point out of the esdf region!");
//         //     return ASTAR_RET::INIT_ERR;
//         // }
//         // if ( start_pt(0) > -1 && start_pt(0) < 0 )
//         //     cout << "start_pt=" << start_pt.transpose() << " end_pt=" << end_pt.transpose() << endl;

//         GridNodePtr startPtr = GridNodeMap_[start_idx(0)][start_idx(1)][start_idx(2)];
//         GridNodePtr endPtr = GridNodeMap_[end_idx(0)][end_idx(1)][end_idx(2)];

//         if ((start_pt - end_pt).squaredNorm() < 1e-6) // <1mm
//         {
//             ROS_ERROR("start_pt=end_pt=%f, %f, %f, return! I should handle it one day", end_pt(0), end_pt(1), end_pt(2));
//             while (ros::ok())
//                 ;
//             return ASTAR_RET::INIT_ERR;
//         }
//         Eigen::Vector3d s2e_vec = (end_pt - start_pt).normalized();
//         double s2e_norm = (end_pt - start_pt).norm();
//         double radius2 = radius * radius;
//         double thres_dist = min(getDistance(Index2Coord(start_idx)), getDistance(Index2Coord(end_idx)));
//         double end_dist = 1e-5;

//         std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> empty;
//         openSet_.swap(empty);

//         GridNodePtr neighborPtr = NULL;
//         GridNodePtr current = NULL;

//         endPtr->index = end_idx;

//         startPtr->index = start_idx;
//         startPtr->rounds = rounds_;
//         // double dist_pen = getHeu(startPtr, endPtr) / step_size_ * 2.0; // multiply by 2.0 to increase defferentitation
//         double dist_pen = 1; // multiply by 2.0 to increase defferentitation
//         startPtr->gScore = dist_pen * (1 / (getDistance(Index2Coord(start_idx)) + 1e-3));
//         startPtr->fScore = getHeu(startPtr, endPtr) + dist_pen * (1 / (getDistance(Index2Coord(start_idx)) + 1e-3));
//         startPtr->state = GridNode::OPENSET; // put start node in open set
//         startPtr->cameFrom = NULL;
//         openSet_.push(startPtr); // put start in open set

//         double tentative_gScore;

//         int closed_set_count = 0;

//         int num_iter = 0;
//         const vector<Eigen::Vector3i> dir{Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 1, 0),
//                                           Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)};
//         while (!openSet_.empty())
//         {
//             num_iter++;
//             current = openSet_.top();
//             histroy.push_back(Index2Coord(current->index));
//             openSet_.pop();
//             double dist = getDistance(Index2Coord(current->index));
//             int region = max((int)ceil(dist / 2.0 / grid_map_->getResolution()), 1);

//             // if ( num_iter < 10000 )
//             //     cout << "current=" << current->index.transpose() << endl;

//             if (Eigen::Vector3i(current->index(0) - endPtr->index(0), current->index(1) - endPtr->index(1), current->index(2) - endPtr->index(2)).norm() <= region)
//             {
//                 // ros::Time time_2 = ros::Time::now();
//                 // printf("\033[34mA star iter:%d, time:%.3f\033[0m\n",num_iter, (time_2 - time_1).toSec()*1000);
//                 // if((time_2 - time_1).toSec() > 0.1)
//                 //     ROS_WARN("Time consume in A star path finding is %f", (time_2 - time_1).toSec() );
//                 gridPath_ = retrievePath(current);
//                 ros::Time t1 = ros::Time::now();
//                 ROS_INFO("A* time=%f, clo_num=%d, open_num=%ld", (t1 - t0).toSec() * 1000, closed_set_count, openSet_.size());
//                 return ASTAR_RET::SUCCESS;
//             }
//             current->state = GridNode::CLOSEDSET; // move current node from open set to closed set.
//             closed_set_count++;

//             for (int z = -region; z <= region; ++z)
//                 for (int y = -region; y <= +region; ++y)
//                 {
//                     if (region * region - z * z - y * y >= 0)
//                     {
//                         double dx = sqrt(region * region - z * z - y * y);
//                         int inx = round(current->index(0) - dx);
//                         int outx = round(current->index(0) + dx);
//                         int zg = z + current->index(2);
//                         int yg = y + current->index(1);

//                         /******************/
//                         if (inx >= 1 && inx < POOL_SIZE_(0) - 1 && yg >= 1 && yg < POOL_SIZE_(1) - 1 && zg >= 1 && zg < POOL_SIZE_(2) - 1)
//                         {
//                             neighborPtr = GridNodeMap_[inx][yg][zg];
//                             neighborPtr->index = Eigen::Vector3i(inx, yg, zg);

//                             bool flag_explored = neighborPtr->rounds == rounds_;

//                             if (!flag_explored || neighborPtr->state != GridNode::CLOSEDSET)
//                             {

//                                 Eigen::Vector3d pt = Index2Coord(neighborPtr->index);

//                                 double proj = (pt - start_pt).dot(s2e_vec);
//                                 if (proj <= s2e_norm && (pt - start_pt).squaredNorm() - proj * proj <= radius2)
//                                 {

//                                     neighborPtr->rounds = rounds_;

//                                     double dist = getDistance(pt);
//                                     // if (dist < min_dist)
//                                     // {
//                                     //     continue;
//                                     // }

//                                     double static_cost = (double)region;
//                                     tentative_gScore = current->gScore + static_cost + dist_pen * (1 / (dist + 1e-3));

//                                     if (!flag_explored)
//                                     {
//                                         // discover a new node
//                                         neighborPtr->state = GridNode::OPENSET;
//                                         neighborPtr->cameFrom = current;
//                                         neighborPtr->gScore = tentative_gScore;
//                                         neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
//                                         openSet_.push(neighborPtr); // put neighbor in open set and record it.
//                                         // cout << "new " << neighborPtr->index.transpose() << " fS=" << neighborPtr->fScore << " gS=" << neighborPtr->gScore << " dist=" << dist << endl;
//                                     }
//                                     else if (tentative_gScore < neighborPtr->gScore)
//                                     { // in open set and need update
//                                         neighborPtr->cameFrom = current;
//                                         neighborPtr->gScore = tentative_gScore;
//                                         neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
//                                         // cout << "old " << neighborPtr->index.transpose() << " fS=" << neighborPtr->fScore << " gS=" << neighborPtr->gScore << " dist=" << dist << endl;
//                                     }
//                                 }
//                             }
//                         }
//                         /******************/

//                         for (int xg = inx + 1; xg < outx; ++xg)
//                         {
//                             if (xg < 1 || xg >= POOL_SIZE_(0) - 1 || yg < 1 || yg >= POOL_SIZE_(1) - 1 || zg < 1 || zg >= POOL_SIZE_(2) - 1)
//                             {
//                                 continue;
//                             }

//                             neighborPtr = GridNodeMap_[xg][yg][zg];
//                             neighborPtr->index = Eigen::Vector3i(xg, yg, zg);
//                             neighborPtr->state = GridNode::CLOSEDSET;
//                             neighborPtr->rounds = rounds_;
//                         }

//                         /*****************/
//                         if (outx >= 1 && outx < POOL_SIZE_(0) - 1 && yg >= 1 && yg < POOL_SIZE_(1) - 1 && zg >= 1 && zg < POOL_SIZE_(2) - 1)
//                         {
//                             neighborPtr = GridNodeMap_[outx][yg][zg];
//                             neighborPtr->index = Eigen::Vector3i(outx, yg, zg);

//                             bool flag_explored = neighborPtr->rounds == rounds_;

//                             if (!flag_explored || neighborPtr->state != GridNode::CLOSEDSET)
//                             {

//                                 Eigen::Vector3d pt = Index2Coord(neighborPtr->index);

//                                 double proj = (pt - start_pt).dot(s2e_vec);
//                                 if (proj <= s2e_norm && (pt - start_pt).squaredNorm() - proj * proj <= radius2)
//                                 {

//                                     neighborPtr->rounds = rounds_;

//                                     double dist = getDistance(pt);
//                                     // if (dist < min_dist)
//                                     // {
//                                     //     continue;
//                                     // }

//                                     double static_cost = (double)region;
//                                     tentative_gScore = current->gScore + static_cost + dist_pen * (1 / (dist + 1e-3));

//                                     if (!flag_explored)
//                                     {
//                                         // discover a new node
//                                         neighborPtr->state = GridNode::OPENSET;
//                                         neighborPtr->cameFrom = current;
//                                         neighborPtr->gScore = tentative_gScore;
//                                         neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
//                                         openSet_.push(neighborPtr); // put neighbor in open set and record it.
//                                         // cout << "new " << neighborPtr->index.transpose() << " fS=" << neighborPtr->fScore << " gS=" << neighborPtr->gScore << " dist=" << dist << endl;
//                                     }
//                                     else if (tentative_gScore < neighborPtr->gScore)
//                                     { // in open set and need update
//                                         neighborPtr->cameFrom = current;
//                                         neighborPtr->gScore = tentative_gScore;
//                                         neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
//                                         // cout << "old " << neighborPtr->index.transpose() << " fS=" << neighborPtr->fScore << " gS=" << neighborPtr->gScore << " dist=" << dist << endl;
//                                     }
//                                 }
//                             }
//                         }
//                         /******************/
//                     }
//                 }

//             // for (auto d : dir)
//             // {

//             //     Vector3i neighborIdx;
//             //     neighborIdx(0) = (current->index)(0) + d(0);
//             //     neighborIdx(1) = (current->index)(1) + d(1);
//             //     neighborIdx(2) = (current->index)(2) + d(2);

//             //     if (neighborIdx(0) < 1 || neighborIdx(0) >= POOL_SIZE_(0) - 1 || neighborIdx(1) < 1 || neighborIdx(1) >= POOL_SIZE_(1) - 1 || neighborIdx(2) < 1 || neighborIdx(2) >= POOL_SIZE_(2) - 1)
//             //     {
//             //         continue;
//             //     }

//             //     neighborPtr = GridNodeMap_[neighborIdx(0)][neighborIdx(1)][neighborIdx(2)];
//             //     neighborPtr->index = neighborIdx;

//             //     bool flag_explored = neighborPtr->rounds == rounds_;

//             //     if (flag_explored && neighborPtr->state == GridNode::CLOSEDSET)
//             //     {
//             //         continue; // in closed set.
//             //     }

//             //     Eigen::Vector3d pt = Index2Coord(neighborPtr->index);

//             //     double proj = (pt - start_pt).dot(s2e_vec);
//             //     if (proj > s2e_norm)
//             //         continue;

//             //     if ((pt - start_pt).squaredNorm() - proj * proj > radius2)
//             //         continue;

//             //     neighborPtr->rounds = rounds_;

//             //     double dist = getDistance(pt);
//             //     // if (dist < min_dist)
//             //     // {
//             //     //     continue;
//             //     // }

//             //     double static_cost = sqrt(d(0) * d(0) + d(1) * d(1) + d(2) * d(2));
//             //     tentative_gScore = current->gScore + static_cost + dist_pen * (1 / (dist + 1e-3));

//             //     if (!flag_explored)
//             //     {
//             //         // discover a new node
//             //         neighborPtr->state = GridNode::OPENSET;
//             //         neighborPtr->cameFrom = current;
//             //         neighborPtr->gScore = tentative_gScore;
//             //         neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
//             //         openSet_.push(neighborPtr); // put neighbor in open set and record it.
//             //         // cout << "new " << neighborPtr->index.transpose() << " fS=" << neighborPtr->fScore << " gS=" << neighborPtr->gScore << " dist=" << dist << endl;
//             //     }
//             //     else if (tentative_gScore < neighborPtr->gScore)
//             //     { // in open set and need update
//             //         neighborPtr->cameFrom = current;
//             //         neighborPtr->gScore = tentative_gScore;
//             //         neighborPtr->fScore = tentative_gScore + getHeu(neighborPtr, endPtr);
//             //         // cout << "old " << neighborPtr->index.transpose() << " fS=" << neighborPtr->fScore << " gS=" << neighborPtr->gScore << " dist=" << dist << endl;
//             //     }
//             // }
//         }

//         ros::Time time_2 = ros::Time::now();
//         if ((time_2 - time_1).toSec() > 0.1)
//             ROS_ERROR("[A*] No solution before openSet is empty. Time=%.3fs, iter=%d", (time_2 - time_1).toSec(), num_iter);

//         return ASTAR_RET::NO_PATH;
//     }

//     vector<Vector3d> AStar::getPath()
//     {
//         vector<Vector3d> path;

//         for (auto ptr : gridPath_)
//             path.push_back(Index2Coord(ptr->index));

//         reverse(path.begin(), path.end());
//         return path;
//     }

// } // namespace oc_a_star
