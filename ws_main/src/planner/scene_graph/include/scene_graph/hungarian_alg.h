#ifndef HUNGARIAN_MATCHER_H
#define HUNGARIAN_MATCHER_H

#include <iostream>
#include <vector>
#include <map>
#include <limits>
#include <algorithm>
#include <numeric>

// 引入Eigen核心部分
#include <Eigen/Dense>

namespace Hungarian {

/**
 * @class HungarianMatcher
 * @brief 使用匈牙利算法来解决分配问题，特别是用于匹配新旧三维向量均值。
 *
 * 该类将原始的自由函数封装成一个独立的单元。
 * matchMeans 是主要的公共接口，可以静态调用。
 */
class HungarianMatcher {
public:
    /**
     * @brief 匹配新旧均值，使得总变化（距离平方和）最小。
     * 这是一个静态方法，可以直接通过类名调用，无需创建实例。
     * @param old_means 旧的平均值ID和向量的映射
     * @param new_means 新的平均值ID和向量的映射
     * @return 一个从新ID到旧ID的匹配映射
     */
    static std::map<int, int> matchMeans(
        const std::map<int, Eigen::Vector3d>& old_means,
        const std::map<int, Eigen::Vector3d>& new_means) {

        if (old_means.empty() || new_means.empty()) {
            return {};
        }

        // 1. 将ID和向量提取到vector中，以建立稳定的索引
        std::vector<int> old_ids, new_ids;
        std::vector<Eigen::Vector3d> old_vecs, new_vecs;
        for(const auto& pair : old_means) {
            old_ids.push_back(pair.first);
            old_vecs.push_back(pair.second);
        }
        for(const auto& pair : new_means) {
            new_ids.push_back(pair.first);
            new_vecs.push_back(pair.second);
        }

        // 2. 确定哪个集合是“行”（较小者），哪个是“列”（较大者）
        bool swapped = old_ids.size() > new_ids.size();

        const auto& row_ids = swapped ? new_ids : old_ids;
        const auto& row_vecs = swapped ? new_vecs : old_vecs;
        const auto& col_ids = swapped ? old_ids : new_ids;
        const auto& col_vecs = swapped ? old_vecs : new_vecs;

        size_t num_rows = row_ids.size();
        size_t num_cols = col_ids.size();
        size_t matrix_size = num_cols; // 成本矩阵设为方阵

        // 3. 构建成本矩阵
        std::vector<std::vector<double>> cost_matrix(matrix_size, std::vector<double>(matrix_size, 0));
        for (size_t i = 0; i < num_rows; ++i) {
            for (size_t j = 0; j < num_cols; ++j) {
                cost_matrix[i][j] = (row_vecs[i] - col_vecs[j]).squaredNorm();
            }
        }

        // 为虚拟行填充一个较大的成本
        double high_cost = std::numeric_limits<double>::max() / 2.0;
        for (size_t i = num_rows; i < matrix_size; ++i) {
            std::fill(cost_matrix[i].begin(), cost_matrix[i].end(), high_cost);
        }

        // 4. 使用匈牙利算法求解
        std::vector<int> assignment; // assignment[i] = j 表示行 i 匹配到列 j
        solve(cost_matrix, assignment);

        // 5. 解析结果，将索引映射回ID
        std::map<int, int> new_to_old_id_map;
        double total_real_cost = 0;

        for (size_t i = 0; i < num_rows; ++i) { // 只需遍历真实的行
            int col_idx = assignment[i];
            if (col_idx < num_cols) { // 确保匹配到的是真实的列
                int row_id = row_ids[i];
                int col_id = col_ids[col_idx];

                int old_id, new_id;
                if (swapped) { // 行是new, 列是old
                    new_id = row_id;
                    old_id = col_id;
                } else { // 行是old, 列是new
                    new_id = col_id;
                    old_id = row_id;
                }
                new_to_old_id_map[new_id] = old_id;
                total_real_cost += cost_matrix[i][col_idx];
            }
        }

        std::cout << "匹配完成，真实匹配的总成本（距离平方和）为: " << total_real_cost << std::endl;
        return new_to_old_id_map;
    }

private:
    /**
     * @brief 匈牙利算法的核心实现，用于解决分配问题（最小化成本）。
     * @param costMatrix 成本矩阵 (必须是方阵)。
     * @param assignment 用于存储结果的向量，assignment[i] = j 表示第 i 行匹配到第 j 列。
     * @return 最小总成本。
     */
    static double solve(const std::vector<std::vector<double>>& costMatrix, std::vector<int>& assignment) {
        const int n = costMatrix.size();
        if (n == 0) return 0.0;

        assignment.assign(n, -1);
        std::vector<double> u(n + 1, 0.0), v(n + 1, 0.0);
        std::vector<int> p(n + 1, 0), way(n + 1, 0);

        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<double> minv(n + 1, std::numeric_limits<double>::max());
            std::vector<bool> used(n + 1, false);
            do {
                used[j0] = true;
                int i0 = p[j0], j1 = 0;
                double delta = std::numeric_limits<double>::max();
                for (int j = 1; j <= n; ++j) {
                    if (!used[j]) {
                        double cur = costMatrix[i0 - 1][j - 1] - u[i0] - v[j];
                        if (cur < minv[j]) {
                            minv[j] = cur;
                            way[j] = j0;
                        }
                        if (minv[j] < delta) {
                            delta = minv[j];
                            j1 = j;
                        }
                    }
                }
                for (int j = 0; j <= n; ++j) {
                    if (used[j]) {
                        u[p[j]] += delta;
                        v[j] -= delta;
                    } else {
                        minv[j] -= delta;
                    }
                }
                j0 = j1;
            } while (p[j0] != 0);
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }

        std::vector<int> result(n);
        for (int j = 1; j <= n; ++j) {
            result[p[j] - 1] = j - 1;
        }
        assignment = result;

        return -v[0]; // 返回最小成本
    }
};

} // namespace Hungarian

/*
// =================================================================
//                      如何使用新的Class
// =================================================================
int main() {
    // --- 示例数据：旧ID少于新ID ---
    // 旧的平均值数据
    std::map<int, Eigen::Vector3d> old_data;
    old_data[20] = Eigen::Vector3d(5.0, 5.0, 5.0);    // ID 20
    old_data[30] = Eigen::Vector3d(10.0, 10.0, 10.0); // ID 30

    // 新的平均值数据
    std::map<int, Eigen::Vector3d> new_data;
    new_data[101] = Eigen::Vector3d(5.1, 5.2, 5.0);   // ID 101, 最接近 ID 20
    new_data[102] = Eigen::Vector3d(0.9, 1.1, 0.8);   // ID 102, 这个应该不会被匹配
    new_data[103] = Eigen::Vector3d(10.5, 9.8, 10.1); // ID 103, 最接近 ID 30

    // --- 执行匹配 ---
    // 注意调用方式的变化：通过类名直接调用静态方法
    std::map<int, int> matches = Hungarian::HungarianMatcher::matchMeans(old_data, new_data);

    // --- 打印结果 ---
    std::cout << "\n匹配结果 (新ID -> 旧ID):" << std::endl;
    for (const auto& match : matches) {
        std::cout << "新 ID: " << match.first << "  ->  匹配到旧 ID: " << match.second << std::endl;
    }

    return 0;
}
*/

#endif // HUNGARIAN_MATCHER_H