#include <iostream>
#include <cstring>
#include "GridMap.h"
#include <TCanvas.h>
#include <TH2F.h>
#include <TColor.h>
#include <TStyle.h>
#include <TAxis.h>
using namespace std;

// 角度转弧度（超声波束角用）
#define DEG2RAD (M_PI / 180.0)

// 栅格地图类

/**
 * 构造函数
 * @param resolution 栅格分辨率（米）【手动设置】
 * @param size_x 地图X长度（米）
 * @param size_y 地图Y长度（米）
 */
GridMap::GridMap(float resolution, float size_x, float size_y) 
    : resolution_(resolution), size_x_(size_x), size_y_(size_y)
{
    // 计算栅格数量
    grid_x_ = ceil(size_x / resolution);
    grid_y_ = ceil(size_y / resolution);
    
    // 对数几率地图（初始=0，未知）
    log_map_ = new float[grid_x_ * grid_y_];
    memset(log_map_, 0, grid_x_ * grid_y_ * sizeof(float));
    
    // 超声逆传感器模型参数
    p_occ_ = 0.8f;    // 占据概率
    p_free_ = 0.2f;    // 空闲概率
    p_prior_ = 0.5f;   // 初始概率
}

GridMap::~GridMap() {
    delete[] log_map_;
}

// 世界坐标 -> 栅格索引
bool GridMap::World2grid(float x, float y, int& gx, int& gy) {
    gx = (int)(x / resolution_);
    gy = (int)(y / resolution_);
    if (gx < 0 || gx >= grid_x_ || gy < 0 || gy >= grid_y_)
        return false;
    return true;
}

// 更新单个栅格（对数几率）
void GridMap::UpdateGrid(float x, float y, float p) {
    int gx, gy;
    if (!World2grid(x, y, gx, gy)) return;

    // 对数几率更新公式
    float log_odds = log(p / (1 - p));
    log_map_[gx * grid_y_ + gy] += log_odds;
}

// ==================== 🔥 超声波逆传感器模型（核心） ====================
// 超声：扇形覆盖（15°波束角），更新扇形内所有栅格
void GridMap::UpdateByUltrasonic(
    float robot_x, float robot_y, float robot_yaw,   // 机器人位姿
    float sensor_dist,                               // 超声测距值
    float beam_angle                                 // 波束角【固定15°】
) {
    if (sensor_dist < 0.02f || sensor_dist > 3.0f)   // 超声有效范围 2cm~3m
        return;

    float half_angle = beam_angle / 2.0f * DEG2RAD;   // 半角
    float step_angle = 1.0f * DEG2RAD;                // 角度步长（密一点）

    // 遍历扇形内所有角度
    for (float angle = -half_angle; angle <= half_angle; angle += step_angle) {
        // 当前射线总角度
        float ray_angle = robot_yaw + angle;
        float cos_a = cos(ray_angle);
        float sin_a = sin(ray_angle);

        // 沿着射线步进（一个栅格一步）
        for (float r = 0; r <= sensor_dist + resolution_; r += resolution_) {
            // 世界坐标
            float x = robot_x + r * cos_a;
            float y = robot_y + r * sin_a;

            // 逆模型判断：空闲 / 占据
            float p;
            if (r < sensor_dist - 0.5f * resolution_)
                p = p_free_;        // 障碍物前 → 空闲
            else if (r > sensor_dist + 0.5f * resolution_)
                p = p_prior_;       // 障碍物后 → 无信息
            else
                p = p_occ_;         // 障碍物位置 → 占据

            UpdateGrid(x, y, p);
        }
    }
}

// 打印地图（控制台简单可视化）
void GridMap::PrintMap() {
    cout << "\n=== 栅格地图（分辨率: " << resolution_ << "m）===\n";
    for (int y = grid_y_ - 1; y >= 0; y--) {
        for (int x = 0; x < grid_x_; x++) {
            float val = log_map_[x * grid_y_ + y];
            if (val > 1.0f)      cout << "■ ";  // 占据
            else if (val < -1.0f) cout << "□ ";  // 空闲
            else                 cout << "· ";  // 未知
        }
        cout << endl;
    }
}

// 使用 ROOT 可视化栅格地图
void GridMap::PrintMapROOT(const char* output_file) {
    // 创建画布
    TCanvas *c1 = new TCanvas("c1", "栅格地图可视化", 800, 800);
    
    // 创建2D直方图，bin数量 = 栅格数量
    // 范围：X轴 [0, size_x_]，Y轴 [0, size_y_]
    TH2F *hist = new TH2F("gridmap", "GridMap;X [m];Y [m]", 
                          grid_x_, 0, size_x_, 
                          grid_y_, 0, size_y_);
    
    // 填充直方图
    for (int x = 0; x < grid_x_; x++) {
        for (int y = 0; y < grid_y_; y++) {
            float val = log_map_[x * grid_y_ + y];
            // 将对数几率转换为可视化值
            // 占据(val > 1) -> 正值, 空闲(val < -1) -> 负值, 未知(接近0) -> 0
            hist->SetBinContent(x + 1, y + 1, val);
        }
    }
    
    // 设置可视化选项
    hist->SetStats(0);  // 不显示统计信息
    hist->SetMinimum(-2.0f);  // 设置最小值
    hist->SetMaximum(2.0f);   // 设置最大值
    
    // 设置颜色调色板
    gStyle->SetPalette(kRainBow);
    // hist->SetContour(1000);  // 颜色平滑过渡
    
    // 绘制
    hist->Draw("COL");
    
    // 保存为图片
    if (output_file != nullptr) {
        c1->Print(output_file);
        cout << "栅格地图已保存到: " << output_file << endl;
    } else {
        c1->Print("gridmap.png");
        cout << "栅格地图已保存到: gridmap.png" << endl;
    }
    
    // 清理（可选，如果是在交互式ROOT会话中可能不需要）
    delete hist;
    delete c1;
}

// // ==================== 主函数测试 ====================
// int main() {
//     // 1. 创建地图：【手动设置分辨率 0.1m】，地图 4m × 4m
//     GridMap map(0.1f, 4.0f, 4.0f);

//     // 2. 模拟机器人与超声数据
//     float robot_x = 2.0f;    // 机器人坐标
//     float robot_y = 2.0f;
//     float robot_yaw = 0.0f;  // 朝向0°
//     float distance = 1.0f;  // 超声测到障碍物 1米

//     // 3. 超声更新：波束角【固定15°】
//     map.updateByUltrasonic(robot_x, robot_y, robot_yaw, distance, 15.0f);

//     // 4. 打印地图
//     map.printMap();

//     return 0;
// }