#ifndef __GRIDMAP_H__
#define __GRIDMAP_H__


#include <cmath>

// 角度转弧度
#define DEG2RAD (M_PI / 180.0)

class GridMap {
    private:
        // 地图参数
        float resolution_;     // 栅格分辨率
        float size_x_, size_y_;// 物理尺寸
        int grid_x_, grid_y_;  // 栅格数量
        float* log_map_;       // 对数几率地图
    
        // 逆传感器模型概率参数
        float p_occ_;
        float p_free_;
        float p_prior_;

    public:
    /**
     * 构造函数
     * @param resolution 栅格分辨率（米）【手动设置】
     * @param size_x 地图X方向物理尺寸（米）
     * @param size_y 地图Y方向物理尺寸（米）
     */
    GridMap(float resolution, float size_x, float size_y);

    // 析构函数
    ~GridMap();

    /**
     * 世界坐标 转 栅格坐标
     * @param x 世界x
     * @param y 世界y
     * @param gx 输出栅格x
     * @param gy 输出栅格y
     * @return 转换是否成功（是否在地图内）
     */
    bool World2grid(float x, float y, int& gx, int& gy);

    /**
     * 更新单个栅格（对数几率形式）
     * @param x 世界x
     * @param y 世界y
     * @param p 占据概率
     */
    void UpdateGrid(float x, float y, float p);

    /**
     * 🔥 超声波传感器更新地图（核心）
     * @param robot_x    机器人x
     * @param robot_y    机器人y
     * @param robot_yaw  机器人朝向角（弧度）
     * @param sensor_dist 超声测距值
     * @param beam_angle  超声波束角（度），默认15°
     */
    void UpdateByUltrasonic(float robot_x, float robot_y, float robot_yaw,
                           float sensor_dist, float beam_angle = 15.0f);

    // 简单打印地图到控制台
    void PrintMap();

    // 使用 ROOT 可视化栅格地图
    void PrintMapROOT(const char* output_file = nullptr);

};



#endif // __GRIDMAP_H__