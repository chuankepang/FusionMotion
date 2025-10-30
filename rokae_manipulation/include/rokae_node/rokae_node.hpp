#pragma once
#include <rokae_rt/robot.h>
#include <string>
#include <vector>
#include <array>
#include <functional>

using namespace rokae;

/**
 * @brief 运动节点类，集中管理所有非实时运动示例
 */
class RokaeNode {
public:
  explicit RokaeNode(const std::string& ip);

  // ---------- 基础工具 ----------
  void waitForFinish(const std::string& traj_id, int index);
  void waitRobot(bool& running);
  void printMoveInfo(const EventInfo& info);

  // ---------- 可配置运动接口（供 main_node.cpp 调用） ----------
  // 1. 严格遵循轴配置的直线/关节运动
  void moveWithForcedConf(
      const std::vector<double>& drag_posture,
      const std::vector<CartesianPosition>& waypoints,
      const std::vector<std::array<int,8>>& conf_data_list = {},
      double speed = 200, double zone = 5);

  // 2. 笛卡尔点位 + 偏移 + 暂停/继续
  void cartesianPointWithOffset(
      const std::array<double,6>& base_pos,
      const std::array<double,6>& offset_z,
      double speed1 = 500, double zone1 = 5,
      double speed2 = 800, double zone2 = 0,
      bool enable_input = true);

  // 3. 螺旋线运动
  void spiralMove(
      const std::vector<double>& joint_start,
      const CartesianPosition& spiral_end1,
      double r0_1, double dr_1, double angle_1, bool cw_1, double v_1,
      const CartesianPosition& spiral_end2,
      double r0_2, double dr_2, double angle_2, bool cw_2, double v_2);

  // 4. 七轴冗余运动（变臂角 + 圆弧）
  void redundantMove(
      const std::vector<double>& drag_posture,
      const CartesianPosition& line_target1, double elbow1,
      const CartesianPosition& line_target2, double elbow2,
      const CartesianPosition& circle_p1, const CartesianPosition& circle_p2,
      const CartesianPosition& circle_a1, const CartesianPosition& circle_a2,
      double elbow_circle,
      double speed = 500, double zone = 0,
      bool enable_collision_recover = true);

  // 5. 奇异点规避（锁定4轴）
  void avoidSingularityMove(
      const std::vector<double>& joint_start,
      const std::vector<CartesianPosition>& waypoints,
      bool enable_avoid = true);

  // 6. 工具工件坐标系运动
  void moveInToolsetCoordinate(
      const std::string& tool_name = "tool0",
      const std::string& wobj_name = "wobj0",
      const std::vector<double>& joint_drag = {},
      const std::vector<CartesianPosition>& waypoints = {},
      double speed = 1000, double zone = 100);

  // 7. 运行时调速
  void adjustSpeed(
      const std::vector<std::vector<double>>& joint_waypoints,
      double init_scale = 0.5,
      bool enable_input = true);

  // 8. 多逆解（confData）示例
  void multiplePosture(
      const CartesianPosition& target,
      const std::vector<std::array<int,8>>& conf_list);

private:
  std::unique_ptr<BaseRobot> robot_;
  Toolset default_toolset_;
};