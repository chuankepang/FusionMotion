#include "rokae_node/rokae_node.hpp"

#include <thread>
#include <cmath>
#include <iostream>

#include <mutex>
#include <array>
#include <termios.h>
#include <fcntl.h>
#include <deque>
#include <stdexcept>
#include <unistd.h>

#include "rokae_rt/robot.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "cust_msgs/msg/stampfloat32array.hpp"

using namespace rokae;
using namespace std;

RokaeNode::RokaeNode(const std::string& ip) {
  robot_ = std::make_unique<xMateRobot>(ip);  // 根据实际机型可改
  error_code ec;
  robot_->setOperateMode(OperateMode::automatic, ec);
  robot_->setPowerState(true, ec);
  robot_->setMotionControlMode(MotionControlMode::NrtCommand, ec);
  robot_->setDefaultSpeed(200, ec);
  robot_->setDefaultZone(50, ec);
  robot_->setEventWatcher(Event::moveExecution, [this](const EventInfo& info){
    this->printMoveInfo(info);
  }, ec);
}

/* ------------------- 基础工具 ------------------- */
void RokaeNode::printMoveInfo(const EventInfo& info) {
  using namespace EventInfoKey::MoveExecution;
  print(cout, "[运动执行信息] ID:", std::any_cast<string>(info.at(ID)),
        "Index:", std::any_cast<int>(info.at(WaypointIndex)),
        "已完成:", std::any_cast<bool>(info.at(ReachTarget)) ? "YES":"NO",
        std::any_cast<error_code>(info.at(Error)));
}

void RokaeNode::waitForFinish(const string& traj_id, int index) {
  error_code ec;
  while(true) {
    auto info = robot_->queryEventInfo(Event::moveExecution, ec);
    auto _id = std::any_cast<string>(info.at(EventInfoKey::MoveExecution::ID));
    auto _idx = std::any_cast<int>(info.at(EventInfoKey::MoveExecution::WaypointIndex));
    if(_id == traj_id && _idx == index &&
       std::any_cast<bool>(info.at(EventInfoKey::MoveExecution::ReachTarget))) {
      print(cout, "路径", traj_id, ":", index, "已完成");
      return;
    }
    if(auto _ec = std::any_cast<error_code>(info.at(EventInfoKey::MoveExecution::Error))) {
      print(cout, "路径", _id, ":", _idx, "错误:", _ec.message());
      return;
    }
    this_thread::sleep_for(chrono::milliseconds(200));
  }
}

void RokaeNode::waitRobot(bool& running) {
  running = true;
  while(running) {
    this_thread::sleep_for(chrono::milliseconds(100));
    error_code ec;
    auto st = robot_->operationState(ec);
    if(st == OperationState::idle || st == OperationState::unknown) running = false;
  }
}

/* ------------------- 可配置运动接口 ------------------- */

/* 1. 严格遵循轴配置 */
void RokaeNode::moveWithForcedConf(
    const vector<double>& drag_posture,
    const vector<CartesianPosition>& waypoints,
    const vector<array<int,8>>& conf_data_list,
    double speed, double zone)
{
  error_code ec;
  string id;
  robot_->setToolset(default_toolset_, ec);
  robot_->setDefaultSpeed(speed, ec);
  robot_->setDefaultZone(zone, ec);

  print(cout, "运动到拖拽位姿");
  robot_->executeCommand({MoveAbsJCommand(drag_posture)}, ec);
  bool running; waitRobot(running);

  // 构造指令序列
  vector<MoveCommand*> cmds;
  for(size_t i = 0; i < waypoints.size(); ++i) {
    auto* j = new MoveJCommand(waypoints[i]);
    auto* l = new MoveLCommand(waypoints[i]);
    if(i < conf_data_list.size()) {
      j->target.confData = conf_data_list[i];
      l->target.confData = conf_data_list[i];
    }
    cmds.push_back(j);
    cmds.push_back(l);
  }

  // 不遵循 conf
  robot_->setDefaultConfOpt(false, ec);
  robot_->moveAppend(cmds, id, ec);
  robot_->moveStart(ec);
  waitForFinish(id, (int)waypoints.size()*2 - 1);

  // 遵循 conf
  robot_->setDefaultConfOpt(true, ec);
  robot_->moveReset(ec);
  robot_->moveAppend(cmds, id, ec);
  robot_->moveStart(ec);
  waitForFinish(id, (int)waypoints.size()*2 - 1);

  robot_->setDefaultConfOpt(false, ec);
  for(auto* p : cmds) delete p;
}

/* 2. 笛卡尔点位 + 偏移 + 暂停/继续 */
void RokaeNode::cartesianPointWithOffset(
    const array<double,6>& base_pos,
    const array<double,6>& offset_z,
    double speed1, double zone1,
    double speed2, double zone2,
    bool enable_input)
{
  error_code ec;
  MoveLCommand moveL1(base_pos, speed1, zone1);
  MoveLCommand moveL2(base_pos, speed2, zone2);
  moveL2.offset = {CartesianPosition::Offset::offs, offset_z};

  MoveJCommand moveJ1(base_pos, 200, 0);
  MoveJCommand moveJ2(base_pos, 1000, 80);
  moveJ2.offset = {CartesianPosition::Offset::relTool, offset_z};

  robot_->executeCommand({moveL1, moveL2}, ec);
  robot_->executeCommand({moveJ1, moveJ2}, ec);

  if(enable_input) {
    thread input([this, &ec]{
      int c{};
      print(cout, "[p]暂停 [c]继续 [q]退出");
      while(c != 'q') {
        c = getchar();
        switch(c){
          case 'p': robot_->stop(ec); break;
          case 'c': robot_->moveStart(ec); break;
        }
      }
    });
    input.join();
  }
  robot_->moveReset(ec);
}

/* 3. 螺旋线运动 */
void RokaeNode::spiralMove(
    const vector<double>& joint_start,
    const CartesianPosition& spiral_end1,
    double r0_1, double dr_1, double angle_1, bool cw_1, double v_1,
    const CartesianPosition& spiral_end2,
    double r0_2, double dr_2, double angle_2, bool cw_2, double v_2)
{
  error_code ec; string id;
  robot_->setToolset(default_toolset_, ec);

  MoveAbsJCommand absj(joint_start);
  MoveSPCommand sp1({spiral_end1, r0_1, dr_1, angle_1, cw_1, v_1});
  MoveSPCommand sp2({spiral_end2, r0_2, dr_2, angle_2, cw_2, v_2});

  robot_->moveAppend({absj}, id, ec);
  robot_->moveAppend({sp1, sp2}, id, ec);
  robot_->moveStart(ec);
  δύο waitForFinish(id, 1);
}

/* 4. 七轴冗余运动 */
void RokaeNode::redundantMove(
    const vector<double>& drag_posture,
    const CartesianPosition& line_target1, double elbow1,
    const CartesianPosition& line_target2, double elbow2,
    const CartesianPosition& circle_p1, const CartesianPosition& circle_p2,
    const CartesianPosition& circle_a1, const CartesianPosition& circle_a2,
    double elbow_circle,
    double speed, double zone,
    bool enable_collision_recover)
{
  error_code ec; string id;
  robot_->setToolset(default_toolset_, ec);
  robot_->setDefaultSpeed(speed, ec);
  robot_->setDefaultZone(zone, ec);

  if(enable_collision_recover) {
    robot_->setEventWatcher(Event::safety, [this](const EventInfo& info){
      bool collided = std::any_cast<bool>(info.at(EventInfoKey::Safety::Collided));
      if(collided){
        this_thread::sleep_for(chrono::seconds(5));
        error_code ec;
        robot_->setPowerState(true, ec);
        robot_->moveStart(ec);
        print(cout, "Recovered from collision");
      }
    }, ec);
  }

  MoveAbsJCommand absj(drag_posture);
  MoveLCommand line1(line_target1); line1.target.elbow = elbow1;
  MoveLCommand line2(line_target2); line2.target.elbow = elbow2;

  robot_->moveAppend({absj, line1, line2}, id, ec);
  robot_->moveStart(ec);
  waitForFinish(id, 0);

  MoveLCommand cp1(circle_p1); cp1.target.elbow = elbow_circle;
  MoveCCommand cc1(circle_p2, circle_a1), cc2(circle_p1, circle_a2);
  cc1.target.elbow = cc2.target.elbow = elbow_circle;

  robot_->moveAppend({cp1}, id, ec);
  robot_->moveAppend({cc1, cc2}, id, ec);
  robot_->moveStart(ec);
  waitForFinish(id, 1);
}

/* 5. 奇异点规避 */
void RokaeNode::avoidSingularityMove(
    const vector<double>& joint_start,
    const vector<CartesianPosition>& waypoints,
    bool enable_avoid)
{
  error_code ec; string id; bool running;
  robot_->setToolset(default_toolset_, ec);

  robot_->executeCommand({MoveAbsJCommand(joint_start)}, ec);
  waitRobot(running);

  vector<MoveLCommand> cmds;
  for(const auto& p : waypoints) cmds.emplace_back(p);

  robot_->setAvoidSingularity(false, ec);
  robot_->moveAppend(cmds, id, ec);
  robot_->moveStart(ec);
  waitForFinish(id, (int)cmds.size()-1);

  robot_->setAvoidSingularity(enable_avoid, ec);
  robot_->moveReset(ec);
  robot_->moveAppend(cmds, id, ec);
  robot_->moveStart(ec);
  waitForFinish(id, (int)cmds.size()-1);
  robot_->setAvoidSingularity(false, ec);
}

/* 6. 工具工件坐标系 */
void RokaeNode::moveInToolsetCoordinate(
    const string& tool_name,
    const string& wobj_name,
    const vector<double>& joint_drag,
    const vector<CartesianPosition>& waypoints,
    double speed, double zone)
{
  error_code ec; string id;
  robot_->setToolset(tool_name, wobj_name, ec);
  print(cout, "当前工具工件:", tool_name, "/", wobj_name);

  if(!joint_drag.empty()){
    robot_->moveAppend({MoveAbsJCommand(joint_drag)}, id, ec);
  }
  vector<MoveLCommand> cmds;
  for(const auto& p : waypoints) cmds.emplace_back(p, speed, zone);
  robot_->moveAppend(cmds, id, ec);
  robot_->moveStart(ec);
  bool moving = true; waitRobot(moving);
}

/* 7. 运行时调速 */
void RokaeNode::adjustSpeed(
    const vector<vector<double>>& joint_waypoints,
    double init_scale,
    bool enable_input)
{
  error_code ec; string id;
  double scale = init_scale;
  robot_->adjustSpeedOnline(scale, ec);

  vector<MoveAbsJCommand> cmds;
  for(const auto& j : joint_waypoints) cmds.emplace_back(j);
  robot_->moveAppend(cmds, id, ec);
  robot_->moveStart(ec);
  bool running = true;

  thread input([&]{
    while(running){
      auto ch = getchar();
      if(ch=='a' && scale>0.1) { scale-=0.1; robot_->adjustSpeedOnline(scale,ec); }
      if(ch=='d' && scale<1.0) { scale+=0.1; robot_->adjustSpeedOnline(scale,ec); }
      print(cout, "速度比例:", scale);
    }
  });

  waitRobot(running);
  if(enable_input) input.join();
}

/* 8. 多逆解 */
void RokaeNode::multiplePosture(
    const CartesianPosition& target,
    const vector<array<int,8>>& conf_list)
{
  error_code ec; string id;
  robot_->setToolset(default_toolset_, ec);

  for(const auto& conf : conf_list){
    MoveJCommand cmd(target);
    cmd.target.confData = conf;
    robot_->moveAppend({cmd}, id, ec);
  }
  robot_->moveStart(ec);
  waitForFinish(id, 0);
}