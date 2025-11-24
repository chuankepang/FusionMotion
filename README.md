# FusionMotion
FusionMotion is a multi-sensory fusion algorithm integrating vision, touch, and motion control for enhanced robotic task execution. It has been validated on real-world platforms, including the Rokae robotic arm and Inspire Hand, demonstrating efficient and precise performance in physical experiments.

## realsense-ros installation
### realsense SDK
1. SDK Download

```
git clone https://github.com/IntelRealSense/librealsense.git
```

2. Dependency Installation

```
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev
```

3. Permission Scripts Installation

```
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 
```

4. Make

```
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
make -j4
sudo make install

```

5. Usage

```
realsense-viewer
```

### realsense-ros

1. realsense-ros installation

```
sudo apt install ros-<ROS_DISTRO>-realsense2-*
```

2. realsense start

```
roslaunch realsense2_camera rs_camera.launch
```

一、 详细流程描述
1. 视觉引导与粗抓取 (Vision-Guided Grasping)
动作： 机械臂基于视觉定位移动至电连接器工头（Male）上方。
关键点： 夹爪闭合抓取。此时允许存在抓取误差（如位置偏差、角度倾斜），不强求视觉的绝对精准，因为后续有触觉补偿。
2. 触觉感知与位姿修正 (Tactile Pose Estimation)
动作： 夹爪内的触觉阵列（Tactile Array）采集接触面的压力/图像信息。
算法： 通过触觉算法解算出工头在夹爪内的真实位姿偏移矩阵 $T_{error}$。
输出： 更新“工头尖端”在机器人基坐标系下的精确坐标 $T_{tip} = T_{flange} \cdot T_{grasp} \cdot T_{error}$。
3. 接近与初定位 (Approach & Alignment)
动作： 机器人以更新后的 $T_{tip}$ 为控制点，将工头移动至母头（Female）前方的预备点（Pre-insertion Point）。
状态： 此时为自由空间运动（Free Space Motion），位置控制模式。
4. 接触检测与寻孔策略 (Contact Detection & Search Strategy)
动作： 沿Z轴低速进给，直到六维力传感器检测到 $F_z$ 超过接触阈值（表明接触母头表面）。
策略（核心）：开启平面导纳控制（XY轴柔顺，Z轴保持恒定接触力）。执行搜索轨迹（如阿基米德螺旋线 Spiral Search）。
判定： 当监测到 $Z$ 轴位置发生突变（陷入孔内）或 $XY$ 平面约束力瞬间减小时，判定**“寻孔成功”**，停止搜索运动。
5. 分阶段柔顺插入 (Phased Compliant Insertion)
策略： 进入插入模式，基于深度 $D$ 分阶段调整导纳参数（$M, B, K$），需加入参数平滑过渡算法防止运动突变：阶段一（引导段）： 低刚度，高阻尼。允许工头顺应孔的倒角自动调整姿态（Align Orientation）。阶段二（摩擦段）： 中高刚度（Z轴），XY轴保持柔顺。克服插拔摩擦力，保证直线推进。异常处理： 实时监控 $F_z$ 和位置变化率，若检测到“卡死（Jamming）”，触发回退重插逻辑。
完成： 当达到目标深度或检测到触底硬接触力（Bottoming Force）时，停止运动。