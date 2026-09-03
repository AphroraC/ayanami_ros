#!/usr/bin/env bash
# ayanami TF 环境自检脚本 (odom<-base_footprint 缺失排查)。
# 运行: bash tf_self_check.sh
# 输出决定性信息: gazebo 实例数 / odom 话题与 TF / 差速插件配置 / 帧树 / 插件加载日志。
# 对照结论:
#   - /odom 有数据且帧树有 odom -> diff drive 正常, 问题在 ayanami 配置;
#   - /odom 有话题但帧树无 odom -> diff drive 只发了 odometry 未发 TF (插件半载/参数问题);
#   - /odom 无话题 -> 差速插件未随模型加载 (检查下方 gazebo 日志的加载错误)。
set -u

echo "================ 1. 节点清单 (看 gazebo/amcl/ayanami 是否都在) ================"
rosnode list 2>&1 | sort

echo
echo "================ 2. gazebo / roslaunch 进程 (正常应各 1 个; 多个=残留实例) ================"
pgrep -af 'gzserver' || echo "(无 gzserver 进程!)"
pgrep -af 'roslaunch' || echo "(无 roslaunch 进程!)"

echo
echo "================ 3. /odom 话题 (3 秒采样; 无输出=diff drive 未发布) ================"
timeout 3 rostopic hz /odom 2>&1 | head -3
echo "-- /odom frame_id:"
timeout 3 rostopic echo -n1 /odom/header/frame_id 2>&1 | tail -1

echo
echo "================ 4. /scan 与 /cmd_vel 话题 ================"
timeout 3 rostopic hz /scan 2>&1 | head -3
timeout 3 rostopic hz /cmd_vel 2>&1 | head -3

echo
echo "================ 5. robot_description 中差速插件配置 ================"
rosparam get /robot_description 2>/dev/null | grep -oE '<publishTf>[^<]*|<publishOdomTF>[^<]*|libgazebo_ros_diff_drive[^"]*|robotBaseFrame>[^<]*' || echo "(读取失败/无插件块)"

echo
echo "================ 6. TF 帧树 (odom 是否在树上) ================"
timeout 5 rostopic echo -n1 /tf/transforms 2>&1 | grep -E 'child_frame_id|frame_id' | sort -u | head -20
echo "-- tf_static:"
timeout 5 rostopic echo -n1 /tf_static/transforms 2>&1 | grep -E 'child_frame_id|frame_id' | sort -u | head -20

echo
echo "================ 7. gazebo 插件加载错误日志 ================"
GLOG=$(ls -t ~/.ros/log/**/gazebo-2*.log ~/.ros/log/gazebo 2>/dev/null | head -1)
if [ -n "${GLOG:-}" ]; then
  grep -iE 'plugin|error|fail|diff' "$GLOG" | tail -20
else
  ls -t ~/.ros/log/ | head -5
fi

echo
echo "================ 8. AMCL (map->odom 发布者) 状态 ================"
timeout 3 rostopic hz /amcl/pose 2>&1 | head -3