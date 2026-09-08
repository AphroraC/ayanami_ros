#!/usr/bin/env bash
# TF 同步压力测试驱动: ROS TF -> AyanamiROS (tf2 buffer) -> syncTransformsToCore
# -> TransformStore (fast_tf) 全链路。
#
# 用法: tf_sync_stress_test.sh [DURATION_S] [FAST_HZ] [NORMAL_HZ]
#   DURATION_S  压测时长秒 (默认 15)
#   FAST_HZ     快边频率 (默认 500; 目标场景 500/1000)
#   NORMAL_HZ   普通边频率 (默认 200)
#
# 10 条边: edge0 = 固定边 (tf_static, map->laser_fixed, 1Hz 重发恒值),
# edge1 = 快边 (odom->n_fast), edge2..9 = 8 条普通边 (odom->n0..n7)。
#
# 通过门限: 聚合同步率 > 98% 且 延迟 mean/p95 < 3ms (结果写
# /tmp/tf_sync_stress/result.json, 退出码 0 = 通过)。
#
# 环境约束: 容器内 11311 被占用, 默认用 http://localhost:11313 的 roscore
# (不存在则自动拉起)。所有读写均在容器内完成。
set -uo pipefail

DURATION="${1:-15}"
FAST_HZ="${2:-500}"
NORMAL_HZ="${3:-200}"
WORKDIR=/tmp/tf_sync_stress
SRC_CONFIG=/share/nav_ws/src/ayanami/config
# 11311 被容器内 VS Code 占用, 强制使用 11313 (可用 TF_SYNC_MASTER 覆盖)
export ROS_MASTER_URI="${TF_SYNC_MASTER:-http://localhost:11313}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-localhost}"

if [ -z "${ROS_DISTRO:-}" ]; then
  source /opt/ros/noetic/setup.bash
fi
if [ -f /share/nav_ws/devel/setup.bash ]; then
  source /share/nav_ws/devel/setup.bash
fi

echo "== [1/6] roscore ($ROS_MASTER_URI) =="
if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "starting roscore on 11313..."
  nohup roscore -p 11313 >/tmp/tf_sync_stress_roscore.log 2>&1 &
  for _ in $(seq 1 30); do
    sleep 1
    timeout 3 rostopic list >/dev/null 2>&1 && break
  done
fi
timeout 5 rostopic list >/dev/null 2>&1 || { echo "FATAL: roscore 不可用"; exit 2; }

echo "== [2/6] 生成测试配置 ($WORKDIR/config) =="
mkdir -p "$WORKDIR"
rm -rf "$WORKDIR/config" "$WORKDIR/probe_ready" "$WORKDIR/events.csv" \
  /dev/shm/tf_sync_stress/probe_ready /dev/shm/tf_sync_stress/events.csv \
  "$WORKDIR/publish.csv" "$WORKDIR/result.json" "$WORKDIR/node.log"
cp -r "$SRC_CONFIG" "$WORKDIR/config"
python3 - "$WORKDIR/config/config_ros.json" <<'PYCFG'
import json, sys
p = sys.argv[1]
cfg = json.load(open(p))
ros = cfg['ayanami_ros']
exts = ros.get('extension_modules', [])
if isinstance(exts, str):
    exts = [exts]
if 'libayanami_tf_sync_probe.so' not in exts:
    exts.append('libayanami_tf_sync_probe.so')
ros['extension_modules'] = exts
# 10 条同步边: edge0 固定 (tf_static), edge1 快边, edge2..9 普通边;
# 顺序即发布器/探针的 edge_idx。
edges = [['map', 'laser_fixed'], ['odom', 'n_fast']]
edges += [['odom', 'n%d' % j] for j in range(8)]
ros['tf_sync_edges'] = edges
ros['tf_static_edges'] = [['map', 'laser_fixed']]
# 探针读顶层 tf_sync_probe 段 (不嵌套在 ayanami_ros 下)
cfg['tf_sync_probe'] = {
    'edges': edges,
    'poll_hz': 2000.0,
    'events_path': '/dev/shm/tf_sync_stress/events.csv',
    'ready_path': '/dev/shm/tf_sync_stress/probe_ready',
}
json.dump(cfg, open(p, 'w'), indent=2, ensure_ascii=False)
check = json.load(open(p))['ayanami_ros']
assert len(check['tf_sync_edges']) == 10, check['tf_sync_edges']
print('config patched: %d sync edges' % len(edges))
PYCFG

# 扩展清单单一来源 = config_extensions.json (含 // 注释, 文本级修补):
# 反注释/插入探针扩展条目
python3 - "$WORKDIR/config/config_extensions.json" <<'PYEXT'
import sys
p = sys.argv[1]
s = open(p).read()
s = s.replace('// "libayanami_tf_sync_probe.so"',
              '"libayanami_tf_sync_probe.so"')
if '"libayanami_tf_sync_probe.so"' not in s:
    s = s.replace('"loading_extension_modules": [',
                  '"loading_extension_modules": [\n      '
                  '"libayanami_tf_sync_probe.so",')
lines = [l for l in s.splitlines()
         if 'libayanami_tf_sync_probe.so' in l and l.strip().startswith('//')]
assert not lines, 'probe entry still commented: %s' % lines
open(p, 'w').write(s)
print('extension list patched')
PYEXT

echo "== [3/6] 清理残留节点, 启动 ayanami_rosnode =="
pkill -9 -f ayanami_rosnode >/dev/null 2>&1 || true
sleep 1
rm -f "$WORKDIR/node.log"
rosrun ayanami_ros ayanami_rosnode _config_path:="$WORKDIR/config" \
  >"$WORKDIR/node.log" 2>&1 &
NODE_PID=$!
echo "ayanami_rosnode pid=$NODE_PID"
# fail-fast: 探针扩展必须在 10s 内完成加载, 否则连同诊断直接退出
PROBE_LOADED=0
for _ in $(seq 1 10); do
  if grep -q 'tf_sync_probe: monitoring' "$WORKDIR/node.log" 2>/dev/null; then
    PROBE_LOADED=1
    break
  fi
  kill -0 "$NODE_PID" 2>/dev/null || break
  sleep 1
done
if [ "$PROBE_LOADED" != "1" ]; then
  echo "FATAL: 探针扩展未加载; 配置 extension_modules:"
  python3 -c "import json;print(json.load(open('$WORKDIR/config/config_ros.json'))['ayanami_ros']['extension_modules'])"
  echo "node.log 头部:"
  head -30 "$WORKDIR/node.log"
  kill "$NODE_PID" 2>/dev/null
  exit 5
fi

# 注: 探针就绪标记 = 首次成功查询, 需要 TF 数据流存在; 因此先起发布器
# (后台), 再等就绪, 最后等发布器自然结束 —— 避免互相等待的死锁。
echo "== [4/6] 压力发布 (快边 ${FAST_HZ}Hz + 8x${NORMAL_HZ}Hz + 固定边) x ${DURATION}s =="
python3 "$(rospack find ayanami_ros)/scripts/tf_sync_publisher.py" \
  --duration "$DURATION" --fast-rate "$FAST_HZ" --normal-rate "$NORMAL_HZ" \
  --out-csv "$WORKDIR/publish.csv" \
  >"$WORKDIR/publisher.log" 2>&1 &
PUB_PID=$!

READY=0
for _ in $(seq 1 90); do
  if [ -f /dev/shm/tf_sync_stress/probe_ready ]; then READY=1; break; fi
  kill -0 "$NODE_PID" 2>/dev/null || break
  kill -0 "$PUB_PID" 2>/dev/null || break
  sleep 1
done
if [ "$READY" != "1" ]; then
  echo "FATAL: 探针未就绪 / 节点或发布器退出; node.log 尾部:"
  tail -20 "$WORKDIR/node.log"
  echo "publisher.log 尾部:"
  tail -10 "$WORKDIR/publisher.log" 2>/dev/null
  kill "$NODE_PID" "$PUB_PID" 2>/dev/null
  exit 3
fi
echo "探针就绪"

wait "$PUB_PID"
PUB_RC=$?

sleep 1
kill -INT "$NODE_PID" 2>/dev/null
sleep 2
kill -9 "$NODE_PID" 2>/dev/null || true

echo "== [6/6] 分析 =="
if [ "$PUB_RC" != "0" ]; then
  echo "FATAL: 发布器异常退出 (rc=$PUB_RC)"; exit 4
fi
python3 "$(rospack find ayanami_ros)/scripts/tf_sync_analyze.py" \
  --publish-csv "$WORKDIR/publish.csv" \
  --events-csv /dev/shm/tf_sync_stress/events.csv \
  --out-json "$WORKDIR/result.json" \
  --rate-gate 0.98 --latency-gate-ms 3.0 --warmup-s 0.5 --fixed-edge 0
