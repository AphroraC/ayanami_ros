#!/usr/bin/env python3
"""TF 同步压力发布器 (多边多速率)。

10 条边: edge 0 = 固定边 (tf_static, 1Hz 重发, 值恒定);
edge 1 = 快边 (默认 500Hz, --fast-rate 可调); edge 2..9 = 8 条 200Hz 边。
各动态边把 seq 编码进 translation.x (x = offset_e + seq * step,
offset_e = 10*edge + 1, step = 1e-3)。每条样本记录
(edge, seq, x, stamp_ns, wall) 到 CSV, 供 tf_sync_stress_test.sh 离线分析。
单线程按时间片调度 (快边每 tick, 200Hz 边按各自到期时刻), 避免 GIL 抖动。
"""

import argparse
import csv
import signal
import time

import rospy
import tf
import tf2_ros
import geometry_msgs.msg


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', type=float, default=15.0)
    parser.add_argument('--fast-rate', type=float, default=500.0)
    parser.add_argument('--normal-rate', type=float, default=200.0)
    parser.add_argument('--num-normal', type=int, default=8)
    parser.add_argument('--static-rate', type=float, default=1.0)
    parser.add_argument('--out-csv', default='/tmp/tf_sync_stress/publish.csv')
    parser.add_argument('--step', type=float, default=1e-3)
    args = parser.parse_args()

    rospy.init_node('tf_sync_stress_publisher', anonymous=True)
    br = tf.TransformBroadcaster()
    sbr = tf2_ros.StaticTransformBroadcaster()

    running = {'flag': True}

    def on_sigint(_sig, _frm):
        running['flag'] = False

    signal.signal(signal.SIGINT, on_sigint)
    signal.signal(signal.SIGTERM, on_sigint)

    out = open(args.out_csv, 'w', newline='')
    writer = csv.writer(out)
    writer.writerow(['edge', 'seq', 'x', 'stamp_ns', 'wall'])

    # 等 /tf 订阅者连接 (ayanami_rosnode 的事件驱动同步订阅)
    waited = 0.0
    while running['flag'] and not rospy.is_shutdown():
        conns = br.tf2_broadcaster.pub_tf.get_num_connections()
        if conns > 0:
            rospy.loginfo('/tf subscriber connected (%d)', conns)
            break
        time.sleep(0.1)
        waited += 0.1
        if waited >= 30.0:
            rospy.logwarn('no /tf subscriber after 30s, starting anyway')
            break

    num_edges = 2 + args.num_normal  # 0=fixed, 1=fast, 2..9=normal
    seqs = [0] * num_edges

    def send_dynamic(edge, parent, child, now):
        x = (10.0 * edge + 1.0) + seqs[edge] * args.step
        stamp = rospy.Time(nsecs=int(now * 1e9))
        br.sendTransform((x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), stamp, child,
                         parent)
        writer.writerow([edge, seqs[edge], repr(x), stamp.to_nsec(),
                         repr(time.time())])
        seqs[edge] += 1

    def send_static(now):
        x = 9.5  # 固定值
        stamp = rospy.Time(nsecs=int(now * 1e9))
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'laser_fixed'
        t.transform.translation.x = x
        t.transform.rotation.w = 1.0
        sbr.sendTransform(t)
        writer.writerow([0, seqs[0], repr(x), stamp.to_nsec(),
                         repr(time.time())])
        seqs[0] += 1

    period = 1.0 / args.fast_rate
    normal_period = 1.0 / args.normal_rate
    static_period = 1.0 / args.static_rate
    start = time.time()
    deadline = start + args.duration
    next_static = start
    next_normal = [start + (j + 1) * normal_period / args.num_normal
                   for j in range(args.num_normal)]
    now = start
    while running['flag'] and not rospy.is_shutdown() and now < deadline:
        now = time.time()
        if now >= deadline:
            break
        send_dynamic(1, 'odom', 'n_fast', now)          # 快边: 每 tick
        for j in range(args.num_normal):                 # 200Hz 边: 按到期时刻
            edge = 2 + j
            if now >= next_normal[j]:
                send_dynamic(edge, 'odom', 'n%d' % j, now)
                next_normal[j] = max(next_normal[j] + normal_period,
                                     now + normal_period * 0.5)
        if now >= next_static:                           # 固定边: 1Hz
            send_static(now)
            next_static = now + static_period
        # 睡到下一个快边 tick
        target = start + (int((now - start) / period) + 1) * period
        rest = target - time.time()
        if rest > 0:
            time.sleep(rest)
    out.flush()
    out.close()
    counts = ['edge%d=%d' % (e, seqs[e]) for e in range(num_edges)]
    print('published: ' + ', '.join(counts))
    rospy.signal_shutdown('done')


if __name__ == '__main__':
    main()
