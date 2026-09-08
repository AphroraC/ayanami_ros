#!/usr/bin/env python3
"""TF 同步压力测试离线分析 (多边多速率版)。

输入: 发布器 CSV (edge,seq,x,stamp_ns,wall) 与探针事件 CSV
(edge,x,core_stamp_ns,wall)。edge 编号: 0=固定边 (tf_static, 值恒定),
1=快边, 2..=普通边; 探针 edges 顺序与之一致。

判据:
- 动态边: 样本已同步 <=> 存在探针事件 (core_stamp_ns >= stamp_ns 且
  时刻 >= 发布时刻) —— 核心最新样本不旧于该发布样本 (突发合并时按数据
  新鲜度记账); 延迟 = 该事件时刻 - 发布时刻。
- 固定边: 静态变换无时间戳序列, 按值精确匹配 (x == x_i)。

排除预热期 (--warmup-s)。门限: 聚合同步率 > --rate-gate 且
mean/p95 延迟 < --latency-gate-ms; 另输出逐边分解。
"""

import argparse
import bisect
import csv
import json
import os


def read_csv(path):
    with open(path) as f:
        rows = list(csv.reader(f))
    return rows[0], rows[1:]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--publish-csv', default='/tmp/tf_sync_stress/publish.csv')
    parser.add_argument('--events-csv', default='/tmp/tf_sync_stress/events.csv')
    parser.add_argument('--out-json', default='/tmp/tf_sync_stress/result.json')
    parser.add_argument('--rate-gate', type=float, default=0.98)
    parser.add_argument('--latency-gate-ms', type=float, default=3.0)
    parser.add_argument('--warmup-s', type=float, default=0.5)
    parser.add_argument('--fixed-edge', type=int, default=0)
    args = parser.parse_args()

    pub_header, pub_rows = read_csv(args.publish_csv)
    assert pub_header == ['edge', 'seq', 'x', 'stamp_ns', 'wall'], pub_header
    ev_header, ev_rows = read_csv(args.events_csv)
    assert ev_header == ['edge', 'x', 'core_stamp_ns', 'wall'], ev_header

    # 事件按边分组、按时间排序
    events_by_edge = {}
    exact_x_by_edge = {}
    for r in ev_rows:
        e = int(r[0])
        events_by_edge.setdefault(e, []).append((int(r[2]), float(r[3])))
        exact_x_by_edge.setdefault(e, set()).add(float.hex(float(r[1])))
    for e in events_by_edge:
        events_by_edge[e].sort(key=lambda ev: ev[1])
    stamps_by_edge = {e: [ev[0] for ev in evs]
                      for e, evs in events_by_edge.items()}

    per_edge = {}
    latencies = []
    synced = 0
    total = 0
    exact_observed = 0
    missed_seqs = []
    warmup_end = None
    for r in pub_rows:
        edge = int(r[0])
        seq = int(r[1])
        x = float(r[2])
        stamp_ns = int(r[3])
        t_pub = float(r[4])
        if warmup_end is None:
            warmup_end = t_pub + args.warmup_s
        if t_pub < warmup_end:
            continue

        # 固定边 (静态变换): 值恒定, 逐发布覆盖无意义 —— 只校验存在性
        # (恒值在核心侧可精确观测一次), 不计入同步率/延迟分母。
        if edge == args.fixed_edge:
            if edge not in per_edge:
                present = float.hex(x) in exact_x_by_edge.get(edge, set())
                per_edge[edge] = {'total': 1,
                                  'synced': 1 if present else 0,
                                  'latencies': [],
                                  'static_present': present}
            continue

        total += 1
        pe = per_edge.setdefault(edge, {'total': 0, 'synced': 0,
                                        'latencies': []})
        pe['total'] += 1
        covered = False
        lat = None
        # 动态边: 核心新鲜度覆盖 —— 存在 core_stamp >= stamp 且时刻在发布
        # 之后的事件即视为已同步 (突发合并免疫); 延迟 = 事件时刻 - 发布时刻。
        evs = events_by_edge.get(edge, [])
        stamps = stamps_by_edge.get(edge, [])
        idx = bisect.bisect_left(stamps, stamp_ns)
        for core_stamp, wall in evs[idx:]:
            if wall >= t_pub:
                covered = True
                lat = max(0.0, (wall - t_pub) * 1000.0)
                break
        if float.hex(x) in exact_x_by_edge.get(edge, set()):
            exact_observed += 1
        if covered:
            synced += 1
            pe['synced'] += 1
            latencies.append(lat)
            pe['latencies'].append(lat)
        else:
            missed_seqs.append((edge, seq))

    def stats_of(lats):
        if not lats:
            return {'mean': float('nan'), 'p50': float('nan'),
                    'p95': float('nan'), 'max': float('nan')}
        s = sorted(lats)
        pick = lambda p: s[min(len(s) - 1, int(round(p * (len(s) - 1))))]
        return {'mean': sum(s) / len(s), 'p50': pick(0.5),
                'p95': pick(0.95), 'max': s[-1]}

    n = len(latencies)
    stats = {
        'published_gate': total,
        'synced': synced,
        'missed': total - synced,
        'sync_rate': (synced / total) if total else 0.0,
        'exact_observed_rate': (exact_observed / total) if total else 0.0,
        'latency_ms': stats_of(latencies),
        'per_edge': {str(e): {
            'total': d['total'], 'synced': d['synced'],
            'sync_rate': (d['synced'] / d['total']) if d['total'] else 0.0,
            'latency_ms': stats_of(d['latencies']),
            **({'static_present': d['static_present']}
               if 'static_present' in d else {}),
        } for e, d in sorted(per_edge.items())},
    }
    rate_pass = stats['sync_rate'] > args.rate_gate
    mean_pass = stats['latency_ms']['mean'] < args.latency_gate_ms
    p95_pass = stats['latency_ms']['p95'] < args.latency_gate_ms
    stats['gate'] = {
        'rate_gate': args.rate_gate,
        'latency_gate_ms': args.latency_gate_ms,
        'warmup_s': args.warmup_s,
        'rate_pass': rate_pass,
        'mean_pass': mean_pass,
        'p95_pass': p95_pass,
        'pass': rate_pass and mean_pass and p95_pass,
    }
    stats['missed_edge_seqs'] = missed_seqs[:50]

    os.makedirs(os.path.dirname(args.out_json), exist_ok=True)
    with open(args.out_json, 'w') as f:
        json.dump(stats, f, indent=2)

    print('gate_samples=%d synced=%d missed=%d sync_rate=%.2f%% '
          '(exact-value observation rate=%.2f%%, 诊断用)' % (
              total, synced, total - synced, stats['sync_rate'] * 100.0,
              stats['exact_observed_rate'] * 100.0))
    print('latency_ms: mean=%.3f p50=%.3f p95=%.3f max=%.3f' % (
        stats['latency_ms']['mean'], stats['latency_ms']['p50'],
        stats['latency_ms']['p95'], stats['latency_ms']['max']))
    for e, d in sorted(per_edge.items()):
        es = stats['per_edge'][str(e)]
        if 'static_present' in d:
            print('  edge%d (fixed/static): present=%s' % (
                e, d['static_present']))
            continue
        print('  edge%d: n=%d rate=%.2f%% lat mean=%.3f p95=%.3f max=%.3f' % (
            e, d['total'], es['sync_rate'] * 100.0,
            es['latency_ms']['mean'], es['latency_ms']['p95'],
            es['latency_ms']['max']))
    if missed_seqs:
        print('missed (edge,seq) up to 50:', missed_seqs[:50])
    print('gate: rate(%s) mean(%s) p95(%s) -> %s' % (
        'PASS' if rate_pass else 'FAIL', 'PASS' if mean_pass else 'FAIL',
        'PASS' if p95_pass else 'FAIL',
        'PASS' if stats['gate']['pass'] else 'FAIL'))
    raise SystemExit(0 if stats['gate']['pass'] else 1)


if __name__ == '__main__':
    main()
