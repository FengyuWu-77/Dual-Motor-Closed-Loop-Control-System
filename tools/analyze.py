#!/usr/bin/env python3
import sys, csv, argparse, math

def try_float(x, default=float('nan')):
    try:
        return float(x)
    except:
        return default

def parse_line(line):
    """
    兼容两种格式：
    1) 旧：CTL,<t_ms>,<mode>,<target>,<cps>,<KP>,<KI>,<err>,<u>
    2) 新（双电机）：CTL,<t_ms>,<t1>,<c1>,<u1>,<t2>,<c2>,<u2>,<kp>,<ki>
    返回统一字典：
    {
      't_ms': float,
      # 单电机时：仅有 m1_* 字段
      'm1_t': float, 'm1_c': float, 'm1_u': float,
      'm2_t': float or None, 'm2_c': float or None, 'm2_u': float or None,
      'kp': float or nan, 'ki': float or nan,
      'mode': int or None   # 旧格式可用
    }
    """
    if not line.startswith("CTL,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 3:
        return None

    # 公共时间
    t_ms = try_float(parts[1], None)
    if t_ms is None:
        return None

    rec = dict(t_ms=t_ms, kp=float('nan'), ki=float('nan'),
               m1_t=None, m1_c=None, m1_u=float('nan'),
               m2_t=None, m2_c=None, m2_u=float('nan'),
               mode=None)

    # 判断长度
    # 旧：CTL, t, mode, target, cps, kp, ki, err, u   -> len=9
    # 新：CTL, t, t1, c1, u1, t2, c2, u2, kp, ki      -> len=10
    n = len(parts)
    if n >= 10:
        # 新双电机
        rec['m1_t'] = try_float(parts[2])
        rec['m1_c'] = try_float(parts[3])
        rec['m1_u'] = try_float(parts[4])
        rec['m2_t'] = try_float(parts[5])
        rec['m2_c'] = try_float(parts[6])
        rec['m2_u'] = try_float(parts[7])
        rec['kp']   = try_float(parts[8])
        rec['ki']   = try_float(parts[9])
        return rec
    elif n >= 5:
        # 旧单电机（尽量解析）
        mode = try_float(parts[2], 1.0)
        rec['mode'] = int(mode) if not math.isnan(mode) else None
        rec['m1_t'] = try_float(parts[3])
        rec['m1_c'] = try_float(parts[4])
        rec['m1_u'] = try_float(parts[8]) if n > 8 else float('nan')
        rec['kp']   = try_float(parts[5]) if n > 5 else float('nan')
        rec['ki']   = try_float(parts[6]) if n > 6 else float('nan')
        return rec
    else:
        return None

def detect_step(series, eps=1e-9):
    """
    series: list of (t_ms, target, cps)
    返回：(t0, target_final, samples_after_t0, all_samples)
    策略：
      - 以首个样本的 target 为基准，检测第一次 target 发生变化的时间为 t0；
      - 若没有变化，则 t0 = 首样本时间；
      - target_final = 末样本的 target。
    """
    if not series:
        return None, None, [], []
    all_samples = series
    t_start = series[0][0]
    t0 = t_start
    prev = series[0][1]
    for (t, targ, _c) in series[1:]:
        if abs(targ - prev) > eps:
            t0 = t
            break
        prev = targ
    target_final = series[-1][1]
    after = [(t, targ, c) for (t, targ, c) in series if t >= t0]
    return t0, target_final, after, all_samples

def compute_metrics(after, target_final, settle_pct=0.02, settle_hold_ms=500):
    """
    after: [(t, target, cps)] 从 t0 及之后的样本
    返回指标 dict，以及 (band_lo, band_hi)
    """
    if not after:
        return dict(error="No samples after t0"), (None, None)

    cps_vals = [c for (_t, _tar, c) in after]
    max_cps = max(cps_vals)
    min_cps = min(cps_vals)

    overshoot_pct = 0.0
    if target_final and target_final != 0:
        overshoot_pct = max(0.0, (max_cps - target_final) / abs(target_final) * 100.0)

    # 稳态：末尾 10% 或最多 20 个样本（不少于 5）
    N = max(5, min(20, int(0.1 * len(after)) if len(after) >= 10 else 5))
    ss_slice = after[-N:]
    ss_cps = sum(c for (_t, _tar, c) in ss_slice) / len(ss_slice)
    ss_err_pct = 0.0 if target_final == 0 else abs(ss_cps - target_final) / abs(target_final) * 100.0

    band = settle_pct * abs(target_final)
    band_lo, band_hi = (target_final - band, target_final + band)

    # 整定时间：进入带宽并连续保持 settle_hold_ms
    t_settle = None
    hold_start = None
    for (t, _tar, c) in after:
        inside = (band_lo <= c <= band_hi)
        if inside:
            if hold_start is None:
                hold_start = t
            if t - hold_start >= settle_hold_ms:
                t_settle = hold_start
                break
        else:
            hold_start = None

    return dict(
        target_final=target_final,
        max_cps=max_cps,
        min_cps=min_cps,
        overshoot_pct=overshoot_pct,
        steady_state_cps=ss_cps,
        steady_state_error_pct=ss_err_pct,
        settle_band_pct=settle_pct * 100.0,
        settle_hold_ms=int(settle_hold_ms),
        settling_time_ms=int(t_settle) if t_settle is not None else None
    ), (band_lo, band_hi)

def main():
    ap = argparse.ArgumentParser(description="Analyze CTL logs (dual-motor supported) and export CSV + print metrics.")
    ap.add_argument("logfile", help="path to your txt log")
    ap.add_argument("--out", default="analysis.csv", help="output CSV path (default: analysis.csv)")
    ap.add_argument("--settle_pct", "--band", dest="settle_pct", type=float, default=0.03,
                    help="settling band as fraction (e.g., 0.02 = 2%%)")
    ap.add_argument("--hold_ms", "--hold", dest="hold_ms", type=int, default=500,
                    help="hold time inside band to declare settled (ms)")
    args = ap.parse_args()

    # 读取
    recs = []
    with open(args.logfile, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            p = parse_line(line.strip())
            if p:
                recs.append(p)

    if not recs:
        print("No CTL lines found.")
        sys.exit(1)

    # 按时间排序（保险）
    recs.sort(key=lambda r: r['t_ms'])

    # 构造两路序列（存在即加入）
    m1_series = []
    m2_series = []
    for r in recs:
        if r['m1_t'] is not None and r['m1_c'] is not None:
            m1_series.append( (r['t_ms'], r['m1_t'], r['m1_c'], r['m1_u']) )
        if r['m2_t'] is not None and r['m2_c'] is not None:
            m2_series.append( (r['t_ms'], r['m2_t'], r['m2_c'], r['m2_u']) )

    if not m1_series and not m2_series:
        print("No motor data found (neither m1 nor m2).")
        sys.exit(1)

    # 检测阶跃并计算指标
    results = {}
    bands = {}
    t0s = {}
    finals = {}

    if m1_series:
        t0_1, tf_1, after_1, _all1 = detect_step([(t,tg,c) for (t,tg,c,_u) in m1_series])
        metrics1, band1 = compute_metrics(after_1, tf_1, args.settle_pct, args.hold_ms)
        results['M1'] = metrics1
        bands['M1'] = band1
        t0s['M1'] = t0_1
        finals['M1'] = tf_1

    if m2_series:
        t0_2, tf_2, after_2, _all2 = detect_step([(t,tg,c) for (t,tg,c,_u) in m2_series])
        metrics2, band2 = compute_metrics(after_2, tf_2, args.settle_pct, args.hold_ms)
        results['M2'] = metrics2
        bands['M2'] = band2
        t0s['M2'] = t0_2
        finals['M2'] = tf_2

    # 写 CSV：统一导出两路曲线 + 两路带宽
    # 字段：t_ms, t1, c1, u1, band1_lo, band1_hi, t2, c2, u2, band2_lo, band2_hi
    with open(args.out, "w", newline="") as fout:
        fieldnames = ["t_ms",
                      "t1", "c1", "u1", "band1_lo", "band1_hi",
                      "t2", "c2", "u2", "band2_lo", "band2_hi"]
        w = csv.DictWriter(fout, fieldnames=fieldnames)
        w.writeheader()

        # 建立时间索引并合并
        # 由于两路时间戳一致来源，直接按 recs 遍历即可
        # 用最近值/空白填充
        i1 = i2 = 0
        last1 = {'t':"", 'c':"", 'u':"", 'tg':""}
        last2 = {'t':"", 'c':"", 'u':"", 'tg':""}
        band1_lo = band1_hi = ""
        band2_lo = band2_hi = ""

        # 预置带宽：当 t >= 对应 t0 才填入数值
        t0_1 = t0s.get('M1', None)
        t0_2 = t0s.get('M2', None)
        b1 = bands.get('M1', (None, None))
        b2 = bands.get('M2', (None, None))

        # 用一个时间集合
        times = sorted(set([r['t_ms'] for r in recs]))
        # 建索引字典，便于按时间取某路的样本
        m1_by_t = {t:(tg,c,u) for (t,tg,c,u) in m1_series}
        m2_by_t = {t:(tg,c,u) for (t,tg,c,u) in m2_series}

        for t in times:
            row = {"t_ms": int(t),
                   "t1":"", "c1":"", "u1":"", "band1_lo":"", "band1_hi":"",
                   "t2":"", "c2":"", "u2":"", "band2_lo":"", "band2_hi":""}

            if t in m1_by_t:
                tg,c,u = m1_by_t[t]
                last1 = {'t':tg, 'c':c, 'u':u, 'tg':tg}
            if t in m2_by_t:
                tg,c,u = m2_by_t[t]
                last2 = {'t':tg, 'c':c, 'u':u, 'tg':tg}

            if last1['tg'] != "":
                row["t1"] = last1['tg']
                row["c1"] = last1['c']
                row["u1"] = last1['u']
                if (t0_1 is not None) and (t >= t0_1) and (b1[0] is not None):
                    row["band1_lo"] = b1[0]
                    row["band1_hi"] = b1[1]

            if last2['tg'] != "":
                row["t2"] = last2['tg']
                row["c2"] = last2['c']
                row["u2"] = last2['u']
                if (t0_2 is not None) and (t >= t0_2) and (b2[0] is not None):
                    row["band2_lo"] = b2[0]
                    row["band2_hi"] = b2[1]

            w.writerow(row)

    # 打印指标
    print("== Metrics ==")
    if 'M1' in results:
        print("[Motor 1]")
        print(f"t0_ms: {int(t0s['M1']) if t0s['M1'] is not None else None}")
        for k, v in results['M1'].items():
            print(f"{k}: {v}")
        print()
    if 'M2' in results:
        print("[Motor 2]")
        print(f"t0_ms: {int(t0s['M2']) if t0s['M2'] is not None else None}")
        for k, v in results['M2'].items():
            print(f"{k}: {v}")
        print()

    print(f"Saved time-series CSV: {args.out}")

if __name__ == "__main__":
    main()