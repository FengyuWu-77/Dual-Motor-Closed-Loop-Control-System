#!/usr/bin/env python3
import sys, csv, argparse, math
import matplotlib
matplotlib.use("Agg")  # 无图形界面环境也能画图
import matplotlib.pyplot as plt

# ========== 工具函数 ==========

def try_float(x, default=float('nan')):
    try:
        return float(x)
    except:
        return default

def parse_line(line):
    """
    支持格式：
    新（双电机）:
      CTL,<t_ms>,<t1>,<c1>,<u1>,<t2>,<c2>,<u2>,<kp>,<ki>
    旧（单电机）:
      CTL,<t_ms>,<mode>,<target>,<cps>,<KP>,<KI>,<err>,<u>

    统一返回:
      {
        't_ms': float,
        'm1_t','m1_c','m1_u',
        'm2_t','m2_c','m2_u',
        'kp','ki','mode'
      }
    """
    if not line.startswith("CTL,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 3:
        return None

    t_ms = try_float(parts[1], None)
    if t_ms is None:
        return None

    rec = dict(
        t_ms=t_ms,
        kp=float('nan'),
        ki=float('nan'),
        m1_t=None, m1_c=None, m1_u=float('nan'),
        m2_t=None, m2_c=None, m2_u=float('nan'),
        mode=None
    )

    n = len(parts)
    # 新双电机
    if n >= 10:
        rec['m1_t'] = try_float(parts[2])
        rec['m1_c'] = try_float(parts[3])
        rec['m1_u'] = try_float(parts[4])
        rec['m2_t'] = try_float(parts[5])
        rec['m2_c'] = try_float(parts[6])
        rec['m2_u'] = try_float(parts[7])
        rec['kp']   = try_float(parts[8])
        rec['ki']   = try_float(parts[9])
        return rec
    # 旧单电机
    elif n >= 5:
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

# ========== 多阶跃检测 & 指标计算 ==========

def extract_steps(series, eps=1e-9):
    """
    series: [(t_ms, target, cps, u), ...]

    逻辑：识别“plateau -> plateau”的阶跃。
    - 将同一方向的连续 ramp（比如 100→180→200）合并为一次阶跃：
        from_target = 阶跃前 plateau
        to_target   = ramp 结束后的 plateau（或最后值）
        t0          = target 第一次离开 plateau 的时刻
    - 不再把 ramp 中间的每个点当成独立阶跃。
    - 不处理上电 0->首目标；那部分在上层补。
    """
    steps = []
    if not series or len(series) < 2:
        return steps

    n = len(series)
    # 当前“已确认 plateau”的目标
    prev_plateau = series[0][1]
    i = 1

    while i < n:
        curr = series[i][1]
        delta = curr - series[i-1][1]

        # 还在 plateau（目标没明显变）
        if abs(delta) <= eps:
            prev_plateau = curr
            i += 1
            continue

        # 出现明显变化：认为是一次 ramp 的开始
        ramp_sign = 1.0 if delta > 0 else -1.0
        t0 = series[i][0]
        from_target = prev_plateau

        j = i
        last_tg = series[i][1]

        while j + 1 < n:
            step = series[j+1][1] - series[j][1]

            # 同方向连续变化：继续 ramp
            if step * ramp_sign > eps:
                last_tg = series[j+1][1]
                j += 1
                continue

            # 变化结束：进入新 plateau（|Δ| 很小）
            if abs(step) <= eps:
                k = j + 1
                # 吃掉后续 plateau 上的小波动
                while k + 1 < n and abs(series[k+1][1] - series[k][1]) <= eps:
                    k += 1
                last_tg = series[k][1]
                j = k
                break

            # 方向反转或者乱跳：认为 ramp 结束
            break

        to_target = last_tg

        # 建立这个阶跃的响应段（从 t0 开始到 j）
        if abs(to_target - from_target) > eps:
            seg = [(t, tg, c) for (t, tg, c, _u) in series[i:j+1]]
            if seg:
                steps.append(dict(
                    t0=t0,
                    from_target=from_target,
                    to_target=to_target,
                    seg=seg
                ))

        # 更新 plateau，继续往后扫
        prev_plateau = to_target
        i = j + 1

    return steps

def compute_step_metrics(step, settle_pct=0.03, settle_hold_ms=500):
    """
    对单个阶跃段计算指标：
      overshoot_pct, steady_state_error_pct, settling_time_s 等
    step: { 't0', 'from_target', 'to_target', 'seg': [(t, target, cps), ...] }
    """
    seg = step['seg']
    if not seg:
        return dict(error="no_data")

    t0 = seg[0][0]
    from_target = step['from_target']
    to_target = step['to_target']

    cps_vals = [c for (_t, _tg, c) in seg]
    max_cps = max(cps_vals)
    min_cps = min(cps_vals)

    # ---- Overshoot ----
    overshoot_pct = 0.0
    if abs(to_target) > 1e-9:
        step_dir = 1.0 if (to_target - from_target) >= 0 else -1.0
        if step_dir > 0:
            if max_cps > to_target:
                overshoot_pct = (max_cps - to_target) / abs(to_target) * 100.0
        else:
            if min_cps < to_target:
                overshoot_pct = (to_target - min_cps) / abs(to_target) * 100.0

    # ---- 稳态（末尾 N 点均值）----
    N = max(5, min(20, max(5, int(0.1 * len(seg)))))
    ss_slice = seg[-N:]
    ss_cps = sum(c for (_t, _tg, c) in ss_slice) / len(ss_slice)

    if abs(to_target) > 1e-9:
        ss_err_pct = abs(ss_cps - to_target) / abs(to_target) * 100.0
        band = settle_pct * abs(to_target)
    else:
        scale = max(1.0, max(abs(c) for c in cps_vals))
        band = settle_pct * scale
        ss_err_pct = abs(ss_cps - to_target) / scale * 100.0

    band_lo, band_hi = to_target - band, to_target + band

    # ---- 整定时间：从 t0 起，进入带宽并保持 settle_hold_ms ----
    t_settle_abs = None
    hold_start = None
    for (t, _tg, c) in seg:
        inside = (band_lo <= c <= band_hi)
        if inside:
            if hold_start is None:
                hold_start = t
            if (t - hold_start) >= settle_hold_ms:
                t_settle_abs = hold_start
                break
        else:
            hold_start = None

    if t_settle_abs is not None:
        settling_time_ms = int(t_settle_abs - t0)
        settling_time_s = settling_time_ms / 1000.0
    else:
        settling_time_ms = None
        settling_time_s = None

    return dict(
        t0_ms=int(t0),
        t0_s=t0 / 1000.0,
        from_target=from_target,
        to_target=to_target,
        max_cps=max_cps,
        min_cps=min_cps,
        overshoot_pct=overshoot_pct,
        steady_state_cps=ss_cps,
        steady_state_error_pct=ss_err_pct,
        settle_band_pct=settle_pct * 100.0,
        settle_hold_ms=int(settle_hold_ms),
        settling_time_ms=settling_time_ms,
        settling_time_s=settling_time_s,
        band_lo=band_lo,
        band_hi=band_hi
    )

# ========== 主逻辑 ==========

def main():
    ap = argparse.ArgumentParser(
        description="Analyze CTL logs (dual-motor, multi-step), export CSV + plots + metrics."
    )
    ap.add_argument("logfile", help="path to your txt log")
    ap.add_argument("--out", default="analysis.csv",
                    help="output time-series CSV path (default: analysis.csv)")
    ap.add_argument("--results", default="results.csv",
                    help="output metrics CSV path (default: results.csv)")
    ap.add_argument("--settle_pct", "--band", dest="settle_pct",
                    type=float, default=0.03,
                    help="settling band as fraction (e.g., 0.02 = 2%)")
    ap.add_argument("--hold_ms", "--hold", dest="hold_ms",
                    type=int, default=500,
                    help="hold time inside band for settling (ms)")
    ap.add_argument("--spread_png", default="plot_spread.png",
                    help="output PNG for speed response")
    ap.add_argument("--u_png", default="plot_U.png",
                    help="output PNG for control output")
    args = ap.parse_args()

    # ---- 读取日志 ----
    recs = []
    with open(args.logfile, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            p = parse_line(line.strip())
            if p:
                recs.append(p)

    if not recs:
        print("No CTL lines found.")
        sys.exit(1)

    recs.sort(key=lambda r: r['t_ms'])

    # ---- 构造时间序列 ----
    m1_series = []
    m2_series = []
    for r in recs:
        if r['m1_t'] is not None and r['m1_c'] is not None:
            m1_series.append((r['t_ms'], r['m1_t'], r['m1_c'], r['m1_u']))
        if r['m2_t'] is not None and r['m2_c'] is not None:
            m2_series.append((r['t_ms'], r['m2_t'], r['m2_c'], r['m2_u']))

    if not m1_series and not m2_series:
        print("No motor data found (neither m1 nor m2).")
        sys.exit(1)

    EPS = 1e-9
    results_rows = []

    # ---- 结合“上电 0->首目标” + plateau 阶跃 ----

    def build_steps_with_initial(series):
        if not series:
            return []
        steps_all = []

        first_target = series[0][1]

        # 人工首阶跃：0 -> first_target（如果首值本身就非 0）
        if abs(first_target) > EPS:
            j = 0
            while j + 1 < len(series) and abs(series[j+1][1] - first_target) <= EPS:
                j += 1
            seg0 = [(t, tg, c) for (t, tg, c, _u) in series[0:j+1]]
            if seg0:
                steps_all.append(dict(
                    t0=series[0][0],
                    from_target=0.0,
                    to_target=first_target,
                    seg=seg0
                ))

        # 后续阶跃（基于 plateau->plateau，合并 ramp）
        rest = extract_steps(series)
        steps_all.extend(rest)

        # 如果啥都没检测到，但有数据：整体看作 0 -> 最终 target
        if not steps_all and len(series) >= 2:
            to_t = series[-1][1]
            if abs(to_t) > EPS:
                seg = [(t, tg, c) for (t, tg, c, _u) in series]
                steps_all.append(dict(
                    t0=series[0][0],
                    from_target=0.0,
                    to_target=to_t,
                    seg=seg
                ))
        return steps_all

    def process_motor(name, series):
        nonlocal results_rows
        if not series:
            return
        steps = build_steps_with_initial(series)
        for idx, st in enumerate(steps, start=1):
            m = compute_step_metrics(st, args.settle_pct, args.hold_ms)
            results_rows.append(dict(
                motor=name,
                step_index=idx,
                t0_ms=m.get('t0_ms'),
                t0_s=m.get('t0_s'),
                from_target=m.get('from_target'),
                to_target=m.get('to_target'),
                max_cps=m.get('max_cps'),
                min_cps=m.get('min_cps'),
                overshoot_pct=m.get('overshoot_pct'),
                steady_state_cps=m.get('steady_state_cps'),
                steady_state_error_pct=m.get('steady_state_error_pct'),
                settle_band_pct=m.get('settle_band_pct'),
                settle_hold_ms=m.get('settle_hold_ms'),
                settling_time_ms=m.get('settling_time_ms'),
                settling_time_s=m.get('settling_time_s')
            ))

    process_motor("M1", m1_series)
    process_motor("M2", m2_series)

    # ---- 写 results.csv ----
    if results_rows:
        with open(args.results, "w", newline="") as fr:
            fieldnames = [
                "motor", "step_index",
                "t0_ms", "t0_s",
                "from_target", "to_target",
                "max_cps", "min_cps",
                "overshoot_pct",
                "steady_state_cps", "steady_state_error_pct",
                "settle_band_pct", "settle_hold_ms",
                "settling_time_ms", "settling_time_s"
            ]
            w = csv.DictWriter(fr, fieldnames=fieldnames)
            w.writeheader()
            for row in results_rows:
                w.writerow(row)
    else:
        print("Warning: no steps detected for metrics (results.csv will be empty).")

    # ---- 写 metrics.txt（人类可读）----
    if results_rows:
        lines = ["== Step Response Metrics =="]

        def fmt(v, nd=3):
            if v is None:
                return "N/A"
            if isinstance(v, float) and (math.isnan(v) or math.isinf(v)):
                return "N/A"
            return f"{v:.{nd}f}"

        for row in results_rows:
            motor = row["motor"]
            idx = row["step_index"]
            t0_s = row["t0_s"]
            ft = row["from_target"]
            tt = row["to_target"]
            ov = row["overshoot_pct"]
            se = row["steady_state_error_pct"]
            st = row["settling_time_s"]

            line = (
                f"[{motor} step{idx}] "
                f"t0={fmt(t0_s,3)} s, "
                f"{fmt(ft,1)} -> {fmt(tt,1)}, "
                f"overshoot={fmt(ov,2)} %, "
                f"steady_err={fmt(se,2)} %, "
                f"settle_time={fmt(st,3)} s"
            )
            lines.append(line)

        with open("metrics.txt", "w", encoding="utf-8") as f:
            f.write("\n".join(lines))

    # ---- analysis.csv：时序 + 带宽 ----

    def collect_steps_info(series):
        steps = build_steps_with_initial(series)
        info = []
        for st in steps:
            m = compute_step_metrics(st, args.settle_pct, args.hold_ms)
            info.append((st, m))
        return info

    def build_band_map(times, steps_info):
        band_map_lo = {}
        band_map_hi = {}
        for st, m in steps_info:
            lo = m.get("band_lo")
            hi = m.get("band_hi")
            if lo is None or hi is None:
                continue
            seg = st.get("seg") or []
            if not seg:
                continue
            t_start = seg[0][0]
            t_end = seg[-1][0]
            for t in times:
                if t_start <= t <= t_end:
                    band_map_lo[t] = lo
                    band_map_hi[t] = hi
        return band_map_lo, band_map_hi

    with open(args.out, "w", newline="") as fout:
        fieldnames = [
            "t_ms",
            "t1", "c1", "u1", "band1_lo", "band1_hi",
            "t2", "c2", "u2", "band2_lo", "band2_hi"
        ]
        w = csv.DictWriter(fout, fieldnames=fieldnames)
        w.writeheader()

        times = sorted(set(r["t_ms"] for r in recs))
        m1_by_t = {t: (tg, c, u) for (t, tg, c, u) in m1_series}
        m2_by_t = {t: (tg, c, u) for (t, tg, c, u) in m2_series}

        steps1_info = collect_steps_info(m1_series) if m1_series else []
        steps2_info = collect_steps_info(m2_series) if m2_series else []

        b1_lo, b1_hi = build_band_map(times, steps1_info) if steps1_info else ({}, {})
        b2_lo, b2_hi = build_band_map(times, steps2_info) if steps2_info else ({}, {})

        last1 = {"tg": "", "c": "", "u": ""}
        last2 = {"tg": "", "c": "", "u": ""}

        for t in times:
            row = {
                "t_ms": int(t),
                "t1": "", "c1": "", "u1": "", "band1_lo": "", "band1_hi": "",
                "t2": "", "c2": "", "u2": "", "band2_lo": "", "band2_hi": ""
            }

            if t in m1_by_t:
                tg, c, u = m1_by_t[t]
                last1 = {"tg": tg, "c": c, "u": u}
            if t in m2_by_t:
                tg, c, u = m2_by_t[t]
                last2 = {"tg": tg, "c": c, "u": u}

            if last1["tg"] != "":
                row["t1"] = last1["tg"]
                row["c1"] = last1["c"]
                row["u1"] = last1["u"]
                if t in b1_lo:
                    row["band1_lo"] = b1_lo[t]
                    row["band1_hi"] = b1_hi[t]

            if last2["tg"] != "":
                row["t2"] = last2["tg"]
                row["c2"] = last2["c"]
                row["u2"] = last2["u"]
                if t in b2_lo:
                    row["band2_lo"] = b2_lo[t]
                    row["band2_hi"] = b2_hi[t]

            w.writerow(row)

    # ---- 画图：转速响应 ----
    plt.figure(figsize=(10, 6))

    if m1_series:
        ts = [t / 1000.0 for (t, _, _, _) in m1_series]
        tgs = [tg for (_, tg, _, _) in m1_series]
        cs  = [c for (_, _, c, _) in m1_series]
        plt.plot(ts, tgs, linestyle="--", label="M1 target")
        plt.plot(ts, cs,  linestyle="-",  label="M1 cps")

    if m2_series:
        ts2  = [t / 1000.0 for (t, _, _, _) in m2_series]
        tgs2 = [tg for (_, tg, _, _) in m2_series]
        cs2  = [c for (_, _, c, _) in m2_series]
        plt.plot(ts2, tgs2, linestyle="--", label="M2 target")
        plt.plot(ts2, cs2,  linestyle="-",  label="M2 cps")

    plt.xlabel("Time (s)")
    plt.ylabel("Speed (cps)")
    plt.title("Motor Speed Response")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.spread_png, dpi=200)
    plt.close()

    # ---- 画图：控制输出 U ----
    plt.figure(figsize=(10, 4))

    if m1_series:
        ts = [t / 1000.0 for (t, _, _, _) in m1_series]
        u1 = [u for (_, _, _, u) in m1_series]
        plt.plot(ts, u1, label="M1 u")

    if m2_series:
        ts2 = [t / 1000.0 for (t, _, _, _) in m2_series]
        u2 = [u for (_, _, _, u) in m2_series]
        plt.plot(ts2, u2, label="M2 u")

    plt.xlabel("Time (s)")
    plt.ylabel("PWM output")
    plt.title("Motor Control Output")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()
    plt.savefig(args.u_png, dpi=200)
    plt.close()

    print(f"Saved time-series CSV: {args.out}")
    print(f"Saved metrics CSV: {args.results}")
    if results_rows:
        print("Saved metrics summary: metrics.txt")
    print(f"Saved speed plot: {args.spread_png}")
    print(f"Saved PWM plot: {args.u_png}")

if __name__ == "__main__":
    main()