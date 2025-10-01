#!/usr/bin/env python3
import sys, csv, argparse

def parse_line(line):
    # 期望格式: CTL,<t_ms>,<mode>,<target>,<cps>,<KP>,<KI>,<e/e_prev>,<u>
    if not line.startswith("CTL,"):
        return None
    parts = [p.strip() for p in line.split(",")]
    if len(parts) < 5:
        return None
    try:
        t_ms   = float(parts[1])
        mode   = int(float(parts[2]))  # -1 boot, 1 closed
        target = float(parts[3])
        cps    = float(parts[4])
        kp     = float(parts[5]) if len(parts) > 5 else float('nan')
        ki     = float(parts[6]) if len(parts) > 6 else float('nan')
        err    = float(parts[7]) if len(parts) > 7 else float('nan')
        u      = float(parts[8]) if len(parts) > 8 else float('nan')
        return dict(t_ms=t_ms, mode=mode, target=target, cps=cps, kp=kp, ki=ki, err=err, u=u)
    except ValueError:
        return None

def detect_step(records, eps=1e-6):
    """
    找阶跃起始 t0 与最终目标 target_final：
    - 取 mode==1（闭环）后的数据；
    - 在闭环数据中第一次检测到 target 发生变化的时间作为 t0；
    - 若未检测到变化，则 t0=闭环首样本时间；target_final=闭环末样本的 target。
    """
    closed = [r for r in records if r['mode'] == 1]
    if not closed:
        return None, None, None
    t_start = closed[0]['t_ms']
    prev = closed[0]['target']
    t0 = t_start
    for r in closed[1:]:
        if abs(r['target'] - prev) > eps:
            t0 = r['t_ms']
            break
        prev = r['target']
    target_final = closed[-1]['target']
    return t0, target_final, closed

def compute_metrics(closed, t0, target_final, settle_pct=0.02, settle_hold_ms=500):
    """
    返回指标 + 导出行（时间序列）。
    移除了 undershoot（欠冲），仅保留 overshoot、稳态、整定时间等。
    """
    if t0 is None:
        return dict(error="No step/closed-loop found"), []

    after = [r for r in closed if r['t_ms'] >= t0]
    if not after:
        return dict(error="No samples after t0"), []

    cps_vals = [r['cps'] for r in after]
    max_cps = max(cps_vals)
    min_cps = min(cps_vals)  # 仅用于参考，不再计算欠冲

    # Overshoot（超调）
    overshoot_pct = 0.0
    if target_final != 0:
        overshoot_pct = max(0.0, (max_cps - target_final) / abs(target_final) * 100.0)

    # 稳态值：末尾 10% 或最多 20 个样本（不少于 5）
    N = max(5, min(20, int(0.1 * len(after)) if len(after) >= 10 else 5))
    ss_slice = after[-N:]
    ss_cps = sum(r['cps'] for r in ss_slice) / len(ss_slice)
    ss_err_pct = 0.0 if target_final == 0 else abs(ss_cps - target_final) / abs(target_final) * 100.0

    # 整定时间：进入 ±settle_pct 带并连续保持 settle_hold_ms
    band = settle_pct * abs(target_final)
    band_lo, band_hi = target_final - band, target_final + band

    t_settle = None
    hold_start = None
    for r in after:
        inside = (band_lo <= r['cps'] <= band_hi)
        if inside:
            if hold_start is None:
                hold_start = r['t_ms']
            if r['t_ms'] - hold_start >= settle_hold_ms:
                t_settle = hold_start
                break
        else:
            hold_start = None

    # 导出曲线：闭环段全量
    out_rows = []
    for r in closed:
        out_rows.append({
            "t_ms": int(r['t_ms']),
            "target": r['target'],
            "cps": r['cps'],
            "band_lo": band_lo if r['t_ms'] >= t0 else "",
            "band_hi": band_hi if r['t_ms'] >= t0 else ""
        })

    metrics = dict(
        t0_ms=int(t0),
        target_final=target_final,
        max_cps=max_cps,
        min_cps=min_cps,  # 仅展示
        overshoot_pct=overshoot_pct,
        steady_state_cps=ss_cps,
        steady_state_error_pct=ss_err_pct,
        settle_band_pct=settle_pct * 100.0,
        settle_hold_ms=int(settle_hold_ms),
        settling_time_ms=int(t_settle) if t_settle is not None else None
    )
    return metrics, out_rows

def main():
    ap = argparse.ArgumentParser(description="Analyze CTL logs and export CSV + print metrics.")
    ap.add_argument("logfile", help="path to your txt log")
    ap.add_argument("--out", default="analysis.csv", help="output CSV path (default: analysis.csv)")
    # 支持两套参数名：--settle_pct/--band, --hold_ms/--hold
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

    t0, target_final, closed = detect_step(recs)
    if t0 is None:
        print("No closed-loop data found.")
        sys.exit(1)

    metrics, rows = compute_metrics(closed, t0, target_final, args.settle_pct, args.hold_ms)

    # 写 CSV
    with open(args.out, "w", newline="") as fout:
        w = csv.DictWriter(fout, fieldnames=["t_ms", "target", "cps", "band_lo", "band_hi"])
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # 打印指标
    print("== Metrics ==")
    for k, v in metrics.items():
        print(f"{k}: {v}")
    print(f"\nSaved time-series CSV: {args.out}")

if __name__ == "__main__":
    main()