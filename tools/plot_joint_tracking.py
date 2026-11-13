#!/usr/bin/env python3
"""Vẽ + định lượng độ bám của joint_states.position so với joint_cmd.position.

Đầu vào: thư mục do ControlModule ghi khi chạy với  MYBIPEDAL_LOG_DIR=<dir>
    joint_cmd.csv    t_s, cmd_<joint>..., vx, vy, wz, ctrl_id
    joint_state.csv  t_s, pos_<joint>..., vel_<joint>..., eff_<joint>...
    meta.txt         (tuỳ chọn) joints=..., ctrl_ids=0:pd_idle,...

Ví dụ:
    python3 tools/plot_joint_tracking.py ./log/trace1 --walk
    python3 tools/plot_joint_tracking.py ./log/trace1 --t0 30 --t1 40 --show
"""
import argparse
import os
import sys

import numpy as np
import pandas as pd

LEG_ROWS = ["hip_pitch", "hip_roll", "hip_yaw", "knee", "ankle_pitch"]


def read_meta(path):
    meta = {}
    p = os.path.join(path, "meta.txt")
    if os.path.exists(p):
        for line in open(p):
            if "=" in line:
                k, v = line.strip().split("=", 1)
                meta[k] = v
    ctrl = {}
    for item in filter(None, meta.get("ctrl_ids", "").split(",")):
        i, name = item.split(":", 1)
        ctrl[int(i)] = name
    meta["ctrl_map"] = ctrl
    return meta


def uniform_grid(t, y, t0, t1, dt):
    tg = np.arange(t0, t1, dt)
    ok = np.isfinite(y)
    return tg, np.interp(tg, t[ok], y[ok])


def tracking_metrics(t_cmd, cmd, t_st, st, t0, t1, max_lag_ms=150, min_lag_ms=-20):
    """Trả về dict: rmse, max_err, bias, gain, lag_ms, corr, p2p_ratio (trên cửa sổ [t0,t1])."""
    dt = 1e-3
    tg, c = uniform_grid(t_cmd, cmd, t0, t1, dt)
    _, s = uniform_grid(t_st, st, t0, t1, dt)
    if len(tg) < 200:
        return None
    err = s - c
    cd, sd = c - c.mean(), s - s.mean()
    var_c = float(np.dot(cd, cd))
    gain = float(np.dot(cd, sd) / var_c) if var_c > 1e-12 else np.nan
    corr = float(np.dot(cd, sd) / np.sqrt(var_c * np.dot(sd, sd))) if var_c > 1e-12 and np.dot(sd, sd) > 1e-12 else np.nan

    # lag: dịch state đi L mili-giây để khớp với cmd nhất (L>0: state TRỄ hơn cmd)
    best_lag, best_val = 0, -np.inf
    if var_c > 1e-12:
        for L in range(int(min_lag_ms), int(max_lag_ms) + 1):
            if L >= 0:
                a, b = cd[: len(cd) - L], sd[L:]
            else:
                a, b = cd[-L:], sd[: len(sd) + L]
            v = float(np.dot(a, b)) / max(len(a), 1)
            if v > best_val:
                best_val, best_lag = v, L
    p2p_c, p2p_s = np.ptp(c), np.ptp(s)
    return dict(
        rmse=float(np.sqrt(np.mean(err**2))),
        max_err=float(np.max(np.abs(err))),
        bias=float(err.mean()),
        gain=gain,
        lag_ms=float(best_lag) if var_c > 1e-12 else np.nan,
        corr=corr,
        p2p_ratio=float(p2p_s / p2p_c) if p2p_c > 1e-9 else np.nan,
    )


def joint_layout(joints):
    """Sắp xếp 5 hàng x 2 cột (trái/phải) nếu tên chuẩn, ngược lại lưới tự động."""
    grid = []
    for base in LEG_ROWS:
        row = [f"{side}_{base}_joint" for side in ("left", "right")]
        if all(j in joints for j in row):
            grid.append(row)
    if grid and sum(len(r) for r in grid) == len(joints):
        return grid
    rows = []
    for i in range(0, len(joints), 2):
        rows.append(joints[i : i + 2])
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logdir")
    ap.add_argument("--t0", type=float, default=None, help="bắt đầu cửa sổ (s, theo cột t_s)")
    ap.add_argument("--t1", type=float, default=None, help="kết thúc cửa sổ (s)")
    ap.add_argument("--walk", action="store_true", help="chỉ lấy đoạn controller rl_* đang active (theo ctrl_id)")
    ap.add_argument("--ctrl", type=str, default=None, help="chỉ lấy đoạn có controller cuối = tên này (vd rl_walk)")
    ap.add_argument("--out", type=str, default=None, help="thư mục lưu PNG (mặc định: logdir)")
    ap.add_argument("--show", action="store_true", help="hiện cửa sổ matplotlib")
    args = ap.parse_args()

    import matplotlib

    if not args.show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out = args.out or args.logdir
    os.makedirs(out, exist_ok=True)
    meta = read_meta(args.logdir)
    cmd = pd.read_csv(os.path.join(args.logdir, "joint_cmd.csv"))
    st = pd.read_csv(os.path.join(args.logdir, "joint_state.csv"))

    joints = [c[4:] for c in cmd.columns if c.startswith("cmd_")]
    t_cmd, t_st = cmd["t_s"].to_numpy(), st["t_s"].to_numpy()

    # ---- chọn cửa sổ thời gian ----
    t0 = max(t_cmd[0], t_st[0]) if args.t0 is None else args.t0
    t1 = min(t_cmd[-1], t_st[-1]) if args.t1 is None else args.t1
    want = args.ctrl
    if args.walk and not want:
        cands = [n for n in meta["ctrl_map"].values() if n.startswith("rl_")]
        want = cands[0] if cands else None
    if want:
        ids = [i for i, n in meta["ctrl_map"].items() if n == want]
        if not ids:
            sys.exit(f"Không thấy controller '{want}' trong meta.txt: {meta['ctrl_map']}")
        mask = (cmd["ctrl_id"].to_numpy() == ids[0]) & (t_cmd >= t0) & (t_cmd <= t1)
        if mask.sum() < 200:
            sys.exit(f"Chỉ có {mask.sum()} mẫu ở controller '{want}' — robot có vào chế độ walk không?")
        t0, t1 = t_cmd[mask][0], t_cmd[mask][-1]
        print(f"[cửa sổ] controller={want}: {t0:.2f}s → {t1:.2f}s")
    print(f"[cửa sổ] {t0:.2f}s → {t1:.2f}s  ({t1 - t0:.1f}s)   cmd rows={len(cmd)}  state rows={len(st)}")

    wc = (t_cmd >= t0) & (t_cmd <= t1)
    ws = (t_st >= t0) & (t_st <= t1)

    # ---- chỉ số cho từng joint ----
    rows = []
    for j in joints:
        m = tracking_metrics(t_cmd, cmd[f"cmd_{j}"].to_numpy(), t_st, st[f"pos_{j}"].to_numpy(), t0, t1)
        if m:
            m["joint"] = j
            m["eff_absmax"] = float(np.nanmax(np.abs(st.loc[ws, f"eff_{j}"]))) if f"eff_{j}" in st else np.nan
            rows.append(m)
    summ = pd.DataFrame(rows).set_index("joint")[["rmse", "max_err", "bias", "gain", "lag_ms", "corr", "p2p_ratio", "eff_absmax"]]
    pd.set_option("display.width", 160)
    print("\n=== Độ bám (state so với cmd) ===")
    print(summ.round(4).to_string())
    print("\nĐọc kết quả: gain≈1 & p2p_ratio≈1 = biên độ theo kịp; lag_ms = state trễ cmd bao nhiêu ms;"
          "\n             corr thấp/rmse lớn = không bám theo hình dạng (rung, sai dấu, bão hoà).")
    summ.to_csv(os.path.join(out, "tracking_summary.csv"))

    layout = joint_layout(joints)
    ncol = max(len(r) for r in layout)

    # ---- Fig 1: cmd vs state ----
    fig, axes = plt.subplots(len(layout), ncol, figsize=(7 * ncol, 2.3 * len(layout)), sharex=True, squeeze=False)
    for r, row in enumerate(layout):
        for c in range(ncol):
            ax = axes[r][c]
            if c >= len(row):
                ax.axis("off")
                continue
            j = row[c]
            ax.plot(t_cmd[wc], cmd.loc[wc, f"cmd_{j}"], "--", lw=1.0, color="tab:red", label="cmd (target)")
            ax.plot(t_st[ws], st.loc[ws, f"pos_{j}"], lw=0.9, color="tab:blue", label="state")
            s = summ.loc[j] if j in summ.index else None
            ttl = j.replace("_joint", "")
            if s is not None:
                ttl += f"   lag={s['lag_ms']:.0f}ms  gain={s['gain']:.2f}  rmse={s['rmse']:.3f}rad  corr={s['corr']:.2f}"
            ax.set_title(ttl, fontsize=9, loc="left")
            ax.set_ylabel("rad")
            ax.grid(alpha=0.3)
            if r == 0 and c == 0:
                ax.legend(loc="upper right", fontsize=8)
    for ax in axes[-1]:
        ax.set_xlabel("t (s)")
    fig.suptitle(f"joint_cmd.position vs joint_states.position  [{args.logdir}]", y=1.0)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "tracking.png"), dpi=110)

    # ---- Fig 2: sai số + effort ----
    fig2, axes2 = plt.subplots(len(layout), ncol, figsize=(7 * ncol, 2.3 * len(layout)), sharex=True, squeeze=False)
    for r, row in enumerate(layout):
        for c in range(ncol):
            ax = axes2[r][c]
            if c >= len(row):
                ax.axis("off")
                continue
            j = row[c]
            _, cg = uniform_grid(t_cmd, cmd[f"cmd_{j}"].to_numpy(), t0, t1, 1e-3)
            tg, sg = uniform_grid(t_st, st[f"pos_{j}"].to_numpy(), t0, t1, 1e-3)
            ax.plot(tg, sg - cg, lw=0.8, color="tab:purple")
            ax.axhline(0, color="k", lw=0.5)
            ax.set_ylabel("state−cmd (rad)")
            ax.set_title(j.replace("_joint", ""), fontsize=9, loc="left")
            ax.grid(alpha=0.3)
            if f"eff_{j}" in st:
                ax2 = ax.twinx()
                ax2.plot(t_st[ws], st.loc[ws, f"eff_{j}"], lw=0.6, color="0.6", alpha=0.8)
                ax2.set_ylabel("effort (Nm)", color="0.4")
    for ax in axes2[-1]:
        ax.set_xlabel("t (s)")
    fig2.tight_layout()
    fig2.savefig(os.path.join(out, "error_effort.png"), dpi=110)

    # ---- Fig 3: timing của control loop và state stream ----
    dt_cmd = np.diff(t_cmd[wc]) * 1e3
    dt_st = np.diff(t_st[ws]) * 1e3
    fig3, ax3 = plt.subplots(1, 3, figsize=(18, 4))
    for ax, d, name in ((ax3[0], dt_cmd, "control loop (joint_cmd)"), (ax3[1], dt_st, "joint_states callback")):
        if len(d):
            ax.hist(d, bins=np.linspace(0, max(5.0, d.max() * 1.05), 140), color="tab:blue")
            ax.set_yscale("log")
            ax.set_title(f"{name}: p50={np.percentile(d, 50):.2f}  p99={np.percentile(d, 99):.2f}  max={d.max():.1f} ms", fontsize=9)
        ax.set_xlabel("dt giữa 2 mẫu (ms)")
        ax.grid(alpha=0.3)
    ax3[2].plot(t_cmd[wc][1:], dt_cmd, lw=0.5, label="cmd dt")
    ax3[2].set_ylabel("ms")
    ax3[2].set_xlabel("t (s)")
    ax3[2].set_title("control-loop dt theo thời gian (kỳ vọng 1.0 ms)")
    ax3[2].grid(alpha=0.3)
    fig3.tight_layout()
    fig3.savefig(os.path.join(out, "timing.png"), dpi=110)
    if len(dt_cmd):
        print(f"\n=== Timing control loop === p50={np.percentile(dt_cmd, 50):.3f}ms  p99={np.percentile(dt_cmd, 99):.3f}ms  "
              f"max={dt_cmd.max():.2f}ms  (>2ms: {(dt_cmd > 2).sum()} lần, >5ms: {(dt_cmd > 5).sum()} lần)")
        print(f"    tốc độ thực tế ≈ {1e3 / np.mean(dt_cmd):.0f} Hz  → policy step ≈ {20 * np.mean(dt_cmd):.2f} ms (kỳ vọng 20.0)")

    print(f"\nĐã lưu: {out}/tracking.png, error_effort.png, timing.png, tracking_summary.csv")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
