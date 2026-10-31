#!/usr/bin/env python3
"""Draw GTKWave-style timing diagrams from a VCD file (no GUI needed).

Usage:
    plot_vcd.py tb_i2c.vcd out.png T0_us T1_us [--title TEXT] [--signals a,b,c] [--decode]

Signals are full hierarchical names (tb_i2c.scl, tb_i2c.dut.state, ...).
1-bit signals are drawn as logic waves, wider ones as buses with hex values.
--decode annotates I2C START/STOP and every sampled bit (on SCL rising edge).
"""
import argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT = ["tb_i2c.scl", "tb_i2c.sda", "tb_i2c.m_scl_oe", "tb_i2c.m_sda_oe",
           "tb_i2c.s_sda_oe", "tb_i2c.tb_scl_oe", "tb_i2c.dut.state", "tb_i2c.dut.phase",
           "tb_i2c.slave.state", "tb_i2c.done", "tb_i2c.dout", "tb_i2c.rx_valid", "tb_i2c.rx_data"]
MASTER_ST = {0: "IDLE", 1: "START", 2: "DATA", 3: "ACK", 4: "STOP"}
SLAVE_ST = {0: "IDLE", 1: "ADDR", 2: "ACK_A", 3: "RX", 4: "ACK_RX", 5: "TX", 6: "ACK_TX"}


def parse_vcd(path, wanted):
    ids, widths, data, scope = {}, {}, {}, []
    t = 0
    with open(path) as f:
        for line in f:
            tok = line.split()
            if not tok:
                continue
            if tok[0] == "$scope":
                scope.append(tok[2])
            elif tok[0] == "$upscope":
                scope.pop()
            elif tok[0] == "$var":
                name = ".".join(scope + [tok[4]])
                if name in wanted:
                    ids.setdefault(tok[3], []).append(name)
                    widths[name] = int(tok[2])
                    data[name] = []
            elif tok[0].startswith("#"):
                t = int(tok[0][1:])
            elif tok[0][0] in "01xz" and len(tok) == 1:
                for n in ids.get(tok[0][1:], []):
                    data[n].append((t, tok[0][0]))
            elif tok[0][0] == "b" and len(tok) == 2:
                for n in ids.get(tok[1], []):
                    data[n].append((t, tok[0][1:]))
    return data, widths


def value_at(ch, t):
    v = None
    for tt, vv in ch:
        if tt > t:
            break
        v = vv
    return v


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("vcd"); ap.add_argument("out")
    ap.add_argument("t0", type=float); ap.add_argument("t1", type=float)
    ap.add_argument("--title", default="")
    ap.add_argument("--signals", default=",".join(DEFAULT))
    ap.add_argument("--decode", action="store_true")
    a = ap.parse_args()
    sigs = a.signals.split(",")
    data, widths = parse_vcd(a.vcd, set(sigs))
    t0, t1 = a.t0 * 1e6, a.t1 * 1e6           # VCD timescale is 1 ps
    h = 0.62 * len(sigs) + 1.2
    fig, ax = plt.subplots(figsize=(16, h))
    fig.patch.set_facecolor("white")
    us = lambda t: t / 1e6
    for row, name in enumerate(sigs):
        y = (len(sigs) - 1 - row) * 1.0
        ch = data.get(name, [])
        pts = [(t0, value_at(ch, t0))] + [(t, v) for t, v in ch if t0 < t < t1] + [(t1, None)]
        label = name.replace("tb_i2c.", "")
        ax.text(us(t0) - (us(t1) - us(t0)) * 0.005, y + 0.3, label, ha="right", va="center", fontsize=9,
                family="monospace")
        if widths.get(name, 1) == 1:
            xs, ys = [], []
            for (ta, va), (tb, _) in zip(pts[:-1], pts[1:]):
                lv = 0.6 if va == "1" else 0.0
                xs += [us(ta), us(tb)]; ys += [y + lv, y + lv]
            color = "#1a7f37" if label in ("scl", "sda") else "#0b5cad"
            ax.plot(xs, ys, color=color, lw=1.3, drawstyle="steps-post")
        else:
            for (ta, va), (tb, _) in zip(pts[:-1], pts[1:]):
                ax.fill_between([us(ta), us(tb)], y + 0.05, y + 0.55, color="#e8eef7", edgecolor="#0b5cad", lw=0.8)
                if va is None or "x" in va:
                    txt = "x"
                else:
                    n = int(va, 2)
                    if label == "dut.state":
                        txt = MASTER_ST.get(n, str(n))
                    elif label == "slave.state":
                        txt = SLAVE_ST.get(n, str(n))
                    elif widths[name] <= 3:
                        txt = str(n)
                    else:
                        txt = f"{n:02X}"
                if us(tb) - us(ta) > (us(t1) - us(t0)) / 90:
                    ax.text((us(ta) + us(tb)) / 2, y + 0.3, txt, ha="center", va="center", fontsize=7.5)
    if a.decode:
        scl, sda = data["tb_i2c.scl"], data["tb_i2c.sda"]
        ytop = (len(sigs) - 1) * 1.0 + 0.8
        for t, v in scl:
            if max(t0, 1e6) < t < t1 and v == "1":
                b = value_at(sda, t)
                ax.text(us(t), ytop, b, ha="center", fontsize=8, color="#cf222e", weight="bold")
                ax.axvline(us(t), color="#cf222e", lw=0.4, alpha=0.35)
        for t, v in sda:
            if max(t0, 1e6) < t < t1 and value_at(scl, t) == "1":
                ax.text(us(t), ytop + 0.35, "S" if v == "0" else "P", ha="center", fontsize=9, color="#8250df",
                        weight="bold")
    ax.set_xlim(us(t0), us(t1))
    ax.set_ylim(-0.4, len(sigs) - 1 + 1.4)
    ax.set_yticks([])
    ax.set_xlabel("time, µs")
    for s in ("left", "right", "top"):
        ax.spines[s].set_visible(False)
    ax.grid(axis="x", alpha=0.25)
    if a.title:
        ax.set_title(a.title, fontsize=11)
    fig.tight_layout()
    fig.savefig(a.out, dpi=130)


if __name__ == "__main__":
    main()
