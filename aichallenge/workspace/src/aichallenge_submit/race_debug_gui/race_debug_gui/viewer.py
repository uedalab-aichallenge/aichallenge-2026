#!/usr/bin/env python3
"""複数車のデバッグ状態を 1 つの窓にまとめて表示するビューア。

各車は別々の docker コンテナ・別々の ROS_DOMAIN_ID で動いているため、
ROS のトピックでは他車の情報を受け取れない。そこで各車の
`gui_node.py` (書き手)が `RACE_GUI_DIR` 下に `<vehicle_id>.json` として
状態を吐き出し、このスクリプト(読み手)がそれをポーリングして
1つの窓に並べて出す。

**ROS には一切依存しない。** ホストで素の python3 として動かす。
  RACE_GUI_DIR=~/aichallenge-racingkart/output/gui_state python3 viewer.py
"""
import glob
import json
import os
import time
import tkinter as tk

BG = "#101418"
FG = "#e8eef4"
SUB = "#8fa3b0"
HL = "#ffd166"
OK = "#7bd88f"
WARN = "#ff6b6b"

STALE_SEC = 3.0
POLL_MS = 100

ROWS = [
    ("blocker", "ahead"),
    ("gap", "gap [m]"),
    ("attempt", "pass tgt"),
    ("offset", "lat now [m]"),
    ("target", "lat cmd [m]"),
    ("cap", "speed cap"),
    ("idx", "track idx"),
    ("boost", "boost left"),
]


def gui_dir():
    return os.environ.get("RACE_GUI_DIR", "/output/gui_state")


def load_states():
    """RACE_GUI_DIR 下の *.json を読み、vehicle_id 昇順のリストで返す。

    書き込み中のファイルと衝突しても落ちないよう、必ず try/except で囲む。
    """
    out = []
    try:
        paths = glob.glob(os.path.join(gui_dir(), "*.json"))
    except Exception:
        return out
    for p in paths:
        try:
            with open(p, "r") as f:
                data = json.load(f)
            out.append(data)
        except Exception:
            # 書き込み中に読んでしまった等。そのファイルは今回スキップ。
            continue
    out.sort(key=lambda d: d.get("vehicle_id", 0))
    return out


class CarColumn:
    """1台分の表示ブロック(既存の1台用表示と同じ項目を出す)。"""

    def __init__(self, parent):
        self.frame = tk.Frame(parent, bg=BG, highlightthickness=1,
                               highlightbackground="#2a3540")
        self.frame.pack(side="left", fill="both", expand=True,
                         padx=6, pady=6)

        self.head = self._big("waiting for data", 20, HL, (10, 2))
        self.lapinfo = self._big("", 12, SUB)

        tk.Frame(self.frame, height=1, bg="#2a3540").pack(fill="x", padx=10, pady=6)

        self.spd = self._big("-- km/h", 30, OK)
        self.spd_sub = self._big("", 11, SUB, (0, 4))

        tk.Frame(self.frame, height=1, bg="#2a3540").pack(fill="x", padx=10, pady=6)

        self.act = self._big("", 14, HL)
        self.rec = self._big("", 13, WARN)

        body = tk.Frame(self.frame, bg=BG)
        body.pack(fill="both", expand=True, padx=12, pady=(6, 10))
        self.rows = {}
        for key, label in ROWS:
            line = tk.Frame(body, bg=BG)
            line.pack(fill="x")
            tk.Label(line, text=label, font=("DejaVu Sans", 10), bg=BG, fg=SUB,
                     width=11, anchor="w").pack(side="left")
            v = tk.Label(line, text="-", font=("DejaVu Sans Mono", 11), bg=BG, fg=FG,
                         anchor="w")
            v.pack(side="left")
            self.rows[key] = v

    def _big(self, text, size, color, pady=(0, 0)):
        w = tk.Label(self.frame, text=text, font=("DejaVu Sans", size, "bold"),
                     bg=BG, fg=color)
        w.pack(pady=pady)
        return w

    def destroy(self):
        self.frame.destroy()

    def update(self, data):
        now = time.time()
        stale = (now - data.get("stamp", 0)) > STALE_SEC
        st = data.get("state") or {}
        recovery = data.get("recovery", "")
        cmd_speed = data.get("cmd_speed", 0.0)
        cmd_steer = data.get("cmd_steer", 0.0)
        actual = data.get("actual", 0.0)
        title = data.get("title", "")

        if stale:
            # 停止中: 消さずに灰色で出す。
            fg_main = SUB
            self.head.config(text="STOPPED  %s" % title, fg=SUB)
        else:
            fg_main = None
            slot = st.get("slot", "?")
            self.head.config(text=title or ("GRID  %s" % slot),
                              fg=HL if slot != "?" else SUB)

        self.lapinfo.config(text="lap %s    rank %s" %
                             (st.get("lap", "-"), st.get("rank", "-")),
                             fg=SUB)
        self.spd.config(text="%.1f km/h" % actual,
                         fg=(SUB if stale else
                             (WARN if st.get("boosting") == "1" else OK)))
        self.spd_sub.config(text="cmd %.1f km/h   steer %+.0f deg" %
                             (cmd_speed, cmd_steer), fg=SUB)
        self.act.config(text=st.get("act", ""), fg=(SUB if stale else HL))
        self.rec.config(text=recovery, fg=(SUB if stale else WARN))
        for key, w in self.rows.items():
            w.config(text=st.get(key, "-"), fg=(SUB if stale else FG))


class Viewer:
    def __init__(self, root):
        self.root = root
        self.container = tk.Frame(root, bg=BG)
        self.container.pack(fill="both", expand=True)
        self.columns = {}  # vehicle_id -> CarColumn

    def tick(self):
        states = load_states()
        ids_now = [d.get("vehicle_id", 0) for d in states]

        # 消えた車(ファイルが無くなった)の列は畳む。
        for vid in list(self.columns.keys()):
            if vid not in ids_now:
                self.columns[vid].destroy()
                del self.columns[vid]

        for data in states:
            vid = data.get("vehicle_id", 0)
            if vid not in self.columns:
                self.columns[vid] = CarColumn(self.container)
            self.columns[vid].update(data)

        self.root.title("race debug (%d cars)" % len(states))
        self.root.after(POLL_MS, self.tick)


def main():
    root = tk.Tk()
    root.configure(bg=BG)
    root.title("race debug (0 cars)")
    root.geometry("1200x680")

    viewer = Viewer(root)
    viewer.tick()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
