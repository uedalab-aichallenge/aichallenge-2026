#!/usr/bin/env python3
"""走行中の速度指令と、いま働いている処理を 1 枚の窓に出すデバッグ表示。

見たいのは「今どの分岐に入っているか」なので、値そのものより
**状態の名前**を大きく出す。走りには一切影響しない(購読するだけ)。

出どころ:
  v2x_overtaker            -> output/status              (自作の状態文字列)
  stuck_recovery_controller-> /control/debug/recovery_status
  車両                     -> /control/command/control_cmd, /vehicle/status/*
"""
import json
import os
import tempfile
import threading
import time
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import VelocityReport

BG = "#101418"
FG = "#e8eef4"
SUB = "#8fa3b0"
HL = "#ffd166"
OK = "#7bd88f"
WARN = "#ff6b6b"


class DebugGui(Node):
    def __init__(self):
        super().__init__("race_debug_gui")
        # 窓のタイトルにだけ、外から与えられた「本当の」グリッド位置を出す
        # (ユーザーの許可。判断には一切使わない。表示専用)。
        # vehicle_id は launch が ROS_DOMAIN_ID から渡す。
        # このローカル検証環境では ドメイン番号 = グリッド番号。
        self.title = self.declare_parameter("window_title", "").value
        self.vehicle_id = int(self.declare_parameter("vehicle_id", 0).value)
        self.state = {}
        self.recovery = ""
        self.recovery_t = 0.0
        self.cmd_speed = 0.0
        self.cmd_steer = 0.0
        self.actual = 0.0
        self.lock = threading.Lock()

        # センサ品質(best effort)で出ている topic があるので合わせる
        be = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, "input/status", self.on_status, 1)
        self.create_subscription(String, "/control/debug/recovery_status",
                                 self.on_recovery, 1)
        self.create_subscription(AckermannControlCommand,
                                 "/control/command/control_cmd", self.on_cmd, 1)
        self.create_subscription(VelocityReport,
                                 "/vehicle/status/velocity_status", self.on_vel, be)

    def on_status(self, msg):
        d = {}
        for line in msg.data.splitlines():
            if "=" in line:
                k, _, v = line.partition("=")
                d[k.strip()] = v.strip()
        with self.lock:
            self.state = d

    def on_recovery(self, msg):
        with self.lock:
            self.recovery = msg.data
            self.recovery_t = self.get_clock().now().nanoseconds * 1e-9

    def on_cmd(self, msg):
        with self.lock:
            self.cmd_speed = msg.longitudinal.speed * 3.6
            self.cmd_steer = msg.lateral.steering_tire_angle * 57.2958

    def on_vel(self, msg):
        with self.lock:
            self.actual = msg.longitudinal_velocity * 3.6

    def snapshot(self):
        with self.lock:
            now = self.get_clock().now().nanoseconds * 1e-9
            rec = self.recovery if (now - self.recovery_t) < 0.8 else ""
            return dict(self.state), rec, self.cmd_speed, self.cmd_steer, self.actual

    def compute_title(self, st):
        """1台用の窓タイトルと同じ文字列を、集約ビューア用にも作る。"""
        slot = st.get("slot", "?") if st else "?"
        if self.title:
            return self.title
        elif self.vehicle_id > 0:
            # 左が外から与えられた本当の値、右が自車コードの判定結果
            return ("d%d = P%d (true)   |   planner says %s"
                    % (self.vehicle_id, self.vehicle_id, slot))
        else:
            return "debug %s" % slot

    def write_state_file(self):
        """今の状態を JSON 1ファイルへ原子的に書き出す(集約ビューア用)。

        書き込み中のファイルを読み手が壊れた状態で読まないよう、
        同じディレクトリに一時ファイルを作ってから os.replace する。
        """
        st, recovery, cmd_speed, cmd_steer, actual = self.snapshot()
        payload = {
            "vehicle_id": self.vehicle_id,
            "title": self.compute_title(st),
            "stamp": time.time(),
            "state": st,
            "recovery": recovery,
            "cmd_speed": cmd_speed,
            "cmd_steer": cmd_steer,
            "actual": actual,
        }
        out_dir = os.environ.get("RACE_GUI_DIR", "/output/gui_state")
        try:
            os.makedirs(out_dir, exist_ok=True)
            fd, tmp_path = tempfile.mkstemp(prefix=".tmp_", dir=out_dir)
            try:
                with os.fdopen(fd, "w") as f:
                    json.dump(payload, f)
                os.replace(tmp_path, os.path.join(out_dir, "%d.json" % self.vehicle_id))
            except Exception:
                try:
                    os.remove(tmp_path)
                except OSError:
                    pass
                raise
        except Exception:
            # ファイル書き出しの失敗で走行に影響させない。
            pass

    def write_loop(self):
        """10Hz で状態ファイルを書き続ける(rclpy.spin と並行して動かす)。"""
        while rclpy.ok():
            self.write_state_file()
            time.sleep(0.1)


def build_window(node):
    root = tk.Tk()
    root.configure(bg=BG)
    root.geometry("380x640")

    rows = {}

    def big(text, size, color=FG, pady=(0, 0)):
        w = tk.Label(root, text=text, font=("DejaVu Sans", size, "bold"),
                     bg=BG, fg=color)
        w.pack(pady=pady)
        return w

    head = big("waiting for data", 26, HL, (10, 0))
    lapinfo = big("", 13, SUB)

    tk.Frame(root, height=1, bg="#2a3540").pack(fill="x", padx=14, pady=8)

    spd = big("-- km/h", 40, OK)
    spd_sub = big("", 12, SUB, (0, 6))

    tk.Frame(root, height=1, bg="#2a3540").pack(fill="x", padx=14, pady=8)

    act = big("", 17, HL)
    rec = big("", 15, WARN)

    body = tk.Frame(root, bg=BG)
    body.pack(fill="both", expand=True, padx=18, pady=(8, 10))
    for key, label in (("blocker", "ahead"), ("gap", "gap [m]"),
                       ("attempt", "pass tgt"), ("offset", "lat now [m]"),
                       ("target", "lat cmd [m]"), ("cap", "speed cap"),
                       ("idx", "track idx"), ("boost", "boost left")):
        line = tk.Frame(body, bg=BG)
        line.pack(fill="x")
        tk.Label(line, text=label, font=("DejaVu Sans", 11), bg=BG, fg=SUB,
                 width=11, anchor="w").pack(side="left")
        v = tk.Label(line, text="-", font=("DejaVu Sans Mono", 12), bg=BG, fg=FG,
                     anchor="w")
        v.pack(side="left")
        rows[key] = v

    def tick():
        st, recovery, cmd, steer, actual = node.snapshot()
        if st:
            slot = st.get("slot", "?")
            root.title(node.compute_title(st))
            head.config(text="GRID  %s" % slot,
                        fg=HL if slot != "?" else SUB)
            lapinfo.config(text="lap %s    rank %s" %
                           (st.get("lap", "-"), st.get("rank", "-")))
            spd.config(text="%.1f km/h" % actual)
            spd_sub.config(text="cmd %.1f km/h   steer %+.0f deg" % (cmd, steer))
            a = st.get("act", "")
            act.config(text=a)
            for k, w in rows.items():
                w.config(text=st.get(k, "-"))
            boosting = st.get("boosting") == "1"
            spd.config(fg=WARN if boosting else OK)
        rec.config(text=recovery)
        root.after(100, tick)

    tick()
    return root


def main():
    # 既定では自分の窓は開かず、状態ファイルを書き出すだけにする。
    # RACE_GUI_WINDOW=1 のときだけ、従来どおり1台用の窓も開く。
    want_window = os.environ.get("RACE_GUI_WINDOW", "0") == "1"

    # 大会の評価環境には画面が無い可能性が高い。
    # 窓を開く設定であっても、画面が無ければ tkinter は使わない
    # (ただしファイル書き出しは続ける。ROSにも触れて良い)。
    use_tk = False
    if want_window and os.environ.get("DISPLAY"):
        try:
            probe = tk.Tk()
            probe.destroy()
            use_tk = True
        except Exception:
            use_tk = False

    rclpy.init()
    node = DebugGui()
    writer = threading.Thread(target=node.write_loop, daemon=True)
    writer.start()

    if use_tk:
        t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
        t.start()
        try:
            root = build_window(node)
            root.mainloop()
        except KeyboardInterrupt:
            pass
        except Exception:
            pass
        finally:
            rclpy.shutdown()
    else:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()


if __name__ == "__main__":
    main()
