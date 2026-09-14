#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ten_tune_tui.py — ROS 2 パラメータのライブ調整 TUI (AI Challenge 提出パッケージ同梱)

概要:
    curses ベースの端末 UI から、走行中のノードのパラメータを
    rclpy の GetParameters / SetParameters / ListParameters サービスクライアント経由で
    直接読み書きする(`ros2 param` サブプロセスは遅いため使わない)。

コンテナ内での実行方法:
    # ROS_DOMAIN_ID は走行中のシステムと合わせること
    export ROS_DOMAIN_ID=<走行中と同じ値>
    ros2 run aichallenge_submit_launch ten_tune_tui.py

    # 設定ファイルを明示する場合
    ros2 run aichallenge_submit_launch ten_tune_tui.py --config /path/to/ten_tune_params.yaml

    # 値を1回だけ表示して終了 (非対話)
    ros2 run aichallenge_submit_launch ten_tune_tui.py --dump

    # 値を1回だけ設定して終了 (非対話)
    ros2 run aichallenge_submit_launch ten_tune_tui.py --set /simple_pure_pursuit_node steering_tire_angle_gain 2.8

デフォルトの設定ファイルは ament_index_python 経由で
    get_package_share_directory('aichallenge_submit_launch') + '/config/ten_tune_params.yaml'
を解決する。パッケージ未インストール環境(ビルド前のホストなど)では
    このスクリプトと同じ階層の ../config/ten_tune_params.yaml にフォールバックする。

操作 (対話 TUI):
    ↑ / ↓      : 項目選択
    ← / →      : step 分だけ増減 (float / int)
    space      : bool をトグル
    Enter      : 値を直接入力
    r          : 全項目を再読込 (GetParameters)
    s          : 現在値を --save で指定した YAML (ros2 params 形式) に保存
    q          : 終了

注意:
    - ノード/パラメータが見つからない場合は "未接続" 表示のまま他の項目の操作を続行する。
    - SetParameters の結果 (成功可否・理由) は画面下部のステータス行に表示する。
      ノード側が動的変更を拒否する/効かない場合はここで理由が分かる。
    - stdlib の curses と rclpy のみを使用 (pip install 不要)。
"""

import argparse
import os
import sys
import time
import traceback

# --- rclpy 等は import できない環境でも --help / py_compile が通るようにガードする ---
try:
    import rclpy
    from rclpy.node import Node
    from rcl_interfaces.srv import GetParameters, SetParameters, ListParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    _HAVE_RCLPY = True
except Exception:  # pragma: no cover - ホスト環境でのフォールバック用
    _HAVE_RCLPY = False

try:
    import yaml
    _HAVE_YAML = True
except Exception:  # pragma: no cover
    _HAVE_YAML = False


DEFAULT_SAVE_PATH = "~/ten_tune_overrides.yaml"
SERVICE_TIMEOUT_SEC = 1.0


def resolve_default_config_path():
    """ament_index 経由でデフォルト設定ファイルを解決する。失敗したら相対パスにフォールバック。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = get_package_share_directory("aichallenge_submit_launch")
        cand = os.path.join(share_dir, "config", "ten_tune_params.yaml")
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.normpath(os.path.join(here, "..", "config", "ten_tune_params.yaml"))
    return cand


def load_config(path):
    if not _HAVE_YAML:
        raise RuntimeError("PyYAML が見つかりません")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    nodes = data.get("nodes", {}) or {}
    items = []
    for node_name, params in nodes.items():
        params = params or {}
        for param_name, spec in params.items():
            spec = spec or {}
            items.append({
                "node": node_name,
                "param": param_name,
                "step": spec.get("step", 1.0),
                "min": spec.get("min", None),
                "max": spec.get("max", None),
                # 実行時に埋める項目
                "value": None,
                "type": None,       # 'double' / 'int' / 'bool' / 'string' / None(未接続)
                "connected": False,
                "status": "",
            })
    return items


# ---------------------------------------------------------------------------
# ROS 2 通信レイヤー
# ---------------------------------------------------------------------------

class ParamClient:
    """1ノードあたりの GetParameters/SetParameters/ListParameters クライアントをまとめて管理する。"""

    def __init__(self, node: "Node"):
        self._node = node
        self._clients = {}  # node_name -> {get, set, list}

    def _get_clients(self, target_node_name):
        if target_node_name in self._clients:
            return self._clients[target_node_name]
        base = target_node_name.rstrip("/")
        c = {
            "get": self._node.create_client(GetParameters, f"{base}/get_parameters"),
            "set": self._node.create_client(SetParameters, f"{base}/set_parameters"),
            "list": self._node.create_client(ListParameters, f"{base}/list_parameters"),
        }
        self._clients[target_node_name] = c
        return c

    def _call(self, client, request, timeout=SERVICE_TIMEOUT_SEC):
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=timeout):
                return None
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
        if future.done():
            try:
                return future.result()
            except Exception:
                return None
        return None

    def get_value(self, node_name, param_name):
        clients = self._get_clients(node_name)
        req = GetParameters.Request()
        req.names = [param_name]
        res = self._call(clients["get"], req)
        if res is None or not res.values:
            return None
        return _pv_to_py(res.values[0])

    def set_value(self, node_name, param_name, py_value, ptype):
        clients = self._get_clients(node_name)
        req = SetParameters.Request()
        p = Parameter()
        p.name = param_name
        p.value = _py_to_pv(py_value, ptype)
        req.parameters = [p]
        res = self._call(clients["set"], req)
        if res is None or not res.results:
            return False, "サービス呼び出し失敗 (タイムアウト/未接続)"
        r = res.results[0]
        return bool(r.successful), (r.reason or "")


def _pv_to_py(pv: "ParameterValue"):
    t = pv.type
    if t == ParameterType.PARAMETER_BOOL:
        return pv.bool_value, "bool"
    if t == ParameterType.PARAMETER_INTEGER:
        return pv.integer_value, "int"
    if t == ParameterType.PARAMETER_DOUBLE:
        return pv.double_value, "double"
    if t == ParameterType.PARAMETER_STRING:
        return pv.string_value, "string"
    return None, None


def _py_to_pv(value, ptype):
    pv = ParameterValue()
    if ptype == "bool":
        pv.type = ParameterType.PARAMETER_BOOL
        pv.bool_value = bool(value)
    elif ptype == "int":
        pv.type = ParameterType.PARAMETER_INTEGER
        pv.integer_value = int(value)
    elif ptype == "double":
        pv.type = ParameterType.PARAMETER_DOUBLE
        pv.double_value = float(value)
    elif ptype == "string":
        pv.type = ParameterType.PARAMETER_STRING
        pv.string_value = str(value)
    else:
        pv.type = ParameterType.PARAMETER_NOT_SET
    return pv


def refresh_item(client: "ParamClient", item):
    val = client.get_value(item["node"], item["param"])
    if val is None or val[1] is None:
        item["connected"] = False
        item["value"] = None
        item["type"] = None
        return
    py_value, ptype = val
    item["connected"] = True
    item["value"] = py_value
    item["type"] = ptype


def refresh_all(client, items):
    for it in items:
        refresh_item(client, it)


def clamp(v, lo, hi):
    if lo is not None:
        v = max(lo, v)
    if hi is not None:
        v = min(hi, v)
    return v


def save_overrides(items, path):
    path = os.path.expanduser(path)
    out = {}
    for it in items:
        if not it["connected"]:
            continue
        node_key = it["node"]
        out.setdefault(node_key, {"ros__parameters": {}})
        out[node_key]["ros__parameters"][it["param"]] = it["value"]
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(out, f, allow_unicode=True, default_flow_style=False, sort_keys=True)
    return path


# ---------------------------------------------------------------------------
# 非対話モード
# ---------------------------------------------------------------------------

def do_dump(node, client, items):
    refresh_all(client, items)
    for it in items:
        if it["connected"]:
            print(f"{it['node']}\t{it['param']}\t{it['value']}\t({it['type']})")
        else:
            print(f"{it['node']}\t{it['param']}\t未接続")


def do_set_one(node, client, target_node, target_param, raw_value):
    # 型を知るために現在値を取得してから合わせる
    val = client.get_value(target_node, target_param)
    if val is None or val[1] is None:
        print(f"エラー: {target_node} の {target_param} が取得できません(未接続)")
        return 1
    _, ptype = val
    if ptype == "bool":
        py_value = raw_value.strip().lower() in ("1", "true", "yes", "on")
    elif ptype == "int":
        py_value = int(raw_value)
    elif ptype == "double":
        py_value = float(raw_value)
    else:
        py_value = raw_value
    ok, reason = client.set_value(target_node, target_param, py_value, ptype)
    print(f"set {target_node} {target_param} = {py_value} -> {'成功' if ok else '失敗'} {reason}")
    return 0 if ok else 2


# ---------------------------------------------------------------------------
# curses TUI
# ---------------------------------------------------------------------------

def run_tui(stdscr, node, client, items, save_path):
    import curses

    curses.curs_set(0)
    stdscr.nodelay(False)
    stdscr.keypad(True)
    try:
        curses.start_color()
        curses.use_default_colors()
        curses.init_pair(1, curses.COLOR_YELLOW, -1)   # 選択行
        curses.init_pair(2, curses.COLOR_RED, -1)      # 未接続
        curses.init_pair(3, curses.COLOR_GREEN, -1)    # 成功
        curses.init_pair(4, curses.COLOR_RED, -1)      # 失敗
        has_color = True
    except Exception:
        has_color = False

    refresh_all(client, items)

    sel = 0
    top = 0
    status = "起動しました。r で再読込, s で保存, q で終了"
    status_pair = 0

    while True:
        stdscr.erase()
        h, w = stdscr.getmaxyx()
        header = "ten_tune_tui — ROS2 パラメータ調整  [↑↓]選択 [←→]増減 [space]bool [Enter]入力 [r]再読込 [s]保存 [q]終了"
        stdscr.addnstr(0, 0, header, max(w - 1, 0))
        stdscr.addnstr(1, 0, "-" * max(w - 1, 0), max(w - 1, 0))

        list_h = max(h - 4, 1)
        if sel < top:
            top = sel
        if sel >= top + list_h:
            top = sel - list_h + 1

        for row, idx in enumerate(range(top, min(top + list_h, len(items)))):
            it = items[idx]
            y = 2 + row
            is_sel = (idx == sel)
            if not it["connected"]:
                line = f"{it['node']:<45} {it['param']:<28} 未接続"
                attr = curses.color_pair(2) if has_color else 0
            else:
                val_str = format_value(it["value"], it["type"])
                line = f"{it['node']:<45} {it['param']:<28} {val_str:<12} step={it['step']}"
                attr = 0
            if is_sel:
                attr |= curses.A_REVERSE
            stdscr.addnstr(y, 0, line, max(w - 1, 0), attr)

        # ステータス行
        stdscr.addnstr(h - 2, 0, "-" * max(w - 1, 0), max(w - 1, 0))
        attr = curses.color_pair(status_pair) if (has_color and status_pair) else 0
        stdscr.addnstr(h - 1, 0, status[: max(w - 1, 0)], max(w - 1, 0), attr)

        stdscr.refresh()

        try:
            ch = stdscr.getch()
        except KeyboardInterrupt:
            break

        if ch in (curses.KEY_UP, ord('k')):
            sel = max(0, sel - 1)
        elif ch in (curses.KEY_DOWN, ord('j')):
            sel = min(len(items) - 1, sel + 1)
        elif ch == curses.KEY_RESIZE:
            pass
        elif ch in (ord('q'), ord('Q')):
            break
        elif ch in (ord('r'), ord('R')):
            status = "再読込中..."
            stdscr.addnstr(h - 1, 0, status[: max(w - 1, 0)], max(w - 1, 0))
            stdscr.refresh()
            refresh_all(client, items)
            status = "再読込しました"
            status_pair = 0
        elif ch in (ord('s'), ord('S')):
            try:
                p = save_overrides(items, save_path)
                status = f"保存しました: {p}"
                status_pair = 3
            except Exception as e:
                status = f"保存失敗: {e}"
                status_pair = 4
        elif ch in (curses.KEY_LEFT, curses.KEY_RIGHT):
            it = items[sel]
            if not it["connected"]:
                status = "未接続のため操作できません"
                status_pair = 2
            elif it["type"] not in ("double", "int"):
                status = "この型は ←→ で変更できません (space または Enter を使用)"
                status_pair = 0
            else:
                delta = it["step"] if ch == curses.KEY_RIGHT else -it["step"]
                new_val = it["value"] + delta
                new_val = clamp(new_val, it["min"], it["max"])
                if it["type"] == "int":
                    new_val = int(round(new_val))
                ok, reason = client.set_value(it["node"], it["param"], new_val, it["type"])
                if ok:
                    it["value"] = new_val
                    status = f"OK: {it['node']} {it['param']} = {format_value(new_val, it['type'])}"
                    status_pair = 3
                else:
                    status = f"拒否: {it['node']} {it['param']}: {reason}"
                    status_pair = 4
        elif ch == ord(' '):
            it = items[sel]
            if not it["connected"]:
                status = "未接続のため操作できません"
                status_pair = 2
            elif it["type"] != "bool":
                status = "bool 型ではありません"
                status_pair = 0
            else:
                new_val = not it["value"]
                ok, reason = client.set_value(it["node"], it["param"], new_val, it["type"])
                if ok:
                    it["value"] = new_val
                    status = f"OK: {it['node']} {it['param']} = {new_val}"
                    status_pair = 3
                else:
                    status = f"拒否: {it['node']} {it['param']}: {reason}"
                    status_pair = 4
        elif ch in (curses.KEY_ENTER, 10, 13):
            it = items[sel]
            if not it["connected"]:
                status = "未接続のため操作できません"
                status_pair = 2
            else:
                raw = prompt_input(stdscr, h - 1, w, f"{it['param']} = ")
                if raw is None:
                    status = "入力キャンセル"
                    status_pair = 0
                else:
                    try:
                        if it["type"] == "bool":
                            new_val = raw.strip().lower() in ("1", "true", "yes", "on")
                        elif it["type"] == "int":
                            new_val = int(raw)
                        elif it["type"] == "double":
                            new_val = float(raw)
                        else:
                            new_val = raw
                        if it["type"] in ("double", "int"):
                            new_val = clamp(new_val, it["min"], it["max"])
                        ok, reason = client.set_value(it["node"], it["param"], new_val, it["type"])
                        if ok:
                            it["value"] = new_val
                            status = f"OK: {it['node']} {it['param']} = {format_value(new_val, it['type'])}"
                            status_pair = 3
                        else:
                            status = f"拒否: {it['node']} {it['param']}: {reason}"
                            status_pair = 4
                    except ValueError:
                        status = "入力値が不正です"
                        status_pair = 4


def format_value(value, ptype):
    if ptype == "double":
        return f"{value:.4f}"
    return str(value)


def prompt_input(stdscr, y, w, prompt):
    import curses
    curses.echo()
    curses.curs_set(1)
    try:
        stdscr.move(y, 0)
        stdscr.clrtoeol()
        stdscr.addnstr(y, 0, prompt, max(w - 1, 0))
        stdscr.refresh()
        try:
            raw = stdscr.getstr(y, len(prompt), 64).decode("utf-8")
        except Exception:
            raw = None
        return raw if raw else None
    finally:
        curses.noecho()
        curses.curs_set(0)


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def build_argparser():
    ap = argparse.ArgumentParser(description="ROS2 パラメータのライブ調整 TUI")
    ap.add_argument("--config", default=None, help="ten_tune_params.yaml のパス")
    ap.add_argument("--save", default=DEFAULT_SAVE_PATH, help="保存先 YAML (ros2 params 形式)")
    ap.add_argument("--dump", action="store_true", help="現在値を1回表示して終了")
    ap.add_argument("--set", nargs=3, metavar=("NODE", "PARAM", "VALUE"), default=None,
                     help="1回だけ値を設定して終了")
    return ap


def main(argv=None):
    ap = build_argparser()
    args = ap.parse_args(argv)

    config_path = args.config or resolve_default_config_path()
    if not os.path.exists(config_path):
        print(f"エラー: 設定ファイルが見つかりません: {config_path}", file=sys.stderr)
        return 1

    if not _HAVE_RCLPY:
        print("エラー: rclpy が import できません。ROS2 コンテナ内で実行してください。", file=sys.stderr)
        return 1

    items = load_config(config_path)

    rclpy.init(args=None)
    node = Node("ten_tune_tui")
    client = ParamClient(node)
    try:
        if args.dump:
            do_dump(node, client, items)
            return 0
        if args.set is not None:
            target_node, target_param, raw_value = args.set
            return do_set_one(node, client, target_node, target_param, raw_value)

        import curses
        curses.wrapper(run_tui, node, client, items, args.save)
        return 0
    except Exception:
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
