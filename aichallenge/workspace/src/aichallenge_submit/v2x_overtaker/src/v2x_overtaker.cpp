// V2X の他車位置を見て走行ラインを横にずらし、追い越し・追従を行う。
//
// simple_trajectory_generatorの軌道へ、他車回避と安全・速度制約を合成する。
//
//   前方に他車がいる -> 反対側へよける。よける幅は corridor.csv の可動域で頭打ちにする。
//   よけきれない     -> 前車速度に合わせて追従し、追突(crash ペナルティ)を避ける。
//   他車がいない     -> オフセットを 0 に戻して元のラインへ復帰。
//
//   オフセットは時間レート制限つきで動かす。急に横へ飛ぶと pure_pursuit が
//   過大な操舵を出すため。
#include "v2x_overtaker/v2x_overtaker.hpp"
#include "v2x_overtaker/runup_sim.hpp"

#include <cstdarg>
#include <cstring>

#include <cctype>
#include <cstdint>
#include <cstring>
#include <iterator>

V2XOvertaker::V2XOvertaker()
: Node("v2x_overtaker"),
  detect_range_(declare_parameter<double>("detect_range", 25.0)),
  front_lane_half_(declare_parameter<double>("front_lane_half", 1.3)),
  contact_vehicle_dist_(declare_parameter<double>("contact_vehicle_dist", 3.5)),
  contact_log_hold_(declare_parameter<double>("contact_log_hold", 6.0)),
  avoid_range_(declare_parameter<double>("avoid_range", 8.0)),
  collision_radius_(declare_parameter<double>("collision_radius", 1.7)),
  avoid_min_lon_(declare_parameter<double>("avoid_min_lon", 2.1)),
  ttc_threshold_(declare_parameter<double>("ttc_threshold", 1.0)),
  big_gap_closing_(declare_parameter<double>("big_gap_closing", 5.0)),
  inside_time_gain_(declare_parameter<double>("inside_time_gain", 1.4)),
  inside_width_gain_(declare_parameter<double>("inside_width_gain", 0.85)),
  latch_width_gain_(declare_parameter<double>("latch_width_gain", 0.90)),
  pass_gap_(declare_parameter<double>("pass_gap", 1.7)),
  pass_gap_clear_ratio_(
    declare_parameter<double>("pass_gap_clear_ratio", 1.0)),
  pass_side_clear_(declare_parameter<double>("pass_side_clear", 0.8)),
  // 門への助走の横逃がしは、スタートの合流が済んでから始める。
  gate_sidestep_after_merge_(
    declare_parameter<bool>("gate_sidestep_after_merge", true)),
  // 相手の予測軌道の検査を「抜きどころを持つ相手」だけでなく全相手に広げる。
  predict_all_targets_(declare_parameter<bool>("predict_all_targets", true)),
  // PASS中は横間隔がpass_beside_sep_未満なら追突防止を補間で緩め、既定値1.45m以上で完全に解除する。
  pass_beside_sep_(declare_parameter<double>("pass_beside_sep", 1.45)),
  offset_rate_(declare_parameter<double>("offset_rate", 1.2)),
  corridor_safety_(declare_parameter<double>("corridor_safety", 0.65)),
  corridor_safety_pass_(declare_parameter<double>("corridor_safety_pass", 0.55)),
  // 追い越し可能ゾーン(pass_ok)だけは、さらに削る。
  // 0.45 で壁に当たったのはコーナーで追従誤差が出たため。
  // pass_ok は「幅 4.0m 以上・曲率半径も十分」を満たす区間として
  // make_corridor.py が選んだ場所で、追従誤差が小さく壁も遠い。
  // corridor_ten.csv 自体が既に壁から 半幅0.73+余裕0.45=1.18m 内側にあるので、
  // ここで0.30mを引いても、生成時に確保した車体余裕を維持できる。
  corridor_safety_zone_(declare_parameter<double>("corridor_safety_zone", 0.30)),
  side_room_ahead_(declare_parameter<double>("side_room_ahead", 5.0)),
  corridor_funnel_(declare_parameter<bool>("corridor_funnel", true)),
  lat_relax_yield_(declare_parameter<bool>("lat_relax_yield", true)),
  pass_need_room_(declare_parameter<bool>("pass_need_room", true)),
  stopped_funnel_(declare_parameter<bool>("stopped_funnel", true)),
  stopped_seq_pass_(declare_parameter<bool>("stopped_seq_pass", true)),
  lat_relax_drop_mps_(declare_parameter<double>("lat_relax_drop_mps", 1.5)),
  funnel_ahead_m_(declare_parameter<double>("funnel_ahead_m", 20.0)),
  funnel_speed_floor_(declare_parameter<double>("funnel_speed_floor", 2.0)),
  // 選んだ側の余地が無い状態がこの秒数連続で続いたら、反対側へ回り直す
  side_flip_hold_(declare_parameter<double>("side_flip_hold", 0.6)),
  // 1台の対象車に対して側を変更してよい回数
  side_flip_max_(declare_parameter<int>("side_flip_max", 2)),
  side_flip_regen_(declare_parameter<double>("side_flip_regen", 15.0)),
  // --- 相手の走行ラインの学習 ---
  // 相手は毎周ほぼ同じラインを走る。1周目に横位置を覚えておき、
  // 2周目以降は「抜き切るまでの区間ぜんぶ」を先に見て側を決める。
  side_window_search_(declare_parameter<bool>("side_window_search", true)),
  side_window_search_m_(declare_parameter<double>("side_window_search_m", 40.0)),
  ov_pass_wall_relax_(declare_parameter<bool>("ov_pass_wall_relax", true)),
  ot_lane_hot_enable_(declare_parameter<bool>("ot_lane_hot_enable", true)),
  ot_lane_hot_range_(declare_parameter<double>("ot_lane_hot_range", 40.0)),
  lat_audit_(declare_parameter<bool>("lat_audit", true)),
  // 1秒に1回では標本が足りない。1レース4台で 150行しか。
  lat_audit_sec_(declare_parameter<double>("lat_audit_sec", 0.25)),
  ov_collide_follow_side_(declare_parameter<bool>("ov_collide_follow_side", true)),
  ov_release_stale_(declare_parameter<bool>("ov_release_stale", true)),
  lat_pred_mode_(declare_parameter<int>("lat_pred_mode", 1)),
  lane_record_enable_(declare_parameter<bool>("lane_record_enable", false)),
  lane_learn_(declare_parameter<bool>("lane_learn", true)),
  lane_map_side_(declare_parameter<bool>("lane_map_side", true)),
  side_model_fill_(declare_parameter<bool>("side_model_fill", true)),
  lane_map_stretch_(declare_parameter<double>("lane_map_stretch", 30.0)),
  lane_map_min_pts_(declare_parameter<int>("lane_map_min_pts", 8)),
  lane_map_margin_(declare_parameter<double>("lane_map_margin", 2.0)),
  // 足りている区間が pass_len のこの倍だけ連続していれば、その側は成立
  lane_map_need_gain_(declare_parameter<double>("lane_map_need_gain", 1.0)),
  zone_look_ahead_(declare_parameter<double>("zone_look_ahead", 40.0)),
  window_full_(declare_parameter<double>("window_full", 15.0)),
  window_end_(declare_parameter<double>("window_end", 30.0)),
  window_back_(declare_parameter<double>("window_back", 3.0)),
  // グリッド(idx237付近)から idx20 までは約 34m。そこで合流を終える。
  start_merge_dist_(declare_parameter<double>("start_merge_dist", 34.0)),
  start_lat_max_(declare_parameter<double>("start_lat_max", 0.9)),
  // 3位スタートのとき、1位(NPC・25km/hハンデで遅い)ではなく
  // 2位(プレイヤー)側へ寄せて出る。
  // 停止している前車がこの距離[m]より近いときは、
  // stop_hold_sec 秒のあいだ下限速度を課さない(突っ込まない)。
  stop_hold_gap_(declare_parameter<double>("stop_hold_gap", 3.0)),
  stop_hold_sec_(declare_parameter<double>("stop_hold_sec", 2.0)),
  // 自車がこの速度[m/s]を超えているときだけ「待つ」。
  // 停止状態から発進するときに待つと、スタートで出遅れるだけ。
  stop_hold_move_(declare_parameter<double>("stop_hold_move", 0.5)),
  slow_leader_speed_(declare_parameter<double>("slow_leader_speed", 2.5)),
  attempt_timeout_(declare_parameter<double>("attempt_timeout", 16.0)),
  attempt_stall_time_(declare_parameter<double>("attempt_stall_time", 0.0)),
  attempt_stall_gain_(declare_parameter<double>("attempt_stall_gain", 0.5)),
  attempt_stall_cool_(declare_parameter<double>("attempt_stall_cool", 5.0)),
  // 「相手より8m前」を完全追越の条件にすると、計画に必要な連続区間が18mへ。
  pass_len_(declare_parameter<double>("pass_len", 2.5)),
  pass_done_len_(declare_parameter<double>("pass_done_len", 5.0)),
  spot_here_enable_(declare_parameter<bool>("spot_here_enable", true)),
  spot_here_range_(declare_parameter<double>("spot_here_range", 30.0)),
  pass_done_sec_(declare_parameter<double>("pass_done_sec", 0.4)),
  // 追い越し1件の所要時間の下限[s]。これ未満は偽陽性として数えない。
  pass_done_min_sec_(declare_parameter<double>("pass_done_min_sec", 1.5)),
  pen_speed_(declare_parameter<double>("pen_speed", 1.38889)),
  pen_tol_(declare_parameter<double>("pen_tol", 0.25)),
  pen_hold_(declare_parameter<double>("pen_hold", 0.6)),
  wall_guard_enable_(declare_parameter<bool>("wall_guard_enable", true)),
  wall_guard_horizon_(declare_parameter<double>("wall_guard_horizon", 0.35)),
  wall_guard_dt_(declare_parameter<double>("wall_guard_dt", 0.1)),
  veh_half_width_(declare_parameter<double>("veh_half_width", 0.73)),
  veh_wheel_base_(declare_parameter<double>("veh_wheel_base", 2.14)),
  veh_front_overhang_(declare_parameter<double>("veh_front_overhang", 0.47)),
  veh_rear_overhang_(declare_parameter<double>("veh_rear_overhang", 0.51)),
  veh_max_steer_(declare_parameter<double>("veh_max_steer", 0.6109)),
  wall_guard_margin_(declare_parameter<double>("wall_guard_margin", 0.02)),
  wall_guard_run_(declare_parameter<double>("wall_guard_run", 1.5)),
  wall_guard_ay_max_(declare_parameter<double>("wall_guard_ay_max", 20.0)),
  wall_guard_corridor_half_(
    declare_parameter<double>("wall_guard_corridor_half", 0.73)),
  occ_enable_(declare_parameter<bool>("occ_enable", true)),
  occ_map_yaml_(declare_parameter<std::string>(
    "occ_map_yaml",
    "/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
    "/env/ten_final_ver3/ten_occupancy_grid_map.yaml")),
  occ_sample_step_(declare_parameter<double>("occ_sample_step", 0.15)),
  occ_steer_bins_(declare_parameter<int>("occ_steer_bins", 41)),
  occ_clear_search_(declare_parameter<double>("occ_clear_search", 1.0)),
  pass_time_limit_(declare_parameter<double>("pass_time_limit", 18.0)),
  boost_retry_sec_(declare_parameter<double>("boost_retry_sec", 6.0)),
  boost_min_speed_(declare_parameter<double>("boost_min_speed", 4.5)),
  boost_accel_(declare_parameter<double>("boost_accel", 0.5)),
  boost_min_headroom_(declare_parameter<double>("boost_min_headroom", 1.0)),
  vehicle_accel_(declare_parameter<double>("vehicle_accel", 0.9)),
  zone_exit_margin_(declare_parameter<double>("zone_exit_margin", 25.0)),
  leader_speed_cap_(declare_parameter<double>("leader_speed_cap", 25.0)),
  rank1_corner_gain_(declare_parameter<double>("rank1_corner_gain", 1.00)),
  rank2_corner_gain_(declare_parameter<double>("rank2_corner_gain", 1.00)),
  rank2_speed_cap_(declare_parameter<double>("rank2_speed_cap", 36.0)),
  rank_shape_enable_(declare_parameter<bool>("rank_shape_enable", true)),
  final_dash_enable_(declare_parameter<bool>("final_dash_enable", true)),
  boost_reserve_final_(declare_parameter<int>("boost_reserve_final", 1)),
  final_dash_dist_(declare_parameter<double>("final_dash_dist", 70.0)),
  final_dash_gap_(declare_parameter<double>("final_dash_gap", 25.0)),
  leader_boost_block_(declare_parameter<bool>("leader_boost_block", true)),
  leader_boost_headroom_(declare_parameter<double>("leader_boost_headroom", 2.5)),
  final_dash_boost_when_leading_(
    declare_parameter<bool>("final_dash_boost_when_leading", false)),
  // --- 余ったブーストを直線で使い切る ---。
  free_boost_enable_(declare_parameter<bool>("free_boost_enable", false)),
  free_boost_gap_(declare_parameter<double>("free_boost_gap", 20.0)),
  free_boost_min_speed_(declare_parameter<double>("free_boost_min_speed", 4.0)),
  free_boost_clear_ahead_(declare_parameter<double>("free_boost_clear_ahead", 15.0)),
  free_boost_straight_(declare_parameter<double>("free_boost_straight", 25.0)),
  free_boost_headroom_(declare_parameter<double>("free_boost_headroom", 1.5)),
  free_boost_skip_(declare_parameter<double>("free_boost_skip", 6.0)),
  // --- 正面衝突を横からの接触に変える ---
  // Crash(10秒)はカート前方で当たったときだけ付き、横からの接触では付かない。
  // 止まりきれないと分かった時点で減速に頼ると、そのまま前から突っ込んで
  // Crash を食らう。間に合わないなら、多少無理でも横へねじ込むほうがよい。
  wedge_enable_(declare_parameter<bool>("wedge_enable", true)),
  wedge_ttc_(declare_parameter<double>("wedge_ttc", 0.7)),
  wedge_room_(declare_parameter<double>("wedge_room", 0.25)),
  // 自力でも抜ける場面で、ブーストがこれだけ[s]短縮するなら使う
  boost_gain_min_(declare_parameter<double>("boost_gain_min", 1.5)),
  // この周回数を過ぎるまでは、抜かれ返される可能性を考えて温存する
  boost_hold_laps_(declare_parameter<int>("boost_hold_laps", 0)),
  free_boost_defend_dist_(declare_parameter<double>("free_boost_defend_dist", 15.0)),
  race_laps_(declare_parameter<int>("race_laps", 6)),
  free_boost_laps_left_(declare_parameter<int>("free_boost_laps_left", 1)),
  // 直線判定を待たずにブーストしてよい区間。"開始:終了" をカンマ区切り。
  // メインストレート(idx232-241)の手前、コーナーの立ち上がりから
  // 加速を始めるために使う。
  boost_zone_spec_(declare_parameter<std::string>("boost_zones", "220:241")),
  no_pass_zone_spec_(declare_parameter<std::string>("no_pass_zones", "18:30,89:95")),
  ot_lane_zone_spec_(declare_parameter<std::string>("ot_lane_zones", "234:21")),
  right_zone_spec_(declare_parameter<std::string>("right_zones", "")),
  // スタートグリッドの座標。"x1:y1,x2:y2,x3:y3" の順に P1,P2,P3。
  // 空なら進行度順にフォールバックする。
  // 値は `スタート位置 P... 座標=(x,y)` のログから書き写す。
  grid_slot_spec_(declare_parameter<std::string>("grid_slots", "")),
  slow_rival_ratio_(declare_parameter<double>("slow_rival_ratio", 0.85)),
  slow_rival_by_top_(declare_parameter<bool>("slow_rival_by_top", false)),
  slow_rival_recent_(declare_parameter<bool>("slow_rival_recent", false)),
  slow_rival_top_ratio_(declare_parameter<double>("slow_rival_top_ratio", 0.85)),
  // スタートが2位・3位のときだけ、序盤に1つ使って前に出る。
  start_boost_enable_(declare_parameter<bool>("start_boost_enable", false)),
  start_boost_laps_(declare_parameter<int>("start_boost_laps", 2)),
  // スタートからこの距離[m]以内なら「スタート直後」とみなす。
  // run_dist_ は合流が終わると約70m で凍るので、40 のままだと
  // 合流後に条件が成立しない(実際の一発制限は start_boost_laps_ と
  // start_boost_used_ が担っている)。凍る値を上回る 120 にして
  // 「序盤かどうか」の判断を lap_ 側へ寄せる。
  start_boost_dist_(declare_parameter<double>("start_boost_dist", 120.0)),
  // 最下位のときに追い越しの条件を緩める割合。
  // 抜かない限り結果が変わらないので、多少の失敗より仕掛けを優先する。
  aggressive_time_gain_(declare_parameter<double>("aggressive_time_gain", 1.5)),
  aggressive_width_gain_(declare_parameter<double>("aggressive_width_gain", 0.85)),
  // 仕掛けどころに合わせて車間を詰める制御
  approach_enable_(declare_parameter<bool>("approach_enable", true)),
  approach_range_(declare_parameter<double>("approach_range", 80.0)),
  approach_gap_max_(declare_parameter<double>("approach_gap_max", 8.0)),
  charge_close_dist_(declare_parameter<double>("charge_close_dist", 20.0)),
  charge_close_gap_k_(declare_parameter<double>("charge_close_gap_k", 1.2)),
  charge_brake_margin_(declare_parameter<double>("charge_brake_margin", 1.5)),
  // --- 相手の減速を先読みする ---
  // カーブでは相手はほぼ確実に落とす。今の速度だけを見て追従すると
  // 後ろから加速していって追突する。
  predict_enable_(declare_parameter<bool>("predict_enable", true)),
  predict_ahead_(declare_parameter<double>("predict_ahead", 20.0)),
  near_radius_(declare_parameter<double>("near_radius", 6.0)),
  predict_floor_(declare_parameter<double>("predict_floor", 0.55)),
  predict_lane_speed_(declare_parameter<bool>("predict_lane_speed", true)),
  predict_op_margin_(declare_parameter<double>("predict_op_margin", 1.10)),
  min_lat_sep_(declare_parameter<double>("min_lat_sep", 1.66)),
  repulse_need_allow_(declare_parameter<bool>("repulse_need_allow", false)),
  repulse_band_margin_(declare_parameter<double>("repulse_band_margin", 0.0)),
  boost_side_gap_(declare_parameter<double>("boost_side_gap", 99.0)),
  min_pass_width_(declare_parameter<double>("min_pass_width", 1.8)),
  rear_end_lat_release_(declare_parameter<bool>("rear_end_lat_release", true)),
  // 車体が触れない横間隔[m]。車幅1.46 + 余裕0.2。これ未満では一切緩めない。
  rear_end_free_min_(declare_parameter<double>("rear_end_free_min", 1.45)),
  caution_zone_spec_(declare_parameter<std::string>("caution_zones", "165:175")),
  side_recheck_(declare_parameter<bool>("side_recheck", true)),
  side_recheck_sec_(declare_parameter<double>("side_recheck_sec", 0.5)),
  side_by_completion_(declare_parameter<bool>("side_by_completion", true)),
  side_plan_enable_(declare_parameter<bool>("side_plan_enable", true)),
  side_plan_reach_gain_(declare_parameter<double>("side_plan_reach_gain", 1.0)),
  // 0.0 -> 0.30。
  side_plan_room_margin_(declare_parameter<double>("side_plan_room_margin", 0.30)),
  side_plan_ay_max_(declare_parameter<double>("side_plan_ay_max", 23.0)),
  side_commit_(declare_parameter<bool>("side_commit", true)),
  side_plan_window_(declare_parameter<double>("side_plan_window", 0.0)),
  side_window_min_m_(declare_parameter<double>("side_window_min_m", 25.0)),
  side_need_rear_free_(declare_parameter<bool>("side_need_rear_free", true)),
  // true -> false。
  side_hold_when_none_(declare_parameter<bool>("side_hold_when_none", false)),
  allow_need_full_(declare_parameter<bool>("allow_need_full", true)),
  attempt_giveup_time_(declare_parameter<double>("attempt_giveup_time", 0.0)),
  side_search_in_zone_(declare_parameter<bool>("side_search_in_zone", true)),
  side_zone_end_clamp_(declare_parameter<bool>("side_zone_end_clamp", true)),
  side_plan_full_scale_(declare_parameter<bool>("side_plan_full_scale", true)),
  pass_need_room_m_(declare_parameter<double>("pass_need_room_m", 0.30)),
  side_zone_geom_default_(declare_parameter<bool>("side_zone_geom_default", false)),
  // 停止車の横を通るとき、要る横移動を「帯に入るまで」で測る(既定 on)。
  stopped_band_edge_(declare_parameter<bool>("stopped_band_edge", true)),
  // 先の閉まり方から逆算して、戻れない縁へ横目標を出さない。
  reachable_lat_clamp_(declare_parameter<bool>("reachable_lat_clamp", true)),
  reachable_lat_ahead_(declare_parameter<double>("reachable_lat_ahead", 6.0)),
  // 既定の側を「相手の反対」ではなく「いま出られる余地の大きい側」で決める。
  side_default_by_room_(declare_parameter<bool>("side_default_by_room", false)),
  side_default_room_hyst_(declare_parameter<double>("side_default_room_hyst", 0.30)),
  side_geom_tail_m_(declare_parameter<double>("side_geom_tail_m", 6.0)),
  side_geom_min_m_(declare_parameter<double>("side_geom_min_m", 12.0)),
  runup_use_plan_(declare_parameter<bool>("runup_use_plan", true)),
  // 助走の加速度にブーストぶんを足す(既定 on)。
  runup_boost_accel_(declare_parameter<bool>("runup_boost_accel", true)),
  // 抜き切り距離に加速フェーズを入れる(既定 on)。
  pass_dist_accel_(declare_parameter<bool>("pass_dist_accel", true)),
  // 抜き切り計算が「ブーストが要る」と答えたら助走ブーストを撃つ(既定 on)。
  pass_boost_gate_(declare_parameter<bool>("pass_boost_gate", true)),
  // 停止車の占有帯にコーナーの張り出し(L^2/8R)を足す(既定 on)。
  occupied_yaw_pad_(declare_parameter<bool>("occupied_yaw_pad", true)),
  // 車体の横方向の張り出しに、コーナーの振り出し(F^2/2R)を足す(既定 on)。
  body_curve_pad_(declare_parameter<bool>("body_curve_pad", true)),
  occupied_real_width_(declare_parameter<bool>("occupied_real_width", true)),
  side_meanlat_live_(declare_parameter<bool>("side_meanlat_live", true)),
  // 相手の横位置だけで決める枝でも、片側しか通せないならそちらを採る(既定 on)。
  side_fallback_room_(declare_parameter<bool>("side_fallback_room", true)),
  // 側を「相手と壁の間の空き」で決める(ライン基準のずれではなく)。
  side_room_first_(declare_parameter<bool>("side_room_first", true)),
  ot_lane_log_(declare_parameter<bool>("ot_lane_log", true)),
  // 連続長の差がこれ[m]を超えたら、その側を採る。
  side_run_tie_(declare_parameter<double>("side_run_tie", 3.0)),
  // 既定の側もライン基準ではなく通せる連続長で決める(既定 on)。
  side_default_run_(declare_parameter<bool>("side_default_run", true)),
  no_pass_attempt_hold_(declare_parameter<bool>("no_pass_attempt_hold", true)),
  no_pass_lat_hold_(declare_parameter<bool>("no_pass_lat_hold", false)),
  side_run_decide_(declare_parameter<bool>("side_run_decide", true)),
  size_pad_(declare_parameter<double>("size_pad", 0.25)),
  stopped_size_pad_(declare_parameter<double>("stopped_size_pad", 0.25)),
  sep_floor_enable_(declare_parameter<bool>("sep_floor_enable", true)),
  // 「通せる」とみなす連続長[m]。抜き切りに要る距離の目安。
  side_run_need_(declare_parameter<double>("side_run_need", 20.0)),
  side_completion_window_(declare_parameter<double>("side_completion_window", 0.5)),
  rear_end_predict_release_(declare_parameter<bool>("rear_end_predict_release", true)),
  rear_end_inpath_predict_(declare_parameter<bool>("rear_end_inpath_predict", false)),
  rear_end_inpath_max_sec_(declare_parameter<double>("rear_end_inpath_max_sec", 1.5)),
  rear_end_predict_floor_(declare_parameter<double>("rear_end_predict_floor", 0.35)),
  rear_end_predict_max_sec_(declare_parameter<double>("rear_end_predict_max_sec", 1.5)),
  // ここまで離れたら完全に開放する[m]
  rear_end_free_full_(declare_parameter<double>("rear_end_free_full", 2.20)),
  rear_end_free_speed_(declare_parameter<double>("rear_end_free_speed", 10.0)),
  min_pass_sep_(declare_parameter<double>("min_pass_sep", 1.15)),
  // 追い越しで相手から確保する横間隔の下限[m]。0 で無効。
  // 追突防止の解除条件(rear_end_free_min=1.66)を下回ると速度差が作れないので、
  // それより少し上に置く。コリドアで丸めるので狭い場所では効かない。
  pass_sep_floor_(declare_parameter<double>("pass_sep_floor", 1.80)),
  min_closing_kmh_(declare_parameter<double>("min_closing_kmh", 12.0)),
  pass_dist_max_(declare_parameter<double>("pass_dist_max", 45.0)),
  stopped_speed_(declare_parameter<double>("stopped_speed", 1.0)),
  stop_nopass_exit_speed_(declare_parameter<double>("stop_nopass_exit_speed", 1.5)),
  stop_nopass_release_sec_(declare_parameter<double>("stop_nopass_release_sec", 1.0)),
  stopped_look_ahead_(declare_parameter<double>("stopped_look_ahead", 30.0)),
  stop_margin_(declare_parameter<double>("stop_margin", 3.0)),
  stop_brake_k_(declare_parameter<double>("stop_brake_k", 0.30)),
  // 追突の待ちを判定するときに、制動距離へ足す余裕[m]。
  stop_hold_margin_(declare_parameter<double>("stop_hold_margin", 0.5)),
  stopped_cluster_span_(declare_parameter<double>("stopped_cluster_span", 8.0)),
  stopped_slack_(declare_parameter<double>("stopped_slack", 0.5)),
  band_enable_(declare_parameter<bool>("band_enable", true)),
  band_clamp_(declare_parameter<bool>("band_clamp", true)),
  band_predict_(declare_parameter<bool>("band_predict", true)),
  band_horizon_(declare_parameter<double>("band_horizon", 80.0)),
  band_long_(declare_parameter<double>("band_long", 2.6)),
  // 【2026-09-18 ユーザー指示】実寸+10cm にする。車体半幅 0.725 × 2 = 1.45m が中心間の物理下限で、
  // 1.30 では車体が 0.15m 重なる位置まで走行可能域を作っていた。
  band_car_w_(declare_parameter<double>("band_car_w", 1.55)),
  band_slope_(declare_parameter<double>("band_slope", 0.20)),
  band_smooth_(declare_parameter<double>("band_smooth", 0.30)),
  band_side_hyst_(declare_parameter<double>("band_side_hyst", 0.60)),
  band_lat_tau_(declare_parameter<double>("band_lat_tau", 15.0)),
  band_stop_speed_(declare_parameter<double>("band_stop_speed", 1.0)),
  band_side_follow_(declare_parameter<bool>("band_side_follow", true)),
  band_side_hold_(declare_parameter<double>("band_side_hold", 0.4)),
  side_fix_cool_(declare_parameter<double>("side_fix_cool", 2.0)),
  // 経路の横移動の上限[m/s]。0 で無効。
  // 既存の offset_rate(1.2) は **横目標** に掛かるが、その後のバンドの
  // クランプで上書きされるため経路には効いていない。こちらは最終出力に掛ける。
  path_rate_(declare_parameter<double>("path_rate", 2.0)),
  // 抜く側を相手の予測経路に沿って決めるか。既定オフ(未評価のため)。
  band_side_pred_(declare_parameter<bool>("band_side_pred", true)),
  pass_window_enable_(declare_parameter<bool>("pass_window_enable", true)),
  pass_window_gap_(declare_parameter<double>("pass_window_gap", 0.30)),
  pass_center_(declare_parameter<bool>("pass_center", true)),
  pass_window_sec_(declare_parameter<double>("pass_window_sec", 1.2)),
  pass_margin_min_(declare_parameter<double>("pass_margin_min", 0.40)),
  pass_margin_r_curve_(declare_parameter<double>("pass_margin_r_curve", 40.0)),
  // 曲率半径がこれ以上なら pass_margin_min まで詰める[m]
  pass_margin_r_straight_(declare_parameter<double>("pass_margin_r_straight", 100.0)),
  predict_check_sec_(declare_parameter<double>("predict_check_sec", 2.0)),
  predict_speed_fast_(declare_parameter<double>("predict_speed_fast", 16.0)),
  predict_speed_slow_(declare_parameter<double>("predict_speed_slow", 12.3)),
  predict_prior_samples_(declare_parameter<int>("predict_prior_samples", 60)),
  band_long_grow_(declare_parameter<double>("band_long_grow", 0.35)),
  band_lat_grow_(declare_parameter<double>("band_lat_grow", 0.05)),
  band_side_lead_(declare_parameter<bool>("band_side_lead", false)),
  cap_slow_log_kmh_(declare_parameter<double>("cap_slow_log_kmh", 20.0)),
  band_side_look_(declare_parameter<double>("band_side_look", 40.0)),
  launch_free_sec_(declare_parameter<double>("launch_free_sec", 6.0)),
  launch_free_gap_(declare_parameter<double>("launch_free_gap", 0.9)),
  launch_free_decel_(declare_parameter<double>("launch_free_decel", 0.48)),
  launch_free_react_(declare_parameter<double>("launch_free_react", 0.35)),
  launch_free_room_(declare_parameter<double>("launch_free_room", 1.20)),
  launch_stopped_grace_(declare_parameter<double>("launch_stopped_grace", 0.60)),
  look_width_ahead_(declare_parameter<double>("look_width_ahead", 20.0)),
  v2x_timeout_(declare_parameter<double>("v2x_timeout", 1.0)),
  safe_gap_(declare_parameter<double>("safe_gap", 3.0)),
  safe_gap_min_(declare_parameter<double>("safe_gap_min", 3.0)),
  safe_gap_max_(declare_parameter<double>("safe_gap_max", 5.0)),
  gap_brake_ratio_(declare_parameter<double>("gap_brake_ratio", 0.5)),
  a_min_(declare_parameter<double>("a_min", 2.5)),
  follow_kp_(declare_parameter<double>("follow_kp", 0.8)),
  // 車間がこれ[m]を上回っている間は、追従の上限を相手速度より下げない。
  // 接触は中心間 2.6m で起きるので、それに余裕を足した値。
  follow_keep_gap_(declare_parameter<double>("follow_keep_gap", 3.5)),
  min_follow_speed_(declare_parameter<double>("min_follow_speed", 2.2)),
  capped_self_enable_(declare_parameter<bool>("capped_self_enable", true)),
  capped_self_closing_(declare_parameter<double>("capped_self_closing", 3.0)),
  capped_self_dist_(declare_parameter<double>("capped_self_dist", 70.0)),
  // --- 横に出切ったら追従キャップを外して抜き切る ---
  // commit_sep は min_lat_sep と同じ値にしてある。回避層は
  // 「横間隔 >= min_lat_sep なら当たらない」として当該車を無視するので、
  // 同じ境界で追従キャップも手放すのが一貫する。
  commit_pass_(declare_parameter<bool>("commit_pass", true)),
  latch_commit_(declare_parameter<bool>("latch_commit", false)),
  commit_sep_(declare_parameter<double>("commit_sep", 1.30)),
  commit_gap_(declare_parameter<double>("commit_gap", 5.0)),
  // 解除のヒステリシス。ただし**車幅を下回らせない**(下の kCarWidth で床を張る)。
  // 0.85 のままだと 1.30*0.85 = 1.11m まで維持してしまい、
  // 重なった状態で加速を続けることになる。
  commit_release_(declare_parameter<double>("commit_release", 0.95)),
  // 抜き切りに入る前に、いまの横位置をこの距離[m]先まで保てるか確かめる。
  commit_look_(declare_parameter<double>("commit_look", 0.0)),
  // 横位置がこれ[m]以上潰されるなら「保てない」とみなす。
  commit_crush_(declare_parameter<double>("commit_crush", 0.30)),
  // 試行中に保持する横オフセットの最小値[m]。決めた側へこれだけは寄せ続ける。
  attempt_hold_side_(declare_parameter<bool>("attempt_hold_side", true)),
  attempt_hold_sep_(declare_parameter<double>("attempt_hold_sep", 1.30)),
  // 横に出られないまま粘る試行を打ち切るまでの秒数。0 で無効(既定)。
  // 有効にするときは 6.0 あたりから。棄却済みの attempt_stall_time とは
  // 見ているものが違う(進展ではなく「横に出られたか」)。
  attempt_latfail_time_(declare_parameter<double>("attempt_latfail_time", 0.0)),
  attempt_require_intent_(declare_parameter<bool>("attempt_require_intent", true)),
  latch_allow_enable_(declare_parameter<bool>("latch_allow_enable", true)),
  latch_never_no_pass_(declare_parameter<bool>("latch_never_no_pass", false)),
  stop_avoid_fix_(declare_parameter<bool>("stop_avoid_fix", true)),
  // 自車の車体(前後 約1.0m)+ 相手の車体 + 余裕。抜け切るまでを覆う。
  stop_avoid_band_local_(declare_parameter<bool>("stop_avoid_band_local", true)),
  stop_avoid_band_span_(declare_parameter<double>("stop_avoid_band_span", 2.0)),
  stop_avoid_span_(declare_parameter<double>("stop_avoid_span", 6.0)),
  stop_avoid_lat_lag_(declare_parameter<double>("stop_avoid_lat_lag", 20.0)),
  stop_avoid_emg_margin_(declare_parameter<double>("stop_avoid_emg_margin", 2.0)),
  stop_avoid_emg_accel_(declare_parameter<double>("stop_avoid_emg_accel", 1.7)),
  // 継続が成立しない状態がこの秒数続いたら試行を終える。0 で無効。
  attempt_infeasible_time_(declare_parameter<double>("attempt_infeasible_time", 1.5)),
  // 打切りの基準: いまの横位置のまま この秒数ぶん進んだときの縁までの余裕が
  // attempt_wall_abort_clear_ を下回る状態が attempt_infeasible_time_ 続いたら降りる。
  attempt_wall_look_time_(declare_parameter<double>("attempt_wall_look_time", 1.5)),
  attempt_wall_abort_clear_(declare_parameter<double>("attempt_wall_abort_clear", 0.10)),
  pass_width_dist_gain_(declare_parameter<double>("pass_width_dist_gain", 1.0)),
  // 幅を見る距離の上限[m]。抜き切る距離は速度差が小さいと発散するため頭を抑える。
  pass_width_dist_max_(declare_parameter<double>("pass_width_dist_max", 60.0)),
  // 並走がこの秒数続いても抜き切れないならブーストを撃つ。0 で無効。
  commit_boost_time_(declare_parameter<double>("commit_boost_time", 2.5)),
  // 追い越しゾーンが射程に入った時点で、並ぶ前にブーストを撃つ。
  // ブーストは10秒続くので、並んでから撃つのでは遅い。
  boost_runup_enable_(declare_parameter<bool>("boost_runup_enable", true)),
  boost_runup_gap_(declare_parameter<double>("boost_runup_gap", 20.0)),
  boost_runup_gap_min_(declare_parameter<double>("boost_runup_gap_min", 6.0)),
  wall_margin_(declare_parameter<double>("wall_margin", 0.65)),
  // 停止車の脇を抜けるときの壁の余裕[m]。通常の wall_margin より小さくする。
  // Wall(5秒 5km/h)より Crash(10秒 5km/h)のほうが倍高いので、
  // 壁に寄ってでも停止車を避けるのが正しい。
  wall_margin_stopped_(declare_parameter<double>("wall_margin_stopped", 0.30)),
  // 直線で追い越しにいくとき、壁とカートの間を通すために詰める余裕[m]。
  straight_pass_enable_(declare_parameter<bool>("straight_pass_enable", true)),
  straight_pass_wall_(declare_parameter<double>("straight_pass_wall", 0.25)),
  straight_pass_sep_(declare_parameter<double>("straight_pass_sep", 1.35)),
  // 区間を出てからも、仕掛けが終わるまでこの秒数[s]は同じ扱いを続ける。
  straight_pass_hold_(declare_parameter<double>("straight_pass_hold", 2.5)),
  // 直線の残り距離にこれだけ[m]足したものを「抜き切りに使える距離」とみなす。
  // コーナー入口へ少しはみ出して完了する形は許す。
  straight_finish_margin_(declare_parameter<double>("straight_finish_margin", 10.0)),
  // 停止車回避の横目標が壁帯でこれ[m]以上潰されたら「通れない」と判断する。
  stop_avoid_crush_(declare_parameter<double>("stop_avoid_crush", 0.10)),
  stop_avoid_fit_(declare_parameter<bool>("stop_avoid_fit", false)),
  stop_avoid_crush_range_(declare_parameter<double>("stop_avoid_crush_range", 8.0)),
  stop_creep_speed_(declare_parameter<double>("stop_creep_speed", 1.4)),
  stop_creep_gap_(declare_parameter<double>("stop_creep_gap", 1.8)),
  // 相手へ寄るときに確保する横間隔[m]。
  //
  // **必ず commit_sep(1.15)以上にすること。**
  // 初版は 1.00 にしていたが、壁回避の「相手側へ寄る」分岐は
  // `target_offset = 相手横 + crash_safe_sep` で横目標を上書きするので、
  // commit_sep を下回っていると**抜き切りモードの条件が原理的に成立しない**。
  // 実戦(ボード1位の走行)の解析で判明:
  //   - この分岐が 61 回発火し、押し戻し後の横目標は中央値 0.29m
  //   - 追従キャップが外れていたのは前方車がいた 319 行中 85 行(27%)だけ、
  //     解除の継続は中央値 0.9 秒
  //   - 追越失敗 21 件のうち 13 件(61%)で直前5秒にこの分岐が発火
  // 横の接触に罰則は無い(Crash は前から当てたときだけ)ので、
  // 離れる方向は安全側。狭所では後段の [wall_lo, wall_hi] クランプが効くため、
  // 値を上げても帯に余裕が無い場所の挙動は変わらない。
  // **必ず commit_sep より大きくすること。**
  // 壁回避がここで横目標を上書きするので、commit_sep 以下だと
  // 抜き切りモードが成立しなくなる(実戦解析 148 で実際に起きた)。
  crash_safe_sep_(declare_parameter<double>("crash_safe_sep", 1.45)),
  wedge_no_flip_(declare_parameter<bool>("wedge_no_flip", true)),
  wedge_forward_(declare_parameter<bool>("wedge_forward", false)),
  wedge_keep_room_(declare_parameter<bool>("wedge_keep_room", false)),
  wall_pick_need_(declare_parameter<double>("wall_pick_need", 0.55)),
  corridor_extra_(declare_parameter<double>("corridor_extra", 0.35)),
  wall_pick_legacy_(declare_parameter<bool>("wall_pick_legacy", false)),
  // 相手の車体が自分の前にあるとみなす前後距離[m]。この範囲なら寄せると Crash。
  crash_front_near_(declare_parameter<double>("crash_front_near", -0.5)),
  crash_front_far_(declare_parameter<double>("crash_front_far", 3.0)),
  // 壁にも相手にも寄れないときの減速率(現在速度に対する比)
  wall_brake_ratio_(declare_parameter<double>("wall_brake_ratio", 0.6)),
  // きついコーナーで壁の余裕を増やす。半径がこの値を下回ると効き始める。
  tight_radius_(declare_parameter<double>("tight_radius", 10.0)),
  wall_margin_tight_(declare_parameter<double>("wall_margin_tight", 0.35)),
  enable_(declare_parameter<bool>("enable", true)),
  // スタート横位置の絶対上限[m]。通常はコリドアで丸めるので効かない。
  // 自己位置が飛んだときの保険。
  start_lat_abs_(declare_parameter<double>("start_lat_abs", 3.4)),
  start_wall_margin_(declare_parameter<double>("start_wall_margin", 0.35)),
  launch_hold_grid_(declare_parameter<bool>("launch_hold_grid", true)),
  launch_hold_dist_(declare_parameter<double>("launch_hold_dist", 12.0)),
  launch_hold_sec_(declare_parameter<double>("launch_hold_sec", 12.0)),
  launch_p1_right_(declare_parameter<bool>("launch_p1_right", false)),
  launch_p1_lat_near_(declare_parameter<double>("launch_p1_lat_near", -0.90)),
  launch_p1_lat_far_(declare_parameter<double>("launch_p1_lat_far", -3.20)),
  launch_p1_clear_(declare_parameter<double>("launch_p1_clear", 4.5)),
  launch_p1_dist_(declare_parameter<double>("launch_p1_dist", 30.0)),
  launch_p1_pass_(declare_parameter<bool>("launch_p1_pass", false)),
  launch_p1_pass_sec_(declare_parameter<double>("launch_p1_pass_sec", 25.0)),
  launch_p1_pass_sep_(declare_parameter<double>("launch_p1_pass_sep", 1.60)),
  launch_p1_pass_ahead_(declare_parameter<double>("launch_p1_pass_ahead", 3.0)),
  launch_p1_lat_step_(declare_parameter<double>("launch_p1_lat_step", 2.00)),
  launch_p1_hold_until_pass_(declare_parameter<bool>("launch_p1_hold_until_pass", true)),
  launch_no_pass_guard_dist_(declare_parameter<double>("launch_no_pass_guard_dist", 12.0)),
  // 合流が終わるまでの横方向のレート[m/s]
  start_offset_rate_(declare_parameter<double>("start_offset_rate", 3.0)),
  // --- 追い越し地点の計画(録画からの逆算) ---
  spot_enable_(declare_parameter<bool>("spot_enable", true)),
  spot_range_(declare_parameter<double>("spot_range", 160.0)),
  spot_min_len_(declare_parameter<double>("spot_min_len", 8.0)),
  spot_recalc_sec_(declare_parameter<double>("spot_recalc_sec", 0.5)),
  spot_slow_bonus_(declare_parameter<double>("spot_slow_bonus", 3.0)),
  // 実行側の追突防止は横間隔1.66m未満では速度上限を解除しない。
  // ここが1.15mだと「計画上は抜けるが実行時は相手速度に張り付く」地点を
  // 選ぶため、さらに余裕を持たせた2.0mを計画下限にする。
  // 1.8m はカート実幅 1.46m(半幅0.73×2)に対し過大で、連続空き区間の長さを
  // 大きく削っていた。接触しない下限は band_car_w によるクランプ層が保証する。
  spot_margin_(declare_parameter<double>("spot_margin", 1.55)),
  // 幾何条件を1点でも割ると区間を閉じていたため、本来つながっている空きが
  // 細切れになっていた。この長さ[m]以内の途切れならまたいで1本の区間として扱う。
  spot_run_gap_(declare_parameter<double>("spot_run_gap", 2.0)),
  spot_gate_(declare_parameter<bool>("spot_gate", true)),
  spot_gate_slack_(declare_parameter<double>("spot_gate_slack", 12.0)),
  // 抜きどころ待ちがこの秒数[s]続いたら、待つのをやめて手前の機会を使う。
  // 8秒で待機を解除すると、100m先に安全な抜きどころを見つけても途中の
  // zone外で仕掛け直していた。低速追従でも1周以内に到達できる時間を待つ。
  lat_lag_by_speed_(declare_parameter<bool>("lat_lag_by_speed", true)),
  lat_lag_sec_(declare_parameter<double>("lat_lag_sec", 0.8)),
  stop_nopass_release_on_pass_(
    declare_parameter<bool>("stop_nopass_release_on_pass", true)),
  lat_lag_base_(declare_parameter<double>("lat_lag_base", 1.0)),
  lat_lag_max_(declare_parameter<double>("lat_lag_max", 60.0)),
  prepare_early_(declare_parameter<bool>("prepare_early", true)),
  prepare_early_margin_(declare_parameter<double>("prepare_early_margin", 1.0)),
  spot_look_local_(declare_parameter<bool>("spot_look_local", true)),
  spot_gate_max_wait_(declare_parameter<double>("spot_gate_max_wait", 45.0)),
  spot_path_tol_(declare_parameter<double>("spot_path_tol", 0.25)),
  zone_free_(declare_parameter<bool>("zone_free", false)),
  zone_free_brake_decel_(declare_parameter<double>("zone_free_brake_decel", 2.4)),
  zone_free_keep_(declare_parameter<double>("zone_free_keep", 1.0)),
  // 上記の許容を連続して超えてよい区間長の上限[m]。これを超えたら閉塞とみなす。
  spot_path_bad_len_(declare_parameter<double>("spot_path_bad_len", 2.0)),
  spot_path_min_w_(declare_parameter<double>("spot_path_min_w", 1.5)),
  spot_w_ref_(declare_parameter<bool>("spot_w_ref", false)),
  spot_w_ref_min_(declare_parameter<double>("spot_w_ref_min", 0.25)),
  spot_stuck_max_(declare_parameter<double>("spot_stuck_max", 2.0)),
  // 破棄した地点をこの秒数[s]の間だけ候補から外す。
  spot_avoid_sec_(declare_parameter<double>("spot_avoid_sec", 8.0)),
  spot_abort_sec_(declare_parameter<double>("spot_abort_sec", 0.8)),
  // 録画から見た左右の平均空き幅の差がこれ[m]を超えたら、広いほうから抜く。
  // 反対側がこの秒数[s]連続で勝っていないと側を入れ替えない(振られ対策)。
  side_room_hold_(declare_parameter<double>("side_room_hold", 0.8)),
  side_pick_zone_spec_(declare_parameter<std::string>("side_pick_zones", "0:241")),
  pass_finish_zone_spec_(declare_parameter<std::string>("pass_finish_zones", "230:25")),
  // 相手の区間平均の横位置がこれ[m]以内なら「どちらとも言えない」-> 右から抜く。
  side_pick_tie_(declare_parameter<double>("side_pick_tie", 0.30)),
  rear_end_guard_(declare_parameter<bool>("rear_end_guard", true)),
  rear_end_range_(declare_parameter<double>("rear_end_range", 20.0)),
  // 横間隔がこれ[m]未満なら「自分の進路上」。車幅 1.30m に余裕を足す。
  rear_end_sep_(declare_parameter<double>("rear_end_sep", 1.45)),
  // 止まりきる位置に残す余裕[m]。カート全長の半分ぶん。
  rear_end_margin_(declare_parameter<double>("rear_end_margin", 3.5)),
  start_gap_floor_(declare_parameter<double>("start_gap_floor", 1.5)),
  rear_end_brake_k_(declare_parameter<double>("rear_end_brake_k", 0.22)),
  rear_end_brake_k_pass_(declare_parameter<double>("rear_end_brake_k_pass", 0.45)),
  // 接触位置へ食い込んだとき、1m につきこれだけ[m/s]相手より遅くする。
  rear_end_back_k_(declare_parameter<double>("rear_end_back_k", 0.8)),
  // 反応の遅れとして見込む時間[s]。この間に進む距離を車間から引く。
  rear_end_time_(declare_parameter<double>("rear_end_time", 0.15)),
  // 前に車がいるとき、これから通る何m先までの帯で横目標を丸めるか。
  squeeze_ahead_(declare_parameter<double>("squeeze_ahead", 0.0)),
  // 前の車がこの車間[m]以内のときだけ、先の帯で横目標を丸める。
  squeeze_gap_(declare_parameter<double>("squeeze_gap", 8.0)),
  // 追突防止の上限を下げるときの最大の減速率[m/s^2]。
  // Over ペナルティは |加速度| > 3.0 で付くので、その下に収める。
  // 速度上限の下げ方に全層まとめて掛けるレート制限。
  // Over は |加速度| > 3.0 で付くので、その下に収める。
  cap_rate_limit_(declare_parameter<bool>("cap_rate_limit", true)),
  cap_decel_(declare_parameter<double>("cap_decel", 2.5)),
  // コーナーでは横ずれで進路上かを判定できない。この距離[m]まで近く、
  // かつ接近速度がある相手は、横ずれに関係なく見る。
  // 軌道座標の横間隔だけでは、コーナーで鼻先が相手を向く接近を見落とす。
  // P1検証では gap=3.3m の d3 に 20.7km/h で近づいた際、横間隔の見積りが
  // 一瞬だけ境界外となって制動せず、0.25秒後に接触した。直線距離4.5m以内で
  // 十分に接近している場合は横間隔にかかわらず追突制動へ入れる。
  rear_end_near_(declare_parameter<double>("rear_end_near", 4.5)),
  rear_end_near_closing_(declare_parameter<double>("rear_end_near_closing", 1.0)),
  // 車間がこれ[m]未満なら、相手の横位置をまたぐ横目標を出さない。
  // 横断に要る時間(2.17s)× 接近速度(3〜4.5m/s)から。
  cross_gap_(declare_parameter<double>("cross_gap", 0.0)),
  cross_dead_(declare_parameter<double>("cross_dead", 0.30)),
  lap_gate_enable_(declare_parameter<bool>("lap_gate_enable", false)),
  npc_slot_(declare_parameter<int>("npc_slot", 3)),
  record_laps_(declare_parameter<int>("record_laps", 1)),
  teammate_pass_lap_(declare_parameter<int>("teammate_pass_lap", 1)),
  leader_pass_last_laps_(declare_parameter<int>("leader_pass_last_laps", 0)),
  zone_fallback_enable_(declare_parameter<bool>("zone_fallback_enable", false)),
  ot_lane_enable_(declare_parameter<bool>("ot_lane_enable", true)),
  // BLOCK(20秒)を避けるガード。レーンを追い越しに使うかとは独立に常時有効。
  ot_lane_guard_(declare_parameter<bool>("ot_lane_guard", true)),
  lane_guard_start_off_(
    declare_parameter<bool>("lane_guard_start_off", true)),
  lane_guard_off_from_idx_(
    declare_parameter<int>("lane_guard_off_from_idx", 230)),
  lane_guard_off_to_idx_(
    declare_parameter<int>("lane_guard_off_to_idx", 30)),
  // 車体の向きが崩れているぶん横の許容範囲を狭める。
  yaw_margin_enable_(declare_parameter<bool>("yaw_margin_enable", true)),
  // 車体の半長[m]。原点(後軸)から前端 1.60 / 後端 0.60 なので、
  // 回転で横に張り出す量としては前側の 1.60 を使う(安全側)。
  yaw_margin_half_len_(declare_parameter<double>("yaw_margin_half_len", 1.60)),
  // ガードを効かせ始める先読み距離。横位置が実現するまでの約20m。
  ot_lane_guard_look_(declare_parameter<double>("ot_lane_guard_look", 20.0)),
  ot_lane_guard_time_(declare_parameter<double>("ot_lane_guard_time", 0.5)),
  // 27km/h がアタッカーの閾値。境界で振動しないよう 1km/h の余裕を持たせる。
  ot_lane_min_kmh_(declare_parameter<double>("ot_lane_min_kmh", 28.0)),
  prepare_free_enable_(declare_parameter<bool>("prepare_free_enable", true)),
  // 「相手が近い」の範囲。20m は 36km/h で 2.0 秒、横位置の実現遅れ(約20m)と
  // ちょうど同じ。これより手前から寄せ始めないと横が間に合わない。
  prepare_free_gap_(declare_parameter<double>("prepare_free_gap", 20.0)),
  pre_position_enable_(declare_parameter<bool>("pre_position_enable", true)),
  spot_keep_completion_side_(
    declare_parameter<bool>("spot_keep_completion_side", false)),
  pre_position_range_(declare_parameter<double>("pre_position_range", 45.0)),
  pre_position_always_(declare_parameter<bool>("pre_position_always", true)),
  pre_position_gain_(declare_parameter<double>("pre_position_gain", 0.6)),
  pre_position_pass_ok_(declare_parameter<bool>("pre_position_pass_ok", false)),
  // の車体位置でコリドア/他車への食い込みを見る層。
  rear_end_sep_plan_ok_(
    declare_parameter<bool>("rear_end_sep_plan_ok", true)),
  rear_end_stop_band_(declare_parameter<bool>("rear_end_stop_band", true)),
  follow_skip_stop_pass_(
    declare_parameter<bool>("follow_skip_stop_pass", true)),
  stopped_bound_enable_(
    declare_parameter<bool>("stopped_bound_enable", true)),
  stopped_bound_range_(
    declare_parameter<double>("stopped_bound_range", 25.0)),
  fwd_clear_floor_(declare_parameter<bool>("fwd_clear_floor", true)),
  fwd_clear_need_m_(declare_parameter<double>("fwd_clear_need_m", 6.0)),
  fwd_clear_floor_kmh_(declare_parameter<double>("fwd_clear_floor_kmh", 8.0)),
  rear_audit_(declare_parameter<bool>("rear_audit", true)),
  rear_audit_sec_(declare_parameter<double>("rear_audit_sec", 0.5)),
  runup_hold_gap_k_(declare_parameter<double>("runup_hold_gap_k", 1.35)),
  runup_hold_until_gap_(
    declare_parameter<bool>("runup_hold_until_gap", true)),
  runup_hold_ratio_(declare_parameter<double>("runup_hold_ratio", 0.9)),
  runup_hold_margin_k_(
  declare_parameter<double>("runup_hold_margin_k", 1.0)),
  runup_react_sec_(declare_parameter<double>("runup_react_sec", 0.35)),
  runup_hold_lat_(declare_parameter<bool>("runup_hold_lat", true)),
  runup_hold_block_charge_(
    declare_parameter<bool>("runup_hold_block_charge", true)),
  runup_hold_gap_abs_(
    declare_parameter<double>("runup_hold_gap_abs", 16.0)),
  runup_hold_min_room_m_(
    declare_parameter<double>("runup_hold_min_room_m", 28.0)),
  runup_open_from_idx_(declare_parameter<int>("runup_open_from_idx", 190)),
  runup_open_to_idx_(declare_parameter<int>("runup_open_to_idx", 241)),
  runup_open_leader_only_(
    declare_parameter<bool>("runup_open_leader_only", true)),
  runup_open_keep_reach_(
    declare_parameter<bool>("runup_open_keep_reach", true)),
  runup_open_dv_auto_(declare_parameter<bool>("runup_open_dv_auto", true)),
  runup_open_dv_max_(declare_parameter<double>("runup_open_dv_max", 2.0)),
  runup_open_look_m_(declare_parameter<double>("runup_open_look_m", 90.0)),
  runup_gap_open_cmd_(declare_parameter<bool>("runup_gap_open_cmd", true)),
  runup_charge_relax_(declare_parameter<bool>("runup_charge_relax", true)),
  runup_charge_follow_gap_(
    declare_parameter<double>("runup_charge_follow_gap", 3.5)),
  start_hold_priority_(declare_parameter<bool>("start_hold_priority", true)),
  npz_look_by_lat_(declare_parameter<bool>("npz_look_by_lat", true)),
  npz_look_max_m_(declare_parameter<double>("npz_look_max_m", 25.0)),
  ot_lane_win_safety_(declare_parameter<double>("ot_lane_win_safety", 0.15)),
  ot_lane_aim_seek_(declare_parameter<bool>("ot_lane_aim_seek", true)),
  ot_lane_aim_seek_max_(
    static_cast<std::size_t>(declare_parameter<int>("ot_lane_aim_seek_max", 30))),
  pass_gap_margin_(declare_parameter<double>("pass_gap_margin", 0.10)),
  pass_wall_keep_(declare_parameter<double>("pass_wall_keep", 0.25)),
  ot_abort_in_nopass_(declare_parameter<bool>("ot_abort_in_nopass", true)),
  body_guard_enable_(declare_parameter<bool>("body_guard_enable", true)),
  body_guard_after_merge_(
    declare_parameter<bool>("body_guard_after_merge", true)),
  start_merge_smooth_(
    declare_parameter<bool>("start_merge_smooth", true)),
  body_guard_react_m_(declare_parameter<double>("body_guard_react_m", 0.0)),
  body_guard_hard_m_(declare_parameter<double>("body_guard_hard_m", 0.35)),
  body_guard_cap_kmh_(declare_parameter<double>("body_guard_cap_kmh", 20.0)),
  pre_position_sep_(declare_parameter<bool>("pre_position_sep", true)),
  pre_position_sep_extra_(declare_parameter<double>("pre_position_sep_extra", 0.15)),
  pre_reject_log_(declare_parameter<bool>("pre_reject_log", true)),
  accel_audit_(declare_parameter<bool>("accel_audit", true)),
  start_gap_by_rearend_(
    declare_parameter<bool>("start_gap_by_rearend", true)),
  // 横目標での先読み解除は効果なし。
  rear_end_target_release_(
    declare_parameter<bool>("rear_end_target_release", false)),
  rear_end_lat_plan_(
    declare_parameter<bool>("rear_end_lat_plan", true)),
  rear_end_target_blend_(
    declare_parameter<double>("rear_end_target_blend", 0.5)),
  runup_fuel_(declare_parameter<bool>("runup_fuel", true)),
  runup_fuel_body_(declare_parameter<bool>("runup_fuel_body", true)),
  runup_sim_(declare_parameter<bool>("runup_sim", true)),
  runup_sim_opp_model_(
    declare_parameter<bool>("runup_sim_opp_model", true)),
  opp_accel_mps2_(declare_parameter<double>("opp_accel_mps2", 0.8)),
  gate_runup_in_lane_(
    declare_parameter<bool>("gate_runup_in_lane", true)),
  gate_runup_ref_gate_(
    declare_parameter<bool>("gate_runup_ref_gate", true)),
  rear_end_sep_true_(
    declare_parameter<bool>("rear_end_sep_true", true)),
  normal_gap_by_rearend_(
    declare_parameter<bool>("normal_gap_by_rearend", true)),
  normal_gap_max_(declare_parameter<double>("normal_gap_max", 14.0)),
  start_gap_need_max_(declare_parameter<double>("start_gap_need_max", 14.0)),
  // PREPARE でも事前寄せを続けると**悪化**した。
  pre_position_in_prepare_(
    declare_parameter<bool>("pre_position_in_prepare", false)),
  // 自由走行で到達できる速度が相手より速いこと。1位は 25km/h に固定され
  // 自分は 36km/h 出せるので構造的な差は 3.06m/s。その 1/3 を要求する。
  prepare_free_vgain_(declare_parameter<double>("prepare_free_vgain", 1.0)),
  // 希望が消えてから PREPARE を降りるまでの猶予。0.6s は横位置が
  // 指令から実現するまでの遅れ(約20m ≒ 2s)より短く、
  // 「1周期の揺れでは降りない」を満たす最小限。
  prepare_free_grace_(declare_parameter<double>("prepare_free_grace", 0.6)),
  ot_lane_prepare_(declare_parameter<bool>("ot_lane_prepare", true)),
  ot_lane_prepare_look_(declare_parameter<double>("ot_lane_prepare_look", 18.0)),
  ot_lane_prepare_time_(declare_parameter<double>("ot_lane_prepare_time", 0.8)),
  ot_lane_inset_(declare_parameter<double>("ot_lane_inset", 0.35)),
  body_margin_asym_(declare_parameter<bool>("body_margin_asym", true)),
  geom_front_(declare_parameter<double>("geom_front", 1.554)),
  geom_rear_(declare_parameter<double>("geom_rear", 0.510)),
  geom_half_width_(declare_parameter<double>("geom_half_width", 0.725)),
  wall_body_span_(declare_parameter<bool>("wall_body_span", true)),
  hold_side_alongside_(declare_parameter<bool>("hold_side_alongside", true)),
  alongside_extra_(declare_parameter<double>("alongside_extra", 0.10)),
  ot_lane_use_zone_spec_(
    declare_parameter<std::string>("ot_lane_use_zones", "239:17")),
  opp_model_enable_(declare_parameter<bool>("opp_model_enable", true)),
  spot_all_slots_(declare_parameter<bool>("spot_all_slots", true)),
  ot_lane_guard_lat_(declare_parameter<double>("ot_lane_guard_lat", 2.0)),
  ot_lane_side_right_(declare_parameter<bool>("ot_lane_side_right", true)),
  ot_lane_guard_predict_(declare_parameter<bool>("ot_lane_guard_predict", true)),
  ot_lane_side_sticky_(declare_parameter<bool>("ot_lane_side_sticky", true)),
  ot_lane_side_over_spot_(declare_parameter<bool>("ot_lane_side_over_spot", true)),
  ot_lane_guard_geom_(declare_parameter<bool>("ot_lane_guard_geom", true)),
  ot_lane_lat_spec_(declare_parameter<std::string>("ot_lane_lat", "234:-2.55:-2.30,235:-3.00:-0.40,236:-3.40:-0.80,237:-3.75:-1.15,238:-4.00:-1.50,239:-4.20:-1.70,240:-4.40:-1.90,241:-4.50:-2.05,0:-4.60:-2.15,1:-4.65:-2.20,2:-4.70:-2.25,3:-4.75:-2.30,4:-4.75:-2.30,5:-4.80:-2.35,6:-4.85:-2.40,7:-4.85:-2.40,8:-4.90:-2.45,9:-5.00:-2.55,10:-5.05:-2.60,11:-5.15:-2.70,12:-5.25:-2.80,13:-5.30:-2.85,14:-5.40:-2.95,15:-5.45:-3.00,16:-5.45:-3.00,17:-5.40:-2.95,18:-5.30:-2.80,19:-5.10:-2.65,20:-4.90:-2.35,21:-4.60:-2.00,48:-6.00:-5.95,49:-6.00:-5.85,50:-6.00:-5.80,51:-6.00:-5.95")),
  // 車体はヨーすると横に広がる。半幅 b、半長 a の矩形なら横の張り出しは
  // b|cos psi| + a|sin psi| で、a=1.03m なので 5度で +0.09m、10度で +0.17m。
  // 触れない判定は安全側に倒したいので、その分を足す。
  ot_lane_touch_margin_(declare_parameter<double>("ot_lane_touch_margin", 0.15)),
  // 側の決定: 録画で決めた側を曲率のイン優先より優先するか(コメントどおりの実装)
  side_pick_over_curve_(declare_parameter<bool>("side_pick_over_curve", true)),
  // 側の決定: 「相手と壁の空き」で決めるか(false でレースラインからのずれ)
  side_pick_by_room_(declare_parameter<bool>("side_pick_by_room", true)),
  // 側を「区間の平均の空き」ではなく「その側が途切れず続く距離」で決めるか。
  side_pick_by_run_(declare_parameter<bool>("side_pick_by_run", false)),
  // 決め直しゾーンが変わったことを側の決め直しの契機にするか。
  side_zone_repick_(declare_parameter<bool>("side_zone_repick", false)),
  // 相手の区間平均横を「いる区間のbinだけ」で取るか。
  side_zone_mean_split_(declare_parameter<bool>("side_zone_mean_split", false)),
  curve_side_enable_(declare_parameter<bool>("curve_side_enable", true)),
  curve_side_in_zone_(declare_parameter<bool>("curve_side_in_zone", true)),
  side_room_use_min_(declare_parameter<bool>("side_room_use_min", true)),
  attempt_lat_hold_enable_(declare_parameter<bool>("attempt_lat_hold_enable", false)),
  spot_entry_gap_(declare_parameter<double>("spot_entry_gap", -1.0)),
  // 相手の前へ出切る量に掛ける係数。1.0 で pass_len そのまま。
  spot_pass_len_gain_(declare_parameter<double>("spot_pass_len_gain", 1.0)),
  // 必要距離に掛ける安全率。1.0 で計算どおり。
  spot_need_margin_(declare_parameter<double>("spot_need_margin", 1.15)),
  boost_min_lap_(declare_parameter<int>("boost_min_lap", 2)),
  trace_dump_sec_(declare_parameter<double>("trace_dump_sec", 30.0)),
  telem_sec_(declare_parameter<double>("telem_sec", 20.0)),
  overtaken_margin_(declare_parameter<double>("overtaken_margin", 2.0)),
  overtaken_hold_(declare_parameter<double>("overtaken_hold", 0.5)),
  contact_decel_(declare_parameter<double>("contact_decel", 3.0)),
  contact_alone_dist_(declare_parameter<double>("contact_alone_dist", 8.0)),
  contact_after_pass_sec_(declare_parameter<double>("contact_after_pass_sec", 3.0)),
  runup_enable_(declare_parameter<bool>("runup_enable", true)),
  runup_dv_(declare_parameter<double>("runup_dv", 2.5)),
  runup_gap_max_(declare_parameter<double>("runup_gap_max", 40.0)),
  runup_margin_(declare_parameter<double>("runup_margin", 3.0)),
  runup_accel_mps2_(declare_parameter<double>("runup_accel_mps2", 3.2)),
  ot_lane_runup_(declare_parameter<bool>("ot_lane_runup", true)),
  ot_lane_runup_look_(declare_parameter<double>("ot_lane_runup_look", 45.0)),
  ot_lane_runup_margin_kmh_(
    declare_parameter<double>("ot_lane_runup_margin_kmh", 3.5)),
  // 試算が門にわずかに届かないだけなら見送らない[km/h]。
  runup_gate_slack_kmh_(
    declare_parameter<double>("runup_gate_slack_kmh", 1.0)),
  ot_lane_entry_pre_m_(
    declare_parameter<double>("ot_lane_entry_pre_m", 3.0)),
  gate_runup_charge_(declare_parameter<bool>("gate_runup_charge", true)),
  gate_runup_margin_m_(declare_parameter<double>("gate_runup_margin_m", 2.0)),
  boost_only_if_decisive_(
    declare_parameter<bool>("boost_only_if_decisive", true)),
  gate_runup_sidestep_(declare_parameter<bool>("gate_runup_sidestep", true)),
  gate_sidestep_extra_(declare_parameter<double>("gate_sidestep_extra", 0.25)),
  stopped_aim_edge_(declare_parameter<bool>("stopped_aim_edge", true)),
  stopped_edge_inset_(declare_parameter<double>("stopped_edge_inset", 0.70)),
  stopped_keep_pass_exclude_(
    declare_parameter<bool>("stopped_keep_pass_exclude", false)),
  ot_lane_aim_(declare_parameter<bool>("ot_lane_aim", true)),
  runup_gap_hold_(declare_parameter<bool>("runup_gap_hold", true)),
  runup_gap_cap_(declare_parameter<bool>("runup_gap_cap", true)),
  // 発火が 8台中 3回/2回 しか無く、。
  runup_gap_open_(declare_parameter<bool>("runup_gap_open", true)),
  runup_open_dv_(declare_parameter<double>("runup_open_dv", 1.0)),
  runup_open_rear_min_(declare_parameter<double>("runup_open_rear_min", -1.0)),
  // 目標を門+余裕(29.5km/h)に抑えたところ、。
  gate_runup_target_gate_(
    declare_parameter<bool>("gate_runup_target_gate", false)),
  // 加速開始点だけを門基準にしても差が出ない。
  gate_runup_late_(declare_parameter<bool>("gate_runup_late", false)),
  ot_lane_aim_look_(declare_parameter<double>("ot_lane_aim_look", 20.0)),
  ot_lane_aim_inset_(declare_parameter<double>("ot_lane_aim_inset", 0.15)),
  stop_creep_by_band_(declare_parameter<bool>("stop_creep_by_band", true)),
  stop_creep_slow_(declare_parameter<double>("stop_creep_slow", 1.0)),
  stop_creep_gap_min_(declare_parameter<double>("stop_creep_gap_min", 1.2))
{
  if (sep_floor_enable_) {
    const double phys = geom_half_width_ * 2.0;          // 1.45m
    const double floor_v = phys + std::max(size_pad_, 0.0);
    auto raise = [&](const char * nm, double & v) {
      if (v < floor_v - 1e-9) {
        RCLCPP_WARN(get_logger(), "横間隔の床 %s %.2f -> %.2f m (物理%.2f + 余裕%.2f)",
                    nm, v, floor_v, phys, size_pad_);
        v = floor_v;
      }
    };
    raise("min_pass_sep", min_pass_sep_);
    raise("band_car_w", band_car_w_);
    raise("commit_sep", commit_sep_);
    raise("attempt_hold_sep", attempt_hold_sep_);
    raise("pass_beside_sep", pass_beside_sep_);
  }

  const auto csv = declare_parameter<std::string>("corridor_csv", "");
  if (!csv.empty() && !loadCorridor(csv)) {
    RCLCPP_ERROR(get_logger(), "corridor_csv を読めない: %s", csv.c_str());
  }

  // "220:241" または "10:20,220:241" の形を解析する。
  // 添字はレースラインの点番号で、start > end は 0 をまたぐ区間を表す
  // (またぎの扱いは参照側でやっているので、ここでは値をそのまま入れる)。
  {
    std::stringstream ss(boost_zone_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      const std::size_t c = item.find(':');
      if (c == std::string::npos) { continue; }
      const auto a = static_cast<std::size_t>(std::stoul(item.substr(0, c)));
      const auto b = static_cast<std::size_t>(std::stoul(item.substr(c + 1)));
      boost_zones_.emplace_back(a, b);
      RCLCPP_INFO(get_logger(), "加速区間 idx%zu-%zu", a, b);
    }
  }

  // 追い越し禁止区間。書式と 0 またぎの扱いは boost_zones と同じ。
  {
    std::stringstream ss(no_pass_zone_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      const std::size_t c = item.find(':');
      if (c == std::string::npos) { continue; }
      const auto a = static_cast<std::size_t>(std::stoul(item.substr(0, c)));
      const auto b = static_cast<std::size_t>(std::stoul(item.substr(c + 1)));
      no_pass_zones_.emplace_back(a, b);
      RCLCPP_INFO(get_logger(), "追越禁止区間 idx%zu-%zu", a, b);
    }
  }

  {
    std::stringstream ss(ot_lane_zone_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      const std::size_t c = item.find(':');
      if (c == std::string::npos) { continue; }
      const auto a = static_cast<std::size_t>(std::stoul(item.substr(0, c)));
      const auto b = static_cast<std::size_t>(std::stoul(item.substr(c + 1)));
      ot_lane_zones_.emplace_back(a, b);
      RCLCPP_INFO(get_logger(),
        "オーバーテイクレーン idx%zu-%zu 使用=%s 要速度=%.0fkm/h 低速時の右上限=%.2fm",
        a, b, ot_lane_enable_ ? "する" : "しない", ot_lane_min_kmh_, ot_lane_guard_lat_);
    }
  }
  {
    std::stringstream ss(ot_lane_use_zone_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      const std::size_t c = item.find(':');
      if (c == std::string::npos) { continue; }
      const auto a = static_cast<std::size_t>(std::stoul(item.substr(0, c)));
      const auto b = static_cast<std::size_t>(std::stoul(item.substr(c + 1)));
      ot_lane_use_zones_.emplace_back(a, b);
      RCLCPP_INFO(get_logger(),
        "レーンで車体を入れられる区間 idx%zu-%zu (コリドアの右端から %.2fm 内側を狙う)",
        a, b, ot_lane_inset_);
    }
  }
  // --- レーンの横範囲("idx:lo:hi,...")---
  {
    std::stringstream ss(ot_lane_lat_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      const std::size_t c1 = item.find(':');
      if (c1 == std::string::npos) { continue; }
      const std::size_t c2 = item.find(':', c1 + 1);
      if (c2 == std::string::npos) { continue; }
      try {
        const auto k = static_cast<std::size_t>(std::stoul(item.substr(0, c1)));
        const double zlo = std::stod(item.substr(c1 + 1, c2 - c1 - 1));
        const double zhi = std::stod(item.substr(c2 + 1));
        ot_lane_lat_[k] = {std::min(zlo, zhi), std::max(zlo, zhi)};
      } catch (...) {
        RCLCPP_WARN(get_logger(), "ot_lane_lat を読めない項目: %s", item.c_str());
      }
    }
    RCLCPP_INFO(get_logger(),
      "レーンの横範囲 %zu点 読み込み(触れない右限に足す余裕 %.2fm)",
      ot_lane_lat_.size(), ot_lane_touch_margin_);
  }
  {
    std::stringstream ss(right_zone_spec_);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const auto c = tok.find(':');
      if (c == std::string::npos) { continue; }
      try {
        const std::size_t a = static_cast<std::size_t>(std::stoul(tok.substr(0, c)));
        const std::size_t b = static_cast<std::size_t>(std::stoul(tok.substr(c + 1)));
        right_zones_.emplace_back(a, b);
        RCLCPP_INFO(get_logger(), "右側から抜く区間 idx%zu-%zu", a, b);
      } catch (...) { }
    }
  }
  {
    std::stringstream ss(side_pick_zone_spec_);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const auto c = tok.find(':');
      if (c == std::string::npos) { continue; }
      try {
        const std::size_t a = static_cast<std::size_t>(std::stoul(tok.substr(0, c)));
        const std::size_t b = static_cast<std::size_t>(std::stoul(tok.substr(c + 1)));
        side_pick_zones_.emplace_back(a, b);
        RCLCPP_INFO(get_logger(), "側を録画で決める区間 idx%zu-%zu", a, b);
      } catch (...) { }
    }
  }
  {
    std::stringstream ss(caution_zone_spec_);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const auto c = tok.find(':');
      if (c == std::string::npos) { continue; }
      try {
        const std::size_t a = static_cast<std::size_t>(std::stoul(tok.substr(0, c)));
        const std::size_t b = static_cast<std::size_t>(std::stoul(tok.substr(c + 1)));
        caution_zones_.emplace_back(a, b);
        RCLCPP_INFO(get_logger(), "要注意区間(遅い相手だけ抜く) idx%zu-%zu", a, b);
      } catch (...) { }
    }
  }
  {
    std::stringstream ss(pass_finish_zone_spec_);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const auto c = tok.find(':');
      if (c == std::string::npos) { continue; }
      try {
        const std::size_t a = static_cast<std::size_t>(std::stoul(tok.substr(0, c)));
        const std::size_t b = static_cast<std::size_t>(std::stoul(tok.substr(c + 1)));
        pass_finish_zones_.emplace_back(a, b);
        RCLCPP_INFO(get_logger(), "抜き切りに使える直線 idx%zu-%zu", a, b);
      } catch (...) { }
    }
  }
  {
    std::stringstream ss(grid_slot_spec_);
    std::string tok;
    while (std::getline(ss, tok, ',')) {
      const auto c = tok.find(':');
      if (c == std::string::npos) { continue; }
      try {
        grid_slots_.emplace_back(std::stod(tok.substr(0, c)),
                                 std::stod(tok.substr(c + 1)));
        RCLCPP_INFO(get_logger(), "グリッド記録 P%zu = (%.2f,%.2f)",
                    grid_slots_.size(), grid_slots_.back().first,
                    grid_slots_.back().second);
      } catch (...) { }
    }
  }

  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  pub_ = create_publisher<Trajectory>("output/trajectory", qos);
  band_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/debug/drivable_band", rclcpp::QoS(1));
  measure_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/planning/debug/v2x_measure", rclcpp::QoS(1));
  layers_pub_ = create_publisher<std_msgs::msg::String>(
      "/planning/debug/v2x_layers", rclcpp::QoS(1));
  status_pub_ = create_publisher<std_msgs::msg::String>("output/status", rclcpp::QoS(1));
  // 診断ログを rosbag へ載せるためのトピック。絶対名にして record.sh から
  // 直接指定できるようにする。QoS は取りこぼしを避けるため深めに取る。
  diag_pub_ = create_publisher<std_msgs::msg::String>("/v2x/diag", rclcpp::QoS(200));
  last_band_pub_ = this->now();
  boost_pub_ = create_publisher<Float32MultiArray>("output/awsim_cmd", rclcpp::QoS(10));
  // 追い越しを試行中かどうかを制御側へ伝える。
  // pure_pursuit は「横にずらした軌道」を受け取るだけで、それが
  // 追い越しのためのものかどうかを知らない。試行中は目標点を近づけて
  // オフセットへ素早く追従させたいので、状態を明示的に渡す。
  overtaking_pub_ = create_publisher<std_msgs::msg::Bool>(
    "output/overtaking", rclcpp::QoS(1));
  // 壁に当たらない舵角の範囲。simple_pure_pursuit が最後にこの範囲へ
  // クランプする。名前は絶対トピックにしてあるので remap は要らない。
  steer_limit_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "/control/wall_guard/steer_limit", rclcpp::QoS(1));
  if (occ_enable_) {
    if (loadOccGrid()) {
      RCLCPP_INFO(get_logger(),
        "壁予測(格子) 読込成功 %s %dx%d res=%.3f origin=(%.3f,%.3f) 占有=%.1f%% "
        "車体=[前%.2f 後%.2f 半幅%.2f] 探索=%.2fm 候補=%d",
        occ_map_yaml_.c_str(), occ_.w, occ_.h, occ_.res, occ_.ox, occ_.oy,
        100.0 * static_cast<double>(occ_.n_occ) /
          std::max<double>(1.0, static_cast<double>(occ_.w) * occ_.h),
        geom_front_, geom_rear_, geom_half_width_,
        occ_clear_search_, occ_steer_bins_);
    } else {
      occ_warned_ = true;
      RCLCPP_WARN(get_logger(),
        "壁予測(格子) 読込失敗 %s -> 占有格子ガードを無効化(CSV 判定のみで動作)",
        occ_map_yaml_.c_str());
    }
  }
  // 制御側が実際に上書きしたかどうかの通知。ログへ併記するためだけに購読する。
  sub_steer_override_ = create_subscription<std_msgs::msg::Bool>(
    "/control/wall_guard/override", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr m) {
      steer_override_active_ = m->data;
    });
  sub_status_ = create_subscription<Float32MultiArray>(
    "input/awsim_status", rclcpp::QoS(10),
    [this](const Float32MultiArray::SharedPtr m) {
      if (m->data.size() > 6) {
        boost_remaining_ = static_cast<int>(m->data[5]);
        is_boosting_ = m->data[6] > 0.5f;
      }
    });
  // レース開始の検知。AWSIM は latch(transient_local) で流すので合わせる。
  // 停止車の余裕を外して測り直すか(devTT5 から取り込み。A/B 用に既定は従来の true)。
  stopped_pad_relax_ = declare_parameter<bool>("stopped_pad_relax", true);
  // 抜き切れないと分かったら追い越しをやめる/始めない(2026-09-18)
  steer_feasible_enable_ = declare_parameter<bool>("steer_feasible_enable", true);
  steer_feasible_ay_use_ = declare_parameter<double>("steer_feasible_ay_use", 0.6);
  steer_feasible_wb_ = declare_parameter<double>("steer_feasible_wheel_base", 1.087);
  steer_feasible_max_steer_ = declare_parameter<double>("steer_feasible_max_steer", 0.31);
  pass_finish_abort_ = declare_parameter<bool>("pass_finish_abort", true);
  pass_finish_no_start_ = declare_parameter<bool>("pass_finish_no_start", true);
  pass_finish_hold_ = declare_parameter<double>("pass_finish_hold", 0.3);
  // 【2026-09-17】AWSIM は state を transient_local で1レース数回しか送らない。
  // volatile だけで購読すると、送出より後に起動したノードは開始を受け取れない。
  // 安全ゲート(volatile と見られる)にも対応するため、両方の QoS で購読する。
  // 同じ通知が2回届くことがあるので onRaceState は冪等にしてある。
  sub_state_ = create_subscription<std_msgs::msg::String>(
    "/awsim/state",
    rclcpp::QoS(10),
    [this](const std_msgs::msg::String::SharedPtr m) { onRaceState(m->data); });
  sub_state_latched_ = create_subscription<std_msgs::msg::String>(
    "/awsim/state",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr m) { onRaceState(m->data); });
  sub_traj_ = create_subscription<Trajectory>(
    "input/trajectory", qos, [this](const Trajectory::SharedPtr m) { traj_ = m; });
  sub_odom_ = create_subscription<Odometry>(
    "input/kinematics", qos, [this](const Odometry::SharedPtr m) { odom_ = m; });
  sub_v2x_ = create_subscription<V2XVehiclePositionArray>(
    "input/v2x", rclcpp::QoS(10),
    std::bind(&V2XOvertaker::onV2X, this, std::placeholders::_1));

  timer_ = create_wall_timer(
    std::chrono::milliseconds(50), std::bind(&V2XOvertaker::onTimer, this));
}

// 曲率半径の「前後の最小値」を作る。点ごとの値は外れ値を含むため、
// 壁の余裕を決めるのにそのまま使うと危険(ヘアピンの中で1点だけ大きく出る)。
void V2XOvertaker::buildRadiusMin()
{
  const std::size_t n = corridor_.radius.size();
  radius_min_.assign(n, 0.0);
  const int win = 4;   // 前後4点 = 約 ±5.5m
  for (std::size_t i = 0; i < n; ++i) {
    double m = corridor_.radius[i];
    for (int k = -win; k <= win; ++k) {
      const std::size_t j = (i + n + static_cast<std::size_t>(k + static_cast<int>(n))) % n;
      m = std::min(m, corridor_.radius[j]);
    }
    radius_min_[i] = m;
  }
}


bool V2XOvertaker::loadCorridor(const std::string & path)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    return false;
  }
  std::string line;
  std::getline(f, line);  // header
  while (std::getline(f, line)) {
    if (line.empty()) {
      continue;
    }
    std::stringstream ss(line);
    std::string a, b, c;
    if (!std::getline(ss, a, ',') || !std::getline(ss, b, ',') || !std::getline(ss, c, ',')) {
      continue;
    }
    corridor_.lo.push_back(std::stod(b));
    corridor_.hi.push_back(std::stod(c));
    std::string d, e;
    std::getline(ss, d, ',');            // radius
    double r = 1e9;
    try { r = std::stod(d); } catch (...) { r = 1e9; }
    corridor_.radius.push_back(r);
    if (std::getline(ss, e, ',')) {
      corridor_.pass_ok.push_back(std::stoi(e) != 0);
    } else {
      corridor_.pass_ok.push_back(true);
    }
  }
  RCLCPP_INFO(get_logger(), "corridor 読み込み %zu 点", corridor_.lo.size());
  buildRadiusMin();
  return !corridor_.lo.empty();
}

void V2XOvertaker::onRaceState(const std::string & state)
{
  const auto reset_launch = [this]() {
    race_start_time_ = this->now().seconds();
    launch_since_ = race_start_time_;
    launch_motion_since_ = -1.0;
    launch_wait_log_at_ = -1.0;
    // 初期位置投入(teleport)の差分速度を物理発進と誤認しない。Start前に
    // 推定器へ残った速度だけを消し、位置と時刻は次標本の差分用に保つ。
    for (auto & kv : others_) {
      kv.second.vx = 0.0;
      kv.second.vy = 0.0;
    }
  };
  // 公式 autostart_orchestrator と同じく Grounded / Ready / Start のどれでも開始とみなす
  // (autostart_orchestrator.param.yaml の start_on_vehicle_state)。
  // AWSIM の state は transient_local・履歴1件で Spawned→Grounded→Ready→Start と進むので、
  // Grounded の送出より後に起動すると最後の Ready しか受け取れない。Ready を見ないと
  // 開始を検知できず、回避・追い越しが無効のまま停止車へ突っ込む(2026-09-18 gate2 で再現)。
  const bool start_state = (state == "Start" || state == "Grounded" || state == "Ready");
  if (!race_started_ && start_state) {
    race_started_ = true;
    race_start_by_grounded_ = (state != "Start");
    reset_launch();
    RCLCPP_INFO(get_logger(), "レース開始を検知(%s)。回避と追い越しを有効化する", state.c_str());
    return;
  }
  // AWSIM では Grounded/Ready の後に Start が来る。合図からの時間窓
  // (launch_p1_pass_sec など)を Grounded から数えると発進前に切れるので、
  // Start が来たら基準を Start に付け替える。まだ動いていないときだけ。
  if (race_started_ && race_start_by_grounded_ && state == "Start") {
    race_start_by_grounded_ = false;
    if (launch_motion_since_ < 0.0) {
      reset_launch();
      RCLCPP_INFO(get_logger(), "Start を検知。合図の時刻を Grounded/Ready から Start に付け替える");
    }
  }
}

void V2XOvertaker::onV2X(const V2XVehiclePositionArray::SharedPtr msg)
{
  const rclcpp::Time now = msg->header.stamp;
  for (const auto & v : msg->vehicles) {
    auto & st = others_[v.vehicle_id];
    if (st.valid) {
      const double dt = (now - st.stamp).seconds();
      if (dt > 1e-3 && dt < 1.0) {
        // 位置しか来ないので差分から速度を推定する。1次の低域通過で暴れを抑える。
        const double a = 0.4;
        st.vx = (1 - a) * st.vx + a * (v.position.x - st.x) / dt;
        st.vy = (1 - a) * st.vy + a * (v.position.y - st.y) / dt;

        // --- 走行データを溜める ---
        // 相手が遅いかどうかを、瞬間の速度差ではなく実績で判断するため。
        if (!line_x_.empty()) {
          const double sp = std::hypot(st.vx, st.vy);
          if (sp > 0.5 && sp < 30.0) {          // 明らかな外れ値は捨てる
            const std::size_t np = line_x_.size();
            std::size_t bi = 0; double bd = 1e18;
            for (std::size_t i = 0; i < np; ++i) {
              const double d = (line_x_[i] - st.x) * (line_x_[i] - st.x) +
                               (line_y_[i] - st.y) * (line_y_[i] - st.y);
              if (d < bd) { bd = d; bi = i; }
            }
            const int sec = static_cast<int>(bi * OtherState::kSections / np);
            st.sec_sum[sec] += sp; st.sec_cnt[sec] += 1;
            st.speed_sum += sp;    st.speed_cnt += 1;
            // 区間0へ戻ったら1周とみなす
            if (st.last_sec >= OtherState::kSections - 2 && sec <= 1) {
              for (int k = 0; k < OtherState::kSections; ++k) {
                st.prev_sec_sum[k] = st.sec_sum[k];
                st.prev_sec_cnt[k] = st.sec_cnt[k];
                st.sec_sum[k] = 0.0; st.sec_cnt[k] = 0;
              }
              st.prev_sec_valid = true;
              const double t = now.seconds();
              if (st.lap_start_time > 0.0) {
                st.last_lap_time = t - st.lap_start_time;
                if (st.best_lap_time <= 0.0 || st.last_lap_time < st.best_lap_time) {
                  st.best_lap_time = st.last_lap_time;
                }
                st.laps += 1;
              }
              st.lap_start_time = t;
            }
            st.last_sec = sec;
          }
        }
      }
    }
    st.x = v.position.x;
    st.y = v.position.y;
    st.stamp = now;
    st.valid = true;
  }
}

// 軌道上で最も近い点の index
size_t V2XOvertaker::nearest(const Trajectory & t, double x, double y)
{
  size_t best = 0;
  double bd = std::numeric_limits<double>::max();
  for (size_t i = 0; i < t.points.size(); ++i) {
    const double dx = t.points[i].pose.position.x - x;
    const double dy = t.points[i].pose.position.y - y;
    const double d = dx * dx + dy * dy;
    if (d < bd) {
      bd = d;
      best = i;
    }
  }
  return best;
}

// index i における進行方向左向きの単位法線
void V2XOvertaker::normalAt(const Trajectory & t, size_t i, double & nx, double & ny)
{
  const size_t n = t.points.size();
  const auto & a = t.points[(i + n - 1) % n].pose.position;
  const auto & b = t.points[(i + 1) % n].pose.position;
  double tx = b.x - a.x, ty = b.y - a.y;
  const double len = std::hypot(tx, ty);
  if (len < 1e-9) {
    nx = 0.0;
    ny = 0.0;
    return;
  }
  tx /= len;
  ty /= len;
  nx = -ty;
  ny = tx;
}

double V2XOvertaker::safetyAt(size_t i) const
{
  const size_t n = corridor_.lo.size();
  if (i >= n) { return corridor_safety_; }
  double r = 0.0;
  if (radius_min_.size() == n) { r = radius_min_[i]; }
  else if (corridor_.radius.size() == n) { r = corridor_.radius[i]; }
  const double t = std::clamp(
      (r - pass_margin_r_curve_) /
          std::max(pass_margin_r_straight_ - pass_margin_r_curve_, 1.0), 0.0, 1.0);
  return corridor_safety_ + t * (pass_margin_min_ - corridor_safety_);
}

void V2XOvertaker::sideRoomMap(const OtherState & o, const Trajectory & in, size_t n,
                 size_t from, double stretch, double olat_now,
                 double & run_left, double & run_right, int & known,
                 double * room_left, double * room_right, int zone_clip,
                 bool * zone_clipped) const
{
  run_left = 0.0;
  run_right = 0.0;
  known = 0;
  double sum_l = 0.0, sum_r = 0.0;
  double min_l = 0.0, min_r = 0.0;
  int cnt_room = 0;
  if (room_left) { *room_left = 0.0; }
  if (room_right) { *room_right = 0.0; }
  if (corridor_.lo.size() != n || corridor_.hi.size() != n) { return; }
  double acc = 0.0, cur_l = 0.0, cur_r = 0.0;
  bool broke_l = false, broke_r = false;
  for (size_t k = 1; k < n; ++k) {
    const size_t a = (from + k - 1) % n, b = (from + k) % n;
    const double step =
      std::hypot(in.points[b].pose.position.x - in.points[a].pose.position.x,
                 in.points[b].pose.position.y - in.points[a].pose.position.y);
    acc += step;
    if (acc > stretch) { break; }
    if (zone_clip >= 0 && sidePickZoneIndex(b) != zone_clip) {
      if (zone_clipped) { *zone_clipped = true; }
      break;
    }
    double ol;
    if (lat_pred_mode_ == 0) {
      ol = side_model_fill_
             ? oppLatAt(o, b, n)
             : o.laneLat(static_cast<int>(b * OtherState::kLatBins / n));
      if (ol > 1e8) { ol = olat_now; } else { known++; }
    } else {
      ol = (lat_pred_mode_ == 2)
             ? olat_now * std::exp(-acc / std::max(band_lat_tau_, 1.0))
             : olat_now;
      if (std::isfinite(olat_now) && std::abs(olat_now) < 6.0) { known++; }
    }
    double sf = safetyAt(b);
    if (corridor_.pass_ok.size() == n && corridor_.pass_ok[b]) {
      sf = std::min(sf, corridor_safety_zone_);
    }
    const double hi = corridor_.hi[b] - sf;
    const double lo = corridor_.lo[b] + sf;
    // その地点で寄れる限界まで寄ったときの、相手との横間隔
    const double sep_l = std::min(hi, ol + pass_gap_) - ol;
    const double sep_r = ol - std::max(lo, ol - pass_gap_);
    const double free_l = std::max(hi - ol, 0.0);
    const double free_r = std::max(ol - lo, 0.0);
    sum_l += free_l;
    sum_r += free_r;
    if (cnt_room == 0) { min_l = free_l; min_r = free_r; }
    else { min_l = std::min(min_l, free_l); min_r = std::min(min_r, free_r); }
    cnt_room++;
    if (!broke_l) {
      if (sep_l >= min_pass_sep_) { cur_l += step; } else { broke_l = true; }
    }
    if (!broke_r) {
      if (sep_r >= min_pass_sep_) { cur_r += step; } else { broke_r = true; }
    }
    run_left = cur_l;
    run_right = cur_r;
    // 連続区間の判定は途中で切り上げてよいが、空き幅の平均は
    // 区間の最後まで見ないと「どちらが空いているか」を取り違える。
    if (broke_l && broke_r && cnt_room * 1.0 > 0 && acc >= stretch) { break; }
  }
  if (cnt_room > 0) {
    // side_room_use_min_ のときは「区間の最小」を返す。平均だと途中で
    // 左右が入れ替わる区間で側が反転する(上のコメント参照)。
    if (room_left)  { *room_left  = side_room_use_min_ ? min_l : sum_l / cnt_room; }
    if (room_right) { *room_right = side_room_use_min_ ? min_r : sum_r / cnt_room; }
  }
}


// ===================================================================
// 20Hz の本体
// ===================================================================

void V2XOvertaker::onTimer()
{
  if (!traj_ || traj_->points.size() < 3 || !odom_) {
    return;
  }
  const Trajectory & in = *traj_;
  const size_t n = in.points.size();

  // 各点までの累積距離（周回長の計算と前後判定に使う）
  std::vector<double> s(n, 0.0);
  double total = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const auto & p = in.points[i].pose.position;
    const auto & q = in.points[(i + 1) % n].pose.position;
    const double d = std::hypot(q.x - p.x, q.y - p.y);
    if (i + 1 < n) {
      s[i + 1] = s[i] + d;
    }
    total += d;
  }

  const double ex = odom_->pose.pose.position.x;
  const double ey = odom_->pose.pose.position.y;
  const double ev = odom_->twist.twist.linear.x;
  const size_t ei = nearest(in, ex, ey);

  const rclcpp::Time now = this->now();

  // その周期の観測。以降の層は読むだけで書き換えない。
  const Frame f{in, s, n, total, ex, ey, ev, ei, now};
  // その周期の指令と中間結果。以降の層がこれを積み上げていく。
  PlanCtx c;
  c.best_gap = detect_range_;

  if (!race_started_) {
    pub_->publish(in);
    publishStatus(f, c);
    return;
  }

  // /awsim/state=Start は物理的な発進許可より先に届く。信号時刻から発進用の
  // 時間窓を数えると、カウントダウン中にグリッド保持や追従解除が失効する。
  // 自車または他車の最初の実移動を一度だけラッチし、制御時間の基準にする。
  if (launch_motion_since_ < 0.0) {
    double fastest_other = 0.0;
    for (const auto & kv : others_) {
      if (!kv.second.valid || (now - kv.second.stamp).seconds() > v2x_timeout_) { continue; }
      fastest_other = std::max(fastest_other, std::hypot(kv.second.vx, kv.second.vy));
    }
    if (v2x_overtaker::launchMotionObserved(ev, fastest_other, 0.30)) {
      launch_motion_since_ = now.seconds();
      RCLCPP_INFO(get_logger(),
        "物理発進を検知 信号から%.1fs 自車=%.1fkm/h 最速他車=%.1fkm/h。発進制御タイマー開始",
        launch_since_ >= 0.0 ? now.seconds() - launch_since_ : -1.0,
        std::abs(ev) * 3.6, fastest_other * 3.6);
    }
    if (launch_motion_since_ < 0.0) {
      if (launch_wait_log_at_ < 0.0 || now.seconds() - launch_wait_log_at_ >= 1.0) {
        launch_wait_log_at_ = now.seconds();
        RCLCPP_INFO(get_logger(),
          "物理発進待ち 信号から%.1fs 自車=%.1fkm/h 最速他車=%.1fkm/h。"
          "回避・追越・膠着状態は更新しない",
          launch_since_ >= 0.0 ? now.seconds() - launch_since_ : -1.0,
          std::abs(ev) * 3.6, fastest_other * 3.6);
      }
      // State=Start後にもAWSIMのカウントダウンがある。その間に追越試行・
      // 停止車ラッチ・deadlockを作ると、物理発進した瞬間に古い状態が発火する。
      // 観測上いずれかの車が動くまで、入力軌道を一切加工せず渡す。
      pub_->publish(in);
      publishStatus(f, c);
      return;
    }
  }

  // 直線を通しきる状態か。層をまたいで同じ値を使うので先に求める。
  straight_pass_now_ = straightPassNow(f);

  logDrivingStats(f);
  estimateRank(f);
  trackRanks(f);
  evaluateZone(f, c);
  checkPressedFromBehind(f, c);
  logStopCause(f);
  findFrontCar(f);
  c.lat_fallback = my_lat_for_target_;

  updatePenalty(f);
  learnOpponentLine(f);
  // グリッド番号の確定 -> 抜きどころの計画 -> 録画の書き出し。
  // どれも観測だけを使う(指令は触らない)。
  assignStartSlots(f);
  updateLaunchPass(f);
  planPassSpot(f);
  dumpTrace(f);

  planOvertake(f, c);

  updateOvertakeState(f, c);

  avoidStoppedCars(f, c);

  if (c.stop_avoid_active && attempt_active_ && !ovPassingTarget(attempt_target_)) {
    const double elapsed = now.seconds() - attempt_start_;
    stop_avoid_target_ = attempt_target_;
    // 停止車の速度推定はV2Xの更新ごとに一瞬だけ動いたように見えることがある。
    // その揺れで即時に再試行すると、同じ横移動を始めては中断するため、
    // 2秒だけ同じ相手への追越しを休止する。停止車が消える/相手が変わる場合は
    // 対象外で、通常の追越しは遅らせない。
    stop_avoid_retry_until_ = now.seconds() + 2.0;
    attempt_active_ = false;
    attempt_ng_++;
    attempt_fail_since_ = -1.0;
    diagLog("追越試行", "追越試行 安全中断 target=%s 所要=%.1fs 理由=停止車回避 rank=%d",
                attempt_target_.c_str(), elapsed, rank_);
    logAttemptFunnel(f, "中断", elapsed);
  }

  avoidCollision(f, c);

  repulseFromNearCars(f, c);

  holdStartLane(f, c);


  recordAttempt(f, c);
  applyAvoidance(f, c);


  // 発進フェーズだけ横目標を最終決定する。壁だけは必ず後段に残す。
  holdGridLane(f, c);

  avoidWall(f, c);
  // 追突防止の速度上限も調停器でmin合成する。
  preventRearEnd(f, c);

  holdSideAlongside(f, c);

  if (yaw_margin_enable_) {
    const auto & q = odom_->pose.pose.orientation;
    const double myyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const size_t a = (f.ei + f.n - 1) % f.n, b = (f.ei + 1) % f.n;
    const double tth = std::atan2(
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y,
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x);
    double e = myyaw - tth;
    while (e > M_PI) { e -= 2.0 * M_PI; }
    while (e < -M_PI) { e += 2.0 * M_PI; }
    const double ae = std::abs(e);
    double add_lo = 0.0, add_hi = 0.0, ext_l = 0.0, ext_r = 0.0;
    if (body_margin_asym_) {
      // 実際の車体の四隅から左右別に求める(bodyExtent のコメント参照)。
      // コーナーの振り出しも見る(curve_sign_/曲率はこの周期の値)。
      bodyExtent(e, ext_l, ext_r, curveRadiusAt(f.ei),
                 (curve_sign_ > 0.0) ? +1 : ((curve_sign_ < 0.0) ? -1 : 0));
      // コリドアは「車体が経路に沿った姿勢」を前提にした幅なので、
      // 差分は「姿勢が揃っているときの半幅」から測る。
      add_hi = std::max(ext_l - geom_half_width_, 0.0);   // 左(lat_hi)側
      add_lo = std::max(ext_r - geom_half_width_, 0.0);   // 右(lat_lo)側
    } else {
      const double need = kCarWidth * 0.5 * std::cos(ae) +
                          yaw_margin_half_len_ * std::sin(ae);
      add_lo = add_hi = std::max(need - kCarWidth * 0.5, 0.0);
      ext_l = ext_r = need;
    }
    if (add_lo > 0.02 || add_hi > 0.02) {
      const double before = c.latWant();
      const double base_lo = c.lat_lo;
      const double base_hi = c.lat_hi;
      const auto shrunk = v2x_overtaker::shrinkLateralInterval(
        base_lo, base_hi, add_lo, add_hi);
      if (base_lo <= base_hi && !shrunk.feasible) {
        // 姿勢の張り出しが帯全体より広いと、停止したままでは姿勢も
        // 直らず、永久停止になる。元の壁帯は壊さず中央へ微速で向け、
        // pure pursuit が車体を経路方向へ戻せる自由度を残す。これは壁/車両
        // の硬制約同士が空の場合とは分け、後者は引き続き完全停止する。
        const double center = 0.5 * (base_lo + base_hi);
        c.requestLat(center, PlanCtx::LatPrio::kCollision, "姿勢復元");
        c.requestCap(1.0, "姿勢復元");
        if ((now - last_yaw_margin_log_).seconds() > 0.5) {
          last_yaw_margin_log_ = now;
          diagWarn("姿勢余裕",
            "姿勢余裕を満たす横位置なし 方位差=%+.1fdeg 元帯=[%.2f,%.2f] "
            "必要縮小=(右%.2f 左%.2f) 中央%.2fへ微速復元",
            e * 180.0 / M_PI, base_lo, base_hi, add_lo, add_hi, center);
        }
      } else if (shrunk.feasible) {
        c.boundLat(shrunk.lo, shrunk.hi, "姿勢ぶんの余裕");
      }
      const double after = c.latWant();
      if (std::abs(before - after) > 0.05 &&
          (now - last_yaw_margin_log_).seconds() > 2.0)
      {
        last_yaw_margin_log_ = now;
        diagLog("姿勢余裕",
                "姿勢余裕 方位差=%+.1fdeg 張り出し 左%.2f 右%.2f "
                "(狭める 左%.2f 右%.2f) 横目標 %.2f -> %.2f",
                e * 180.0 / M_PI, ext_l, ext_r, add_hi, add_lo, before, after);
      }
    }
  }

  // 壁衝突の予測監視。すべての意図と制約が出そろった後、確定の直前に
  // 評価する(latWant() が最終値と一致するのはこの位置だけ)。
  // 出力は boundLat / requestCap のみで、意図は出さない。
  wallGuard(f, c);

  const double ot_look = ot_lane_guard_look_ +
                         std::max(my_speed_for_gap_, 0.0) * ot_lane_guard_time_;
  const bool avoid_wins =
    static_cast<int>(c.lat_intent.prio) >= static_cast<int>(PlanCtx::LatPrio::kStoppedCar);
  if (avoid_wins && ot_lane_guard_ && !laneGuardSuppressed(f) && !ot_lane_zones_.empty() &&
      otLaneAhead(f.ei, ot_look) && my_speed_for_gap_ * 3.6 < ot_lane_min_kmh_ &&
      (now - last_ot_lane_log_).seconds() > 1.0)
  {
    last_ot_lane_log_ = now;
    diagLog("追越レーン",
            "追越レーン 回避が優先するのでガードを掛けない 意図=%s(%.2fm) idx=%zu",
            c.lat_intent.why ? c.lat_intent.why : "?", c.lat_intent.v, f.ei);
  }
  const bool will_be_fast_enough =
    ot_lane_guard_predict_ &&
    otLaneApproach(f.ei, my_speed_for_gap_, rankSpeedCap());
  if (!avoid_wins && ot_lane_guard_ && !laneGuardSuppressed(f) && !ot_lane_zones_.empty() &&
      otLaneAhead(f.ei, ot_look) &&
      my_speed_for_gap_ * 3.6 < ot_lane_min_kmh_ &&
      !will_be_fast_enough &&
      otLaneHot(f))
  {
    const double before = c.latWant();
    // 固定値 -ot_lane_guard_lat(2.0m)ではなく、。
    const double geom = ot_lane_guard_geom_ ? otLaneNoTouchAhead(f.ei, ot_look) : -1e9;
    const double bound = (geom > -1e8) ? geom : -ot_lane_guard_lat_;
    c.boundLat(bound, 1e9, "追越レーン(低速で入らない)");
    const double after = c.latWant();
    if (std::abs(before - after) > 0.05 &&
        (now - last_ot_lane_log_).seconds() > 1.0)
    {
      last_ot_lane_log_ = now;
      diagLog("追越レーン",
              "追越レーン 低速で入らない 自車=%.1fkm/h(要%.0f) idx=%zu "
              "触れない右限=%.2f(固定値なら%.2f) 横目標 %.2f -> %.2f",
              my_speed_for_gap_ * 3.6, ot_lane_min_kmh_, f.ei,
              bound, -ot_lane_guard_lat_, before, after);
    }
  }

  if (ot_lane_aim_ && ot_lane_enable_ && !ot_lane_lat_.empty() &&
      ot_lane_side_ && will_be_fast_enough && !ot_lane_zones_.empty())
  {
    const std::size_t nn = f.n;
    std::size_t aim_idx = f.ei;
    if (nn > 1) {
      double acc = 0.0;
      for (std::size_t k = 0; k < nn; ++k) {
        const std::size_t a2 = (f.ei + k) % nn, b2 = (f.ei + k + 1) % nn;
        acc += std::hypot(
          f.in.points[b2].pose.position.x - f.in.points[a2].pose.position.x,
          f.in.points[b2].pose.position.y - f.in.points[a2].pose.position.y);
        if (acc >= ot_lane_aim_look_) { aim_idx = b2; break; }
      }
    }
    // 狙い点が「まだレーンに入っていない」ときは、。
    double wlo = 0.0, whi = 0.0;
    bool has_win = otLaneAimWindow(aim_idx, wlo, whi);
    if (!has_win && ot_lane_aim_seek_) {
      for (std::size_t k = 1; k <= ot_lane_aim_seek_max_; ++k) {
        const std::size_t j = (aim_idx + k) % nn;
        if (otLaneAimWindow(j, wlo, whi)) { aim_idx = j; has_win = true; break; }
      }
    }
    if (has_win) {
      // 右側(負)が壁側なので、内側の端は whi。そこから少しだけ中へ入れる。
      const double aim = whi - std::min(ot_lane_aim_inset_, std::max(whi - wlo, 0.0));
      c.requestLat(aim, PlanCtx::LatPrio::kOvertake, "追越レーンへ入る");
      if ((now - last_lane_aim_log_).seconds() > 1.0) {
        last_lane_aim_log_ = now;
        diagLog("レーンへ寄せる",
          "レーンへ寄せる idx=%zu 狙いidx=%zu 窓[%.2f,%.2f] 狙い%.2f "
          "自車実横%.2f 自車%.1fkm/h",
          f.ei, aim_idx, wlo, whi, aim, my_lat_for_target_,
          std::abs(f.ev) * 3.6);
      }
    }
  }

  // --- 横位置の確定。ここまでに積まれた意図と制約を調停する。
  if (stopped_bound_enable_ && odom_ && my_prog_init_) {
    const double occ = geom_half_width_ * 2.0 + stoppedPad();
    const double me = my_lat_for_target_;
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || !o.prog_init) { continue; }
      if ((f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
      if (std::hypot(o.vx, o.vy) > stopped_speed_) { continue; }   // 動いている
      double d = o.prog - my_prog_;
      if (d < 0.0) { d += f.total; }
      if (d <= 0.0 || d > stopped_bound_range_) { continue; }      // 前方の一定範囲だけ
      const std::size_t oi2 = nearest(f.in, o.x, o.y);
      double nx2, ny2; normalAt(f.in, oi2, nx2, ny2);
      const auto & lp2 = f.in.points[oi2].pose.position;
      const double olat2 = (o.x - lp2.x) * nx2 + (o.y - lp2.y) * ny2;
      const double before = c.latWant();
      if (me >= olat2) {
        c.boundLat(olat2 + occ, c.lat_hi, "停止車の側へ寄らない");
      } else {
        c.boundLat(c.lat_lo, olat2 - occ, "停止車の側へ寄らない");
      }
      const double after = c.latWant();
      if (std::abs(before - after) > 0.05 &&
          (f.now - last_stopped_bound_log_).seconds() > 1.0) {
        last_stopped_bound_log_ = f.now;
        diagLog("停止車の側へ寄らない",
          "停止車の側へ寄らない 相手=%s まで%.1fm 相手横%.2f 自車横%.2f "
          "占有%.2f 横目標 %.2f -> %.2f 帯[%.2f,%.2f]",
          kv.first.c_str(), d, olat2, me, occ, before, after, c.lat_lo, c.lat_hi);
      }
    }
  }
  bodyGuardByMeasured(f, c);
  c.applyLatDecision();
  if (lat_audit_ && inPassFinishZone(f.ei) && std::abs(f.ev) > 4.0 &&
      (f.now - last_lat_audit_log_).seconds() > lat_audit_sec_)
  {
    last_lat_audit_log_ = f.now;
    char buf[1400];
    int off = std::snprintf(buf, sizeof(buf),
      "横監査 idx=%zu 自車%.1fkm/h 側=%s(決定元=%s) 相手=%s 状態=%s 確定=%.2f 自車実横=%.2f "
      "自車座標=(%.2f,%.2f) "
      "帯=[%.2f,%.2f] 勝者=%s(%.2f) "
      "側の入力[窓の主=%s 相手座標=(%.2f,%.2f) 相手idx=%d 相手横=%.2f(実使用%.2f) 抜き切り%.0fm 窓の空き 左%.2f 右%.2f 探索+%.0fm 窓=%.1f..%.1fm 標本%d 縮尺%.2f 標本域%.1fm 窓の側=%d 必要間隔=%.2f 門[現在地=%d 抜切地点=%d] 計画[開始%.1fm 狙い%.2f 通せる側=%d 最早左%.1f 最早右%.1f 却下左(空%d帯%d寄%d曲%d) 却下右(空%d帯%d寄%d曲%d) 相手観測%.2f秒前 新鮮%d 幾何既定%d]] |",
      f.ei, std::abs(f.ev) * 3.6, side_sign_ > 0.0 ? "左" : "右",
      side_src_ ? side_src_ : "?",
      c.blocker.empty() ? "-" : c.blocker.c_str(), ovStateName(),
      c.target_offset, my_lat_for_target_, f.ex, f.ey, c.lat_lo, c.lat_hi,
      c.lat_intent.why ? c.lat_intent.why : "?", c.lat_intent.v,
      audit_room_target_.empty() ? "なし" : audit_room_target_.c_str(),
      audit_opp_x_, audit_opp_y_, audit_opp_idx_,
      audit_olat_, audit_used_olat_, audit_pass_len_, audit_room_l_, audit_room_r_,
      audit_found_at_, audit_win_from_, audit_win_to_, audit_win_used_,
      audit_win_scale_, audit_win_span_, audit_cs_, audit_need_here_,
      audit_cfit_now_ ? 1 : 0, audit_cfit_ahead_ ? 1 : 0,
      audit_plan_start_, audit_plan_y_, audit_plan_feas_,
      audit_early_l_, audit_early_r_,
      audit_rej_l_[0], audit_rej_l_[1], audit_rej_l_[2], audit_rej_l_[3],
      audit_rej_r_[0], audit_rej_r_[1], audit_rej_r_[2], audit_rej_r_[3],
      audit_opp_age_, audit_cs_fresh_, audit_geom_);
    // 次の周期で completionSide が呼ばれなければ、上の値は「前周期の残り」。
    audit_cs_fresh_ = 0;
    for (int i = 0; i < c.lat_trace_n && off > 0 && off < static_cast<int>(sizeof(buf)) - 60; ++i) {
      const auto & t = c.lat_trace[i];
      if (t.kind == 'R' || t.kind == 'r') {
        off += std::snprintf(buf + off, sizeof(buf) - off, " %s要求%s=%.2f",
          t.won ? "◎" : "×", t.why, t.a);
      } else {
        off += std::snprintf(buf + off, sizeof(buf) - off, " %s制約%s=[%.2f,%.2f]",
          t.won ? "◎" : "×", t.why, t.a, t.b);
      }
    }
    diagLog("横監査", "%s", buf);
  }
  if (!c.lat_feasible && (now - last_lat_conflict_log_).seconds() > 0.3) {
    last_lat_conflict_log_ = now;
    RCLCPP_ERROR(
      get_logger(),
      "横制約空集合: 範囲=[%.2f,%.2f] 下限=%s 上限=%s "
      "実測横=%.2f 意図=%s(%.2f) 中点へは進まず停止",
      c.lat_lo, c.lat_hi, c.lat_lo_why, c.lat_hi_why,
      c.lat_fallback, c.lat_intent.why, c.lat_intent.v);
  }
  if ((now - last_lat_log_).seconds() > 2.0) {
    last_lat_log_ = now;
    diagLog("横位置", "横位置 %.2fm 意図=%s(%.2fm) 範囲=[%.2f,%.2f] 縛り=%s",
                c.target_offset, c.lat_intent.why, c.lat_intent.v,
                c.lat_lo, c.lat_hi, c.lat_bound_why);
  }

  // 空集合を壁側で解いたか。次の周期の preventRearEnd がこれを読む
  // (この層より後で立つ値なので、同一周期では使えない)。
  lat_relaxed_prev_ = c.lat_relaxed;

  applyOffsetRateLimit(c);
  publishTrajectory(f, c);

  manageBoost(f, c);

  publishStatus(f, c);

  logBlocker(f, c);
  logContactContext(f, c);
}


// ===================================================================
// 観測と記録の層
// ===================================================================

// 溜めた走行データの要約を定期的に出す。
void V2XOvertaker::logDrivingStats(const Frame & f)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  const rclcpp::Time stats_now = this->now();
  if ((stats_now - last_stats_log_).seconds() > kStatsLogSec) {
    last_stats_log_ = stats_now;
    for (const auto & kv : others_) {
      const auto & o = kv.second;
      if (o.meanSpeed() < 0.0) { continue; }
      RCLCPP_INFO(get_logger(),
                  "相手 %s: 平均%.1fkm/h 周回%d 直近ラップ%.1fs 最速%.1fs "
                  "(自車 平均%.1fkm/h)",
                  kv.first.c_str(), o.meanSpeed() * 3.6, o.laps,
                  o.last_lap_time, o.best_lap_time,
                  my_speed_sum_ > 0 && my_speed_cnt_ > 0
                    ? my_speed_sum_ / my_speed_cnt_ * 3.6 : 0.0);
    }
  }
  // 自車の平均も同じ条件で溜める(比較の土台をそろえる)
  {
    const double sp = std::abs(odom_->twist.twist.linear.x);
    if (sp > 0.5 && sp < 30.0) {
      my_speed_sum_ += sp; my_speed_cnt_ += 1;
      // 区間ごとにも溜める。**相手側(1170行付近)と同じ式**:
      // 走行ラインの最近傍点を求め、その添字を kSections で割って区間にする。
      if (!line_x_.empty()) {
        const double mx = odom_->pose.pose.position.x;
        const double my = odom_->pose.pose.position.y;
        const std::size_t np = line_x_.size();
        std::size_t bi = 0; double bd = 1e18;
        for (std::size_t i = 0; i < np; ++i) {
          const double d = (line_x_[i] - mx) * (line_x_[i] - mx) +
                           (line_y_[i] - my) * (line_y_[i] - my);
          if (d < bd) { bd = d; bi = i; }
        }
        const int sec = static_cast<int>(bi * OtherState::kSections / np);
        if (sec >= 0 && sec < OtherState::kSections) {
          // 相手側とまったく同じ形で、周回をまたいだら控えて捨てる。
          if (my_last_sec_ >= OtherState::kSections - 2 && sec <= 1) {
            for (int k = 0; k < OtherState::kSections; ++k) {
              my_prev_sec_sum_[k] = my_sec_sum_[k];
              my_prev_sec_cnt_[k] = my_sec_cnt_[k];
              my_sec_sum_[k] = 0.0; my_sec_cnt_[k] = 0;
            }
            my_prev_sec_valid_ = true;
          }
          my_last_sec_ = sec;
          my_sec_sum_[sec] += sp; my_sec_cnt_[sec] += 1;
        }
      }
    }
  }

  // 相手の走行データ集計用に経路を控える(最初の1回だけ)
  if (line_x_.size() != n) {
    line_x_.clear(); line_y_.clear();
    line_x_.reserve(n); line_y_.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      line_x_.push_back(in.points[i].pose.position.x);
      line_y_.push_back(in.points[i].pose.position.y);
    }
  }

  // 周回数を数える(経路上の位置が一周ぶん戻ったら1周)。
  // ブーストを「終盤まで温存する」判断に使うので、早い段階で更新する。
  if (prev_ei_set_ && prev_ei_ > n * 3 / 4 && ei < n / 4) { ++lap_; }
  prev_ei_ = ei;
  prev_ei_set_ = true;

  // これから直線に入るか。直線の手前でブーストを撃つための判定。
  // 横へ出てから撃つと直線を半分使ってから加速し始めることになり、
  // 直線の終わり(コーナー入口)で並んだまま突っ込んで失敗する。
  {
    straight_ahead_ = true;
    double skipped = 0.0;
    size_t k0 = 0;
    for (; k0 + 2 < n && skipped < free_boost_skip_; ++k0) {
      const auto & a = in.points[(ei + k0) % n].pose.position;
      const auto & b = in.points[(ei + k0 + 1) % n].pose.position;
      skipped += std::hypot(b.x - a.x, b.y - a.y);
    }
    double travelled = 0.0;
    for (size_t k = k0; k + 4 < n && travelled < 25.0; ++k) {
      const auto & a = in.points[(ei + k) % n].pose.position;
      const auto & b = in.points[(ei + k + 2) % n].pose.position;
      const auto & p3 = in.points[(ei + k + 4) % n].pose.position;
      travelled += std::hypot(b.x - a.x, b.y - a.y);
      const double ab = std::hypot(b.x - a.x, b.y - a.y);
      const double bc = std::hypot(p3.x - b.x, p3.y - b.y);
      const double ca = std::hypot(a.x - p3.x, a.y - p3.y);
      const double cross = (b.x - a.x) * (p3.y - a.y) - (b.y - a.y) * (p3.x - a.x);
      if (std::abs(cross) < 1e-9) { continue; }
      if (ab * bc * ca / (2.0 * std::abs(cross)) < free_boost_straight_) {
        straight_ahead_ = false;
        break;
      }
    }
  }

}

// 現在の順位を推定する。
void V2XOvertaker::estimateRank(const Frame & f)
{
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const double total = f.total;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // --- 現在の順位を推定する
  // AWSIM は順位をトピックに出さないので、V2X の他車位置をレースラインに投影し、
  // 周回をまたいだ回数を数えて累積進行度を作り、自車と比較して順位を出す。
  {
    double my_s = s[ei];
    if (!my_prog_init_) {
      my_prog_ = my_s;
      my_last_s_ = my_s;
      my_prog_init_ = true;
    } else {
      double d = my_s - my_last_s_;
      if (d < -total * 0.5) {
        d += total;           // 周回をまたいだ
      } else if (d > total * 0.5) {
        d -= total;
      }
      my_prog_ += d;
      my_last_s_ = my_s;
    }

    int ahead_count = 0;
    const rclcpp::Time tnow = this->now();
    for (auto & kv : others_) {
      OtherState & o = kv.second;
      if (!o.valid || (tnow - o.stamp).seconds() > v2x_timeout_) {
        continue;
      }
      const size_t oi = nearest(in, o.x, o.y);
      const double os = s[oi];
      if (!o.prog_init) {
        if (!my_prog_init_) {
          continue;   // 自車の基準がまだ無い。この周期は見送る
        }
        double d0 = os - my_s;
        if (d0 < -total * 0.5) {
          d0 += total;
        } else if (d0 >= total * 0.5) {
          d0 -= total;
        }
        o.prog = my_prog_ + d0;
        o.last_s = os;
        o.prog_init = true;
      } else {
        double d = os - o.last_s;
        if (d < -total * 0.5) {
          d += total;
        } else if (d > total * 0.5) {
          d -= total;
        }
        o.prog += d;
        o.last_s = os;
      }
      if (o.prog > my_prog_) {
        ahead_count++;
      }
    }
    // 現在の先頭車の名前を控える。先頭は 25km/h のハンデを受けているので、
    // 2位以下から見ると最高速で構造的に上回れる相手 = 抜きにいってよい相手。
    {
      cur_leader_.clear();
      double best_prog = my_prog_;
      for (const auto & kv : others_) {
        if (!kv.second.valid || !kv.second.prog_init) { continue; }
        if (kv.second.prog > best_prog) {
          best_prog = kv.second.prog;
          cur_leader_ = kv.first;
        }
      }
    }
    const int new_rank = ahead_count + 1;
    if (new_rank != rank_) {
      RCLCPP_INFO(get_logger(), "順位変化: %d位 -> %d位", rank_, new_rank);
      rank_ = new_rank;
    }
    if (start_rank_ == 0 && race_started_) {
      start_rank_ = new_rank;
      RCLCPP_INFO(get_logger(),
                  "スタート順位 %d位%s", start_rank_,
                  start_rank_ >= 2 ? " -> 序盤にブーストを1つ使う"
                                   : " -> 前が空いているのでブーストは温存");
    }
  }

}

void V2XOvertaker::trackRanks(const Frame & f)
{
  const double total = f.total;
  const double t = f.now.seconds();
  const rclcpp::Time tnow = this->now();

  for (const auto & kv : others_) {
    if (!kv.second.valid) { continue; }
    auto & h = spd_hist_[kv.first];
    h.push_back(std::hypot(kv.second.vx, kv.second.vy));
    while (h.size() > 10) { h.pop_front(); }
  }

  if (!my_prog_init_) { return; }

  // --- 全車の順位
  std::vector<std::pair<double, std::string>> order;
  order.reserve(others_.size() + 1);
  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid || !o.prog_init) { continue; }
    if ((tnow - o.stamp).seconds() > v2x_timeout_) { continue; }
    order.emplace_back(o.prog, kv.first);
  }
  order.emplace_back(my_prog_, std::string());     // 自車は空名で入れる
  std::sort(order.begin(), order.end(),
            [](const std::pair<double, std::string> & a,
               const std::pair<double, std::string> & b) {
              return a.first > b.first;
            });
  rank_of_.clear();
  for (std::size_t i = 0; i < order.size(); ++i) {
    const int r = static_cast<int>(i) + 1;
    if (order[i].second.empty()) { my_rank_obs_ = r; }
    else { rank_of_[order[i].second] = r; }
  }

  // --- 被追越の検出
  // 順位ではなく**進行度の前後関係**で見る(順位はラップの数え方で揺れる)。
  // 誤検出を防ぐ条件:
  //   1) レース開始後だけ。グリッド待機中の位置の揺れを数えない。
  //   2) 進行度差が半周未満のときだけ。周回のまたぎで符号が反転するため。
  //   3) V2X が新しいときだけ。古い位置で「前に出た」と誤判定しない。
  //   4) 「一度 margin 以上後ろで確定した相手」だけを対象にする。
  //      初観測で前にいる相手を被追越として数えないため。
  //   5) margin 以上前に出た状態が overtaken_hold_ 秒続いたときだけ確定。
  if (!race_started_) { return; }
  for (const auto & kv : others_) {
    const std::string & name = kv.first;
    const OtherState & o = kv.second;
    OvtWatch & w = ovt_[name];
    if (!o.valid || !o.prog_init ||
        (tnow - o.stamp).seconds() > v2x_timeout_) {
      w.since = -1.0;
      continue;
    }
    const double diff = o.prog - my_prog_;      // 正 = 相手が前
    if (std::abs(diff) > total * 0.5) { w.since = -1.0; continue; }

    if (diff < -overtaken_margin_) {            // はっきり後ろ
      w.was_behind = true;
      w.since = -1.0;
      continue;
    }
    if (diff <= overtaken_margin_) {            // 並走域。確定を保留
      w.since = -1.0;
      continue;
    }
    // ここから先は「はっきり前に出ている」
    if (!w.was_behind) { w.since = -1.0; continue; }
    if (w.since < 0.0) {
      w.since = t;
      w.rank0 = my_rank_obs_;
      continue;
    }
    if ((t - w.since) < overtaken_hold_) { continue; }

    // 確定。イベント時のみ1行。
    w.was_behind = false;
    w.since = -1.0;
    last_overtaken_t_ = t;
    diagWarn("被追越", "被追越 相手=%s 自順位=P%d->P%d 相手順位=P%d idx=%zu "
                "自車=%.1fkm/h 相手=%.1fkm/h 状態=%s "
                "自車ペナ=%d 相手ペナ=%d 種別=%s",
                name.c_str(), w.rank0, my_rank_obs_, oppRank(name), f.ei,
                std::abs(f.ev) * 3.6,
                std::hypot(o.vx, o.vy) * 3.6, ovStateName(),
                selfPenalized() ? 1 : 0, isPenalized(name) ? 1 : 0,
                passKind(name));
  }
}

void V2XOvertaker::logContactContext(const Frame & f, PlanCtx & c)
{
  const double t = f.now.seconds();
  const double sp = std::abs(f.ev);
  const double prev = contact_prev_speed_;
  contact_prev_speed_ = sp;
  if (!race_started_ || prev < 0.0 || !odom_) { return; }

  // --- 検出
  const auto & q = odom_->pose.pose.orientation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  double clear = 1e9;
  bool have_clear = false;
  bool veh_overlap = false;
  if (occ_.ok) {
    clear = bodyClearance(f.ex, f.ey, yaw, 0.0);
    have_clear = true;
    veh_overlap = otherHits(f.ex, f.ey, yaw, 0.0);
  } else if (corridor_.lo.size() == f.n &&
             corridor_.hi.size() == corridor_.lo.size()) {
    // 占有格子が無いときはコリドア境界までの距離で代用する。
    const double lat = my_lat_for_target_;
    clear = std::min(lat - corridor_.lo[f.ei], corridor_.hi[f.ei] - lat);
    have_clear = true;
  }
  const bool wall_hit = have_clear && clear < 0.0;
  const bool decel_hit = (prev - sp) >= contact_decel_;
  if (!wall_hit && !decel_hit) { return; }
  // 1回の接触を復帰のもがきで何度も数えない。既存の接触ログと同じ間隔を使う。
  if ((f.now - last_contact_ctx_log_).seconds() < contact_log_hold_) { return; }
  last_contact_ctx_log_ = f.now;

  // --- 文脈を集める
  double nearest = 1e9;
  std::string who;
  int near_cnt = 0;
  for (const auto & kv : others_) {
    if (!kv.second.valid) { continue; }
    const double d = std::hypot(kv.second.x - f.ex, kv.second.y - f.ey);
    if (d < contact_alone_dist_) { ++near_cnt; }
    if (d < nearest) { nearest = d; who = kv.first; }
  }

  const char * kind = "不明";
  if (veh_overlap && wall_hit) { kind = "車両+壁"; }
  else if (veh_overlap) { kind = "車両"; }
  else if (wall_hit) { kind = "壁"; }

  const bool avoiding =
    c.lat_intent.why != nullptr &&
    (std::string(c.lat_intent.why) == "衝突回避" ||
     std::string(c.lat_intent.why) == "停止車回避");
  const char * phase = "その他";
  if (near_cnt == 0) { phase = "単独"; }
  else if (ovPassing()) { phase = "追越中"; }
  else if ((t - last_pass_ok_t_) <= contact_after_pass_sec_) { phase = "追越直後"; }
  else if (avoiding) { phase = "回避中"; }
  else if ((t - last_overtaken_t_) <= 3.0) { phase = "被追越中"; }
  else if (!c.blocker.empty()) { phase = "追従中"; }

  const double ospd = (nearest < 1e8 && others_.count(who))
    ? std::hypot(others_.at(who).vx, others_.at(who).vy) * 3.6 : -1.0;
  const double since_pass = (last_pass_ok_t_ < -1e8) ? -1.0 : (t - last_pass_ok_t_);

  diagWarn("接触記録", "接触記録 種別=%s 相手=%s idx=%zu 自車=%.1fkm/h 相手=%.1fkm/h "
              "状態=%s 局面=%s 横位置=%.2f 壁まで=%.2fm 近傍車=%d "
              "自車ペナ=%d 相手ペナ=%d 相手急減速=%.1fkm/h/s 直前追越=%.1fs前",
              kind, who.empty() ? "-" : who.c_str(), f.ei,
              sp * 3.6, ospd, ovStateName(), phase,
              my_lat_for_target_, have_clear ? clear : -1.0, near_cnt,
              selfPenalized() ? 1 : 0,
              who.empty() ? 0 : (isPenalized(who) ? 1 : 0),
              who.empty() ? 0.0 : oppAccelKmh(who), since_pass);
}

double V2XOvertaker::minEdgeClearAhead(const Frame & f, double lat, double dist) const
{
  const std::size_t n = f.n;
  if (n == 0 || corridor_.lo.size() != n || corridor_.hi.size() != n) { return 1e9; }
  double worst = 1e9;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t i = (f.ei + k) % n;
    double g = f.s[i] - f.s[f.ei];
    if (g < 0.0) { g += f.total; }
    if (g > dist) { break; }
    worst = std::min(worst, std::min(lat - corridor_.lo[i], corridor_.hi[i] - lat));
  }
  return worst;
}

double V2XOvertaker::minWidthAhead(const Frame & f, double dist) const
{
  const std::size_t n = f.n;
  if (n == 0 || corridor_.hi.size() != n || corridor_.lo.size() != n) { return 1e9; }
  if (!(dist > 0.0)) { return 1e9; }
  double w_min = 1e9;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t i = (f.ei + k) % n;
    double g = f.s[i] - f.s[f.ei];
    if (g < 0.0) { g += f.total; }
    if (g > dist) { break; }
    w_min = std::min(w_min, corridor_.hi[i] - corridor_.lo[i]);
  }
  return w_min;
}

// この区間で並走できるかを判定する(使える幅と、その区間の残り距離)。
void V2XOvertaker::evaluateZone(const Frame & f, PlanCtx & c)
{
  const std::vector<double> & s = f.s;
  const size_t n = f.n;
  const double total = f.total;
  const size_t ei = f.ei;

  // --- この区間で並走できるか判定する
  // ヘアピン(左右合計 2.90 m)ではカート2台(1.45 m x2)で隙間ゼロ。
  // 物理的に並べない場所で横間隔を取ろうとすると、互いを壁へ押し込んで両方詰まる。
  // 狭い区間では追い越しをあきらめ、縦に並んで追従する。
  // 追い越しは「事前に決めたゾーン」でのみ行う。
  // 抜けない場所で試みると、横に出ても抜けずに減速するだけで遅くなる。
  // ゾーンは make_corridor.py が幅(>=5m)と曲率半径(>=12m)から算出して
  // corridor CSV の pass_ok 列に入れてある。
  // 追い越しに使える残り距離。ゾーンが尽きるまでに抜き切れないなら出ない。
  if (corridor_.pass_ok.size() == n) {
    c.in_zone = false;
    c.avail_width = 0.0;
    room_hi_ = 1e9;
    room_lo_ = -1e9;
    v2x_overtaker::FunnelInput funnel_in;
    funnel_ds_acc_ = -1.0;
    funnel_empty_at_ = -1;
    bool first = true;
    bool zone_started = false;
    c.zone_remain = 0.0;
    for (size_t k = 0; k < n; ++k) {
      const size_t i = (ei + k) % n;
      double g = s[i] - s[ei];
      if (g < 0) {
        g += total;
      }
      if (g > std::max(look_width_ahead_, zone_look_ahead_)) {
        break;
      }
      if (g <= zone_look_ahead_) {
        if (corridor_.pass_ok[i]) {
          c.in_zone = true;
          zone_started = true;
          c.zone_remain = g;        // ゾーンが続く限り伸ばす
        } else if (zone_started) {
          break;                  // ゾーンが途切れたらそこまで
        }
      }
      // 幅と左右の余地は「今すぐ通る範囲」で見る。遠くまで含めると
      // 一箇所でも狭い所があるだけで出られなくなる。
      if (g > look_width_ahead_) {
        continue;
      }
      const double w = corridor_.hi[i] - corridor_.lo[i];
      if (first || w < c.avail_width) {
        c.avail_width = w;
        first = false;
      }
      double safety = safetyAt(i);
      if (attempt_active_) { safety = std::min(safety, corridor_safety_pass_); }
      // 幅も曲率も検証済みのゾーンでは、縁までの余裕をさらに削って
      // 横の余地を稼ぐ。却下の 70% は「側の余地なし」だった。
      if (corridor_.pass_ok.size() == n && corridor_.pass_ok[i]) {
        safety = std::min(safety, corridor_safety_zone_);
      }
      if (g <= side_room_ahead_) {
        room_hi_ = std::min(room_hi_, corridor_.hi[i] - safety);
        room_lo_ = std::max(room_lo_, corridor_.lo[i] + safety);
      }
      if (reachable_lat_clamp_ && g <= reachable_lat_ahead_) {
        const double v_ref = std::max(std::abs(my_speed_for_gap_), 5.0);
        const double reach = std::max(offset_rate_, 0.05) * (g / v_ref);
        room_hi_ = std::min(room_hi_, (corridor_.hi[i] - safety) + reach);
        room_lo_ = std::max(room_lo_, (corridor_.lo[i] + safety) - reach);
      }
      // 漏斗(後ろ向き到達可能性)用に、余裕を引いた可動域を先まで溜める。
      if (corridor_funnel_ && g <= funnel_ahead_m_) {
        funnel_in.lo.push_back(corridor_.lo[i] + safety);
        funnel_in.hi.push_back(corridor_.hi[i] - safety);
        if (funnel_in.lo.size() >= 2 && funnel_ds_acc_ <= 0.0) {
          funnel_ds_acc_ = std::max(g, 0.1);   // 最初の区間長を代表値にする
        }
      }
    }
    // 先で入れない場所へ通じる横位置を、いま禁じる。
    if (corridor_funnel_ && funnel_in.lo.size() >= 2) {
      funnel_in.ds = (funnel_ds_acc_ > 0.0)
        ? funnel_ds_acc_ : (funnel_ahead_m_ / funnel_in.lo.size());
      funnel_in.lat_rate_mps = offset_rate_;
      // my_speed_for_gap_ はこの関数の末尾で更新されるため、ここでは
      // 1周期前の値になる。速度は漏斗の幅を直に決めるので odom から取る。
      funnel_in.speed_mps = odom_ ? std::abs(odom_->twist.twist.linear.x)
                                  : std::abs(my_speed_for_gap_);
      funnel_in.speed_floor = funnel_speed_floor_;
      const auto fr = v2x_overtaker::corridorFunnel(funnel_in);
      if (fr.valid) {
        const double here_lo = corridor_.lo[ei] + safetyAt(ei);
        const double here_hi = corridor_.hi[ei] - safetyAt(ei);
        room_lo_ = std::clamp(fr.lo, here_lo, here_hi);
        room_hi_ = std::clamp(fr.hi, here_lo, here_hi);
        if (room_lo_ > room_hi_) { room_lo_ = here_lo; room_hi_ = here_hi; }
        funnel_empty_at_ = fr.empty_at;
      }
    }
  }
  // 相手が極端に遅い(止まっている・壁に当たっている)ときは、
  // ゾーン外でも幅さえあれば抜く。壊れた車の後ろで待ち続けるのは損なので。

  my_speed_for_gap_ = odom_->twist.twist.linear.x;

  // レーンが側を右に決めているか。chooseSide の中でも更新するが、
  // 前方に相手がいない周期は chooseSide が呼ばれず値が古いまま残る。
  // ここで毎周期そろえておく(buildBand など別の関数からも読むため)。
  ot_lane_side_ =
    ot_lane_side_right_ && otLaneApproach(ei, my_speed_for_gap_, rankSpeedCap());
}

// 後ろから詰められているかを1周期に1回だけ求める。
void V2XOvertaker::checkPressedFromBehind(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;

  // --- 後ろから詰められているか(1周期に1回だけ求める)
  // 追従の減速とブーストの両方で使う。後ろに車がいるのに前車より遅くまで
  // 落ちると、抜けないうえに自分が抜かれる。
  {
    const auto & pa = in.points[ei].pose.position;
    const auto & pb = in.points[(ei + 2) % n].pose.position;
    double fx = pb.x - pa.x, fy = pb.y - pa.y;
    const double fl = std::hypot(fx, fy);
    if (fl > 1e-9) { fx /= fl; fy /= fl; }
    for (const auto & kv : others_) {
      if (!kv.second.valid) { continue; }
      const double dx = kv.second.x - ex, dy = kv.second.y - ey;
      const double b = -(dx * fx + dy * fy);          // 正なら後方
      const double side = std::abs(-dx * fy + dy * fx);
      if (b > 0.0 && b < free_boost_defend_dist_ && side < front_lane_half_ * 2.0) {
        c.pressed_from_behind = true;
        c.rear_gap = std::min(c.rear_gap, b);   // いちばん近い後方車まで
      }
    }
  }

}

void V2XOvertaker::logStopCause(const Frame & f)
{
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  {
    const double sp = std::abs(odom_->twist.twist.linear.x);
    bool slowing_for_corner = false;
    if (corridor_.lo.size() == n) {
      const double lo = corridor_.lo[ei], hi = corridor_.hi[ei];
      const double lat = my_lat_for_target_;
      // 左右どちらの境界からも余裕があるなら、壁には当たっていない
      if (lat > lo + kContactMargin && lat < hi - kContactMargin) {
        slowing_for_corner = true;
      }
    }
    if (!race_started_) {
      stall_since_ = this->now();
      stall_logged_ = false;
    } else if (sp < 0.4 && !slowing_for_corner) {
      const rclcpp::Time tn = this->now();
      if (!stall_logged_ && (tn - stall_since_).seconds() > 0.8 &&
          (tn - last_contact_log_).seconds() > contact_log_hold_) {
        stall_logged_ = true;
        last_contact_log_ = tn;
        // 近くに他車がいれば車両接触、いなければ壁とみなす
        double nearest = 1e9;
        std::string who;
        for (const auto & kv : others_) {
          if (!kv.second.valid) {
            continue;
          }
          const double d = std::hypot(kv.second.x - ex, kv.second.y - ey);
          if (d < nearest) {
            nearest = d;
            who = kv.first;
          }
        }
        if (nearest < contact_vehicle_dist_) {
          // 回避層が何をしていたかを一緒に残す。これが無いと
          // 「回避が働かなかった」のか「働いたが足りなかった」のかを
          // 区別できず、どちらを直すべきか判断できない。
          RCLCPP_INFO(get_logger(),
                      "接触種別=車両 相手=%s 距離=%.1fm rank=%d "
                      "回避[横%.2fm 減速%.1fkm/h TTC%.2fs]",
                      who.c_str(), nearest, rank_,
                      dbg_avoid_offset_,
                      dbg_avoid_cap_ < 0.0 ? -1.0 : dbg_avoid_cap_ * 3.6,
                      dbg_avoid_ttc_);
        } else {
          // どのコーナーで当たっているかを特定するため位置と経路上の番号を残す。
          RCLCPP_INFO(get_logger(),
                      "接触種別=壁 最近傍車=%.1fm rank=%d idx=%zu 位置=(%.1f,%.1f) 横=%.2f "
                      "回避[横%.2fm 減速%.1fkm/h]",
                      nearest, rank_, ei, ex, ey, my_lat_for_target_,
                      dbg_avoid_offset_,
                      dbg_avoid_cap_ < 0.0 ? -1.0 : dbg_avoid_cap_ * 3.6);
        }
      }
    } else {
      stall_since_ = this->now();
      stall_logged_ = false;
    }
  }

}

// 前方の他車を探し、現在地点の曲率の向きを求める。スタート前は何もしない。
void V2XOvertaker::findFrontCar(const Frame & f)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;

  // --- 前方の他車を探す
  // スタート前は何もしない。
  // 準備段階(Ready)で回避や追い越しが動くと、無意味に横へ出た状態で
  // グリッドに並ぶことになり、スタート直後の接触につながる。
  // /awsim/state は車両ドメインにも remap されているので購読できる。
  if (!race_started_) {
    pub_->publish(in);
    return;
  }

  // 反発計算で使う自車の横位置
  {
    double nx0, ny0;
    normalAt(in, ei, nx0, ny0);
    const auto & lp0 = in.points[ei].pose.position;
    my_lat_for_target_ = (ex - lp0.x) * nx0 + (ey - lp0.y) * ny0;
    // 少し先の経路の曲がる向きを求める。イン/アウトの判定に使う。
    // 外積の符号が正なら左旋回。
    if (n > 20) {
      const auto & a = in.points[ei].pose.position;
      const auto & b = in.points[(ei + 8) % n].pose.position;
      const auto & p3 = in.points[(ei + 16) % n].pose.position;
      const double cross = (b.x - a.x) * (p3.y - b.y) - (b.y - a.y) * (p3.x - b.x);
      curve_sign_ = (std::abs(cross) < 0.5) ? 0.0 : ((cross > 0) ? 1.0 : -1.0);
    }
  }
}


void V2XOvertaker::buildInsideTable(const Trajectory & in)
{
  const std::size_t n = in.points.size();
  if (n < 24) { inside_at_.clear(); return; }
  if (inside_at_.size() == n) { return; }   // 点数が変わらない間は作り直さない
  inside_at_.assign(n, 0.0);
  // イン側の強さの基準になる曲率半径[m]。これより小さいほど強く 1.0 へ寄る。
  constexpr double kRefRadius = 25.0;
  for (std::size_t i = 0; i < n; ++i) {
    const auto & a = in.points[i].pose.position;
    const auto & b = in.points[(i + 8) % n].pose.position;
    const auto & c = in.points[(i + 16) % n].pose.position;
    // 外積の符号が正なら左旋回。左旋回のイン側は左(横位置が正)。
    const double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
    const double sign = (std::abs(cross) < 0.5) ? 0.0 : ((cross > 0.0) ? 1.0 : -1.0);
    double strength = 0.0;
    if (corridor_.radius.size() == n) {
      const double r = corridor_.radius[i];
      if (r > 1.0 && r < 1e6) { strength = std::clamp(kRefRadius / r, 0.0, 1.0); }
    }
    inside_at_[i] = sign * strength;
  }
  RCLCPP_INFO(get_logger(), "相手モデルのイン側の表を作成 %zu 点", n);
}

double V2XOvertaker::insideAt(std::size_t idx) const
{
  if (idx >= inside_at_.size()) { return 0.0; }
  return inside_at_[idx];
}

// 相手の横位置の地図が「側を決めるのに足りているか」。
int V2XOvertaker::mapMinPts(bool zone_clipped) const
{
  return zone_clipped ? std::min(lane_map_min_pts_, 3) : lane_map_min_pts_;
}
bool V2XOvertaker::mapKnownOk(int known, bool zone_clipped) const
{
  return known >= mapMinPts(zone_clipped);
}

double V2XOvertaker::oppLatAt(
  const OtherState & o, std::size_t idx, std::size_t n) const
{
  const int bin = (n > 0)
                    ? static_cast<int>(idx * OtherState::kLatBins / n) : -1;
  const double rec = o.laneLatProvisional(bin);
  if (rec < 1e8) { return rec; }            // 実際に通った地点の観測を優先
  if (!opp_model_enable_) { return 1e9; }
  return o.modelLat(insideAt(idx));
}

double V2XOvertaker::queueCapFor(const OtherState & o, double look) const
{
  if (line_x_.empty() || !o.prog_init) { return -1.0; }
  double cap = -1.0;
  for (const auto & kv : others_) {
    const OtherState & q = kv.second;
    if (&q == &o || !q.valid || !q.prog_init) { continue; }
    const double d = q.prog - o.prog;
    if (d > 0.0 && d <= look) {
      const double v = std::hypot(q.vx, q.vy);
      cap = (cap < 0.0) ? v : std::min(cap, v);
    }
  }
  // 自分が相手の前にいる場合も同じ(相手はこちらに詰まっている)。
  if (my_prog_init_) {
    const double d = my_prog_ - o.prog;
    if (d > 0.0 && d <= look) {
      const double v = std::max(my_speed_for_gap_, 0.0);
      cap = (cap < 0.0) ? v : std::min(cap, v);
    }
  }
  return cap;
}

double V2XOvertaker::oppSpdAt(
  const OtherState & o, std::size_t idx, std::size_t n) const
{
  const int bin = (n > 0)
                    ? static_cast<int>(idx * OtherState::kLatBins / n) : -1;
  const double rec = o.laneSpdProvisional(bin);
  if (rec > 0.0) { return rec; }
  if (!opp_model_enable_) { return -1.0; }
  const double r = (corridor_.radius.size() > idx) ? corridor_.radius[idx] : 1e9;
  double v = o.modelSpd(r);
  if (v <= 0.0) { return v; }
  // 前が詰まっているなら、その車の速度を超えることはできない。
  const double cap = queueCapFor(o, 12.0);
  if (cap >= 0.0) { v = std::min(v, std::max(cap, 0.5)); }
  return v;
}

// 相手の走行ラインを学習する(前方かどうかに関係なく毎周期)。
// あわせてブースト要求を毎周期作り直す。
void V2XOvertaker::learnOpponentLine(const Frame & f)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double total = f.total;
  const rclcpp::Time now = f.now;

  // --- 相手の走行ラインを学習する(前方かどうかに関係なく毎周期)。
  if (lane_learn_ && n > 0 && start_merge_done_) {
    for (auto & kv : others_) {
      OtherState & os = kv.second;
      if (!os.valid || (now - os.stamp).seconds() > v2x_timeout_) { continue; }
      const size_t li = nearest(in, os.x, os.y);
      double lnx, lny;
      normalAt(in, li, lnx, lny);
      const auto & llp = in.points[li].pose.position;
      const double llat = (os.x - llp.x) * lnx + (os.y - llp.y) * lny;
      // コリドアから明らかに外れた値(接触・スピン・自己位置の飛び)は覚えない。
      if (std::abs(llat) > 6.0) { continue; }
      const int lbin = static_cast<int>(li * OtherState::kLatBins / n);
      if (lane_record_enable_) { os.noteLat(lbin, llat); }
      if (opp_model_enable_) {
        buildInsideTable(in);
        const double lr = (corridor_.radius.size() == n) ? corridor_.radius[li] : 1e9;
        os.noteModel(std::hypot(os.vx, os.vy), lr, llat, insideAt(li));
      }
      {
        const double osp = std::hypot(os.vx, os.vy);
        if (lane_record_enable_ && osp > 0.3 && osp < 30.0) { os.noteSpd(lbin, osp); }
      }

      // --- 抜いた/抜かれたを数える
      // 進行度差は周回のまたぎで大きく振れるので、妥当な範囲のときだけ見る。
      // 差が 0 付近で揺れるだけで数えないよう、pass_len の半分まで
      // 離れた状態が連続して続いたときだけ反転を確定させる。
      if (my_prog_init_ && os.prog_init) {
        const double diff = my_prog_ - os.prog;
        if (std::abs(diff) < total * 0.5) {
          const int want = (diff > pass_len_ * 0.5) ? +1
                         : ((diff < -pass_len_ * 0.5) ? -1 : 0);
          if (want != 0 && want != os.rel_sign) {
            if (++os.rel_hold >= 5) {
              if (os.rel_sign != 0) {
                if (want > 0) {
                  os.passed_cnt++;
                  RCLCPP_INFO(get_logger(),
                    "抜いた target=%s 累計=%d 差=%.1fm %d周目",
                    kv.first.c_str(), os.passed_cnt, diff, lap_ + 1);
                } else {
                  os.overtaken_cnt++;
                  RCLCPP_INFO(get_logger(),
                    "抜かれた target=%s 累計=%d 差=%.1fm %d周目",
                    kv.first.c_str(), os.overtaken_cnt, diff, lap_ + 1);
                }
              }
              os.rel_sign = want;
              os.rel_hold = 0;
            }
          } else {
            os.rel_hold = 0;
          }
        }
      }
    }
  }

  // ブーストの要求は毎周期その場で作り直す。
  // 以前は前方車を処理したときにしか false へ戻らず、要求が残ったまま
  // 次の周期で二重発射したり、!want_boost_ を条件にしている
  // 「残ったブーストの使い道」が永久に塞がれたりしていた。
  // 判定はこの下で毎周期やり直すので、条件が続いていれば
  // 「armする周期 -> 撃つ周期」の2周期はそのまま成立する。
  want_boost_ = false;
  start_boost_pending_ = false;
}


void V2XOvertaker::assignStartSlots(const Frame & f)
{
  if (!race_started_) { return; }
  if (race_start_time_ < 0.0) {
    race_start_time_ = f.now.seconds();
    launch_since_ = race_start_time_;
  }
  if (slots_assigned_ || grid_slots_.empty()) { return; }

  auto match = [&](double x, double y, int & slot) {
    double bd = 1e18;
    slot = 0;
    for (std::size_t i = 0; i < grid_slots_.size(); ++i) {
      const double d = std::hypot(x - grid_slots_[i].first,
                                  y - grid_slots_[i].second);
      if (d < bd) { bd = d; slot = static_cast<int>(i) + 1; }
    }
    return bd;
  };

  int mine = 0;
  const double my_d = match(f.ex, f.ey, mine);

  // グリッドから離れていれば、まだ照合できる位置にいない(あるいは
  // 記録済み座標が今回のグリッドと違う)。少しの間だけ待つ。
  const double waited = f.now.seconds() - race_start_time_;
  int assigned = 0;
  std::map<std::string, int> got;
  for (const auto & kv : others_) {
    if (!kv.second.valid) { continue; }
    int sl = 0;
    const double d = match(kv.second.x, kv.second.y, sl);
    if (d < 3.0 && sl != mine) { got[kv.first] = sl; assigned++; }
  }
  const std::size_t car_n = 1 + others_.size();
  const bool grid_usable =
    !grid_slots_.empty() && grid_slots_.size() >= car_n && my_d < 3.0;
  if (!grid_usable && !others_.empty() &&
      (waited >= 2.0 || others_.size() + 1 >= car_n)) {
    std::vector<std::pair<double, std::string>> ord;
    ord.emplace_back(my_prog_, std::string());
    for (const auto & kv : others_) {
      if (kv.second.valid) { ord.emplace_back(kv.second.prog, kv.first); }
    }
    std::sort(ord.begin(), ord.end());
    int slot = 1;
    std::string list2;
    for (const auto & e : ord) {
      if (e.second.empty()) { start_slot_ = slot; }
      else { others_[e.second].slot = slot; }
      list2 += " " + (e.second.empty() ? std::string("自車") : e.second) +
               "=P" + std::to_string(slot);
      ++slot;
    }
    slots_assigned_ = true;
    std::string pos = "自車(" + std::to_string(f.ex) + "," + std::to_string(f.ey) + ")";
    for (const auto & kv : others_) {
      pos += " " + kv.first + "(" + std::to_string(kv.second.x) + "," +
             std::to_string(kv.second.y) + ")";
    }
    RCLCPP_WARN(get_logger(),
      "グリッド確定(進行度順): 台数=%zu grid_slots=%zu 照合=%.2fm%s / 実測座標 %s",
      car_n, grid_slots_.size(), my_d, list2.c_str(), pos.c_str());
    return;
  }

  const bool ready = (my_d < 3.0) && (assigned + 1 >= static_cast<int>(car_n));
  if (!ready && waited < 2.0) { return; }   // V2X の到着を少し待つ

  start_slot_ = (my_d < 3.0) ? mine : start_slot_;
  for (auto & kv : others_) {
    const auto it = got.find(kv.first);
    if (it != got.end()) { kv.second.slot = it->second; }
  }
  slots_assigned_ = true;
  std::string list;
  for (const auto & kv : others_) {
    list += " " + kv.first + "=P" + std::to_string(kv.second.slot);
  }
  RCLCPP_INFO(get_logger(),
    "グリッド確定: 自車=P%d(照合 %.2fm)%s  NPC=P%d 録画周=%d "
    "僚車解禁=%d周目 ブースト解禁=%d周目",
    start_slot_, my_d, list.c_str(), npc_slot_, record_laps_,
    teammate_pass_lap_ + 1, boost_min_lap_ + 1);
}


bool V2XOvertaker::straightPassNow(const Frame & f)
{
  if (!straight_pass_enable_) { straight_pass_latch_ = false; return false; }
  const bool in_zone = inSidePickZone(f.ei);
  const bool trying = attempt_active_ || can_pass_now_;
  const double t = f.now.seconds();
  if (in_zone && trying) {
    straight_pass_latch_ = true;
    straight_pass_since_ = t;
    return true;
  }
  if (straight_pass_latch_) {
    // 仕掛けが終わったか、区間を出てから straight_pass_hold 秒たったら解く。
    if (!trying || (t - straight_pass_since_) > straight_pass_hold_) {
      straight_pass_latch_ = false;
    }
  }
  return straight_pass_latch_;
}

bool V2XOvertaker::sideStaysOpen(const Frame & f, double off, double ahead) const
{
  if (ahead <= 0.0) { return true; }
  const std::size_t n = f.n;
  if (corridor_.lo.size() != n || corridor_.hi.size() != n) { return true; }
  // 予測バンドがあれば壁だけでなく、到達時刻をそろえた相手の掃引領域も見る。
  // これにより「壁側は開いているが、相手がその側へ来る」場面では
  // 速度上限を解除して抜き切りへ入らない。
  const bool use_band = band_enable_ && band_lo_.size() == n && band_hi_.size() == n;
  double acc = 0.0;
  for (std::size_t k = 1; k < n; ++k) {
    const std::size_t a = (f.ei + k - 1) % n, b = (f.ei + k) % n;
    acc += std::hypot(
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x,
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y);
    if (acc > ahead) { break; }
    const double lo = use_band ? band_lo_[b] : corridor_.lo[b] + safetyAt(b);
    const double hi = use_band ? band_hi_[b] : corridor_.hi[b] - safetyAt(b);
    const double kept = std::clamp(off, std::min(lo, hi), std::max(lo, hi));
    if (std::abs(kept - off) > commit_crush_) { return false; }
  }
  return true;
}

int V2XOvertaker::sidePickZoneIndex(std::size_t idx) const
{
  for (std::size_t i = 0; i < side_pick_zones_.size(); ++i) {
    const auto & z = side_pick_zones_[i];
    const bool inside = (z.first <= z.second)
                          ? (idx >= z.first && idx <= z.second)
                          : (idx >= z.first || idx <= z.second);
    if (inside) { return static_cast<int>(i); }
  }
  return -1;
}

bool V2XOvertaker::inSidePickZone(std::size_t idx) const
{
  for (const auto & z : side_pick_zones_) {
    const bool inside = (z.first <= z.second)
                          ? (idx >= z.first && idx <= z.second)
                          : (idx >= z.first || idx <= z.second);
    if (inside) { return true; }
  }
  return false;
}

// 直線(抜き切りに使える区間)の中か。side_pick_zones_ とは目的が違う。
bool V2XOvertaker::inPassFinishZone(std::size_t idx) const
{
  for (const auto & z : pass_finish_zones_) {
    const bool inside = (z.first <= z.second)
                          ? (idx >= z.first && idx <= z.second)
                          : (idx >= z.first || idx <= z.second);
    if (inside) { return true; }
  }
  return false;
}

// いまいる直線の終わりまでの距離[m]。直線の外なら -1。
double V2XOvertaker::distToPassFinishZoneEnd(const Frame & f) const
{
  if (!inPassFinishZone(f.ei)) { return -1.0; }
  double acc = 0.0;
  for (std::size_t k = 1; k < f.n; ++k) {
    const std::size_t a = (f.ei + k - 1) % f.n, b = (f.ei + k) % f.n;
    acc += std::hypot(
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x,
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y);
    if (!inPassFinishZone(b)) { return acc; }
  }
  return acc;
}

double V2XOvertaker::distToSidePickZoneEnd(const Frame & f) const
{
  if (!inSidePickZone(f.ei)) { return -1.0; }
  double acc = 0.0;
  for (std::size_t k = 1; k < f.n; ++k) {
    const std::size_t a = (f.ei + k - 1) % f.n, b = (f.ei + k) % f.n;
    acc += std::hypot(
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x,
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y);
    if (!inSidePickZone(b)) { return acc; }
  }
  return acc;
}

double V2XOvertaker::zoneMeanLat(const OtherState & o, std::size_t n, int & known,
                                 int zone) const
{
  double sum = 0.0;
  known = 0;
  if (n == 0) { return 1e9; }
  for (int b = 0; b < OtherState::kLatBins; ++b) {
    const std::size_t idx = static_cast<std::size_t>(b) * n / OtherState::kLatBins;
    // ゾーン番号で絞る。
    if (zone >= 0 && side_zone_mean_split_) {
      if (sidePickZoneIndex(idx) != zone) { continue; }
    } else if (!inSidePickZone(idx)) { continue; }
    const double l = side_model_fill_ ? oppLatAt(o, idx, n) : o.laneLat(b);
    if (l > 1e8) { continue; }
    sum += l;
    known++;
  }
  if (known < lane_map_min_pts_) { return 1e9; }
  return sum / known;
}


void V2XOvertaker::planPassSpot(const Frame & f)
{
  if (!spot_enable_ || !race_started_) { return; }

  // planOvertake が前周期に確定した「実際の目前車」。経路上で最も近い車を
  // ここでも同じ基準で扱い、奥にいる別車の計画で目前車を塞がない。
  const OtherState * live_blocker = nullptr;
  double live_blocker_ahead = 1e18;
  const auto bit = others_.find(c_blocker_);
  if (bit != others_.end() && bit->second.valid &&
      (f.now - bit->second.stamp).seconds() <= v2x_timeout_) {
    const size_t bi = nearest(f.in, bit->second.x, bit->second.y);
    double bd = f.s[bi] - f.s[f.ei];
    if (bd < 0.0) { bd += f.total; }
    if (bd > 0.5 && bd <= detect_range_) {
      live_blocker = &bit->second;
      live_blocker_ahead = bd;
    }
  }

  if (spot_valid_ && slots_assigned_ && start_slot_ == 1) {
    if (live_blocker && c_blocker_ != spot_target_) {
      spot_valid_ = false;
      spot_calc_at_ = -1.0;
    }
  }
  if (spot_valid_ && slots_assigned_ && start_slot_ == 1) {
    bool target_ahead = false;
    const auto it = others_.find(spot_target_);
    if (it != others_.end() && it->second.valid &&
        (f.now - it->second.stamp).seconds() <= v2x_timeout_) {
      const size_t oi = nearest(f.in, it->second.x, it->second.y);
      double od = f.s[oi] - f.s[f.ei];
      if (od < 0.0) { od += f.total; }
      target_ahead = od > 0.5 && od <= spot_range_;
    }
    const bool in_span = (spot_begin_ <= spot_end_)
                           ? (f.ei >= spot_begin_ && f.ei <= spot_end_)
                           : (f.ei >= spot_begin_ || f.ei <= spot_end_);
    double d = f.s[spot_begin_] - f.s[f.ei];
    if (d < 0.0) { d += f.total; }
    spot_in_now_ = in_span;
    spot_dist_ = in_span ? 0.0 : d;
    // 固定20秒では、低速車を追従して80m先へ向かう間に期限が切れ、
    // 到着前に地点と左右が再選択される。対象と入口が幾何的に有効な間は
    // 同じ時空間計画を保持する。入口通過後は d がほぼ1周分となり範囲外になる。
    if (target_ahead &&
        (attempt_active_ || in_span || d <= spot_range_)) {
      return;
    }
    spot_valid_ = false;
  }
  if (f.now.seconds() - spot_calc_at_ < spot_recalc_sec_) { return; }
  spot_calc_at_ = f.now.seconds();

  spot_valid_ = false;
  spot_dbg_need_ = 0.0;
  spot_dbg_need_len_ = 0.0;
  spot_dbg_need_vo_ = -1.0;
  spot_dbg_l_ = 0.0;
  spot_dbg_r_ = 0.0;
  spot_dbg_known_ = 0;
  spot_in_now_ = false;
  spot_dist_ = -1.0;
  spot_side_ = 0.0;
  spot_offset_ = 0.0;
  spot_len_ = 0.0;
  spot_ospeed_ = -1.0;

  if (!slots_assigned_) { return; }
  if (!spot_all_slots_ && start_slot_ != 1) { return; }

  const Trajectory & in = f.in;
  const size_t n = f.n;
  const size_t ei = f.ei;
  if (corridor_.lo.size() != n || corridor_.hi.size() != n) { return; }
  const bool use_band_spot =
      band_enable_ && band_lo_ex_.size() == n && band_hi_ex_.size() == n;

  const OtherState * tgt = nullptr;
  std::string tname;
  double best_ahead = 1e18;
  // 目前車が存在するなら、その車以外を計画対象にしない。目前車が周回規則で
  // まだ追越禁止なら、奥の車を対象にするのでなく計画を待つ。
  if (live_blocker) {
    if (!passAllowedThisLap(c_blocker_, *live_blocker, false)) { return; }
    tgt = live_blocker;
    tname = c_blocker_;
    best_ahead = live_blocker_ahead;
  }
  for (const auto & kv : others_) {
    if (live_blocker) { break; }
    const OtherState & o = kv.second;
    if (!o.valid) { continue; }
    if ((f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
    if (!passAllowedThisLap(kv.first, o, false)) { continue; }
    const size_t oi = nearest(in, o.x, o.y);
    double d = f.s[oi] - f.s[ei];
    if (d < 0.0) { d += f.total; }
    if (d <= 0.5 || d > spot_range_) { continue; }
    if (d < best_ahead) { best_ahead = d; tgt = &o; tname = kv.first; }
  }
  if (!tgt) { return; }

  // 禁止区間の判定(既存の指定をそのまま尊重する)
  auto in_no_pass = [&](size_t idx) {
    for (const auto & z : no_pass_zones_) {
      const bool inside = (z.first <= z.second)
                            ? (idx >= z.first && idx <= z.second)
                            : (idx >= z.first || idx <= z.second);
      if (inside) { return true; }
    }
    return false;
  };

  const double rank_cap = ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
  const bool op_is_npc = (tgt->slot == npc_slot_);
  // 直線加速度をそのまま使うと、操舵中の損失・制御追従・安全上限を無視して
  // 到達距離を約3割短く見積もった。予測計画では実値の60%を使う。
  const double plan_accel = std::max(vehicle_accel_ * 0.60, 0.05);

  const size_t oi_now = nearest(in, tgt->x, tgt->y);
  double onx_now = 0.0, ony_now = 0.0;
  normalAt(in, oi_now, onx_now, ony_now);
  const auto & op_ref_now = in.points[oi_now].pose.position;
  const double olat_now = (tgt->x - op_ref_now.x) * onx_now +
                          (tgt->y - op_ref_now.y) * ony_now;
  // V2X速度を現在地点の法線へ射影する。未学習区間で横速度を無視すると、
  // 相手がこちらの選んだ側へ寄り続けても「現在位置を維持」と予測し、
  // 同じ側を追いかけて横間隔が1.3m前後から増えなくなる。
  const double ovlat_now = tgt->vx * onx_now + tgt->vy * ony_now;
  std::vector<double> op_arrival(n, 1e18);
  op_arrival[oi_now] = 0.0;
  const double op_mean = tgt->meanSpeed();
  const double op_now_speed = std::hypot(tgt->vx, tgt->vy);
  {
    size_t cur = oi_now;
    double top = 0.0;
    const double slot_cap =
      (op_is_npc ? predict_speed_slow_ : predict_speed_fast_) / 3.6;
    const double op_rank_cap =
      ((!cur_leader_.empty() && tname == cur_leader_)
         ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
    const double op_cap = std::min(slot_cap, op_rank_cap);
    for (size_t k = 1; k < n; ++k) {
      const size_t nx = (cur + 1) % n;
      double ds = f.s[nx] - f.s[cur];
      if (ds < 0.0) { ds += f.total; }
      double vop = oppSpdAt(*tgt, cur, n);
      const bool have_learned_v = vop > 0.0;
      if (!have_learned_v) { vop = (op_mean > 0.0) ? op_mean : std::hypot(tgt->vx, tgt->vy); }
      vop = vop * predict_op_margin_;
      if (!have_learned_v) { vop = std::max(vop, op_now_speed * 0.9); }
      vop = std::clamp(std::max(vop, 0.5), 0.5, std::max(op_cap, 0.5));
      top += ds / vop;
      op_arrival[nx] = top;
      cur = nx;
    }
  }

  struct Run { double from; double len; double side; double vsum; int vcnt;
               double arrival_delta;
               double offset;
               double self_v;
               size_t b; size_t e;
               double bad_len{0.0}; };
  Run cur_l{-1.0, 0.0, +1.0, 0.0, 0, 0.0, 0.0, 0.0, 0, 0};
  Run cur_r{-1.0, 0.0, -1.0, 0.0, 0, 0.0, 0.0, 0.0, 0, 0};
  double best_score = 0.0;
  Run best{};
  bool have_best = false;

  auto close_run = [&](Run & r) {
    r.len -= r.bad_len;
    r.bad_len = 0.0;
    // 地点が決まらないときに「あと何mで届くのか」を読めるようにする。
    if (r.side > 0.0) { spot_dbg_l_ = std::max(spot_dbg_l_, r.len); }
    else { spot_dbg_r_ = std::max(spot_dbg_r_, r.len); }
    // 固定15mで先に落とすと、完全追越に必要な8mと加速距離を満たす12m区間まで
    // 候補から消える。最低長は完全追越長とし、直後の運動計算(pass_d*1.15)と
    // 実行時の0.1秒シミュレーションで、本当に出口までに抜ける区間だけを残す。
    const double min_candidate_len = std::max(spot_min_len_, pass_len_);
    if (r.from < 0.0 || r.len < min_candidate_len) { r.from = -1.0; r.len = 0.0;
                                                r.vsum = 0.0; r.vcnt = 0; return; }
    const double vo = (r.vcnt > 0) ? r.vsum / r.vcnt : -1.0;
    // 入口速度から加速し、予測相手へ追いついて車体半分だけ前へ出るまでの
    // 距離を二段階(上限到達前/後)で計算する。幅が長くても、この距離が
    // 区間に収まらなければ「横へ出られるが抜けない」ので候補にしない。
    if (vo > 0.0 && r.self_v > 0.0) {
      const double entry_gap = (spot_entry_gap_ >= 0.0)
                                 ? spot_entry_gap_
                                 : (rear_end_margin_ + 1.0);
      const double need_gain = std::max(r.arrival_delta, 0.0) * vo +
                               entry_gap + pass_len_ * spot_pass_len_gain_;
      const double v_entry = std::min(r.self_v, rank_cap);
      auto required_distance = [&](double acc_v) {
        const double t_cap = std::max(rank_cap - v_entry, 0.0) / acc_v;
        const double gain_acc = std::max(v_entry - vo, 0.0) * t_cap +
                                0.5 * acc_v * t_cap * t_cap;
        double pass_d = v_entry * t_cap + 0.5 * acc_v * t_cap * t_cap;
        if (gain_acc < need_gain) {
          const double closing_cap = rank_cap - vo;
          if (closing_cap <= 0.1) { return 1e18; }
          pass_d += rank_cap * (need_gain - gain_acc) / closing_cap;
        }
        return pass_d;
      };
      const double base_d = required_distance(plan_accel);
      const bool boost_available = (is_boosting_ ||
        (boost_remaining_ > 0 && boostLapOk()));
      const double boost_d = boost_available
        ? required_distance(plan_accel + boost_accel_) : 1e18;
      const double need_d = std::min(base_d, boost_d) * spot_need_margin_;
      if (need_d < spot_dbg_need_ || spot_dbg_need_ <= 0.0) {
        spot_dbg_need_ = need_d;
        spot_dbg_need_len_ = r.len;
        spot_dbg_need_vo_ = vo;
      }
      if (need_d > r.len) {
        r.from = -1.0; r.len = 0.0; r.vsum = 0.0; r.vcnt = 0; return;
      }
    }
    // 相手が遅い区間ほど詰めやすい = 抜きやすい
    const double slow = (vo > 0.0) ? std::max(rank_cap - vo, 0.0) : 0.0;
    double score = r.len + spot_slow_bonus_ * slow;
    // 遅着は後から取り戻せないので重く罰する。一方、最速到着が相手より早い
    // 場合は今回の縦シミュレーションが加速開始を遅らせて同期できるため、軽い
    // 待機コストだけにする。以前は左右対称に罰して、遅い相手ほど候補が消えた。
    if (r.arrival_delta > 0.7) {
      score -= 3.0 * (r.arrival_delta - 0.7);
    } else if (r.arrival_delta < 0.0) {
      score -= 0.3 * (-r.arrival_delta);
    }
    score /= (1.0 + r.from / 100.0);        // 手前を優先する
    if (score > best_score) {
      best_score = score; best = r; best.vsum = vo; best.vcnt = 1; have_best = true;
    }
    r.from = -1.0; r.len = 0.0; r.vsum = 0.0; r.vcnt = 0;
  };

  double acc = 0.0;
  double self_arrival = 0.0;
  const double v0 = std::max(std::abs(f.ev), 0.5);
  for (size_t k = 1; k < n; ++k) {
    const size_t a = (ei + k - 1) % n, b = (ei + k) % n;
    const double step =
      std::hypot(in.points[b].pose.position.x - in.points[a].pose.position.x,
                 in.points[b].pose.position.y - in.points[a].pose.position.y);
    const double d0 = acc;      // この点までの距離
    acc += step;
    if (acc > spot_range_) { break; }

    // 自車が全開加速した場合の到着可能時刻。順位上限と地点速度表は超えない。
    const double v_acc = std::sqrt(v0 * v0 +
      2.0 * plan_accel * std::max(d0, 0.0));
    const double v_path = std::max<double>(in.points[a].longitudinal_velocity_mps, 0.5);
    const double v_self = std::max(std::min({v_acc, rank_cap, v_path}), 0.5);
    self_arrival += step / v_self;

    const double arrival_delta = self_arrival - op_arrival[b];
    const bool after_opponent = acc + 0.5 >= best_ahead;
    const bool interceptable = after_opponent && op_arrival[b] < 1e17 &&
                               arrival_delta >= -20.0 && arrival_delta <= 4.0;

    double ol = oppLatAt(*tgt, b, n);
    const double ov = oppSpdAt(*tgt, b, n);
    const double ov_plan = (ov > 0.0)
      ? std::max(ov * predict_op_margin_, 0.5)
      : std::max({op_mean, op_now_speed * 0.9, 0.5});
    const bool observed = ol < 1e8;
    // 相手がまだ一度も通っていない区間には録画軌跡がない。そこで何周も
    // 待つのではなく、運営が明示した追越可能ゾーン内だけは「現在の横位置が
    // 続く」と仮定する。短期衝突バンドは毎周期の実位置で別途更新されるため、
    // 相手が寄ってきた場合は計画に固執せず減速・回避できる。
    const bool mapped_pass_zone = corridor_.pass_ok.size() == n && corridor_.pass_ok[b];
    if (!observed && mapped_pass_zone) {
      const double t_lat = std::min(op_arrival[b], 2.0);
      ol = olat_now + ovlat_now * t_lat;
    }
    const bool avoid_recent =
      f.now.seconds() < spot_avoid_until_ &&
      spot_avoid_begin_ < n &&
      std::min<std::size_t>(
        (b + n - spot_avoid_begin_) % n,
        (spot_avoid_begin_ + n - b) % n) <= 5;
    if (avoid_recent) {
      close_run(cur_l);
      close_run(cur_r);
      continue;
    }
    bool geom_l = false, geom_r = false;
    double safe_lo = corridor_.lo[b] + corridor_safety_;
    double safe_hi = corridor_.hi[b] - corridor_safety_;
    if (observed) { spot_dbg_known_++; }
    if (ol < 1e8 && !in_no_pass(b)) {
      double sf = corridor_safety_;
      if (corridor_.pass_ok.size() == n && corridor_.pass_ok[b]) {
        sf = std::min(sf, corridor_safety_zone_);
      }
      safe_hi = corridor_.hi[b] - sf;
      safe_lo = corridor_.lo[b] + sf;
      // 最終段 avoidWall と同じ壁境界を候補探索にも適用する。
      // これを入れないと、計画上は1.8m空くラインが実行時に0.6m以上
      // 相手側へ押し戻され、横間隔を作れない候補を選んでいた。
      double wm = wall_margin_;
      if (!corridor_.radius.empty() && tight_radius_ > 0.0) {
        const double r = corridor_.radius[b];
        if (r < tight_radius_) {
          wm += wall_margin_tight_ *
                std::clamp((tight_radius_ - r) / tight_radius_, 0.0, 1.0);
        }
      }
      const double wall_lo = std::min(corridor_.lo[b] + wm, 0.0);
      const double wall_hi = std::max(corridor_.hi[b] - wm, 0.0);
      safe_lo = std::max(safe_lo, wall_lo);
      safe_hi = std::min(safe_hi, wall_hi);
      if (use_band_spot) {
        safe_lo = band_lo_ex_[b];
        safe_hi = band_hi_ex_[b];
      }
      geom_l = (safe_hi - ol) >= spot_margin_;
      geom_r = (ol - safe_lo) >= spot_margin_;
    }
    // 到達時刻の同期は候補区間の「入口」だけで要求する。加速中の自車は
    // 区間内で相手の予想到達時刻を追い越すため、全点で要求すると安全な
    // 幅が十分続いていても run が数mで切れ、追越地点が一切成立しない。
    // 入口を同期して選んだ後は、幅と追越禁止区間だけで出口まで延長する。
    const bool ok_l = geom_l && (cur_l.from >= 0.0 || interceptable);
    const bool ok_r = geom_r && (cur_r.from >= 0.0 || interceptable);
    // 左
    if (ok_l) {
      if (cur_l.from < 0.0) {
        cur_l.from = d0; cur_l.b = b; cur_l.arrival_delta = arrival_delta;
        cur_l.self_v = v_self;
        const double near = ol + std::max(band_car_w_, min_pass_sep_);
        cur_l.offset = std::clamp(0.5 * (near + safe_hi), near, safe_hi);
      }
      cur_l.len += step; cur_l.e = b;
      cur_l.bad_len = 0.0;
      if (ov_plan > 0.0) { cur_l.vsum += ov_plan; cur_l.vcnt++; }
    } else if (cur_l.from >= 0.0 && cur_l.bad_len + step <= spot_run_gap_) {
      cur_l.bad_len += step;
      cur_l.len += step; cur_l.e = b;
    } else {
      close_run(cur_l);
    }
    // 右
    if (ok_r) {
      if (cur_r.from < 0.0) {
        cur_r.from = d0; cur_r.b = b; cur_r.arrival_delta = arrival_delta;
        cur_r.self_v = v_self;
        const double near = ol - std::max(band_car_w_, min_pass_sep_);
        cur_r.offset = std::clamp(0.5 * (near + safe_lo), safe_lo, near);
      }
      cur_r.len += step; cur_r.e = b;
      cur_r.bad_len = 0.0;
      if (ov_plan > 0.0) { cur_r.vsum += ov_plan; cur_r.vcnt++; }
    } else if (cur_r.from >= 0.0 && cur_r.bad_len + step <= spot_run_gap_) {
      cur_r.bad_len += step;
      cur_r.len += step; cur_r.e = b;
    } else {
      close_run(cur_r);
    }
  }
  close_run(cur_l);
  close_run(cur_r);
  if (!have_best) { return; }

  spot_valid_ = true;
  spot_target_ = tname;
  spot_begin_ = best.b;
  spot_end_ = best.e;
  spot_side_ = best.side;
  spot_offset_ = best.offset;
  spot_dist_ = best.from;
  spot_len_ = best.len;
  spot_ospeed_ = best.vsum;
  spot_in_now_ = (best.from <= 0.5);
  spot_lock_until_ = f.now.seconds() + 20.0;
}

// 予測した抜きどころへ向けて、何m手前から横移動を始める必要があるか。
// 固定距離では速度が高いほど横移動が間に合わないため、横移動量と現在速度から
// 到達距離を求める。操舵応答の遅れを1.6倍、認識・制御余裕を1.5秒見込む。
double V2XOvertaker::spotGateDistance(const Frame & f, const OtherState & o) const
{
  if (!spot_valid_ || spot_side_ == 0.0) { return spot_gate_slack_; }
  const double lat_dist = std::abs(spot_offset_ - my_lat_for_target_);
  const double lat_time = 1.6 * lat_dist / std::max(offset_rate_, 0.1) + 1.5;
  const double v = std::max(std::abs(f.ev), 1.0);
  const double setup_dist = v * lat_time +
    0.5 * std::max(vehicle_accel_ * 0.60, 0.0) * lat_time * lat_time;

  // 遅い相手へ追いつく場合は、横へ出られない周期にも停止可能な距離を残す。
  const double ov = std::hypot(o.vx, o.vy);
  const double closing = std::max(v - ov, 0.0);
  const double brake_dist = closing * closing /
    (2.0 * std::max(std::abs(a_min_), 0.5)) + charge_brake_margin_;
  return std::clamp(std::max({spot_gate_slack_, setup_dist, brake_dist}),
                    spot_gate_slack_, std::min(40.0, spot_range_));
}

// 計画横位置を現在から出力しても、抜き切るまで予測バンド内に収まるか。
// 現在の publishTrajectory は offset_ を近傍経路全体へ適用するため、検査だけを
// 「入口までの線形移行」にすると、検査と実行が別軌道になる。任意曲線を実際の
// 出力へ実装するまでは、固定計画ラインが全点で安全という保守条件を使う。
bool V2XOvertaker::passPathCapable(
  const Frame & f, double ahead, double w_need,
  double & fail_at, double & min_w) const
{
  fail_at = -1.0; min_w = 1e9;
  if (ahead <= 0.0) { return true; }
  // 帯(band)は他車の予測を毎周期織り込んだ通行可能区間。無ければコリドア。
  const bool have_band =
    band_enable_ && band_lo_ex_.size() == f.n && band_hi_ex_.size() == f.n;
  const bool have_corr =
    corridor_.lo.size() == f.n && corridor_.hi.size() == f.n;
  if (!have_band && !have_corr) { return true; }   // 判定材料が無い
  double acc = 0.0, bad_len = 0.0;
  for (std::size_t k = 1; k < f.n; ++k) {
    const std::size_t a = (f.ei + k - 1) % f.n;
    const std::size_t b = (f.ei + k) % f.n;
    const double ds = std::hypot(
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x,
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y);
    acc += ds;
    if (acc > ahead) { break; }
    double lo = have_band ? band_lo_ex_[b] : corridor_.lo[b] + safetyAt(b);
    double hi = have_band ? band_hi_ex_[b] : corridor_.hi[b] - safetyAt(b);
    // 実行の最後に掛かる avoidWall の境界とも交差させる
    // (spotPathSafe と同じ式。判定と実行の基準をそろえる)。
    if (have_corr) {
      double wm = wall_margin_;
      if (!corridor_.radius.empty() && tight_radius_ > 0.0) {
        const double r = corridor_.radius[b];
        if (r < tight_radius_) {
          wm += wall_margin_tight_ *
                std::clamp((tight_radius_ - r) / tight_radius_, 0.0, 1.0);
        }
      }
      lo = std::max(lo, std::min(corridor_.lo[b] + wm, 0.0));
      hi = std::min(hi, std::max(corridor_.hi[b] - wm, 0.0));
    }
    const double width = std::max(hi - lo, 0.0);
    min_w = std::min(min_w, width);
    if (width < w_need) {
      bad_len += ds;
      if (bad_len >= spot_path_bad_len_) { fail_at = acc; return false; }
    } else {
      bad_len = 0.0;
    }
  }
  return true;
}

bool V2XOvertaker::spotPathSafe(
  const Frame & f, const OtherState & o, double ahead) const
{
  if (!spot_valid_ || ahead <= 0.0) { return true; }
  (void)o;
  const bool have_band = band_enable_ && band_lo_ex_.size() == f.n && band_hi_ex_.size() == f.n;
  const bool have_corr = corridor_.lo.size() == f.n && corridor_.hi.size() == f.n;
  if (!have_band && !have_corr) { return true; }

  spot_path_min_w_seen_ = 1e9;
  double acc = 0.0;
  double bad_len = 0.0;
  for (std::size_t k = 1; k < f.n; ++k) {
    const std::size_t a = (f.ei + k - 1) % f.n;
    const std::size_t b = (f.ei + k) % f.n;
    const double ds = std::hypot(
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x,
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y);
    acc += ds;
    if (acc > ahead) { break; }
    const double planned = spot_offset_;
    double lo = have_band ? band_lo_ex_[b] : corridor_.lo[b] + safetyAt(b);
    double hi = have_band ? band_hi_ex_[b] : corridor_.hi[b] - safetyAt(b);
    // 実行の最後に掛かる avoidWall の境界とも交差させる。
    if (have_corr) {
      double wm = wall_margin_;
      if (!corridor_.radius.empty() && tight_radius_ > 0.0) {
        const double r = corridor_.radius[b];
        if (r < tight_radius_) {
          wm += wall_margin_tight_ *
                std::clamp((tight_radius_ - r) / tight_radius_, 0.0, 1.0);
        }
      }
      lo = std::max(lo, std::min(corridor_.lo[b] + wm, 0.0));
      hi = std::min(hi, std::max(corridor_.hi[b] - wm, 0.0));
    }
    const double width = std::max(hi - lo, 0.0);
    spot_path_min_w_seen_ = std::min(spot_path_min_w_seen_, width);
    const double kept = std::clamp(planned, std::min(lo, hi), std::max(lo, hi));
    const bool line_ok = std::abs(kept - planned) <= spot_path_tol_;
    const double need_w = spot_w_ref_ ? spot_w_ref_min_ : spot_path_min_w_;
    const bool width_ok_here = width >= need_w;
    if (!line_ok && !width_ok_here) {
      bad_len += ds;
      if (bad_len >= spot_path_bad_len_) {
        spot_path_fail_at_ = acc;
        spot_path_fail_w_ = width;
        // その地点で「車がいなければ何m あったか」を残す。
        spot_path_fail_idx_ = static_cast<int>(b);
        spot_path_fail_lo_ = lo;
        spot_path_fail_hi_ = hi;
        spot_path_fail_free_w_ = have_corr
          ? std::max((corridor_.hi[b] - safetyAt(b)) -
                     (corridor_.lo[b] + safetyAt(b)), 0.0)
          : -1.0;
        return false;
      }
    } else {
      bad_len = 0.0;
    }
  }
  spot_path_fail_at_ = -1.0;
  spot_path_fail_w_ = -1.0;
  return true;
}

double V2XOvertaker::latestPassAccelDelay(
  const Frame & f, const std::string & name, const OtherState & o,
  double gap, double exit_distance, bool use_boost,
  double pass_start_dist) const
{
  if (exit_distance <= pass_len_ || f.n < 3) { return -1.0; }
  constexpr double dt = 0.1;
  const double self_rank_cap =
    ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
  const bool op_is_npc = o.slot == npc_slot_;
  const double op_slot_cap =
    (op_is_npc ? predict_speed_slow_ : predict_speed_fast_) / 3.6;
  const double op_rank_cap =
    ((!cur_leader_.empty() && name == cur_leader_)
       ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
  const double op_cap = std::max(std::min(op_slot_cap, op_rank_cap), 0.5);
  const double op_mean = o.meanSpeed();
  const double op_now = std::hypot(o.vx, o.vy);
  const std::size_t oi0 = nearest(f.in, o.x, o.y);

  // 前方距離に対応する経路点を単調に進める。各simulationは独立したcursorを持つ。
  auto advance_index = [&](std::size_t start, double wanted, std::size_t & cursor,
                           double & cursor_dist) {
    while (cursor_dist < wanted && cursor + 1 < f.n) {
      const std::size_t a = (start + cursor) % f.n;
      const std::size_t b = (a + 1) % f.n;
      double ds = f.s[b] - f.s[a];
      if (ds < 0.0) { ds += f.total; }
      if (cursor_dist + ds > wanted) { break; }
      cursor_dist += ds;
      ++cursor;
    }
    return (start + cursor) % f.n;
  };

  const double base_accel = std::max(vehicle_accel_ * 0.60, 0.05);
  constexpr double boost_duration = 10.0;
  auto succeeds = [&](double delay) {
    double t = 0.0;
    double sx = 0.0;
    double ox = 0.0;
    double sv = std::max(std::abs(f.ev), 0.0);
    double ov = std::clamp(std::max({op_now, op_mean, 0.5}), 0.5, op_cap);
    std::size_t sc = 0, oc = 0;
    double sd = 0.0, od = 0.0;
    const double max_time = std::min(35.0, exit_distance / 0.5 + 5.0);
    while (t < max_time && sx <= exit_distance) {
      const std::size_t si = advance_index(f.ei, sx, sc, sd);
      const std::size_t oi = advance_index(oi0, ox, oc, od);
      const double self_path_cap = std::max<double>(
        std::min<double>(self_rank_cap,
                         f.in.points[si].longitudinal_velocity_mps), 0.5);
      double learned = oppSpdAt(o, oi, f.n);
      const bool have_learned = learned > 0.0;
      if (!have_learned) { learned = (op_mean > 0.0) ? op_mean : op_now; }
      double op_target = learned * predict_op_margin_;
      if (!have_learned) { op_target = std::max(op_target, op_now * 0.9); }
      op_target = std::clamp(std::max(op_target, 0.5), 0.5, op_cap);
      ov = op_target;

      const double sv0 = sv;
      if (t + 1e-6 >= delay) {
        // ブーストが必要と判定された計画では、実装仕様どおり10秒間の加速度
        // 上乗せも到達可能性へ含める。これが無いと「ブーストなら抜ける」候補を
        // 通常加速だけで再判定して却下し、ブースト自体が永遠に発火しない。
        const double accel = base_accel +
          ((use_boost && t - delay < boost_duration) ? boost_accel_ : 0.0);
        sv = std::min(sv + accel * dt, self_path_cap);
      } else {
        // 待機中は相手速度まで。現在それ以上なら安全側に即時で合わせる。
        sv = std::min({sv + base_accel * dt, self_path_cap, op_target});
      }
      sx += 0.5 * (sv0 + sv) * dt;
      ox += ov * dt;
      t += dt;
      if (sx >= std::max(pass_start_dist, 0.0) &&
          sx - (gap + ox) >= pass_len_) {
        return sx <= exit_distance;
      }
    }
    return false;
  };

  if (!succeeds(0.0)) { return -1.0; }
  const double base_v = std::max(std::min(std::abs(f.ev), op_cap), 0.5);
  double lo = 0.0;
  double hi = std::min(20.0, exit_distance / base_v + 2.0);
  if (succeeds(hi)) { return hi; }
  for (int i = 0; i < 12; ++i) {
    const double mid = 0.5 * (lo + hi);
    if (succeeds(mid)) { lo = mid; }
    else { hi = mid; }
  }
  return lo;
}


void V2XOvertaker::dumpTrace(const Frame & f)
{
  if (!race_started_) { return; }

  {
    const double pinned = 1.38889;
    const bool now_pinned = std::abs(f.ev - pinned) < 0.05;
    if (now_pinned) {
      if (penalty_since_ < 0.0) {
        penalty_since_ = f.now.seconds();
        penalty_at_idx_ = f.ei;
        penalty_near_ = "";
        double bd = 1e18;
        for (const auto & kv : others_) {
          if (!kv.second.valid) { continue; }
          const double d = std::hypot(kv.second.x - f.ex, kv.second.y - f.ey);
          if (d < bd) { bd = d; penalty_near_ = kv.first; }
        }
        penalty_near_dist_ = bd;
      }
    } else if (penalty_since_ >= 0.0) {
      const double dur = f.now.seconds() - penalty_since_;
      if (dur > 1.0) {
        const char * kind = (dur > 7.0) ? "Crash(10s)"
                          : ((dur > 3.5) ? "Wall(5s)" : "Over(2s)");
        RCLCPP_WARN(get_logger(),
          "ペナルティ %s 継続%.1fs idx=%zu 最寄り=%s %.2fm "
          "レース開始から%.1fs %d周目",
          kind, dur, penalty_at_idx_, penalty_near_.c_str(), penalty_near_dist_,
          (launch_since_ >= 0.0) ? (f.now.seconds() - launch_since_) : -1.0,
          lap_ + 1);
      }
      penalty_since_ = -1.0;
    }
  }

  if (telem_sec_ > 0.0 && launch_since_ >= 0.0 &&
      (f.now.seconds() - launch_since_) < telem_sec_ &&
      (telem_at_ < 0.0 || f.now.seconds() - telem_at_ >= 0.5))
  {
    telem_at_ = f.now.seconds();
    std::string oth;
    for (const auto & kv : others_) {
      char b[96];
      double d = kv.second.prog - my_prog_;
      snprintf(b, sizeof(b), " %s(P%d)=%.1fkm/h 差%+.1fm",
               kv.first.c_str(), kv.second.slot,
               std::hypot(kv.second.vx, kv.second.vy) * 3.6, d);
      oth += b;
    }
    RCLCPP_INFO(get_logger(),
      "発進計測 t=%.1fs P%d 自車=%.1fkm/h 上限=%.1fkm/h 横=%.2f%s",
      f.now.seconds() - launch_since_, start_slot_, f.ev * 3.6,
      (last_speed_cap_ < 0.0 ? 99.0 : last_speed_cap_ * 3.6),
      my_lat_for_target_, oth.c_str());
  }

  if (trace_dump_sec_ <= 0.0) { return; }
  if (trace_dump_at_ < 0.0) { trace_dump_at_ = f.now.seconds(); return; }
  if (f.now.seconds() - trace_dump_at_ < trace_dump_sec_) { return; }
  trace_dump_at_ = f.now.seconds();

  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    int known = 0;
    for (int i = 0; i < OtherState::kLatBins; ++i) {
      if (o.laneLat(i) < 1e8) { known++; }
    }
    if (known == 0) { continue; }
    // 24 区間へまとめて出す。256 個そのままではログが読めない。
    std::string lat_s, spd_s;
    const int step = OtherState::kLatBins / OtherState::kSections;
    for (int sec = 0; sec < OtherState::kSections; ++sec) {
      double ls = 0.0, vs = 0.0;
      int lc = 0, vc = 0;
      for (int i = sec * step; i < (sec + 1) * step; ++i) {
        const double l = o.laneLat(i);
        const double v = o.laneSpd(i);
        if (l < 1e8) { ls += l; lc++; }
        if (v > 0.0) { vs += v; vc++; }
      }
      char buf[32];
      snprintf(buf, sizeof(buf), "%s%.2f", sec ? "," : "", lc ? ls / lc : 9.99);
      lat_s += buf;
      snprintf(buf, sizeof(buf), "%s%.1f", sec ? "," : "", vc ? vs / vc * 3.6 : -1.0);
      spd_s += buf;
    }
    RCLCPP_INFO(get_logger(),
      "録画 target=%s P%d 学習点=%d/%d 平均=%.1fkm/h ラップ=%.1fs\n"
      "  横位置[m] %s\n  速度[km/h] %s",
      kv.first.c_str(), o.slot, known, OtherState::kLatBins,
      o.meanSpeed() > 0.0 ? o.meanSpeed() * 3.6 : -1.0, o.last_lap_time,
      lat_s.c_str(), spd_s.c_str());
  }
  // --- 当てはめたモデルのパラメータを残す(検証用) ---
  // これが出ていないとモデルが効いているのか録画が効いているのか分からない。
  if (opp_model_enable_ &&
      (this->now() - last_opp_model_log_).seconds() > 5.0)
  {
    last_opp_model_log_ = this->now();
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || o.m_samples < 5) { continue; }
      const double det = o.m_n * o.m_sxx - o.m_sx * o.m_sx;
      const double bias = (std::abs(det) < 1e-6) ? (o.m_sy / std::max(o.m_n, 1.0))
                          : (o.m_sxx * o.m_sy - o.m_sx * o.m_sxy) / det;
      const double gain = (std::abs(det) < 1e-6) ? 0.0
                          : (o.m_n * o.m_sxy - o.m_sx * o.m_sy) / det;
      RCLCPP_INFO(get_logger(),
        "相手モデル %s: 最高速%.1fkm/h 横加速度の限界%.1fm/s^2 "
        "横の寄り%+.2fm インの取り方%+.2f (サンプル%d 当てはまり%d)",
        kv.first.c_str(), o.m_v_top * 3.6, o.m_ay_max, bias, gain,
        o.m_samples, o.modelReady() ? 1 : 0);
    }
  }
  if (spot_valid_) {
    RCLCPP_INFO(get_logger(),
      "抜きどころ target=%s 側=%s 入口idx%zu まで%.0fm 長さ%.0fm "
      "相手%.1fkm/h %d周目",
      spot_target_.c_str(), (spot_side_ > 0.0) ? "左" : "右",
      spot_begin_, spot_dist_, spot_len_,
      (spot_ospeed_ > 0.0) ? spot_ospeed_ * 3.6 : -1.0, lap_ + 1);
  } else {
    RCLCPP_INFO(get_logger(),
      "抜きどころ なし: 連続区間 左%.0fm 右%.0fm (最低長%.0fm) 学習点%d %d周目 "
      "抜き切るのに要る距離=%.0fm(そのときの区間長%.0fm 相手%.1fkm/h)",
      spot_dbg_l_, spot_dbg_r_, spot_min_len_, spot_dbg_known_, lap_ + 1,
      spot_dbg_need_, spot_dbg_need_len_,
      spot_dbg_need_vo_ > 0.0 ? spot_dbg_need_vo_ * 3.6 : -1.0);
  }
}


void V2XOvertaker::updatePenalty(const Frame & f)
{
  const double t = f.now.seconds();
  auto step = [&](PenState & st, double v, const char * who) {
    const bool near5 = std::abs(v - pen_speed_) <= pen_tol_;
    if (near5) {
      if (st.since < 0.0) { st.since = t; }
      if (!st.on && (t - st.since) >= pen_hold_) {
        st.on = true; st.began = st.since;
        diagWarn("ペナルティ", "ペナルティ推定 開始 車=%s 速度=%.2fkm/h idx=%zu", who, v * 3.6, f.ei);
      }
    } else {
      if (st.on) {
        const double dur = t - st.began;
        const char * kind = (dur < 1.5)  ? "誤検出"
                          : (dur < 3.5)  ? "Over"
                          : (dur < 7.5)  ? "Wall"
                          : (dur < 13.0) ? "Crash" : "不明";
        // 損失距離: その秒数を 5km/h で走る代わりに通常速度で走れた距離との差。
        const double lost = (dur < 1.5) ? 0.0
                          : dur * (std::max(std::abs(f.ev), pen_speed_) - pen_speed_);
        diagWarn("ペナルティ", "ペナルティ推定 終了 車=%s 種別=%s 継続=%.1fs 損失=%.0fm idx=%zu",
          who, kind, dur, lost, f.ei);
      }
      st.on = false; st.since = -1.0;
    }
  };
  step(pen_self_, std::abs(f.ev), "自車");
  for (const auto & kv : others_) {
    if (!kv.second.valid) { continue; }
    step(pen_[kv.first], std::hypot(kv.second.vx, kv.second.vy), kv.first.c_str());
  }
}

bool V2XOvertaker::passAllowedThisLap(const std::string & name, const OtherState & o,
                                      bool clearly_slower) const
{
  (void)name;
  if (leader_pass_last_laps_ > 0 && !clearly_slower &&
      !cur_leader_.empty() && name == cur_leader_ &&
      lap_ < race_laps_ - leader_pass_last_laps_) {
    return false;
  }

  // ここから下は「最初の数周は仕掛けない」ための周回ゲート。
  // 上のハンデの規則とは目的が別なので、こちらだけを無効にできる。
  if (!lap_gate_enable_) { return true; }
  if (!slots_assigned_) { return false; }
  if (clearly_slower && (o.slot == npc_slot_ || lap_ >= record_laps_)) { return true; }
  if (o.slot == npc_slot_) { return true; }        // 運営NPC は常に対象
  if (lap_ < record_laps_) { return false; }       // 1周目は録画に専念
  if (o.slot != npc_slot_ && lap_ < teammate_pass_lap_) { return false; }

  return true;
}

const char * V2XOvertaker::ovStateName(OvState s)
{
  switch (s) {
    case OvState::kFollow:   return "FOLLOW";
    case OvState::kPrepare:  return "PREPARE";
    case OvState::kMoveOut:  return "MOVE_OUT";
    case OvState::kPass:     return "PASS";
    case OvState::kMerge:    return "MERGE";
    case OvState::kCooldown: return "COOLDOWN";
  }
  return "?";
}

void V2XOvertaker::updateOvertakeState(const Frame & f, PlanCtx & c)
{
  const double t = f.now.seconds();
  const OvState prev = ov_state_;
  const std::string prev_target = ov_target_;
  const char * reason = "-";

  // 対象が前方にいるか。planPassSpot の target_ahead と同じ求め方
  // (軌道上の弧長差が 0.5m〜spot_range_)を使う。
  auto target_ahead = [&](const std::string & nm) {
    if (nm.empty()) { return false; }
    const auto it = others_.find(nm);
    if (it == others_.end() || !it->second.valid) { return false; }
    if ((f.now - it->second.stamp).seconds() > v2x_timeout_) { return false; }
    const size_t oi = nearest(f.in, it->second.x, it->second.y);
    double od = f.s[oi] - f.s[f.ei];
    if (od < 0.0) { od += f.total; }
    return od > 0.5 && od <= spot_range_;
  };

  // 「抜き切った」は recordAttempt の passed 判定と同じ式(進行度差が
  // pass_len_ の半分を超え、周回のまたぎでない範囲)を使う。
  auto passed_target = [&](const std::string & nm) {
    if (nm.empty()) { return false; }
    const auto it = others_.find(nm);
    if (it == others_.end() || !it->second.valid) { return false; }
    const double diff = my_prog_ - it->second.prog;
    return diff > pass_len_ * 0.5 && diff < f.total * 0.5;
  };

  if (ovPassing()) {
    if (c.avail_width < band_car_w_) {
      if (ov_narrow_since_ < 0.0) { ov_narrow_since_ = t; }
    } else {
      ov_narrow_since_ = -1.0;
    }
  } else {
    ov_narrow_since_ = -1.0;
  }

  const bool emergency_ttc = false;
  (void)dbg_avoid_cap_;
  (void)wall_push_now_;
  const bool no_room = ov_narrow_since_ >= 0.0 &&
                       (t - ov_narrow_since_) >= spot_abort_sec_;
  // 既存のタイムアウトと停滞判定。recordAttempt が実際に降ろすので、ここでは
  // 「降ろされた/降ろされる」ことを既存の値から読むだけ。
  const bool timeout_abort = attempt_active_ && !ov_target_.empty() &&
                             attempt_target_ == ov_target_ &&
                             (t - attempt_start_) > attempt_timeout_;
  const bool stall_abort = !ov_target_.empty() &&
                           attempt_stall_name_ == ov_target_ &&
                           t < attempt_stall_until_;

  OvState next = ov_state_;
  const bool in_attempt = (ov_state_ == OvState::kPrepare) || ovPassing();
  if (in_attempt &&
      (emergency_ttc || no_room || timeout_abort || stall_abort)) {
    next = OvState::kCooldown;
    prepare_free_active_ = false;
    ov_cooldown_until_ = t + attempt_stall_cool_;
    reason = emergency_ttc ? "緊急TTC"
           : no_room       ? "横の余地なし"
           : timeout_abort ? "時間切れ"
           :                 "停滞";
  } else {
    switch (ov_state_) {
      case OvState::kCooldown:
        if (t >= ov_cooldown_until_) { next = OvState::kFollow; reason = "休止明け"; }
        break;
      case OvState::kFollow:
        if (attempt_active_ && !attempt_target_.empty()) {
          ov_target_ = attempt_target_;
          next = OvState::kMoveOut;
          reason = "試行開始(計画なし)";
        } else if (spot_valid_ && target_ahead(spot_target_)) {
          ov_target_ = spot_target_;
          next = OvState::kPrepare;
          prepare_free_active_ = false;
          reason = "計画あり";
        } else if (!prepare_wish_.empty() && target_ahead(prepare_wish_)) {
          ov_target_ = prepare_wish_;
          next = OvState::kPrepare;
          prepare_free_active_ = true;
          prepare_lost_since_ = -1.0;
          reason = "接近(録画なし)";
        }
        break;
      case OvState::kPrepare:
        if (attempt_active_) {
          if (!attempt_target_.empty()) { ov_target_ = attempt_target_; }
          next = OvState::kMoveOut;
          prepare_free_active_ = false;
          reason = "試行開始";
        } else if (prepare_free_active_) {
          // 録画なしで入った PREPARE は、その相手に対する準備の条件が。
          if (prepare_wish_ == ov_target_) { prepare_lost_since_ = -1.0; }
          else if (prepare_lost_since_ < 0.0) { prepare_lost_since_ = t; }
          const bool hard_gone = prepare_target_seen_ && !prepare_target_hard_ok_;
          const bool lost_long = prepare_lost_since_ >= 0.0 &&
                                 (t - prepare_lost_since_) >= prepare_free_grace_;
          if (!target_ahead(ov_target_) || hard_gone || lost_long) {
            next = OvState::kFollow;
            prepare_free_active_ = false;
            prepare_lost_since_ = -1.0;
            reason = !target_ahead(ov_target_) ? "対象が前にいない"
                   : hard_gone                 ? "側/幅/禁止区間"
                   :                             "準備の条件が消えた";
          }
        } else if (!spot_valid_) {
          next = OvState::kFollow;
          reason = "計画消失";
        }
        break;
      case OvState::kMoveOut:
        if (std::abs(pass_sep_) >= commit_sep_) {
          next = OvState::kPass;
          reason = "横へ出切った";
        } else if (ov_release_stale_ && !target_ahead(ov_target_)) {
          // MOVE_OUT でも同様に、対象が前にいなくなったら解放する。
          next = OvState::kFollow;
          reason = "対象が前にいない";
        }
        break;
      case OvState::kPass:
        if (passed_target(ov_target_)) { next = OvState::kMerge; reason = "抜き切り"; }
        else if (ov_release_stale_ && !target_ahead(ov_target_)) {
          next = OvState::kFollow;
          reason = "対象が前にいない";
        } else if (ov_release_stale_ && !c.blocker.empty() &&
                   c.blocker != ov_target_) {
          next = OvState::kFollow;
          reason = "別の車が前に入った";
        }
        break;
      case OvState::kMerge:
        if (std::abs(pass_sep_) < min_pass_sep_ * 0.3) {
          if (ov_merge_back_since_ < 0.0) { ov_merge_back_since_ = t; }
        } else {
          ov_merge_back_since_ = -1.0;
        }
        if (!attempt_active_) {
          if (ov_attempt_off_since_ < 0.0) { ov_attempt_off_since_ = t; }
        } else {
          ov_attempt_off_since_ = -1.0;
        }
        if (ov_merge_back_since_ >= 0.0 && (t - ov_merge_back_since_) >= 0.5) {
          next = OvState::kFollow;
          reason = "ラインへ復帰";
        } else if (ov_attempt_off_since_ >= 0.0 &&
                   (t - ov_attempt_off_since_) >= 1.0) {
          next = OvState::kFollow;
          reason = "試行終了";
        }
        break;
    }
  }

  if (next != OvState::kMerge) { ov_merge_back_since_ = -1.0; ov_attempt_off_since_ = -1.0; }

  if (next != prev) {
    // 遷移前の対象名で出す(kFollow/kCooldown へ落ちる際に空にするため)。
    const std::string shown = ov_target_.empty() ? prev_target : ov_target_;
    double gap = -1.0;
    {
      const auto it = others_.find(shown);
      if (it != others_.end() && it->second.valid) {
        const size_t oi = nearest(f.in, it->second.x, it->second.y);
        double od = f.s[oi] - f.s[f.ei];
        if (od < 0.0) { od += f.total; }
        gap = od;
      }
    }
    diagLog("追越状態", "追越状態 %s -> %s target=%s 車間=%.1fm 横間隔=%.2fm 理由=%s",
                ovStateName(prev), ovStateName(next),
                shown.empty() ? "-" : shown.c_str(), gap, pass_sep_, reason);
    ov_state_ = next;
    ov_state_since_ = t;
    if (next == OvState::kFollow || next == OvState::kCooldown) {
      ov_target_.clear();
      ov_narrow_since_ = -1.0;
    }
  }
}


bool V2XOvertaker::passUnderway(const std::string & name, double lat_sep) const
{
  const bool underway =
    (attempt_active_ && attempt_target_ == name) ||
    (spot_accel_active_ && spot_accel_target_ == name) ||
    (spot_enable_ && spot_valid_ && spot_target_ == name && spot_in_now_);
  return underway && lat_sep >= pass_side_clear_;
}


// ===================================================================
// 走り方を決める層(追う / 抜く)
// ===================================================================

// 前方の相手を追う / 抜くかを決める中心の層。
// 追従の車間制御・追い越しの可否判定・側の選択・助走ブーストを含む。
void V2XOvertaker::planOvertake(const Frame & f, PlanCtx & c)
{
  {
    const int cur = sidePickZoneIndex(f.ei);
    pick_zone_changed_now_ = (cur >= 0 && cur != side_pick_zone_seen_);
    side_pick_zone_seen_ = cur;
  }
  if (enable_) {
    // 録画なし PREPARE の希望はこの周期の判定だけで決める。持ち越さない。
    prepare_wish_.clear();
    prepare_wish_gap_ = 1e18;
    prepare_target_seen_ = false;
    prepare_target_hard_ok_ = true;
    for (const auto & kv : others_) {
      evaluateOpponent(f, c, kv.first, kv.second);
    }
    // 前方車が1台も見つからなかった周期では、追い越し状態も明示的に落とす。
    // これらは前方車のループの中でしか更新しないので、V2X が切れて相手が
    // 消えると最後の値のまま凍り、誰もいないのに横オフセット約1.9m を
    // attempt_timeout(16秒)まで保持し続けていた。
    // ただし試行中は落とさない。抜き切った相手は前方車の集合から消えるので、
    // 「成功した瞬間に blocker が空になり pass_sep_ が 0 になる」。
    // 下の記録では失敗条件(|pass_sep_| 小)が先に成立するため、
    // 成功した追い越しがそのまま失敗として記録されていた。
    if (c.blocker.empty()) {
      // 相手が居ない周期では抜き切りモードのヒステリシスも落とす。
      // 残したままだと、次に別の相手へ近づいたとき緩い側の閾値で始まる。
      commit_now_ = false;
    }
    if (c.blocker.empty() && !attempt_active_) {
      pass_sep_ = 0.0;
      can_pass_now_ = false;
    }

    if (attempt_lat_hold_enable_ && attempt_active_ && attempt_lat_valid_ &&
        !attempt_lat_fresh_) {
      c.requestLat(attempt_lat_, PlanCtx::LatPrio::kOvertake, "追越");
      if ((f.now - last_lat_hold_log_).seconds() > 2.0) {
        last_lat_hold_log_ = f.now;
        diagLog("追越保持", "追越保持 target=%s 横目標=%.2fm "
                "(前方車から外れたが試行中なので保持)",
                attempt_target_.c_str(), attempt_lat_);
      }
    }
    attempt_lat_fresh_ = false;
  }
}

// 他車 1 台を評価する。段は 5 つ。
//
//   isNearestBlocker  いま自分の前をふさぐ最も近い1台か(違えば打ち切り)
//   chooseSide        どちら側から抜くか
//   decideAllow       抜きにいってよいか(判断材料を OppEval に畳み込む)
//   chargeBoost       助走ブーストを撃つか
//   followAndCommit   車間制御と抜き切り
//
// 前半 3 段が観測から OppEval を作り、後半 2 段がそれを使って指令を出す。
void V2XOvertaker::evaluateOpponent(const Frame & f, PlanCtx & c,
                      const std::string & name, const OtherState & o)
{
  size_t oi = 0;
  double gap = 0.0;
  if (!isNearestBlocker(f, c, name, o, oi, gap)) { return; }

  double olat = 0.0;
  double ospeed_for_gate = 0.0;
  chooseSide(f, c, o, oi, olat, ospeed_for_gate);

  OppEval ev;
  decideAllow(f, c, name, o, oi, gap, olat, ospeed_for_gate, ev);

  // isNearestBlocker は近い相手が見つかるたび blocker を更新するため、許可証も
  // 必ず同時に置き換える。これにより、先に評価した遠い相手の許可や横意図を
  // 後から見つかった目前車へ流用できない。false のとき明示的に消すことが重要。
  c.pass_authorized_target = ev.allow ? name : std::string{};

  chargeBoost(f, c, ev);
  followAndCommit(f, c, o, ev);
}

// この相手が「いま自分の前をふさいでいる最も近い1台」かを判定する。
//
// 進行度差だけで前後を判定すると、スタートのグリッドのように横に並んだ車が
// 「差ほぼ0」で前車として検出されず、速度制限が掛からないまま追突する。
// 車体の向きから見た実際の前方距離でも判定し、小さい方を車間として採る。
//
// 該当すれば c.best_gap / c.blocker を更新して true。そうでなければ false
// (呼び出し側はこの相手の評価を打ち切る)。
bool V2XOvertaker::isNearestBlocker(const Frame & f, PlanCtx & c, const std::string & name,
                      const OtherState & o, size_t & oi, double & gap)
{
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const double total = f.total;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  if (!o.valid) {
    return false;
  }
  if ((now - o.stamp).seconds() > v2x_timeout_) {
    return false;   // 情報が古い。信用しない
  }
  oi = nearest(in, o.x, o.y);
  gap = s[oi] - s[ei];
  if (gap < 0) {
    gap += total;      // 周回をまたぐ
  }
  // 進行度差だけで前後を判定すると、スタートのグリッドのように
  // 横に並んでいる車が「差ほぼ0」となって前車として検出されず、
  // 速度制限が掛からないまま加速して追突する。
  // 車体の向きから見た実際の前方距離でも判定する。
  double fwd_real = 1e9;
  {
    const auto & qq = odom_->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (qq.w * qq.z + qq.x * qq.y),
                                  1.0 - 2.0 * (qq.y * qq.y + qq.z * qq.z));
    const double dxr = o.x - ex, dyr = o.y - ey;
    const double f = dxr * std::cos(yaw) + dyr * std::sin(yaw);
    const double sd = std::abs(-dxr * std::sin(yaw) + dyr * std::cos(yaw));
    if (f > 0.0 && sd < front_lane_half_) {
      fwd_real = f;      // 自分の進路上の前方にいる
    }
  }
  const bool ahead_by_prog = (gap > 0.5 && gap < detect_range_);
  const bool ahead_by_geom = (fwd_real < detect_range_);
  if (!ahead_by_prog && !ahead_by_geom) {
    return false;
  }
  // 距離は小さい方を採用する(横並びでも実距離で反応できる)
  gap = std::min(gap, fwd_real);
  if (gap >= c.best_gap) {
    return false;
  }
  c.best_gap = gap;
  c.blocker = name;
  return true;
}

// どちら側から抜くかを決める。
double V2XOvertaker::stoppedPad() const
{
  return std::max(std::max(stopped_size_pad_, size_pad_), 0.0);
}

double V2XOvertaker::occupiedHalfWidth(std::size_t idx, bool pad) const
{
  double h = occupied_real_width_ ? geom_half_width_ * 2.0 : kCarWidth;
  if (pad) { h += std::max(size_pad_, 0.0); }
  if (occupied_yaw_pad_ && corridor_.radius.size() > idx) {
    const double R = corridor_.radius[idx];
    if (R > 0.5 && R < 1e6) {
      const double L = geom_front_ + geom_rear_;   // 全長 2.064m
      h += 2.0 * (L * L / (8.0 * R));              // 自車と相手のぶん
    }
  }
  return h;
}

int V2XOvertaker::completionSide(
  const Frame & f, const OtherState & o, double pass_dist,
  double need_sep, double olat_now, double & room_l, double & room_r) const
{
  room_l = -1e9; room_r = -1e9;
  audit_used_set_ = false;
  audit_used_olat_ = 9.99;
  if (!side_by_completion_) { return 0; }
  if (!std::isfinite(pass_dist) || pass_dist <= 1.0) { return 0; }
  const std::size_t n = f.n;
  if (n == 0 || corridor_.lo.size() != n || corridor_.hi.size() != n) { return 0; }

  v2x_overtaker::PassSideInput in;
  in.pass_dist = pass_dist;
  in.need_sep = need_sep;
  in.window_ratio = std::clamp(side_completion_window_, 0.0, 1.0);
  if (side_plan_enable_) {
    in.window_ratio = std::clamp(side_plan_window_, 0.0, 1.0);
  }
  in.plan_mode = side_plan_enable_;
  in.y_now = my_lat_for_target_;
  in.v_ego = std::max(std::abs(my_speed_for_gap_), 1.0);
  in.lat_rate = std::max(offset_rate_, 0.05);
  in.reach_gain = side_plan_reach_gain_;
  in.room_margin = side_plan_room_margin_;
  // 経路の追従可能性。
  in.ay_max = side_plan_ay_max_;
  in.curve_sign = (curve_sign_ > 0.0) ? +1 : ((curve_sign_ < 0.0) ? -1 : 0);

  // 標本を pass_dist で打ち切っていた。
  double search_limit = std::max(side_window_search_m_, 0.0);
  // 窓の**後端**をいまいる直線の中に収める。
  if (side_zone_end_clamp_) {
    const double zleft = distToPassFinishZoneEnd(f);
    if (zleft > 0.0) { in.zone_end_s = zleft; }
  }
  in.plan_full_scale = side_plan_full_scale_;
  if (side_search_in_zone_) {
    const double straight_left = distToPassFinishZoneEnd(f);
    if (straight_left > 5.0) {
      // 横に寄り切るのに要る距離。ここまでは必ず探索させる。
      const double v_now = std::max(std::abs(my_speed_for_gap_), 3.0);
      const double d_lat = std::abs(need_sep) / std::max(offset_rate_, 0.05) * v_now;
      // 窓の**始点**を直線の中に収める(後端ではなく)。
      search_limit = std::min(search_limit, std::max(straight_left, d_lat));
    }
  }
  // 標本は探索する距離ぶん先まで作る。上で絞ったのと同じ値を使う
  // (別々にすると、標本はあるのに探索しない/探索するのに標本が無い、が起きる)。
  const double sample_span = pass_dist + (side_window_search_ ? search_limit : 0.0);
  std::vector<std::size_t> sample_idx;   // in.samples[k] の地点の軌道 idx
  double acc = 0.0;
  for (std::size_t k = 1; k < n && acc <= sample_span; ++k) {
    const std::size_t a = (f.ei + k - 1) % n, b = (f.ei + k) % n;
    acc += std::hypot(
      f.in.points[b].pose.position.x - f.in.points[a].pose.position.x,
      f.in.points[b].pose.position.y - f.in.points[a].pose.position.y);
    if (acc > sample_span) { break; }
    v2x_overtaker::PassSideSample sm;
    sm.s = acc;
    sample_idx.push_back(b);   // 距離 acc に対応する軌道 idx(計画の入口を idx で持つため)
    // safetyAt はその地点で壁から取りたい余裕。コリドアをそのぶん内側へ寄せる。
    const double safe = safetyAt(b);
    sm.lo = corridor_.lo[b] + safe;
    sm.hi = corridor_.hi[b] - safe;
    // その地点の曲率半径。追従可能性(横加速度)の判定に使う。
    sm.radius_m = (corridor_.radius.size() == n) ? corridor_.radius[b] : 0.0;
    // 相手の横位置は。
    double opl = 1e9;
    if (lat_pred_mode_ == 0) {
      const double v = oppLatAt(o, b, n);
      if (std::isfinite(v) && std::abs(v) < 1e7) { opl = v; }
    } else if (std::isfinite(olat_now) && std::abs(olat_now) < 6.0) {
      opl = (lat_pred_mode_ == 2)
              ? olat_now * std::exp(-sm.s / std::max(band_lat_tau_, 1.0))
              : olat_now;
    }
    if (opl < 1e8) {
      sm.opp_lat = opl;
      sm.opp_known = true;
      // 監査ログの「相手横」と、この関数が実際に使う値が。
      if (!audit_used_set_) { audit_used_olat_ = opl; audit_used_set_ = true; }
    }
    in.samples.push_back(sm);
  }
  // 固定位置の窓で「どちらも通れない」なら、。
  double found_at = -1.0;
  const auto r = side_window_search_
    ? v2x_overtaker::passCompletionSideSearch(in, search_limit, 4.0, found_at)
    : v2x_overtaker::passCompletionSide(in);
  room_l = r.room_left;
  room_r = r.room_right;
  audit_found_at_ = found_at;
  audit_win_from_ = r.s_from;
  audit_win_to_ = r.s_to;
  audit_win_used_ = r.used;
  audit_win_scale_ = r.scale;
  audit_win_span_ = in.samples.empty() ? 0.0 : in.samples.back().s;
  audit_plan_start_ = r.start_s;
  audit_plan_y_ = r.y_target;
  audit_plan_feas_ = (r.feas_left ? 1 : 0) | (r.feas_right ? 2 : 0);
  audit_early_l_ = r.earliest_left;
  audit_early_r_ = r.earliest_right;
  audit_rej_l_[0] = r.rej_room_l; audit_rej_l_[1] = r.rej_band_l;
  audit_rej_l_[2] = r.rej_reach_l; audit_rej_l_[3] = r.rej_ay_l;
  audit_rej_r_[0] = r.rej_room_r; audit_rej_r_[1] = r.rej_band_r;
  audit_rej_r_[2] = r.rej_reach_r; audit_rej_r_[3] = r.rej_ay_r;
  audit_cs_fresh_ = 1;
  // 相手の観測が何秒前か。ログの古さと実際のずれの取り違えを防ぐ。
  audit_opp_age_ = (o.valid && o.stamp.nanoseconds() > 0)
                     ? (f.now - o.stamp).seconds() : -1.0;
  audit_geom_ = 0;
  int side_out = r.valid ? r.side : 0;

  plan_start_valid_ = false;
  if (side_out != 0 && r.reach_ok && r.start_s >= 0.0 && !sample_idx.empty()) {
    // start_s にいちばん近い標本の idx を採る。
    std::size_t k_best = 0;
    double d_best = 1e18;
    for (std::size_t k = 0; k < in.samples.size() && k < sample_idx.size(); ++k) {
      const double d = std::abs(in.samples[k].s - r.start_s);
      if (d < d_best) { d_best = d; k_best = k; }
    }
    plan_start_idx_ = sample_idx[k_best];
    plan_y_target_ = r.y_target;
    plan_start_valid_ = true;
  }

  if (side_out == 0 && side_zone_geom_default_ && in.plan_mode && !in.samples.empty()) {
    // 直線の残りが抜き切り距離に足りないなら、**幾何の既定も出さない**。
    const double end_s = in.pass_dist;
    const bool fits = (in.zone_end_s <= 0.0) || (in.zone_end_s + 1e-6 >= in.pass_dist);
    if (fits && end_s >= side_geom_min_m_) {
      const double from_s = std::max(end_s - side_geom_tail_m_, 0.0);
      double g_left = 1e9, g_right = 1e9;
      int used = 0;
      for (const auto & sm : in.samples) {
        if (sm.s < from_s || sm.s > end_s) { continue; }
        ++used;
        // 相手は見ない。**走行ラインを境にどちらがどれだけ開いているか**だけ。
        g_left = std::min(g_left, sm.hi);
        g_right = std::min(g_right, -sm.lo);
      }
      if (used > 0) {
        const double margin = std::max(side_plan_room_margin_, 0.0);
        const double pick = std::max(g_left, g_right);
        if (pick > margin) {
          int gs = (g_left > g_right) ? +1 : -1;
          // 幾何が選んだ側に**相手がいる**なら、そこへは行かない。
          if (in.samples.front().opp_known) {
            const double ol = in.samples.front().opp_lat;
            const bool opp_on_pick = (gs > 0) ? (ol > in.need_sep * 0.5)
                                              : (ol < -in.need_sep * 0.5);
            if (opp_on_pick) {
              const double other = (gs > 0) ? g_right : g_left;
              if (other > margin) { gs = -gs; }
              else { gs = 0; }
            }
          }
          if (gs == 0) { return 0; }
          side_out = gs;
          // 呼び出し側の門(`rl/rr >= room_need`)が同じ値で判断できるよう、
          // 空きもこの幾何の値にそろえる。片方だけ別の出所にすると、
          // 「側は幾何・空きは計画」というちぐはぐな組合せになる。
          room_l = g_left;
          room_r = g_right;
          audit_geom_ = 1;
        }
      }
    }
  }
  return side_out;
}

void V2XOvertaker::chooseSide(const Frame & f, PlanCtx & c, const OtherState & o,
                size_t oi, double & olat, double & ospeed_for_gate)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // 相手の横位置（ライン基準の符号付き）
  double nx, ny;
  normalAt(in, oi, nx, ny);
  const auto & lp = in.points[oi].pose.position;
  olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;

  // 前車の進行方向速度を先に求める(抜くかどうかの判定に使う)
  
  {
    const auto & a2 = in.points[(oi + n - 1) % n].pose.position;
    const auto & b2 = in.points[(oi + 1) % n].pose.position;
    double tx2 = b2.x - a2.x, ty2 = b2.y - a2.y;
    const double l2 = std::hypot(tx2, ty2);
    if (l2 > 1e-9) {
      tx2 /= l2;
      ty2 /= l2;
    }
    ospeed_for_gate = o.vx * tx2 + o.vy * ty2;
  }

  const double reach_left = std::min(olat + pass_gap_, room_hi_);
  const double reach_right = std::max(olat - pass_gap_, room_lo_);
  bool fit_left = (reach_left - olat) >= min_pass_sep_;
  bool fit_right = (olat - reach_right) >= min_pass_sep_;
  double map_left = 0.0, map_right = 0.0;
  int map_known = 0;
  double room_l_mean = 0.0, room_r_mean = 0.0;
  // 窓が「区間の終わり」で打ち切られたか。打ち切られた窓は短いので、
  // 点数の門(lane_map_min_pts_=8)をそのまま当てると発火しなくなる。
  bool map_zone_clipped = false;
  if (lane_map_side_) {
    // 窓は相手の idx を起点にするので、**その起点が属する区間**で打ち切る。
    // 区間を跨いで平均すると、開いている側が反転する地形で符号が消える。
    sideRoomMap(o, in, n, oi, lane_map_stretch_, olat, map_left, map_right, map_known,
                &room_l_mean, &room_r_mean,
                side_zone_mean_split_ ? sidePickZoneIndex(oi) : -1, &map_zone_clipped);
    // 学習データが区間の大半にある場合だけ信用する。
    // map_left/right は「足りている区間が連続で何m続くか」。
    // 抜き切るのに要る長さ(pass_len)を満たしていれば、その側は成立。
    const double need = pass_len_ * lane_map_need_gain_;
    if (mapKnownOk(map_known, map_zone_clipped)) {
      fit_left = fit_left || (map_left >= need);
      fit_right = fit_right || (map_right >= need);
    }
  }
  dbg_map_l_ = map_left; dbg_map_r_ = map_right; dbg_map_n_ = map_known;
  dbg_room_l_ = room_l_mean; dbg_room_r_ = room_r_mean;
  const int cur_pick_zone = sidePickZoneIndex(ei);
  // 遷移の判定は planOvertake の先頭で毎周期おこなっている。ここでは読むだけ。
  const bool pick_zone_changed =
    side_zone_repick_ && pick_zone_changed_now_ && !attempt_active_;
  const bool side_committed = side_commit_ && side_committed_ &&
                              side_commit_target_ == c.blocker;
  const bool recheck_side = side_recheck_ && !attempt_active_ && !side_committed &&
    (now.seconds() - side_decided_at_) >= side_recheck_sec_;
  ot_lane_side_ = ot_lane_side_right_ &&
    otLaneApproach(ei, my_speed_for_gap_, rankSpeedCap());
  no_pass_side_ = false;
  no_pass_side_target_.clear();
  const bool ot_lane_keep = ot_lane_side_ && ot_lane_side_sticky_;
  // レーン区間に入ったら、その場で側を右にする。
  if (ot_lane_keep && !attempt_active_ && side_sign_ > 0.0) {
    side_sign_ = -1.0;
    side_decided_at_ = now.seconds();
    side_unfit_since_ = -1.0;
    if ((now - last_ot_lane_keep_log_).seconds() > 2.0) {
      last_ot_lane_keep_log_ = now;
      diagLog("レーンで右へ",
              "レーンで右へ 側を左から右にする target=%s idx=%zu 自車%.1fkm/h",
              c.blocker.c_str(), ei, my_speed_for_gap_ * 3.6);
    }
  }
  if (c.blocker != side_blocker_ || pick_zone_changed || recheck_side) {
    if (c.blocker != side_blocker_) { side_zone_repicked_ = false; }
    else {
      side_zone_repicked_ = true;
      RCLCPP_INFO(get_logger(),
        "側を決め直す(区間が変わった) target=%s 区間=%d idx=%zu",
        c.blocker.c_str(), cur_pick_zone, ei);
    }
    side_blocker_ = c.blocker;
    side_decided_at_ = now.seconds();
    side_flip_cnt_ = 0;           // 対象車が変わったら側の変更枠を戻す
    side_flip_at_ = now.seconds();
    side_unfit_since_ = -1.0;
    side_other_fit_since_ = -1.0;
    // 側の優先順位: イン > 相手の反対側。
    const char * side_src = "既定(相手の反対)";
    // 抜き切り地点の空きで決めたか。現在地の都合で反転させないために使う。
    bool & want_from_completion = want_from_completion_;
    bool & completion_room_ok = completion_room_ok_;
    want_from_completion = false;
    completion_room_ok = false;
    double mlat_dbg = 9.99;   // 決定時ログ用(mlat は内側の block にしかない)
    // --- 既定の側 ---。
    double want;
    if (side_default_run_ && std::abs(map_left - map_right) > side_run_tie_) {
      want = (map_right > map_left) ? -1.0 : +1.0;
      side_src = "既定(通せる連続長)";
    } else {
      want = (olat >= 0.0) ? -1.0 : +1.0;
    }
    if (side_default_by_room_) {
      const double room_l = room_hi_;            // 左へ出られる量
      const double room_r = -room_lo_;           // 右へ出られる量
      if (std::isfinite(room_l) && std::isfinite(room_r) &&
          std::abs(room_l - room_r) > side_default_room_hyst_) {
        const double by_room = (room_l > room_r) ? +1.0 : -1.0;
        if (by_room != want) { side_src = "既定(出られる側)"; }
        want = by_room;
      }
    }
    bool by_room = false;
    if (lane_map_side_ && inSidePickZone(ei)) {
      int zknown = 0;
      double mlat = zoneMeanLat(o, n, zknown, sidePickZoneIndex(ei));
      const double mlat_rec = mlat;
      if (side_meanlat_live_ && std::isfinite(olat) && std::abs(olat) < 6.0) {
        mlat = olat;
        zknown = std::max(zknown, 1);
      }
      if (std::abs(mlat_rec) < 1e8 &&
          (this->now() - last_meanlat_log_).seconds() > 3.0) {
        last_meanlat_log_ = this->now();
        diagLog("相手の横",
                "相手の横 モデル=%.2fm 実測=%.2fm 差=%.2fm 採用=%.2fm idx=%zu target=%s",
                mlat_rec, olat, olat - mlat_rec, mlat, ei, c.blocker.c_str());
      }
      mlat_dbg = mlat;
      if (mlat < 1e8) {
        // 判断の基準を「レースラインからのずれ」から。
        if (side_pick_by_room_ && side_pick_by_run_ &&
            mapKnownOk(map_known, map_zone_clipped)) {
          side_src = "連続長";
          want = (map_right > map_left + side_pick_tie_) ? -1.0
               : (map_left > map_right + side_pick_tie_) ? +1.0
               :                                           -1.0;
        } else if (side_pick_by_room_ && mapKnownOk(map_known, map_zone_clipped)) {
          side_src = "区間平均";
          want = (room_r_mean > room_l_mean + side_pick_tie_) ? -1.0   // 右が空く -> 右から
               : (room_l_mean > room_r_mean + side_pick_tie_) ? +1.0   // 左が空く -> 左から
               :                                                -1.0;  // 同じくらい -> 右から
        } else {
          if (side_room_first_ &&
              std::abs(map_left - map_right) > side_run_tie_) {
            want = (map_right > map_left) ? -1.0 : +1.0;
            side_src = "相手と壁の空き(連続長)";
          } else {
          side_src = "相手の区間平均横";
          want = (mlat > side_pick_tie_) ? -1.0        // 相手が左 -> 右から
               : (mlat < -side_pick_tie_) ? +1.0       // 相手が右 -> 左から
               :                            -1.0;      // 同じぐらい -> 右から
          }
          if (side_fallback_room_) {
            const double need = std::max(side_run_need_, 1.0);
            const bool l_ok = map_left  >= need;
            const bool r_ok = map_right >= need;
            if (r_ok && !l_ok && want > 0.0) {
              want = -1.0; side_src = "相手の横だが右しか通せない";
            } else if (l_ok && !r_ok && want < 0.0) {
              want = +1.0; side_src = "相手の横だが左しか通せない";
            }
          }
        }
        by_room = true;
        if ((this->now() - last_wallpick_log_).seconds() > 3.0) {
          last_wallpick_log_ = this->now();
          RCLCPP_INFO(get_logger(),
            "側を録画で決定 target=%s 側=%s 相手の区間平均横=%.2fm(点%d) "
            "空き 左%.2fm 右%.2fm 連続長 左%.1fm 右%.1fm idx=%zu",
            c.blocker.c_str(), (want > 0.0) ? "左" : "右", mlat, zknown,
            room_l_mean, room_r_mean, map_left, map_right, ei);
        }
      }
    }
    const bool ot_lane_side = ot_lane_side_;   // 上で毎周期更新している
    if (ot_lane_side) { want = -1.0; side_src = "レーンで右固定"; }
    const bool in_right_zone = by_room || ot_lane_side;   // 録画で決めた側は曲率より優先する
    // 上のコメント「録画で決めた側は曲率より優先する」が。
    const bool curve_may_override =
      curve_side_enable_ &&
      (curve_side_in_zone_ || !inSidePickZone(ei)) &&
      !ot_lane_side && !(side_pick_over_curve_ && in_right_zone);
    if (curve_may_override && curve_sign_ != 0.0) {
      const double inside_sign = (curve_sign_ > 0.0) ? +1.0 : -1.0;
      const bool inside_fit = (inside_sign > 0.0) ? fit_left : fit_right;
      if (inside_fit) { want = inside_sign; side_src = "曲率のイン優先"; }
    }
    const double side_spot_gate = spotGateDistance(f, o);
    if (spot_enable_ && spot_valid_ && c.blocker == spot_target_ &&
        spot_dist_ <= side_spot_gate && spot_side_ != 0.0) {
      const bool spot_fit = (spot_side_ > 0.0) ? fit_left : fit_right;
      if (spot_fit) { want = spot_side_; side_src = "抜きどころ"; }
    }
    const bool spot_controls_side = spot_enable_ && spot_valid_ &&
      c.blocker == spot_target_ &&
      (spot_dist_ <= side_spot_gate ||
       (attempt_active_ && attempt_target_ == spot_target_));
    if (band_side_lead_ && band_enable_ && band_predict_ && !spot_controls_side) {
      const int bs = bandSide(c.blocker);
      if (bs != 0) {
        const bool bfit = (bs > 0) ? fit_left : fit_right;
        if (bfit) { want = (bs > 0) ? +1.0 : -1.0; side_src = "予測バンド"; }
      }
    }
    if (side_by_completion_ && !spot_controls_side && !ot_lane_side) {
      double rl = -1e9, rr = -1e9;
      const double need_here = std::max(
        std::max(min_pass_sep_, veh_half_width_ * 2.0),
        side_need_rear_free_ ? rear_end_free_min_ : 0.0);
      // 抜き切るまでに自車が走る距離[m]。
      const double v_cap = std::max(rankSpeedCap(), std::max(f.ev, 3.0));
      const double rel_need = pass_len_ + (geom_front_ + geom_rear_) * 2.0;
      double v_potential = v_cap;
      double closing_for_side = std::max(v_cap - ospeed_for_gate, 1.0);
      double t_pass_side = rel_need / closing_for_side;
      double pass_len_for_side = std::clamp(v_potential * t_pass_side, 10.0, 120.0);

      if (pass_dist_accel_) {
        const double v0 = std::max(std::abs(f.ev), 0.5);
        const double vo = std::max(ospeed_for_gate, 0.0);
        // 相対距離 rel_need を稼ぐのに要る時間と、その間に自車が進む距離。
        auto solve = [&](double a)->std::pair<double,double> {
          const double t_to_cap = (v_cap > v0 && a > 1e-6) ? (v_cap - v0) / a : 0.0;
          // 加速し切るまでに稼げる相対距離
          const double rel_at_cap =
            (v0 - vo) * t_to_cap + 0.5 * a * t_to_cap * t_to_cap;
          double t;
          if (rel_at_cap >= rel_need) {
            // 加速の途中で抜き切れる。 (v0-vo)t + a t^2/2 = rel_need を解く
            const double b = v0 - vo;
            const double disc = b * b + 2.0 * a * rel_need;
            t = (a > 1e-6) ? (-b + std::sqrt(std::max(disc, 0.0))) / a
                           : rel_need / std::max(b, 0.1);
          } else {
            // 上限に達した後は一定の closing で残りを稼ぐ
            const double rest = rel_need - rel_at_cap;
            const double cl = std::max(v_cap - vo, 0.1);
            t = t_to_cap + rest / cl;
          }
          if (!std::isfinite(t) || t <= 0.0) { t = rel_need / std::max(v_cap - vo, 0.1); }
          // その間に自車が進む距離
          const double t_acc = std::min(t, t_to_cap);
          const double d = v0 * t + 0.5 * a * t_acc * t_acc + v_cap * std::max(t - t_to_cap, 0.0)
                           - v_cap * 0.0;
          return {t, std::max(d, 1.0)};
        };
        const double a_plain = std::max(vehicle_accel_ * 0.60, 0.05);
        const double a_boost = a_plain + std::max(boost_accel_, 0.0);
        const auto plain = solve(a_plain);
        const auto boost = solve(a_boost);
        // 直線の残りに収まるか(この先で上書きされる straight_left と同じ考え方)
        pass_need_boost_ = false;
        double use_d = plain.second, use_t = plain.first;
        const double room = distToPassFinishZoneEnd(f);
        if (room > 5.0 && plain.second > room && boost.second <= room &&
            boost_remaining_ > 0) {
          // ブーストを使えば直線の中で抜き切れる。使う。
          pass_need_boost_ = true;
          use_d = boost.second; use_t = boost.first;
        }
        t_pass_side = use_t;
        pass_len_for_side = std::clamp(use_d, 10.0, 120.0);
        closing_for_side = std::max(rel_need / std::max(use_t, 0.1), 0.1);
        if ((f.now - last_pass_accel_log_).seconds() > 2.0) {
          last_pass_accel_log_ = f.now;
          diagLog("抜き切り",
            "抜き切り(加速込み) 自車%.1f 相手%.1f 上限%.1fkm/h 直線の残り%.0fm | "
            "ブーストなし %.0fm(%.1fs) / あり %.0fm(%.1fs) -> 採用%.0fm %s | "
            "従来式なら %.0fm",
            std::abs(f.ev) * 3.6, ospeed_for_gate * 3.6, v_cap * 3.6, room,
            plain.second, plain.first, boost.second, boost.first, use_d,
            pass_need_boost_ ? "**ブーストを使う**" : "ブーストなし",
            std::clamp(v_cap * (rel_need / std::max(v_cap - ospeed_for_gate, 1.0)), 10.0, 120.0));
        }
      }
      // 窓の長さに下限を入れる。
      pass_len_for_side = std::max(pass_len_for_side, side_window_min_m_);
      const double straight_left = distToPassFinishZoneEnd(f);
      if (straight_left > 5.0) {
        pass_len_for_side = std::min(pass_len_for_side, straight_left);
      }
      bool ideal_decided = false;
      if (side_run_decide_ && mapKnownOk(map_known, map_zone_clipped)) {
        const double need_run = pass_len_for_side;
        const bool run_l_ok = (map_left  >= need_run);
        const bool run_r_ok = (map_right >= need_run);
        if (run_l_ok || run_r_ok) {
          // 片側だけ足りるならその側。両方足りるなら長いほう。
          const double pick = (run_l_ok && run_r_ok)
                                ? ((map_left > map_right) ? +1.0 : -1.0)
                                : (run_l_ok ? +1.0 : -1.0);
          want = pick;
          side_src = "隙間が続く距離>=抜き切り距離";
          want_from_completion = true;
          ideal_decided = true;
          if ((f.now - last_run_decide_log_).seconds() > 2.0) {
            last_run_decide_log_ = f.now;
            diagLog("側の決定",
              "側=%s 隙間が続く距離 左%.1fm 右%.1fm >= 抜き切り%.1fm "
              "(相手と壁の間に %.2fm 以上の隙間) idx=%zu target=%s",
              pick > 0 ? "左" : "右", map_left, map_right, need_run,
              min_pass_sep_, ei, c.blocker.c_str());
          }
        } else if ((f.now - last_run_short_log_).seconds() > 2.0) {
          last_run_short_log_ = f.now;
          diagLog("側の決定",
            "隙間が続く距離が不足 左%.1fm 右%.1fm < 抜き切り%.1fm idx=%zu target=%s",
            map_left, map_right, need_run, ei, c.blocker.c_str());
        }
      }
      const int cs = completionSide(f, o, pass_len_for_side, need_here, olat, rl, rr);
      // 計画の「抜き始める地点」は誰に対するものか。助走がこれで照合する。
      if (plan_start_valid_) { plan_start_target_ = c.blocker; }
      {
        audit_olat_ = olat;
        audit_pass_len_ = pass_len_for_side;
        audit_room_l_ = rl;
        audit_room_r_ = rr;
        audit_room_target_ = c.blocker;
        // 相手の**生の座標**と最近傍点の idx。
        audit_opp_x_ = o.x;
        audit_opp_y_ = o.y;
        audit_opp_idx_ = static_cast<int>(oi);
      }
      if (no_pass_attempt_hold_ && inSidePickZone(ei) && !attempt_active_ &&
          rl > -1e8 && rr > -1e8 &&        // 値が取れているときだけ
          rl < need_here && rr < need_here) {
        no_pass_side_ = true;
        // chooseSide は相手ごとに呼ばれ、この値は毎周期上書きされる。
        no_pass_side_target_ = c.blocker;
      }
      if (cs != 0) {
        const bool cfit_now = (cs > 0) ? fit_left : fit_right;
        const double room_need = (pass_need_room_m_ >= 0.0) ? pass_need_room_m_ : need_here;
        const bool cfit_ahead = (cs > 0) ? (rl >= room_need) : (rr >= room_need);
        int cs_use = cs;
        bool cfit = cfit_now || cfit_ahead;
        if (pass_need_room_) {
          const double room_cs = (cs > 0) ? rl : rr;
          const double room_op = (cs > 0) ? rr : rl;
          if (room_cs >= room_need) {
            cfit = true;                       // 選んだ側で足りる
          } else if (room_op >= room_need) {
            cs_use = -cs;                      // 反対側なら足りる
            cfit = true;
            if ((f.now - last_side_swap_log_).seconds() > 2.0) {
              last_side_swap_log_ = f.now;
              ++side_swap_n_;
              diagLog("側の入れ替え",
                      "側の入れ替え 累計%zu %s は空き%.2fm で必要%.2fm に足りず "
                      "%s(空き%.2fm)にする idx=%zu",
                      side_swap_n_, cs > 0 ? "左" : "右", room_cs, need_here,
                      cs_use > 0 ? "左" : "右", room_op, ei);
            }
          } else {
            const double need_run = pass_len_for_side;
            const bool run_l_ok = (map_left  >= need_run);
            const bool run_r_ok = (map_right >= need_run);
            if (side_run_decide_ && (run_l_ok || run_r_ok)) {
              // 片側だけ足りるならその側。両方足りるなら長いほう。
              const double pick = (run_l_ok && run_r_ok)
                                    ? ((map_left > map_right) ? +1.0 : -1.0)
                                    : (run_l_ok ? +1.0 : -1.0);
              want = pick;
              cs_use = static_cast<int>(pick);
              cfit = true;
              want_from_completion = true;
              completion_room_ok = true;
              side_src = "隙間が続く距離>=抜き切り距離";
              if ((f.now - last_run_decide_log_).seconds() > 2.0) {
                last_run_decide_log_ = f.now;
                diagLog("側の決定",
                  "側=%s 隙間が続く距離 左%.1fm 右%.1fm >= 抜き切り%.1fm "
                  "(相手と壁の間に %.2fm 以上の隙間) idx=%zu target=%s",
                  pick > 0 ? "左" : "右", map_left, map_right, need_run,
                  min_pass_sep_, ei, c.blocker.c_str());
              }
            } else {
            no_pass_side_ = no_pass_attempt_hold_ && inSidePickZone(ei) &&
                            !attempt_active_;
            }
            // 「もう抜けない」と答えたので確定を解く(ここで初めて決め直せる)。
            if (side_commit_ && side_commit_target_ == c.blocker) {
              side_committed_ = false;
            }
            ++side_none_n_;
            if ((f.now - last_side_none_log_).seconds() > 2.0) {
              last_side_none_log_ = f.now;
              diagLog("どちらも足りない",
                      "どちらも足りない 累計%zu 左%.2fm 右%.2fm 必要%.2fm "
                      "抜き切り%.0fm 従来の判断に委ねる idx=%zu",
                      side_none_n_, rl, rr, need_here, pass_len_for_side, ei);
            }
          }
        }
        // ここが「右と出たのに左になる」最有力の門。
        audit_cs_ = cs;
        audit_need_here_ = need_here;
        audit_cfit_now_ = cfit_now;
        audit_cfit_ahead_ = cfit_ahead;
        if (cfit && !ideal_decided) {
          const double before_want = want;
          want = (cs_use > 0) ? +1.0 : -1.0;
          side_src = "抜き切り地点の空き";
          want_from_completion = true;
          // 計算が「この側で抜ける」と答えた。失敗するまで決め直さない。
          if (side_commit_ && !attempt_active_) {
            if (!side_committed_ || side_commit_target_ != c.blocker) {
              side_committed_ = true;
              side_commit_target_ = c.blocker;
              diagLog("側を確定",
                      "側を確定 target=%s 側=%s idx=%zu "
                      "(計算が抜けると答えた。失敗するまで決め直さない)",
                      c.blocker.c_str(), want > 0.0 ? "左" : "右", ei);
            }
          }
          // 抜き切り地点で必要間隔が足りている側を選べたか。
          completion_room_ok = pass_need_room_
            ? (((cs_use > 0) ? rl : rr) >= room_need)
            : cfit_ahead;
          if (before_want != want) {
            ++completion_side_count_;
            if ((f.now - last_completion_side_log_).seconds() > 1.0) {
              last_completion_side_log_ = f.now;
              diagLog("側の決定",
                "側の決定 抜き切り地点で変更 累計%zu 相手=%s %s -> %s "
                "抜き切り距離%.0fm 窓の空き[左%.2f 右%.2f](要%.2f) "
                "現在地で寄れる=%d idx=%zu",
                completion_side_count_, c.blocker.c_str(),
                before_want > 0 ? "左" : "右", want > 0 ? "左" : "右",
                pass_len_for_side, rl, rr, need_here, cfit_now ? 1 : 0, ei);
            }
          }
        }
      }
    }
    side_src_ = side_src;
    RCLCPP_INFO(get_logger(),
      "側を決めた target=%s 側=%s 決定元=%s idx=%zu ゾーン内=%d 区間=%d "
      "相手の区間平均横=%.2f 空き左=%.2f 空き右=%.2f 連続長左=%.1f 連続長右=%.1f",
      c.blocker.c_str(), (want > 0.0) ? "左" : "右", side_src, ei,
      inSidePickZone(ei) ? 1 : 0, sidePickZoneIndex(ei),
      (mlat_dbg > 1e8) ? 9.99 : mlat_dbg, room_l_mean, room_r_mean, map_left, map_right);
    // ここが**第二の関門**だった。
    const bool keep_right = want_from_completion || ot_lane_keep;
    if (want < 0.0) {
      side_sign_ = (fit_right || keep_right)
                     ? -1.0 : (fit_left ? +1.0 : -1.0);
    } else {
      side_sign_ = (fit_left || want_from_completion)
                     ? +1.0 : (fit_right ? -1.0 : +1.0);
    }
    // 両側とも成立するなら、並走できる区間が長いほうを選ぶ。
    // イン優先は「相手が避けざるを得ない」ための策だが、
    // 抜き切るまでの区間で明らかに狭ければ意味がない。
    if (!in_right_zone && !want_from_completion && lane_map_side_ &&
        mapKnownOk(map_known, map_zone_clipped) && fit_left && fit_right) {
      if (std::abs(map_left - map_right) > lane_map_margin_) {
        side_sign_ = (map_left > map_right) ? +1.0 : -1.0;
      }
    }
  }
  if (lane_map_side_ && inSidePickZone(ei) && !attempt_active_ && !ot_lane_keep &&
      std::abs(offset_) < pass_gap_ * 0.5)
  {
    int zknown2 = 0;
    const double mlat2 = zoneMeanLat(o, n, zknown2, sidePickZoneIndex(ei));
    // diff の符号を「左を選びたいときに正」にそろえる。
    const double diff = (mlat2 > 1e8) ? 0.0
                      : ((mlat2 < -side_pick_tie_) ? +1.0 : -1.0);
    const double want2 = (diff > 0.0) ? +1.0 : -1.0;
    if (diff != 0.0 && want2 != side_sign_) {
      if (side_room_want_ != want2) {
        side_room_want_ = want2;
        side_room_since_ = now.seconds();
      }
    } else {
      side_room_want_ = 0.0;
      side_room_since_ = -1.0;
    }
    if (side_room_want_ != 0.0 && side_room_since_ >= 0.0 &&
        (now.seconds() - side_room_since_) >= side_room_hold_)
    {
      const bool fit2 = (want2 > 0.0) ? fit_left : fit_right;
      if (fit2 && want2 != side_sign_) {
        side_sign_ = want2;
        side_decided_at_ = now.seconds();
        side_unfit_since_ = -1.0;
        side_room_want_ = 0.0;
        side_room_since_ = -1.0;
        if ((this->now() - last_wallpick_log_).seconds() > 2.0) {
          last_wallpick_log_ = this->now();
          RCLCPP_INFO(get_logger(),
            "側を録画で選び直し target=%s 側=%s 相手の区間平均横=%.2fm(点%d) "
            "空き 左%.2fm 右%.2fm idx=%zu",
            c.blocker.c_str(), (side_sign_ > 0.0) ? "左" : "右",
            mlat2, zknown2, room_l_mean, room_r_mean, ei);
        }
      }
    }
  }

  // 対象車が同じまま後から時空間計画が成立した場合も、その計画側へ揃える。
  // side_blocker_ の変化時だけ設定すると、古い瞬間判断が残り続ける。
  const bool execute_spot_side = spot_enable_ && spot_valid_ &&
    c.blocker == spot_target_ && spot_side_ != 0.0 &&
    (spot_dist_ <= spotGateDistance(f, o) ||
     (attempt_active_ && attempt_target_ == spot_target_));
  if (execute_spot_side) {
    const bool planned_fits = (spot_side_ > 0.0) ? fit_left : fit_right;
    // 抜き切り地点の空きから側が決まっているときは。
    const bool keep_completion_side =
      spot_keep_completion_side_ && want_from_completion_ && completion_room_ok_ &&
      spot_side_ != side_sign_;
    // レーン区間では、抜きどころが左を出しても従わない。
    const bool keep_ot_lane_side =
      ot_lane_side_over_spot_ && ot_lane_keep && spot_side_ > 0.0;
    if (keep_ot_lane_side) {
      ++ot_lane_side_kept_n_;
      if ((this->now() - last_ot_lane_keep_log_).seconds() > 2.0) {
        last_ot_lane_keep_log_ = this->now();
        diagLog("レーンの側を保持",
                "レーンの側を保持 累計%zu 抜きどころ=左 を退けて右を残す "
                "target=%s idx=%zu",
                ot_lane_side_kept_n_, c.blocker.c_str(), ei);
      }
    }
    if (planned_fits && !keep_completion_side && !keep_ot_lane_side) {
      side_sign_ = spot_side_;
    } else if (keep_completion_side) {
      ++spot_side_kept_n_;
      if ((this->now() - last_spot_keep_log_).seconds() > 2.0) {
        last_spot_keep_log_ = this->now();
        diagLog("側の保持",
                "側の保持 累計%zu 抜きどころ=%s を退け 抜き切りの側=%s を残す "
                "target=%s idx=%zu",
                spot_side_kept_n_, spot_side_ > 0.0 ? "左" : "右",
                side_sign_ > 0.0 ? "左" : "右", c.blocker.c_str(), ei);
      }
    }
  }

  // ここが**第三の関門**。
  side_fits_ = (side_sign_ > 0.0) ? fit_left : fit_right;
  if (!side_fits_ && want_from_completion_ && completion_room_ok_) {
    side_fits_ = true;
  }
  // レーンで右に決めた側も、現在地の狭さで「余地なし」に。
  if (!side_fits_ && ot_lane_keep && side_sign_ < 0.0) {
    side_fits_ = true;
  }

  if (!side_fits_) {
    if (side_unfit_since_ < 0.0) { side_unfit_since_ = now.seconds(); }
  } else {
    side_unfit_since_ = -1.0;
  }
  // 使った枠を時間で戻す。総量制のままでは、同じ相手が長く前にいる間に
  // 枠を使い切って「余地の無い側に張り付いたまま」になる。
  if (side_flip_regen_ > 0.0 && side_flip_cnt_ > 0) {
    const double dt = now.seconds() - side_flip_at_;
    const int credit = static_cast<int>(dt / side_flip_regen_);
    if (credit > 0) {
      side_flip_cnt_ = std::max(0, side_flip_cnt_ - credit);
      side_flip_at_ += credit * side_flip_regen_;
    }
  }
  const bool other_fits = (side_sign_ > 0.0) ? fit_right : fit_left;
  if (other_fits) {
    if (side_other_fit_since_ < 0.0) { side_other_fit_since_ = now.seconds(); }
  } else {
    side_other_fit_since_ = -1.0;
  }
  const bool other_fits_held =
    other_fits && side_other_fit_since_ >= 0.0 &&
    (now.seconds() - side_other_fit_since_) >= side_flip_hold_;
  const bool committed = attempt_active_;
  // レーンで右に決めている間は反転させない(上で side_fits_ を立てているので
  // 通常は到達しないが、別経路で偽になった場合の保険)。
  if (!side_fits_ && !(ot_lane_keep && side_sign_ < 0.0) &&
      side_flip_cnt_ < side_flip_max_ && other_fits_held && !committed &&
      side_unfit_since_ >= 0.0 &&
      (now.seconds() - side_unfit_since_) >= side_flip_hold_)
  {
    side_sign_ = -side_sign_;
    side_fits_ = true;            // 回った先は余地があると確認済み
    side_flip_cnt_++;
    side_flip_at_ = now.seconds();
    side_unfit_since_ = -1.0;
    side_other_fit_since_ = -1.0;
    side_decided_at_ = now.seconds();
    RCLCPP_INFO(get_logger(),
      "追越 側を変更(%d/%d) target=%s 側=%s 相手横=%.2f 余地=[%.2f,%.2f] offset=%.2f",
      side_flip_cnt_, side_flip_max_,
      c.blocker.c_str(), (side_sign_ > 0.0) ? "左" : "右",
      olat, room_lo_, room_hi_, offset_);
  }
  // いま side_sign_ が指している相手を覚えておく。
  // バンド側(buildBand)が独自に決めていた「どちら側を開けるか」を
  // これに合わせるために要る(下の band_side_follow_)。
  side_target_ = c.blocker;
}

void V2XOvertaker::decideAllow(const Frame & f, PlanCtx & c, const std::string & name,
                 const OtherState & o, size_t oi, double gap, double olat,
                 double ospeed_for_gate, OppEval & ev)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // 抜いてよいかは「ゾーン内」または「相手が極端に遅い」場合。
  // それに加えて「ブーストを使えば抜ける」と判断できるならゾーン外でも抜く。
  c.slow_leader = (ospeed_for_gate < slow_leader_speed_);
  const double my_speed = odom_->twist.twist.linear.x;

  // 幅さえあれば、ブーストで詰められるかを見る。
  // 速度差が小さくて自力では抜けないが、ブーストの上乗せがあれば
  // 追い越しに要する距離を pass_len_ 以内に収められる場合に使う。
  // ブーストの効果は「加速度 +0.5 m/s^2 を 10 秒」(parameter.md)。
  // 最高速は上がらないので、既に頭打ちの速度域で撃っても無意味。
  // 加速余地(目標速度との差)がある場面でのみ効く。
  // --- 追い越せるかを「自車の性能」から判定する
  //
  // これまでは closing(今の速度差)だけを見ていたため、
  // 自分が1位で 25 km/h 上限なのに、瞬間的に速度が出ている場面で
  // 「抜ける」と誤判定して横に出て、抜けないまま並走して接触していた。
  //
  // 正しくは「自分がこれから到達できる速度」と「必要な加速時間」で判断する。
  //   自車の速度上限 = 順位による handicap (1位:25km/h, 2位以下:36km/h)
  //                    と、その地点の速度マップの小さい方
  //   到達までの時間 = (目標速度 - 現在速度) / 実効加速度
  const double v_target_here = in.points[ei].longitudinal_velocity_mps;
  const double rank_cap = ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
  const double v_reach = std::min(v_target_here, rank_cap);   // 自分が出せる上限
  const double headroom = v_reach - my_speed;                 // まだ伸ばせる速度

  // 相手を抜くのに必要な相対速度。相手より速くなれなければ抜けない。
  const double closing_max = v_reach - ospeed_for_gate;

  // 加速に要する時間を引いた「実際に使える時間」で距離を稼ぐ
  const double accel_eff = std::max(vehicle_accel_, 0.05);
  const double t_accel = std::max(headroom, 0.0) / accel_eff;
  // 車体2台ぶんが抜けていた。
  const double need = allow_need_full_
    ? (gap + pass_len_ + (geom_front_ + geom_rear_) * 2.0)
    : (gap + pass_len_);                           // 抜き切るのに詰める距離

  // ブースト: 加速度 +0.5 m/s^2 を 10 秒。最高速は上げないので
  // 加速余地がある場面でのみ効く(到達を早めるだけ)。
  const double accel_boosted = accel_eff + boost_accel_;
  const double t_accel_boosted = std::max(headroom, 0.0) / accel_boosted;

  // 追い越しに要する時間の見積り。
  // 加速中は平均的に closing_max/2 で詰め、到達後は closing_max で詰める。
  auto pass_time = [&](double ta) {
    if (closing_max <= 0.2) {
      return 1e9;                                   // そもそも相手より速くなれない
    }
    const double d_accel = closing_max * 0.5 * ta;  // 加速中に詰まる距離
    if (d_accel >= need) {
      return ta * (need / std::max(d_accel, 1e-6));
    }
    return ta + (need - d_accel) / closing_max;
  };

  // --- 抜き切るのに必要な距離が、使える距離に収まるか
  // 時間だけでなく距離でも見る。ゾーンが途中で終わるなら出ない。
  auto pass_dist_a = [&](double ta, double a_use) {
    const double t = pass_time(ta);
    if (t >= 1e8) {
      return 1e9;
    }
    // その間に自車が進む距離
    return my_speed * t + 0.5 * a_use * std::min(t, ta) * std::min(t, ta);
  };
  auto pass_dist = [&](double ta) { return pass_dist_a(ta, accel_eff); };

  const double closing_kmh = closing_max * 3.6;
  double usable;
  if (c.slow_leader || closing_kmh >= big_gap_closing_) {
    usable = 1e9;                       // 相手が明らかに遅い。距離で縛らない
  } else {
    usable = c.zone_remain + zone_exit_margin_;
  }

  const bool inside = (curve_sign_ != 0.0) && (side_sign_ * curve_sign_ > 0.0);
  const bool inside_ok = inside && c.in_zone;
  const bool aggressive = (rank_ >= 3);
  const double t_limit = pass_time_limit_ * (inside ? inside_time_gain_ : 1.0)
                         * (aggressive ? aggressive_time_gain_ : 1.0);
  // 幅の割引は掛け合わせない。
  // 両方が効くと 3.2 * 0.85 * 0.85 = 2.31m となり、カート2台ぶんの
  // 物理的な下限(1.45 x 2 = 約2.9m)を割って必ず接触する。
  // 効く条件のうち最も緩い1つだけを使う。
  double w_gain = 1.0;
  if (inside_ok) { w_gain = std::min(w_gain, inside_width_gain_); }
  if (aggressive) { w_gain = std::min(w_gain, aggressive_width_gain_); }
  const double w_need = min_pass_width_ * w_gain;
  // 幅は「抜き切るまでの区間」で見る。pass_dist(t_accel) は下の
  // 「ゾーン残距離不足」判定が既に使っている、抜き切るのに要る距離。
  // 同じ量を幅の判定にも使うことで、判定の基準を一つに揃える。
  double w_avail = c.avail_width;
  if (pass_width_dist_gain_ > 0.0) {
    const double d = std::min(pass_dist(t_accel) * pass_width_dist_gain_,
                              pass_width_dist_max_);
    w_avail = std::min(w_avail, minWidthAhead(f, d));
  }
  const bool width_ok = w_avail >= w_need;
  bool clearly_slower = false;
  double dbg_slow_o = -1.0, dbg_slow_m = -1.0;
  {
    const double om = slow_rival_recent_ ? o.topSectionSpeedRecent()
                    : slow_rival_by_top_ ? o.topSectionSpeed() : o.meanSpeed();
    const double mm = slow_rival_recent_ ? mySectionTopRecent()
                    : slow_rival_by_top_
                      ? mySectionTop()
                      : ((my_speed_cnt_ > 20) ? my_speed_sum_ / my_speed_cnt_ : -1.0);
    const double ratio = (slow_rival_by_top_ || slow_rival_recent_)
                           ? slow_rival_top_ratio_ : slow_rival_ratio_;
    dbg_slow_o = om; dbg_slow_m = mm;
    if (om > 0.0 && mm > 0.0 && om < mm * ratio) {
      clearly_slower = true;
    }
    // その区間での実績も見る。全体が遅くても、その場所だけ速いことがある。
    // 最大で比べるときは行わない。最大に対して「この区間は速い」を課すと
    // 「どこかで自由に走れた実力を見る」という趣旨と矛盾する。
    if (!slow_rival_by_top_ && !slow_rival_recent_ &&
        clearly_slower && !line_x_.empty()) {
      const int sec = static_cast<int>(oi * OtherState::kSections / n);
      const double os = o.sectionSpeed(sec);
      if (os > 0.0 && mm > 0.0 && os > mm * slow_rival_ratio_) {
        clearly_slower = false;
      }
    }
  }
  const bool capped_leader = (rank_ >= 2) && !cur_leader_.empty() &&
                             (name == cur_leader_);
  const bool closing_ok = (closing_max * 3.6 >= min_closing_kmh_)
                          && (pass_dist(t_accel) <= pass_dist_max_);
  const bool capped_self = capped_self_enable_ && (rank_ == 1) &&
                           (closing_max * 3.6 >= capped_self_closing_) &&
                           (pass_dist(t_accel) <= capped_self_dist_);
  const bool self_ok = width_ok
                       && pass_time(t_accel) <= t_limit
                       && pass_dist(t_accel) <= usable
                       && (c.slow_leader || clearly_slower || capped_leader ||
                           capped_self || closing_ok);

  // ブーストで抜けるようになるか。
  // 「自力では抜けない(self_ok が偽)」ときだけ見ると、pass_time_limit を
  // 緩めた結果ほとんどが self_ok になり、ブーストが一切使われなくなった。
  // 自力で抜ける場合でも、ゾーンの残りが足りずに距離条件で落ちるなら
  // ブーストで間に合わせる価値がある。
  bool boost_would_help = false;
  if (width_ok && !c.slow_leader && boost_remaining_ > 0 &&
      headroom > boost_min_headroom_) {
    const bool ok_boost = pass_time(t_accel_boosted) <= pass_time_limit_
                          && pass_dist(t_accel_boosted) <= usable;
    const double t_self = pass_time(t_accel);
    const double t_bst = pass_time(t_accel_boosted);
    boost_gain_time_ = ok_boost ? (t_self - t_bst) : 0.0;
    // 「自力では抜けない場合だけ」に限ると、ぎりぎり抜ける計算に
    // なった場面でブーストを温存し、直線の終わり(コーナー入口)で
    // 並んだまま突っ込んで失敗していた。
    // 自力で抜ける場合でも、ブーストで明確に短時間で抜けるなら使う。
    // 並走時間が短いほど接触の危険も小さい。
    boost_would_help = ok_boost &&
                       (!self_ok || boost_gain_time_ > boost_gain_min_);
  }

  // 抜けると判断できたときだけ横に出る。
  // 相手が極端に遅い(止まっている)場合は幅さえあれば抜きにいく。
  const bool kinematic_feasible = side_fits_ &&
                       ((c.slow_leader && width_ok) || self_ok || boost_would_help);
  const bool lap_traffic = o.slot == npc_slot_ && o.passed_cnt > 0;
  const double spot_gate = spotGateDistance(f, o);
  const bool spot_ready_planned = v2x_overtaker::plannedSpotReady(
    spot_enable_, spot_valid_, name == spot_target_, spot_dist_, spot_gate);
  const double here_room = (side_sign_ > 0.0) ? dbg_map_l_ : dbg_map_r_;
  double here_delay = -1.0;
  const bool here_is_blocker = !c.blocker.empty() && c.blocker == name;
  if (spot_here_enable_ && here_is_blocker && !attempt_active_ && gap > 0.0 &&
      gap <= spot_here_range_ && dbg_map_n_ >= lane_map_min_pts_ &&
      here_room > pass_len_) {
    here_delay = latestPassAccelDelay(
      f, name, o, gap, std::min(here_room, 200.0), false, 0.0);
  }
  const bool spot_here = here_delay >= 0.0;
  const bool spot_ready = spot_ready_planned || spot_here;
  // 相手の時刻同期済み予測バンドに対し、現在位置から計画ラインへ移る軌道と
  // 抜き切る区間を検査する。幅・速度だけ成立しても、相手が同じ側へ移る予測なら
  // 仕掛けない。予測は前周期の20Hz計算値なので制御遅延は1周期だけ。
  const bool using_spot = spot_enable_ && spot_valid_ && name == spot_target_;
  const double pass_need = std::max(pass_dist(t_accel), pass_len_);
  // ログ用。check_spot_path の本体は planned_look の後で定義される。
  const bool check_spot_path_pre = using_spot &&
    (spot_ready || (attempt_active_ && attempt_target_ == name));
  // ここは「計画した抜きどころまでの道」を検査していた。
  const double planned_look_full = std::min(
    spot_range_, std::max(spot_dist_, 0.0) + std::min(spot_len_, pass_need));
  const bool look_shortened =
    spot_look_local_ && pass_need > 0.0 && pass_need < std::max(spot_dist_, 0.0);
  const double planned_look = look_shortened ? pass_need : planned_look_full;
  if (look_shortened && check_spot_path_pre &&
      (now - last_look_short_log_).seconds() > 1.0)
  {
    last_look_short_log_ = now;
    ++look_short_count_;
    diagLog("追越判定",
      "追越判定 検査距離を短縮 累計%zu 相手=%s 完遂%.0fm < 抜きどころ%.0fm "
      "(従来は%.0fmまで検査していた)",
      look_short_count_, name.c_str(), pass_need, spot_dist_, planned_look_full);
  }
  // 予測を「抜きどころ」を持つ相手だけでなく全相手に適用する。
  const bool predict_all_here =
    predict_all_targets_ && !check_spot_path_pre &&
    dbg_map_n_ >= lane_map_min_pts_ && gap > 0.0 && here_room > pass_len_;
  const bool check_spot_path = check_spot_path_pre || predict_all_here;
  const double predict_look = predict_all_here ? pass_need : planned_look;
  const bool predicted_path_ok = !check_spot_path ||
                                 spotPathSafe(f, o, predict_look);
  double spot_exit_distance = 0.0;
  if (predict_all_here) {
    spot_exit_distance = std::min(here_room, 200.0);
  } else if (check_spot_path) {
    double to_end = f.s[spot_end_] - f.s[ei];
    if (to_end < 0.0) { to_end += f.total; }
    spot_exit_distance = spot_in_now_
      ? to_end : std::max(spot_dist_, 0.0) + spot_len_;
  }
  const double predict_spot_dist = predict_all_here
    ? 0.0 : std::max(spot_dist_, 0.0);
  const double base_accel_delay = check_spot_path
    ? latestPassAccelDelay(
        f, name, o, gap, spot_exit_distance, false,
        predict_spot_dist) : 0.0;
  const bool timed_boost_available = is_boosting_ ||
    (boost_remaining_ > 0 && boostLapOk() && !lap_traffic);
  const double boost_accel_delay =
    (check_spot_path && base_accel_delay < 0.0 && timed_boost_available)
      ? latestPassAccelDelay(
          f, name, o, gap, spot_exit_distance, true,
          predict_spot_dist) : -1.0;
  const bool timed_boost = check_spot_path && base_accel_delay < 0.0 &&
                           boost_accel_delay >= 0.0 && !is_boosting_;
  const double accel_delay = (base_accel_delay >= 0.0)
    ? base_accel_delay : boost_accel_delay;
  const bool predictive_timing_ok = !check_spot_path || accel_delay >= 0.0;
  const bool timed_feasible = check_spot_path && accel_delay >= 0.0 &&
                              side_fits_ && width_ok;
  const bool feasible =
    (kinematic_feasible || timed_feasible) &&
    predicted_path_ok && predictive_timing_ok;
  bool in_no_pass = false;
  for (const auto & z : no_pass_zones_) {
    const bool inside = (z.first <= z.second)
                          ? (ei >= z.first && ei <= z.second)
                          : (ei >= z.first || ei <= z.second);
    if (inside) { in_no_pass = true; break; }
  }
  const bool require_spot_plan = slots_assigned_ && start_slot_ == 1 &&
                                 lap_ >= record_laps_ &&
                                 (o.slot != npc_slot_ || lap_traffic);
  const bool ot_lane_here = otLaneApproach(ei, my_speed_for_gap_, rank_cap);
  const bool zone_ok = spot_ready || ot_lane_here ||
                       (zone_fallback_enable_ && c.in_zone && !require_spot_plan);

  if (attempt_active_ && attempt_target_ == name && using_spot) {
    const bool predicted_plan_ok = predicted_path_ok && predictive_timing_ok;
    if (!predicted_plan_ok) {
      if (spot_unsafe_since_ < 0.0) { spot_unsafe_since_ = now.seconds(); }
      if (now.seconds() - spot_unsafe_since_ >= spot_abort_sec_ &&
          (now - last_spot_unsafe_log_).seconds() > 2.0)
      {
        last_spot_unsafe_log_ = now;
        RCLCPP_INFO(get_logger(),
          "追越 予測不成立(中断はしない) target=%s 所要=%.1fs 理由=%s",
          name.c_str(), now.seconds() - attempt_start_,
          predicted_path_ok ? "区間内の追越成立不能" : "相手予測経路が計画ラインへ侵入");
      }
    } else {
      spot_unsafe_since_ = -1.0;
    }
  } else if (!attempt_active_) {
    spot_unsafe_since_ = -1.0;
  }
  const bool latch_width_ok = c.avail_width >= min_pass_width_ * latch_width_gain_;
  const bool latched = attempt_active_ && predicted_path_ok &&
                       predictive_timing_ok && latch_width_ok &&
                       (now.seconds() - attempt_start_) < attempt_timeout_;
  // 禁止区間では新しく仕掛けない。ただし既に並走している(latched)場合は
  // そのまま続けさせる。狭い所で急にラインへ戻るほうが危ないため。
  // 直前に「進展なし」で降りた相手には、しばらく仕掛け直さない。
  // これが無いと降りた次の周期で条件が揃い直し、4秒ごとに横へ出ては
  // 戻るだけになる(打切を早めた意味が無くなる)。
  // ただし相手が明らかに遅くなったなら話が別なので、その場合は解除する。
  const bool stall_block = !c.slow_leader && !clearly_slower &&
                           name == attempt_stall_name_ &&
                           now.seconds() < attempt_stall_until_;
  const bool stop_avoid_block = name == stop_avoid_target_ &&
                                now.seconds() < stop_avoid_retry_until_;
  const bool predictive_start_safe = check_spot_path && predicted_path_ok &&
                                     predictive_timing_ok;
  double start_gap_need = rear_end_margin_ +
                          (predictive_start_safe ? 0.3 : 1.0);
  if (start_gap_by_rearend_) {
    const double cap_v = rankSpeedCap();
    double v_aim = std::min(ospeed_for_gate + runup_dv_, cap_v);
    if (ot_lane_here) {
      v_aim = std::max(v_aim,
                       std::min((ot_lane_min_kmh_ + ot_lane_runup_margin_kmh_) / 3.6, cap_v));
    }
    const double brake_a_re = std::max(std::abs(a_min_) * rear_end_brake_k_, 0.3);
    const double dv = std::max(v_aim - ospeed_for_gate, 0.0);
    const double need_rear =
      rear_end_margin_ + v_aim * rear_end_time_ + dv * dv / (2.0 * brake_a_re);
    start_gap_need = std::max(start_gap_need, std::min(need_rear, start_gap_need_max_));
  }
  // 「追い越しを開始するのに最低 4.5m の車間が必要」という。
  const bool start_gap_ok = v2x_overtaker::overtakeStartGapOk(
    attempt_active_, gap, start_gap_need,
    spot_ready_planned, ot_lane_here, start_gap_floor_);
  const bool lap_ok = passAllowedThisLap(name, o, clearly_slower || c.slow_leader);

  bool spot_block_requested = false;
  if (spot_enable_ && spot_gate_ && !attempt_active_ && lap_ >= record_laps_) {
    if (spot_valid_ && name == spot_target_) {
      spot_block_requested = (spot_dist_ > spot_gate);
    }
    if (spot_block_requested) {
      if (spot_wait_since_ < 0.0) { spot_wait_since_ = now.seconds(); }
      if (!require_spot_plan && spot_gate_max_wait_ > 0.0 &&
          (now.seconds() - spot_wait_since_) > spot_gate_max_wait_) {
        spot_block_requested = false;
      }
    } else {
      spot_wait_since_ = -1.0;
    }
  }
  bool straight_short = false;
  {
    // 【2026-09-18】区間の残りと「抜き切るのに要る距離」は、遅い相手でも試行中でも毎周期出す。
    // 以前は「相手が明らかに遅い」「試行中」のときに計算自体を飛ばしていたため、
    // レーンの出口で抜き切れないまま合流していた。
    const double d_end0 = distToPassFinishZoneEnd(f);
    finish_left_m_ = d_end0;
    finish_need_m_ = -1.0;
    finish_short_now_ = false;
    if (d_end0 >= 0.0) {
      double need0 = pass_dist(t_accel);
      if (pass_boost_gate_ && boost_remaining_ > 0 && boostLapOk()) {
        need0 = std::min(need0, pass_dist_a(t_accel_boosted, accel_boosted));
      }
      finish_need_m_ = need0;
      finish_short_now_ = need0 > d_end0 + straight_finish_margin_;
    }
    // ここは「直線の終わり」が要る。
    const double d_end = distToPassFinishZoneEnd(f);
    if (d_end >= 0.0 && !c.slow_leader && !clearly_slower && !attempt_active_) {
      straight_need_boost_ = false;
      double d_need = pass_dist(t_accel);
      const bool boost_avail =
        pass_boost_gate_ && boost_remaining_ > 0 && boostLapOk();
      if (boost_avail) {
        const double d_bst = pass_dist_a(t_accel_boosted, accel_boosted);
        if (d_bst < d_need) {
          d_need = d_bst;
          // 「ブーストが要る」= 素の加速では届かないが、ブーストなら届く。
          if (pass_dist(t_accel) > d_end + straight_finish_margin_ &&
              d_bst <= d_end + straight_finish_margin_) {
            pass_need_boost_ = true;
            straight_need_boost_ = true;
          }
        }
      }
      straight_short = d_need > d_end + straight_finish_margin_;
    }
  }
  // latch は「横に出続けてよい」だけを許す。速度上限の解除(commit)まで。
  double zf_fail_at = -1.0, zf_min_w = 1e9;
  const bool local_width_through = passPathCapable(
    f, pass_need, spot_w_ref_ ? spot_w_ref_min_ : w_need,
    zf_fail_at, zf_min_w);
  const bool zf_width_through = !zone_free_ || local_width_through;
  bool zf_lat_return = false;
  {
    // (a) いまの地点で、レースライン(横位置 0)が帯の中にあるか。
    //     あるなら、破綻しても横へ戻る道がある。
    const bool have_band =
      band_enable_ && band_lo_ex_.size() == f.n && band_hi_ex_.size() == f.n;
    const bool have_corr =
      corridor_.lo.size() == f.n && corridor_.hi.size() == f.n;
    if (have_band || have_corr) {
      const std::size_t b = ei;
      const double lo = have_band ? band_lo_ex_[b] : corridor_.lo[b] + safetyAt(b);
      const double hi = have_band ? band_hi_ex_[b] : corridor_.hi[b] - safetyAt(b);
      zf_lat_return = (lo <= 0.0 && hi >= 0.0);
    }
  }
  // (b) 速度差を抜重で消すのに要る距離が車間に収まるか。
  const double zf_dv = std::max(0.0, v_reach - ospeed_for_gate);
  const double zf_stop_dist = (zone_free_brake_decel_ > 0.0)
    ? zf_dv * zf_dv / (2.0 * zone_free_brake_decel_) : 1e9;
  const bool zf_back_off = zf_stop_dist <= std::max(0.0, gap - zone_free_keep_);
  const bool local_abortable = zf_lat_return || zf_back_off;
  const bool zf_abortable = !zone_free_ || local_abortable;
  const bool zf_ok = zf_width_through && zf_abortable && side_fits_ && width_ok;
  // A recorded spot ranks opportunities; it cannot veto a certified local path.
  const bool local_capable = local_width_through && local_abortable && feasible &&
                             side_fits_ && width_ok;
  const bool spot_block = spot_block_requested && !local_capable;
  const bool place_ok = zone_free_ ? zf_ok : (zone_ok || local_capable);
  bool short_ok = zone_free_ || !straight_short || local_capable;
  // 【2026-09-18】「区間内に抜き切れない」を local_capable(通せる幅がある・引き返せる)で
  // 上書きしていたため、抜き切れないと分かっている状況で試行を始めていた。
  if (pass_finish_no_start_ && finish_short_now_ && !attempt_active_) { short_ok = false; }
  bool caution_block = false;
  if (!caution_zones_.empty() && !attempt_active_) {
    for (const auto & z : caution_zones_) {
      const bool inside = (z.first <= z.second)
                            ? (ei >= z.first && ei <= z.second)
                            : (ei >= z.first || ei <= z.second);
      if (!inside) { continue; }
      const bool clearly_slow =
        c.slow_leader || clearly_slower || capped_leader ||
        ospeed_for_gate < stopped_speed_;   // 停止・極低速の相手は待たない
      if (!clearly_slow) { caution_block = true; }
      break;
    }
  }

  const bool allow_base = (place_ok && feasible && lap_ok && !spot_block &&
                           short_ok && !in_no_pass && !caution_block && !stall_block &&
                           !stop_avoid_block && start_gap_ok);
  const bool latch_hard_block = latch_never_no_pass_ && (in_no_pass || !side_fits_);
  const bool allow = allow_base ||
                     (latch_allow_enable_ && latched && !latch_hard_block);
  dbg_allow_ = allow; dbg_width_ = w_avail; dbg_zone_ = c.in_zone;
  if (attempt_active_ && attempt_target_ == name) {
    const double look = std::max(std::abs(f.ev), 1.0) * attempt_wall_look_time_;
    const double edge = minEdgeClearAhead(f, my_lat_for_target_, look);
    if (edge >= attempt_wall_abort_clear_) { attempt_infeasible_since_ = -1.0; }
    else if (attempt_infeasible_since_ < 0.0) {
      attempt_infeasible_since_ = now.seconds();
    }
  }
  dbg_latched_ = latched; dbg_feasible_ = feasible; dbg_zone_ok_ = zone_ok;

  const double prepare_gate_dist = prepare_early_
    ? v2x_overtaker::prepareGateDistance(
        my_speed_for_gap_, min_pass_sep_, offset_rate_,
        lat_lag_sec_, lat_lag_base_, lat_lag_max_,
        prepare_free_gap_, prepare_early_margin_)
    : prepare_free_gap_;
  if (prepare_early_ && prepare_gate_dist > prepare_free_gap_ + 0.5 &&
      gap > prepare_free_gap_ && gap <= prepare_gate_dist &&
      (now - last_prepare_gate_log_).seconds() > 2.0)
  {
    last_prepare_gate_log_ = now;
    ++prepare_early_count_;
    diagLog("接近準備",
      "接近準備 早めに開始 累計%zu 相手=%s 車間%.1fm 門%.1fm(従来%.1fm) 自車%.1fkm/h",
      prepare_early_count_, name.c_str(), gap, prepare_gate_dist,
      prepare_free_gap_, my_speed_for_gap_ * 3.6);
  }
  if (prepare_free_enable_ && !attempt_active_ && lap_ok &&
      !in_no_pass && !stall_block && !stop_avoid_block && side_fits_ &&
      // 物理的な制約は残す。ここで外すのは「開始車間」と「ゾーン」だけで、
      // 幅と予測経路の閉塞は準備であっても越えてはいけない。
      width_ok && predicted_path_ok &&
      gap > 0.0 && gap <= prepare_gate_dist &&
      (c.slow_leader || clearly_slower || capped_leader || capped_self ||
       closing_ok || v_reach > ospeed_for_gate + prepare_free_vgain_))
  {
    // 複数台が条件を満たすなら**いちばん近い**相手を選ぶ。
    // others_ の走査順(名前順)で先に来た車を採ると、遠い車を対象にして
    // 目の前の車を無視した横位置を作ってしまう。
    if (prepare_wish_.empty() || gap < prepare_wish_gap_) {
      prepare_wish_ = name;
      prepare_wish_gap_ = gap;
    }
  }
  // 準備を続けてよいかの「安全に直結する条件」だけを、いまの対象について残す。
  // 猶予つきで保持するのは希望(prepare_wish_)の側だけで、
  // ここが偽になったら猶予を待たずに降りる。
  if (!ov_target_.empty() && name == ov_target_) {
    prepare_target_seen_ = true;
    prepare_target_hard_ok_ = (!in_no_pass && side_fits_ && width_ok);
  }

  const char * why =
      (pass_finish_no_start_ && finish_short_now_) ? "区間内に抜き切れない"
    : straight_short                 ? "直線内に抜き切れない"
    : !lap_ok                        ? "周回で禁止"
    : spot_block                     ? "抜きどころ待ち"
    : in_no_pass                     ? "禁止区間"
    : caution_block                  ? "要注意区間(壁が多い)"
    : stop_avoid_block               ? "停止車回避後の待機"
    : !start_gap_ok                  ? "開始車間不足"
    : stall_block                    ? "打切直後"
    : !predicted_path_ok             ? "予測経路が閉塞"
    : !predictive_timing_ok          ? "今から加速しても区間内に抜けない"
    : !side_fits_                    ? "側の余地なし"
    : !width_ok                      ? "幅不足"
    : (pass_time(t_accel) > t_limit) ? "時間超過"
    : (pass_dist(t_accel) > usable)  ? "ゾーン残距離不足"
    : !(c.slow_leader || clearly_slower || capped_leader ||
        capped_self || closing_ok)   ? "速度差不足"
    : (zone_free_ && !zf_width_through) ? "能力不足(通しの幅)"
    : (zone_free_ && !zf_abortable)     ? "能力不足(引き返せない)"
    : !place_ok                      ? (zone_free_ ? "能力不足" : "ゾーン外")
    :                                  "その他";

  if (latched && !allow_base && (now - last_latch_log_).seconds() > 2.0) {
    last_latch_log_ = now;
    diagLog("追越継続", "追越継続 latchで継続 target=%s allow_base落ち=%s "
            "幅=%.2f(要%.2f) 先読幅=%.2f 経過=%.1fs idx=%zu 自車=%.1fkm/h",
            name.c_str(), why, c.avail_width, w_need, w_avail,
            now.seconds() - attempt_start_, f.ei, std::abs(f.ev) * 3.6);
  }

  if (!allow) {
    ++reject_total_n_;
    ++reject_why_n_[why];
    if (inSidePickZone(ei)) {
      ++reject_why_straight_n_[std::string(why) +
                               ((side_sign_ > 0.0) ? " 左" : " 右")];
    }
  }
  if (reject_total_n_ > 0 && (now - last_reject_sum_log_).seconds() > 10.0) {
    last_reject_sum_log_ = now;
    auto dump = [](const std::map<std::string, size_t> & m, size_t tot) {
      std::vector<std::pair<size_t, std::string>> v;
      v.reserve(m.size());
      for (const auto & kv : m) { v.emplace_back(kv.second, kv.first); }
      std::sort(v.begin(), v.end(), [](const auto & a2, const auto & b2) {
        return a2.first > b2.first;
      });
      std::string out;
      char buf[160];
      for (size_t i = 0; i < v.size() && i < 8; ++i) {
        std::snprintf(buf, sizeof(buf), "%s%s=%zu(%.0f%%)", out.empty() ? "" : " ",
                      v[i].second.c_str(), v[i].first,
                      tot > 0 ? 100.0 * static_cast<double>(v[i].first) /
                                  static_cast<double>(tot)
                              : 0.0);
        out += buf;
      }
      return out;
    };
    size_t stot = 0;
    for (const auto & kv : reject_why_straight_n_) { stot += kv.second; }
    diagLog("却下の内訳",
            "却下の内訳 全区間 累計%zu周期 [%s] / 直線 累計%zu周期 [%s]",
            reject_total_n_, dump(reject_why_n_, reject_total_n_).c_str(),
            stot, dump(reject_why_straight_n_, stot).c_str());
  }
  if (!allow && (now - last_reject_log_).seconds() > 2.0) {
    last_reject_log_ = now;

    diagLog("追越却下", "追越却下 決め手=%s self_ok=%d capped自=%d capped先=%d 遅相手=%d ブ助=%d "
      "gap=%.1f zone=%d %s 幅=%.1f(要%.1f) 残距離=%.0f "
      "v_reach=%.1f 相手=%.1f closing=%.1f 所要=%.1fs 距離=%.0f rank=%d "
      "側OK=%d 側=%s 相手横=%.2f 余地=[%.2f,%.2f] idx=%zu 同速=%d 禁止区=%d レーン=%d "
      "枠=%d/%d 不成立=%.1fs 学習連続=[%.1f,%.1f]m/%d点 空き=[%.2f,%.2f]m "
      "抜きどころ=%s/%s/%.0fm 展開距離=%.0fm 加速待=%.1fs 周回=%d 相手P%d"
      " 帯幅min=%.2f 閉塞位置=%.0fm 閉塞幅=%.2f"
      " 助走要車間=%.1fm 加速開始=%.1fm 側の決定元=%s"
      " 遅判定[相手%.1f 自車%.1f km/h 方式=%s]"
      " 能力[通し幅=%d 最小幅%.2f 破綻%.0fm / 引返可=%d(横%d 抜重%d) 要制動%.1fm 車間%.1fm / 完遂%.0fm]"
      " 閉塞の内訳[idx=%d 帯=[%.2f,%.2f] 車なしの帯幅=%.2fm]",
      why, self_ok ? 1 : 0, capped_self ? 1 : 0, capped_leader ? 1 : 0,
      clearly_slower ? 1 : 0, (boost_would_help || timed_boost) ? 1 : 0,
      gap, c.in_zone ? 1 : 0, inside ? "イン" : "アウト", c.avail_width, w_need, usable,
      v_reach * 3.6, ospeed_for_gate * 3.6, closing_max * 3.6,
      pass_time(t_accel), pass_dist(t_accel), rank_,
      side_fits_ ? 1 : 0, (side_sign_ > 0.0) ? "左" : "右",
      olat, room_lo_, room_hi_, ei,
      (!c.slow_leader && !closing_ok) ? 1 : 0, in_no_pass ? 1 : 0,
      ot_lane_here ? 1 : 0,
      side_flip_cnt_, side_flip_max_,
      (side_unfit_since_ >= 0.0) ? (now.seconds() - side_unfit_since_) : -1.0,
      dbg_map_l_, dbg_map_r_, dbg_map_n_, dbg_room_l_, dbg_room_r_,
      spot_valid_ ? spot_target_.c_str() : "なし",
      (spot_side_ > 0.0) ? "左" : "右", spot_dist_, spot_gate, accel_delay,
      lap_ + 1, o.slot,
      spot_path_min_w_seen_, spot_path_fail_at_, spot_path_fail_w_,
      runup_need_gap_, runup_accel_at_, side_src_,
      dbg_slow_o * 3.6, dbg_slow_m * 3.6,
      slow_rival_recent_ ? "直近1周の区間最大"
        : slow_rival_by_top_ ? "区間最大" : "全体平均",
      zf_width_through ? 1 : 0, zf_min_w, zf_fail_at,
      zf_abortable ? 1 : 0, zf_lat_return ? 1 : 0, zf_back_off ? 1 : 0,
      zf_stop_dist, gap, pass_need,
      spot_path_fail_idx_, spot_path_fail_lo_, spot_path_fail_hi_,
      spot_path_fail_free_w_);
  }
  double tgt_lat = 0.0;
  // 試行中は allow の瞬間値に関わらず追越の意図を出し続ける。
  {
    const bool gate_open = allow || (attempt_active_ && attempt_target_ == name) ||
                           ovPreparingTarget(name) || ovPassingTarget(name);
    const double closing_now = my_speed_for_gap_ - ev.ospeed_for_gate;
    if (!gate_open && closing_now > 1.5 && gap > 0.0 && gap < 30.0 &&
        (f.now - last_latgate_log_).seconds() > 0.5)
    {
      last_latgate_log_ = f.now;
      ++latgate_block_count_;
      diagLog("横へ出ない",
        "横へ出ない 累計%zu 相手=%s 車間%.1fm 接近%.1fkm/h 自車%.1fkm/h "
        "許可=%d 試行中=%d(対象=%s) 状態=%s 状態の対象=%s 側=%s idx=%zu",
        latgate_block_count_, name.c_str(), gap, closing_now * 3.6,
        my_speed_for_gap_ * 3.6, allow ? 1 : 0, attempt_active_ ? 1 : 0,
        attempt_target_.empty() ? "-" : attempt_target_.c_str(),
        ovStateName(), ov_target_.empty() ? "-" : ov_target_.c_str(),
        side_sign_ > 0.0 ? "左" : "右", f.ei);
    }
  }
  // --- 追い越す側へ、着く前から寄せておく(事前寄せ) ---。
  const bool pre_gate_allow = pre_position_pass_ok_ ? true : !allow;
  if (pre_reject_log_ && inSidePickZone(f.ei) && !ovPassingTarget(name)) {
    const char * why =
        !pre_position_enable_                       ? "無効"
      : !pre_gate_allow                             ? "抜けると判断"
      : attempt_active_                             ? "試行中"
      : ovPreparingTarget(name)                     ? "準備中の相手"
      : !(pre_position_always_ || want_from_completion_ || ot_lane_side_)
                                                    ? "側が確定していない"
      : (side_sign_ == 0.0)                         ? "側=0"
      : !(gap > 0.0)                                ? "車間が負"
      : (gap > pre_position_range_)                 ? "車間が遠い"
      :                                               "発火";
    ++pre_reject_n_[why];
    if ((f.now - last_pre_reject_log_).seconds() > 10.0) {
      last_pre_reject_log_ = f.now;
      std::string out;
      char buf[128];
      size_t tot = 0;
      for (const auto & kv : pre_reject_n_) { tot += kv.second; }
      for (const auto & kv : pre_reject_n_) {
        std::snprintf(buf, sizeof(buf), "%s%s=%zu(%.0f%%)", out.empty() ? "" : " ",
                      kv.first.c_str(), kv.second,
                      tot ? 100.0 * static_cast<double>(kv.second) /
                              static_cast<double>(tot) : 0.0);
        out += buf;
      }
      diagLog("事前寄せの内訳",
              "事前寄せの内訳 直線 累計%zu周期 [%s] 車間%.1fm 側=%.0f idx=%zu",
              tot, out.c_str(), gap, side_sign_, f.ei);
    }
  }
  const bool pre_skip_prepare = pre_position_in_prepare_ ? false
                                                         : ovPreparingTarget(name);
  if (pre_position_enable_ && pre_gate_allow && !attempt_active_ &&
      !pre_skip_prepare && !ovPassingTarget(name) &&
      (pre_position_always_ || want_from_completion_ || ot_lane_side_) &&
      side_sign_ != 0.0 &&
      gap > 0.0 && gap <= pre_position_range_)
  {
    const double sep_min = std::max(band_car_w_, min_pass_sep_);
    const double need = std::max(sep_min, pass_sep_floor_ > 0.0
                                            ? pass_sep_floor_ : sep_min);
    // 相手から必要間隔だけ離れた側。ここまで寄っておけば、
    // 追い越しに入った瞬間に追突防止が解除された状態で始められる。
    double pre = olat + side_sign_ * need;
    pre = std::clamp(pre, room_lo_, room_hi_);
    // 寄せ量を段階的にする。遠いうちから目一杯寄ると、。
    const double frac = std::clamp(
      (pre_position_range_ - gap) /
        std::max(pre_position_range_ * pre_position_gain_, 1.0), 0.0, 1.0);
    double pre_lat = pre * frac;
    // 追突防止が外れる量を下限にする。
    if (pre_position_sep_) {
      const double rel = v2x_overtaker::physicalPassSeparation(
        pass_beside_sep_, rear_end_free_min_);
      const double want_abs = olat + side_sign_ * (rel + pre_position_sep_extra_);
      const double want_cl = std::clamp(want_abs, room_lo_, room_hi_);
      // 相手より外側へ、帯の中で行けるところまで。frac は掛けたままにして
      // 遠いうちから徐々に寄せる(急な横移動は近接車回避を誘発する)。
      const double target = want_cl * frac;
      pre_lat = (side_sign_ > 0.0) ? std::max(pre_lat, target)
                                   : std::min(pre_lat, target);
    }
    c.requestLat(pre_lat, PlanCtx::LatPrio::kPrePosition, "事前寄せ");
    if ((f.now - last_pre_position_log_).seconds() > 2.0) {
      last_pre_position_log_ = f.now;
      ++pre_position_count_;
      diagLog("事前寄せ",
              "事前寄せ 累計%zu 相手=%s 側=%s 車間%.1fm 相手横%.2f "
              "狙い%.2f(満%.2f) 帯[%.2f,%.2f] idx=%zu",
              pre_position_count_, name.c_str(),
              side_sign_ > 0.0 ? "左" : "右", gap, olat, pre_lat, pre,
              room_lo_, room_hi_, f.ei);
    }
  }
  const bool hold_for_no_side =
    no_pass_lat_hold_ && no_pass_side_ &&
    !(attempt_active_ && attempt_target_ == name);
  if (hold_for_no_side && (f.now - last_no_side_log_).seconds() > 2.0) {
    last_no_side_log_ = f.now;
    ++no_side_hold_n_;
    diagLog("抜けないので出ない",
            "抜けないので出ない 累計%zu 相手=%s idx=%zu 自車%.1fkm/h "
            "(どちらの側も必要間隔に届かない)",
            no_side_hold_n_, name.c_str(), f.ei, my_speed_for_gap_ * 3.6);
  }
  if (!hold_for_no_side &&
      (allow || (attempt_active_ && attempt_target_ == name) ||
       ovPreparingTarget(name) || ovPassingTarget(name))) {
    const double sep_min = std::max(band_car_w_, min_pass_sep_);
    const double near_p = olat + side_sign_ * sep_min;
    const double far_p = (side_sign_ > 0.0) ? room_hi_ : room_lo_;
    const bool has_room = (side_sign_ > 0.0) ? (far_p > near_p) : (far_p < near_p);
    const bool execute_spot = spot_ready ||
      (spot_valid_ && name == spot_target_ && attempt_active_ &&
       attempt_target_ == spot_target_);
    // 予測地点で選んだ固定ラインを最後まで使う。現在の相手位置を基準に。
    double want_gap = pass_gap_;
    if (pass_gap_clear_ratio_ > 0.0) {
      const double w = geom_half_width_ * 2.0;               // 1.45
      const double sep_min_gap  = w + pass_gap_margin_;      // 輪郭 + 安全マージン
      const double sep_want_gap = sep_min_gap + w * pass_gap_clear_ratio_;
      // 相手からその側の帯の縁までの距離。ここから壁ぶんを残す。
      const double edge = (side_sign_ >= 0.0) ? room_hi_ : room_lo_;
      const double reach = std::abs(edge - olat);
      const double by_wall = std::max(sep_min_gap, reach - pass_wall_keep_);
      want_gap = std::min(sep_want_gap, by_wall);
      if (want_gap < sep_want_gap - 0.01 &&
          (f.now - last_pass_gap_log_).seconds() > 2.0)
      {
        last_pass_gap_log_ = f.now;
        diagLog("横の隙間",
          "横の隙間 target=%s 側=%s 相手横%.2f 帯[%.2f,%.2f] "
          "希望%.2f -> 壁の余地で%.2f (下限%.2f 壁に残す%.2f)",
          name.c_str(), side_sign_ > 0.0 ? "左" : "右", olat,
          room_lo_, room_hi_, sep_want_gap, want_gap, sep_min_gap,
          pass_wall_keep_);
      }
    }
    double tgt = execute_spot
                         ? spot_offset_
                         : ((pass_center_ && has_room)
                              ? 0.5 * (near_p + far_p)
                              : (olat + side_sign_ * want_gap));
    if (pass_sep_floor_ > 0.0) {
      const double need = std::max(sep_min, pass_sep_floor_);
      const double pushed = (side_sign_ > 0.0)
                              ? std::max(tgt, olat + need)
                              : std::min(tgt, olat - need);
      if (std::abs(pushed - tgt) > 0.01 &&
          (f.now - last_sep_floor_log_).seconds() > 2.0)
      {
        last_sep_floor_log_ = f.now;
        diagLog("横間隔の下限",
                "横間隔の下限 target=%s 側=%s 相手横=%.2f 要る間隔=%.2f "
                "横目標 %.2f -> %.2f (帯[%.2f,%.2f])",
                name.c_str(), side_sign_ > 0.0 ? "左" : "右", olat, need,
                tgt, pushed, room_lo_, room_hi_);
      }
      tgt = pushed;
    }
    if (ot_lane_inset_ >= 0.0 && side_sign_ < 0.0 &&
        otLaneApproach(f.ei, my_speed_for_gap_, rankSpeedCap()))
    {
      tgt = std::min(tgt, room_lo_ + ot_lane_inset_);
    }
    tgt_lat = std::clamp(tgt, room_lo_, room_hi_);
    bool npz_here = false;
    if (ot_abort_in_nopass_) {
      const std::size_t nn = corridor_.pass_ok.size();
      if (nn == f.n && f.ei < nn) {
        npz_here = !corridor_.pass_ok[f.ei];
        if (!npz_here && npz_look_by_lat_) {
          const double move = std::abs(tgt_lat - my_lat_for_target_);
          const double look = std::min(
            move / std::max(offset_rate_, 0.1) * std::max(my_speed_for_gap_, 1.0),
            npz_look_max_m_);
          double acc = 0.0;
          for (std::size_t k = 0; k < nn && acc < look; ++k) {
            const std::size_t i0 = (f.ei + k) % nn, i1 = (f.ei + k + 1) % nn;
            acc += std::hypot(
              f.in.points[i1].pose.position.x - f.in.points[i0].pose.position.x,
              f.in.points[i1].pose.position.y - f.in.points[i0].pose.position.y);
            if (!corridor_.pass_ok[i1]) { npz_here = true; break; }
          }
        }
      }
    }
    if (npz_here) {
      if ((f.now - last_npz_abort_log_).seconds() > 1.0) {
        last_npz_abort_log_ = f.now;
        diagLog("禁止区間で中断",
          "禁止区間で中断 idx=%zu 状態=%s 相手=%s 横目標%.2f を取り下げて基準へ戻す",
          f.ei, ovStateName(), name.c_str(), tgt_lat);
      }
    } else {
      c.requestLat(tgt_lat, PlanCtx::LatPrio::kOvertake, "追越");
    }
    // 試行中の横位置は「試行」が持つ。ここは値を作る場所であって、
    // 保持する場所ではない。作った値を試行の状態へ預ける。
    if (attempt_active_ && attempt_target_ == name) {
      attempt_lat_ = tgt_lat;
      attempt_lat_valid_ = true;
      attempt_lat_fresh_ = true;
    }
  }
  pass_sep_ = my_lat_for_target_ - olat;
  can_pass_now_ = allow;

  if (spot_valid_ && name == spot_target_ && spot_ready && !attempt_active_ &&
      !allow) {
    if (spot_stuck_since_ < 0.0) {
      spot_stuck_since_ = now.seconds();
    } else if (now.seconds() - spot_stuck_since_ >= spot_stuck_max_) {
      RCLCPP_INFO(get_logger(),
        "抜きどころ破棄 %s 残%.0fm 次を探す",
        spot_target_.c_str(), spot_dist_);
      spot_avoid_begin_ = spot_begin_;
      spot_avoid_until_ = now.seconds() + spot_avoid_sec_;
      spot_valid_ = false;
      spot_calc_at_ = -1.0;
      spot_stuck_since_ = -1.0;
    }
  } else if (!spot_ready || allow) {
    spot_stuck_since_ = -1.0;
  }

  ev.oi = oi; ev.gap = gap; ev.olat = olat;
  ev.ospeed_for_gate = ospeed_for_gate;
  ev.my_speed = my_speed; ev.v_reach = v_reach; ev.headroom = headroom;
  ev.clearly_slower = clearly_slower; ev.capped_leader = capped_leader;
  ev.boost_would_help = boost_would_help || timed_boost; ev.self_ok = self_ok;
  ev.lap_traffic = lap_traffic;
  ev.timed_plan = check_spot_path;
  ev.planned_spot_ready = spot_ready_planned;
  ev.accel_delay = accel_delay;
  ev.timed_boost = timed_boost;
  ev.allow = allow;
  ev.allow_commit = latch_commit_ ? allow : allow_base;
}

void V2XOvertaker::chargeBoost(const Frame & f, PlanCtx & c, const OppEval & ev)
{
  const rclcpp::Time now = f.now;
  const double gap = ev.gap;
  const bool allow = ev.allow;
  const double my_speed = ev.my_speed;
  const double v_reach = ev.v_reach;
  const double headroom = ev.headroom;
  const double ospeed_for_gate = ev.ospeed_for_gate;
  const bool clearly_slower = ev.clearly_slower;
  const bool capped_leader = ev.capped_leader;
  const bool boost_would_help = ev.boost_would_help;
  const bool self_ok = ev.self_ok;

  // 周回遅れのNPCは速度差だけで十分抜ける。真後ろからのブーストは
  // 横移動が崩れた際の追突エネルギーだけを増やすので使用しない。
  if (ev.lap_traffic) { return; }
  if (ev.timed_plan && ev.accel_delay < 0.0) { return; }
  if (ev.timed_boost &&
      std::abs(my_lat_for_target_ - ev.olat) < rear_end_free_min_) { return; }

  if (boost_runup_enable_ && !want_boost_ && boost_remaining_ > 0 &&
      !is_boosting_ && start_merge_done_ && my_speed > boost_min_speed_ &&
      headroom > boost_min_headroom_ && boostLapOk() &&
      (c.in_zone || ev.planned_spot_ready ||
       gate_boost_want_ || straight_need_boost_) &&
      gap >= boost_runup_gap_min_ && gap < boost_runup_gap_ &&
      ((boost_only_if_decisive_
          ? false
          : (capped_leader || clearly_slower || c.slow_leader)) ||
       gate_boost_want_ || straight_need_boost_ ||
       (pass_boost_gate_ && pass_need_boost_)))
  {
    const double since = (now - last_boost_time_).seconds();
    if (boost_used_ == 0 || since > boost_retry_sec_) {
      want_boost_ = true;
      RCLCPP_INFO(get_logger(),
        "ブースト使用(助走) target=%s 車間=%.1fm 相手=%.1fkm/h "
        "自車上限=%.1fkm/h 余地=%.1f 先頭ハンデ=%d 遅相手=%d "
        "抜き切り計算=%s %d周目 残り%d rank=%d",
        c.blocker.c_str(), gap, ospeed_for_gate * 3.6, v_reach * 3.6,
        headroom, capped_leader ? 1 : 0, clearly_slower ? 1 : 0,
        // 抜き切り距離の計算が。
        gate_boost_want_ ? "**門に間に合わせる**"
                         : (straight_need_boost_ ? "**直線内に抜き切る**"
                            : (pass_need_boost_ ? "**ブーストが要る**" : "-")),
        lap_ + 1, boost_remaining_, rank_);
    }
  }

  const bool moved_out = std::abs(offset_) > pass_gap_ * 0.5 || straight_ahead_;
  const bool start_phase_over = start_merge_done_;
  const bool fast_enough = my_speed > boost_min_speed_;
  const bool late_enough = lap_ >= boost_hold_laps_ && boostLapOk();
  if (allow && moved_out && start_phase_over && fast_enough && late_enough &&
      boost_remaining_ > 0 && !is_boosting_) {
    // 加速余地が無い場面(既に目標速度に達している)では撃たない。
    // 1位のときは 25 km/h で頭打ちなので直線ではまず余地が無い。
    const bool has_headroom = headroom > boost_min_headroom_;
    const bool need_boost = has_headroom && boost_would_help;
    if (need_boost) {
      const double since = (now - last_boost_time_).seconds();
      // 1個目を使った直後は効果を見る。効かなければ 2個目を許す
      if (boost_used_ == 0 || since > boost_retry_sec_) {
        want_boost_ = true;
        // この経路で撃てたことをログに残す。自由発射のログしか無かったため、
        // 「追い越しのために撃った」のか「余ったから撃った」のかを
        // 後から区別できなかった。
        RCLCPP_INFO(get_logger(),
          "ブースト使用(追い越し) target=%s 車間=%.1fm 短縮=%.1fs 自力=%d "
          "余地=%.1f %d周目 残り%d rank=%d",
          c.blocker.c_str(), gap, boost_gain_time_, self_ok ? 1 : 0,
          headroom, lap_ + 1, boost_remaining_, rank_);
      }
    }
  }
}

// 前の相手に対する車間制御と、抜き切りの判断。
//
// evaluateOpponent の最後の段。ここまでで「抜きにいってよいか(allow)」は
// 決まっているので、ここは速度上限と横間隔を実際に作る。
void V2XOvertaker::followAndCommit(const Frame & f, PlanCtx & c,
                     const OtherState & o, const OppEval & ev)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;
  const size_t oi = ev.oi;
  const double gap = ev.gap;
  const double olat = ev.olat;
  const double ospeed_for_gate = ev.ospeed_for_gate;
  const bool allow = ev.allow;

  // 前車の速度（進行方向成分）
  double tx, ty;
  {
    const auto & a = in.points[(oi + n - 1) % n].pose.position;
    const auto & b = in.points[(oi + 1) % n].pose.position;
    tx = b.x - a.x;
    ty = b.y - a.y;
    const double len = std::hypot(tx, ty);
    if (len > 1e-9) {
      tx /= len;
      ty /= len;
    }
  }
  double ospeed = o.vx * tx + o.vy * ty;

  double ospeed_pred = ospeed;
  if (predict_enable_) {
    double look = 0.0;
    for (size_t k = 0; k < n && look < predict_ahead_; ++k) {
      const size_t a = (oi + k) % n;
      const size_t b = (oi + k + 1) % n;
      look += std::hypot(in.points[b].pose.position.x - in.points[a].pose.position.x,
                         in.points[b].pose.position.y - in.points[a].pose.position.y);
      ospeed_pred = std::min<double>(ospeed_pred,
                                     in.points[a].longitudinal_velocity_mps);
      if (predict_lane_speed_) {
        const double lv = o.laneSpd(static_cast<int>(a * OtherState::kLatBins / n));
        if (lv > 0.0) { ospeed_pred = std::min(ospeed_pred, lv); }
      }
    }
    // 相手が今その速度を出せていないなら、それ以上は落ちないとみなす
    ospeed_pred = std::max(ospeed_pred, ospeed * predict_floor_);
  }
  // 車間に比例した追従制御。前車速度をそのまま上限にすると、
  // スタート直後のように全車が密集して止まっている場面で
  // 全員が 0 km/h に張り付いて膠着する。
  // 目標: 車間 safe_gap を保ちつつ、詰まっている分だけ減速する。
  // 車間は速度に応じて変える。
  // 低速でも一定の余裕を残しつつ、高速では前車が急停止しても止まれる距離を取る。
  // 制動距離 = v^2 / (2*|a_min|) を目安にし、上限で頭打ちにする。
  // 上限を設けるのは、開けすぎると追い越し機会を失い順位を落とすため。
  const double v_now = std::max(my_speed_for_gap_, 0.0);
  const double brake_dist = (v_now * v_now) / (2.0 * std::max(std::abs(a_min_), 0.5));
  double dyn_safe = std::clamp(safe_gap_ + brake_dist * gap_brake_ratio_,
                               safe_gap_min_, safe_gap_max_);
  if (normal_gap_by_rearend_ && !c.blocker.empty()) {
    const double v_want = std::min(
      static_cast<double>(in.points[ei].longitudinal_velocity_mps), rankSpeedCap());
    const double brake_a_n = std::max(std::abs(a_min_) * rear_end_brake_k_, 0.3);
    const double dv_n = std::max(v_want - ospeed, 0.0);
    const double need_n =
      rear_end_margin_ + v_want * rear_end_time_ + dv_n * dv_n / (2.0 * brake_a_n);
    const double target = std::min(need_n, normal_gap_max_);
    if (target > dyn_safe) {
      dyn_safe = target;
      if ((f.now - last_normal_gap_log_).seconds() > 3.0) {
        last_normal_gap_log_ = f.now;
        diagLog("通常の車間",
          "通常の車間 目標%.1fm (出したい%.1fkm/h 相手%.1fkm/h) "
          "従来の上限%.1fm idx=%zu",
          target, v_want * 3.6, ospeed * 3.6, safe_gap_max_, ei);
      }
    }
  }
  // 並走できない区間は少し広めに(詰めても抜けないうえ追突する)
  if (!can_pass_now_) {
    dyn_safe = std::min(dyn_safe * 1.3, safe_gap_max_);
  }
  // --- 仕掛けどころに合わせて車間を詰める ---
  //
  // 追突(Crash)は 10 秒間 5km/h に固定される。通常 35km/h で走ることを
  // 考えると 80m 以上の損失で、追い越し1回ぶんより遥かに重い。
  // だから普段は widely 空けておきたい。
  // 一方で、仕掛ける瞬間に車間が空いていては抜けない。
  //
  // 幸い最高速度は順位で決まっており(1位 25km/h / 2位以下 36km/h)、
  // 自分が2位なら前の1位より速い。この差で「いつでも詰められる」ので、
  // **仕掛けどころに着いた瞬間に車間が縮まっている**ように逆算する。
  //
  //   仕掛けどころまでの距離 d、詰める速度差 closing、自車速度 v のとき
  //   到達までの時間 t = d / v、その間に詰められる量 = closing * t
  //   よって今保ってよい車間 = 目標車間 + closing * t
  double eff_safe = dyn_safe;
  bool charge_now = false;
  const bool predictive_approach = spot_enable_ && spot_valid_ &&
                                   c.blocker == spot_target_;
  // 周回遅れNPCには助走で車間を詰めない。予測した側へ出られない周期でも
  // 安全車間を維持し、ラインが空いた周期に横移動を先行させる。
  if (approach_enable_ && !can_pass_now_ && !ev.lap_traffic) {
    double d_zone = -1.0;
    if (spot_enable_ && spot_valid_ && c.blocker == spot_target_ &&
        spot_dist_ >= 0.0 && spot_dist_ <= approach_range_) {
      d_zone = std::max(spot_dist_, 0.0);
    }
    if (runup_use_plan_ && plan_start_valid_ && plan_start_target_ == c.blocker) {
      double acc_p = 0.0;
      bool found = false;
      for (std::size_t k = 1; k < n; ++k) {
        const std::size_t a = (ei + k - 1) % n, b = (ei + k) % n;
        acc_p += std::hypot(
          in.points[b].pose.position.x - in.points[a].pose.position.x,
          in.points[b].pose.position.y - in.points[a].pose.position.y);
        if (b == plan_start_idx_) { found = true; break; }
        if (acc_p > approach_range_) { break; }
      }
      if (found) {
        d_zone = acc_p;
      } else {
        // 入口を通り過ぎた(前方 approach_range_ の中に無い)。
        // もう待つ地点ではないので、計画の助走はここで終わり。
        plan_start_valid_ = false;
      }
    }
    if (d_zone < 0.0) {
      double acc = 0.0;
      for (size_t k = 1; k < n; ++k) {
        const size_t a = (ei + k - 1) % n, b = (ei + k) % n;
        acc += std::hypot(in.points[b].pose.position.x - in.points[a].pose.position.x,
                          in.points[b].pose.position.y - in.points[a].pose.position.y);
        if (acc > approach_range_) { break; }
        bool zone_here = (corridor_.pass_ok.size() == n && corridor_.pass_ok[b]);
        if (!zone_here) {
          for (const auto & z : boost_zones_) {
            const bool inside = (z.first <= z.second)
                                  ? (b >= z.first && b <= z.second)
                                  : (b >= z.first || b <= z.second);
            if (inside) { zone_here = true; break; }
          }
        }
        if (zone_here) { d_zone = acc; break; }
      }
    }
    if (d_zone > 0.0) {
      const double rank_cap = ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
      const int zsec = static_cast<int>(oi * OtherState::kSections / n);
      double o_zone_v = o.sectionSpeed(zsec);
      // 抜きどころが決まっているなら、その地点の**録画速度**で見積もる。
      // 逆算の目的は「入口に着いたときに pass_gap になっている」ことなので、
      // 今いる場所の速度ではなく入口での速度で詰まる量を計算するのが正しい。
      if (spot_enable_ && spot_valid_ && c.blocker == spot_target_ &&
          spot_ospeed_ > 0.0) {
        o_zone_v = std::max(o_zone_v, spot_ospeed_);
      }
      const double o_ref = (o_zone_v > 0.0) ? std::max(ospeed, o_zone_v) : ospeed;
      const double closing = std::max(rank_cap - o_ref, 0.0);
      const double t_zone = d_zone / std::max(v_now, 1.0);
      // 仕掛けどころで pass_gap になるように、今は余分に空けておく
      const double want = pass_gap_ + closing * t_zone;
      eff_safe = std::clamp(std::max(eff_safe, want), safe_gap_min_, approach_gap_max_);
      bool runup_charge = false;
      dbg_runup_reached_ = false;
      // 毎周期リセットする。残したままだと一度助走に入っただけで
      // follow_safe の割引が永久に外れる。
      runup_holding_gap_ = 0.0;
      if (runup_enable_) {
        dbg_runup_reached_ = true;   // 観測のみ
        // 助走の加速度をブースト対応にする。
        const double a_eff = runupAccelMps2();
        const double v_self_now = std::max(std::abs(f.ev), 0.5);
        const double v_opp = std::max(o_ref, 0.5);      // 既存の相手速度見積り
        double v_tgt = std::min(v_opp + runup_dv_, rank_cap);
        double d_ref = d_zone;
        double d_gate = -1.0;
        if (ot_lane_runup_ && ot_lane_enable_ && !ot_lane_zones_.empty() &&
            !inOtLane(f.ei)) {
          d_gate = otLaneSpeedDeadlineDistance(f.ei, ot_lane_runup_look_);
          if (d_gate >= 0.0) {
            d_gate = std::max(d_gate - ot_lane_entry_pre_m_, 0.0);
          }
        }
        const double v_gate = (ot_lane_min_kmh_ + ot_lane_runup_margin_kmh_) / 3.6;
        const bool in_lane_now = inOtLane(f.ei);
        if (gate_runup_in_lane_ && in_lane_now && v_gate <= rank_cap) {
          d_gate = 0.0;
        }
        const bool gate_runup =
          ((d_gate >= 0.0) || (gate_runup_in_lane_ && in_lane_now)) &&
          (v_gate <= rank_cap);
        if (gate_runup) {
          v_tgt = gate_runup_target_gate_
                    ? std::min(std::max(v_tgt, v_gate), v_gate)
                    : std::max(v_tgt, v_gate);
          d_ref = gate_runup_ref_gate_ ? d_gate : std::min(d_ref, d_gate);
        }
        dbg_gate_runup_ = gate_runup;
        dbg_d_gate_ = d_gate;
        dbg_runup_vtgt_ = v_tgt;
        dbg_runup_dref_ = d_ref;
        gate_boost_want_ = false;
        if (gate_runup_charge_ && gate_runup) {
          // runupAccelMps2() と同じ基準値を使う。
          const double a_plain = std::max(runupAccelBase(), 0.05);
          const double a_boost = a_plain + boost_accel_;
          const double dv2 = v_gate * v_gate - v_self_now * v_self_now;
          const double need_plain = (dv2 > 0.0) ? dv2 / (2.0 * a_plain) : 0.0;
          const double need_boost = (dv2 > 0.0) ? dv2 / (2.0 * a_boost) : 0.0;
          const bool boost_avail = boost_remaining_ > 0 && boostLapOk();
          const double need_now =
            (boost_avail ? need_boost : need_plain) + gate_runup_margin_m_;
          if (d_gate <= need_now) {
            runup_charge = true;               // 車間に関わらず全開
            gate_boost_want_ =
              boost_avail &&
              (d_gate < need_plain) &&      // 素では届かない
              (d_gate >= need_boost);       // ブーストなら届く
            if (gate_runup_sidestep_ &&
                (start_merge_done_ || !gate_sidestep_after_merge_)) {
              const double need_sep = v2x_overtaker::physicalPassSeparation(
                pass_beside_sep_, rear_end_free_min_);
              const double sgn = (side_sign_ >= 0.0) ? +1.0 : -1.0;
              const double aim = olat + sgn * (need_sep + gate_sidestep_extra_);
              c.requestLat(aim, PlanCtx::LatPrio::kPrePosition,
                           "門への助走(横へ逃がす)");
            }
            if ((now - last_gate_boost_log_).seconds() > 2.0) {
              last_gate_boost_log_ = now;
              diagLog("門への助走",
                "門への助走 入口まで%.1fm 自車%.1fkm/h 門%.1fkm/h "
                "必要距離 素%.1fm ブースト%.1fm(+余裕%.1f) -> 全開 %s idx=%zu",
                d_gate, v_self_now * 3.6, v_gate * 3.6, need_plain, need_boost,
                gate_runup_margin_m_,
                gate_boost_want_ ? "**ブーストを使う**" : "ブーストなし", ei);
            }
          }
        }
        if (v_tgt > v_self_now) {
          const double t_acc = (v_tgt - v_self_now) / a_eff;
          const double d_self = v_self_now * t_acc + 0.5 * a_eff * t_acc * t_acc;
          const double d_opp  = v_opp * t_acc;
          const double shrink = std::max(d_self - d_opp, 0.0);
          const double brake_a_re = std::max(std::abs(a_min_) * rear_end_brake_k_, 0.3);
          const double dv_hold = std::max(v_tgt - v_opp, 0.0);
          const double room_hold = dv_hold * dv_hold / (2.0 * brake_a_re);
          const double gap_hold =
            rear_end_margin_ + v_tgt * rear_end_time_ + room_hold;
          const double need_gap =
            std::min(shrink + (runup_gap_hold_ ? std::max(pass_gap_, gap_hold)
                                               : pass_gap_),
                     runup_gap_max_);
          double accel_at = d_self + runup_margin_;
          if (gate_runup_late_ && gate_runup) {
            const double dv2g = v_gate * v_gate - v_self_now * v_self_now;
            const double d_gate_need = (dv2g > 0.0) ? dv2g / (2.0 * a_eff) : 0.0;
            accel_at = std::min(accel_at, d_gate_need + runup_margin_);
          }
          if (runup_sim_ && gate_runup && d_gate > 0.0) {
            v2x_overtaker::RunupSimIn si;
            si.v_now = v_self_now;
            si.v_opp = v_opp;
            std::array<double, 81> opp_v{};
            int opp_n = 0;
            if (runup_sim_opp_model_ && o.modelReady()) {
              double so = 0.0;               // 相手の進んだ距離
              std::size_t oi_k = oi;
              for (int k2 = 0; k2 < static_cast<int>(opp_v.size()); ++k2) {
                const double R = curveRadiusAt(oi_k);
                double vk = o.modelSpd(R);
                if (vk <= 0.0) { vk = v_opp; }
                // 経路の指示速度も上限として効かせる(相手も同じ道を走る)
                const double vmap = in.points[oi_k].longitudinal_velocity_mps;
                if (vmap > 0.1) { vk = std::min(vk, static_cast<double>(vmap)); }
                const double opp_cap =
                  ((!cur_leader_.empty() && c.blocker == cur_leader_)
                     ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
                const double t_k = 0.1 * static_cast<double>(k2);
                vk = std::min(std::min(vk, v_opp + opp_accel_mps2_ * t_k), opp_cap);
                opp_v[k2] = std::max(vk, 0.0);
                so += vk * 0.1;
                // so に対応する idx まで進める
                while (true) {
                  const std::size_t nx = (oi_k + 1) % n;
                  const double step = std::hypot(
                    in.points[nx].pose.position.x - in.points[oi_k].pose.position.x,
                    in.points[nx].pose.position.y - in.points[oi_k].pose.position.y);
                  if (so < step) { break; }
                  so -= step; oi_k = nx;
                }
                opp_n = k2 + 1;
              }
              si.opp_v = opp_v.data();
              si.opp_n = opp_n;
            }
            si.gap   = gap;
            si.body  = runup_fuel_body_ ? (geom_front_ + geom_rear_) : 0.0;
            si.dist  = d_gate;
            si.a_max = a_eff;
            si.v_cap = rank_cap;
            si.rear_margin = rear_end_margin_;
            si.rear_time   = rear_end_time_;
            si.brake_a = std::max(std::abs(a_min_) * rear_end_brake_k_, 0.3);
            const auto so = v2x_overtaker::simulateRunup(si);
            if (so.valid) {
              // 入口で門を超えられるなら点火。超えられないなら待つ。
              const bool reach =
                so.v_at_dist >= v_gate - runup_gate_slack_kmh_ / 3.6;
              runup_sim_reach_ = reach;
              runup_sim_v_ = so.v_at_dist;
              runup_sim_min_gap_ = so.min_gap;
              if ((now - last_sim_log_).seconds() > 2.0) {
                last_sim_log_ = now;
                diagLog("助走の試算",
                  "助走の試算 入口まで%.1fm 自車%.1fkm/h 相手%.1fkm/h 車間%.1fm "
                  "-> 入口で%.1fkm/h(門%.1f) 最小車間%.1fm 頭打ち=%d "
                  "相手予測[初%.1f 終%.1f]km/h -> %s",
                  d_gate, v_self_now * 3.6, v_opp * 3.6, gap,
                  so.v_at_dist * 3.6, v_gate * 3.6, so.min_gap,
                  so.capped ? 1 : 0,
                  (opp_n > 0 ? opp_v[0] : v_opp) * 3.6,
                  (opp_n > 0 ? opp_v[opp_n - 1] : v_opp) * 3.6,
                  reach ? "点火" : "待つ");
              }
              accel_at = reach ? 1e9 : -1.0;   // 点火 / 待つ
            }
          }
          double accel_at_fuel = accel_at;
          if (runup_fuel_ && !runup_sim_ && gate_runup) {
            const double brake_a_re2 =
              std::max(std::abs(a_min_) * rear_end_brake_k_, 0.3);
            const double dv_hold2 = std::max(v_tgt - v_opp, 0.0);
            const double thr =
              rear_end_margin_ + v_tgt * rear_end_time_ +
              dv_hold2 * dv_hold2 / (2.0 * brake_a_re2);
            // 車体長が抜けていた。
            const double body = runup_fuel_body_ ? (geom_front_ + geom_rear_) : 0.0;
            const double fuel = std::max(gap - thr - body, 0.0);
            if (fuel > 0.01) {
              // (v0-v_opp)T + a T^2/2 = fuel を T について解く
              const double dv0 = v_self_now - v_opp;
              const double disc = dv0 * dv0 + 2.0 * a_eff * fuel;
              const double T = (std::sqrt(std::max(disc, 0.0)) - dv0) / a_eff;
              if (T > 0.0 && std::isfinite(T)) {
                accel_at_fuel = v_self_now * T + 0.5 * a_eff * T * T;
              }
            } else {
              accel_at_fuel = 0.0;   // 滑走路が無い。まだ点火しない
            }
            accel_at = std::min(accel_at, accel_at_fuel);
            if ((now - last_fuel_log_).seconds() > 2.0) {
              last_fuel_log_ = now;
              diagLog("加速の点火",
                "加速の点火 車間%.1fm - 閾値%.1fm = 滑走路%.1fm -> 点火距離%.1fm "
                "(従来%.1fm) 入口まで%.1fm 目標%.1fkm/h 自車%.1fkm/h",
                gap, thr, std::max(gap - thr, 0.0), accel_at_fuel,
                d_self + runup_margin_, d_gate, v_tgt * 3.6, v_self_now * 3.6);
            }
          }
          bool hold_for_gap = false;
          if (runup_hold_until_gap_) {
            const double d_ent_raw = otLaneSpeedDeadlineDistance(f.ei, -1.0);
            const double d_tgt = (d_ent_raw >= 0.0)
              ? std::max(d_ent_raw - ot_lane_entry_pre_m_, 0.0) : -1.0;
            const double v_goal =
              (ot_lane_min_kmh_ + ot_lane_runup_margin_kmh_) / 3.6;
            const double a_eff2 = runupAccelMps2();
            const double v_now2 = std::max(my_speed_for_gap_, 0.5);
            double d_need = 0.0;
            if (v_goal > v_now2) {
              d_need = (v_goal * v_goal - v_now2 * v_now2) / (2.0 * a_eff2);
            }
            d_need += v_now2 * runup_react_sec_;
            // 横へ寄るのに要る距離。レーンに入るには横に動く必要があり、
            // 速度が間に合っても横が間に合わなければ入れない。
            double lat_need = 0.0;
            if (runup_hold_lat_ && d_ent_raw >= 0.0) {
              double wlo = 0.0, whi = 0.0;
              std::size_t aim_i = f.ei;
              for (std::size_t k = 0; k < f.n; ++k) {
                const std::size_t j = (f.ei + k) % f.n;
                if (otLaneAimWindow(j, wlo, whi)) { aim_i = j; break; }
              }
              if (whi > wlo || std::abs(whi) > 1e-9) {
                const double want_lat = whi - ot_lane_aim_inset_;
                const double move = std::abs(want_lat - my_lat_for_target_);
                lat_need = move / std::max(offset_rate_, 0.1) * v_now2;
              }
            }
            const double d_need_all = std::max(d_need, lat_need);
            dbg_want_hold_ = d_need_all;
            // 残りが要る距離より十分長い＝まだ待てる。
            if (d_tgt > 0.0 && d_tgt > d_need_all * runup_hold_margin_k_) {
              hold_for_gap = true;
            }
            if ((now - last_runup_time_log_).seconds() > 1.0) {
              last_runup_time_log_ = now;
              diagLog("助走の点火時期",
                "助走の点火時期 idx=%zu 目標まで%.1fm 要る距離%.1fm"
                "(速度%.1f 横%.1f 遅れ%.1f) 自車%.1fkm/h 目標%.1fkm/h "
                "加速度%.2f ブースト=%d 車間%.1fm -> %s",
                f.ei, d_tgt, d_need_all, d_need - v_now2 * runup_react_sec_,
                lat_need, v_now2 * runup_react_sec_, v_now2 * 3.6,
                v_goal * 3.6, a_eff2, is_boosting_ ? 1 : 0, gap,
                hold_for_gap ? "待つ(車間を開ける)" : "点火してよい");
            }
          }
          dbg_hold_for_gap_ = hold_for_gap;
          if (d_ref > accel_at || hold_for_gap) {
            // まだ加速するには早い。車間を開けて待つ。
            const double cap = runup_gap_cap_
                                 ? std::max(approach_gap_max_, runup_gap_max_)
                                 : approach_gap_max_;
            eff_safe = std::clamp(std::max(eff_safe, need_gap),
                                  safe_gap_min_, cap);
            runup_holding_gap_ = std::min(
              std::max(need_gap * runup_hold_gap_k_, runup_hold_gap_abs_),
              runup_gap_max_);   // 観測用
            // ここで先行の runup_charge を打ち消す。
          } else {
            // 加速開始点に達した。全開で速度を作る。
            runup_charge = true;
          }
          dbg_runup_charge_ = runup_charge;
          dbg_runup_accel_at_ = accel_at;
          dbg_eff_safe_ = eff_safe;
          dbg_gap_ = gap;
          runup_need_gap_ = need_gap;      // 観測用
          runup_accel_at_ = accel_at;
          // 状態が変わったときだけ2秒に1回出す。
          const char * st = runup_charge ? "加速" : "待機";
          const double since = (now - runup_log_last_).seconds();
          if (runup_log_state_ != st && since >= 2.0) {
            runup_log_state_ = st;
            runup_log_last_ = now;
            diagLog("助走",
              "助走 target=%s 抜きどころまで=%.1fm(出所=%s 狙い横=%.2f) 車間=%.1fm 要車間=%.1fm "
              "加速開始=%.1fm 自車=%.1fkm/h 相手=%.1fkm/h 目標=%.1fkm/h 状態=%s"
              " レーン入口まで=%.1fm 門助走=%d 保持に要る車間=%.1fm",
              c.blocker.c_str(), d_zone,
              (runup_use_plan_ && plan_start_valid_ &&
               plan_start_target_ == c.blocker) ? "計画" : "録画/区間",
              plan_y_target_, gap, need_gap, accel_at,
              v_self_now * 3.6, v_opp * 3.6, v_tgt * 3.6, st,
              d_gate, gate_runup ? 1 : 0, gap_hold);
          }
        }
      }
      const double predictive_start_gap = rear_end_margin_ + 1.5;
      if (predictive_approach) {
        eff_safe = std::max(eff_safe, predictive_start_gap);
      }
      // 車間が目標より大きければ助走(全開で詰める)。
      charge_now = gap > eff_safe + 0.3;
      if (d_zone < charge_close_dist_) {
        const double close_goal = predictive_approach
          ? predictive_start_gap : pass_gap_ * charge_close_gap_k_;
        charge_now = gap > close_goal;
      }
      if (runup_charge) { charge_now = true; }
      if (runup_hold_block_charge_ && dbg_hold_for_gap_) { charge_now = false; }
      dbg_charge_now_ = charge_now;   // 助走の上書き後の値を残す(観測のみ)
    if (accel_audit_ && (f.now - last_accel_audit_log_).seconds() > 1.0) {
      last_accel_audit_log_ = f.now;
      diagLog("加速の判断",
        "加速の判断 全開=%d 内訳[助走=%d 車間=%.1fm>要求%.1fm+0.3 予測接近=%d 保留=%d 目標車間%.1fm] "
        "助走[到達=%d 門助走=%d 目標%.1fkm/h 加速開始%.1fm 基準%.1fm] "
        "ブースト[要求=%d 残%d 発射中=%d 門要=%d 直線要=%d 抜切要=%d] "
        "自車%.1fkm/h 相手%.1fkm/h idx=%zu",
        charge_now ? 1 : 0, runup_charge ? 1 : 0, gap, eff_safe,
        predictive_approach ? 1 : 0, dbg_hold_for_gap_ ? 1 : 0, dbg_want_hold_,
        dbg_runup_reached_ ? 1 : 0, dbg_gate_runup_ ? 1 : 0,
        dbg_runup_vtgt_ * 3.6, dbg_runup_accel_at_, dbg_runup_dref_,
        want_boost_ ? 1 : 0, boost_remaining_, is_boosting_ ? 1 : 0,
        gate_boost_want_ ? 1 : 0, straight_need_boost_ ? 1 : 0,
        pass_need_boost_ ? 1 : 0,
        std::abs(f.ev) * 3.6, ospeed * 3.6, f.ei);
    }
      if (runup_charge && attempt_active_) { attempt_runup_used_ = true; }
      const double closing_now = std::max(my_speed_for_gap_ - ospeed, 0.0);
      const double brake_need =
        closing_now * closing_now / (2.0 * std::max(std::abs(a_min_), 0.5));
      if (gap < brake_need + charge_brake_margin_) {
        charge_now = false;
      }
    }
  }
  // --- 予測から逆算した加速開始時刻を速度制御へ接続する ---
  // accel_delay>0 の間は相手速度で待ち、0.25秒以内になったら横間隔が
  // 車幅+余裕まで確保できた場合だけ追従キャップを外す。横へ出る前の全開は
  // 追突になるため、時刻条件だけでは解除しない。
  bool predictive_accel = false;
  if (ev.timed_plan && allow && ev.accel_delay >= 0.0) {
    const double lat_sep = std::abs(my_lat_for_target_ - olat);
    if (lat_sep >= rear_end_free_min_) {
      charge_now = true;
      predictive_accel = true;
    }
  }
  if (predictive_accel) {
    if (!spot_accel_active_ || spot_accel_target_ != c.blocker) {
      RCLCPP_INFO(get_logger(),
        "予測加速開始 target=%s 車間=%.1fm 横間隔=%.2fm 地点まで=%.1fm 出口まで=%.1fm",
        c.blocker.c_str(), gap, std::abs(my_lat_for_target_ - olat),
        spot_dist_, spot_dist_ + spot_len_);
    }
    spot_accel_active_ = true;
    spot_accel_target_ = c.blocker;
  } else if (!attempt_active_) {
    // 同一周期に複数車を評価しても、進行中の対象の状態を別車で消さない。
    spot_accel_active_ = false;
    spot_accel_target_.clear();
  }

  const double eff_follow = std::min(dyn_safe * 1.8, safe_gap_max_ * 1.8);
  bool passing_now = allow && std::abs(offset_) > pass_gap_ * 0.5;
  if (ospeed_for_gate < slow_leader_speed_) { passing_now = false; }
  const bool launch_pass_now =
      launchPassActive() && (&o == launchNpc()) &&
      std::abs(my_lat_for_target_ - olat) >= launch_p1_pass_sep_ && gap > 1.0;
  if (launch_pass_now) { passing_now = true; }
  if (straight_pass_now_ && allow && ospeed_for_gate >= stopped_speed_) {
    passing_now = true;
  }
  {
    const double lat_sep_pre = std::abs(my_lat_for_target_ - olat);
    if (ospeed_for_gate < stopped_speed_ && lat_sep_pre >= commit_sep_ + 0.2 &&
        gap > 1.0) {
      passing_now = true;
    }
  }
  // 追い越し中でなければ、相手がこれから落とす速度に合わせる。
  // 追い越し中に効かせると、相手の減速に自分も付き合って並んだまま
  // 抜けなくなるので、そのときは今の速度のまま扱う。
  if (!passing_now) { ospeed = std::min(ospeed, ospeed_pred); }
  // 追い越し中も追従を完全には切らない。切ると相手が止まっても。
  double follow_safe =
    (passing_now && !(runup_gap_cap_ && runup_holding_gap_ > 0.0))
      ? eff_safe * 0.6 : eff_safe;
  // 点火中(助走で詰めに行っている間)は、追従の。
  if (runup_charge_relax_ && dbg_runup_charge_) {
    follow_safe = std::min(follow_safe, runup_charge_follow_gap_);
  }

  const double lat_sep_now = my_lat_for_target_ - olat;
  bool commit_now = false;
  if (commit_pass_ && ev.allow_commit &&
      (ospeed_for_gate > slow_leader_speed_ || launch_pass_now)) {
    // 解除側でも**車幅を下回らせない**。重なった状態で加速を続けないため。
    const double need_sep = v2x_overtaker::physicalPassSeparation(
      commit_now_ ? std::max(commit_sep_ * commit_release_, kCarWidth) : commit_sep_,
      rear_end_free_min_);
    const double need_gap = commit_now_ ? commit_gap_ * 1.5 : commit_gap_;
    commit_now = (std::abs(lat_sep_now) >= need_sep) && (gap < need_gap) &&
                 sideStaysOpen(f, offset_, commit_look_);
  }
  if (commit_now && !commit_now_) {
    commit_since_ = now.seconds();
    RCLCPP_INFO(get_logger(),
                "並走から抜き切りへ target=%s 車間=%.1fm 横間隔=%.2fm "
                "相手=%.1fkm/h 上限解除 rank=%d idx=%zu",
                c.blocker.c_str(), gap, lat_sep_now, ospeed * 3.6, rank_, ei);
  }
  if (!commit_now) { commit_since_ = -1.0; }
  commit_now_ = commit_now;

  const bool truly_side_by_side = gap < boost_side_gap_;
  if (commit_now && truly_side_by_side &&
      commit_boost_time_ > 0.0 && commit_since_ > 0.0 &&
      (now.seconds() - commit_since_) >= commit_boost_time_ &&
      boostLapOk() && boost_remaining_ > 0 && !is_boosting_ && !want_boost_)
  {
    const double since_last = (now - last_boost_time_).seconds();
    if (boost_used_ == 0 || since_last > boost_retry_sec_) {
      want_boost_ = true;
      RCLCPP_INFO(get_logger(),
                  "ブースト要求(並走%.1fs 抜き切れず) target=%s 横間隔=%.2fm "
                  "車間=%.1fm 相手=%.1fkm/h 残り%d rank=%d",
                  now.seconds() - commit_since_, c.blocker.c_str(),
                  lat_sep_now, gap, ospeed * 3.6, boost_remaining_, rank_);
    }
  }
  if (attempt_active_ && std::abs(lat_sep_now) > attempt_max_sep_) {
    attempt_max_sep_ = std::abs(lat_sep_now);
  }
  const bool launch_free =
      v2x_overtaker::launchWindowActive(
        launch_motion_since_, now.seconds(), launch_free_sec_) &&
      gap > launch_free_gap_ && [&]() {
        const double closing = my_speed_for_gap_ - std::max(ospeed, 0.0);
        if (closing <= 0.0) { return true; }
        const double react = closing * launch_free_react_;
        const double brake = closing * closing / (2.0 * std::max(launch_free_decel_, 0.1));
        return (react + brake) < gap - launch_free_room_;
      }();
  if (gap < eff_follow && !charge_now && !commit_now && !launch_free) {
    double v_target = ospeed + follow_kp_ * (gap - follow_safe);
    double floor = min_follow_speed_;
    if (gap < follow_safe * 0.5 && ospeed > stopped_speed_) {
      floor = 0.0;
    } else if (ospeed <= stopped_speed_ && wouldRearEnd(gap, ospeed)) {
      // 相手が止まっていて、ぶつかる距離にいる
      if (stop_hold_since_ < 0.0) { stop_hold_since_ = now.seconds(); }
      if ((now.seconds() - stop_hold_since_) < stop_hold_sec_) {
        floor = 0.0;                     // まだ待つ。突っ込まない
      }
    } else {
      stop_hold_since_ = -1.0;
    }
    // 後ろから詰められているときは、前車の速度を下回るまで落とさない。
    {
      double f_press = std::max(ospeed, 0.0);
      bool open_zone = true;
      if (runup_open_from_idx_ >= 0) {
        const int ei_i = static_cast<int>(f.ei);
        const int lo = runup_open_from_idx_, hi = runup_open_to_idx_;
        open_zone = (lo <= hi) ? (ei_i >= lo && ei_i <= hi)
                               : (ei_i >= lo || ei_i <= hi);
      }
      const bool open_leader_ok =
        !runup_open_leader_only_ ||
        (!cur_leader_.empty() && c.blocker == cur_leader_);
      // 後方車の条件を削除する。
      const bool rear_ok =
        (runup_open_rear_min_ < 0.0) || (c.rear_gap > runup_open_rear_min_);
      const bool opening =
        runup_gap_open_ && runup_holding_gap_ > 0.0 &&
        gap < runup_holding_gap_ && rear_ok &&
        open_zone && open_leader_ok;
      if (opening) {
        // 必要な減速量を「残り距離」から逆算する。
        double dv_need = runup_open_dv_;
        if (runup_open_dv_auto_) {
          const double shortfall = std::max(runup_holding_gap_ - gap, 0.0);
          // ここだけ長い地平(runup_open_look_m_)を使う。
          const double d_ent =
            otLaneSpeedDeadlineDistance(f.ei, runup_open_look_m_);
          const double v_use = std::max(my_speed_for_gap_, 3.0);
          if (shortfall > 0.05 && d_ent > 1.0) {
            const double t_left = d_ent / v_use;
            if (t_left > 0.2) {
              dv_need = std::clamp(shortfall / t_left,
                                   runup_open_dv_, runup_open_dv_max_);
            } else {
              dv_need = runup_open_dv_max_;   // もう時間が無い
            }
          }
        }
        double v_floor_gate = 0.0;
        if (runup_open_keep_reach_) {
          const double d_ent2 = otLaneSpeedDeadlineDistance(f.ei, -1.0);
          const double d_t = (d_ent2 >= 0.0)
            ? std::max(d_ent2 - ot_lane_entry_pre_m_, 0.0) : -1.0;
          if (d_t > 0.0) {
            const double vg = (ot_lane_min_kmh_ + ot_lane_runup_margin_kmh_) / 3.6;
            const double a2 = runupAccelMps2();
            const double b = 2.0 * a2 * runup_react_sec_;
            const double cq = 2.0 * a2 * d_t - vg * vg;
            const double disc = b * b - 4.0 * cq;
            if (disc >= 0.0) {
              const double root = (b + std::sqrt(disc)) * 0.5;
              if (std::isfinite(root) && root > 0.0) { v_floor_gate = root; }
            } else {
              // 判別式が負 = 「いまの速度がいくらでも。
              v_floor_gate = 0.0;
            }
          }
        }
        f_press = std::max(std::max(0.0, ospeed - dv_need), v_floor_gate);
        if (runup_gap_open_cmd_) {
          c.requestCap(std::max(f_press, 0.0), "車間を開ける");
        }
        if ((now - last_gap_open_log_).seconds() > 2.0) {
          last_gap_open_log_ = now;
          diagLog("車間を開ける",
            "車間を開ける target=%s 車間%.1fm 目標%.1fm 後方%.1fm "
            "相手%.1fkm/h 下限 %.1f -> %.1fkm/h",
            c.blocker.c_str(), gap, runup_holding_gap_, c.rear_gap,
            ospeed * 3.6, std::max(ospeed, 0.0) * 3.6, f_press * 3.6);
        }
      }
      if (c.pressed_from_behind && gap >= follow_safe * 0.5) {
        floor = std::max(floor, f_press);
      }
    }
    if (gap > follow_keep_gap_ && ospeed > 0.0) {
      floor = std::max(floor, ospeed);
    }
    if (passUnderway(c.blocker, std::abs(lat_sep_now))) {
      v_target = std::max(v_target, ospeed);
    }
    const bool clear_of_prepare_target =
      ovPreparingTarget(c.blocker) &&
      std::abs(lat_sep_now) >= pass_side_clear_;
    if (!ovPassingTarget(c.blocker) && !clear_of_prepare_target) {
      const bool follow_skip_stopped =
        follow_skip_stop_pass_ && c_stop_avoid_pass_ &&
        !c_stop_avoid_target_.empty() && c.blocker == c_stop_avoid_target_;
      if (!follow_skip_stopped) {
        c.requestCap(std::max(floor, v_target), "追従");
      } else if ((now - last_follow_skip_log_).seconds() > 1.0) {
        last_follow_skip_log_ = now;
        diagLog("追従を外す",
          "追従を外す 相手=%s は停止車回避が通せると判定。追従の上限%.1fkm/h を出さない",
          c.blocker.c_str(), std::max(floor, v_target) * 3.6);
      }
    }
  }
}


// ===================================================================
// 当たらないようにする層
// ===================================================================

bool V2XOvertaker::otLaneAhead(std::size_t idx, double look) const
{
  if (ot_lane_zones_.empty() || line_x_.empty()) { return false; }
  const std::size_t n = line_x_.size();
  double acc = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t a = (idx + k) % n;
    if (inOtLane(a)) { return true; }
    const std::size_t b = (idx + k + 1) % n;
    acc += std::hypot(line_x_[b] - line_x_[a], line_y_[b] - line_y_[a]);
    if (acc > look) { break; }
  }
  return false;
}

bool V2XOvertaker::inOtLane(std::size_t idx) const
{
  for (const auto & z : ot_lane_zones_) {
    const bool inside = (z.first <= z.second)
                          ? (idx >= z.first && idx <= z.second)
                          : (idx >= z.first || idx <= z.second);
    if (inside) { return true; }
  }
  return false;
}

// 車体を丸ごとレーンへ入れられる区間の中か(読み込み時のコメント参照)。
bool V2XOvertaker::inOtLaneUse(std::size_t idx) const
{
  const auto & zs = ot_lane_use_zones_.empty() ? ot_lane_zones_ : ot_lane_use_zones_;
  for (const auto & z : zs) {
    const bool inside = (z.first <= z.second)
                          ? (idx >= z.first && idx <= z.second)
                          : (idx >= z.first || idx <= z.second);
    if (inside) { return true; }
  }
  return false;
}

double V2XOvertaker::rankSpeedCap() const
{
  return ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
}

bool V2XOvertaker::laneGuardSuppressed(const Frame & f) const
{
  if (!lane_guard_start_off_) { return false; }
  if (start_merge_done_ && lap_ >= 1) { return false; }
  const int ei = static_cast<int>(f.ei);
  const int lo = lane_guard_off_from_idx_, hi = lane_guard_off_to_idx_;
  const bool in = (lo <= hi) ? (ei >= lo && ei <= hi) : (ei >= lo || ei <= hi);
  return in;
}

// 助走が使う加速度[m/s^2]。ブースト分の加算も含めた唯一の計算箇所。
double V2XOvertaker::runupAccelBase() const
{
  return (runup_accel_mps2_ > 0.0) ? runup_accel_mps2_ : vehicle_accel_ * 0.60;
}

double V2XOvertaker::runupAccelMps2() const
{
  return std::max(
    runupAccelBase() +
      ((runup_boost_accel_ && is_boosting_) ? boost_accel_ : 0.0),
    0.05);
}

double V2XOvertaker::otLaneEntryDistance(std::size_t idx, double look) const
{
  if (ot_lane_zones_.empty() || line_x_.empty()) { return -1.0; }
  if (inOtLaneUse(idx)) { return -1.0; }
  const std::size_t n = line_x_.size();
  double acc = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t a = (idx + k) % n;
    const std::size_t b = (idx + k + 1) % n;
    acc += std::hypot(line_x_[b] - line_x_[a], line_y_[b] - line_y_[a]);
    // look <= 0 なら打ち切らず、1周ぶん探して必ず距離を返す。
    if (look > 0.0 && acc > look) { return -1.0; }
    if (inOtLaneUse(b)) { return acc; }
  }
  return -1.0;
}

// 速度到達判定専用。ヘッダのコメント参照。
double V2XOvertaker::otLaneSpeedDeadlineDistance(std::size_t idx, double look) const
{
  if (ot_lane_zones_.empty() || line_x_.empty()) { return -1.0; }
  if (inOtLane(idx)) { return -1.0; }
  const std::size_t n = line_x_.size();
  double acc = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t a = (idx + k) % n;
    const std::size_t b = (idx + k + 1) % n;
    acc += std::hypot(line_x_[b] - line_x_[a], line_y_[b] - line_y_[a]);
    if (look > 0.0 && acc > look) { return -1.0; }
    if (inOtLane(b)) {
      return std::max(acc - geom_front_, 0.0);
    }
  }
  return -1.0;
}

bool V2XOvertaker::otLaneApproach(std::size_t idx, double v_now, double v_cap) const
{
  if (!ot_lane_enable_ || ot_lane_zones_.empty()) { return false; }
  if (inOtLane(idx)) { return v_now * 3.6 >= ot_lane_min_kmh_; }
  if (!ot_lane_prepare_) { return false; }
  const double look = ot_lane_runup_
    ? ot_lane_runup_look_
    : (ot_lane_prepare_look_ + std::max(v_now, 0.0) * ot_lane_prepare_time_);
  // このガードの安全条件は「車体がレーンに。
  const double d = otLaneSpeedDeadlineDistance(idx, look);
  if (d < 0.0) { return false; }
  const double a_use = ot_lane_runup_ ? runupAccelMps2() : 1.2;
  // 入口での到達速度。
  const double v_entry = std::min(
    std::sqrt(std::max(v_now, 0.5) * std::max(v_now, 0.5) + 2.0 * a_use * d),
    (v_cap > 0.0) ? v_cap : 1e9);
  const bool ok = v_entry * 3.6 >= ot_lane_min_kmh_;
  // 判定の中身を残す。
  if (ot_lane_log_ && (this->now() - last_otlane_log_).seconds() > 1.0) {
    last_otlane_log_ = this->now();
    RCLCPP_INFO(get_logger(),
      "レーン到達判定 idx=%zu 入口まで%.1fm 自車%.1fkm/h 上限%.1fkm/h "
      "入口での到達%.1fkm/h 門%.1fkm/h -> %s",
      idx, d, v_now * 3.6, v_cap * 3.6, v_entry * 3.6, ot_lane_min_kmh_,
      ok ? "使う" : "**使わない**");
  }
  return ok;
}

bool V2XOvertaker::otLaneHot(const Frame & f) const
{
  if (!ot_lane_hot_enable_) { return true; }
  const std::size_t n = f.n;
  if (n == 0) { return false; }
  const std::size_t a = (f.ei + n - 1) % n, b = (f.ei + 1) % n;
  double fx = f.in.points[b].pose.position.x - f.in.points[a].pose.position.x;
  double fy = f.in.points[b].pose.position.y - f.in.points[a].pose.position.y;
  const double fl = std::hypot(fx, fy);
  if (fl > 1e-9) { fx /= fl; fy /= fl; }
  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid || (f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
    const double sp = std::hypot(o.vx, o.vy) * 3.6;
    if (sp < ot_lane_min_kmh_ - 1.0) { continue; }        // アタッカーになれない速度
    const double dx = o.x - f.ex, dy = o.y - f.ey;
    const double back = -(dx * fx + dy * fy);             // 正なら後方
    if (back <= 0.0 || back > ot_lane_hot_range_) { continue; }
    // レーン側(右=負)にいるか。位置しか無いので横位置の符号で見る。
    const std::size_t oi = nearest(f.in, o.x, o.y);
    double nx, ny;
    normalAt(f.in, oi, nx, ny);
    const auto & lp = f.in.points[oi].pose.position;
    const double olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;
    if (ot_lane_side_right_ ? (olat < 0.5) : (olat > -0.5)) { return true; }
  }
  return false;
}

// その idx で「車体がレーンに一切触れない」中心位置の右限[m]。
//
// レーンは [zone_lo, zone_hi](左が正)。車体は中心 ± half の帯を占めるので、
// 触れない条件は **中心 - half >= zone_hi**、つまり 中心 >= zone_hi + half。
// half は車体半幅にヨーぶんの余裕を足したもの。
// レーンが無い idx では制限なし(-1e9)。
double V2XOvertaker::otLaneNoTouchLat(std::size_t idx) const
{
  const auto it = ot_lane_lat_.find(idx);
  if (it == ot_lane_lat_.end()) { return -1e9; }
  return it->second.second + veh_half_width_ + ot_lane_touch_margin_;
}

// idx から look[m] 先までで最も厳しい(= 最も左寄りの)右限を返す。
// 横位置は指令から約20m遅れて実現するので、入ってから戻すのでは間に合わない。
double V2XOvertaker::otLaneNoTouchAhead(std::size_t idx, double look) const
{
  if (ot_lane_lat_.empty() || line_x_.empty()) { return -1e9; }
  const std::size_t n = line_x_.size();
  double worst = -1e9, acc = 0.0;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t a = (idx + k) % n;
    worst = std::max(worst, otLaneNoTouchLat(a));
    const std::size_t b = (idx + k + 1) % n;
    acc += std::hypot(line_x_[b] - line_x_[a], line_y_[b] - line_y_[a]);
    if (acc > look) { break; }
  }
  return worst;
}

bool V2XOvertaker::otLaneAimWindow(std::size_t idx, double & lo_out,
                                   double & hi_out) const
{
  const auto it = ot_lane_lat_.find(idx);
  if (it == ot_lane_lat_.end()) { return false; }
  const double zlo = it->second.first;    // レーンの外側(壁側)
  const double zhi = it->second.second;   // レーンの内側(ライン側)
  const std::size_t n = corridor_.lo.size();
  if (n == 0 || idx >= n || corridor_.hi.size() != n) { return false; }
  const double sf = (ot_lane_win_safety_ >= 0.0) ? ot_lane_win_safety_
                                                 : safetyAt(idx);
  const double clo = corridor_.lo[idx] + sf;
  const double chi = corridor_.hi[idx] - sf;
  // 車体全体がレーン内: 中心 - 半幅 >= zlo かつ 中心 + 半幅 <= zhi
  const double wlo = zlo + geom_half_width_;
  const double whi = zhi - geom_half_width_;
  lo_out = std::max(clo, wlo);
  hi_out = std::min(chi, whi);
  return hi_out >= lo_out;
}

bool V2XOvertaker::otLaneUsable(std::size_t idx) const
{
  if (!ot_lane_enable_ || ot_lane_zones_.empty()) { return false; }
  if (!inOtLane(idx)) { return false; }
  return my_speed_for_gap_ * 3.6 >= ot_lane_min_kmh_;
}

double V2XOvertaker::sepRate(
  const std::string & name, double sep_now, const rclcpp::Time & now)
{
  constexpr double kTau = 0.25;        // なましの時定数[s]
  constexpr double kStaleSec = 0.5;
  auto & t = sep_track_[name];
  const double s_now = std::abs(sep_now);
  if (!t.init) {
    t.sep = s_now; t.stamp = now; t.rate = 0.0; t.init = true;
    return 0.0;
  }
  const double dt = (now - t.stamp).seconds();
  if (dt <= 1e-3) { return t.rate; }
  if (dt > kStaleSec) {
    t.sep = s_now; t.stamp = now; t.rate = 0.0;
    return 0.0;
  }
  const double raw = (s_now - t.sep) / dt;
  const double a = dt / (kTau + dt);
  t.rate += (raw - t.rate) * a;
  t.sep = s_now;
  t.stamp = now;
  return t.rate;
}

void V2XOvertaker::preventRearEnd(const Frame & f, PlanCtx & c)
{
  rear_dbg_ = RearEndDbg{};
  if (!rear_end_guard_) { return; }
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const size_t n = f.n;
  const size_t ei = f.ei;
  const double total = f.total;

  // 自分がこれから通る横位置。いまの位置と、寄せようとしている先の両方を見て
  // 「どちらかで重なる」なら進路上とみなす(寄せている最中に当てないため)。
  const double my_now = my_lat_for_target_;
  const double my_want = c.latWant();

  const bool straight_pass = straight_pass_now_;
  // 通すときは「本当に車体が重なるか」だけで見る(車幅 1.30m + わずかな余裕)。
  const double sep_th = straight_pass ? straight_pass_sep_ : rear_end_sep_;
  rear_dbg_.ran = true;
  rear_dbg_.sep_th = sep_th;
  rear_dbg_.my_now = my_now;
  rear_dbg_.my_want = my_want;

  double best_gap = 1e9;
  double best_speed = 0.0;
  std::string best_name;
  double best_sep = 0.0;
  double best_olat = 0.0;
  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid || (f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
    const size_t oi = nearest(in, o.x, o.y);
    double gap = s[oi] - s[ei];
    if (gap < 0.0) { gap += total; }
    if (gap <= 0.0 || gap > rear_end_range_) { continue; }
    double nxo, nyo;
    normalAt(in, oi, nxo, nyo);
    const auto & lpo = in.points[oi].pose.position;
    const double olat = (o.x - lpo.x) * nxo + (o.y - lpo.y) * nyo;
    // 通る帯に入っているか。
    const double closing_potential =
      std::max(rankSpeedCap(), my_speed_for_gap_) - std::hypot(o.vx, o.vy);
    double sep;
    if (rear_end_sep_true_) {
      double nxe, nye;
      normalAt(in, ei, nxe, nye);
      const double off_now = (o.x - f.ex) * nxe + (o.y - f.ey) * nye;
      const double off_want = off_now - (my_want - my_now);
      const double need_sep_now = v2x_overtaker::physicalPassSeparation(
        pass_beside_sep_, rear_end_free_min_);
      if (rear_end_sep_plan_ok_ && std::abs(off_want) >= need_sep_now) {
        sep = std::abs(off_now);
      } else {
        sep = std::min(std::abs(off_now), std::abs(off_want));
      }
      if (rear_end_stop_band_ && c_stop_avoid_pass_ &&
          !c_stop_avoid_target_.empty() && kv.first == c_stop_avoid_target_ &&
          c_stop_avoid_hi_ > c_stop_avoid_lo_)
      {
        const double band_c = 0.5 * (c_stop_avoid_lo_ + c_stop_avoid_hi_);
        const double sep_band = std::abs(band_c - olat);
        if (sep_band > sep) { sep = sep_band; }
      }
    } else {
      sep = rear_end_inpath_predict_
        ? v2x_overtaker::rearEndPredictedSeparation(
            my_now, my_want, olat, gap, closing_potential, offset_rate_,
            rear_end_inpath_max_sec_)
        : std::min(std::abs(my_now - olat), std::abs(my_want - olat));
    }
    bool in_path = (sep < sep_th);
    if (!in_path && rear_end_near_ > 0.0) {
      const double d3 = std::hypot(o.x - f.ex, o.y - f.ey);
      const double closing = my_speed_for_gap_ - std::hypot(o.vx, o.vy);
      if (d3 < rear_end_near_ && closing > rear_end_near_closing_ && gap > 0.5) {
        in_path = true;
      }
    }
    if (!in_path) { continue; }                // 横に十分ずれている。当たらない
    if (gap < best_gap) {
      {
        double nxe2, nye2;
        normalAt(in, ei, nxe2, nye2);
        best_sep_true_ = std::abs((o.x - f.ex) * nxe2 + (o.y - f.ey) * nye2);
        best_d3_ = std::hypot(o.x - f.ex, o.y - f.ey);
      }
      best_gap = gap; best_name = kv.first; best_olat = olat;
      best_speed = std::max(o.vx * 0.0 + std::hypot(o.vx, o.vy), 0.0);
      best_sep = sep;
    }
  }
  if (squeeze_ahead_ > 0.0 && !straight_pass && !best_name.empty() &&
      best_gap < squeeze_gap_ &&
      corridor_.lo.size() == n && corridor_.hi.size() == n)
  {
    double lo_min = -1e9, hi_min = 1e9, acc = 0.0;
    for (size_t k = 0; k < n; ++k) {
      const size_t a = (ei + k) % n, b = (ei + k + 1) % n;
      acc += std::hypot(in.points[b].pose.position.x - in.points[a].pose.position.x,
                        in.points[b].pose.position.y - in.points[a].pose.position.y);
      if (acc > squeeze_ahead_) { break; }
      lo_min = std::max(lo_min, corridor_.lo[b] + safetyAt(b));
      hi_min = std::min(hi_min, corridor_.hi[b] - safetyAt(b));
    }
    if (lo_min < hi_min) {
      const double before = c.latWant();
      c.boundLat(lo_min, hi_min, "追突防止(先の帯)");
      const double after = c.latWant();
      if (std::abs(before - after) > 0.15 &&
          (f.now - last_rearend_log_).seconds() > 1.0)
      {
        last_rearend_log_ = f.now;
        RCLCPP_INFO(get_logger(),
          "先で狭くなるので横目標を戻す %.2f -> %.2f (先%.0fm の帯 [%.2f,%.2f]) "
          "前方=%s %.1fm",
          before, after, squeeze_ahead_, lo_min, hi_min,
          best_name.c_str(), best_gap);
      }
    }
  }

  if (cross_gap_ > 0.0 && !best_name.empty() && best_gap < cross_gap_) {
    const double my_lat = my_lat_for_target_;
    if (std::abs(my_lat - best_olat) > cross_dead_) {
      const double before = c.latWant();
      if (my_lat > best_olat) {
        c.boundLat(best_olat, 1e9, "追突防止(またがない)");
      } else {
        c.boundLat(-1e9, best_olat, "追突防止(またがない)");
      }
      const double after = c.latWant();
      if (std::abs(before - after) > 0.15 &&
          (f.now - last_rearend_log_).seconds() > 1.0)
      {
        last_rearend_log_ = f.now;
        RCLCPP_INFO(get_logger(),
          "相手をまたがない %s まで %.1fm 相手横=%.2f 自車横=%.2f "
          "横目標 %.2f -> %.2f",
          best_name.c_str(), best_gap, best_olat, my_lat,
          before, after);
      }
    }
  }

  if (best_name.empty()) {
    return;
  }

  const bool pass_underway = attempt_active_ && side_fits_ && dbg_width_ >= min_pass_width_;
  const double brake_k = pass_underway ? rear_end_brake_k_pass_ : rear_end_brake_k_;
  const double brake_a = std::max(std::abs(a_min_) * brake_k, 0.3);
  const double react = std::max(my_speed_for_gap_, 0.0) * rear_end_time_;
  const double room = best_gap - rear_end_margin_ - react;
  double v_allow;
  if (room > 0.0) {
    v_allow = std::sqrt(2.0 * brake_a * room) + best_speed;
  } else {
    // 既に接触が近い。相手と同じ速度では**開き直せない**ので、
    // 食い込んだぶんだけ相手より遅くして車間を開ける。
    // Crash は 10秒 5km/h 固定(通常35km/h に対し 80m 以上の損失)なので、
    // ここで数秒詰まるほうが遥かに安い。
    v_allow = std::max(best_speed - rear_end_back_k_ * (-room), 0.0);
  }
  if (c.stop_avoid_active && c.stop_avoid_have_gap &&
      best_gap > stop_creep_gap_ && v_allow < stop_creep_speed_) {
    v_allow = stop_creep_speed_;
  }
  if (stop_creep_by_band_ && c.stop_avoid_active && c.stop_avoid_have_gap &&
      v_allow < stop_creep_slow_) {
    const bool inside_band =
      (my_lat_for_target_ >= c.stop_avoid_lo && my_lat_for_target_ <= c.stop_avoid_hi);
    if (!inside_band && best_gap > stop_creep_gap_min_) {
      v_allow = stop_creep_slow_;
      if ((f.now - last_creep_log_).seconds() > 1.0) {
        last_creep_log_ = f.now;
        diagLog("にじり出し",
          "にじり出し 車間%.1fm 自車横%.2fm 空き帯[%.2f,%.2f] "
          "上限 %.1f -> %.1fkm/h (帯に入るまで微速を許す)",
          best_gap, my_lat_for_target_, c.stop_avoid_lo, c.stop_avoid_hi,
          0.0, stop_creep_slow_ * 3.6);
      }
    }
  }

  // 追突防止が計算した上限を、出力段の「急減速を避ける」平滑化で
  // 高いまま残してはいけない。submit_13 のCrashでは、車間3.6m・自車18.8km/h
  // に対してここは7.0km/hを要求していたのに、後段が17.6km/hへ戻し、
  // 1秒後に車間1.2mまで食い込んだ。通常の追従では平滑化を維持し、
  // 制動余裕が尽きた場合だけ最大制動を許す。
  if (best_gap < rear_end_margin_ + react + 1.0 &&
      my_speed_for_gap_ > v_allow + 1.0) {
    c.emergency_brake = true;
  }

  double sep_for_release = best_sep;
  if (rear_end_target_release_ && attempt_active_ &&
      best_gap > rear_end_margin_)
  {
    const double aim_sep = std::abs(c.latWant() - best_olat);
    if (aim_sep > best_sep) {
      const double blend = std::clamp(rear_end_target_blend_, 0.0, 1.0);
      sep_for_release = best_sep + blend * (aim_sep - best_sep);
      if (sep_for_release > rear_end_free_min_ &&
          (f.now - last_target_release_log_).seconds() > 2.0) {
        last_target_release_log_ = f.now;
        diagLog("追突防止の先読み解除",
          "追突防止の先読み解除 %s 実測横間隔%.2fm 横目標%.2fm -> 判定に%.2fm "
          "(境界%.2f) 車間%.1fm",
          best_name.c_str(), best_sep, aim_sep, sep_for_release,
          rear_end_free_min_, best_gap);
      }
    }
  }
  if (rear_end_lat_release_ && attempt_active_ && sep_for_release > rear_end_free_min_) {
    {
      const double t = std::clamp(
          (sep_for_release - rear_end_free_min_) /
              std::max(rear_end_free_full_ - rear_end_free_min_, 1e-3), 0.0, 1.0);
      // 完全に外れたときの上限 = 制限なし。途中は相手速度との間を補間する。
      const double relaxed = best_speed + t * (rear_end_free_speed_ - best_speed);
      if (relaxed > v_allow) {
        v_allow = relaxed;
        if ((f.now - last_release_log_).seconds() > 2.0) {
          last_release_log_ = f.now;
          RCLCPP_INFO(get_logger(),
            "追突防止を緩めた %s 横間隔%.2fm(境界%.2f) 上限 -> %.1fkm/h 相手%.1fkm/h",
            best_name.c_str(), best_sep, min_lat_sep_, v_allow * 3.6,
            best_speed * 3.6);
        }
      }
    }
  }
  const double passing_beside_need =
    v2x_overtaker::physicalPassSeparation(pass_beside_sep_, rear_end_free_min_);
  // 解除の判定を「いまの横間隔」から。
  v2x_overtaker::RearEndReleaseInput rin;
  rin.sep_now = best_sep;
  rin.need = passing_beside_need;
  rin.gap = best_gap;
  rin.closing = my_speed_for_gap_ - best_speed;
  rin.floor = rear_end_predict_floor_;
  rin.max_predict_sec = rear_end_predict_max_sec_;
  rin.sep_rate = (rear_end_predict_release_ && !best_name.empty())
                   ? sepRate(best_name, best_sep, f.now) : 0.0;
  if (!rear_end_predict_release_) { rin.sep_rate = 0.0; }
  const auto rrel = v2x_overtaker::rearEndRelease(rin);
  const bool passing_beside =
    rrel.release &&
    (passUnderway(best_name, best_sep) || ovPassingTarget(best_name));
  // 予測ぶんで解除した周期は必ず残す。**発火しているかを数えられないと、
  // 効果があるかどうかも判定できない**(この案件で4回繰り返した誤り)。
  if (rrel.by_prediction && passing_beside) {
    ++rear_predict_release_count_;
    if ((f.now - last_rear_predict_log_).seconds() > 1.0) {
      last_rear_predict_log_ = f.now;
      diagLog("追突防止",
              "追突防止 予測で解除 累計%zu 相手=%s いまの横間隔%.2fm(要%.2f) "
              "変化率%+.2fm/s 並ぶまで%.2fs 予測横間隔%.2fm 車間%.1fm 接近%.1fkm/h",
              rear_predict_release_count_, best_name.c_str(),
              std::abs(best_sep), passing_beside_need, rin.sep_rate,
              rrel.t_meet, rrel.sep_pred, best_gap, rin.closing * 3.6);
    }
  }
  // 【2026-09-18】「横へ避けるから減速しない」を、舵の限界で裏取りする。
  // 必要な横ずれを、今の車間で、舵18度の範囲で作れないなら解除しない(作れる速度まで落とす)。
  double steer_cap = -1.0;
  if (steer_feasible_enable_ && !best_name.empty() && best_gap > 0.0) {
    const double need_sep =
      v2x_overtaker::physicalPassSeparation(pass_beside_sep_, rear_end_free_min_);
    v2x_overtaker::SteerFeasibleInput sin2;
    sin2.lateral_need_m = std::max(need_sep - std::abs(best_sep), 0.0);
    sin2.distance_m = std::max(best_gap - (geom_front_ + geom_rear_), 0.0);
    sin2.wheel_base_m = steer_feasible_wb_;
    sin2.max_steer_rad = steer_feasible_max_steer_;
    sin2.ay_max = side_plan_ay_max_;
    sin2.ay_use = steer_feasible_ay_use_;
    const auto sf = v2x_overtaker::steerFeasible(sin2);
    if (sf.need_move) {
      if (!sf.possible) {
        // 舵を全部使っても、この車間ではその横ずれを作れない。相手の速度まで落とす。
        steer_cap = std::max(best_speed, 0.0);
      } else if (sf.v_max_mps >= 0.0 && sf.v_max_mps < my_speed_for_gap_) {
        steer_cap = sf.v_max_mps;
      }
      if (steer_cap >= 0.0) {
        ++steer_feasible_n_;
        if ((f.now - last_release_log_).seconds() > 2.0) {
          last_release_log_ = f.now;
          diagLog("舵で間に合わない",
            "舵で間に合わない 累計%zu 相手=%s 車間%.1fm 要る横ずれ%.2fm "
            "要る旋回半径%.1fm(舵の限界%.1fm) 上限%.1fkm/h 自車%.1fkm/h",
            steer_feasible_n_, best_name.c_str(), best_gap, sin2.lateral_need_m,
            sf.radius_need_m, sf.radius_min_m, steer_cap * 3.6,
            my_speed_for_gap_ * 3.6);
        }
      }
    }
  }
  if (passing_beside) {
    // 並走している対象に対しては上限を出さない。
    v_allow = -1.0;
  } else if (ovPassingTarget(best_name)) {
    // まだ横へ出切っていない追い越し対象。相手速度は下回らせない。
    v_allow = std::max(v_allow, best_speed);
  }
  // ここは「最後の砦」として他層の値を上書き。
  if (rear_end_lat_plan_ && !best_name.empty() && v_allow >= 0.0) {
    double nxe3, nye3;
    normalAt(in, ei, nxe3, nye3);
    const auto it_b = others_.find(best_name);
    if (it_b != others_.end() && it_b->second.valid) {
      const OtherState & ob = it_b->second;
      const double off_now = (ob.x - f.ex) * nxe3 + (ob.y - f.ey) * nye3;
      const double off_want = off_now - (my_want - my_now);
      const double need = v2x_overtaker::physicalPassSeparation(
        pass_beside_sep_, rear_end_free_min_);
      const double body_len = geom_front_ + geom_rear_;
      const double gap_eff = best_gap - body_len;
      // 横の計画が離れる向きに出ているか(=回避しようとしているか)
      const bool moving_away = std::abs(off_want) > std::abs(off_now) + 0.05;
      const bool plan_clears = std::abs(off_want) >= need;
      if (moving_away && plan_clears && gap_eff > rear_end_margin_) {
        const double delta = std::max(need - std::abs(off_now), 0.0);
        const double t_lat = delta / std::max(offset_rate_, 0.05);
        const double closing = std::max(my_speed_for_gap_ - best_speed, 0.05);
        const double t_avail = gap_eff / closing;
        if (t_lat <= t_avail) {
          v_allow = -1.0;                       // 間に合う。減速しない
          if ((f.now - last_latplan_log_).seconds() > 2.0) {
            last_latplan_log_ = f.now;
            diagLog("横で回避するので減速しない",
              "横で回避するので減速しない %s 車間%.1fm 横間隔%.2f->%.2f(要%.2f) "
              "横移動%.1fs 余裕%.1fs", best_name.c_str(), best_gap,
              std::abs(off_now), std::abs(off_want), need, t_lat, t_avail);
          }
        } else {
          // 間に合う速度 = 相手速度 + 車間/横移動に要る時間
          const double v_fit = best_speed + gap_eff / std::max(t_lat, 0.1);
          if (v_fit > v_allow) {
            const double before = v_allow;
            v_allow = v_fit;
            if ((f.now - last_latplan_log_).seconds() > 2.0) {
              last_latplan_log_ = f.now;
              diagLog("横に間に合う速度へ",
                "横に間に合う速度へ %s 車間%.1fm 横移動%.1fs>余裕%.1fs "
                "上限 %.1f -> %.1fkm/h", best_name.c_str(), best_gap,
                t_lat, t_avail, before * 3.6, v_allow * 3.6);
            }
          }
        }
      }
    }
  }
  if (rear_audit_ && v_allow >= 0.0 && !best_name.empty() &&
      (f.now - last_rear_audit_log_).seconds() > rear_audit_sec_)
  {
    last_rear_audit_log_ = f.now;
    const double room_dbg = best_gap - rear_end_margin_ -
                            my_speed_for_gap_ * rear_end_time_;
    const double brake_dbg = std::max(std::abs(a_min_) * rear_end_brake_k_, 0.3);
    const double exp_dbg = (room_dbg > 0.0)
      ? (std::sqrt(2.0 * brake_dbg * room_dbg) + best_speed) : 0.0;
    diagLog("追突防止の内訳",
      "追突防止の内訳 idx=%zu 対象=%s 車間%.2fm 相手%.1fkm/h 自車%.1fkm/h "
      "横間隔 判定%.2f 真値%.2f 要%.2f 接近%.1fkm/h "
      "room%.2fm 制動%.2f -> 上限%.1fkm/h(式なら%.1f) 並走=%d 追越対象=%d",
      f.ei, best_name.c_str(), best_gap, best_speed * 3.6,
      my_speed_for_gap_ * 3.6, best_sep, best_sep_true_, best_d3_,
      std::max(my_speed_for_gap_ - best_speed, 0.0) * 3.6,
      room_dbg, brake_dbg, v_allow * 3.6, exp_dbg * 3.6,
      passing_beside ? 1 : 0, ovPassingTarget(best_name) ? 1 : 0);
  }
  rear_dbg_.name = best_name;
  rear_dbg_.gap = best_gap;
  rear_dbg_.sep = best_sep;
  rear_dbg_.cap = v_allow;
  if (v_allow >= 0.0) { c.requestCap(v_allow, "追突防止"); }
  // 舵で間に合わないぶんは、解除されていても効かせる(この層だけ別に出す)。
  if (steer_cap >= 0.0) { c.requestCap(steer_cap, "舵で間に合わない"); }
  // --- 壁に押されて相手側へ寄せられているときは、縦に譲って後ろへ下がる ---。
  if (lat_relax_yield_ && lat_relaxed_prev_ && !best_name.empty() &&
      !passing_beside) {
    const double back = std::max(best_speed - lat_relax_drop_mps_, min_follow_speed_);
    c.requestCap(back, "壁に押されて後退");
    if ((f.now - last_relax_yield_log_).seconds() > 1.0) {
      last_relax_yield_log_ = f.now;
      ++lat_relax_yield_n_;
      diagLog("壁に押されて後退",
              "壁に押されて後退 累計%zu 相手=%s 相手%.1fkm/h -> 上限%.1fkm/h "
              "横間隔%.2fm 車間%.1fm idx=%zu",
              lat_relax_yield_n_, best_name.c_str(), best_speed * 3.6,
              back * 3.6, std::abs(best_sep), best_gap, f.ei);
    }
  }
  if ((f.now - last_rearend_log_).seconds() > 1.0 &&
      (my_speed_for_gap_ > v_allow + 0.5 || best_gap < 4.0))
  {
    last_rearend_log_ = f.now;
    RCLCPP_INFO(get_logger(),
      "追突防止 %s まで %.1fm 横間隔 %.2fm(真値%.2fm 直線%.1fm 境界%.2f) "
      "相手 %.1fkm/h 要求上限 %.1fkm/h 自車 %.1fkm/h",
      best_name.c_str(), best_gap, best_sep, best_sep_true_, best_d3_,
      v2x_overtaker::physicalPassSeparation(pass_beside_sep_, rear_end_free_min_),
      best_speed * 3.6, v_allow * 3.6, my_speed_for_gap_ * 3.6);
  }
}

// 停止車両への突入を防ぐ。複数台が同じ場所で止まっている場合を含む。
// 止まっている車の集団に対して空いている横位置の区間を求め、
// 通れる区間があればそこへ、無ければ手前で止まる。
void V2XOvertaker::avoidStoppedCars(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const size_t n = f.n;
  const double total = f.total;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // カウントダウン中のグリッド全車を故障停止車として扱わない。最初の実移動後も
  // V2Xの標本時刻差だけ待つ。その後も動かない車は本物の停止車として直ちに
  // 通常処理へ戻るので、停止MPCの回避能力は失わない。
  if (v2x_overtaker::suppressStoppedCarsAtLaunch(
      launch_motion_since_, now.seconds(), launch_stopped_grace_))
  {
    deadlock_since_ = -1.0;
    c_stop_avoid_active_ = false;
    return;
  }

  {
    const double brake_a = std::max(std::abs(a_min_) * stop_brake_k_, 0.3);
    struct StoppedCar
    {
      double gap;
      double lat;
      size_t idx;
      double speed;
      std::string name;
    };
    std::vector<StoppedCar> stopped;
    // The stopped list has a deliberately tight *entry* threshold.  Latch
    // ownership must instead be observed from raw V2X with a wider exit
    // threshold, otherwise one noisy speed sample can erase the same ID.
    bool latched_target_relevant = false;
    double latched_target_speed = -1.0;
    double latched_target_gap = -1.0;
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || (now - o.stamp).seconds() > v2x_timeout_) {
        continue;
      }
      const size_t oi = nearest(in, o.x, o.y);
      double gap = s[oi] - s[ei];
      if (gap < 0) {
        gap += total;
      }
      const double speed = std::hypot(o.vx, o.vy);
      const bool is_latched_target =
        stop_nopass_latch_.latched() && kv.first == stop_nopass_latch_.target();
      const bool latched_target_in_hysteresis =
        v2x_overtaker::stopNoPassTargetRelevant(
          is_latched_target, true, gap, stopped_look_ahead_, speed,
          stop_nopass_exit_speed_);
      if (latched_target_in_hysteresis)
      {
        latched_target_relevant = true;
        latched_target_speed = speed;
        latched_target_gap = gap;
      }
      // Merely retaining the latch flag is insufficient: avoidStoppedCars()
      // emits its longitudinal cap only for cars in this list.  Keep the exact
      // latched ID in the avoidance calculation throughout the exit-speed
      // hysteresis, otherwise a 1.0 -> 1.1m/s sample drops the brake request
      // while the latch misleadingly remains true.
      if (speed > stopped_speed_ && !latched_target_in_hysteresis) {
        continue;              // 動いている。通常の追従制御に任せる
      }
      if (gap > stopped_look_ahead_ || gap < 0.5) {
        continue;
      }
      double nxo, nyo;
      normalAt(in, oi, nxo, nyo);
      const auto & lpo = in.points[oi].pose.position;
      const double olat_s = (o.x - lpo.x) * nxo + (o.y - lpo.y) * nyo;
      // --- 止まっている車を「追い越し対象」にしても停止車回避を止めない ---。
      if (stopped_keep_pass_exclude_) {
        if (ovPassingTarget(kv.first) &&
            std::abs(my_lat_for_target_ - olat_s) >= band_car_w_) {
          continue;
        }
      }
      stopped.push_back({gap, olat_s, oi, std::hypot(o.vx, o.vy), kv.first});
    }

    // The current nearest stopped car must not own this state.  A latched d2
    // can remain a hazard while d3 becomes `stopped.front()`.  Conversely, a
    // raw-fresh d2 at 1.1m/s is still held by the exit hysteresis even though
    // it no longer enters `stopped` (whose threshold is 1.0m/s).
    const std::string released_target = stop_nopass_latch_.target();
    const auto release = stop_nopass_latch_.update(
      now.seconds(), latched_target_relevant,
      !stopped.empty() && my_speed_for_gap_ < 0.5, stop_nopass_release_sec_);
    if (release != v2x_overtaker::StopNoPassRelease::kNone) {
      RCLCPP_INFO(get_logger(),
        "停止車ラッチ解除 reason=%s target=%s raw_speed=%.2fm/s raw_gap=%.1fm front=%s",
        v2x_overtaker::stopNoPassReleaseName(release), released_target.c_str(),
        latched_target_speed, latched_target_gap,
        stopped.empty() ? "なし" : stopped.front().name.c_str());
    }

    if (!stopped.empty()) {
      std::sort(stopped.begin(), stopped.end(),
                [](const StoppedCar & a, const StoppedCar & b) { return a.gap < b.gap; });
      const double base = stopped.front().gap;
      const size_t bi = stopped.front().idx;

      // その地点で使える横方向の範囲
      double lo = -3.0, hi = 3.0;
      if (corridor_.lo.size() == n) {
        lo = corridor_.lo[bi] + safetyAt(bi);
        hi = corridor_.hi[bi] - safetyAt(bi);
      }
      if (stop_avoid_fix_ && corridor_.lo.size() == n) {
        // 6m では停止車クラスタ(8m先まで同じ。
        const double span_eff = stop_avoid_band_local_
                                  ? std::max(stop_avoid_band_span_, 0.5)
                                  : stop_avoid_span_;
        double acc = 0.0;
        // 手前側は半区間ぶん戻ってから、前方へ span ぶん見る。
        size_t k0 = bi;
        double back = 0.0;
        while (back < span_eff * 0.5) {
          const size_t prev = (k0 + n - 1) % n;
          back += std::hypot(in.points[k0].pose.position.x - in.points[prev].pose.position.x,
                             in.points[k0].pose.position.y - in.points[prev].pose.position.y);
          k0 = prev;
        }
        // ここは span 全体のコリドアの**共通部分**を取って。
        v2x_overtaker::FunnelInput fin;
        double ds_first = -1.0;
        for (size_t k = 0; k < n; ++k) {
          const size_t a = (k0 + k) % n, b = (k0 + k + 1) % n;
          fin.lo.push_back(corridor_.lo[a] + safetyAt(a));
          fin.hi.push_back(corridor_.hi[a] - safetyAt(a));
          const double seg = std::hypot(
            in.points[b].pose.position.x - in.points[a].pose.position.x,
            in.points[b].pose.position.y - in.points[a].pose.position.y);
          if (ds_first < 0.0) { ds_first = std::max(seg, 0.1); }
          acc += seg;
          if (acc > span_eff) { break; }
        }
        bool funnel_used = false;
        if (stopped_funnel_ && fin.lo.size() >= 2) {
          fin.ds = ds_first;
          fin.lat_rate_mps = offset_rate_;
          fin.speed_mps = odom_ ? std::abs(odom_->twist.twist.linear.x) : 0.0;
          fin.speed_floor = funnel_speed_floor_;
          const auto fr = v2x_overtaker::corridorFunnel(fin);
          if (fr.valid && fr.lo <= fr.hi) {
            lo = std::max(lo, fr.lo);
            hi = std::min(hi, fr.hi);
            funnel_used = true;
          }
        }
        if (!funnel_used) {
          for (size_t k = 0; k < fin.lo.size(); ++k) {
            lo = std::max(lo, fin.lo[k]);
            hi = std::min(hi, fin.hi[k]);
          }
        }
      }

      // 手前の集団(先頭から stopped_cluster_span 以内)が塞ぐ横位置。
      double best_w = -1.0, best_a = 0.0, best_b = 0.0;
      int group = 0;          // 2段探索の外で持つ(下のログが使う)
      bool occ_pad = true;
      // stopped_pad_relax=false なら停止車の余裕は外さない。帯が無ければ手前で止まる。
      for (int occ_pass = 0; occ_pass < (stopped_pad_relax_ ? 2 : 1); ++occ_pass) {
      occ_pad = (occ_pass == 0);
      best_w = -1.0; best_a = 0.0; best_b = 0.0;
      std::vector<std::pair<double, double>> blocked;
      group = 0;
      for (const auto & st : stopped) {
        if (st.gap - base > stopped_cluster_span_) {
          break;
        }
        const double ohw = occupiedHalfWidth(st.idx, false) +
                           (occ_pad ? stoppedPad() : 0.0);
        blocked.emplace_back(st.lat - ohw, st.lat + ohw);
        group++;
      }
      std::sort(blocked.begin(), blocked.end());

      if (stopped_seq_pass_ && !blocked.empty()) {
        // 候補の帯。最初はコリドアの全幅。
        std::vector<std::pair<double, double>> segs{{lo, hi}};
        double prev_gap = base;
        std::size_t bi = 0;
        for (const auto & st : stopped) {
          if (st.gap - base > stopped_cluster_span_) { break; }
          if (bi >= blocked.size()) { break; }
          ++bi;
          // 前の車からここまでに横へ動ける量。
          const double v_now = std::max(
            odom_ ? std::abs(odom_->twist.twist.linear.x) : 0.0, funnel_speed_floor_);
          const double move = std::max(st.gap - prev_gap, 0.0) / v_now * offset_rate_;
          prev_gap = st.gap;
          const double ohw2 = occupiedHalfWidth(st.idx, false) +
                              (occ_pad ? stoppedPad() : 0.0);
          const double blo = st.lat - ohw2, bhi = st.lat + ohw2;
          std::vector<std::pair<double, double>> next;
          for (auto & sg : segs) {
            // 動ける量だけ帯を広げ、コリドアで丸める。
            const double a = std::max(sg.first - move, lo);
            const double b = std::min(sg.second + move, hi);
            if (b <= a) { continue; }
            // この車の占有帯を引く(左右に割れることがある)。
            if (bhi <= a || blo >= b) { next.emplace_back(a, b); continue; }
            if (blo > a) { next.emplace_back(a, blo); }
            if (bhi < b) { next.emplace_back(bhi, b); }
          }
          segs.swap(next);
          if (segs.empty()) { break; }
        }
        for (const auto & sg : segs) {
          const double w = sg.second - sg.first;
          if (w > best_w) { best_w = w; best_a = sg.first; best_b = sg.second; }
        }
      } else {
      double cur = lo;
      for (const auto & b : blocked) {
        if (b.first > cur) {
          const double w = b.first - cur;
          if (w > best_w) {
            best_w = w;
            best_a = cur;
            best_b = b.first;
          }
        }
        cur = std::max(cur, b.second);
      }
      if (hi > cur) {
        const double w = hi - cur;
        if (w > best_w) {
          best_w = w;
          best_a = cur;
          best_b = hi;
        }
      }
      }
      // 余裕込みで通れる帯が見つかったら、そこで確定。
      // 見つからなければ 2回目(余裕なし)で測り直す。
      if (best_w > 0.0) { break; }
      if (occ_pass == 0 && stoppedPad() > 0.0 &&
          (this->now() - last_occ_relax_log_).seconds() > 3.0) {
        last_occ_relax_log_ = this->now();
        diagLog("占有の余裕を外す",
                "占有の余裕を外す 余裕%.2fm込みでは通れる帯が無い。"
                "ギリギリで測り直す(停止車%zu台 idx=%zu)",
                stoppedPad(), stopped.size(), ei);
      }
      }

      const double d = base - stop_margin_;
      const double v_stop = (d > 0.0) ? std::sqrt(2.0 * brake_a * d) : 0.0;

      const double band_center = (best_a + best_b) * 0.5;
      std::string act;

      v2x_overtaker::StoppedPassReachabilityInput reach_in;
      reach_in.base_distance_m = base;
      reach_in.free_width_m = best_w;
      reach_in.required_width_m = stopped_slack_;
      reach_in.band_center_m = band_center;
      reach_in.ego_lateral_m = my_lat_for_target_;
      // 固定 20m をやめ、速度に応じた距離にする。
      reach_in.lateral_lag_m = lat_lag_by_speed_
        ? v2x_overtaker::lateralLagDistance(
            my_speed_for_gap_, lat_lag_sec_, lat_lag_base_, lat_lag_max_)
        : stop_avoid_lat_lag_;
      reach_in.lateral_rate_mps = offset_rate_;
      // 要る横移動を「帯の中央まで」から。
      reach_in.use_band_edge = stopped_band_edge_;
      reach_in.band_lo_m = best_a;
      reach_in.band_hi_m = best_b;
      reach_in.ego_speed_mps = std::max(my_speed_for_gap_, 0.0);
      const auto reach = stop_avoid_fix_
        ? v2x_overtaker::evaluateStoppedPassReachability(reach_in)
        : v2x_overtaker::StoppedPassReachabilityResult{};
      const bool pass_now = !stop_avoid_fix_ ||
        (reach.state == v2x_overtaker::StoppedPassReachability::kPassNow &&
         !stop_nopass_latch_.latched());
      const bool pass_with_speed_cap = stop_avoid_fix_ &&
        reach.state == v2x_overtaker::StoppedPassReachability::kPassWithSpeedCap &&
        !stop_nopass_latch_.latched();
      // ラッチは「自車が止まってから」しか解けなかった。
      if (stop_nopass_release_on_pass_ && stop_nopass_latch_.latched() &&
          reach.state != v2x_overtaker::StoppedPassReachability::kNoPass)
      {
        stop_nopass_latch_.clear();
        if ((now - last_stop_fix_log_).seconds() > 1.0) {
          last_stop_fix_log_ = now;
          RCLCPP_INFO(get_logger(),
            "停止車ラッチ解除 reason=通れる判定に戻った target=%s 空き幅%.2fm "
            "要る横移動%.2fm 必要距離%.1fm 手前%.1fm",
            stopped.front().name.c_str(), best_w, reach.delta_lateral_m,
            reach.needed_distance_m, base);
        }
      }
      const bool pass_now2 = !stop_avoid_fix_ ||
        (reach.state == v2x_overtaker::StoppedPassReachability::kPassNow &&
         !stop_nopass_latch_.latched());
      const bool pass_with_speed_cap2 = stop_avoid_fix_ &&
        reach.state == v2x_overtaker::StoppedPassReachability::kPassWithSpeedCap &&
        !stop_nopass_latch_.latched();
      const bool pass_ok = pass_now2 || pass_with_speed_cap2;
      if (stop_avoid_fix_ && !pass_ok && !stop_nopass_latch_.latched()) {
        stop_nopass_latch_.latch(stopped.front().name, now.seconds());
        if ((now - last_stop_fix_log_).seconds() > 1.0) {
          last_stop_fix_log_ = now;
          RCLCPP_WARN(get_logger(),
            "停止車到達性 target=%s state=%s base_m=%.1f free_width_m=%.2f req_width_m=%.2f "
            "band_center_m=%.2f ego_lateral_m=%.2f need_m=%.1f v_lat_max_mps=%.2f",
            stopped.front().name.c_str(),
            v2x_overtaker::stoppedPassReachabilityName(reach.state),
            base, best_w, stopped_slack_, band_center, my_lat_for_target_,
            reach.needed_distance_m, reach.v_lat_max_mps);
        }
      }
      if (pass_now2) {
        // 余裕をもって通れる。帯の中央へ寄せる。
        c.requestLat(band_center, PlanCtx::LatPrio::kStoppedCar, "停止車回避");
        c.stop_avoid_active = true;
        c.stop_avoid_v_stop = v_stop;
        c.stop_avoid_have_gap = true;
        c.stop_avoid_lo = best_a;
        c.stop_avoid_hi = best_b;
        c.stop_avoid_dist = base;
        // 速度は他層のcapに従い、ここでは上げ直さない。
        act = "通過";
      } else if (pass_with_speed_cap2) {
        double aim = band_center;
        if (stopped_aim_edge_) {
          const double me = my_lat_for_target_;
          if (me < best_a)      { aim = std::min(best_a + stopped_edge_inset_, band_center); }
          else if (me > best_b) { aim = std::max(best_b - stopped_edge_inset_, band_center); }
          else                  { aim = me; }   // 既に帯の中。動く必要がない
        }
        c.requestLat(aim, PlanCtx::LatPrio::kStoppedCar, "停止車回避");
        c.stop_avoid_active = true;
        c.stop_avoid_v_stop = v_stop;
        c.stop_avoid_have_gap = true;
        c.stop_avoid_lo = best_a;
        c.stop_avoid_hi = best_b;
        c.stop_avoid_dist = base;
        // This cap is the largest speed that reaches the clear band before the obstacle.
        c.requestCap(reach.v_lat_max_mps, "停止車回避(横到達)");
        act = "減速して通過";
      } else {
        // 通れない。手前で止まる。
        c.requestCap(v_stop, "停止車回避");
        act = "停止";
      }

      if (stop_avoid_fix_ && !pass_ok) {
        const double v = std::max(my_speed_for_gap_, 0.0);
        const double a_emg = std::max(stop_avoid_emg_accel_, 0.5);
        // 反応(V2X 遅延 + 1周期)で進む距離 + 制動距離 + 余裕
        const double d_emg = v * 0.25 + (v * v) / (2.0 * a_emg) + stop_avoid_emg_margin_;
        if (base <= d_emg && v > 1.0) {
          c.emergency_brake = true;
          c.requestCap(0.0, "停止車回避(最大制動)");
          if ((now - last_stop_fix_log_).seconds() > 1.0) {
            last_stop_fix_log_ = now;
            RCLCPP_WARN(get_logger(),
              "停止車 %s まで %.1fm。避けられず制動距離(%.1fm)を割ったので最大制動",
              stopped.front().name.c_str(), base, d_emg);
          }
        }
      }
      // --- 膠着の検出(警告のみ)
      // stuck_recovery_controller は「指令速度が閾値未満なら stuck ではない」と
      // 判定する(stuck_recovery_controller.cpp の updateStuckDetection)。
      // つまり自分から止まれと指令している間は復帰が動かない。
      // 新しい通知経路を足すのではなく、上の (a)(b) で「止まれと指令しない」
      // ように直してある。ここでは効いているかを見るための記録だけ残す。
      if (my_speed_for_gap_ < 0.5 && base < 5.0) {
        if (deadlock_since_ < 0.0) { deadlock_since_ = now.seconds(); }
        if ((now.seconds() - deadlock_since_) >= 3.0 &&
            (now - last_deadlock_log_).seconds() > 2.0)
        {
          last_deadlock_log_ = now;
          RCLCPP_WARN(get_logger(),
                      "膠着 停止車両 %s まで %.1fm 自車 %.2fm/s が %.1f秒 "
                      "空き幅 %.2fm 上限 %.1fkm/h 横目標 %.2f",
                      stopped.front().name.c_str(), base, my_speed_for_gap_,
                      now.seconds() - deadlock_since_, best_w,
                      (c.speed_cap < 0.0 ? 99.0 : c.speed_cap) * 3.6, c.latWant());
        }
      } else {
        deadlock_since_ = -1.0;
      }
      if ((now - last_stopped_log_).seconds() > 2.0) {
        last_stopped_log_ = now;
        RCLCPP_INFO(get_logger(),
                    "停止車到達性 target=%s group=%d state=%s base_m=%.1f free_width_m=%.2f "
                    "delta_lat_m=%.2f need_m=%.1f v_lat_max_mps=%.2f cap_mps=%.2f action=%s latched=%d",
                    stopped.front().name.c_str(), group,
                    v2x_overtaker::stoppedPassReachabilityName(reach.state),
                    base, best_w, reach.delta_lateral_m, reach.needed_distance_m,
                    reach.v_lat_max_mps, c.speed_cap, act.c_str(),
                    stop_nopass_latch_.latched() ? 1 : 0);
      }
    }
  }
}

// 衝突回避層。他車と壁の両方を見て、横へよける量と速度上限を出す。
void V2XOvertaker::avoidCollision(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  {
    const auto & q = odom_->pose.pose.orientation;
    const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    const double fx = std::cos(yaw), fy = std::sin(yaw);
    const double vx = odom_->twist.twist.linear.x * fx;
    const double vy = odom_->twist.twist.linear.x * fy;

    double worst_ttc = 1e9;
    double worst_sep = 0.0;   // 最も危険な相手の横位置(正なら相手は自分の右)
    double push = 0.0;
    wedge_active_ = false;
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || (this->now() - o.stamp).seconds() > v2x_timeout_) {
        continue;
      }
      const double dx = o.x - ex, dy = o.y - ey;
      const double dist = std::hypot(dx, dy);
      if (dist > avoid_range_) {
        continue;
      }
      // 後ろから来る車は避けない。
      //
      // ここで前後を区別していなかったため、後方から接近してくる車も
      // 「接近率が正」になって衝突時間が短く出て、横へ逃げていた。
      // その結果、抜かれる側なのに自分から壁へ寄って接触していた。
      // 後ろの車をよけるのは相手の仕事で、こちらが動く必要はない。
      {
        const auto & fa = in.points[ei].pose.position;
        const auto & fb = in.points[(ei + 2) % n].pose.position;
        double fx = fb.x - fa.x, fy = fb.y - fa.y;
        const double fl = std::hypot(fx, fy);
        if (fl > 1e-9) { fx /= fl; fy /= fl; }
        if (dx * fx + dy * fy < -kRearIgnore) { continue; }
        // 縦に重なっている(真横に並んでいる)車は正面衝突の相手ではない。
        // 距離が collision_radius より近いと TTC が負になり、同じ向きに並走している
        // だけの車を「止まれない正面衝突」とみなして壁側へ押していた(4台レースの
        // スタートで P3 が壁に接触)。真横の相手は並走の層(追突防止・車体ガード)が扱う。
        if (dx * fx + dy * fy < avoid_min_lon_) { continue; }
      }
      const size_t oi2 = nearest(in, o.x, o.y);
      double nx2, ny2;
      normalAt(in, oi2, nx2, ny2);
      const auto & lp2 = in.points[oi2].pose.position;
      const double olat2 = (o.x - lp2.x) * nx2 + (o.y - lp2.y) * ny2;
      const double sep = my_lat_for_target_ - olat2;
      if (std::abs(sep) >= min_lat_sep_) {
        continue;                    // 横に離れている。追い越し中なので邪魔しない
      }

      // 相対速度から衝突までの時間(TTC)を出す
      const double rvx = o.vx - vx, rvy = o.vy - vy;
      const double closing_rate = -(dx * rvx + dy * rvy) / std::max(dist, 0.1);
      const double ttc = (closing_rate > 0.1)
                         ? (dist - collision_radius_) / closing_rate : 1e9;
      if (ttc < worst_ttc) {
        worst_ttc = ttc;
        worst_sep = sep;             // その相手が自分のどちら側にいるか
      }
      // 近すぎる/迫っているなら横へ逃げる量を決める
      if (dist < collision_radius_ * 2.0 || ttc < ttc_threshold_) {
        const double dir = (std::abs(sep) > 0.15) ? ((sep > 0) ? 1.0 : -1.0) : side_sign_;
        const double need = (min_lat_sep_ - std::abs(sep));
        if (need > 0 && std::abs(need) > std::abs(push)) {
          push = need * dir;
        }
      }
    }

    // 止まりきれないなら、横へねじ込んで正面衝突を横からの接触に変える。
    //
    // Crash(10秒)はカート前方で当たったときだけ付き、横からの接触では付かない。
    // ここで減速だけを選ぶと、止まりきれないまま前から突っ込んで Crash になる。
    // 間に合わないと分かった時点では、コリドアの縁まで使ってでも横へ出る。
    if (wedge_enable_ && worst_ttc < wedge_ttc_ && corridor_.lo.size() == n) {
      const double lo = corridor_.lo[ei] + wedge_room_;
      const double hi = corridor_.hi[ei] - wedge_room_;
      const double room_left = hi - my_lat_for_target_;
      const double room_right = my_lat_for_target_ - lo;
      const double planned_room = (side_sign_ > 0.0) ? room_left : room_right;
      const bool planned_ok = ov_collide_follow_side_ &&
        (ovPassing() || attempt_active_) && planned_room >= min_lat_sep_;
      const double dir = planned_ok
                           ? side_sign_
                           : ((std::abs(worst_sep) > 0.15)
                                ? ((worst_sep > 0.0) ? 1.0 : -1.0)
                                : ((room_left >= room_right) ? 1.0 : -1.0));
      const double avail = std::max(room_left, room_right);
      const double want = std::min(avail, min_lat_sep_);
      if (want > std::abs(push)) {
        push = want * dir;
        c.avoid_offset = std::clamp(my_lat_for_target_ + push, lo, hi);
        wedge_active_ = true;
        // ここでは減速しない。止まれないのだから、
        // 落とすほど相手の正面に居座る時間が延びるだけになる。
        c.avoid_speed_cap = -1.0;
        if ((this->now() - last_wedge_log_).seconds() > 1.0) {
          last_wedge_log_ = this->now();
          RCLCPP_INFO(get_logger(),
                      "正面衝突を回避 TTC%.2fs 横へ%.2fm(%s) 空き[左%.2f 右%.2f] "
                      "相手横間隔=%.2f",
                      worst_ttc, push, dir > 0 ? "左" : "右", room_left, room_right,
                      worst_sep);
        }
      }
    }

    // 横へ逃げる余地があるなら、減速より先に横移動で解決させる。
    // 減速すると追い越しが不成立になるため。
    if (worst_ttc < ttc_threshold_ && std::abs(push) < 1e-3) {
      // 迫っているなら減速する。TTC が短いほど強く落とす
      const double ratio = std::clamp(worst_ttc / ttc_threshold_, 0.0, 1.0);
      c.avoid_speed_cap = std::max(my_speed_for_gap_ * ratio, min_follow_speed_);
    }

    dbg_avoid_ttc_ = (worst_ttc < 1e8) ? worst_ttc : -1.0;

    // 逃げ先が壁でないかを確認する。壁側へ押されるなら逆へ、それも駄目なら減速のみ。
    if (std::abs(push) > 1e-3 && corridor_.lo.size() == n) {
      const double edge = (wedge_active_ && wedge_keep_room_) ? wedge_room_
                                                             : safetyAt(ei);
      const double lo = corridor_.lo[ei] + edge;
      const double hi = corridor_.hi[ei] - edge;
      const double want_cand = my_lat_for_target_ + push;
      const double cand_a = std::clamp(want_cand, lo, hi);
      const double cand_b = std::clamp(my_lat_for_target_ - push, lo, hi);
      const double got_a = cand_a - my_lat_for_target_;   // 望んだ側で実際に寄れる量
      const double got_b = cand_b - my_lat_for_target_;   // 逆側で寄れる量
      double cand = cand_a;
      // 相手が特定できている間は、空きが広くても相手側へは反転させない。
      const bool opp_known = (worst_ttc < 1e8) && (std::abs(worst_sep) > 0.15);
      const bool block_flip = wedge_no_flip_ && opp_known;
      if (!block_flip && std::abs(got_b) > std::abs(got_a) + 1e-3) {
        cand = cand_b;                                    // 逆側のほうが寄れる
      }
      const double got = cand - my_lat_for_target_;
      if (std::abs(got) < std::abs(push) * 0.5) {
        // 逃げ場が無い並走。ここで減速すると相手の真横に居座る時間が延びる。
        // Crash(10秒)は「自分の前で当てた側」に付くので、横が無いなら縦で解く。
        if (wedge_forward_ && wedge_active_ && commit_now_) {
          c.avoid_speed_cap = -1.0;      // 上限なし。前へ出る
        } else {
          // 半分も寄れないなら、横移動だけでは足りない。減速も併用する。
          c.avoid_speed_cap = std::max(my_speed_for_gap_ * 0.5, min_follow_speed_);
        }
      }
      const double push_req = push;
      push = got;
      if (std::abs(push) > 1e-3) {
        c.avoid_offset = cand;
      }
      if ((this->now() - last_wedge_fix_log_).seconds() > 0.5) {
        last_wedge_fix_log_ = this->now();
        RCLCPP_INFO(get_logger(),
                    "回避の逃げ先 idx=%zu wedge=%d 自車横=%.2f 帯=[%.2f,%.2f] "
                    "要求=%.2f 逃げ側=%.2f 相手側=%.2f 採用=%.2f 反転=%d "
                    "上限=%.1f 相手横間隔=%.2f",
                    ei, wedge_active_ ? 1 : 0, my_lat_for_target_, lo, hi,
                    push_req, got_a, got_b, got, (cand == cand_b) ? 1 : 0,
                    c.avoid_speed_cap * 3.6, worst_sep);
      }
    }
  }
}

// 近接車から横方向へ反発する。
void V2XOvertaker::repulseFromNearCars(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // --- 近接車からの横方向の反発
  // 「前方の車」だけを見ていると、真横に並んだ車を無視して寄っていき接触する。
  // 前後方向の距離に関係なく、一定半径内の車とは横方向の間隔を確保する。
  {
    double my_lat;
    {
      double nx0, ny0;
      normalAt(in, ei, nx0, ny0);
      const auto & lp0 = in.points[ei].pose.position;
      my_lat = (ex - lp0.x) * nx0 + (ey - lp0.y) * ny0;
    }
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || (now - o.stamp).seconds() > v2x_timeout_) {
        continue;
      }
      const double dist = std::hypot(o.x - ex, o.y - ey);
      if (dist > near_radius_) {
        continue;
      }
      const size_t oi = nearest(in, o.x, o.y);
      double nx, ny;
      normalAt(in, oi, nx, ny);
      const auto & lp = in.points[oi].pose.position;
      const double olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;
      const double sep = my_lat - olat;
      if (std::abs(sep) < min_lat_sep_) {
        // 足りない分だけ相手と逆向きに押しのける。
        // 真横に重なっている(sep~0)ときは、決めておいた側へ確実に逃がす。
        const double dir = (std::abs(sep) > 0.15) ? ((sep > 0) ? 1.0 : -1.0) : side_sign_;
        const double need = (min_lat_sep_ - std::abs(sep)) * dir;
        // 最も強い反発を採用する(複数車に囲まれても発散させない)
        if (std::abs(need) > std::abs(c.repulse)) {
          c.repulse = need;
        }
      }
    }
  }
  if (std::abs(c.repulse) > 1e-3) {
    const bool launch_phase = v2x_overtaker::launchWindowActive(
      launch_motion_since_, now.seconds(), launch_free_sec_);
    double want = my_lat_for_target_ + c.repulse;
    bool room_ok;
    if (!repulse_need_allow_ && !launch_phase &&
        ei < band_lo_.size() && band_lo_[ei] < band_hi_[ei]) {
      double lo = band_lo_[ei] + repulse_band_margin_;
      double hi = band_hi_[ei] - repulse_band_margin_;
      if (lo > hi) {
        const double mid = 0.5 * (band_lo_[ei] + band_hi_[ei]);
        lo = hi = mid;
      }
      want = std::clamp(want, lo, hi);
      room_ok = std::abs(want - my_lat_for_target_) > 0.10;
    } else {
      room_ok = can_pass_now_;
    }
    if (room_ok) {
      // band による丸めは上で want に対して掛けてある(この意図の内部の制限)。
      c.requestLat(want, PlanCtx::LatPrio::kRepulse, "近接車反発");
      if (c.blocker.empty()) {
        c.blocker = "近接車";
      }
    }
  }
}


// ===================================================================
// 横位置を保つ層
// ===================================================================

// スタート直後はグリッドの横位置を保持し、距離とともに 0 へ減衰させる。
// 全車が一斉に同じレースラインへ収束して団子になるのを防ぐ。
// 発進の一発追い越し(第3段)が有効か。
// P1 のとき・1周目・合図から launch_p1_pass_sec 以内だけ。
// start_slot_ は assignStartSlots(合図と同時)まで 0 なので、合図前は自動的に無効。
bool V2XOvertaker::launchPassActive() const
{
  if (!launch_p1_pass_ || !launch_p1_right_) { return false; }
  if (launch_pass_done_) { return false; }   // 一度降りたら二度と上がらない
  if (start_slot_ != 1) { return false; }    // P1 のときだけ
  if (lap_ > 1) { return false; }
  if (launch_since_ < 0.0) { return false; }
  return v2x_overtaker::launchWindowActive(
    launch_motion_since_, this->now().seconds(), launch_p1_pass_sec_);
}

// 対象は「slot が npc_slot_ の車」だけ。居なければ何もしない
// (npc_slot_ が決め打ちなので、本番で並びが違ったときの保険)。
const OtherState * V2XOvertaker::launchNpc() const
{
  for (const auto & kv : others_) {
    if (kv.second.slot == npc_slot_ && kv.second.valid && kv.second.prog_init) {
      return &kv.second;
    }
  }
  return nullptr;
}

// 完了判定。観測だけを行い、指令には触らない。
void V2XOvertaker::updateLaunchPass(const Frame & f)
{
  if (launch_pass_done_ || launch_since_ < 0.0) { return; }
  const OtherState * npc = launchNpc();
  if (!npc) { return; }
  const double d_prog = npc->prog - my_prog_;   // 負 = 自分が前
  if (launchPassActive() && d_prog < -launch_p1_pass_ahead_) {
    launch_pass_done_ = true;
    start_merge_done_ = true;
    launch_want_valid_ = false;
    RCLCPP_INFO(get_logger(),
                "発進追抜 完了 t=%.1fs 弧長差=%+.1fm 走行=%.1fm 自車横=%.2f",
                launch_motion_since_ >= 0.0 ? f.now.seconds() - launch_motion_since_ : 0.0,
                d_prog, launch_run_,
                my_lat_for_target_);
  } else if (launchPassActive() && launch_no_pass_guard_dist_ > 0.0 &&
             !no_pass_zones_.empty()) {
    // 発進追越だけは通常の planOvertake より後の holdGridLane が横目標を
    // 上書きする。そのまま狭い禁止区間へ入ると、衝突回避が戻そうとした
    // 経路まで右側へ保持して NPC の正面へ押し込む。次の禁止区間までの
    // 距離を経路上で測り、まだ抜き切れていなければ手前で保持を解除する。
    double dist_to_no_pass = 1e9;
    double acc = 0.0;
    for (std::size_t k = 0; k < f.n; ++k) {
      const std::size_t b = (f.ei + k) % f.n;
      bool inside = false;
      for (const auto & z : no_pass_zones_) {
        inside = (z.first <= z.second)
                   ? (b >= z.first && b <= z.second)
                   : (b >= z.first || b <= z.second);
        if (inside) { break; }
      }
      if (inside) {
        dist_to_no_pass = acc;
        break;
      }
      const std::size_t next = (b + 1) % f.n;
      acc += std::hypot(
        f.in.points[next].pose.position.x - f.in.points[b].pose.position.x,
        f.in.points[next].pose.position.y - f.in.points[b].pose.position.y);
    }
    if (dist_to_no_pass <= launch_no_pass_guard_dist_) {
      launch_pass_done_ = true;
      start_merge_done_ = true;
      launch_want_valid_ = false;
      RCLCPP_WARN(get_logger(),
                  "発進追抜 安全終了: 禁止区間まで%.1fm 弧長差=%+.1fm 走行=%.1fm",
                  dist_to_no_pass, d_prog, launch_run_);
    }
  } else if (launch_p1_pass_ && !launchPassActive() && start_slot_ == 1) {
    launch_pass_done_ = true;
    start_merge_done_ = true;
    launch_want_valid_ = false;
    RCLCPP_WARN(get_logger(),
                "発進追抜 失効 t=%.1fs 弧長差=%+.1fm 走行=%.1fm",
                launch_motion_since_ >= 0.0 ? f.now.seconds() - launch_motion_since_ : 0.0,
                d_prog, launch_run_);
  }
}

// グリッド座標 P(slot) の横位置[m]。座標照合なので**停止中でも求まる**。
double V2XOvertaker::gridLat(const Frame & f, int slot) const
{
  if (slot < 1 || slot > static_cast<int>(grid_slots_.size())) { return 0.0; }
  const double gx = grid_slots_[slot - 1].first;
  const double gy = grid_slots_[slot - 1].second;
  const size_t gi = nearest(f.in, gx, gy);
  double nx, ny;
  normalAt(f.in, gi, nx, ny);
  const auto & lp = f.in.points[gi].pose.position;
  return (gx - lp.x) * nx + (gy - lp.y) * ny;   // 正=左 / 負=右
}

void V2XOvertaker::holdGridLane(const Frame & f, PlanCtx & c)
{
  if (!launch_hold_grid_ && !launch_p1_right_) { return; }
  if (grid_slots_.empty() || corridor_.lo.size() != f.n) { return; }

  // 走行距離は自前で数える。holdStartLane の run_dist_ は
  // start_captured_ の後しか進まないので、この層では使えない。
  if (launch_lv_) {
    const double st = std::hypot(f.ex - launch_lx_, f.ey - launch_ly_);
    if (st < 5.0) { launch_run_ += st; }
  }
  launch_lx_ = f.ex;
  launch_ly_ = f.ey;
  launch_lv_ = true;

  const bool pass_hold = launch_p1_hold_until_pass_ && launchPassActive();
  if (!pass_hold && launch_motion_since_ >= 0.0 &&
      (f.now.seconds() - launch_motion_since_) > launch_hold_sec_) { return; }

  // スロットは assignStartSlots を待たない。自車座標の照合だけで停止中から決まる。
  int slot = start_slot_;
  if (slot <= 0) {
    double best = 1e18;
    for (size_t i = 0; i < grid_slots_.size(); ++i) {
      const double d = std::hypot(f.ex - grid_slots_[i].first, f.ey - grid_slots_[i].second);
      if (d < best) { best = d; slot = static_cast<int>(i) + 1; }
    }
    if (best > 3.0) { return; }   // グリッドに居ない = 発進フェーズではない
  }

  double want = gridLat(f, slot);
  double span = launch_hold_dist_;
  const char * mode = "grid";

  if (launch_p1_right_ && slot == 1) {
    double lead = -1e9;
    for (const auto & kv : others_) {
      if (kv.second.slot == 2 && kv.second.valid && kv.second.prog_init) {
        lead = kv.second.prog - my_prog_;
      }
    }
    const bool p2_clear = (lead > launch_p1_clear_);
    want = p2_clear ? launch_p1_lat_far_ : launch_p1_lat_near_;
    span = launch_p1_dist_;
    mode = p2_clear ? "P1右(遠)" : "P1右(近)";
    if (pass_hold) { span = 1e9; }
    if (launch_p1_lat_step_ > 0.0) {
      const double tn = f.now.seconds();
      if (!launch_want_valid_) {
        launch_want_prev_ = my_lat_for_target_;
        launch_want_t_ = tn;
        launch_want_valid_ = true;
      }
      const double dt = std::clamp(tn - launch_want_t_, 0.0, 0.5);
      launch_want_t_ = tn;
      want = std::clamp(want, launch_want_prev_ - launch_p1_lat_step_ * dt,
                              launch_want_prev_ + launch_p1_lat_step_ * dt);
      launch_want_prev_ = want;
    }
  }
  if (!launch_hold_grid_ && slot != 1) { return; }

  // 壁からは start_wall_margin だけ空ける(後段の avoidWall がさらに詰める)
  double lo = corridor_.lo[f.ei] + start_wall_margin_;
  double hi = corridor_.hi[f.ei] - start_wall_margin_;
  if (lo > hi) { const double m = 0.5 * (lo + hi); lo = hi = m; }
  want = std::clamp(std::clamp(want, -start_lat_abs_, start_lat_abs_), lo, hi);

  const double w = (span > 0.0) ? std::clamp(1.0 - launch_run_ / span, 0.0, 1.0) : 0.0;
  if (w <= 1e-3) { return; }
  const double blended = want * w + c.latWant() * (1.0 - w);
  c.requestLat(blended, PlanCtx::LatPrio::kGridLane, "グリッド保持");
  if (c.blocker.empty()) { c.blocker = "発進レーン"; }

  if ((this->now() - last_launch_log_).seconds() > 0.5) {
    last_launch_log_ = this->now();
    RCLCPP_INFO(get_logger(),
                "発進レーン P%d %s 保持=%.2f 重み=%.2f 走行=%.1fm "
                "自車横=%.2f -> 横目標=%.2f",
                slot, mode, want, w, launch_run_, my_lat_for_target_, blended);
  }
}

void V2XOvertaker::holdStartLane(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;

  // --- スタート直後はグリッドの横位置を保持する
  // 4台が一斉に同じレースラインへ収束すると、スタート直後に必ず接触して団子になる。
  // 発進時の自車の横位置を記録し、それを基準オフセットとして距離とともに 0 へ減衰させ、
  // 各車が自分のレーンを保ったまま加速してから合流するようにする。
  // 副作用として、スタート時に横ずれを一気に詰めようとする挙動も消える。
  {
    const double sp = std::hypot(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y);
    if (!start_captured_ && sp > 0.3) {
      double nx0, ny0;
      normalAt(in, ei, nx0, ny0);
      const auto & lp0 = in.points[ei].pose.position;
      start_lat_ = (ex - lp0.x) * nx0 + (ey - lp0.y) * ny0;
      {
        double lo_here = -start_lat_max_, hi_here = start_lat_max_;
        if (corridor_.lo.size() == in.points.size()) {
          lo_here = corridor_.lo[ei] + start_wall_margin_;
          hi_here = corridor_.hi[ei] - start_wall_margin_;
          if (lo_here > hi_here) {
            const double mid = 0.5 * (lo_here + hi_here);
            lo_here = hi_here = mid;
          }
        }
        // 極端な値(自己位置の飛び)だけは絶対値でも抑える
        start_lat_ = std::clamp(start_lat_, -start_lat_abs_, start_lat_abs_);
        start_lat_ = std::clamp(start_lat_, lo_here, hi_here);
      }
      {
        struct Slot { double prog; double lat; double x; double y; std::string name; };
        std::vector<Slot> slots;
        slots.push_back(Slot{my_prog_, start_lat_, ex, ey, "自車"});
        for (const auto & kv : others_) {
          const OtherState & o = kv.second;
          if (!o.valid || !o.prog_init) { continue; }
          const size_t oi2 = nearest(in, o.x, o.y);
          double nx2, ny2;
          normalAt(in, oi2, nx2, ny2);
          const auto & lp2 = in.points[oi2].pose.position;
          slots.push_back(Slot{o.prog, (o.x - lp2.x) * nx2 + (o.y - lp2.y) * ny2,
                               o.x, o.y, kv.first});
        }
        std::sort(slots.begin(), slots.end(),
                  [](const Slot & a, const Slot & b) { return a.prog < b.prog; });
        start_slot_lat_.clear();
        int mine = 0;
        for (size_t i = 0; i < slots.size(); ++i) {
          start_slot_lat_.push_back(slots[i].lat);
          if (slots[i].name == "自車") { mine = static_cast<int>(i) + 1; }
          // **記録用**: この行の x,y をそのまま grid_slots パラメータに書き写す。
          // 進行度による並べ替えは V2X の到着に依存して不安定なので、
          // 一度これで記録し、以後は grid_slots との照合で確定させる。
          RCLCPP_INFO(get_logger(),
                      "スタート位置 P%zu = %s 座標=(%.2f,%.2f) 横=%.2f 進行度=%.1f",
                      i + 1, slots[i].name.c_str(), slots[i].x, slots[i].y,
                      slots[i].lat, slots[i].prog);
        }

        int by_grid = 0;
        if (!grid_slots_.empty()) {
          double best = 1e18;
          for (size_t i = 0; i < grid_slots_.size(); ++i) {
            const double d = std::hypot(ex - grid_slots_[i].first,
                                        ey - grid_slots_[i].second);
            if (d < best) { best = d; by_grid = static_cast<int>(i) + 1; }
          }
          RCLCPP_INFO(get_logger(),
                      "グリッド照合: 自車(%.2f,%.2f) は記録の P%d に最も近い(%.2fm)",
                      ex, ey, by_grid, best);
        }
        start_slot_ = (by_grid > 0) ? by_grid : mine;
        RCLCPP_INFO(get_logger(),
                    "自車のスタート位置は P%d (照合=%d 進行度順=%d)",
                    start_slot_, by_grid, mine);

      }
      start_s_ = s[ei];
      start_captured_ = true;
      RCLCPP_INFO(get_logger(), "スタート横位置を記録: %.2f m", start_lat_);
    }
    if (start_captured_ && !start_merge_done_ && start_merge_dist_ > 0.0) {
      // 走行距離の累積で測る。
      // 周回位置(s[ei]-start_s_)で判定すると、1周してスタート地点に戻ったときに
      // 再びレーン保持が復活し、目標オフセットが急に start_lat_ へ飛ぶ。
      // その結果スタート地点手前で突然ステアリングが切れて壁に当たっていた。
      // レース開始時に一度だけ効かせる。
      const double dx = ex - last_x_;
      const double dy = ey - last_y_;
      if (last_valid_) {
        const double step = std::hypot(dx, dy);
        if (step < 5.0) {           // 自己位置の飛びは加算しない
          run_dist_ += step;
        }
      }
      last_x_ = ex;
      last_y_ = ey;
      last_valid_ = true;
      const double run = run_dist_;
      if (run >= start_merge_dist_) {
        start_merge_done_ = true;
        RCLCPP_INFO(get_logger(), "スタートレーン保持を終了(走行 %.1f m)", run);
      }
      if (run < start_merge_dist_) {
        const double u = std::clamp(run / start_merge_dist_, 0.0, 1.0);
        const double w = start_merge_smooth_
          ? (1.0 - (3.0 * u * u - 2.0 * u * u * u))
          : (1.0 - u);
        const double sl = c.blocker.empty()
                            ? (start_lat_ * w)
                            : (c.latWant() * (1 - w) + start_lat_ * w);
        c.requestLat(sl,
          start_hold_priority_ ? PlanCtx::LatPrio::kStartHold
                               : PlanCtx::LatPrio::kGridLane,
          "スタートレーン保持");
      }
    }
  }
}

// の車体位置で、コリドアと他車への食い込みを見て引き戻す。
void V2XOvertaker::bodyGuardByMeasured(const Frame & f, PlanCtx & c)
{
  if (!body_guard_enable_ || !odom_) { return; }
  // グリッドはコリドアの外にあるため、発進直後はこの層が。
  if (body_guard_after_merge_ && !start_merge_done_) { return; }
  const std::size_t n = corridor_.lo.size();
  if (n == 0 || corridor_.hi.size() != n || f.ei >= n) { return; }

  // 進行方向に対する車体の張り出し(姿勢ぶんを含む)
  const auto & q = odom_->pose.pose.orientation;
  const double myyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const std::size_t a = (f.ei + f.n - 1) % f.n, b = (f.ei + 1) % f.n;
  const double tth = std::atan2(
    f.in.points[b].pose.position.y - f.in.points[a].pose.position.y,
    f.in.points[b].pose.position.x - f.in.points[a].pose.position.x);
  double e = myyaw - tth;
  while (e > M_PI) { e -= 2.0 * M_PI; }
  while (e < -M_PI) { e += 2.0 * M_PI; }
  double ext_l = 0.0, ext_r = 0.0;
  bodyExtent(e, ext_l, ext_r, curveRadiusAt(f.ei),
             (curve_sign_ > 0.0) ? +1 : ((curve_sign_ < 0.0) ? -1 : 0));

  const double me = my_lat_for_target_;
  const double wall_hi = corridor_.hi[f.ei];
  const double wall_lo = corridor_.lo[f.ei];
  const double add_l = std::max(ext_l - geom_half_width_, 0.0);
  const double add_r = std::max(ext_r - geom_half_width_, 0.0);
  // いまの車体外縁と壁の余裕。負なら既にはみ出している。
  const double m_hi = wall_hi - (me + add_l);
  const double m_lo = (me - add_r) - wall_lo;
  const double margin = std::min(m_hi, m_lo);

  const double body_len = geom_front_ + geom_rear_;
  const double need_car = geom_half_width_ * 2.0 + size_pad_;
  double car_margin = 1e9;
  double car_push = 0.0;          // 正なら左へ、負なら右へ逃がしたい
  std::string car_id;
  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid || !o.prog_init || !my_prog_init_) { continue; }
    if ((f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
    if (std::abs(o.prog - my_prog_) > body_len) { continue; }
    const std::size_t oi = nearest(f.in, o.x, o.y);
    double nx, ny; normalAt(f.in, oi, nx, ny);
    const auto & lp = f.in.points[oi].pose.position;
    const double olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;
    const double sep = std::abs(me - olat);
    const double m = sep - need_car;
    if (m < car_margin) {
      car_margin = m; car_id = kv.first;
      car_push = (me >= olat) ? +1.0 : -1.0;   // 相手から離れる向き
    }
  }

  const double worst = std::min(margin, car_margin);
  if (worst >= body_guard_react_m_) { return; }

  // --- 引き戻す先 ---
  // 壁からは react だけ内側、相手からは need_car + react だけ離れた点。
  double aim = me;
  if (m_hi < body_guard_react_m_) { aim = std::min(aim, wall_hi - add_l - body_guard_react_m_); }
  if (m_lo < body_guard_react_m_) { aim = std::max(aim, wall_lo + add_r + body_guard_react_m_); }
  if (car_margin < body_guard_react_m_) {
    aim += car_push * (body_guard_react_m_ - car_margin);
  }
  // 壁の内側からは出さない。帯が狭すぎるときは中央へ。
  const double in_lo = wall_lo + add_r;
  const double in_hi = wall_hi - add_l;
  aim = (in_lo <= in_hi) ? std::clamp(aim, in_lo, in_hi) : 0.5 * (in_lo + in_hi);
  c.requestLat(aim, PlanCtx::LatPrio::kBodyGuard, "車体の実位置を戻す");

  // --- 速度 ---
  // 既にはみ出している間だけ落とす。まだ余裕があるうちは横だけで直す。
  if (worst < -body_guard_hard_m_ && body_guard_cap_kmh_ > 0.0) {
    c.requestCap(body_guard_cap_kmh_ / 3.6, "車体の実位置");
  }
  if ((f.now - last_body_guard_log_).seconds() > 1.0) {
    last_body_guard_log_ = f.now;
    diagLog("車体の実位置",
      "車体の実位置 idx=%zu 実横%.2f 張出[左%.2f 右%.2f] 壁[%.2f,%.2f] "
      "余裕[壁上%.2f 壁下%.2f 相手%.2f(%s)] -> 戻し先%.2f 上限%s",
      f.ei, me, ext_l, ext_r, wall_lo, wall_hi, m_hi, m_lo,
      (car_margin > 1e8 ? -1.0 : car_margin), car_id.empty() ? "-" : car_id.c_str(),
      aim, (worst < 0.0) ? "あり" : "なし");
  }
}

// 並走中は横オフセットを保持する。
// 横に並ぶと相手が前方帯から外れて検出されなくなり、目標が 0 に戻ってしまうため。
void V2XOvertaker::holdSideBySide(const Frame & f, PlanCtx & c)
{
  const size_t n = f.n;
  const size_t ei = f.ei;

  if (attempt_active_ && std::abs(c.target_offset) < pass_gap_ * 0.5 &&
      std::abs(attempt_offset_) > 1e-3) {
    double held = attempt_offset_;
    // 保持する場合も壁だけは避ける。
    // 現在地点の幅が足りなくなったら保持をやめてラインへ戻す。
    // ここを見ないと、狭い区間へ横に出たまま進入して壁に当たる。
    if (corridor_.lo.size() == n) {
      const double lo = corridor_.lo[ei] + safetyAt(ei);
      const double hi = corridor_.hi[ei] - safetyAt(ei);
      if ((hi - lo) < min_pass_width_ * latch_width_gain_) {
        held = 0.0;
      } else {
        held = std::clamp(held, lo, hi);
      }
    }
    if (std::abs(held) > pass_gap_ * 0.5) {
      c.target_offset = held;
    }
  }
}

void V2XOvertaker::logAttemptFunnel(const Frame & f, const char * result, double elapsed)
{
  static const char * const kStageName[] = {
    "未到達", "試行開始", "横移動完了", "並走", "先行", "維持", "復帰"};
  int st = attempt_stage_;
  if (st < 0) { st = 0; }
  if (st > 6) { st = 6; }
  if (attempt_fail_first_.empty() && std::string(result) != "成功") {
    attempt_fail_first_ = "その他";
  }
  double tgt_v = -1.0;
  {
    const auto it = others_.find(attempt_target_);
    if (it != others_.end() && it->second.valid) {
      tgt_v = std::hypot(it->second.vx, it->second.vy) * 3.6;
    }
  }
  diagWarn("追越ファネル",
              "追越ファネル target=%s 結果=%s 到達段階=%d(%s) 最初の失敗=%s 所要=%.1fs "
              "開始idx=%zu 終了idx=%zu 開始車間=%.1fm 最大横間隔=%.2fm "
              "自車=%.1fkm/h 相手=%.1fkm/h 相手ペナ=%d 自車ペナ=%d 助走=%d",
              attempt_target_.empty() ? "-" : attempt_target_.c_str(),
              result, st, kStageName[st],
              attempt_fail_first_.empty() ? "-" : attempt_fail_first_.c_str(),
              elapsed, attempt_idx0_, f.ei, attempt_gap0_, attempt_max_sep_,
              std::abs(f.ev) * 3.6, tgt_v,
              attempt_tgt_pen_ ? 1 : 0, attempt_self_pen_ ? 1 : 0,
              attempt_runup_used_ ? 1 : 0);
}

// 追い越しの試行・成功・失敗を数えてログに残す。sweep.sh がこれを集計する。
void V2XOvertaker::recordAttempt(const Frame & f, PlanCtx & c)
{
  const double total = f.total;
  const rclcpp::Time now = f.now;

  // --- 追い越しの試行と結果を記録する
  // 「横に出て抜きにいった」を試行開始、「相手を前後で追い越した」を成功、
  // 「横に出たが抜けずにラインへ戻った」を失敗として数える。
  // sweep.sh がこのログを集計してパラメータの良し悪しを判断する。
  {
    // 停止車回避が有効な周期は、新規の追い越し試行も始めない。
    // 継続中の試行は onTimer で後段の横位置保持より前に安全中断済みである。
    if (c.stop_avoid_active) {
      return;
    }

    // 試行の開始は「本当に横へ出る指令が出たとき」にする。
    // min_pass_sep*0.5(0.58m)では、追従中のわずかな横ずれまで試行と数えて
    // 「試行47回・成功0回」のような数字になり、何が本当の仕掛けか読めなかった。
    // pass_gap の 70%(1.9m -> 1.33m)まで寄せる指令が出て初めて試行とみなす。
    const bool moving_out = std::abs(pass_sep_) > pass_gap_ * 0.7;
    const bool overtake_intent =
      (c.lat_intent.why != nullptr && std::strcmp(c.lat_intent.why, "追越") == 0);
    const bool start_ok = v2x_overtaker::overtakeAttemptStartAuthorized(
      attempt_active_, moving_out, overtake_intent, attempt_require_intent_,
      c.blocker, c.pass_authorized_target);
    if (!attempt_active_ && moving_out && overtake_intent && !start_ok &&
        (now - last_attempt_auth_log_).seconds() > 1.0) {
      last_attempt_auth_log_ = now;
      diagLog("追越開始拒否",
              "追越開始拒否 blocker=%s 許可対象=%s idx=%zu 車間=%.1fm "
              "横間隔=%.2fm 理由=今周期の対象別許可なし",
              c.blocker.empty() ? "-" : c.blocker.c_str(),
              c.pass_authorized_target.empty() ? "-" : c.pass_authorized_target.c_str(),
              f.ei, c.best_gap, pass_sep_);
    }
    if (!attempt_active_ && start_ok && no_pass_side_ &&
        (now - last_no_start_log_).seconds() > 2.0) {
      last_no_start_log_ = now;
      diagLog("抜けないので始めない",
              "抜けないので始めない target=%s idx=%zu 自車%.1fkm/h 車間%.1fm "
              "(どちらの側も必要間隔に届かない)",
              c.blocker.c_str(), f.ei, my_speed_for_gap_ * 3.6, c.best_gap);
    }
    if (!attempt_active_ && start_ok && !no_pass_side_) {
      attempt_active_ = true;
      attempt_target_ = c.blocker;
      attempt_start_ = now.seconds();
      attempt_boosts_ = boost_used_;
      attempt_offset_ = c.latWant();
      attempt_fail_since_ = -1.0;
      attempt_infeasible_since_ = -1.0;
      attempt_lat_valid_ = false;
      attempt_lead_cnt_ = 0;
      attempt_diff0_ = 1e18;
      attempt_max_sep_ = 0.0;
      attempt_latok_ = false;
      attempt_stage_ = 1;                 // 1 = 試行開始
      attempt_fail_first_.clear();
      attempt_runup_used_ = false;
      attempt_rel_v_since_ = -1.0;
      attempt_third_since_ = -1.0;
      {
        const auto itb = others_.find(c.blocker);
        attempt_gap0_ = (itb != others_.end() && itb->second.valid)
                          ? (itb->second.prog - my_prog_) : -1.0;
      }
      attempt_idx0_ = f.ei;
      attempt_rank0_ = my_rank_obs_;
      attempt_tgt_pen_ = isPenalized(c.blocker);
      attempt_self_pen_ = selfPenalized();
      attempt_predictive_ = spot_enable_ && spot_valid_ &&
                            c.blocker == spot_target_;
      attempt_plan_dist_ = attempt_predictive_ ? spot_dist_ : -1.0;
      diagLog("追越試行", "追越試行 開始 target=%s 車間=%.1fm rank=%d 方式=%s 入口まで=%.1fm",
                  c.blocker.c_str(), c.best_gap, rank_,
                  attempt_predictive_ ? "予測" : "汎用", attempt_plan_dist_);
    } else if (attempt_active_) {
      // 前方車が見えている間は保持値を最新の指令で更新する
      if (moving_out) {
        attempt_offset_ = c.latWant();
      }
      if (isPenalized(attempt_target_)) { attempt_tgt_pen_ = true; }
      if (selfPenalized()) { attempt_self_pen_ = true; }
      bool passed = false;
      bool stalled = false;
      double funnel_diff = 0.0;
      bool funnel_diff_ok = false;
      auto it = others_.find(attempt_target_);
      if (it != others_.end() && it->second.valid) {
        const double diff = my_prog_ - it->second.prog;
        if (std::abs(diff) < total * 0.5) { funnel_diff = diff; funnel_diff_ok = true; }
        // 進行度差は周回のまたぎで大きく振れる。妥当な範囲のときだけ使う。
        if (attempt_stall_time_ > 0.0 && std::abs(diff) < total * 0.5) {
          if (attempt_diff0_ > 1e17) { attempt_diff0_ = diff; }
          else if ((now.seconds() - attempt_start_) > attempt_stall_time_ &&
                   (diff - attempt_diff0_) < attempt_stall_gain_) {
            stalled = true;
          }
        }
        const int need_cycles =
          std::max(1, static_cast<int>(pass_done_sec_ * 20.0 + 0.5));
        const double elapsed_now = now.seconds() - attempt_start_;
        if (diff > pass_done_len_ && diff < total * 0.5 &&
            elapsed_now >= pass_done_min_sec_) {
          if (++attempt_lead_cnt_ >= need_cycles) { passed = true; }
        } else {
          attempt_lead_cnt_ = 0;
        }
      }
      const double elapsed = now.seconds() - attempt_start_;

      {
        const double sep_now = std::abs(pass_sep_);
        const double veh_len = geom_front_ + geom_rear_;

        // --- 到達した最大の段階。段階は排他で、下がらない。
        int st = (attempt_stage_ < 1) ? 1 : attempt_stage_;   // 1 = 試行開始
        const bool lat_done = sep_now >= commit_sep_;          // 2 = 横移動完了
        if (lat_done && st < 2) { st = 2; }
        if (lat_done && funnel_diff_ok && std::abs(funnel_diff) <= veh_len &&
            st < 3) { st = 3; }                                // 3 = 並走
        if (funnel_diff_ok && funnel_diff > pass_done_len_ && st < 4) { st = 4; }
        if (passed && st < 5) { st = 5; }                      // 5 = 維持(成功成立)
        // 6 = 復帰。成功した周期で試行が終わるため通常は到達しない
        //(成功後もこの関数が呼ばれ続ける構造になったときのために残す)。
        if (attempt_stage_ >= 5 && sep_now < min_pass_sep_ * 0.3 && st < 6) {
          st = 6;
        }
        attempt_stage_ = st;

        double rel_v = 1e9;   // 自車 - 相手 の速度差[m/s]
        if (it != others_.end() && it->second.valid) {
          rel_v = std::abs(f.ev) - std::hypot(it->second.vx, it->second.vy);
        }
        if (attempt_stage_ >= 2 && rel_v < 0.5) {
          if (attempt_rel_v_since_ < 0.0) { attempt_rel_v_since_ = now.seconds(); }
        } else {
          attempt_rel_v_since_ = -1.0;
        }
        const bool third_blocker =
          !c.blocker.empty() && c.blocker != attempt_target_;
        if (third_blocker) {
          if (attempt_third_since_ < 0.0) { attempt_third_since_ = now.seconds(); }
        } else {
          attempt_third_since_ = -1.0;
        }

        if (attempt_fail_first_.empty()) {
          const char * why = nullptr;
          if (isPenalized(attempt_target_)) {
            why = "相手ペナ";
          } else if (selfPenalized()) {
            why = "自車ペナ";
          } else if (last_contact_ctx_log_.nanoseconds() > 0 &&
                     (f.now - last_contact_ctx_log_).seconds() <= 0.5) {
            why = "接触";
          } else {
            // 壁余裕。占有格子が使えるときだけ見る(logContactContext と同じ計算)。
            bool wall_tight = false;
            if (occ_.ok && odom_) {
              const auto & q = odom_->pose.pose.orientation;
              const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                            1.0 - 2.0 * (q.y * q.y + q.z * q.z));
              wall_tight = bodyClearance(f.ex, f.ey, yaw, 0.0) < 0.0;
            }
            // 横に出られないと見なす時間。attempt_latfail_time_ は既定 0(無効)
            // なので、無効のときは既存の打切時間の半分を使う。新しい
            // パラメータは足さない。
            const double latfail_t = (attempt_latfail_time_ > 0.0)
              ? attempt_latfail_time_ : (attempt_timeout_ * 0.5);
            if (wall_tight) {
              why = "壁余裕なし";
            } else if (attempt_stage_ < 2 && elapsed > latfail_t) {
              why = "横に出られない";
            } else if (attempt_rel_v_since_ >= 0.0 &&
                       (now.seconds() - attempt_rel_v_since_) >= 1.0) {
              why = "速度差なし";
            } else if (attempt_third_since_ >= 0.0 &&
                       (now.seconds() - attempt_third_since_) >= 0.5) {
              why = "第三者に阻まれた";
            } else if (elapsed >= attempt_timeout_) {
              why = "時間切れ";
            }
          }
          if (why != nullptr) { attempt_fail_first_ = why; }
        }
      }

      bool lat_stalled = false;
      if (attempt_latfail_time_ > 0.0) {
        if (attempt_max_sep_ >= commit_sep_) { attempt_latok_ = true; }
        if (!attempt_latok_ && elapsed > attempt_latfail_time_) {
          lat_stalled = true;
        }
      }
      const bool infeasible_stop =
        attempt_infeasible_time_ > 0.0 && attempt_infeasible_since_ >= 0.0 &&
        (now.seconds() - attempt_infeasible_since_) >= attempt_infeasible_time_;
      if (attempt_giveup_time_ > 0.0 && no_pass_side_ &&
          !no_pass_side_target_.empty() &&
          no_pass_side_target_ == attempt_target_) {
        if (attempt_nopass_since_ < 0.0) { attempt_nopass_since_ = now.seconds(); }
      } else {
        attempt_nopass_since_ = -1.0;
      }
      const bool nopass_stop =
        attempt_giveup_time_ > 0.0 && attempt_nopass_since_ >= 0.0 &&
        (now.seconds() - attempt_nopass_since_) >= attempt_giveup_time_;
      // 【2026-09-18 ユーザー指示】この区間では抜き切れないと分かったら追い越しをやめる。
      // やめれば次の周期から側と可否を計算し直すので、左が空いていれば左で仕切り直せる。
      if (pass_finish_abort_ && finish_short_now_) {
        if (attempt_finish_short_since_ < 0.0) { attempt_finish_short_since_ = now.seconds(); }
      } else {
        attempt_finish_short_since_ = -1.0;
      }
      const bool finish_stop =
        pass_finish_abort_ && attempt_finish_short_since_ >= 0.0 &&
        (now.seconds() - attempt_finish_short_since_) >= pass_finish_hold_;
      if (!passed && std::abs(pass_sep_) < min_pass_sep_ * 0.3) {
        if (attempt_fail_since_ < 0.0) { attempt_fail_since_ = now.seconds(); }
      } else {
        attempt_fail_since_ = -1.0;
      }
      if (passed) {
        attempt_active_ = false;
        attempt_ok_++;
        double tgt_v = -1.0, tgt_lat = 0.0;
        {
          const auto it2 = others_.find(attempt_target_);
          if (it2 != others_.end() && it2->second.valid) {
            tgt_v = std::hypot(it2->second.vx, it2->second.vy) * 3.6;
          }
          tgt_lat = pass_sep_;
        }
        const double road_w = (f.ei < corridor_.hi.size() && corridor_.lo.size() == corridor_.hi.size())
          ? (corridor_.hi[f.ei] - corridor_.lo[f.ei]) : -1.0;
        diagLog("追越記録", "追越記録 成功 target=%s 所要=%.1fs 方式=%s idx=%zu(開始idx=%zu) "
                    "幅=%.2fm 自車=%.1fkm/h 相手=%.1fkm/h 差=%.1fkm/h "
                    "横間隔=%.2fm(最大%.2fm) 車間 開始=%.1fm "
                    "ブースト=%d rank=%d 相手ペナ=%d 自車ペナ=%d "
                    "自順位=P%d->P%d 相手順位=P%d 種別=%s",
                    attempt_target_.c_str(), elapsed,
                    attempt_predictive_ ? "予測" : "即時",
                    f.ei, attempt_idx0_, road_w,
                    std::abs(f.ev) * 3.6, tgt_v,
                    (tgt_v >= 0.0) ? (std::abs(f.ev) * 3.6 - tgt_v) : 0.0,
                    tgt_lat, attempt_max_sep_, attempt_gap0_,
                    boost_used_ - attempt_boosts_, rank_,
                    attempt_tgt_pen_ ? 1 : 0, attempt_self_pen_ ? 1 : 0,
                    attempt_rank0_, my_rank_obs_, oppRank(attempt_target_),
                    passKind(attempt_target_));
        last_pass_ok_t_ = now.seconds();
        logAttemptFunnel(f, "成功", elapsed);
      } else if (attempt_fail_since_ >= 0.0 && elapsed > 1.0 &&
                 (now.seconds() - attempt_fail_since_) >= 0.5) {
        attempt_active_ = false;
        attempt_ng_++;
        // 失敗したので側の確定を解く。
        side_committed_ = false;
        diagLog("追越試行", "追越試行 失敗 target=%s 所要=%.1fs ブースト=%d rank=%d "
                    "横間隔=%.2f(要%.2f) 最大実測横間隔=%.2f allow=%d zone=%d "
                    "zone_ok=%d feasible=%d "
                    "latch=%d 幅=%.1f",
                    attempt_target_.c_str(), elapsed,
                    boost_used_ - attempt_boosts_, rank_,
                    pass_sep_, min_pass_sep_ * 0.3, attempt_max_sep_, dbg_allow_ ? 1 : 0,
                    dbg_zone_ ? 1 : 0, dbg_zone_ok_ ? 1 : 0,
                    dbg_feasible_ ? 1 : 0, dbg_latched_ ? 1 : 0, dbg_width_);
        logAttemptFunnel(f, "失敗", elapsed);
      } else if (stalled || lat_stalled || elapsed > attempt_timeout_ ||
                 infeasible_stop || nopass_stop || finish_stop) {
        attempt_active_ = false;
        attempt_ng_++;
        // 失敗したので側の確定を解く。
        side_committed_ = false;
        if (finish_stop) {
          ++attempt_finish_abort_n_;
          diagWarn("抜き切れないのでやめる",
                   "抜き切れないのでやめる 累計%zu target=%s 所要=%.1fs idx=%zu "
                   "要る距離%.1fm > 区間の残り%.1fm(+余裕%.1fm) 自車%.1fkm/h "
                   "横間隔%.2fm(最大%.2fm) 車間%.1fm",
                   attempt_finish_abort_n_, attempt_target_.c_str(), elapsed, f.ei,
                   finish_need_m_, finish_left_m_, straight_finish_margin_,
                   std::abs(f.ev) * 3.6, pass_sep_, attempt_max_sep_, c.best_gap);
        }
        attempt_finish_short_since_ = -1.0;
        if (nopass_stop) {
          ++attempt_nopass_n_;
          diagLog("抜けないので降りる",
                  "抜けないので降りる 累計%zu target=%s 所要=%.1fs idx=%zu "
                  "自車%.1fkm/h 最大横間隔=%.2fm "
                  "(どちらの側も必要間隔に届かない状態が%.1fs続いた)",
                  attempt_nopass_n_, attempt_target_.c_str(), elapsed, f.ei,
                  std::abs(f.ev) * 3.6, attempt_max_sep_, attempt_giveup_time_);
        }
        attempt_nopass_since_ = -1.0;
        if (stalled) {
          attempt_stall_name_ = attempt_target_;
          attempt_stall_until_ = now.seconds() + attempt_stall_cool_;
        }
        if (lat_stalled) {
          attempt_stall_name_ = attempt_target_;
          attempt_stall_until_ = now.seconds() + attempt_stall_cool_;
        }
        RCLCPP_INFO(get_logger(),
                    "追越試行 打切 target=%s 所要=%.1fs ブースト=%d rank=%d 理由=%s "
                    "最大横間隔=%.2f(要%.2f)",
                    attempt_target_.c_str(), elapsed,
                    boost_used_ - attempt_boosts_, rank_,
                    stalled ? "進展なし"
                            : (lat_stalled ? "横に出られない"
                                           : (infeasible_stop ? "壁に当たる" : "時間切れ")),
                    attempt_max_sep_, commit_sep_);
        logAttemptFunnel(f, "打切", elapsed);
      }
    }
  }

}

// 衝突回避層の結果を最優先で指令へ反映する。
// 追い越しの都合より当たらないことを優先する(Crash 10秒 / Wall 5秒)。
void V2XOvertaker::applyAvoidance(const Frame & f, PlanCtx & c)
{
  const rclcpp::Time now = f.now;

  // --- 衝突回避を最優先で適用する
  // 追い越しの都合より、当たらないことを優先する。
  // Crash は 10 秒 5km/h、Wall は 5 秒 5km/h と罰則が重く、
  // 追い越し1回の利得より損失が大きい。
  dbg_avoid_offset_ = c.avoid_offset;
  dbg_avoid_cap_ = c.avoid_speed_cap;
  if (std::abs(c.avoid_offset) > 1e-3) {
    c.requestLat(c.avoid_offset, PlanCtx::LatPrio::kCollision, "衝突回避");
    if (c.blocker.empty()) {
      c.blocker = "回避";
    }
  }
  if (c.avoid_speed_cap >= 0.0) {
    c.requestCap(c.avoid_speed_cap, "衝突回避");
    if ((this->now() - last_avoid_log_).seconds() > 2.0) {
      last_avoid_log_ = this->now();
      RCLCPP_INFO(get_logger(), "衝突回避 減速=%.1fkm/h 横=%.2fm",
                  c.avoid_speed_cap * 3.6, c.avoid_offset);
    }
  }
}

void V2XOvertaker::holdAttemptSide(PlanCtx & c)
{
  const bool lateral_avoidance = std::abs(c.avoid_offset) > 0.05;
  if (attempt_hold_side_ && attempt_active_ && !c.blocker.empty() &&
      !lateral_avoidance) {
    // recordAttempt は回避層より前に、計画から出た横目標を attempt_offset_ へ
    // 保存している。最小間隔だけを保持すると、計画+2.47mが+1.30mへ縮み、
    // 相手が寄った瞬間に横間隔が消える。計画ラインを基本値として保持する。
    double want = (std::abs(attempt_offset_) > 1e-3)
                    ? attempt_offset_ : side_sign_ * attempt_hold_sep_;
    if (spot_valid_ && attempt_target_ == spot_target_ &&
        spot_side_ * spot_offset_ > 0.0) {
      want = spot_offset_;
    }
    // コリドアの余地には従う(壁側へは出ない)
    const double held = std::clamp(want, room_lo_, room_hi_);
    // すでに同じ側へ十分寄っているならそのまま。足りないときだけ引き上げる。
    if (side_sign_ * c.target_offset < side_sign_ * held) {
      c.target_offset = held;
    }
  }
}


// ===================================================================
// 最終判断と出力
// ===================================================================

// 壁を最優先で避ける。避けきれないときだけ相手側へ寄る。
// ここは横目標を決める最後の段。以降はレート制限を掛けて出すだけ。
void V2XOvertaker::avoidWall(const Frame & f, PlanCtx & c)
{
  // 状態機械が読む「壁に押し出される」の記録。毎周期ここで落としてから、
  // 実際に判定した場所で入れ直す。早期 return した周期は偽のまま残る。
  wall_push_now_ = false;
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  if (corridor_.lo.size() == n && !wedge_active_) {
    double margin = wall_margin_;
    if (!corridor_.radius.empty() && tight_radius_ > 0.0) {
      const double r = corridor_.radius[ei];
      if (r < tight_radius_) {
        margin += wall_margin_tight_ *
                  std::clamp((tight_radius_ - r) / tight_radius_, 0.0, 1.0);
      }
    }
    if (c.stop_avoid_active) {
      margin = std::min(margin, wall_margin_stopped_);
    }
    if (ov_pass_wall_relax_ && (ovPassing() || attempt_active_)) {
      margin = std::min(margin, straight_pass_wall_);
    }
    if (straight_pass_now_) {
      margin = std::min(margin, straight_pass_wall_);
    }
    double wall_lo_raw = corridor_.lo[ei];
    double wall_hi_raw = corridor_.hi[ei];
    if (wall_body_span_) {
      double back = 0.0;
      for (std::size_t k = 1; k < n && back < geom_rear_; ++k) {
        const std::size_t i0 = (ei + n - k + 1) % n, i1 = (ei + n - k) % n;
        back += std::hypot(in.points[i0].pose.position.x - in.points[i1].pose.position.x,
                           in.points[i0].pose.position.y - in.points[i1].pose.position.y);
        wall_lo_raw = std::max(wall_lo_raw, corridor_.lo[i1]);
        wall_hi_raw = std::min(wall_hi_raw, corridor_.hi[i1]);
      }
      double fwd = 0.0;
      for (std::size_t k = 0; k < n && fwd < geom_front_; ++k) {
        const std::size_t i0 = (ei + k) % n, i1 = (ei + k + 1) % n;
        fwd += std::hypot(in.points[i1].pose.position.x - in.points[i0].pose.position.x,
                          in.points[i1].pose.position.y - in.points[i0].pose.position.y);
        wall_lo_raw = std::max(wall_lo_raw, corridor_.lo[i1]);
        wall_hi_raw = std::min(wall_hi_raw, corridor_.hi[i1]);
      }
    }
    double wall_lo = std::min(wall_lo_raw + margin, 0.0);
    double wall_hi = std::max(wall_hi_raw - margin, 0.0);
    if (wall_hi > wall_lo) {
      const double want = c.latWant();
      const double safe = std::clamp(want, wall_lo, wall_hi);
      c.boundLat(wall_lo, wall_hi, "壁回避");
      const bool pushed_to_wall = std::abs(want - safe) > 1e-3;
      wall_push_now_ = pushed_to_wall;

      bool cannot_pass = std::abs(want - safe) > stop_avoid_crush_;
      if (cannot_pass && stop_avoid_fit_ && c.stop_avoid_have_gap &&
          safe >= c.stop_avoid_lo && safe <= c.stop_avoid_hi) {
        cannot_pass = false;
      }
      const bool crush_in_range =
        (c.stop_avoid_dist < 0.0) || (c.stop_avoid_dist <= stop_avoid_crush_range_);
      if (c.stop_avoid_active && cannot_pass && crush_in_range) {
        c.requestCap(c.stop_avoid_v_stop, "壁回避");
        if ((this->now() - last_crush_log_).seconds() > 1.0) {
          last_crush_log_ = this->now();
          RCLCPP_WARN(get_logger(),
                      "停止車を避けきれない 横目標 %.2f が壁帯[%.2f,%.2f]で "
                      "%.2f に潰された -> 上限 %.1fkm/h へ減速 (停止車まで%.1fm)",
                      want, wall_lo, wall_hi, safe, c.stop_avoid_v_stop * 3.6,
                      c.stop_avoid_dist);
        }
      }
      if (pushed_to_wall) {
        const double wall_dir = (want > safe) ? 1.0 : -1.0;
        bool crash_risk = false;
        double nearest_lat = 0.0;
        bool have_car = false;
        {
          const auto & qq = odom_->pose.pose.orientation;
          const double yaw = std::atan2(2.0 * (qq.w * qq.z + qq.x * qq.y),
                                        1.0 - 2.0 * (qq.y * qq.y + qq.z * qq.z));
          double best_d = 1e9;
          for (const auto & kv : others_) {
            const OtherState & o = kv.second;
            if (!o.valid || (this->now() - o.stamp).seconds() > v2x_timeout_) { continue; }
            const double dx = o.x - ex, dy = o.y - ey;
            const double dist = std::hypot(dx, dy);
            if (dist > near_radius_) { continue; }
            const double fwd = dx * std::cos(yaw) + dy * std::sin(yaw);
            const size_t oi3 = nearest(in, o.x, o.y);
            double nx3, ny3;
            normalAt(in, oi3, nx3, ny3);
            const auto & lp3 = in.points[oi3].pose.position;
            const double olat3 = (o.x - lp3.x) * nx3 + (o.y - lp3.y) * ny3;
            // 壁から逃げる側にいる相手だけが問題になる
            const double side = (olat3 - my_lat_for_target_) * (-wall_dir);
            if (side < 0.0) { continue; }
            if (dist < best_d) { best_d = dist; nearest_lat = olat3; have_car = true; }
            // 自分の前に相手の車体がある = 寄せると前から当てる = Crash
            if (fwd > crash_front_near_ && fwd < crash_front_far_) { crash_risk = true; }
          }
        }
        // 他車がいないなら、帯に収めた時点で壁の危険は無い。減速しない。
        if (have_car && !crash_risk) {
          double clearance = -1.0;
          double deficit = 0.0;
          double toward;
          if (wall_pick_legacy_) {
            toward = nearest_lat + crash_safe_sep_ * wall_dir;
          } else {
            const double edge = (wall_dir > 0.0) ? corridor_.hi[ei] : corridor_.lo[ei];
            clearance = (edge - safe) * (-wall_dir) + corridor_extra_;
            deficit = std::max(0.0, wall_pick_need_ - clearance);
            toward = safe + deficit * (-wall_dir);
            const double limit = nearest_lat + crash_safe_sep_ * wall_dir;
            if ((toward - limit) * (-wall_dir) > 0.0) { toward = limit; }
            // 壁側へ戻すことは絶対にしない
            if ((toward - safe) * (-wall_dir) < 0.0) { toward = safe; }
          }
          // ここは「相手側へ寄れ」という**意図**。壁に押されて逃げ場が
          // 無いときの安全指令なので、最上位(kCollision)で出す。
          const double toward_c = std::clamp(toward, wall_lo, wall_hi);
          c.requestLat(toward_c, PlanCtx::LatPrio::kCollision, "壁回避");
          if ((this->now() - last_wallpick_log_).seconds() > 1.0) {
            last_wallpick_log_ = this->now();
            RCLCPP_INFO(get_logger(),
                        "壁回避 相手側へ寄る 横目標 %.2f -> %.2f (壁側=%s "
                        "相手横=%.2f 自車横=%.2f 帯=[%.2f,%.2f] "
                        "実余裕=%.2f 不足=%.2f 押し量=%.2f)",
                        want, toward_c, wall_dir > 0 ? "左" : "右",
                        nearest_lat, my_lat_for_target_, wall_lo, wall_hi,
                        clearance, deficit, (toward_c - safe) * (-wall_dir));
          }
        } else if (have_car && crash_risk) {
          const double base = std::max<double>(in.points[ei].longitudinal_velocity_mps, 0.0);
          const double slow = std::max(base * wall_brake_ratio_, min_follow_speed_);
          c.requestCap(slow, "壁回避");
          if ((this->now() - last_wallpick_log_).seconds() > 1.0) {
            last_wallpick_log_ = this->now();
            RCLCPP_INFO(get_logger(),
                        "壁回避 減速で両方避ける 横目標 %.2f -> %.2f 上限 %.1fkm/h "
                        "(壁側=%s 前に相手あり)",
                        want, safe, slow * 3.6, wall_dir > 0 ? "左" : "右");
          }
        }
      }
    }
  }

}

void V2XOvertaker::diagEmit(const char * type, bool warn, const char * text)
{
  if (warn) { RCLCPP_WARN(get_logger(), "%s", text); }
  else      { RCLCPP_INFO(get_logger(), "%s", text); }
  if (!diag_pub_) { return; }
  std::string esc;
  esc.reserve(std::strlen(text) + 32);
  for (const char * q = text; *q; ++q) {
    switch (*q) {
      case '"':  esc += "\\\""; break;
      case '\\': esc += "\\\\"; break;
      case '\n': esc += "\\n";  break;
      case '\r': break;
      default:   esc.push_back(*q);
    }
  }
  char head[192];
  std::snprintf(head, sizeof(head),
                "{\"t\":%.3f,\"node\":\"%s\",\"lvl\":\"%s\",\"type\":\"%s\",\"msg\":\"",
                this->now().seconds(), this->get_name(), warn ? "warn" : "info", type);
  std_msgs::msg::String m;
  m.data = std::string(head) + esc + "\"}";
  diag_pub_->publish(m);
}

void V2XOvertaker::diagLog(const char * type, const char * fmt, ...)
{
  char buf[1400];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  diagEmit(type, false, buf);
}

void V2XOvertaker::diagWarn(const char * type, const char * fmt, ...)
{
  char buf[1400];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  diagEmit(type, true, buf);
}

void V2XOvertaker::publishSteerLimit(double lo, double hi, bool viol)
{
  if (!steer_limit_pub_) { return; }
  geometry_msgs::msg::Vector3Stamped m;
  m.header.stamp = this->now();
  m.header.frame_id = "base_link";
  m.vector.x = lo;
  m.vector.y = hi;
  m.vector.z = viol ? 1.0 : 0.0;
  steer_limit_pub_->publish(m);
}

// ===================================================================。

namespace
{
// Felzenszwalb & Huttenlocher の厳密 EDT(1 次元)。
// f は二乗距離の初期値、d へ結果(二乗距離)を書く。
void edt1d(const std::vector<double> & f, std::vector<double> & d, int n)
{
  static thread_local std::vector<int> v;
  static thread_local std::vector<double> z;
  v.assign(static_cast<std::size_t>(n), 0);
  z.assign(static_cast<std::size_t>(n) + 1, 0.0);
  const double kInf = 1e20;
  int k = 0;
  v[0] = 0;
  z[0] = -kInf;
  z[1] = kInf;
  for (int q = 1; q < n; ++q) {
    double s = 0.0;
    while (true) {
      const double den = 2.0 * static_cast<double>(q) - 2.0 * static_cast<double>(v[k]);
      s = ((f[q] + static_cast<double>(q) * q) -
           (f[v[k]] + static_cast<double>(v[k]) * v[k])) / den;
      if (s <= z[k] && k > 0) { --k; } else { break; }
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = kInf;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < static_cast<double>(q)) { ++k; }
    const double dx = static_cast<double>(q) - static_cast<double>(v[k]);
    d[q] = dx * dx + f[v[k]];
  }
}

// mask[i] != 0 のセルを「種」として、各セルから最寄りの種までの距離[セル]を返す。
std::vector<double> edt2d(const std::vector<uint8_t> & mask, int w, int h)
{
  const double kInf = 1e20;
  std::vector<double> d(static_cast<std::size_t>(w) * h, 0.0);
  for (std::size_t i = 0; i < d.size(); ++i) { d[i] = mask[i] ? 0.0 : kInf; }
  std::vector<double> f(static_cast<std::size_t>(std::max(w, h)));
  std::vector<double> t(static_cast<std::size_t>(std::max(w, h)));
  // 列方向
  for (int c = 0; c < w; ++c) {
    for (int r = 0; r < h; ++r) { f[r] = d[static_cast<std::size_t>(r) * w + c]; }
    edt1d(f, t, h);
    for (int r = 0; r < h; ++r) { d[static_cast<std::size_t>(r) * w + c] = t[r]; }
  }
  // 行方向
  for (int r = 0; r < h; ++r) {
    for (int c = 0; c < w; ++c) { f[c] = d[static_cast<std::size_t>(r) * w + c]; }
    edt1d(f, t, w);
    for (int c = 0; c < w; ++c) {
      d[static_cast<std::size_t>(r) * w + c] = std::sqrt(std::max(0.0, t[c]));
    }
  }
  return d;
}

// PGM(P2 テキスト / P5 バイナリ)を読む。外部ライブラリは使わない。
bool readPgm(const std::string & path, int & w, int & h, std::vector<uint8_t> & px)
{
  std::ifstream ifs(path, std::ios::binary);
  if (!ifs) { return false; }
  std::string blob((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
  if (blob.size() < 8) { return false; }
  std::size_t i = 0;
  auto skip_ws = [&]() {
    while (i < blob.size()) {
      const char ch = blob[i];
      if (ch == '#') { while (i < blob.size() && blob[i] != '\n') { ++i; } }
      else if (std::isspace(static_cast<unsigned char>(ch))) { ++i; }
      else { break; }
    }
  };
  auto token = [&](std::string & out) {
    skip_ws();
    const std::size_t a = i;
    while (i < blob.size() && !std::isspace(static_cast<unsigned char>(blob[i]))) { ++i; }
    if (i <= a) { return false; }
    out = blob.substr(a, i - a);
    return true;
  };
  std::string magic, sw, sh, smax;
  if (!token(magic) || !token(sw) || !token(sh) || !token(smax)) { return false; }
  if (magic != "P2" && magic != "P5") { return false; }
  w = std::atoi(sw.c_str());
  h = std::atoi(sh.c_str());
  const int maxv = std::atoi(smax.c_str());
  if (w <= 0 || h <= 0 || maxv <= 0) { return false; }
  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  px.assign(n, 0);
  if (magic == "P5") {
    ++i;                       // 最大値の直後の空白 1 文字だけを読み飛ばす
    if (blob.size() < i + n) { return false; }
    for (std::size_t k = 0; k < n; ++k) {
      px[k] = static_cast<uint8_t>(blob[i + k]);
    }
  } else {
    std::string tk;
    for (std::size_t k = 0; k < n; ++k) {
      if (!token(tk)) { return false; }
      int val = std::atoi(tk.c_str());
      val = std::clamp(val * 255 / maxv, 0, 255);
      px[k] = static_cast<uint8_t>(val);
    }
  }
  return true;
}
}  // namespace

// 占有格子地図(yaml + pgm)を読み、符号付き距離場を作る。
// 失敗したら false を返し、呼び出し側が機能を丸ごと無効化する。
bool V2XOvertaker::loadOccGrid()
{
  occ_ = OccGrid{};
  std::ifstream ifs(occ_map_yaml_);
  if (!ifs) { return false; }

  std::string image;
  double res = 0.0, ox = 0.0, oy = 0.0;
  double occ_th = 0.65, free_th = 0.196;
  int negate = 0;
  int origin_seen = 0;
  bool in_origin = false;
  std::string line;
  auto after = [](const std::string & s, const char * key) {
    const std::size_t p = s.find(key);
    if (p == std::string::npos) { return std::string(); }
    std::string v = s.substr(p + std::strlen(key));
    // 前後の空白・引用符・角括弧・カンマを落とす
    std::string out;
    for (char ch : v) {
      if (ch == '#') { break; }
      out.push_back(ch);
    }
    return out;
  };
  while (std::getline(ifs, line)) {
    if (!line.empty() && line.back() == '\r') { line.pop_back(); }
    std::string t = line;
    const std::size_t hp = t.find('#');
    if (hp == 0) { continue; }
    if (t.find("image:") != std::string::npos) {
      std::string v = after(t, "image:");
      std::size_t a = v.find_first_not_of(" \t\"'");
      std::size_t b = v.find_last_not_of(" \t\"'\r\n");
      if (a != std::string::npos) { image = v.substr(a, b - a + 1); }
      in_origin = false;
    } else if (t.find("resolution:") != std::string::npos) {
      res = std::atof(after(t, "resolution:").c_str());
      in_origin = false;
    } else if (t.find("occupied_thresh:") != std::string::npos) {
      occ_th = std::atof(after(t, "occupied_thresh:").c_str());
      in_origin = false;
    } else if (t.find("free_thresh:") != std::string::npos) {
      free_th = std::atof(after(t, "free_thresh:").c_str());
      in_origin = false;
    } else if (t.find("negate:") != std::string::npos) {
      negate = std::atoi(after(t, "negate:").c_str());
      in_origin = false;
    } else if (t.find("origin:") != std::string::npos) {
      // インライン形式 origin: [x, y, yaw] とブロック形式の両方に対応する
      std::string v = after(t, "origin:");
      for (char & ch : v) { if (ch == '[' || ch == ']' || ch == ',') { ch = ' '; } }
      std::istringstream is(v);
      double d = 0.0;
      while (is >> d) {
        if (origin_seen == 0) { ox = d; }
        else if (origin_seen == 1) { oy = d; }
        ++origin_seen;
      }
      in_origin = (origin_seen < 2);
    } else if (in_origin) {
      // ブロック形式 ("  - 89608.69" が 3 行続く)
      const std::size_t a = t.find_first_not_of(" \t");
      if (a == std::string::npos || t[a] != '-') { in_origin = false; continue; }
      std::istringstream is(t.substr(a + 1));
      double d = 0.0;
      if (is >> d) {
        if (origin_seen == 0) { ox = d; }
        else if (origin_seen == 1) { oy = d; }
        ++origin_seen;
        if (origin_seen >= 3) { in_origin = false; }
      } else {
        in_origin = false;
      }
    }
  }
  if (image.empty() || res <= 0.0 || origin_seen < 2) { return false; }

  std::string dir;
  const std::size_t sl = occ_map_yaml_.find_last_of('/');
  if (sl != std::string::npos) { dir = occ_map_yaml_.substr(0, sl + 1); }
  const std::string pgm = (image.front() == '/') ? image : (dir + image);

  int w = 0, h = 0;
  std::vector<uint8_t> px;
  if (!readPgm(pgm, w, h, px)) { return false; }

  const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
  std::vector<uint8_t> is_occ(n, 0), is_free(n, 0);
  std::size_t n_occ = 0;
  for (std::size_t k = 0; k < n; ++k) {
    // map_server と同じ規約。negate=0 なら暗いほど占有。
    const double p = negate ? (static_cast<double>(px[k]) / 255.0)
                            : ((255.0 - static_cast<double>(px[k])) / 255.0);
    if (p < free_th) {
      is_free[k] = 1;
    } else if (p > occ_th) {
      is_occ[k] = 1;
      ++n_occ;
    } else {
      // 未知(free_thresh 〜 occupied_thresh)。コース外の可能性があるので
      // 占有側へ倒す(保守側)。
      is_occ[k] = 1;
      ++n_occ;
    }
  }
  if (n_occ == 0 || n_occ == n) { return false; }

  // 符号付き距離場。自由セルは「最寄りの占有セルまで」を正、
  // 占有セルは「最寄りの自由セルまで」を負にする。
  const std::vector<double> d_to_occ = edt2d(is_occ, w, h);
  const std::vector<double> d_to_free = edt2d(is_free, w, h);
  occ_.sd.assign(n, 0.0f);
  const double cap = std::max(occ_clear_search_, 0.1);
  for (std::size_t k = 0; k < n; ++k) {
    const double v = is_free[k] ? (d_to_occ[k] * res) : (-d_to_free[k] * res);
    occ_.sd[k] = static_cast<float>(std::clamp(v, -cap, cap));
  }
  occ_.w = w;
  occ_.h = h;
  occ_.res = res;
  occ_.ox = ox;
  occ_.oy = oy;
  occ_.n_occ = n_occ;
  occ_.ok = true;
  return true;
}

// (x, y) の符号付き余裕[m]。地図外は「コース外」なので最大の食い込み扱い。
double V2XOvertaker::occSignedDist(double x, double y) const
{
  const double cap = std::max(occ_clear_search_, 0.1);
  if (!occ_.ok) { return cap; }
  const int c = static_cast<int>(std::floor((x - occ_.ox) / occ_.res));
  const int rb = static_cast<int>(std::floor((y - occ_.oy) / occ_.res));
  const int r = occ_.h - 1 - rb;
  if (c < 0 || c >= occ_.w || r < 0 || r >= occ_.h) { return -cap; }
  return static_cast<double>(occ_.sd[static_cast<std::size_t>(r) * occ_.w + c]);
}

// 後軸中心 (cx, cy)・向き yaw の車体矩形の、壁までの最小余裕[m]。
// 四隅と、辺上を occ_sample_step_ で刻んだ点を格子へ引く。
double V2XOvertaker::bodyClearance(double cx, double cy, double yaw, double t_ahead) const
{
  (void)t_ahead;                 // 静的地図なので時刻に依らない
  const double cap = std::max(occ_clear_search_, 0.1);
  if (!occ_.ok) { return cap; }
  // ここは車体の角がどこにあるかという**幾何**なので、。
  const double hw = geom_half_width_;
  const double fr = geom_front_;   // 後軸 -> 前端 (1.087+0.467)
  const double re = geom_rear_;    // 後軸 -> 後端 (0.510)
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  const double step = std::max(occ_sample_step_, 0.02);
  double worst = cap;
  auto probe = [&](double lx, double ly) {
    const double d = occSignedDist(cx + lx * cs - ly * sn, cy + lx * sn + ly * cs);
    if (d < worst) { worst = d; }
  };
  // 前後方向の辺(左右 2 本)。端点(四隅)を必ず含める。
  {
    const int m = static_cast<int>(std::ceil((fr + re) / step));
    for (int k = 0; k <= m; ++k) {
      const double lx = -re + std::min(static_cast<double>(k) * step, fr + re);
      probe(lx, hw);
      probe(lx, -hw);
    }
  }
  // 左右方向の辺(前後 2 本)。
  {
    const int m = static_cast<int>(std::ceil((2.0 * hw) / step));
    for (int k = 0; k <= m; ++k) {
      const double ly = -hw + std::min(static_cast<double>(k) * step, 2.0 * hw);
      probe(fr, ly);
      probe(-re, ly);
    }
  }
  return worst;
}

bool V2XOvertaker::bodyHits(double cx, double cy, double yaw, double t_ahead) const
{
  return bodyClearance(cx, cy, yaw, t_ahead) < 0.0;
}

bool V2XOvertaker::otherHits(double cx, double cy, double yaw, double t_ahead) const
{
  const double hw = geom_half_width_;
  const double fr = geom_front_;
  const double re = geom_rear_;
  const double half_len = 0.5 * (fr + re);
  const double off = 0.5 * (fr - re);     // 後軸中心から矩形中心までの前方距離
  const double ax = std::cos(yaw), ay = std::sin(yaw);
  const double ex = cx + off * ax, ey = cy + off * ay;

  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid) { continue; }
    // 静止物として扱わない。同じ速度で走る前車が「壁」になってしまう。
    const double px = o.x + o.vx * t_ahead;
    const double py = o.y + o.vy * t_ahead;
    const double sp = std::hypot(o.vx, o.vy);
    const double oyaw = (sp > 0.5) ? std::atan2(o.vy, o.vx) : yaw;
    const double bx = std::cos(oyaw), by = std::sin(oyaw);
    const double ox2 = px + off * bx, oy2 = py + off * by;

    const double dx = ox2 - ex, dy = oy2 - ey;
    if (std::hypot(dx, dy) > 2.0 * (half_len + hw)) { continue; }   // 早期棄却

    // 分離軸: 自車の前後 / 左右、相手の前後 / 左右
    const double au[2] = {ax, ay};
    const double av[2] = {-ay, ax};
    const double bu[2] = {bx, by};
    const double bv[2] = {-by, bx};
    const double * axes[4] = {au, av, bu, bv};
    bool sep = false;
    for (int i = 0; i < 4; ++i) {
      const double * L = axes[i];
      const double dist = std::abs(dx * L[0] + dy * L[1]);
      const double ra = half_len * std::abs(au[0] * L[0] + au[1] * L[1]) +
                        hw * std::abs(av[0] * L[0] + av[1] * L[1]);
      const double rb = half_len * std::abs(bu[0] * L[0] + bu[1] * L[1]) +
                        hw * std::abs(bv[0] * L[0] + bv[1] * L[1]);
      if (dist > ra + rb) { sep = true; break; }
    }
    if (!sep) { return true; }
  }
  return false;
}

// 占有格子で舵角の許容範囲を求める。PlanCtx には一切触らない(副作用なし)。
V2XOvertaker::OccSteerResult V2XOvertaker::occSteerGuard(const Frame & f) const
{
  OccSteerResult r;
  if (!occ_enable_ || !occ_.ok || !odom_) { return r; }
  const int bins = std::max(occ_steer_bins_, 3);
  const double smax = std::max(veh_max_steer_, 0.05);
  const double v = std::abs(f.ev);
  if (v < 0.5) { return r; }

  const auto & q = odom_->pose.pose.orientation;
  const double yaw0 = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double dt = std::max(wall_guard_dt_, 0.02);
  const int steps = std::max(1,
    static_cast<int>(std::ceil(std::max(wall_guard_horizon_, 0.05) / dt)));
  const double wb = std::max(veh_wheel_base_, 0.3);   // 予測は等価ホイールベース

  double lo = 1e9, hi = -1e9;
  int surv = 0;
  int other_hit = 0;
  double best_clear = -1e9;
  double best_depth = -1e9;
  double best_depth_steer = 0.0;

  for (int b = 0; b < bins; ++b) {
    const double s = -smax + 2.0 * smax * static_cast<double>(b) /
                             static_cast<double>(bins - 1);
    const double curv = std::tan(s) / wb;
    double x = f.ex, y = f.ey, yaw = yaw0;
    double min_clear = 1e9;
    bool hit_other = false;
    for (int k = 1; k <= steps; ++k) {
      yaw += v * curv * dt;
      x += v * std::cos(yaw) * dt;
      y += v * std::sin(yaw) * dt;
      const double t = static_cast<double>(k) * dt;
      const double cl = bodyClearance(x, y, yaw, t);
      if (cl < min_clear) { min_clear = cl; }
      if (!hit_other && otherHits(x, y, yaw, t)) { hit_other = true; }
    }
    if (hit_other) { ++other_hit; }        // 記録するだけ。候補からは落とさない
    if (min_clear >= 0.0) {
      ++surv;
      lo = std::min(lo, s);
      hi = std::max(hi, s);
      if (min_clear > best_clear) { best_clear = min_clear; }
    }
    if (min_clear > best_depth) { best_depth = min_clear; best_depth_steer = s; }
  }

  r.used = true;
  r.bins = bins;
  r.surv = surv;
  r.blocked = (other_hit >= bins);
  if (surv > 0) {
    r.lo = -smax;
    r.hi = smax;
    r.best_clear = best_clear;
  } else {
    // 逃げ場が無い。フェイルオープンせず「最も食い込みが浅い舵角」へ固定する。
    // 食い込みが最小の舵角は幾何的に最も壁と平行に近く、正面から突っ込むより
    // 接触が浅くなる(= 壁に沿う)。
    r.forced = true;
    r.lo = best_depth_steer;
    r.hi = best_depth_steer;
    r.forced_depth = best_depth;
    r.best_clear = best_depth;
  }
  return r;
}


// 方位差 e[rad](正=経路に対して左を向く)のときの、後軸中心から見た。
double V2XOvertaker::curveRadiusAt(std::size_t idx) const
{
  if (radius_min_.size() > idx && radius_min_[idx] > 0.0) { return radius_min_[idx]; }
  if (corridor_.radius.size() > idx && corridor_.radius[idx] > 0.0) {
    return corridor_.radius[idx];
  }
  return -1.0;
}

void V2XOvertaker::bodyExtent(double e, double & ext_left, double & ext_right,
                              double curve_radius, int curve_sign) const
{
  const double F = geom_front_;
  const double R = geom_rear_;
  const double hw = geom_half_width_;
  const double si = std::sin(e);
  const double co = std::abs(std::cos(e));
  // 四隅の横座標は d*sin(e) + w*cos(e) (d は前後、w は左右)。
  ext_left  = std::max(F * si, -R * si) + hw * co;
  ext_right = std::max(-F * si, R * si) + hw * co;

  if (size_pad_ > 0.0 && !attempt_active_) {
    ext_left  += size_pad_;
    ext_right += size_pad_;
  }
  if (body_curve_pad_ && curve_radius > 0.5 && curve_radius < 1e6 && curve_sign != 0) {
    const double swing_out = F * F / (2.0 * curve_radius);   // 前端が外へ
    const double cut_in    = R * R / (2.0 * curve_radius);   // 後端が内へ
    if (curve_sign > 0) {          // 左カーブ: 内=左 / 外=右
      ext_right += swing_out;
      ext_left  += cut_in;
    } else {                       // 右カーブ: 内=右 / 外=左
      ext_left  += swing_out;
      ext_right += cut_in;
    }
  }
}

void V2XOvertaker::holdSideAlongside(const Frame & f, PlanCtx & c)
{
  if (!hold_side_alongside_ || !odom_ || line_x_.empty() || !my_prog_init_) { return; }
  const auto & q = odom_->pose.pose.orientation;
  const double myyaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const std::size_t a = (f.ei + f.n - 1) % f.n, b = (f.ei + 1) % f.n;
  const double tth = std::atan2(
    f.in.points[b].pose.position.y - f.in.points[a].pose.position.y,
    f.in.points[b].pose.position.x - f.in.points[a].pose.position.x);
  double e = myyaw - tth;
  while (e > M_PI) { e -= 2.0 * M_PI; }
  while (e < -M_PI) { e += 2.0 * M_PI; }
  double ext_l = 0.0, ext_r = 0.0;
  bodyExtent(e, ext_l, ext_r);
  // 縦に重なっているとみなす距離。自車と相手の全長ぶん。
  const double body_len = geom_front_ + geom_rear_;
  const double my_lat = my_lat_for_target_;
  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid || !o.prog_init) { continue; }
    if ((f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
    if (std::abs(o.prog - my_prog_) > body_len) { continue; }   // 縦に重なっていない
    const std::size_t oi = nearest(f.in, o.x, o.y);
    double nx, ny;
    normalAt(f.in, oi, nx, ny);
    const auto & lp = f.in.points[oi].pose.position;
    const double olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;
    const double sep = std::abs(my_lat - olat);
    // 相手がいる側の張り出し + 相手の半幅 + 余裕。これより離れていれば縛らない。
    const double before = c.latWant();
    const double side_delta = olat - my_lat;
    // 中心がほぼ一致した場合だけ、意図した退避方向と反対側に相手を置く。
    // これで完全重なりから side_sign に関係なく意図した側へ離れられる。
    const bool on_left = side_delta > 0.05 ? true
                       : side_delta < -0.05 ? false
                       : before < olat;
    const double need = (on_left ? ext_l : ext_r) + geom_half_width_ + alongside_extra_;
    // 092203/d2: measured separation was still 1.66m (safe), but the selected。
    const bool launch_transition = !start_merge_done_;
    if (launch_transition && sep >= need) { continue; }
    const double safe_limit = launch_transition ? my_lat
      : v2x_overtaker::alongsideSafeLimit(on_left, my_lat, olat, need);
    const bool toward_opponent = on_left ? before > safe_limit : before < safe_limit;
    if (!toward_opponent) { continue; }
    if (on_left) {
      c.boundLat(c.lat_lo, safe_limit, "並走中は安全境界を越えない");
    } else {
      c.boundLat(safe_limit, c.lat_hi, "並走中は安全境界を越えない");
    }
    const double after = c.latWant();
    if (std::abs(before - after) > 0.05 &&
        (this->now() - last_alongside_log_).seconds() > 1.0)
    {
      last_alongside_log_ = this->now();
      diagLog("並走中は寄せない",
              "並走中は寄せない target=%s %s 縦のずれ%.1fm 横間隔%.2fm(要%.2f) "
              "方位差%.1fdeg 張り出し左%.2f 右%.2f 横目標 %.2f -> %.2f",
              kv.first.c_str(), on_left ? "相手は左" : "相手は右",
              o.prog - my_prog_, sep, need, e * 180.0 / M_PI,
              ext_l, ext_r, before, after);
    }
  }
}

void V2XOvertaker::wallGuard(const Frame & f, PlanCtx & c)
{
  // 作動しなかったときも「予測した最小余裕」は毎周期残す(観測用)。
  wall_guard_min_room_ = 1e9;

  const double steer_free = std::max(veh_max_steer_, 0.0);
  if (!wall_guard_enable_) {
    const OccSteerResult og_only = occSteerGuard(f);
    if (og_only.used) {
      publishSteerLimit(og_only.lo, og_only.hi, og_only.forced);
      if ((this->now() - last_wall_guard_log_).seconds() > 2.0) {
        last_wall_guard_log_ = this->now();
        diagLog("壁予測", "壁予測(格子) 監視のみ 候補=%d 生存=%d 舵角範囲=[%.3f,%.3f] 強制=%d idx=%zu",
          og_only.bins, og_only.surv, og_only.lo, og_only.hi,
          og_only.forced ? 1 : 0, f.ei);
      }
    }
    return;
  }

  const OccSteerResult og = occSteerGuard(f);
  // 舵角範囲の publish はここを通す。占有格子が使えるならその値を優先する。
  auto emit = [&](double lo_csv, double hi_csv, bool viol_csv) {
    if (og.used) {
      // 検知フラグは「最終回避を強制したか」にする。surv<bins は格子有効時ほぼ
      // 常時 1 になり、意味を持たない指標だった。
      publishSteerLimit(og.lo, og.hi, og.forced || viol_csv);
    } else {
      publishSteerLimit(lo_csv, hi_csv, viol_csv);
    }
  };
  if (og.used && (f.now - last_occ_log_).seconds() > 2.0) {
    last_occ_log_ = f.now;
    if (og.forced) {
      diagWarn("壁予測", "壁予測(格子) 最終回避 舵角=%.3frad(%.1fdeg) 食い込み=%.2fm 候補=%d 生存=0 idx=%zu",
        og.lo, og.lo * 180.0 / M_PI, og.forced_depth, og.bins, f.ei);
    } else {
      RCLCPP_INFO(get_logger(),
        "壁予測(格子) 候補=%d 生存=%d 舵角範囲=[%+.2f,%+.2f] 前方閉塞=%d "
        "最良余裕=%.2fm idx=%zu",
        og.bins, og.surv, og.lo, og.hi, og.blocked ? 1 : 0, og.best_clear, f.ei);
    }
  }
  const std::size_t n = f.n;
  if (n < 3) { emit(-steer_free, steer_free, false); return; }
  if (corridor_.lo.size() != n || corridor_.hi.size() != n) {
    emit(-steer_free, steer_free, false); return;
  }

  const double v = std::abs(f.ev);
  if (v < 0.5) {                  // 停止中は予測しない
    emit(-steer_free, steer_free, false); return;
  }

  // 評価対象は「今このサイクルで採用されている意図を、そこまでの制約で
  // 丸めた値」。applyLatDecision の直前に呼ばれるので確定値と一致する。
  const double want = c.latWant();
  const double lat0 = offset_;
  // 横移動のレートは applyOffsetRateLimit と同じ条件で選ぶ(予測と実挙動を
  // 合わせるため)。通常は offset_rate_(既定 1.2m/s)。
  const double rate = (start_merge_done_ || !race_started_)
                        ? offset_rate_ : start_offset_rate_;

  // 車両が物理的に取りうる最小の旋回半径(最大舵角から)。
  // これより小さい R を入れると内輪差が発散するので下限に使う。
  const double tan_max = std::tan(veh_max_steer_);
  const double r_veh = (tan_max > 1e-3) ? (veh_wheel_base_ / tan_max) : veh_wheel_base_;
  const double r_floor = std::max({veh_wheel_base_, r_veh, 0.5});
  // 半幅のうち corridor に控除されていないぶん(既定では 0)。
  const double base_extra = std::max(0.0, veh_half_width_ - wall_guard_corridor_half_);

  // 弧長で前方の点をたどる。cursor は 1 周を越えないので添字は壊れない。
  std::size_t cursor = 0;
  double cursor_dist = 0.0;
  auto advance = [&](double wanted) {
    while (cursor_dist < wanted && cursor + 1 < n) {
      const std::size_t a = (f.ei + cursor) % n;
      const std::size_t b = (a + 1) % n;
      double ds = f.s[b] - f.s[a];
      if (ds < 0.0) { ds += f.total; }   // 周回のまたぎ
      if (cursor_dist + ds > wanted) { break; }
      cursor_dist += ds;
      ++cursor;
    }
    return (f.ei + cursor) % n;
  };

  double lo_ok = -1e9;      // 予測区間すべてを満たす現在オフセットの下限
  double hi_ok = 1e9;       // 同 上限
  double run = 0.0;         // 連続して違反している長さ[m]
  double run_max = 0.0;
  double worst = 0.0;       // 最大の違反量[m]
  double r_min = 1e9;       // 予測区間の最小曲率半径[m]
  double r_at_worst = 1e9;
  std::size_t worst_i = f.ei;
  std::size_t room_i = f.ei;      // 最小余裕になった点(作動しないときのログ用)
  double room_r = 1e9;            // その点の曲率半径[m]
  bool any = false;

  const double dt = std::max(wall_guard_dt_, 0.01);
  const int steps =
    static_cast<int>(std::ceil(std::max(wall_guard_horizon_, 0.0) / dt));
  double s_ahead = 0.0;
  for (int k = 1; k <= steps; ++k) {
    const double t = k * dt;
    s_ahead += v * dt;
    // 1 周を越えるほど先は見ない(巻き戻って別の場所を評価しないため)
    if (f.total > 1.0 && s_ahead >= f.total * 0.9) { break; }
    const std::size_t i = advance(s_ahead);

    double R = 1e9;
    if (corridor_.radius.size() == n && std::isfinite(corridor_.radius[i])) {
      R = std::min(R, corridor_.radius[i]);
    }
    if (radius_min_.size() == n && std::isfinite(radius_min_[i])) {
      R = std::min(R, radius_min_[i]);
    }
    if (!std::isfinite(R) || R <= 0.0) { R = 1e9; }
    R = std::max(R, r_floor);
    r_min = std::min(r_min, R);

    const double offtrack    = veh_wheel_base_ * veh_wheel_base_ / (2.0 * R);
    const double front_swing = veh_front_overhang_ * (veh_wheel_base_ / R);
    const double rear_swing  = veh_rear_overhang_ * veh_rear_overhang_ / (2.0 * R);
    const double out_extra   = front_swing + rear_swing;

    // --- 曲がる向き。normalAt と同じく左が正。
    int turn = 0;   // +1 左曲がり / -1 右曲がり / 0 不明(直線)
    {
      const auto & pm = f.in.points[(i + n - 1) % n].pose.position;
      const auto & p0 = f.in.points[i].pose.position;
      const auto & pp = f.in.points[(i + 1) % n].pose.position;
      const double ax = p0.x - pm.x, ay = p0.y - pm.y;
      const double bx = pp.x - p0.x, by = pp.y - p0.y;
      const double cr = ax * by - ay * bx;
      const double sc = std::hypot(ax, ay) * std::hypot(bx, by);
      if (sc > 1e-9 && std::abs(cr) / sc > 1e-3) { turn = (cr > 0.0) ? 1 : -1; }
    }
    double ext_left = 0.0, ext_right = 0.0;
    if (turn > 0) {            // 左コーナー: 内側=左
      ext_left = offtrack; ext_right = out_extra;
    } else if (turn < 0) {     // 右コーナー: 内側=右
      ext_right = offtrack; ext_left = out_extra;
    } else {
      // 向きが取れない。保守側に倒して両側へ大きいほうを足す。
      const double m = std::max(offtrack, out_extra);
      ext_left = m; ext_right = m;
    }

    const double lo_need = corridor_.lo[i] + wall_guard_margin_ + base_extra + ext_right;
    const double hi_need = corridor_.hi[i] - wall_guard_margin_ - base_extra - ext_left;

    // その時刻に自車が居る横位置。現在の offset_ から want へレート制限で
    // 近づくと仮定して線形に補間する。
    const double reach = rate * t;
    const double lat_t = lat0 + std::clamp(want - lat0, -reach, reach);

    const double room = std::min(lat_t - lo_need, hi_need - lat_t);
    if (room < wall_guard_min_room_) {
      wall_guard_min_room_ = room;
      room_i = i;
      room_r = R;
    }
    lo_ok = std::max(lo_ok, lo_need);
    hi_ok = std::min(hi_ok, hi_need);

    const double viol = std::max(0.0, -room);
    if (viol > 1e-6) {
      run += v * dt;
      if (run > run_max) { run_max = run; }
      if (viol > worst) { worst = viol; worst_i = i; r_at_worst = R; }
    } else {
      run = 0.0;
    }
    any = true;
  }

  if (!any) { emit(-steer_free, steer_free, false); return; }

  if (run_max < wall_guard_run_) {
    const rclcpp::Time now_ok = f.now;
    if ((now_ok - last_wall_guard_log_).seconds() > 2.0) {
      last_wall_guard_log_ = now_ok;
      diagLog("壁予測", "壁予測 監視 idx=%zu 速度=%.1fkm/h 最小余裕=%.2fm R=%.1fm 舵角上書き=%d",
        room_i, v * 3.6, wall_guard_min_room_, room_r,
        steer_override_active_ ? 1 : 0);
    }
    emit(-steer_free, steer_free, false);
    return;
  }

  double cl = corridor_.lo[f.ei] + wall_guard_margin_;
  double ch = corridor_.hi[f.ei] - wall_guard_margin_;
  if (cl > ch) { const double m = 0.5 * (cl + ch); cl = ch = m; }
  lo_ok = std::clamp(lo_ok, cl, ch);
  hi_ok = std::clamp(hi_ok, cl, ch);

  // どの横位置でも違反が消えない = そのコーナーが今の速度に対してきつすぎる。
  const bool infeasible = (lo_ok > hi_ok);
  if (infeasible) { const double m = 0.5 * (lo_ok + hi_ok); lo_ok = hi_ok = m; }
  c.boundLat(lo_ok, hi_ok, "壁予測");

  // --- 出力 2: 速度の上限。横で逃げられないときだけ。
  double v_ok = -1.0;
  if (infeasible) {
    v_ok = std::sqrt(std::max(wall_guard_ay_max_, 0.0) * std::max(r_min, 0.1));
    c.requestCap(v_ok, "壁予測");
  }

  {
    const double t_h = std::max(wall_guard_horizon_, 0.05);
    const double denom = std::max(v * v * t_h * t_h, 1e-6);
    const double room_left  = std::max(0.0, hi_ok - offset_);
    const double room_right = std::max(0.0, offset_ - lo_ok);
    const double k_left  = 2.0 * room_left  / denom;
    const double k_right = 2.0 * room_right / denom;
    const double s_left  = std::atan(veh_wheel_base_ * k_left);
    const double s_right = std::atan(veh_wheel_base_ * k_right);
    emit(-std::min(s_right, steer_free), std::min(s_left, steer_free), true);
  }

  const rclcpp::Time now = f.now;
  if ((now - last_wall_guard_log_).seconds() > 2.0) {
    last_wall_guard_log_ = now;
    char capbuf[32];
    if (v_ok >= 0.0) { std::snprintf(capbuf, sizeof(capbuf), "%.1fkm/h", v_ok * 3.6); }
    else { std::snprintf(capbuf, sizeof(capbuf), "-"); }
    diagWarn("壁予測", "壁予測 作動 idx=%zu 速度=%.1fkm/h 予測違反=%.2fm(連続%.1fm) "
      "横 want=%.2f -> 制約[%.2f,%.2f] 速度上限=%s R=%.1fm 舵角上書き=%d",
      worst_i, v * 3.6, worst, run_max, want, lo_ok, hi_ok, capbuf,
      std::isfinite(r_at_worst) && r_at_worst < 1e8 ? r_at_worst : r_min,
      steer_override_active_ ? 1 : 0);
  }
}

// 決まった横目標へ、レート制限つきで現在のオフセットを近づける。
void V2XOvertaker::applyOffsetRateLimit(PlanCtx & c)
{
  // 交差が空な硬制約に「近い側」は存在しない。過去の offset_。
  if (!c.lat_feasible) {
    offset_ = c.target_offset;
    return;
  }
  // --- オフセットをレート制限つきで目標へ動かす
  const double dt = 0.05;
  const double rate = (start_merge_done_ || !race_started_)
                        ? offset_rate_ : start_offset_rate_;
  const double step = rate * dt;
  if (c.target_offset > offset_) {
    offset_ = std::min(c.target_offset, offset_ + step);
  } else {
    offset_ = std::max(c.target_offset, offset_ - step);
  }

}

// 横オフセットを乗せた軌道を作り、順位に応じた速度上限を掛けて publish する。

// デバッグ用の GUI へ、いま何をしているかを流す。
// 走りには一切影響しない。10Hz。
void V2XOvertaker::publishStatus(const Frame & f, const PlanCtx & c)
{
  if (!status_pub_) { return; }
  if ((f.now - last_status_pub_).seconds() < 0.1) { return; }
  last_status_pub_ = f.now;

  const char * slot_s = (start_slot_ == 1) ? "P1"
                      : (start_slot_ == 2) ? "P2"
                      : (start_slot_ == 3) ? "P3" : "?";
  std::string act;
  auto add = [&act](const char * t) { if (!act.empty()) { act += " / "; } act += t; };
  if (!race_started_) { add("PRE-RACE"); }
  if (attempt_active_) { add("OVERTAKING"); }
  if (c.stop_avoid_active) { add("AVOID STOPPED CAR"); }
  if (wedge_active_) { add("AVOID HEAD-ON"); }
  if (is_boosting_) { add("BOOST ACTIVE"); }
  else if (want_boost_) { add("BOOST REQUESTED"); }
  if (act.empty()) { add("CRUISING"); }

  char buf[1024];
  std::snprintf(buf, sizeof(buf),
    "slot=%s\nlap=%d\nrank=%d\nspeed=%.1f\ncap=%.1f\noffset=%.2f\ntarget=%.2f\n"
    "blocker=%s\ngap=%.1f\nact=%s\nattempt=%s\nboost=%d\nboosting=%d\nidx=%zu\n",
    slot_s, lap_, rank_, f.ev * 3.6,
    c.speed_cap >= 0.0 ? c.speed_cap * 3.6 : -1.0,
    offset_, c.target_offset,
    c.blocker.empty() ? "-" : c.blocker.c_str(), c.best_gap,
    act.c_str(), attempt_target_.empty() ? "-" : attempt_target_.c_str(),
    boost_remaining_, is_boosting_ ? 1 : 0, f.ei);

  std_msgs::msg::String m;
  m.data = buf;
  status_pub_->publish(m);
}

// 走行可能な横方向の範囲を周回全域で作る。
//
// 壁(コリドア)は静的なのでそのまま使う。他車は **自分がその地点へ着く時刻**
// まで等速で進めてから塞ぐ。「今その点に居るか」で塞ぐと、実際に自分が着く
// ころには相手はもう居ないので、無用に狭くなる。
//
// 塞ぐ側(相手の左を通るか右を通るか)は点ごとに決めず、相手 1 台につき
// 1 回だけ決める。点ごとに広いほうを選ぶとバンドが不連続になり、
// そこへクランプした軌道が折れる。
void V2XOvertaker::buildBand(const Frame & f)
{
  const size_t n = f.n;
  if (n == 0 || f.s.size() != n) { return; }
  std::vector<double> lo(n, -kBandAbs), hi(n, kBandAbs);
  if (corridor_.lo.size() == n) {
    const bool pass_mode = attempt_active_ || c_stop_avoid_active_ || straight_pass_now_;
    for (size_t i = 0; i < n; ++i) {
      double safety = safetyAt(i);
      if (pass_mode) { safety = std::min(safety, corridor_safety_pass_); }
      lo[i] = corridor_.lo[i] + safety;
      hi[i] = corridor_.hi[i] - safety;
      if (lo[i] > hi[i]) { const double m = 0.5 * (lo[i] + hi[i]); lo[i] = hi[i] = m; }
    }
  }

  std::vector<double> lo_ex(lo), hi_ex(hi);

  if (band_predict_) {
    const double half = f.total * 0.5;
    // 参照ラインの速度に対する実速度の比。
    // 「相手がその位置から参照ラインを走ったらどうなるか」を、
    // 相手ごとの能力(比)を保ったまま速度プロファイルで積分して出す。
    auto ratio = [&f](size_t idx, double v) {
      const double vr = std::max<double>(f.in.points[idx].longitudinal_velocity_mps, 1.0);
      return std::clamp(v / vr, 0.2, 1.3);
    };
    const double k_me = ratio(f.ei, std::max(f.ev, 0.5));

    const std::string & band_pass_target =
      (attempt_active_ && !attempt_target_.empty()) ? attempt_target_ : spot_target_;

    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid) { continue; }
      // 【2026-09-18】以前は停止車回避の対象車をバンドから除外していた(continue)。
      // 対象車の回避が横目標だけに任され、横目標が縛られたり側が変わったりすると
      // 軌道が対象車に重なるのを止める層が無かった(rviz の走行可能域が車体に掛かっていた)。
      // 除外せず、停止車回避が選んだ隙間の側を塞がない向きで入れる(下の is_stop_blocker)。
      const bool is_stop_blocker = c_stop_avoid_active_ && kv.first == c_blocker_;
      const size_t oi = nearest(f.in, o.x, o.y);
      double nx = 0.0, ny = 0.0;
      normalAt(f.in, oi, nx, ny);
      const auto & lp = f.in.points[oi].pose.position;
      const double olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;
      const double ov = std::hypot(o.vx, o.vy);
      const bool op_is_npc = (o.slot == npc_slot_);
      const double slot_cap_kmh =
          op_is_npc ? predict_speed_slow_ : predict_speed_fast_;
      // 順位のハンデ。先頭は 25km/h で頭打ちになる。
      const double rank_cap_kmh =
          (!cur_leader_.empty() && kv.first == cur_leader_)
            ? leader_speed_cap_ : rank2_speed_cap_;
      const double cap_op = std::min(slot_cap_kmh, rank_cap_kmh) / 3.6;
      const bool have_meas = (o.speed_cnt >= predict_prior_samples_);
      const double k_op =
          have_meas ? ratio(oi, std::max(ov, 0.2)) : ratio(oi, cap_op);
      const bool op_stopped = (ov < band_stop_speed_);

      const double a = olat - band_car_w_;
      const double b = olat + band_car_w_;
      int & side = band_side_[kv.first];
      const bool is_side_target =
          !side_target_.empty() && kv.first == side_target_;
      const double room_l = hi[oi] - b;
      const double room_r = a - lo[oi];
      const bool in_launch =
          !start_merge_done_ ||
          (race_start_time_ > 0.0 &&
           (f.now.seconds() - race_start_time_) < launch_p1_pass_sec_);
      double lead_l = 1e9, lead_r = 1e9;
      if (band_side_pred_ && !in_launch) {
        size_t oc2 = oi;
        double run2 = 0.0;
        const double k_lead = k_op;
        for (size_t step = 1; step < n && run2 < band_side_look_; ++step) {
          const size_t nx3 = (oc2 + 1) % n;
          double ds2 = f.s[nx3] - f.s[oc2];
          if (ds2 < 0.0) { ds2 += f.total; }
          run2 += ds2;
          oc2 = nx3;
          if (op_stopped) { break; }          // 止まっている相手は動かない
          const double latp = olat * std::exp(-run2 / std::max(band_lat_tau_, 1.0));
          lead_l = std::min(lead_l, hi[oc2] - (latp + band_car_w_));
          lead_r = std::min(lead_r, (latp - band_car_w_) - lo[oc2]);
          (void)k_lead;
        }
      }
      double use_l = (lead_l < 1e8) ? lead_l : room_l;
      double use_r = (lead_r < 1e8) ? lead_r : room_r;
      if (is_side_target) {
        side = (side_sign_ > 0.0) ? 1 : -1;
        band_side_pending_.erase(kv.first);
      } else if (side == 0) {
        side = (use_l >= use_r) ? 1 : -1;
      } else if (side > 0 && use_r > use_l + band_side_hyst_) {
        side = -1;
      } else if (side < 0 && use_l > use_r + band_side_hyst_) {
        side = 1;
      }

      const auto wit = win_map_.find(kv.first);
      win_found_ = (wit != win_map_.end()) && wit->second.found;
      if (win_found_) {
        win_side_ = wit->second.side; win_t_ = wit->second.t;
        win_d_ = wit->second.d; win_gap_ = wit->second.gap; win_dur_ = wit->second.dur;
      }
      if (win_found_) {
        const int use_side = is_side_target
                               ? ((side_sign_ > 0.0) ? 1 : -1)
                               : win_side_;
        if (use_side > 0) { use_l = std::max(use_l, win_gap_); use_r = -1.0; }
        else              { use_r = std::max(use_r, win_gap_); use_l = -1.0; }
        // 他車の窓はその車を避けるバンドにだけ使う。追越対象でない車の窓で
        // side_sign_ まで変えると、対象車に対する planOvertake の側と反転し、
        // 横へ出た直後に「側の不一致」でレースラインへ戻される。
        if ((f.now - last_window_log_).seconds() > 2.0) {
          last_window_log_ = f.now;
          RCLCPP_INFO(get_logger(),
            "追越窓 target=%s 側=%s まで%.1fm(%.1fs後) 継続%.1fs 最小隙間%.2fm",
            kv.first.c_str(), (win_side_ > 0) ? "左" : "右",
            win_d_, win_t_, win_dur_, win_gap_);
        }
      } else if (pass_window_enable_ && !c_blocker_.empty() &&
                 kv.first == c_blocker_) {
        if ((f.now - last_window_log_).seconds() > 5.0) {
          last_window_log_ = f.now;
          RCLCPP_INFO(get_logger(), "追越窓 target=%s 見つからず(%.0fm 先まで)",
                      kv.first.c_str(), band_horizon_);
        }
      }
      if (band_side_follow_ && !side_target_.empty() && kv.first == side_target_) {
        int want = (side_sign_ > 0.0) ? 1 : -1;
        const double room_want = (want > 0) ? room_l : room_r;
        const double room_other = (want > 0) ? room_r : room_l;
        const double tnow2 = f.now.seconds();
        bool may_fix = (tnow2 - side_fix_at_) >= side_fix_cool_;
        if (!attempt_active_ && room_want < 0.0 &&
            room_other >= band_car_w_ && may_fix) {
          if (side_fix_since_ < 0.0) { side_fix_since_ = tnow2; }
        } else {
          side_fix_since_ = -1.0;
        }
        const bool fix_ready =
            side_fix_since_ >= 0.0 && (tnow2 - side_fix_since_) >= band_side_hold_;
        // レーンで右に決めている間はここでも倒さない。
        if (!attempt_active_ && room_want < 0.0 && !(ot_lane_side_ && ot_lane_side_sticky_ && want < 0) &&
            room_other >= band_car_w_ && may_fix && fix_ready) {
          want = -want;
          side_sign_ = (want > 0) ? 1.0 : -1.0;
          side_fits_ = true;
          side_fix_at_ = tnow2;
          side_fix_since_ = -1.0;
          if ((f.now - last_side_fix_log_).seconds() > 2.0) {
            last_side_fix_log_ = f.now;
            RCLCPP_INFO(get_logger(),
              "追越側を修正 target=%s 余地の無い側を選んでいた -> %s へ "
              "(空き 左%.2f 右%.2f)",
              kv.first.c_str(), (want > 0) ? "左" : "右", room_l, room_r);
          }
        }
        if (side != want) {
          auto & pend = band_side_pending_[kv.first];
          if (pend.first != want) { pend = {want, f.now.seconds()}; }
          if (f.now.seconds() - pend.second < band_side_hold_) {
            // まだ保持時間に達していない。今の側を維持する。
            want = side;
          }
        } else {
          band_side_pending_.erase(kv.first);
        }
        if (side != want) {
          ++band_side_conflict_;
          if ((f.now - last_side_conflict_log_).seconds() > 2.0) {
            last_side_conflict_log_ = f.now;
            RCLCPP_INFO(get_logger(),
              "側の不一致を解消 target=%s 追越側=%s バンド側=%s -> 追越側に合わせる "
              "(空き 左%.2f 右%.2f 累計%d回)",
              kv.first.c_str(), (want > 0) ? "左" : "右",
              (side > 0) ? "左" : "右", room_l, room_r, band_side_conflict_);
          }
        }
        side = want;
      }

      // 停止車回避の対象車は、停止車回避が選んだ隙間(c_stop_avoid_lo_/hi_)の側を空ける。
      // バンドと停止車回避で通る側が食い違うと、横目標がバンドで押し戻されるため。
      if (is_stop_blocker && c_stop_avoid_pass_ && c_stop_avoid_hi_ > c_stop_avoid_lo_) {
        const double gap_c = 0.5 * (c_stop_avoid_lo_ + c_stop_avoid_hi_);
        side = (gap_c >= olat) ? 1 : -1;
      }

      // 追い越し窓の探索状態を、この相手についてこれから作り直す。
      win_found_ = false; win_side_run_ = 0;
      win_run_start_t_ = win_run_end_t_ = win_run_start_d_ = 0.0;
      win_run_min_gap_ = 0.0;
      {
        double g0 = f.s[oi] - f.s[f.ei];
        if (g0 >  f.total * 0.5) { g0 -= f.total; }
        if (g0 < -f.total * 0.5) { g0 += f.total; }
        win_gap0_ = g0;
      }

      auto & pred = band_pred_[kv.first];
      pred.clear();

      size_t oc = oi;      // 相手の添字
      double run_o = 0.0;  // 相手が進んだ距離[m]
      double t_op = 0.0;   // 相手がそこへ着く時刻[s]
      double t_me = 0.0;   // 自車がそこへ着く時刻[s]
      double run = 0.0;    // 自車からの前方距離[m]
      size_t guard = 0;
      for (size_t step = 1; step < n; ++step) {
        const size_t i  = (f.ei + step) % n;
        const size_t ip = (f.ei + step - 1) % n;
        double ds = f.s[i] - f.s[ip];
        if (ds < 0.0) { ds += f.total; }
        run += ds;
        if (run > band_horizon_) { break; }
        // 自車がそこへ着く時刻。**自分の到達可能速度**で積分する。
        // 順位のハンデ(1位25km/h / 2位以下36km/h)を超えては走れないので上限を掛ける。
        // これを入れないと、1位のとき「実際には出せない速度」で到達時刻を見積もり、
        // 追い越し窓の時刻が早すぎる方向にずれる。
        const double my_cap =
            ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
        const double vme = std::max<double>(
            std::min(k_me * f.in.points[ip].longitudinal_velocity_mps, my_cap), 0.5);
        t_me += ds / vme;
        // 相手を「自車がここへ着く時刻」まで進める
        while (!op_stopped && t_op < t_me && guard++ < 4 * n) {
          const size_t nx2 = (oc + 1) % n;
          double dso = f.s[nx2] - f.s[oc];
          if (dso < 0.0) { dso += f.total; }
          // 仮定した MPC の最高速度と順位ハンデを超えないようにする。
          double vop = std::min(k_op * f.in.points[oc].longitudinal_velocity_mps, cap_op);
          if (predict_lane_speed_) {
            const int obin = static_cast<int>(oc * OtherState::kLatBins / n);
            const double learned_v = o.laneSpd(obin);
            if (learned_v > 0.0) { vop = std::min(learned_v, cap_op); }
          }
          vop = std::max(vop, 0.3);
          t_op += dso / vop;
          run_o += dso;
          oc = nx2;
        }
        const int obin_lat = static_cast<int>(oc * OtherState::kLatBins / n);
        const double learned_lat = o.laneLat(obin_lat);
        const double lat_p = (learned_lat < 1e8)
          ? learned_lat
          : olat * std::exp(-run_o / std::max(band_lat_tau_, 1.0));
        {
          double pnx = 0.0, pny = 0.0;
          normalAt(f.in, oc, pnx, pny);
          pred.emplace_back(f.in.points[oc].pose.position.x + lat_p * pnx,
                            f.in.points[oc].pose.position.y + lat_p * pny);
        }
        const double lat_half = band_car_w_ + band_lat_grow_ * t_me;
        const double long_half = band_long_ + band_long_grow_ * t_me;
        if (pass_window_enable_) {
          const double gap_l = hi[i] - (lat_p + band_car_w_);
          const double gap_r = (lat_p - band_car_w_) - lo[i];
          const double best_gap = std::max(gap_l, gap_r);
          const int    gap_side = (gap_l >= gap_r) ? 1 : -1;
          // 追いついているか: 自車の進んだ距離が、相手の進んだ距離 + 初期車間 を超える
          const double caught_up = run - (run_o + win_gap0_);
          const bool wide = best_gap >= pass_window_gap_ && caught_up >= 0.0;
          if (wide && win_side_run_ == gap_side) {
            win_run_end_t_ = t_me;                       // 同じ側で継続中
            win_run_min_gap_ = std::min(win_run_min_gap_, best_gap);
          } else if (wide) {
            win_side_run_ = gap_side;                    // 側が変わった/始まった
            win_run_start_t_ = t_me; win_run_end_t_ = t_me;
            win_run_start_d_ = run;  win_run_min_gap_ = best_gap;
          } else {
            win_side_run_ = 0;                           // 途切れた
          }
          if (win_side_run_ != 0 && !win_found_ &&
              (win_run_end_t_ - win_run_start_t_) >= pass_window_sec_) {
            win_found_ = true;
            win_side_ = win_side_run_;
            win_t_ = win_run_start_t_;
            win_d_ = win_run_start_d_;
            win_gap_ = win_run_min_gap_;
            win_dur_ = win_run_end_t_ - win_run_start_t_;
          }
        }
        const double ap = lat_p - lat_half;
        const double bp = lat_p + lat_half;
        double d = f.s[oc] - f.s[i];
        if (d >  half) { d -= f.total; }
        if (d < -half) { d += f.total; }
        if (std::abs(d) > long_half) { continue; }
        if (side > 0) { lo[i] = std::max(lo[i], bp); }
        else          { hi[i] = std::min(hi[i], ap); }
        if (lo[i] > hi[i]) { const double m = 0.5 * (lo[i] + hi[i]); lo[i] = hi[i] = m; }
        if (kv.first != band_pass_target) {
          if (side > 0) { lo_ex[i] = std::max(lo_ex[i], bp); }
          else          { hi_ex[i] = std::min(hi_ex[i], ap); }
          if (lo_ex[i] > hi_ex[i]) {
            const double m = 0.5 * (lo_ex[i] + hi_ex[i]); lo_ex[i] = hi_ex[i] = m;
          }
        }
      }
      // この相手について求めた窓を保存する(次の周期で側の決定に使う)
      auto & w = win_map_[kv.first];
      w.found = win_found_; w.side = win_side_; w.t = win_t_;
      w.d = win_d_; w.gap = win_gap_; w.dur = win_dur_;
    }
  }

  if (predict_check_sec_ > 0.0) {
    const double tnow = f.now.seconds();
    while (!pred_checks_.empty() && pred_checks_.front().due <= tnow) {
      const auto pc = pred_checks_.front();
      pred_checks_.pop_front();
      auto it = others_.find(pc.id);
      if (it == others_.end() || !it->second.valid) { continue; }
      const double err = std::hypot(it->second.x - pc.x, it->second.y - pc.y);
      pred_err_sum_ += err; pred_err_cnt_++;
      if (err > pred_err_max_) { pred_err_max_ = err; }
      if ((f.now - last_pred_log_).seconds() > 5.0) {
        last_pred_log_ = f.now;
        RCLCPP_INFO(get_logger(),
          "予測誤差 %s %.1fs先 今回%.2fm 平均%.2fm 最大%.2fm (n=%d)",
          pc.id.c_str(), predict_check_sec_, err,
          pred_err_sum_ / std::max(1, pred_err_cnt_), pred_err_max_, pred_err_cnt_);
      }
    }
    // 新しい予測を積む。相手ごとに predict_check_sec 秒に1回だけ。
    for (const auto & kv : band_pred_) {
      if (kv.second.empty()) { continue; }
      double & last = pred_push_at_[kv.first];
      if (tnow - last < predict_check_sec_) { continue; }
      last = tnow;
      auto io = others_.find(kv.first);
      if (io == others_.end() || !io->second.valid) { continue; }
      // 相手の速度で predict_check_sec 秒ぶん進んだ先を予測経路から拾う
      const double ov2 = std::hypot(io->second.vx, io->second.vy);
      const double want_d = ov2 * predict_check_sec_;
      double acc = 0.0;
      auto pt = kv.second.front();
      for (size_t k = 1; k < kv.second.size(); ++k) {
        acc += std::hypot(kv.second[k].first - kv.second[k - 1].first,
                          kv.second[k].second - kv.second[k - 1].second);
        pt = kv.second[k];
        if (acc >= want_d) { break; }
      }
      pred_checks_.push_back({kv.first, tnow + predict_check_sec_, pt.first, pt.second});
      if (pred_checks_.size() > 64) { pred_checks_.pop_front(); }
    }
  }

  // 時間方向に平滑化する。相手の位置は 20Hz で細かく揺れるので、
  // そのままバンドにするとクランプ先が毎周期変わり、経路が震える。
  if (band_lo_.size() != n || band_hi_.size() != n) {
    band_lo_ = lo; band_hi_ = hi;
  } else {
    const double al = std::clamp(band_smooth_, 0.05, 1.0);
    for (size_t i = 0; i < n; ++i) {
      band_lo_[i] += (lo[i] - band_lo_[i]) * al;
      band_hi_[i] += (hi[i] - band_hi_[i]) * al;
    }
  }
  if (band_lo_ex_.size() != n || band_hi_ex_.size() != n) {
    band_lo_ex_ = lo_ex; band_hi_ex_ = hi_ex;
  } else {
    const double al = std::clamp(band_smooth_, 0.05, 1.0);
    for (size_t i = 0; i < n; ++i) {
      band_lo_ex_[i] += (lo_ex[i] - band_lo_ex_[i]) * al;
      band_hi_ex_[i] += (hi_ex[i] - band_hi_ex_[i]) * al;
    }
  }
}

// rviz 用。バンドの左右の縁を線で出す。
// 「先を見越して何を避けようとしているか」が一目で分かるようにするのが目的。
void V2XOvertaker::publishBand(const Frame & f)
{
  if (!band_pub_ || band_lo_.size() != f.n) { return; }
  if ((f.now - last_band_pub_).seconds() < 0.05) { return; }
  last_band_pub_ = f.now;

  visualization_msgs::msg::MarkerArray arr;
  for (int side = 0; side < 2; ++side) {
    visualization_msgs::msg::Marker m;
    m.header = f.in.header;
    m.ns = "drivable_band";
    m.id = side;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.10;
    m.color.a = 0.9f;
    m.color.r = (side == 0) ? 0.1f : 1.0f;
    m.color.g = (side == 0) ? 0.9f : 0.6f;
    m.color.b = 0.1f;
    m.pose.orientation.w = 1.0;
    const double half = f.total * 0.5;
    for (size_t i = 0; i < f.n; ++i) {
      double ahead = f.s[i] - f.s[f.ei];
      if (ahead >  half) { ahead -= f.total; }
      if (ahead < -half) { ahead += f.total; }
      if (ahead < -2.0 || ahead > band_horizon_) { continue; }
      double nx = 0.0, ny = 0.0;
      normalAt(f.in, i, nx, ny);
      const double off = (side == 0) ? band_lo_[i] : band_hi_[i];
      geometry_msgs::msg::Point p;
      p.x = f.in.points[i].pose.position.x + off * nx;
      p.y = f.in.points[i].pose.position.y + off * ny;
      p.z = f.in.points[i].pose.position.z;
      m.points.push_back(p);
    }
    if (m.points.size() >= 2) { arr.markers.push_back(m); }
  }
  // 相手ごとの予測経路。「MPC で走ったらどこを通るか」をそのまま出す。
  int id = 10;
  for (const auto & kv : band_pred_) {
    if (kv.second.size() < 2) { continue; }
    visualization_msgs::msg::Marker m;
    m.header = f.in.header;
    m.ns = "predicted_opponents";
    m.id = id++;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.scale.x = 0.18;
    m.color.a = 0.95f;
    m.color.r = 1.0f;
    m.color.g = 0.2f;
    m.color.b = 0.9f;
    m.pose.orientation.w = 1.0;
    for (const auto & p : kv.second) {
      geometry_msgs::msg::Point q;
      q.x = p.first; q.y = p.second; q.z = 0.2;
      m.points.push_back(q);
    }
    arr.markers.push_back(m);
  }
  if (!arr.markers.empty()) { band_pub_->publish(arr); }
}

void V2XOvertaker::publishMeasure(const Frame & f, PlanCtx & c)
{
  if (!measure_pub_ || f.n < 3) { return; }
  if ((f.now - last_measure_pub_).seconds() < 0.2) { return; }   // 5Hz
  last_measure_pub_ = f.now;
  const Trajectory & in = f.in;
  const size_t n = f.n;
  visualization_msgs::msg::MarkerArray arr;
  {
    visualization_msgs::msg::Marker clear;
    clear.header = in.header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
  }
  int id = 0;
  const auto make = [&](const char * ns, int type, double r, double g, double b,
                        double scale) {
    visualization_msgs::msg::Marker m;
    m.header = in.header;
    m.ns = ns;
    m.id = id++;
    m.type = type;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = scale;
    m.scale.y = scale;
    m.scale.z = scale;
    m.color.a = 0.95f;
    m.color.r = static_cast<float>(r);
    m.color.g = static_cast<float>(g);
    m.color.b = static_cast<float>(b);
    return m;
  };
  const auto pt = [](double x, double y, double z) {
    geometry_msgs::msg::Point p; p.x = x; p.y = y; p.z = z; return p;
  };
  // 参照ラインに沿って、自車から from..to [m] 前方を横 lat [m] ずらした線
  const auto along = [&](visualization_msgs::msg::Marker & m, double from, double to,
                         double lat, double z) {
    double run = 0.0;
    for (size_t k = 0; k < n && run <= to; ++k) {
      const size_t i = (f.ei + k) % n;
      if (k > 0) {
        double ds = f.s[i] - f.s[(f.ei + k - 1) % n];
        if (ds < 0.0) { ds += f.total; }
        run += ds;
      }
      if (run < from) { continue; }
      double nx, ny;
      normalAt(in, i, nx, ny);
      const auto & p = in.points[i].pose.position;
      m.points.push_back(pt(p.x + lat * nx, p.y + lat * ny, z));
    }
  };
  const auto text = [&](double x, double y, const std::string & s, double r, double g, double b) {
    auto m = make("labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING, r, g, b, 0.45);
    m.pose.position = pt(x, y, 1.6);
    m.text = s;
    arr.markers.push_back(m);
  };
  char buf[200];

  // --- 相手ごとの当たり判定
  //   band_body (水色) : バンドが相手を塞ぐ範囲 横±band_car_w 前後±band_long(予測の広がり前の値)
  //   stopped_body(赤) : 停止車回避が使う占有 横±(occupiedHalfWidth + 停止車の余裕)
  for (const auto & kv : others_) {
    const OtherState & o = kv.second;
    if (!o.valid || (f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
    const size_t oi = nearest(in, o.x, o.y);
    double nx, ny;
    normalAt(in, oi, nx, ny);
    const double tx = -ny, ty = nx;   // 進行方向
    const double sp = std::hypot(o.vx, o.vy);
    const bool stopped = sp < band_stop_speed_;
    const auto rect = [&](const char * ns, double lat_half, double long_half,
                          double r, double g, double b) {
      auto m = make(ns, visualization_msgs::msg::Marker::LINE_STRIP, r, g, b, 0.06);
      const double cs[5][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}, {-1, -1}};
      for (const auto & q : cs) {
        const double dl = q[0] * long_half, dw = q[1] * lat_half;
        m.points.push_back(pt(o.x + dl * tx + dw * nx, o.y + dl * ty + dw * ny, 0.3));
      }
      arr.markers.push_back(m);
    };
    rect("band_body", band_car_w_, band_long_, 0.2, 0.8, 1.0);
    if (stopped) {
      rect("stopped_body", occupiedHalfWidth(oi, false) + stoppedPad(), 1.0, 1.0, 0.2, 0.2);
    }
    std::snprintf(buf, sizeof(buf), "%s %.1fkm/h%s", kv.first.c_str(), sp * 3.6,
                  stopped ? " STOP" : "");
    text(o.x, o.y, buf, 1.0, 1.0, 1.0);
  }

  // --- 停止車回避が選んだ隙間(緑): 対象車の位置で横 [lo, hi]
  if (c.stop_avoid_active && c.stop_avoid_have_gap && !c.blocker.empty()) {
    const auto it = others_.find(c.blocker);
    if (it != others_.end() && it->second.valid) {
      const size_t oi = nearest(in, it->second.x, it->second.y);
      double nx, ny;
      normalAt(in, oi, nx, ny);
      const auto & p = in.points[oi].pose.position;
      auto m = make("stop_avoid_gap", visualization_msgs::msg::Marker::LINE_STRIP, 0.1, 1.0, 0.1, 0.18);
      m.points.push_back(pt(p.x + c.stop_avoid_lo * nx, p.y + c.stop_avoid_lo * ny, 0.2));
      m.points.push_back(pt(p.x + c.stop_avoid_hi * nx, p.y + c.stop_avoid_hi * ny, 0.2));
      arr.markers.push_back(m);
    }
  }

  // --- 追突防止(橙): 進路上とみなす帯 と 近距離の円
  if (rear_dbg_.ran) {
    const double lo = std::min(rear_dbg_.my_now, rear_dbg_.my_want) - rear_dbg_.sep_th;
    const double hi = std::max(rear_dbg_.my_now, rear_dbg_.my_want) + rear_dbg_.sep_th;
    auto ml = make("rear_end_inpath(追突防止の進路帯)", visualization_msgs::msg::Marker::LINE_STRIP, 1.0, 0.55, 0.0, 0.07);
    along(ml, 0.0, rear_end_range_, lo, 0.15);
    arr.markers.push_back(ml);
    auto mh = make("rear_end_inpath(追突防止の進路帯)", visualization_msgs::msg::Marker::LINE_STRIP, 1.0, 0.55, 0.0, 0.07);
    along(mh, 0.0, rear_end_range_, hi, 0.15);
    arr.markers.push_back(mh);
    if (rear_end_near_ > 0.0) {
      auto mc = make("rear_end_near_circle(進路外でも見る円)", visualization_msgs::msg::Marker::LINE_STRIP, 1.0, 0.55, 0.0, 0.05);
      for (int k = 0; k <= 36; ++k) {
        const double a = k * M_PI / 18.0;
        mc.points.push_back(pt(f.ex + rear_end_near_ * std::cos(a),
                               f.ey + rear_end_near_ * std::sin(a), 0.15));
      }
      arr.markers.push_back(mc);
    }
    if (!rear_dbg_.name.empty()) {
      const auto it = others_.find(rear_dbg_.name);
      if (it != others_.end()) {
        if (rear_dbg_.cap >= 0.0) {
          std::snprintf(buf, sizeof(buf), "REAR cap %.1fkm/h gap %.1fm sep %.2f/%.2f",
                        rear_dbg_.cap * 3.6, rear_dbg_.gap, rear_dbg_.sep, rear_dbg_.sep_th);
        } else {
          std::snprintf(buf, sizeof(buf), "REAR released gap %.1fm sep %.2f/%.2f",
                        rear_dbg_.gap, rear_dbg_.sep, rear_dbg_.sep_th);
        }
        auto m = make("labels", visualization_msgs::msg::Marker::TEXT_VIEW_FACING, 1.0, 0.55, 0.0, 0.45);
        m.pose.position = pt(it->second.x, it->second.y, 2.3);
        m.text = buf;
        arr.markers.push_back(m);
      }
    }
  }

  // --- 横目標(白)と、横目標に掛かった許容範囲(紫)。自車から window_full_ 前方まで
  {
    const double want = c.latWant();
    auto mw = make("lat_target", visualization_msgs::msg::Marker::LINE_STRIP, 1.0, 1.0, 1.0, 0.05);
    along(mw, 0.0, window_full_, want, 0.25);
    arr.markers.push_back(mw);
    // 制限が実際に掛かっている側だけ描く(以前は無制限のとき ±6m の線を描いており、
    // 壁の中に線が出て「走行可能域が壁に埋まっている」ように見えていた)。
    const double lo = c.lat_lo, hi = c.lat_hi;
    if (c.lat_lo > -1e8) {
      auto m = make("lat_bounds", visualization_msgs::msg::Marker::LINE_STRIP, 0.8, 0.2, 1.0, 0.05);
      along(m, 0.0, window_full_, lo, 0.25);
      arr.markers.push_back(m);
    }
    if (c.lat_hi < 1e8) {
      auto m = make("lat_bounds", visualization_msgs::msg::Marker::LINE_STRIP, 0.8, 0.2, 1.0, 0.05);
      along(m, 0.0, window_full_, hi, 0.25);
      arr.markers.push_back(m);
    }
  }
  // --- 今この周期に軌道と速度へ効いている層の一覧(自車の左上に表示)
  {
    // rviz の文字は日本語を表示できないことがあるので、主要な理由は英字に置き換える。
    static const std::pair<const char *, const char *> kNames[] = {
      {"基準", "BASE"}, {"壁回避", "WALL_AVOID"}, {"停止車回避(最大制動)", "STOPPED_EMG_BRAKE"},
      {"停止車回避(横到達)", "STOPPED_LAT_REACH"}, {"停止車回避", "STOPPED_AVOID"},
      {"壁予測", "WALL_PREDICT"}, {"並走中は安全境界を越えない", "ALONGSIDE_BOUND"},
      {"停止車の側へ寄らない", "STOPPED_SIDE_BOUND"}, {"追突防止(またがない)", "REAR_NO_STRADDLE"},
      {"追突防止(先の帯)", "REAR_AHEAD_BAND"}, {"追突防止", "REAR_END"}, {"追越レーンへ入る", "OT_LANE_ENTER"},
      {"追越レーン(低速で入らない)", "OT_LANE_SLOW_BLOCK"}, {"追越", "OVERTAKE"}, {"衝突回避", "COLLISION"},
      {"姿勢復元", "POSE_RESTORE"}, {"壁に押されて後退", "PUSHED_YIELD"}, {"追従", "FOLLOW"},
      {"車体の実位置を戻す", "BODY_RETURN"}, {"車体の実位置", "BODY_POS"}, {"車間を開ける", "OPEN_GAP"},
      {"事前寄せ", "PRE_POSITION"}, {"姿勢ぶんの余裕", "YAW_MARGIN"}, {"近接車反発", "REPULSE"},
      {"グリッド保持", "GRID_HOLD"}, {"スタートレーン保持", "START_HOLD"}, {"なし", "none"},
      {"前方は空いている", "FRONT_CLEAR"},
    };
    const auto en = [](const char * w) -> std::string {
      if (!w) { return "-"; }
      for (const auto & kv : kNames) { if (std::strcmp(w, kv.first) == 0) { return kv.second; } }
      return w;
    };
    std::string ascii, jp;
    char line[256];
    std::snprintf(line, sizeof(line), "[LAT] want %.2f <- %s | bound [%.2f %s, %.2f %s]\n",
                  c.latWant(), en(c.lat_intent.why).c_str(),
                  std::max(c.lat_lo, -9.99), en(c.lat_lo_why).c_str(),
                  std::min(c.lat_hi, 9.99), en(c.lat_hi_why).c_str());
    ascii += line;
    std::snprintf(line, sizeof(line), "横目標 %.2f <- %s | 範囲 [%.2f %s, %.2f %s]\n",
                  c.latWant(), c.lat_intent.why ? c.lat_intent.why : "-",
                  std::max(c.lat_lo, -9.99), c.lat_lo_why ? c.lat_lo_why : "-",
                  std::min(c.lat_hi, 9.99), c.lat_hi_why ? c.lat_hi_why : "-");
    jp += line;
    // 要求(R=採用 r=優先度で却下)と制約(B=狭めた b=効かず)
    std::string req_a = "  lat:", req_j = "  横の要求:";
    for (int k = 0; k < c.lat_trace_n; ++k) {
      const auto & t = c.lat_trace[k];
      if (t.kind == 'R' || t.kind == 'r') {
        std::snprintf(line, sizeof(line), " %c:%s=%.2f", t.kind, en(t.why).c_str(), t.a);
        req_a += line;
        std::snprintf(line, sizeof(line), " %c:%s=%.2f", t.kind, t.why, t.a);
        req_j += line;
      } else if (t.kind == 'B') {
        std::snprintf(line, sizeof(line), " B:%s[%.2f,%.2f]", en(t.why).c_str(),
                      std::max(t.a, -9.99), std::min(t.b, 9.99));
        req_a += line;
        std::snprintf(line, sizeof(line), " B:%s[%.2f,%.2f]", t.why,
                      std::max(t.a, -9.99), std::min(t.b, 9.99));
        req_j += line;
      }
    }
    ascii += req_a + "\n";
    jp += req_j + "\n";
    std::snprintf(line, sizeof(line), "[SPEED] cap %.1fkm/h <- %s%s | ego %.1fkm/h\n",
                  c.speed_cap >= 0.0 ? c.speed_cap * 3.6 : -1.0, en(c.cap_why).c_str(),
                  c.emergency_brake ? " EMERGENCY" : "", f.ev * 3.6);
    ascii += line;
    std::snprintf(line, sizeof(line), "速度上限 %.1fkm/h <- %s%s | 自車 %.1fkm/h\n",
                  c.speed_cap >= 0.0 ? c.speed_cap * 3.6 : -1.0, c.cap_why ? c.cap_why : "-",
                  c.emergency_brake ? " 最大制動" : "", f.ev * 3.6);
    jp += line;
    std::string caps_a = "  caps:";
    for (const auto & r : c.cap_reqs) {
      std::snprintf(line, sizeof(line), " %s=%.1f", en(r.why).c_str(), r.v * 3.6);
      caps_a += line;
    }
    ascii += caps_a + "\n";
    jp += "  速度の要求: " + c.cap_all + "\n";
    std::string st;
    if (attempt_active_) { st += " OVERTAKING(" + attempt_target_ + ")"; }
    if (c.stop_avoid_active) { st += " AVOID_STOPPED(" + c.blocker + ")"; }
    if (wedge_active_) { st += " AVOID_HEAD_ON"; }
    if (is_boosting_) { st += " BOOST"; }
    if (launch_motion_since_ < 0.0) { st += " WAIT_LAUNCH"; }
    if (st.empty()) { st = " CRUISING"; }
    ascii += "[STATE]" + st + "\n";
    jp += "[状態]" + st + "\n";
    // 層の一覧は rviz の左パネル(ten_debug_rviz_plugin)へ。地図の上には出さない。
    // rviz は日本語を文字化けさせるので英字版を出す(日本語はログに出ている)。
    (void)jp;
    if (layers_pub_) {
      std_msgs::msg::String sm;
      sm.data = ascii;
      layers_pub_->publish(sm);
    }
  }
  measure_pub_->publish(arr);
}

void V2XOvertaker::publishTrajectory(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const size_t n = f.n;
  const double total = f.total;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // --- 軌道を作り直す
  Trajectory out = in;
  const bool have_corr = corridor_.lo.size() == n;

  // 停止車回避が働いているかを buildBand へ伝える(壁の余裕を詰める条件に使う)。
  c_stop_avoid_active_ = c.stop_avoid_active;
  c_stop_avoid_pass_ = c.stop_avoid_active && c.stop_avoid_have_gap;
  c_stop_avoid_lo_ = c.stop_avoid_lo;
  c_stop_avoid_hi_ = c.stop_avoid_hi;
  c_stop_avoid_target_ = c.blocker;
  c_blocker_ = c.blocker;
  // 全域バンドを作る(壁 + 他車の予測位置)。rviz にも出す。
  if (band_enable_) { buildBand(f); publishBand(f); }
  const bool use_band = band_enable_ && band_clamp_ && band_lo_.size() == n;

  // 先にオフセットの配列を作る。バンドへ収めたあと、
  // 横方向の傾きに上限を掛けてから軌道へ適用する
  // (点ごとにクランプするだけだと軌道が折れて、pure pursuit が跳ねる)。
  std::vector<double> offs(n, 0.0);
  for (size_t i = 0; i < n; ++i) {
    // 自車からの前後距離。軌道全体を一律にずらすと、避ける必要のない区間でも
    // ラインが壁側に寄ってしまうので、近傍だけに掛けて遠方は素のラインに戻す。
    double gap = s[i] - s[ei];
    if (gap > total * 0.5) {
      gap -= total;
    } else if (gap < -total * 0.5) {
      gap += total;
    }
    double w;
    if (gap >= -window_back_ && gap <= window_full_) {
      w = 1.0;
    } else if (gap > window_full_ && gap < window_end_) {
      w = (window_end_ - gap) / (window_end_ - window_full_);
    } else if (gap < -window_back_ && gap > -window_end_ * 0.5) {
      w = (gap + window_end_ * 0.5) / (window_end_ * 0.5 - window_back_);
    } else {
      w = 0.0;
    }
    double off = offset_ * w;
    if (use_band) {
      // 診断: バンドが目標をどれだけ動かしたか。
      // 「予測が効いているのか」がログから一切分からなかったため入れた。
      const double before = off;
      off = std::clamp(off, band_lo_[i], band_hi_[i]);
      double ahead_d = s[i] - s[ei];
      if (ahead_d > total * 0.5) { ahead_d -= total; }
      if (ahead_d < -total * 0.5) { ahead_d += total; }
      if (ahead_d > 0.0 && ahead_d < band_horizon_) {
        const double moved = std::abs(off - before);
        if (moved > band_dbg_moved_) { band_dbg_moved_ = moved; band_dbg_at_ = ahead_d; }
      }
    } else if (have_corr) {
      off = std::clamp(off, corridor_.lo[i] + safetyAt(i),
                       corridor_.hi[i] - safetyAt(i));
    }
    offs[i] = off;
  }


  // 診断: バンドが素の目標をどれだけ押しのけたかを定期的に出す。
  if (use_band && (now - last_band_dbg_log_).seconds() > 3.0) {
    last_band_dbg_log_ = now;
    if (band_dbg_moved_ > 0.05) {
      RCLCPP_INFO(get_logger(), "バンドが経路を %.2fm 動かした(前方%.1fm)",
                  band_dbg_moved_, band_dbg_at_);
    }
    band_dbg_moved_ = 0.0;
  }

  // 傾きの上限。自車地点から前へ向かって伝播させる
  // (後ろは既に通り過ぎているので直さない)。
  if (use_band && band_slope_ > 0.0) {
    for (size_t k = 1; k < n; ++k) {
      const size_t i = (ei + k) % n;
      const size_t j = (ei + k - 1) % n;
      double ds = s[i] - s[j];
      if (ds < 0.0) { ds += total; }
      if (ds <= 0.0) { continue; }
      const double lim = band_slope_ * ds;
      double v = std::clamp(offs[i], offs[j] - lim, offs[j] + lim);
      // バンドの外へは出さない。出るしかないときはバンドを優先する。
      offs[i] = std::clamp(v, band_lo_[i], band_hi_[i]);
    }
  }

  if (path_rate_ > 0.0 && prev_offs_.size() == n) {
    const double dt = std::clamp((now - last_offs_time_).seconds(), 0.01, 0.2);
    const double lim = path_rate_ * dt;
    for (size_t i = 0; i < n; ++i) {
      offs[i] = std::clamp(offs[i], prev_offs_[i] - lim, prev_offs_[i] + lim);
    }
  }
  last_offs_time_ = now;

  {
    double jump = 0.0; std::size_t at = 0;
    if (prev_offs_.size() == n) {
      for (size_t k = 1; k < n; ++k) {
        const size_t i = (ei + k) % n;
        double ahead = s[i] - s[ei];
        if (ahead > total * 0.5) { ahead -= total; }
        if (ahead < -total * 0.5) { ahead += total; }
        if (ahead < 0.0 || ahead > 30.0) { continue; }
        const double dj = std::abs(offs[i] - prev_offs_[i]);
        if (dj > jump) { jump = dj; at = i; }
      }
    }
    prev_offs_ = offs;
    if (jump > path_jump_max_) { path_jump_max_ = jump; path_jump_at_ = at; }
    if ((now - last_jump_log_).seconds() > 3.0) {
      last_jump_log_ = now;
      if (path_jump_max_ > 0.10) {
        RCLCPP_INFO(get_logger(), "経路の跳び 最大%.2fm/周期 (idx%zu)",
                    path_jump_max_, path_jump_at_);
      }
      path_jump_max_ = 0.0;
    }
  }



  for (size_t i = 0; i < n; ++i) {
    double nx, ny;
    normalAt(in, i, nx, ny);
    out.points[i].pose.position.x += offs[i] * nx;
    out.points[i].pose.position.y += offs[i] * ny;
  }

  // --- 順位に応じた速度プロファイルの切替。
  if (rank_shape_enable_) {
    const double fade = leader_speed_cap_ / 3.6;   // 25 km/h 相当
    const double gain = (rank_ == 1) ? rank1_corner_gain_ : rank2_corner_gain_;
    const double cap  = (rank_ == 1) ? fade : (rank2_speed_cap_ / 3.6);
    if (std::abs(gain - 1.0) > 1e-3) {
      for (size_t i = 0; i < n; ++i) {
        double v = out.points[i].longitudinal_velocity_mps;
        if (v < cap) {
          v = std::min(v * gain, cap);
        }
        out.points[i].longitudinal_velocity_mps = static_cast<float>(v);
      }
    }
  }

  c.applyCapRequests();
  // ===================================================================。
  if (fwd_clear_floor_ && !c.emergency_brake &&
      c.speed_cap >= 0.0 && c.speed_cap < fwd_clear_floor_kmh_ / 3.6)
  {
    double clear = 0.0;
    const std::size_t n = f.n;
    const double half = geom_half_width_;
    for (std::size_t k = 0; k < n && clear < fwd_clear_need_m_ + 1.0; ++k) {
      const std::size_t i0 = (f.ei + k) % n, i1 = (f.ei + k + 1) % n;
      const double seg = std::hypot(
        f.in.points[i1].pose.position.x - f.in.points[i0].pose.position.x,
        f.in.points[i1].pose.position.y - f.in.points[i0].pose.position.y);
      // その地点でいまの横位置に車体が収まるか(コリドアの生の値で判定)
      if (corridor_.lo.size() == n && corridor_.hi.size() == n) {
        const double lo = corridor_.lo[i1], hi = corridor_.hi[i1];
        if (my_lat_for_target_ - half < lo || my_lat_for_target_ + half > hi) { break; }
      }
      clear += seg;
    }
    // 前方の相手までの距離でも切る
    double car_ahead = 1e9;
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || !o.prog_init || !my_prog_init_) { continue; }
      if ((f.now - o.stamp).seconds() > v2x_timeout_) { continue; }
      double d = o.prog - my_prog_;
      if (d < 0.0) { d += f.total; }
      if (d > 0.0 && d < car_ahead) { car_ahead = d; }
    }
    clear = std::min(clear, std::max(car_ahead - (geom_front_ + geom_rear_), 0.0));
    if (clear >= fwd_clear_need_m_) {
      const double floor_v = fwd_clear_floor_kmh_ / 3.6;
      if ((f.now - last_fwd_clear_log_).seconds() > 1.0) {
        last_fwd_clear_log_ = f.now;
        diagLog("前方は空いている",
          "前方は空いている idx=%zu 前方クリア%.1fm(要%.1f) 前の車まで%.1fm "
          "上限 %.1f -> %.1fkm/h (決め手だった層=%s 全要求[%s])",
          f.ei, clear, fwd_clear_need_m_,
          (car_ahead > 1e8 ? -1.0 : car_ahead),
          c.speed_cap * 3.6, floor_v * 3.6,
          c.cap_why ? c.cap_why : "-", c.cap_all.c_str());
      }
      c.speed_cap = floor_v;
      c.cap_why = "前方は空いている";
    }
  }
  if (ot_lane_enable_ && !ot_lane_zones_.empty() &&
      (f.now - last_runup_audit_log_).seconds() > 1.0)
  {
    const double d_ent = otLaneEntryDistance(f.ei, ot_lane_runup_look_);
    const bool in_lane = inOtLaneUse(f.ei);
    if (d_ent >= 0.0 || in_lane) {
      last_runup_audit_log_ = f.now;
      diagLog("助走監査",
        "助走監査 idx=%zu %s 入口まで%.1fm 自車%.1fkm/h 上限%.1fkm/h(%s) "
        "門%.0fkm/h | 助走[到達=%d 全開=%d 門助走=%d 目標%.1fkm/h "
        "加速開始%.1fm 基準%.1fm 車間%.1fm 要車間%.1fm 最終全開=%d]",
        f.ei, in_lane ? "レーン内" : "接近中", d_ent,
        std::abs(f.ev) * 3.6,
        (c.speed_cap >= 0.0) ? c.speed_cap * 3.6 : -1.0, c.cap_why,
        ot_lane_min_kmh_,
        dbg_runup_reached_ ? 1 : 0, dbg_runup_charge_ ? 1 : 0,
        dbg_gate_runup_ ? 1 : 0, dbg_runup_vtgt_ * 3.6,
        dbg_runup_accel_at_, dbg_runup_dref_, dbg_gap_, dbg_eff_safe_,
        dbg_charge_now_ ? 1 : 0);
    }
  }
  if (c.speed_cap >= 0.0 && c.speed_cap * 3.6 < 1.0 &&
      (this->now() - last_stall_log_).seconds() > 0.5)
  {
    last_stall_log_ = this->now();
    std::string reqs;
    for (const auto & r : c.cap_reqs) {
      reqs += std::string(r.why ? r.why : "?") + "=" +
              std::to_string(static_cast<int>(r.v * 3.6 + 0.5)) + " ";
    }
    std::string others;
    for (const auto & kv : others_) {
      const auto & o = kv.second;
      if (!o.valid) { continue; }
      const double age = (f.now - o.stamp).seconds();
      double dp = o.prog_init ? (o.prog - my_prog_) : 9999.0;
      others += kv.first + "(前後" + std::to_string(static_cast<int>(dp * 10) / 10.0).substr(0, 5) +
                "m 速" + std::to_string(static_cast<int>(std::hypot(o.vx, o.vy) * 3.6)) +
                "km/h 齢" + std::to_string(static_cast<int>(age * 100) / 100.0).substr(0, 4) + "s) ";
    }
    RCLCPP_WARN(get_logger(),
      "停止の内訳 上限%.2fkm/h 決め手=%s 自車%.1fkm/h 状態=%s 対象=%s idx=%zu "
      "経路速度=%.1fkm/h 要求[%s] 他車[%s]",
      c.speed_cap * 3.6, c.cap_why, std::abs(f.ev) * 3.6, ovStateName(),
      c.blocker.empty() ? "なし" : c.blocker.c_str(), f.ei,
      (f.ei < f.in.points.size())
        ? f.in.points[f.ei].longitudinal_velocity_mps * 3.6 : -1.0,
      reqs.c_str(), others.c_str());
  }
  if (c.speed_cap >= 0.0 && c.speed_cap * 3.6 < cap_slow_log_kmh_ &&
      (this->now() - last_slow_cap_log_).seconds() > 0.3)
  {
    last_slow_cap_log_ = this->now();
    const double ref_kmh2 =
      (f.ei < f.in.points.size())
        ? f.in.points[f.ei].longitudinal_velocity_mps * 3.6 : -1.0;
    std::string reqs;
    for (const auto & r : c.cap_reqs) {
      reqs += std::string(r.why ? r.why : "?") + "=" +
              std::to_string(static_cast<int>(r.v * 3.6 + 0.5)) + " ";
    }
    RCLCPP_WARN(get_logger(),
      "遅い原因 上限%.1fkm/h 決め手=%s 自車=%.1fkm/h 経路速度=%.1fkm/h "
      "状態=%s idx=%zu 要求[%s]",
      c.speed_cap * 3.6, c.cap_why, std::abs(f.ev) * 3.6, ref_kmh2,
      ovStateName(), f.ei, reqs.c_str());
  }
  if ((this->now() - last_cap_arbitration_log_).seconds() > 2.0) {
    last_cap_arbitration_log_ = this->now();
    double ov_sp = -1.0;
    if (!ov_target_.empty()) {
      auto it = others_.find(ov_target_);
      if (it != others_.end() && it->second.valid) {
        ov_sp = std::hypot(it->second.vx, it->second.vy) * 3.6;
      }
    }
    const double ref_kmh =
      (f.ei < f.in.points.size())
        ? f.in.points[f.ei].longitudinal_velocity_mps * 3.6 : -1.0;
    diagLog("速度上限", "速度上限 %.1fkm/h 決め手=%s 要求数=%zu 状態=%s 自車=%.1fkm/h "
                "相手=%.1fkm/h 経路速度=%.1fkm/h 横=%.2fm 実横=%.2fm idx=%zu",
                (c.speed_cap < 0.0) ? -1.0 : c.speed_cap * 3.6,
                c.cap_why, c.cap_reqs.size(), ovStateName(),
                std::abs(f.ev) * 3.6, ov_sp, ref_kmh, offset_,
                my_lat_for_target_, f.ei);
    // 勝者だけでなく**全要求**を残す。
    diagLog("速度上限の全要求", "速度上限の全要求 [%s] -> %.1fkm/h(%s)",
            c.cap_all.c_str(),
            (c.speed_cap >= 0.0) ? c.speed_cap * 3.6 : -1.0, c.cap_why);
  }

  if (cap_rate_limit_ && c.speed_cap >= 0.0 && !c.emergency_brake) {
    const double dt = 0.05;
    const double floor_now = std::max(my_speed_for_gap_, 0.0) - cap_decel_ * dt;
    if (last_cap_out_ >= 0.0) {
      c.speed_cap = std::max(c.speed_cap,
                             std::min(last_cap_out_, floor_now));
    } else {
      c.speed_cap = std::max(c.speed_cap, floor_now);
    }
  }
  last_cap_out_ = c.speed_cap;

  // 横変形後の軌道で pure_pursuit が選ぶ最近傍点は、素の軌道で求めた ei と。
  const size_t output_ei = nearest(out, f.ex, f.ey);
  const double raw_ei_before_cap = out.points[ei].longitudinal_velocity_mps;
  const double output_ei_before_cap = out.points[output_ei].longitudinal_velocity_mps;
  const size_t capped_points = v2x_overtaker::applyTrajectorySpeedCap(out, c.speed_cap);

  if (capped_points > 0) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 500,
      "速度上限配送 cap=%.1fkm/h emergency=%d 対象=%zu/%zu "
      "raw_idx=%zu 速度=%.1f->%.1fkm/h out_idx=%zu 速度=%.1f->%.1fkm/h 差=%zd",
      c.speed_cap * 3.6, c.emergency_brake ? 1 : 0, capped_points, n,
      ei, raw_ei_before_cap * 3.6,
      static_cast<double>(out.points[ei].longitudinal_velocity_mps) * 3.6,
      output_ei, output_ei_before_cap * 3.6,
      static_cast<double>(out.points[output_ei].longitudinal_velocity_mps) * 3.6,
      static_cast<std::ptrdiff_t>(output_ei) - static_cast<std::ptrdiff_t>(ei));
  }

  // 次の層(manageBoost)が使うので、自車地点の目標速度を残す。
  // 下位制御と同じく、横変形後の最近傍点を使う。
  c.ego_target_speed = out.points[output_ei].longitudinal_velocity_mps;
  last_speed_cap_ = c.speed_cap;

  out.header.stamp = this->now();
  pub_->publish(out);
  // 速度上限が確定した後に出す(publishMeasure は層の一覧も出すため)
  publishMeasure(f, c);
  {
    std_msgs::msg::Bool ov;
    // 追い越し試行中は pure_pursuit の目標点を近づけ、横オフセットへ
    // 素早く追従させる。
    ov.data = attempt_active_;
    overtaking_pub_->publish(ov);
  }
}

void V2XOvertaker::manageBoost(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // --- 残ったブーストの使い道 ---。
  bool start_push = false;
  if (start_boost_enable_ && boostLapOk() && !start_boost_used_ && start_rank_ >= 2 &&
      boost_used_ == 0 && lap_ < start_boost_laps_ &&
      run_dist_ < start_boost_dist_)
  {
    const double v_now_push = odom_->twist.twist.linear.x;
    if (v_now_push > 1.5 && c.best_gap > 5.0) {
      start_push = true;
    }
  }
  if (!want_boost_ && free_boost_enable_ && boostLapOk() &&
      boost_remaining_ > 0 && !is_boosting_ &&
      (start_merge_done_ || start_push) &&
      (boost_used_ == 0 || (now - last_boost_time_).seconds() > free_boost_gap_))
  {
    const double my_v = odom_->twist.twist.linear.x;
    // 到達できる速度は「その地点の目標」と「順位のハンデ」の小さい方。
    // 追い越し判定と同じ考え方にそろえる。
    const double want_v = c.ego_target_speed;
    double rank_cap = ((rank_ == 1) ? leader_speed_cap_ : rank2_speed_cap_) / 3.6;
    if (my_v > rank_cap) { rank_cap = 1e9; }
    const double v_reach = std::min<double>(want_v, rank_cap);
    const bool power_limited = my_v > free_boost_min_speed_ &&
                               v_reach - my_v > free_boost_headroom_;

    // (a) 後ろから詰められているか(このサイクルの頭で求めてある)
    const bool pressed = c.pressed_from_behind;
    const bool last_lap = lap_ >= race_laps_ - free_boost_laps_left_;
    (void)pressed; (void)start_push;
    const bool occasion = last_lap;
    // 前方が空いていること。詰まっていると加速してもすぐ緩めることになる。
    bool ahead_clear = true;
    {
      // 進行方向は経路の接線で取る(自車の姿勢より素直で、横滑りの影響も無い)
      const auto & pa = in.points[ei].pose.position;
      const auto & pb = in.points[(ei + 2) % n].pose.position;
      double fx = pb.x - pa.x, fy = pb.y - pa.y;
      const double fl = std::hypot(fx, fy);
      if (fl > 1e-9) { fx /= fl; fy /= fl; }
      for (const auto & kv : others_) {
        if (!kv.second.valid) { continue; }
        const double dx = kv.second.x - ex, dy = kv.second.y - ey;
        const double f = dx * fx + dy * fy;
        const double side = std::abs(-dx * fy + dy * fx);
        if (f > 0.0 && f < free_boost_clear_ahead_ && side < front_lane_half_ * 2.0) {
          ahead_clear = false;
          break;
        }
      }
    }
    // 先が直線であること。コーナーで撃っても曲がれずに膨らむだけ。
    // 直線判定は関数の先頭で求めたものを使う(コーナー出口で撃てる形)
    const bool straight = straight_ahead_;
    // 指定した区間では、直線判定を待たずに撃つ。
    // 終盤にメインストレートで仕掛けるとき、直線に入ってから撃つと
    // 加速の開始が遅れて直線を半分使ってしまう。コーナーの立ち上がり
    // (idx220付近)から加速を始めれば、直線に入った時点で伸びている。
    bool in_boost_zone = false;
    for (const auto & z : boost_zones_) {
      const bool inside = (z.first <= z.second)
                            ? (ei >= z.first && ei <= z.second)
                            : (ei >= z.first || ei <= z.second);
      if (inside) { in_boost_zone = true; break; }
    }
    const bool clear_ok = ahead_clear && c.blocker.empty();
    const bool place_ok = straight || in_boost_zone;
    const bool power_ok = power_limited || start_push;
    if (start_push && (this->now() - last_push_log_).seconds() > 1.0) {
      last_push_log_ = this->now();
      RCLCPP_INFO(get_logger(),
                  "スタートブースト判定: 加速余地%d(速度%.1f/到達%.1f) 前方空き%d "
                  "直線%d 加速区間%d 残数%d 発射中%d 合流%d",
                  power_limited ? 1 : 0, my_v, v_reach, ahead_clear ? 1 : 0,
                  straight ? 1 : 0, in_boost_zone ? 1 : 0,
                  boost_remaining_, is_boosting_ ? 1 : 0, start_merge_done_ ? 1 : 0);
    }
    if (occasion && power_ok && clear_ok && place_ok) {
      // 使用済みの印は「実際に発射した時点」で立てる。
      // ここで立てると次の周期に start_push が消え、arm した直後に
      // 要求が下りて 1.0 を送れないまま終わる(2周期かけて撃つため)。
      if (start_push) { start_boost_pending_ = true; }
      want_boost_ = true;
      RCLCPP_INFO(get_logger(),
                  "ブースト使用(%s%s) %d周目 残り%d 速度%.1f 到達%.1f m/s rank=%d",
                  start_push ? "スタート直後" :
                    (pressed ? "後方から詰められている" : "終盤"),
                  in_boost_zone ? "/加速区間" : "",
                  lap_ + 1, boost_remaining_, my_v, v_reach, rank_);
    }
  }

  const double dist_to_line   = f.total - f.s[ei];
  const int    laps_left      = std::max(0, race_laps_ - 1 - lap_);
  const double dist_to_finish = laps_left * f.total + dist_to_line;
  const bool   final_dash     = final_dash_enable_ &&
                                (dist_to_finish <= final_dash_dist_);
  const bool   final_chase    = final_dash_enable_ && rank_ >= 2 &&
                                lap_ >= std::max(0, race_laps_ - 2) &&
                                straight_ahead_ && !c.blocker.empty() &&
                                c.best_gap < final_dash_gap_;
  const bool   final_release  = final_dash || final_chase;

  // (1) 最終区間、または最終2周の直線追撃。ここで予備の1個を解放する。
  if (final_release && !want_boost_ && boost_remaining_ > 0 && !is_boosting_ &&
      !c.blocker.empty() && c.best_gap < final_dash_gap_ &&
      (rank_ >= 2 || final_dash_boost_when_leading_))
  {
    want_boost_ = true;
    RCLCPP_INFO(get_logger(),
                "ブースト使用(%s) 残り%.1fm target=%s 車間=%.1fm rank=%d 残%d",
                final_chase && !final_dash ? "終盤追撃" : "最終区間",
                dist_to_finish, c.blocker.c_str(), c.best_gap, rank_,
                boost_remaining_);
  }

  // (2) 温存と無駄撃ちの抑止。発射経路4つすべてがここを通る。
  if (final_dash_enable_ && want_boost_ && !final_release) {
    const char * hold = nullptr;
    if (boost_remaining_ <= boost_reserve_final_) {
      hold = "最終区間用に温存";
    } else if (leader_boost_block_ && rank_ == 1) {
      const double head = leader_speed_cap_ / 3.6 - f.ev;
      if (head < leader_boost_headroom_) { hold = "1位で加速余地なし"; }
    }
    if (hold) {
      want_boost_ = false;
      boost_armed_ = false;
      if ((f.now - last_hold_log_).seconds() > 2.0) {
        last_hold_log_ = f.now;
        RCLCPP_INFO(get_logger(),
                    "ブースト見送り(%s) 残%d rank=%d フィニッシュまで%.0fm",
                    hold, boost_remaining_, rank_, dist_to_finish);
      }
    }
  }

  if (want_boost_ && !is_boosting_ && boost_remaining_ > 0) {
    Float32MultiArray bm;
    if (!boost_armed_) {
      bm.data = {0.0f};
      boost_pub_->publish(bm);
      boost_armed_ = true;
    } else {
      bm.data = {1.0f};
      boost_pub_->publish(bm);
      boost_armed_ = false;
      boost_used_++;
      last_boost_time_ = now;
      // 撃ったら要求を下ろす。残したままだと次の周期でもう1個撃ってしまう。
      want_boost_ = false;
      if (start_boost_pending_) { start_boost_used_ = true; }   // スタートの1本は1回だけ
      RCLCPP_INFO(get_logger(), "ブースト発動 (%d個目, 残り%d)", boost_used_, boost_remaining_);
    }
  } else if (!want_boost_) {
    boost_armed_ = false;
  }
}

// 前をふさいでいる相手の状況を1秒に1回だけ出す。
void V2XOvertaker::logBlocker(const Frame & f, PlanCtx & c)
{
  const double ev = f.ev;
  const rclcpp::Time now = f.now;

  if (!c.blocker.empty() && (this->now() - last_log_).seconds() > 1.0) {
    last_log_ = this->now();
    RCLCPP_INFO(
      get_logger(), "前方 %s まで %.1f m / 横オフセット %.2f -> %.2f m / 速度上限 %.1f km/h",
      c.blocker.c_str(), c.best_gap, offset_, c.target_offset,
      c.speed_cap >= 0 ? c.speed_cap * 3.6 : -1.0);
    (void)ev;
  }
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<V2XOvertaker>());
  rclcpp::shutdown();
  return 0;
}
