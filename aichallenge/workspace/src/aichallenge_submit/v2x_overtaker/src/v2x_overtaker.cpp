// V2X の他車位置を見て走行ラインを横にずらし、追い越し・追従を行う。
//
// 設計:
//   simple_trajectory_generator が出す素の軌道を受け取り、横オフセットを掛けて
//   publish しなおす。制御(simple_pure_pursuit)は触らないので、単独走行時の
//   タイムアタック性能をそのまま保てる。
//
//   前方に他車がいる -> 反対側へよける。よける幅は corridor.csv の可動域で頭打ちにする。
//   よけきれない     -> 前車速度に合わせて追従し、追突(crash ペナルティ)を避ける。
//   他車がいない     -> オフセットを 0 に戻して元のラインへ復帰。
//
//   オフセットは時間レート制限つきで動かす。急に横へ飛ぶと pure_pursuit が
//   過大な操舵を出すため。
#include "v2x_overtaker/v2x_overtaker.hpp"

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
  ttc_threshold_(declare_parameter<double>("ttc_threshold", 1.0)),
  big_gap_closing_(declare_parameter<double>("big_gap_closing", 5.0)),
  inside_time_gain_(declare_parameter<double>("inside_time_gain", 1.4)),
  inside_width_gain_(declare_parameter<double>("inside_width_gain", 0.85)),
  latch_width_gain_(declare_parameter<double>("latch_width_gain", 0.90)),
  pass_gap_(declare_parameter<double>("pass_gap", 1.7)),
  pass_side_clear_(declare_parameter<double>("pass_side_clear", 0.8)),
  follow_gap_(declare_parameter<double>("follow_gap", 7.0)),
  offset_rate_(declare_parameter<double>("offset_rate", 1.2)),
  corridor_safety_(declare_parameter<double>("corridor_safety", 0.65)),
  // 追い越しを仕掛けている最中だけ縁までの余裕を削る。
  // 実測(3レース): 却下の 83% は幅・時間・距離が足りているのに side_fits_=false。
  // 幅 4.5m の場所でも corridor_safety=0.65 を両側で引くと使えるのは 3.2m しかなく、
  // 相手の横位置しだいで片側の余地が min_pass_sep(1.15m) に届かない。
  // 実測(3レース): 0.45 では壁接触2件とも走行可能領域の外 0.56〜0.68m に
  // いた。削りすぎると縁を割るので 0.55 に戻す。
  corridor_safety_pass_(declare_parameter<double>("corridor_safety_pass", 0.55)),
  // 追い越し可能ゾーン(pass_ok)だけは、さらに削る。
  // 0.45 で壁に当たったのはコーナーで追従誤差が出たため。
  // pass_ok は「幅 4.0m 以上・曲率半径も十分」を満たす区間として
  // make_corridor.py が選んだ場所で、追従誤差が小さく壁も遠い。
  // corridor_ten.csv 自体が既に壁から 半幅0.73+余裕0.45=1.18m 内側にあるので、
  // ここで 0.30 を引いても壁までは 1.48m、車体半幅 0.65 を除いて 0.83m 残る。
  corridor_safety_zone_(declare_parameter<double>("corridor_safety_zone", 0.30)),
  // 左右の余地を見る先読み距離。8m だと先の狭まりを今の制約として引き込み、
  // 目の前は空いているのに出られなくなっていた。車が進めば毎周期引き直される。
  // ただし 5m だとレースライン約10点ごとに余地の符号が反転し、
  // 側を変更した直後にその判断が古くなる(実測: 却下 128 件のうち 98 件=77% は
  // 反対側なら成立していた)。
  // 一方 15m はコーナーで共通部分が空集合(余地=[0.30,-0.30] のような反転)になり、
  // 両側とも「余地なし」で却下の 93% を占めた(実測 2レース、成功 0)。
  // 8m は追い越しが実際に成立した(オフセット -2.25m を5秒保持)ときの値。
  // 8m の共通部分でクランプすると、判断(学習マップ 30m・相手の常用ライン基準)が
  // 通っているのに横目標が出せない。実測(5レース): 追越失敗の最多が
  // allow=1 zone=1 feasible=1 latch=1(28件)= 出てよいのに出られずラインへ戻る。
  // 5m は過去に側が振れて悪化した値だが、それは側の変更枠が総量制で
  // 学習マップも無かった頃の話。今は flip の時間回復と学習で側が安定している。
  side_room_ahead_(declare_parameter<double>("side_room_ahead", 5.0)),
  // 選んだ側の余地が無い状態がこの秒数連続で続いたら、反対側へ回り直す
  side_flip_hold_(declare_parameter<double>("side_flip_hold", 0.6)),
  // 1台の対象車に対して側を変更してよい回数
  side_flip_max_(declare_parameter<int>("side_flip_max", 2)),
  // 枠を時間で回復させる。総量制のままだと、同じ相手が長く前にいるレースで
  // 序盤に枠を使い切り、以降ずっと余地の無い側に張り付いたままになる。
  // 実測(6レース、却下ログ476件): 側OK=0 のうち 228件(48%)は
  // 「反対側なら成立していた」場面だった。
  // この秒数につき1回ぶん回復。バースト4回・以降は 1回/この秒数 に制限される。
  side_flip_regen_(declare_parameter<double>("side_flip_regen", 15.0)),
  // --- 相手の走行ラインの学習 ---
  // 相手は毎周ほぼ同じラインを走る。1周目に横位置を覚えておき、
  // 2周目以降は「抜き切るまでの区間ぜんぶ」を先に見て側を決める。
  lane_learn_(declare_parameter<bool>("lane_learn", true)),
  lane_map_side_(declare_parameter<bool>("lane_map_side", true)),
  lane_map_stretch_(declare_parameter<double>("lane_map_stretch", 30.0)),
  lane_map_min_pts_(declare_parameter<int>("lane_map_min_pts", 8)),
  lane_map_margin_(declare_parameter<double>("lane_map_margin", 2.0)),
  // 足りている区間が pass_len のこの倍だけ連続していれば、その側は成立
  lane_map_need_gain_(declare_parameter<double>("lane_map_need_gain", 1.0)),
  zone_look_ahead_(declare_parameter<double>("zone_look_ahead", 40.0)),
  window_full_(declare_parameter<double>("window_full", 15.0)),
  window_end_(declare_parameter<double>("window_end", 30.0)),
  window_back_(declare_parameter<double>("window_back", 3.0)),
  start_merge_dist_(declare_parameter<double>("start_merge_dist", 70.0)),
  start_lat_max_(declare_parameter<double>("start_lat_max", 0.9)),
  // 3位スタートのとき、1位(NPC・25km/hハンデで遅い)ではなく
  // 2位(プレイヤー)側へ寄せて出る。
  start_p3_follow_(declare_parameter<bool>("start_p3_follow", true)),
  start_p3_lat_(declare_parameter<double>("start_p3_lat", 0.6)),
  // 停止している前車がこの距離[m]より近いときは、
  // stop_hold_sec 秒のあいだ下限速度を課さない(突っ込まない)。
  stop_hold_gap_(declare_parameter<double>("stop_hold_gap", 3.0)),
  stop_hold_sec_(declare_parameter<double>("stop_hold_sec", 2.0)),
  // 自車がこの速度[m/s]を超えているときだけ「待つ」。
  // 停止状態から発進するときに待つと、スタートで出遅れるだけ。
  stop_hold_move_(declare_parameter<double>("stop_hold_move", 0.5)),
  slow_leader_speed_(declare_parameter<double>("slow_leader_speed", 2.5)),
  attempt_timeout_(declare_parameter<double>("attempt_timeout", 16.0)),
  // 打切(16s)まで引っ張ると、その間ずっと横に出たままで前にも出られない。
  // 同速の相手には最初の数秒で進展が出るかどうかが決まるので、
  // 進展がなければ早めに降りてラインへ戻る。
  // 実測(charge2、3レース): 4.0/0.5 では試行の大半(17-27回/レース)がこれで
  // 降り、追越成功が 3レースとも 0 になった(ベースラインは3レースで1回成功)。
  // 伸びかけた試行まで降ろしている。0 以下で無効。切り分けのため既定は無効。
  attempt_stall_time_(declare_parameter<double>("attempt_stall_time", 0.0)),
  attempt_stall_gain_(declare_parameter<double>("attempt_stall_gain", 0.5)),
  attempt_stall_cool_(declare_parameter<double>("attempt_stall_cool", 5.0)),
  // 「相手より8m前」を完全追越の条件にすると、計画に必要な連続区間が18mへ
  // 膨らみ、実在する区間(中央値1m、上位10%で11〜15m)と噛み合わなかった。
  // 4m 前に出ていれば追い越しは成立している。計画と実行で同じ値を使う方針は
  // 維持する(以前この2つが 4m と 8m でずれていて不整合バグを起こした)。
  pass_len_(declare_parameter<double>("pass_len", 4.0)),
  pass_done_len_(declare_parameter<double>("pass_done_len", 5.0)),
  spot_here_enable_(declare_parameter<bool>("spot_here_enable", true)),
  spot_here_range_(declare_parameter<double>("spot_here_range", 30.0)),
  pass_done_sec_(declare_parameter<double>("pass_done_sec", 0.4)),
  // 追い越し1件の所要時間の下限[s]。これ未満は偽陽性として数えない。
  pass_done_min_sec_(declare_parameter<double>("pass_done_min_sec", 1.5)),
  pen_speed_(declare_parameter<double>("pen_speed", 1.38889)),
  pen_tol_(declare_parameter<double>("pen_tol", 0.25)),
  pen_hold_(declare_parameter<double>("pen_hold", 0.6)),
  // --- 壁衝突の予測監視(2026-09-03 ユーザー指示)
  // 【調整方針 2026-09-03 ユーザー指示】これは最終ストッパーであり、通常走行では
  // 作動しないことが正しい。したがって余裕は限界ぎりぎりに置く。実走で壁接触が
  // 残る場合にだけ、wall_guard_margin を 0.02 -> 0.05 -> 0.10 のように少しずつ
  // 増やして詰める。最初から余裕を持たせると、本来通れるラインを塞いで
  // 追い越しの機会を減らすうえ、どのパラメータが効いているのか分からなくなる。
  // 寸法は本プロジェクトの実測値を既定にする(hpp のコメント参照)。
  // 公式 vehicle_info.param.yaml の wheel_base 1.087 / 車幅 1.30 とは
  // 食い違うが、実測はホイールベース 2.14m / 実幅 1.46m。
  wall_guard_enable_(declare_parameter<bool>("wall_guard_enable", true)),
  // 【修正 2026-09-03】0.8秒は長すぎた。30km/h で 6.7m 直進する想定になり、
  // コーナーでは「直進すればコースを出る」が常に成立して、raceline 上の 64% の
  // 地点で許容範囲が直進を含まなくなっていた(実測)。それは「壁にぶつかりそう」
  // ではなく「経路を追従していない」の検出であり、最終手段が経路追従器に化ける。
  // 車体は後軸から前へ 2.61m あるので、0.35秒(25km/h で 2.4m 前進)でも
  // 差し迫った接触は捉えられる。
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
  // --- 占有格子による舵角ガード(2026-09-03 ユーザー指示)
  // 既定の地図は make_corridor.py が使っているものと同じ
  // ten_final_ver3/ten_occupancy_grid_map.{yaml,pgm}。コンテナ内では
  // /aichallenge がリポジトリの aichallenge/ にマウントされている。
  occ_enable_(declare_parameter<bool>("occ_enable", true)),
  occ_map_yaml_(declare_parameter<std::string>(
    "occ_map_yaml",
    "/aichallenge/workspace/install/multi_purpose_mpc_ros/share/multi_purpose_mpc_ros"
    "/env/ten_final_ver3/ten_occupancy_grid_map.yaml")),
  occ_sample_step_(declare_parameter<double>("occ_sample_step", 0.15)),
  occ_steer_bins_(declare_parameter<int>("occ_steer_bins", 41)),
  occ_clear_search_(declare_parameter<double>("occ_clear_search", 1.0)),
  pass_time_limit_(declare_parameter<double>("pass_time_limit", 18.0)),
  boost_gain_(declare_parameter<double>("boost_gain", 3.0)),
  boost_retry_sec_(declare_parameter<double>("boost_retry_sec", 6.0)),
  boost_min_speed_(declare_parameter<double>("boost_min_speed", 4.5)),
  // 【未検証 2026-09-04】0.5 m/s^2 という値の出典は過去のコード内コメントだけで、
  // 本体 docs にも AWSIM バイナリの文字列にも根拠を確認できていない。
  // ブーストを使った追い越しの成否はこの値に依存するので、実測すること。
  boost_accel_(declare_parameter<double>("boost_accel", 0.5)),
  boost_duration_(declare_parameter<double>("boost_duration", 10.0)),
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
  // --- 余ったブーストを直線で使い切る ---
  // 追い越し成立を前提にした条件だけだと、実測で1レース2個とも一度も
  // 撃たれなかった(単独走行 0回、3台走行でも全試行が ブースト=0)。
  // 使わないブーストの価値はゼロなので、安全な直線で使い切る。
  // 追い越しに結びつかないブーストは撃たない(ユーザー方針)。
  // 実測(3レース、lanemap): 3個のうち撃たれた2個はすべて
  // 「スタート直後」「終盤」の自由発射で、追い越し用の発射経路
  // (allow && boost_would_help)は一度も発火していなかった。
  // うち1個は 前方空き0(前が詰まったまま)で撃たれていて丸損。
  // ブーストは「抜けると判断したとき」だけに使う。
  free_boost_enable_(declare_parameter<bool>("free_boost_enable", true)),
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
  // 残りこの周回数から、余ったブーストを使ってよい。
  // 1(最終ラップのみ)だと使い切れずに完走後へずれ込む。
  // 前が空いているときの自由発射は「最終ラップだけ」(ユーザー指示)。
  // 6 = 全周で撃てる設定だった。1 にすると最終ラップ(6周中の6周目)のみ。
  free_boost_laps_left_(declare_parameter<int>("free_boost_laps_left", 1)),
  // 直線判定を待たずにブーストしてよい区間。"開始:終了" をカンマ区切り。
  // メインストレート(idx232-241)の手前、コーナーの立ち上がりから
  // 加速を始めるために使う。
  boost_zone_spec_(declare_parameter<std::string>("boost_zones", "220:241")),
  // 追い越しを試みてはいけない区間。"開始:終了" をカンマ区切り(0またぎ可)。
  // 実測: idx78-92 はコース最狭部で、幅 2.3m に対し必要 2.7m。
  // ここでは「側の余地あり」と「幅あり」が両立しないので、仕掛けても
  // latch の時間と側の変更枠を消費するだけで終わる。
  // 禁止区間は「本当に2台入らない場所」だけにする。
  // 実測(corridor_ten.csv 242点): 幅が足りないのは idx91-93 の
  // 2.30/2.30/2.95m だけで、idx76-90 は 3.20〜3.80m あり2台入る(要 2.60m)。
  // idx80-88 はコリドアが左に寄っている(左 hi が 1.75->0.35 まで縮む)だけで、
  // 右側には 1.9〜2.85m の余地が残っている。右からなら抜ける。
  // idx94-95 は幅こそ広いが曲率半径 6.7/5.7m のヘアピン入口なので残す。
  // 89:95 に絞った(却下85件のうち 41件=48% がこの禁止区間だった)。
  no_pass_zone_spec_(declare_parameter<std::string>("no_pass_zones", "18:30,89:95")),
  // 公式オーバーテイクレーンの idx 範囲。AWSIM のシーンから実測した値。
  // local_script/overtaking_zone.py で再取得できる(AWSIM 更新時は確認する)。
  ot_lane_zone_spec_(declare_parameter<std::string>("ot_lane_zones", "234:21")),
  // 右側から抜くと決めている区間(ユーザー指示)。書式は boost_zones と同じで
  // 0 をまたぐ指定もできる。既定はメインストレート idx220 -> 30。
  // 【空にした(ユーザー指示 2026-08-29)】区間を焼き込んで「ここは右から」と
  // 決める指定。本番は相手が変わるので外れる。側は録画した相手のラインの
  // 空き幅から動的に決める(chooseSide)。互換のため機能自体は残してある。
  right_zone_spec_(declare_parameter<std::string>("right_zones", "")),
  // スタートグリッドの座標。"x1:y1,x2:y2,x3:y3" の順に P1,P2,P3。
  // 空なら進行度順にフォールバックする。
  // 値は `スタート位置 P... 座標=(x,y)` のログから書き写す。
  grid_slot_spec_(declare_parameter<std::string>("grid_slots", "")),
  // 相手の平均速度が自車のこの割合を下回っていたら「明らかに遅い」とみなす。
  // 実測: MPC のラップ 74.7s に対し自コード 49.9s(比 0.67)。
  slow_rival_ratio_(declare_parameter<double>("slow_rival_ratio", 0.85)),
  // スタートが2位・3位のときだけ、序盤に1つ使って前に出る。
  // 1位で始まったなら前が空いているので使わない。
  start_boost_enable_(declare_parameter<bool>("start_boost_enable", true)),
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
  // 14 は空けすぎだった。追従則の遅れで入口までに回収し切れず、
  // ゾーン入口で 9m 残って直線内に抜き切れない(実測: 所要9秒級の失敗)。
  approach_gap_max_(declare_parameter<double>("approach_gap_max", 8.0)),
  // ゾーン入口の直前では、逆算した want ではなく「抜くのに要る車間」まで
  // 詰めきる。want は入口到達時点で pass_gap になる想定だが、追従則の
  // 一次遅れが残るため実際には入口で 5-6m 残っていた(実測)。
  charge_close_dist_(declare_parameter<double>("charge_close_dist", 20.0)),
  charge_close_gap_k_(declare_parameter<double>("charge_close_gap_k", 1.2)),
  charge_brake_margin_(declare_parameter<double>("charge_brake_margin", 1.5)),
  // --- 相手の減速を先読みする ---
  // カーブでは相手はほぼ確実に落とす。今の速度だけを見て追従すると
  // 後ろから加速していって追突する。
  predict_enable_(declare_parameter<bool>("predict_enable", true)),
  predict_ahead_(declare_parameter<double>("predict_ahead", 12.0)),
  near_radius_(declare_parameter<double>("near_radius", 6.0)),
  predict_floor_(declare_parameter<double>("predict_floor", 0.55)),
  // 相手の減速の先読みに、録画した地点ごとの相手の速度も使うか。
  // 1周分の実測が揃った後は、相手を自車の速度表で走らせるより、相手自身の
  // 地点別速度を使う。動く相手との合流時刻を合わせるため既定で有効にする。
  predict_lane_speed_(declare_parameter<bool>("predict_lane_speed", true)),
  // 【修正O 2026-09-02】相手の学習速度(地点別)に掛ける安全率。
  predict_op_margin_(declare_parameter<double>("predict_op_margin", 1.10)),
  // 【2026-08-31 修正】1.15 -> 1.66。**車幅と食い違っていた。**
  // カートの車幅は 1.46m(半幅0.73×2)。1.15m を目標に横へ出ても
  // **車体は 0.31m 重なったまま**で、抜けずに当たる。
  // さらに追突防止の緩和は「横間隔 1.66m 以上」で発動するのに、
  // 横制御は 1.15m しか狙わないので**緩和が永久に発動しない**。
  // 低速のMPCを抜けない直接の原因。車幅1.46 + 余裕0.2 = 1.66 にそろえる。
  min_lat_sep_(declare_parameter<double>("min_lat_sep", 1.66)),
  repulse_need_allow_(declare_parameter<bool>("repulse_need_allow", false)),
  repulse_band_margin_(declare_parameter<double>("repulse_band_margin", 0.0)),
  boost_side_gap_(declare_parameter<double>("boost_side_gap", 99.0)),
  // 【2026-08-31 修正】3.2 -> 1.8。**自車の車体幅を二重に数えていた。**
  //
  // 比較相手の avail_width は「車体中心を置いてよい幅」で、
  // corridor_ten.csv の時点で既に 車体半幅0.73 + 余裕0.45 = 1.18m を
  // 両側から控除してあり、さらに corridor_safety(0.65)も引いてある。
  // つまり avail_width = 物理的な壁の間隔 - 3.66m。
  // そこへ「カート2台ぶん 3.2m」を要求すると、実質
  // **壁の間隔 6.86m 以上**を求めることになる。コースの平均は 6.76m なので
  // ほぼどこでも成立しない。実測: 低速のMPC(10.5km/h)に対し
  // `幅=2.6` で却下され、14.5km/h の速度差がありながら抜けなかった。
  //
  // 物理的に2台並ぶのに要るのは 壁間 1.46x2 + 隙間0.2x2 = 3.32m で、
  // これは avail_width >= -0.34m に相当する。安全側に倒して 1.8
  // (= 壁間 5.46m 相当、物理的必要量より 2.1m 余裕)とする。
  // なお実際の横位置はバンドが従来どおり制御するので、
  // これは「仕掛けるかどうか」の閾値でしかない。
  min_pass_width_(declare_parameter<double>("min_pass_width", 1.8)),
  rear_end_lat_release_(declare_parameter<bool>("rear_end_lat_release", true)),
  // 車体が触れない横間隔[m]。車幅1.46 + 余裕0.2。これ未満では一切緩めない。
  rear_end_free_min_(declare_parameter<double>("rear_end_free_min", 1.66)),
  // ここまで離れたら完全に開放する[m]
  rear_end_free_full_(declare_parameter<double>("rear_end_free_full", 2.20)),
  rear_end_free_speed_(declare_parameter<double>("rear_end_free_speed", 10.0)),
  min_pass_sep_(declare_parameter<double>("min_pass_sep", 1.15)),
  // --- 同速の相手には仕掛けない ---
  // 実測(3レース): 同型の僚車(自分と同じ速度)への試行は抜き切るのに
  // 46〜90m 必要で、事実上成立しない。一方 MPC(遅い)は 42〜46m で足りる。
  // 相対速度と必要距離の両方で足切りする。
  min_closing_kmh_(declare_parameter<double>("min_closing_kmh", 12.0)),
  pass_dist_max_(declare_parameter<double>("pass_dist_max", 45.0)),
  stopped_speed_(declare_parameter<double>("stopped_speed", 1.0)),
  stopped_look_ahead_(declare_parameter<double>("stopped_look_ahead", 30.0)),
  stop_margin_(declare_parameter<double>("stop_margin", 3.0)),
  stop_brake_k_(declare_parameter<double>("stop_brake_k", 0.30)),
  // 追突の待ちを判定するときに、制動距離へ足す余裕[m]。
  stop_hold_margin_(declare_parameter<double>("stop_hold_margin", 0.5)),
  stopped_cluster_span_(declare_parameter<double>("stopped_cluster_span", 8.0)),
  stopped_slack_(declare_parameter<double>("stopped_slack", 0.5)),
  stopped_thread_speed_(declare_parameter<double>("stopped_thread_speed", 3.0)),
  stopped_clear_speed_(declare_parameter<double>("stopped_clear_speed", 8.0)),
  stopped_clear_gain_(declare_parameter<double>("stopped_clear_gain", 4.0)),
  stopped_clear_in_(declare_parameter<double>("stopped_clear_in", 0.40)),
  band_enable_(declare_parameter<bool>("band_enable", true)),
  band_clamp_(declare_parameter<bool>("band_clamp", true)),
  band_predict_(declare_parameter<bool>("band_predict", true)),
  band_horizon_(declare_parameter<double>("band_horizon", 80.0)),
  band_long_(declare_parameter<double>("band_long", 2.6)),
  band_car_w_(declare_parameter<double>("band_car_w", 1.30)),
  band_v_floor_(declare_parameter<double>("band_v_floor", 3.0)),
  band_slope_(declare_parameter<double>("band_slope", 0.20)),
  // バンドの一次遅れ係数。20Hz で x += (目標 - x) * band_smooth。
  // 0.30 は 時定数 0.14s / 95%到達 0.42s で、相手が急に横へ動いたときの
  // 反応が遅かった(ユーザー指摘)。0.45 = 時定数 0.083s / 95%到達 0.25s。
  // 【差し戻し 2026-08-31 15:35】0.45(95%到達0.25s)にしたところ、
  // 同じビルドで Wall が増え自車平均が 22.6 -> 17.6km/h に落ちた。
  // 帯の詰めすぎ(下記 pass_margin_min)との切り分けができていないので、
  // 既知の最良である 0.30 に戻す。速める場合は単独走行で壁接触を見ながら。
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
  // 追い越し中に「相手の端と壁の中点」を狙うか(ユーザー指示)。偽なら従来の pass_gap 固定。
  pass_center_(declare_parameter<bool>("pass_center", true)),
  pass_window_sec_(declare_parameter<double>("pass_window_sec", 1.2)),
  // 追い越し・回避の最中だけ、直線で壁の余裕をここまで詰める[m]。
  // 【差し戻し 2026-08-31 15:35】0.20 では実測で壁に当たった。
  // 追従誤差が ±0.24m あるので 0.20m の余裕では足りない。
  // 既定は corridor_safety と同値にして **この仕組みを無効**にしておく。
  // 詰める場合は単独走行(evalrun)で公式ペナルティを見ながら段階的に。
  // 【2026-08-31 17:55 差し戻し】0.40 -> 0.65(= corridor_safety と同値 = 無効)。
  // ユーザー報告「前に誰もいないのに P2 が壁に衝突」。実測:
  //   接触種別=壁 idx=238 横=-2.48 回避[横0.00m] 最近傍車=4.0m
  // 回避は働いておらず、経路の跳びも 0.12m と小さい。
  // 走行可能帯の外側 0.58m まで出ている。壁側に寄れる余地を広げたことが効いている。
  // この仕組みは効果を確認できていないので、安全側に戻して無効化する。
  // 仕組み自体は残してあるので、単独で計測すれば再有効化できる。
  // 【2026-08-31 18:00 復帰】0.65(無効) -> 0.40。差し戻しは私の誤りだった。
  // この処理は pass_mode = attempt_active_ || stop_avoid || straight_pass のときだけ働く。
  // 壁接触の場面(P2が1位・前に誰もいない・回避[横0.00m]・最近傍車4.0m)では
  // **いずれの条件も成立しないので、そもそも適用されていなかった**。
  // 「帯の外側0.58m」だけを見て原因と決めつけたのが誤り。
  // 壁接触の真因は追従誤差か別の層による押し出しで、別途切り分ける。
  pass_margin_min_(declare_parameter<double>("pass_margin_min", 0.40)),
  // 曲率半径がこれ以下なら従来の余裕のまま[m]
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
  band_side_look_(declare_parameter<double>("band_side_look", 40.0)),
  launch_free_sec_(declare_parameter<double>("launch_free_sec", 6.0)),
  launch_free_gap_(declare_parameter<double>("launch_free_gap", 0.9)),
  launch_free_decel_(declare_parameter<double>("launch_free_decel", 0.48)),
  launch_free_react_(declare_parameter<double>("launch_free_react", 0.35)),
  launch_free_room_(declare_parameter<double>("launch_free_room", 1.20)),
  look_width_ahead_(declare_parameter<double>("look_width_ahead", 20.0)),
  v2x_timeout_(declare_parameter<double>("v2x_timeout", 1.0)),
  safe_gap_(declare_parameter<double>("safe_gap", 3.0)),
  safe_gap_min_(declare_parameter<double>("safe_gap_min", 3.0)),
  safe_gap_max_(declare_parameter<double>("safe_gap_max", 5.0)),
  gap_brake_ratio_(declare_parameter<double>("gap_brake_ratio", 0.5)),
  // 【実測 2026-09-04】単独走行(公式 make eval、接触0件)の
  // /localization/acceleration で減速側は 最小 -2.44 m/s^2 に達していた。
  // 2.5 という仮定はこれとほぼ一致する。ただし**フルブレーキの専用計測は
  // まだしていない**(単独走行では減速指令が一度も出ていない。
  // cmd_accel の最小が +0.79)。追突の距離計算がこの値に依存するので、
  // 直線で最大減速指令を出す単独テストで確認する価値がある。
  a_min_(declare_parameter<double>("a_min", 2.5)),
  follow_kp_(declare_parameter<double>("follow_kp", 0.8)),
  // 車間がこれ[m]を上回っている間は、追従の上限を相手速度より下げない。
  // 接触は中心間 2.6m で起きるので、それに余裕を足した値。
  follow_keep_gap_(declare_parameter<double>("follow_keep_gap", 3.5)),
  min_follow_speed_(declare_parameter<double>("min_follow_speed", 2.2)),
  side_hold_time_(declare_parameter<double>("side_hold_time", 3.0)),
  // --- 自分が先頭でハンデを受けている間の追い越し許可 ---
  // 1位は 25km/h に制限される。相手が 13km/h 以上で走っていれば
  // closing は構造的に min_closing_kmh(12km/h) へ届かず、
  // 「同速」と判定されて永久に仕掛けられない。
  // 実測(3レース・却下471件): 却下の 77% が rank=1。
  // 直線手前 idx215-241 に限ると却下53件のうち48件(91%)が
  // 「ゾーン・幅・側はすべて成立していて同速だけが理由」だった。
  // ここでは closing の足切りを下げ、代わりに距離で縛る。
  capped_self_enable_(declare_parameter<bool>("capped_self_enable", true)),
  capped_self_closing_(declare_parameter<double>("capped_self_closing", 3.0)),
  capped_self_dist_(declare_parameter<double>("capped_self_dist", 70.0)),
  // --- 横に出切ったら追従キャップを外して抜き切る ---
  // commit_sep は min_lat_sep と同じ値にしてある。回避層は
  // 「横間隔 >= min_lat_sep なら当たらない」として当該車を無視するので、
  // 同じ境界で追従キャップも手放すのが一貫する。
  commit_pass_(declare_parameter<bool>("commit_pass", true)),
  latch_commit_(declare_parameter<bool>("latch_commit", false)),
  // 追従キャップを外す(= 全開で加速する)のに要る**実測**の横間隔[m]。
  //
  // 【1.15 は危険だった。実戦で Crash 4件を出している。】
  // カート幅は 1.30m。2台が触れずに並ぶには中心間で 1.30m 以上の
  // 横間隔が要る。1.15 では **0.15m 重なっている**状態で全開加速していた。
  // 実戦の bag から実測した公式ペナルティ 6件・計44.6秒のうち、
  // **Crash 4件はすべて相手が中心間 1.7〜2.0m** のときに起きている
  // (カート全長は約2.6m なので、この距離で横に重なりがあれば必ず当たる)。
  // Crash は 10秒 5km/h 固定 = 通常35km/h なら 80m 以上の損失で、
  // 追い越し1回の利得を大きく上回る。
  //
  // min_pass_sep(1.15)が車幅を下回っているのは「抜けるかどうかを**計画**する」
  // ための意図的な値(開発メモ)。同じ値を「**全開で加速してよい**」の判断に
  // 流用したのが設計ミスだった。ここは物理の車幅を下回ってはいけない。
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
  // 幅の判定を「抜き切るのに要る距離」まで伸ばす。
  // 1.0 でその距離ぶん、0 で従来どおり look_width_ahead_(20m)だけを見る(A/B用)。
  latch_allow_enable_(declare_parameter<bool>("latch_allow_enable", true)),
  latch_never_no_pass_(declare_parameter<bool>("latch_never_no_pass", false)),
  // 【採用 2026-09-05】レース単位で各3レース比較し、壁ペナルティ 17.3 -> 7.8s/車レース、
  // 総ペナルティ 23.2 -> 17.0s/車レース、完走できなかった車 2/12 -> 0/12。既定を true にした。
  stop_avoid_fix_(declare_parameter<bool>("stop_avoid_fix", true)),
  // 自車の車体(前後 約1.0m)+ 相手の車体 + 余裕。抜け切るまでを覆う。
  stop_avoid_span_(declare_parameter<double>("stop_avoid_span", 6.0)),
  // 横位置が実現するまでの距離。前セッションの実測「横位置が20m遅れて実現する」。
  // 純粋な遅れとして扱う(そのほうが安全側)。
  stop_avoid_lat_lag_(declare_parameter<double>("stop_avoid_lat_lag", 20.0)),
  stop_avoid_emg_margin_(declare_parameter<double>("stop_avoid_emg_margin", 2.0)),
  // AWSIM は加速度指令を 1.37 に切り捨てる(開発メモ の実測)。
  // 要求できる減速度の上限がそれなので、発火距離もこれで計算する。
  // 【実測 2026-09-05】急ブレーキを実測した(local_script/brake_measure2.py)。
  // 強い制動(指令 <= -1.2 m/s^2)を出した窓だけを切り出し、衝突の衝撃(|ax|>5)を
  // 除いた結果:
  //   達成された平均減速度 中央 1.66 / p90 1.85 / 最大 1.94 m/s^2
  //   瞬間の減速度        中央 1.92 / 最大 2.38 m/s^2
  //   指令は ±2.00 でクランプされている
  // 制動距離は 8m/s から 19.3m、10m/s から 30.2m。
  // 2.0 は楽観(10m/s で 25m)だったので実測の中央値へ寄せる。
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
  // **車間の下限**。これが無いと「助走」にならない。
  // 実測: 下限なしだと車間 1.4〜1.5m で発火していた。相手の直後に張り付いた
  // 状態で加速するので、助走にならないどころか後部に突っ込んで Crash になる。
  // 加速してから届くだけの距離を残して撃つ。
  boost_runup_gap_min_(declare_parameter<double>("boost_runup_gap_min", 6.0)),
  // --- 壁回避の優先(ユーザー方針)
  // 横目標を必ず [lo+wall_margin, hi-wall_margin] に収める。
  // corridor_safety(0.65) と同じにしてあるので、通常時の挙動は変わらない。
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
  // 【2026-08-31 有効化】0.0(無効) -> 1.4m/s(5km/h)。
  // 【実測(ユーザー報告「低速MPCを抜けない」の直接原因)】
  //   停止車両 1台 先頭 d3 まで 3.1m 空き幅 1.83m -> 通過 (上限 10.8km/h)
  //   追突防止 d3 まで 3.1m 相手 0.0km/h 上限 10.8 -> 0.0km/h 自車 -0.0km/h
  //   前方 d3 まで 3.1m / 横オフセット -0.06 -> -2.20 m / 速度上限 0.0 km/h
  // 停止車回避は「脇を通れる」と判断し横目標も出しているのに、
  // room = 車間3.1 - rear_end_margin3.5 < 0 で追突防止が上限を 0 に潰す。
  // **速度0のカートは横に動けない**(横移動は前進でしか起きない)ので、
  // 脇に 1.83m の通り道があるのに、そこへ寄るための前進が禁じられる循環になる。
  // 通れる帯があると確認できている間だけ、寄るのに必要な微速を許す。
  // 過去に 1.8m/s で試して「改善なし」と棄却されているが、それは
  // 停止車の予測・幅の要求・横間隔の目標がいずれも誤っていた頃の判定。
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
  corridor_extra_(declare_parameter<double>("corridor_extra", 0.45)),
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
  // スタートの横位置を丸めるときに壁から空ける量[m]。
  // スタート直線は幅 5m 近くあり直線なので、通常の wall_margin(0.65)より
  // 詰めてよい。詰めないとグリッド右端の P2 が左へ寄せられ、
  // 前の P3 と右から来る P1 に挟まれる(実測)。
  start_wall_margin_(declare_parameter<double>("start_wall_margin", 0.35)),
  launch_hold_grid_(declare_parameter<bool>("launch_hold_grid", true)),
  launch_hold_dist_(declare_parameter<double>("launch_hold_dist", 12.0)),
  launch_hold_sec_(declare_parameter<double>("launch_hold_sec", 12.0)),
  launch_p1_right_(declare_parameter<bool>("launch_p1_right", false)),
  launch_p1_lat_near_(declare_parameter<double>("launch_p1_lat_near", -0.90)),
  launch_p1_lat_far_(declare_parameter<double>("launch_p1_lat_far", -3.20)),
  // submit_11 の公式P1では P2 が約4m先へ出た段階から右へ移り、P3との
  // 横間隔を約3m作って初回で抜けていた。7mまで待つ変更後のローカル実測では
  // 横移動が約2秒遅れ、狭窄まで実間隔1.3〜1.5mのまま速度制限を受けた。
  // P2との縦間隔を確保しつつ、元の3.5mより少し保守的な4.5mで解放する。
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
  spot_gate_max_wait_(declare_parameter<double>("spot_gate_max_wait", 45.0)),
  // spotPathSafeで許容するクランプ量[m]。band は時間平滑化されており
  // 5cm 精度を全点で要求すると実走では通らない(2026-09-02)。
  spot_path_tol_(declare_parameter<double>("spot_path_tol", 0.25)),
  // 上記の許容を連続して超えてよい区間長の上限[m]。これを超えたら閉塞とみなす。
  spot_path_bad_len_(declare_parameter<double>("spot_path_bad_len", 2.0)),
  // 線が外れても、計画した側に車1台ぶん通れる帯[m]がこの幅以上あれば
  // 通行可とみなす(2026-09-02 修正F)。
  spot_path_min_w_(declare_parameter<double>("spot_path_min_w", 1.5)),
  // 抜きどころに着いているのに仕掛けられない状態がこの秒数[s]続いたら、
  // その計画を破棄して次の候補を探す(2026-09-02 修正D)。
  spot_stuck_max_(declare_parameter<double>("spot_stuck_max", 2.0)),
  // 破棄した地点をこの秒数[s]の間だけ候補から外す。
  spot_avoid_sec_(declare_parameter<double>("spot_avoid_sec", 8.0)),
  spot_abort_sec_(declare_parameter<double>("spot_abort_sec", 0.8)),
  // 録画から見た左右の平均空き幅の差がこれ[m]を超えたら、広いほうから抜く。
  side_room_margin_(declare_parameter<double>("side_room_margin", 0.30)),
  // 反対側がこの秒数[s]連続で勝っていないと側を入れ替えない(振られ対策)。
  side_room_hold_(declare_parameter<double>("side_room_hold", 0.8)),
  // 側を録画で決める区間(ユーザー指示: メインストレート idx220 -> 25)。
  // 【実測でこの範囲に直した(2026-08-29)】ユーザー指示は「220から25あたりの直線」
  // だったが、コリドアを測ると **同じ範囲の中で開いている側が入れ替わる**。
  //   idx218-231 : 曲率 6.5〜27  右の幅 0.55〜1.55m / 左 3.0〜4.65m
  //   idx232-17  : 曲率 27〜1484 右の幅 3.20〜4.35m / 左 0.35〜1.60m  <- 真の直線
  //   idx18-30   : 曲率 26 -> 5.0 右の幅 4.10 -> 0.75m               <- コーナー入口
  // idx18 以降は右が急に閉じるので、右いっぱいで並走したまま入ると
  // 相手の正面へ押し込まれる(実測: idx26 で Crash が繰り返し発生)。
  // 直線として扱うのは曲率が大きく右が開いている idx232->17 に限る。
  side_pick_zone_spec_(declare_parameter<std::string>("side_pick_zones", "232:17")),
  // 相手の区間平均の横位置がこれ[m]以内なら「どちらとも言えない」-> 右から抜く。
  side_pick_tie_(declare_parameter<double>("side_pick_tie", 0.30)),
  // --- 対象車以外への追突を止める(ユーザー報告のスタート Crash 対策)
  rear_end_guard_(declare_parameter<bool>("rear_end_guard", true)),
  rear_end_range_(declare_parameter<double>("rear_end_range", 20.0)),
  // 横間隔がこれ[m]未満なら「自分の進路上」。車幅 1.30m に余裕を足す。
  rear_end_sep_(declare_parameter<double>("rear_end_sep", 1.45)),
  // 止まりきる位置に残す余裕[m]。カート全長の半分ぶん。
  rear_end_margin_(declare_parameter<double>("rear_end_margin", 3.5)),
  // 開始車間緩和(start_gap_settled)を許す接近速度の上限[m/s](2026-09-02)。
  start_gap_closing_max_(declare_parameter<double>("start_gap_closing_max", 1.5)),
  start_gap_floor_(declare_parameter<double>("start_gap_floor", 1.5)),
  // 減速の見積りを割り引く係数。指令どおりには効かない。
  // 【実測で見直した 2026-09-05】0.22 は |a_min|(2.5) に対して 0.55 m/s^2 の
  // 制動を前提にしていた。この値は「追突防止が減速を掛けている区間の実測
  // 中央値 0.48」から決めたものだが、**その区間では上限が現在速度のすぐ下に
  // しか置かれていないので、弱い減速しか要求していない**。
  // つまり「弱いと仮定して早めに減速するから弱い要求しか出ない」という
  // 自己充足的な校正になっていた。
  //
  // 強い制動を指令した窓だけを実測すると **1.66 m/s^2 出ている**
  // (local_script/brake_measure2.py)。制動距離は減速度に反比例するので、
  // 0.55 前提では**必要より約3倍手前から速度を落としている**。
  // これが「せっかく加速して得た速度を落として抜けない」の直接の原因。
  rear_end_brake_k_(declare_parameter<double>("rear_end_brake_k", 0.22)),
  // 抜く算段が付いているときの制動係数。実測 1.66 m/s^2 の下限側(1.09)相当。
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
  guard_decel_(declare_parameter<double>("guard_decel", 2.5)),
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
  // --- 周回による解禁(ユーザー指示) ---
  // 1周目は相手の走りを録るために NPC 以外を抜かない。
  // P1 が僚車を抜き始めるのは3周目以降。ブーストも3周目以降。
  // 【2026-09-02 ユーザー指示】次戦がシム決勝のため、最初の数周を追越禁止に
  // する周回ゲートを一時的に無効化する。相手の地点別速度の学習はレース中も
  // 継続して溜まるので、学習を待たずに1周目から仕掛けられるようにする。
  // 恒久的な設定ではない。戻すときは true にする。
  lap_gate_enable_(declare_parameter<bool>("lap_gate_enable", false)),
  npc_slot_(declare_parameter<int>("npc_slot", 3)),
  record_laps_(declare_parameter<int>("record_laps", 1)),
  // 【ユーザー指示 2026-08-31】5 -> 3。表示上は「4周目から解禁」。
  // 早く前に出ると 25km/h のハンデを食うので遅らせていたが、遅すぎた。
  // 【2026-08-31 修正】3 -> 1。実測で
  //   `追越却下 決め手=周回で禁止 self_ok=1 幅=4.5 closing=13.2 所要=5.7s idx=18`
  // のように **抜ける条件が全部揃っているのに周回ゲートだけが止めて**いた。
  // 抜けないと追従に落ち、追突防止が速度を相手に合わせて下げ、接触に至る。
  // 「早く前に出るとハンデを食う」という当初の狙いは残しつつ(1周目は禁止)、
  // 2周目からは抜けるようにする。
  teammate_pass_lap_(declare_parameter<int>("teammate_pass_lap", 1)),
  // 先頭を抜いてよいのは残りこの周回数になってから。0 で無効(いつでも抜く)。
  // 【2026-09-04 ユーザー指示】時期を限定する処理はデバッグを難しくするので既定は無効(0)。
  // 理屈(先頭は25km/hに落とされるので前に出ると抜き返される)は devnote に残してある。
  leader_pass_last_laps_(declare_parameter<int>("leader_pass_last_laps", 0)),
  zone_fallback_enable_(declare_parameter<bool>("zone_fallback_enable", false)),
  // --- 公式オーバーテイクレーン(SIM決勝) ---
  // 既定は false。A/B で効果を確かめてから有効にする。
  ot_lane_enable_(declare_parameter<bool>("ot_lane_enable", false)),
  // BLOCK(20秒)を避けるガード。レーンを追い越しに使うかとは独立に常時有効。
  ot_lane_guard_(declare_parameter<bool>("ot_lane_guard", true)),
  // ガードを効かせ始める先読み距離。横位置が実現するまでの約20m。
  ot_lane_guard_look_(declare_parameter<double>("ot_lane_guard_look", 20.0)),
  ot_lane_guard_time_(declare_parameter<double>("ot_lane_guard_time", 0.5)),
  // 27km/h がアタッカーの閾値。境界で振動しないよう 1km/h の余裕を持たせる。
  ot_lane_min_kmh_(declare_parameter<double>("ot_lane_min_kmh", 28.0)),
  // レーンは自車ラインの右 2.15m から始まる(実測)。
  // 速度が足りないときはその手前で止める。
  ot_lane_guard_lat_(declare_parameter<double>("ot_lane_guard_lat", 2.0)),
  ot_lane_side_right_(declare_parameter<bool>("ot_lane_side_right", true)),
  // 側の決定: 録画で決めた側を曲率のイン優先より優先するか(コメントどおりの実装)
  side_pick_over_curve_(declare_parameter<bool>("side_pick_over_curve", true)),
  // 側の決定: 「相手と壁の空き」で決めるか(false でレースラインからのずれ)
  side_pick_by_room_(declare_parameter<bool>("side_pick_by_room", true)),
  // 側の空きを「区間の最小」で見るか(false で従来の平均)
  side_room_use_min_(declare_parameter<bool>("side_room_use_min", true)),
  // 試行中の横位置を試行の状態が保持するか。false で 2026-09-04 以前の挙動。
  attempt_lat_hold_enable_(declare_parameter<bool>("attempt_lat_hold_enable", false)),
  // --- 抜きどころの成立条件の調整幅(詳細は planPassSpot の need_gain の説明) ---
  // 横へ出る前に取り返す縦の安全車間[m]。負で従来どおり rear_end_margin+1.0。
  spot_entry_gap_(declare_parameter<double>("spot_entry_gap", -1.0)),
  // 相手の前へ出切る量に掛ける係数。1.0 で pass_len そのまま。
  spot_pass_len_gain_(declare_parameter<double>("spot_pass_len_gain", 1.0)),
  // 必要距離に掛ける安全率。1.0 で計算どおり。
  spot_need_margin_(declare_parameter<double>("spot_need_margin", 1.15)),
  // 【ユーザー指示 2026-08-31】2 -> 3。表示上は「4周目から解禁」。
  // lap_ は0始まり。ユーザー方針「3周目以降」は内部番号2から解禁する。
  // 3を指定すると実際には4周目となり、P1が追い上げる最初の機会を失っていた。
  boost_min_lap_(declare_parameter<int>("boost_min_lap", 2)),
  trace_dump_sec_(declare_parameter<double>("trace_dump_sec", 30.0)),
  telem_sec_(declare_parameter<double>("telem_sec", 20.0)),
  // --- 【追加 2026-09-03】観測ログ専用。制御には一切使わない ---
  overtaken_margin_(declare_parameter<double>("overtaken_margin", 2.0)),
  overtaken_hold_(declare_parameter<double>("overtaken_hold", 0.5)),
  contact_decel_(declare_parameter<double>("contact_decel", 3.0)),
  contact_alone_dist_(declare_parameter<double>("contact_alone_dist", 8.0)),
  contact_after_pass_sec_(declare_parameter<double>("contact_after_pass_sec", 3.0)),
  // --- 【追加 2026-09-03】助走(run-up): 抜きどころへ速度を持って着く ---
  runup_enable_(declare_parameter<bool>("runup_enable", true)),
  runup_dv_(declare_parameter<double>("runup_dv", 2.5)),
  runup_gap_max_(declare_parameter<double>("runup_gap_max", 25.0)),
  runup_margin_(declare_parameter<double>("runup_margin", 3.0))
{
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

  // 公式オーバーテイクレーン。書式と 0 またぎの扱いは boost_zones と同じ。
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
  // 占有格子地図を一度だけ読み、符号付き距離場を作る。
  // 失敗したら機能を丸ごと無効化して従来動作へ戻る(警告は 1 回だけ)。
  if (occ_enable_) {
    if (loadOccGrid()) {
      RCLCPP_INFO(get_logger(),
        "壁予測(格子) 読込成功 %s %dx%d res=%.3f origin=(%.3f,%.3f) 占有=%.1f%% "
        "車体=[前%.2f 後%.2f 半幅%.2f] 探索=%.2fm 候補=%d",
        occ_map_yaml_.c_str(), occ_.w, occ_.h, occ_.res, occ_.ox, occ_.oy,
        100.0 * static_cast<double>(occ_.n_occ) /
          std::max<double>(1.0, static_cast<double>(occ_.w) * occ_.h),
        veh_wheel_base_ + veh_front_overhang_, veh_rear_overhang_, veh_half_width_,
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
  sub_state_ = create_subscription<std_msgs::msg::String>(
    "/awsim/state",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](const std_msgs::msg::String::SharedPtr m) {
      if (!race_started_ && m->data == "Start") {
        race_started_ = true;
        race_start_time_ = this->now().seconds();
        // 発進計測ログの基準。ここで入れないと assignStartSlots 側の
        // 「race_start_time_ < 0 のとき」に入らず、ログが1行も出ない。
        launch_since_ = race_start_time_;
        RCLCPP_INFO(get_logger(), "レース開始を検知。回避と追い越しを有効化する");
      }
    });
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

// 急コーナーの進入手前で追い越しを禁じる。
//
// 【実測(2026-09-03、自コード4台×11レース、接触270件)】
// 接触の 26.7% が「pass_ok=1 なのに 20点先までに曲率半径 5m 級のコーナーがある」
// 区間で起きていた。とりわけ idx21-25 の5点だけで接触52件(全体の19%)あり、
// その 77%(40件)が 局面=追越中、37件が壁を巻き込んでいる。
// この5点は corridor_ten.csv 上 pass_ok=1、幅 4.6〜4.95m、
// そして 20点先に R=4.97m のヘアピンがある。
//
// 幅 4.6m に対し車体は 1.46m 幅なので、2台並ぶと片方はヘアピンの外側で
// 壁際を通ることになる。「今は幅がある」ことと「抜き切るまで幅がある」ことは
// 別で、pass_ok は前者しか見ていなかった。
//
// ここでは後者を足す。先読み点数ぶんの最小曲率半径がしきい値未満なら、
// その地点の pass_ok を落とす。0 を渡せば無効(A/B 用に元の挙動へ戻せる)。

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
      corridor_.pass_ok.push_back(true);  // 旧形式の CSV は全区間許可扱い
    }
  }
  RCLCPP_INFO(get_logger(), "corridor 読み込み %zu 点", corridor_.lo.size());
  buildRadiusMin();
  return !corridor_.lo.empty();
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

// 曲率でコリドアの余裕(壁からの追加マージン)を変える(ユーザー指示 2026-08-31)。
//   カーブ(半径 pass_margin_r_curve 以下) -> corridor_safety のまま(従来値)
//   直線(半径 pass_margin_r_straight 以上) -> pass_margin_min まで詰める
// 半径は点ごとの値をそのまま使ってはいけない。実測でヘアピン(idx160-170、
// 半径4〜18m)の途中 idx167 だけ 257m と出るため、前後 kRadWin 点の最小値
// (radius_min_)を使う。
// 【なぜ全域へ広げたか】以前は追い越し中(attempt_active_)だけ余裕を詰めていたが、
// 追い越しを開始する条件の判定は詰める前の値で行われるため
// 「緩めるには仕掛ける必要があり、仕掛けるには緩んでいる必要がある」という
// 循環になっていた(実測: lap5 で MPC 10.8km/h の後ろで idx205 余地1.10m、
// 左右どちらも 1.60m に届かず却下、追突防止で 11km/h に張り付いた)。
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

// 学習した相手のラインを使って、その地点から stretch[m] のあいだ
// 左右それぞれで確保できる横間隔の最小値を求める。
//
// 側の判断を「今この瞬間の相手の横位置」と「8m先までのコリドアの共通部分」で
// やると、ラインが振れる区間で共通部分がほぼ消え、余地の無い側に張り付く。
// 相手は毎周ほぼ同じラインを走るので、抜く区間ぜんぶを先に見て決められる。
// データの無い地点は「今の横位置がそのまま続く」とみなす。
// 返すのは「min_pass_sep 以上の横間隔を確保できる区間が、
// 連続で何メートル続くか」。区間全体の最小値ではない。
//
// 最小値で見ると、30m のどこか一点が狭いだけで側が丸ごと潰れる
// (実測 lanemap: 学習=[0.47,0.32] のような値ばかりで、
//  side_fits_ が改善せず却下件数も順位も変わらなかった)。
// 抜き切るのに要るのは「並走している間ずっと足りていること」なので、
// 連続して足りている区間の長さで判定する。
void V2XOvertaker::sideRoomMap(const OtherState & o, const Trajectory & in, size_t n,
                 size_t from, double stretch, double olat_now,
                 double & run_left, double & run_right, int & known,
                 double * room_left, double * room_right) const
{
  run_left = 0.0;
  run_right = 0.0;
  known = 0;
  // --- 「相手がどちらへ寄っているか」を録画から読む(ユーザー指示 2026-08-29)
  //
  // 連続区間(run)は「その側が成立するか」の真偽でしかない。
  // どちらから抜くかを決めるには、**空いている量そのもの**が要る。
  // 相手の録画ラインとコリドアの縁の距離を、抜き切るまでの区間で平均する。
  // 例: メインストレートで相手が左に寄っていれば room_right が大きくなり、
  // 「空いているほうから抜く」が区間の指定なしで成り立つ。
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
    double ol = o.laneLat(static_cast<int>(b * OtherState::kLatBins / n));
    if (ol > 1e8) {
      ol = olat_now;
    } else {
      known++;
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
    // 「今いる場所から連続して」足りていることを要求する。
    // 区間内の最長の窓で見ると、20m 先にある窓を根拠に今すぐ横へ出てしまう
    // (実測 lanerun: 学習連続=13〜23m と出て側はほぼ常に成立、
    //  追越失敗が 30〜35回/レースに増え、順位は 3/3/3 に落ちた)。
    // 空いている量そのもの(縁までの距離)。側の選択に使う。
    const double free_l = std::max(hi - ol, 0.0);
    const double free_r = std::max(ol - lo, 0.0);
    sum_l += free_l;
    sum_r += free_r;
    // 【追加 2026-09-04】区間の**最小**も取る。
    //
    // 【なぜ平均では駄目か(実測)】メインストレートではレースラインの位置が
    // 途中で左右に入れ替わる。corridor_ten.csv の実測:
    //   idx220-233 は左に3.0〜4.6m/右に0.35〜1.55m 空く
    //   idx  0-20  は左に0.35〜1.60m/右に3.2〜4.35m 空く
    // 平均を取ると左右が拮抗し、走る場所によって答えが反転する。
    // 実測(4レース)では、平均で側を決めた車は同じ相手に対して
    // **側の反転が21%**(4/19)起き、従来(常にイン優先で一貫)は0%だった。
    // 反転すると左右どちらにも寄り切れず、追い越しが0回になった。
    //
    // 抜き切るまで通れる側は「区間の最小の空き」で決まる。最小で見れば
    // 途中で塞がる側は最初から落ちるので、反転が起きない。
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

// ===================================================================
// 20Hz の本体。
//
// やることは「観測(Frame)を作り、層を順に呼んで指令(PlanCtx)を積み上げ、
// 最後に publish する」だけ。層の順序がそのまま優先順位になっている。
//
//   観測と記録   logDrivingStats / estimateRank / evaluateZone
//                checkPressedFromBehind / logStopCause / findFrontCar
//                learnOpponentLine          相手のライン・速度を録画する
//                assignStartSlots           グリッド番号(P1..P3)を確定する
//                planPassSpot               録画から抜きどころと側を決める
//                dumpTrace                  録画を要約してログへ
//   走り方を決める planOvertake            追う / 抜く
//   当たらないようにする
//                avoidStoppedCars         止まっている車の脇を通す
//                avoidCollision           他車と壁の回避
//                repulseFromNearCars      近接車から離れる
//   横位置を保つ  holdStartLane            スタートのレーン
//                holdSideBySide           並走中
//                holdAttemptSide          寄ると決めた側
//   最終判断     avoidWall                壁が最優先(横位置)
//                preventRearEnd           対象以外への追突を止める(速度)
//   出力         applyOffsetRateLimit / publishTrajectory / manageBoost
//
// **速度と横位置は後勝ちではなく調停で決まる(2026-09-02)。**
//   速度   : 各層が requestCap() を積み、applyCapRequests() が最小値を採る。
//   横位置 : 各層が requestLat()(意図・優先度で1つだけ採用)と
//            boundLat()(制約・区間を交差)を積み、applyLatDecision() が
//            採用した意図を制約へクランプして c.target_offset を確定する。
// 観測・記録の層と、c.avoid_offset のような中間結果の受け渡しだけが順序に依存する。
// 潰されたことを前の層へ知らせないと「避けられない位置で全開前進」になるので、
// 必要なものは PlanCtx を通して受け渡すこと(stop_avoid_active がその例)。
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

  // 直線を通しきる状態か。層をまたいで同じ値を使うので先に求める。
  straight_pass_now_ = straightPassNow(f);

  logDrivingStats(f);
  estimateRank(f);
  // 【追加 2026-09-03】全車の順位を数え直し、被追越を検出する(記録のみ)。
  trackRanks(f);
  evaluateZone(f, c);
  checkPressedFromBehind(f, c);
  logStopCause(f);
  findFrontCar(f);

  updatePenalty(f);
  learnOpponentLine(f);
  // グリッド番号の確定 -> 抜きどころの計画 -> 録画の書き出し。
  // どれも観測だけを使う(指令は触らない)。
  assignStartSlots(f);
  updateLaunchPass(f);
  planPassSpot(f);
  dumpTrace(f);

  planOvertake(f, c);

  // 【追加 2026-09-02】追い越しの状態機械を更新する。
  // planOvertake が pass_sep_ を、recordAttempt が attempt_active_ を更新する
  // ので、ここで見るのは「今周期の pass_sep_ と前周期の attempt_active_」に
  // なる(50ms の遅れ)。以降の層(停止車回避・追従・追突防止)は、この状態で
  // 「追い越し中の相手か」を判断する。
  updateOvertakeState(f, c);

  avoidStoppedCars(f, c);

  // 停止車を通す間は、通常の追い越し状態を持ち越さない。
  // 両者は横オフセットを使うが、停止車回避は安全な空き帯へ戻るため、
  // 追い越しは相手を抜き切るための指令であり、同時に保持してはいけない。
  // ここで解除してから後段の holdSideBySide / holdAttemptSide を通すことで、
  // 古い attempt_offset_ が停止車回避の横目標を一周期でも上書きしないようにする。
  // 【修正 2026-09-02】いま追い越している相手が「停止車」と判定されただけの
  // ときは中断しない。相手が遅いことは追い越す理由であって中断する理由では
  // ない。実測(3レース)で「安全中断 理由=停止車回避」が中断理由の最多
  // (1レース10件中4件)を占め、抜こうとしている当の相手を停止車として
  // 回避していた。第三者が停止している場合の中断は従来どおり残す。
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
    // 【追加 2026-09-03】試行の出口はここも含めて4か所。すべてから1行残す。
    logAttemptFunnel(f, "中断", elapsed);
  }

  avoidCollision(f, c);

  repulseFromNearCars(f, c);

  holdStartLane(f, c);

  // 【修正 2026-09-02】横位置を調停器へ統一したため無効化した。この2つは
  // 「後の層に上書きされた追越の横目標を、もう一度上書きし返す」ための
  // 対症療法であり、優先度で意図を選ぶ設計では不要かつ有害(調停の結果を
  // 後段でまた壊す)。戻せるよう関数本体は残してある。
  // holdSideBySide(f, c);

  recordAttempt(f, c);
  applyAvoidance(f, c);

  // 【修正 2026-09-02】同上。holdAttemptSide も対症療法なので無効化した。
  // holdAttemptSide(c);

  // 発進フェーズだけ横目標を最終決定する。壁だけは必ず後段に残す。
  holdGridLane(f, c);

  avoidWall(f, c);
  // 追突防止は**すべての層の後**に置く。
  // 【実測 2026-08-29】planOvertake の直後に置いていたとき、
  // 直後の avoidStoppedCars が `c.speed_cap = std::max(c.speed_cap, 3.0)` で
  // **上限を 10.8km/h へ戻していた**(ログ: 追突防止が 4.2km/h に落とした
  // 0.35秒後に 上限=10.8km/h)。速度上限を下げる層は、上げる層より後に
  // 置かなければ効かない。
  preventRearEnd(f, c);

  // 壁衝突の予測監視。すべての意図と制約が出そろった後、確定の直前に
  // 評価する(latWant() が最終値と一致するのはこの位置だけ)。
  // 出力は boundLat / requestCap のみで、意図は出さない。
  wallGuard(f, c);

  // --- BLOCK ペナルティ(20秒 5km/h 固定)を受けない ---
  //
  // 公式オーバーテイクレーンに **27km/h 未満で触れた瞬間**、
  // アタッカーがいれば BLOCK が発火する(公式FAQ)。また 27km/h 以下で
  // レーンに触れている間は3秒以内の完全退出が義務で、**加速では免除されない**。
  // 横位置は指令から約20m(25km/hで2.9秒)遅れて実現するので、
  // 「入ってから出る」は間に合わない。入らないのが唯一の正解。
  //
  // 1位はハンデで 25km/h に固定されるのでこの条件を満たせない。
  // ユーザー指摘のとおり、現状の左寄りの走りはそのおかげで BLOCK を
  // 受けていない。その性質を**速度という一つの条件で明示的に保証する**。
  // (順位推定には依存させない。推定はずれることが分かっている)
  // 【実測 2026-09-05】`ot_lane_enable=false`(レーンを追い越しに使わない)でも
  // **BLOCK が発生した**(4レース中1件)。レーンはコース上に実在するので、
  // 使わない選択をしても低速で触れれば違反になる。
  // したがってガードは「レーンを使うか」とは独立に常時効かせる。
  // 手前から効かせる。必要な先読みは「横位置が実現するまでの距離」。
  const double ot_look = ot_lane_guard_look_ +
                         std::max(my_speed_for_gap_, 0.0) * ot_lane_guard_time_;
  if (ot_lane_guard_ && !ot_lane_zones_.empty() && otLaneAhead(f.ei, ot_look) &&
      my_speed_for_gap_ * 3.6 < ot_lane_min_kmh_)
  {
    const double before = c.latWant();
    c.boundLat(-ot_lane_guard_lat_, 1e9, "追越レーン(低速で入らない)");
    const double after = c.latWant();
    if (std::abs(before - after) > 0.05 &&
        (now - last_ot_lane_log_).seconds() > 1.0)
    {
      last_ot_lane_log_ = now;
      diagLog("追越レーン",
              "追越レーン 低速で入らない 自車=%.1fkm/h(要%.0f) idx=%zu "
              "横目標 %.2f -> %.2f",
              my_speed_for_gap_ * 3.6, ot_lane_min_kmh_, f.ei, before, after);
    }
  }

  // --- 横位置の確定。ここまでに積まれた意図と制約を調停する。
  // 以降(レート制限・publish・ログ)は確定した c.target_offset を読むだけ。
  c.applyLatDecision();
  if ((now - last_lat_log_).seconds() > 2.0) {
    last_lat_log_ = now;
    diagLog("横位置", "横位置 %.2fm 意図=%s(%.2fm) 範囲=[%.2f,%.2f] 縛り=%s",
                c.target_offset, c.lat_intent.why, c.lat_intent.v,
                c.lat_lo, c.lat_hi, c.lat_bound_why);
  }

  applyOffsetRateLimit(c);
  publishTrajectory(f, c);

  manageBoost(f, c);

  publishStatus(f, c);

  logBlocker(f, c);
  // 【追加 2026-09-03】接触の要因を残す。すべての層の後に置き、
  // 確定した横位置の意図(c.lat_intent.why)と blocker を読むだけ。
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

  // --- 溜めた走行データの要約を定期的に出す ---
  // 「遅い相手かどうか」を推測ではなく実測で判断できるようにする。
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
    if (sp > 0.5 && sp < 30.0) { my_speed_sum_ += sp; my_speed_cnt_ += 1; }
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
        // 生の弧長 s をそのまま初期値にすると、s=0/total の継ぎ目を挟んで
        // 初観測された車が永久に約1周ぶんずれ、順位が固定でおかしくなる
        // (実測: いつも3位のまま)。自車と同じ基準に載せて初期化する。
        // 前提: 初観測の時点で相手は半周以内にいる(レース開始時は必ず成立)。
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
    // スタート時の順位を1度だけ確定させる。
    //
    // 合流完了(70m 走行)まで待つ設計にしていたが、その時点では既に
    // 順位が入れ替わっており、2位スタートの車が「1位」と判定されて
    // ブーストを温存してしまった(実測)。
    // グリッド位置がそのまま順位なので、レース開始の合図を受けた
    // 時点で確定させる。
    if (start_rank_ == 0 && race_started_) {
      start_rank_ = new_rank;
      RCLCPP_INFO(get_logger(),
                  "スタート順位 %d位%s", start_rank_,
                  start_rank_ >= 2 ? " -> 序盤にブーストを1つ使う"
                                   : " -> 前が空いているのでブーストは温存");
    }
  }

}

// ===================================================================
// 【追加 2026-09-03】全車の順位を数え直し、被追越を検出する。
//
// **観測専用。** rank_ / ov_state_ / pass_sep_ など制御が読む値には
// 一切書き込まない。書き込むのは rank_of_ / my_rank_obs_ / ovt_ /
// spd_hist_ / last_overtaken_t_ だけで、これらを読むのはログだけ。
//
// 順位は estimateRank と同じ考え方で累積進行度 prog の降順にする。
// prog は周回をまたぐたびに周回長ぶん増えるので、
// 「周回数 -> 周回内の進行度」の辞書式降順と同じ順序になる。
// ===================================================================
void V2XOvertaker::trackRanks(const Frame & f)
{
  const double total = f.total;
  const double t = f.now.seconds();
  const rclcpp::Time tnow = this->now();

  // --- 相手の速度履歴(0.5秒 = 20Hz で10点)。接触時の「相手急減速」用。
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

// ===================================================================
// 【追加 2026-09-03】接触したときの要因を1行で残す。
//
// **観測専用。** 既存の logStopCause(停止から壁/車両を推定する既存ロジック)は
// 一切変更していない。こちらは「速度が急に落ちた」または「車体が走行可能領域へ
// 食い込んだ」を独立に検出し、そのときの文脈だけを書き出す追加の記録である。
// PlanCtx は読むだけで書き換えない。
// ===================================================================
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

// 現在地から dist[m] 先までの、コリドアの最小幅[m]を返す。
//
// 【なぜ要るか】evaluateZone が出す c.avail_width は look_width_ahead_(既定20m)
// の最小幅で、「いま横に出られるか」を見るには正しい。しかし追い越しが成立
// するかは「抜き切るまでのあいだ2台が並べるか」で決まり、その距離は 20m より
// ずっと長いことがある(速度差が小さいほど長い)。
//
// 【実測(2026-09-03、自コード4台×11レース、接触270件)】
// idx21-25 の5点だけで接触52件(全体の19%)、うち 77% が 局面=追越中。
// この5点の 20m 先までの最小幅は 4.55m あるが、34m 先まで見ると 3.90m、
// 41m 先では 3.45m まで落ちる。カート2台は 1.46m x2 = 2.92m なので、
// 3.45m では左右の余裕が合計 0.5m しかない。
// **「今は幅がある」ことと「抜き切るまで幅がある」ことが別**だったのが、
// この場所で壁に当たり続けていた理由。
//
// 曲率で場所を名指しして禁じるのではなく、既にある幅の判定を
// 正しい区間へ伸ばすことでこれを扱う。
// いまの横位置を保ったまま dist[m] 進んだときの、走行可能領域の縁までの
// 最小余裕[m]。負なら縁を割る。
//
// 【なぜこの量か】ユーザー指示: 「そのまま進んだら**自分が**壁にぶつかるなら
// 試行をやめる。ぶつかるのが相手なら、そのまま並走して抜き切る。」
// 追い越しをやめる条件は「相手より遅い」でも「ゾーン外」でもなく、
// **自分が壁に当たるか**だけにする。
//
// corridor_.lo/hi は make_corridor.py が壁から車体半幅0.73+余裕0.45を
// 引いた線なので、ここが 0 を割ることは「車体が壁に触れうる」を意味する。
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
      // ゾーンの検出だけは遠くまで見る。
      // 直線に入ってから横に出始めると、抜き切る前に直線が終わってしまう。
      // 直線の手前で「この先に並走できる区間がある」と分かっていれば、
      // 進入前から横へ動き出して直線をフルに使える
      // (ユーザー報告:「直線で抜き始める判断が遅く、ぎりぎりで抜かすことになっている」)。
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
      // 「幅の合計」だけ見ても、走行ラインが片側に寄っている区間では
      // 出られる側が足りない。先読み区間で左右それぞれの余地を別に求める。
      // これを見ないと、右に 0.9m しか無い場所へ 1.7m 寄せようとして壁に当たる
      // (ユーザー報告:「抜こうとしたら壁にぶつかりタイムロスしている」)。
      // ただし余地を求める範囲は短くする。
      // look_width_ahead_(20m) 全体の共通部分を取ると、走行ラインが
      // コリドア内で左右に振れる区間で共通部分がほぼ消え、
      // 幅 4.5〜5.5m あるのに出られる量が ±0.25m になっていた
      // (実測: closing 18.7km/h・所要5.7s で抜けるはずの場面が side_fits_=false で却下)。
      // 実際には少し先まで確保できていれば足り、車が進めば毎周期引き直される。
      // 仕掛けている最中は縁までの余裕を削る。詰まったまま走り続ける損失
      // (実測: race2 の 66% を MPC の後ろで消費)のほうが、
      // 0.2m ぶん縁に寄る危険より大きい。試行していない間は元の余裕のまま。
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
    }
  }
  // 相手が極端に遅い(止まっている・壁に当たっている)ときは、
  // ゾーン外でも幅さえあれば抜く。壊れた車の後ろで待ち続けるのは損なので。

  my_speed_for_gap_ = odom_->twist.twist.linear.x;

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
        break;
      }
    }
  }

}

// 停止したとき、原因が壁か他車かを判定してログに出す。
// 【注意】これは自作の推定であって公式ペナルティではない(開発メモ)。
void V2XOvertaker::logStopCause(const Frame & f)
{
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // --- 停止したとき、原因が壁か他車かを判定してログに出す
  // 壁(Wall 5秒)と車両接触(Crash 10秒)は罰則も対策も違うので分けて数える。
  {
    const double sp = std::abs(odom_->twist.twist.linear.x);
    // 「止まった = 接触」ではない。
    //
    // この判定は接触を一切見ておらず、速度が落ちたことと
    // 近くに他車がいるかどうかだけで種別を決めていた。そのため
    // 「他車32m先・コリドア内側・横-0.40m」で減速しただけの場面が
    // 壁接触として記録され、その数字を根拠に対策を3つ試して
    // 4.5秒以上を無駄にした。
    //
    // 本物の接触なら、止まった場所で車体が壁の近くにあるはず。
    // 走行可能領域の内側で十分な余裕がある場所での停止は接触ではない。
    bool slowing_for_corner = false;
    if (corridor_.lo.size() == n) {
      const double lo = corridor_.lo[ei], hi = corridor_.hi[ei];
      const double lat = my_lat_for_target_;
      // 左右どちらの境界からも余裕があるなら、壁には当たっていない
      if (lat > lo + kContactMargin && lat < hi - kContactMargin) {
        slowing_for_corner = true;
      }
    }
    // スタート前のグリッド待機を接触として数えない。
    // 実測で、記録された「壁接触」が idx=0 位置=(89629.1,43131.4) 横=0.00、
    // つまりスタートライン上の停止だった。これを数えていたため
    // 壁接触の集計に常時 1〜2 回の偽陽性が乗っていた。
    if (!race_started_) {
      stall_since_ = this->now();
      stall_logged_ = false;
    } else if (sp < 0.4 && !slowing_for_corner) {
      const rclcpp::Time tn = this->now();
      // 復帰動作は後退 -> 前進を繰り返すので、そのたびに sp が 0.4 を超えて
      // stall_logged_ が落ちる。同じ 1 回の接触が 4〜5 回数えられていた
      // (実測: 壁 9 回のうち 4 回は 1 回のもがきの再カウント)。
      // 直前のログから contact_log_hold_ 秒は同一の接触として数えない。
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

// 相手の走行ラインを学習する(前方かどうかに関係なく毎周期)。
// あわせてブースト要求を毎周期作り直す。
void V2XOvertaker::learnOpponentLine(const Frame & f)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double total = f.total;
  const rclcpp::Time now = f.now;

  // --- 相手の走行ラインを学習する(前方かどうかに関係なく毎周期)
  // 相手がどの地点でどれだけ横にいるかを覚えておき、側の判断に使う。
  // 前方車のループの中でやると、抜いた後・離れている間のサンプルが
  // 取れず、次に追いついたときにデータが無い。
  if (lane_learn_ && n > 0) {
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
      os.noteLat(lbin, llat);
      // 横位置と同じ地点分解能で**速度も**録る(ユーザー指示の「録画」)。
      // 抜きどころを決めるとき、相手が遅い場所ほど詰めやすいため。
      {
        const double osp = std::hypot(os.vx, os.vy);
        if (osp > 0.3 && osp < 30.0) { os.noteSpd(lbin, osp); }
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


// ===================================================================
// スタートのグリッド番号を確定する(ユーザー指示 2026-08-29)
// ===================================================================
//
// 自分と**相手の**グリッド番号(P1..P3)を、記録済みのグリッド座標との
// 照合でレース開始時に1度だけ決める。
//
// なぜ相手の番号まで要るのか:
//   - 「1周目は運営NPC(最前列 P3)以外を抜かない」
//   - 「P1 のとき僚車(P2)を抜き始めるのは3周目以降」
//   - 「P1 は P2 が少し前に出るのを待ってから発進する」
// のすべてが「どの車がどのグリッドから出たか」を必要とする。
//
// 進行度順に並べる方法(holdStartLane の従来実装)は V2X の到着に依存して
// 崩れるうえ、**自分が動き出すまで実行されない**ので発進制御には使えない。
// グリッドは毎回同じ場所なので、座標の照合が確実で、しかも停止中に決まる。
void V2XOvertaker::assignStartSlots(const Frame & f)
{
  if (!race_started_) { return; }
  if (race_start_time_ < 0.0) {
    race_start_time_ = f.now.seconds();
    launch_since_ = race_start_time_;   // 発進計測ログの基準時刻
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
  // 【追加 2026-09-03】台数が grid_slots の登録数を超える構成(自コード4台など)
  // では、固定座標との照合が必ず破綻する(4台目に対応する座標が無く、既に
  // 使われたスロットへ二重に割り当たる)。照合できないときは、レース開始時の
  // 進行度順で決める。P1 が最後尾、進行度の小さい順に P1, P2, ... とする
  // (grid_slots のコメントにある実測 P1=329.2 / P2=330.4 / P3=333.5 と同じ並び)。
  const std::size_t car_n = 1 + others_.size();
  const bool grid_usable =
    !grid_slots_.empty() && grid_slots_.size() >= car_n && my_d < 3.0;
  // 【修正 2026-09-03】以前は waited>=2.0 を必須にしていたため、下の照合経路が
  // それより先に確定してしまい、このフォールバックへ到達しなかった。実測では
  // 4台構成で d4 が P0(未割当)のまま確定していた。台数がグリッド登録数を
  // 超えている場合は、V2X が揃い次第すぐこちらを使う。
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
    // 後で grid_slots へ固定値として書けるよう、実測の開始座標も出す。
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

  // 【修正 2026-09-03】待機の条件を「登録されたグリッド数」ではなく
  // 「いま居る台数」にする。登録が4なのに3台で走ると永久に ready にならず、
  // 2秒待ってから確定する遅延が毎回入っていた。
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


// 【削除(ユーザー指示 2026-08-29)】
// P1 が「P2 が少し前に出るまで待ち、右へ切って P3 を避ける」発進制御は
// 実走で機能しなかったため削除した。残していた実装は次の3つ。
//   - manageLaunch      : P2 の進行度差を見て発進を待ち、右へ寄せる層
//   - start_path        : P1 のスタートだけ経路を斜めに作る(publishTrajectory)
//   - holdStartLane の P1 分岐 : 保持する横位置を P2 側へ書き換える
// いずれも「グリッドの横位置をそのまま保つ」という基本の挙動を上書きしていた。
// 現在は全車が自分のグリッドの横位置を保ったまま発進する
// (その横位置をコリドアで丸める処理は残してある。固定値 0.9 で丸めていたのが
//  P2 が左へ寄ってしまう原因だった)。
// 直線の追い越しを「通しきる」状態か。
//
// 【なぜラッチが要るのか(実測 2026-08-29)】
// 区間(idx220-25)の判定をそのまま使うと、**出口で免除が一斉に切れる**。
//   壁の余裕 0.25 -> 0.65m / 先で狭くなる側のクランプが有効化 /
//   追突とみなす横間隔 1.35 -> 1.80m
// 並走したまま高速でこれが起きるので、横目標が内側へ飛んで相手に当たる。
// 実測では d2 が idx26 で Crash、その直後に d1 が idx32 で Wall(玉突き)。
// 区間の中で仕掛け始めたら、**その仕掛けが終わるまで**は同じ扱いを続ける。
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

// いま出ている横位置が、この先 ahead[m] の間も帯の中に収まるか。
//
// 【なぜ要るのか(実測 2026-08-29、再現する Crash)】
//   750.5  抜き切りへ target=d3 車間=3.8m 横間隔=-1.30m 上限解除 idx=18
//   752.7  追突防止 d3 まで 1.4m 横間隔 1.09m
//   753.3  壁回避 横目標 -0.20 -> 0.00 (壁側=右 前に相手あり)
// idx18 で右へ 1.30m 出て抜き切りに入ったが、そこは**右側が閉じ始める地点**
// (右の幅 4.10m@idx18 -> 0.75m@idx26)。壁に押し戻されて横間隔が
// 車幅(1.30m)を割り、車間 1.4m で追突した。
// **抜き切り(速度上限を外す)に入る前に、その横位置を保てるかを見る。**
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

// いまの地点が「側を録画で決める区間」の中か(ユーザー指示: idx220->25 の直線)。
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

// いまいる「直線」の終わりまでの距離[m]。区間の外にいるなら負を返す。
//
// 直線の中で抜き切れるかを判定するために要る。
// 【実測 2026-08-29】直線(idx232-17)で並びかけ、抜き切れないまま
// コーナー入口(idx18以降、右の幅が 4.10m -> 0.75m へ縮む)に
// 並走のまま入って相手の正面へ押し込まれる、という接触が繰り返し起きた。
// 抜き切れないなら、そもそも横に出ないほうが速い。
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

// その区間で相手が**平均してどちらに寄っているか**を録画から求める[m]。
// 正が左、負が右。データが足りなければ 1e9 を返す。
//
// ユーザー指示(2026-08-29): 「相手の平均して寄っている方の反対側から抜く。
// 同じぐらいだったら右側から抜く」。瞬間の横位置ではなく区間の平均で見るのは、
// 相手がラインを横切っている一瞬を捉えて逆側を選ばないようにするため。
double V2XOvertaker::zoneMeanLat(const OtherState & o, std::size_t n, int & known) const
{
  double sum = 0.0;
  known = 0;
  if (n == 0) { return 1e9; }
  for (int b = 0; b < OtherState::kLatBins; ++b) {
    const std::size_t idx = static_cast<std::size_t>(b) * n / OtherState::kLatBins;
    if (!inSidePickZone(idx)) { continue; }
    const double l = o.laneLat(b);
    if (l > 1e8) { continue; }
    sum += l;
    known++;
  }
  if (known < lane_map_min_pts_) { return 1e9; }
  return sum / known;
}


// ===================================================================
// 相手の走りの録画から「どこで・どちら側から抜くか」を決める
// (ユーザー指示 2026-08-29)
// ===================================================================
//
// 【なぜ地点を決めるのか】
// 従来は「いま抜けるか」を毎周期その場で判定していた。そのため
//   - 幅が足りない場所で仕掛けては降りる、を繰り返す
//   - 仕掛けどころに着いたときには車間が空いている
// という失敗が多かった(実戦: 試行18〜20回に対し抜き切り13〜18回)。
//
// 相手(NPC も僚車も)はほぼ同じラインを毎周なぞる。1周目に録った
// 「地点ごとの横位置と速度」があれば、**2周目以降は先に地点を決められる**。
// 決まっていれば、そこへ着くまでに加速しておき、入口でちょうど
// 車間が縮まっているように逆算できる(followAndCommit の助走)。
//
// 【選び方】
//   ある地点で相手の録画ラインが分かれば、コリドアの縁との差が
//   「その側に自分が入れる余地」になる。余地が spot_margin 以上ある点が
//   連続して続く長さを左右それぞれで測り、
//     点数 = 連続長 + 相手が遅いほどの加点
//   が最大の区間を選ぶ。遠い候補は距離で割り引く(手前を優先)。
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

  // 一度作った時空間計画は、入口を通過するか対象が前方から消えるまで保持する。
  // 旧実装は1秒ごとに target/side/入口を選び直したため、ログ上でも同じ試行中に
  // 左右が交互に現れた。予測を行動へ使うには、横移動・加速・抜き切りが同じ
  // 仮説に従う必要がある。
  if (spot_valid_ && slots_assigned_ && start_slot_ == 1) {
    // 計画後に別車が前へ割り込んだ場合、奥の対象がまだ前方にいるという理由で
    // 古い計画を保持しない。実走では spot=d3 / blocker=d2 のまま入口を通過し、
    // どちらにも仕掛けられなかった。
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

  // submit_11 の公式結果では P2 は 5戦5勝だった一方、唯一の敗戦は P1。
  // 新しい時刻同期型の抜きどころ計画を P2 にも適用すると、勝っていた判断を
  // 不要に変えてしまう。さらに公式第6戦ではグリッド確定の約0.7秒前に P2 を
  // 対象とした誤試行が始まっていた。スロットが確定するまで計画を作らず、
  // 現段階では問題が再現している P1 のみに限定する。
  if (!slots_assigned_ || start_slot_ != 1) { return; }

  const Trajectory & in = f.in;
  const size_t n = f.n;
  const size_t ei = f.ei;
  if (corridor_.lo.size() != n || corridor_.hi.size() != n) { return; }
  // 【修正I 2026-09-02】候補の幾何判定を band に揃える。band が使えるときは
  // band_lo_ex_/band_hi_ex_ を使う(下の geom_l/geom_r 算出箇所を参照)。
  // 【修正K 2026-09-02】計画も除外 band(追い越し対象を削らない band)に揃える。
  // spotPathSafe と同じ幾何を使うことで、計画と検査の不一致(自己矛盾)を防ぐ。
  const bool use_band_spot =
      band_enable_ && band_lo_ex_.size() == n && band_hi_ex_.size() == n;

  // 対象は「自分の前にいて、いちばん近い1台」。
  //
  // 【直したバグ(実測 2026-08-29)】前後の判定に累積進行度の差
  // (o.prog - my_prog_)を使っていた。累積進行度は周回を跨ぐたびに
  // 周長ぶん増えるので、**1周差がつくと 5m 前の車が -700m 後ろ**と出る。
  // その結果、車間 5.6m の相手が目の前にいるのに対象が1台も選ばれず、
  // 抜きどころが最後まで決まらなかった(ログ: 抜きどころ なし ... 学習点0)。
  // 周回に依らない「経路上の前方距離」で見る(isNearestBlocker と同じ)。
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
    // 実行段で禁止される相手を先に計画ロックすると、その車より前にいる
    // NPCへの追越しまで spot_target 不一致で塞いでしまう。計画対象と実行対象は
    // 必ず同じ周回ルールに従わせる。速度不足の例外は実測が安定してから
    // 実行段で判定するため、ここでは保守的に false を渡す。
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

  // 相手が各地点へ着く時刻を、録画した地点別速度で積分する。
  // 旧実装は候補地点までの自車距離だけを見ており、相手がそこを通過する
  // 時刻との対応が無かった。そのため「幅は広いが、着いた頃には相手がいない」
  // 地点を抜きどころに選び、そこまで追従するだけになっていた。
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
      const int bin = static_cast<int>(cur * OtherState::kLatBins / n);
      double vop = tgt->laneSpdProvisional(bin);
      const bool have_learned_v = vop > 0.0;
      if (!have_learned_v) { vop = (op_mean > 0.0) ? op_mean : std::hypot(tgt->vx, tgt->vy); }
      // 【修正O 2026-09-02】O-1 と同じ理由。相手の到着時刻も地点別の学習速度で
      // 積分する。平均で床を張ると、相手が遅い地点の到着時刻が実際より早く
      // 見積もられ、そこを追い越し候補にできなくなる。
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
               // 【修正 2026-09-02】1点でも幾何条件を割ると区間を閉じていたため、
               // 本来つながっている空きが細切れになり、必要長18mに対し18m以上の連続
               // 区間はコース上に 2〜9% しか存在しなかった(実測 20260902-133252)。
               // spot_run_gap 以下の短い狭窄はまたいで1本の区間として扱う。実行時の
               // 横位置は band へクランプされるので、狭窄部で帯からはみ出すことはない。
               double bad_len{0.0}; };
  Run cur_l{-1.0, 0.0, +1.0, 0.0, 0, 0.0, 0.0, 0.0, 0, 0};
  Run cur_r{-1.0, 0.0, -1.0, 0.0, 0, 0.0, 0.0, 0.0, 0, 0};
  double best_score = 0.0;
  Run best{};
  bool have_best = false;

  auto close_run = [&](Run & r) {
    // 【修正 2026-09-02】狭窄をまたいで継続した区間を閉じるときは、末尾に
    // 積んだ bad_len(狭窄分)を長さから差し引く。またがずに閉じる通常の
    // 経路では bad_len は 0 のまま(既存挙動を変えない)。
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
      // 実行段と同じだけ完全に前へ出ることを要求する。以前は pass_len の
      // 半分しか要求せず、計画では成立しても実走では16秒打切りになっていた。
      // 入口で相手と同一点に同期する計算では、実車が保持すべき縦車間が消える。
      // 横へ出る前の安全車間も取り返してから pass_len だけ前へ出られる区間を選ぶ。
      // --- 抜き切るのに要る「相対的な前進量」 ---
      //
      // 内訳は3つ。
      //   (a) 到着のずれを埋める分   arrival_delta * 相手速度
      //   (b) 横へ出る前に取り返す縦の安全車間   spot_entry_gap_
      //   (c) 相手の前へ出切る分     pass_len_ * spot_pass_len_gain_
      //
      // **この値は大きすぎても小さすぎても追い越しを壊す。**
      //   大きい → 必要距離が伸び、連続区間に収まらず「抜きどころなし」で
      //            一度も仕掛けられなくなる(実測: 先頭を12レースで0回しか抜けていない)
      //   小さい → 抜き切る前に区間が終わり、並走したままコーナーへ入って接触する
      //
      // 実測(2026-09-04)の目安:
      //   既定 spot_entry_gap=4.5(= rear_end_margin 3.5 + 1.0) / pass_len=4.0
      //   → need_gain = 8.5m。先頭(25km/h)に対し自車上限36km/h なら
      //     速度差 3.06m/s で 2.8秒、その間に自車は約28m進む。
      //     さらに spot_need_margin(1.15)を掛けて約32m。
      //   一方、学習から得られる連続区間は実測で17〜20m しかない。
      //   **つまり既定値では構造的に足りない。** ここを詰めるための調整幅を出す。
      //
      // spot_entry_gap を負にすると従来どおり rear_end_margin_+1.0 を使う。
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
      // 【観測 2026-09-04】「抜きどころ なし: 連続区間 左20m 右17m (要8m)」という
      // ログが出るのに候補が無い、という状態を読み解けなかった。
      // 表示していた「要8m」は spot_min_len_ という最低長でしかなく、
      // 実際に効いている条件はこの required_distance(抜き切るのに要る距離)。
      // どれだけ足りないかを残す。これが分からないと、
      // パラメータで届く距離なのか構造的に無理なのかを区別できない。
      // 安全率。1.0 で「計算どおりぴったり」、大きいほど余裕を要求する。
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

    // この地点を相手が先に通過してから何秒後に自車が最速で着くか。
    // 遅着は4秒までだが、早着は加速を待てば同期できる。二分探索の上限と同じ
    // 20秒まで許す。旧条件(-0.5秒)は速い自車ほど抜きどころを失う逆判定だった。
    const double arrival_delta = self_arrival - op_arrival[b];
    const bool after_opponent = acc + 0.5 >= best_ahead;
    const bool interceptable = after_opponent && op_arrival[b] < 1e17 &&
                               arrival_delta >= -20.0 && arrival_delta <= 4.0;

    const int bin = static_cast<int>(b * OtherState::kLatBins / n);
    // 1周目の観測も候補探索には使う。最終的な側判定や衝突バンドは従来どおり
    // 3サンプル値を使い、ここでは15m連続条件で単発ノイズを排除する。
    double ol = tgt->laneLatProvisional(bin);
    const double ov = tgt->laneSpdProvisional(bin);
    // 【修正O 2026-09-02】ここも学習した地点別速度を周回平均で床張りして
    // いた。相手が自分の平均より遅い地点(=抜くべき場所)の情報が消え、
    // required_distance が過大になって候補が落ちる。この値は vsum/vcnt を
    // 経て候補の採否そのものを決めるので影響が大きい。O-1/O-2 と揃える。
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
      // 横速度の直線外挿は遠方で暴れるため2秒で頭打ちにする。2秒あれば
      // MPCのレーン移動(実測1～1.5m)は捉えられ、その後は到達位置を維持する。
      const double t_lat = std::min(op_arrival[b], 2.0);
      ol = olat_now + ovlat_now * t_lat;
    }
    // 直前に「着いたのに仕掛けられなかった」地点は、しばらく選び直さない。
    // これが無いと同じ死んだ地点を選び続けて前へ進めない(2026-09-02 修正D-3)。
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
      // 【修正 2026-09-02】候補の幾何判定を band に揃えた。
      // 従来はコリドア(+safetyAt)と spot_margin で候補を選ぶ一方、実行直前の
      // spotPathSafe() は band の幅で検査していた。band は相手の占有分を削り、
      // 壁余裕を引き、時間平滑化もされているため構造的にコリドアより狭く、
      // 計画器が選んだ地点を検査が落とすのは必然だった(実測 20260902-142620:
      // 抜きどころ到着13回のうち9回が「予測経路が閉塞」で却下、追越試行0回)。
      // 計画と検査で同じ幾何を使えば、この不一致は構造的に起きない。
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
      // 【修正 2026-09-02】短い狭窄(spot_run_gap_以内)はまたいで区間を継続する。
      // 狭窄部分も len に含める(実行時は band クランプが吸収する)。
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
      // 【修正 2026-09-02】短い狭窄(spot_run_gap_以内)はまたいで区間を継続する。
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
bool V2XOvertaker::spotPathSafe(
  const Frame & f, const OtherState & o, double ahead) const
{
  if (!spot_valid_ || ahead <= 0.0) { return true; }
  (void)o;
  // 【修正K 2026-09-02】追い越し対象を除いた band を使う。対象を含む band で
  // 検査すると「今から抜く相手を含めて道が空いているか」という自己矛盾になり、
  // 相手が狭い区間にいる限り必ず閉塞と判定されていた。対象との横間隔は
  // planPassSpot の spot_margin_ が担保する。第三者車と壁は従来どおり効く。
  const bool have_band = band_enable_ && band_lo_ex_.size() == f.n && band_hi_ex_.size() == f.n;
  const bool have_corr = corridor_.lo.size() == f.n && corridor_.hi.size() == f.n;
  if (!have_band && !have_corr) { return true; }

  // 【修正J 2026-09-02】落ちた理由を後から追えるよう診断値を残す。
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
    // 【修正 2026-09-02】固定線 spot_offset_ が band に入っているかではなく、
    // 「計画した側に車1台ぶん通れる帯が続いているか」で判定する。
    // planPassSpot は corridor と spot_margin_ から線を決めるが、band は
    // 他車の予測を毎周期織り込んで変わるため基準が違い、実際には通れる場所でも
    // 線が外れて閉塞になっていた(実測 20260902-131831: 到着時却下の最多が
    // 「予測経路が閉塞」7件、うち6件は band 側と計画側が一致していた)。
    // band は相手側を削った残りの通行可能区間なので、その幅が通行可否を表す。
    // 実行時の横目標はどのみち band へクランプされる(offs[i] の clamp)。
    const double width = std::max(hi - lo, 0.0);
    spot_path_min_w_seen_ = std::min(spot_path_min_w_seen_, width);
    const double kept = std::clamp(planned, std::min(lo, hi), std::max(lo, hi));
    const bool line_ok = std::abs(kept - planned) <= spot_path_tol_;
    const bool width_ok_here = width >= spot_path_min_w_;
    if (!line_ok && !width_ok_here) {
      bad_len += ds;
      if (bad_len >= spot_path_bad_len_) {
        spot_path_fail_at_ = acc;
        spot_path_fail_w_ = width;
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

// 追越可能区間の出口までに完全に前へ出られる「最も遅い全開開始時刻」を求める。
// 自車と相手を0.1秒刻みで進め、時刻を二分探索する。負値は今すぐ全開でも
// 抜き切れないことを表す。これにより、瞬間速度差から距離を一度だけ割る旧判定でなく、
// 地点ごとの速度上限・相手の学習速度・自車加速を区間全体にわたり評価する。
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
      const int obin = static_cast<int>(oi * OtherState::kLatBins / f.n);
      double learned = o.laneSpdProvisional(obin);
      const bool have_learned = learned > 0.0;
      if (!have_learned) { learned = (op_mean > 0.0) ? op_mean : op_now; }
      // 【修正O 2026-09-02】学習した地点別速度を主に使う。以前は
      // max(learned, op_mean, op_now*0.9) としていたため、相手が自分の周回平均
      // より遅い地点(＝コーナー＝抜くべき場所)の学習値が全部平均で上書きされ、
      // 地点別に学習している意味が消えていた。追い越しの機会そのものを捨てる
      // 判定になっており、実測で抜きどころ到着後の却下理由の最多が
      // 「今から加速しても区間内に抜けない」になっていた(20260902-184411)。
      // buildBand は同じ学習値をそのまま使っており(min(learned_v, cap_op))、
      // 計画・シミュレーションだけが別基準だったので揃える。
      // 安全率は predict_op_margin(既定1.10)で明示的に持つ。
      double op_target = learned * predict_op_margin_;
      // 学習値が無い地点だけは、瞬間値でも下振れしないよう従来の下限を残す。
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


// ===================================================================
// 相手の走りの録画をログへ書き出す(ユーザー指示 2026-08-29)
// ===================================================================
//
// 走行中の判断は上の planPassSpot がメモリ上の録画から直接行う。
// ここで出すのは**後から人が読むため**の要約。
// 走行後に「どこで相手が遅いのか」「どちら側が空いているのか」を
// 実測で確かめられるようにしておく。ログは output/<日時>/dN/autoware.log に残る。
void V2XOvertaker::dumpTrace(const Frame & f)
{
  if (!race_started_) { return; }

  // --- 公式ペナルティの検出 ---
  // AWSIM はペナルティ中、速度を **1.38889 m/s (5km/h) に固定**する
  // (Assembly-CSharp.dll の VehiclePenaltyController)。トピックは無いので
  // 速度が貼り付いている時間で種別が分かる:
  //   10秒 = Crash(自分の前で当てた) / 5秒 = Wall / 2秒 = Over
  // 3台走行では公式の結果ファイルが出ないので、これが唯一の確実な指標になる。
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

  // --- スタート直後の計測(ユーザー報告の切り分け用) ---
  // 「P1 がすぐ動き出せない」「P2 が P3 の後ろへ行く」が
  //   (a) 自分が速度を落としているのか
  //   (b) そもそも AWSIM が車を放していないのか
  // をログだけで判別できるようにする。合図からの時刻・自車速度・
  // 各車の速度をそろえて出す。
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


// ===================================================================
// 周回による追い越しの解禁(ユーザー指示 2026-08-29)
// ===================================================================
//
//   1周目 : 相手の走りを録るために、運営NPC 以外は抜きにいかない。
//           ただし「相手が想定より遅かったら抜いてよい」(ユーザー但し書き)
//           ので、実測で明らかに遅い相手・止まっている相手は対象外。
//   P1のとき: 僚車(P2)を抜き始めるのは3周目以降。
//
// 目的は「録画のための1周を確保する」ことと、
// 同速の僚車に序盤から仕掛けて接触・時間を失うのを防ぐこと。
// 実測では同速の相手への試行は 46〜90m 必要で、事実上成立しない。
// ペナルティ中かどうかを速度から推定する。
// AWSIM は違反時に「その秒数だけ最高速度を 5km/h に固定」する。通知は無い。
// 5km/h ちょうどに張り付いている状態が pen_hold_ 秒続いたら中とみなす。
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
        // 【追加 2026-09-03 実測】継続時間から種別を判別する。
        // AWSIM の VehiclePenaltyController は種別ごとに固定秒数だけ 5km/h に
        // 制限する: Crash 10.0s / Wall 5.0s / Over 2.0s。
        // 実測 694 件の継続時間は 2 / 5 / 10 秒付近に明確に分かれ、この値と一致した。
        // 1.5 秒未満はコーナーやスタートで一時的に 5km/h 付近を通過しただけの
        // 誤検出(実測で全体の 42%)なので、ペナルティとして数えない。
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
  // --- 先頭を抜くのは終盤だけにする ---
  //
  // 【実測 2026-09-04】1台のみ・handicap ON で走らせると、自車の速度は
  // 中央値24.7 / p90 24.9 / **最大25.4 km/h** で頭打ちになる。
  // 同じコードを handicap OFF(公式 make eval)で走らせると
  // ラップ35.04s = 平均34.3km/h、ペナルティ0件。
  // **1位は25km/hに固定される**というハンデが、実測で確認できた。
  // 4台レースのラップ中央値46.1秒は、334m を 25km/h で回る48.1秒とほぼ一致する。
  // つまり全車が先頭の25km/hに合わせて走らされている。
  //
  // したがって**先頭に立つことは不利**である。前に出た瞬間に自分が25km/hになり、
  // 後ろは36km/hのまま(実測で自車は最大37.7km/hまで出る)なので抜き返される。
  // このコードには既に同じ理屈がコメントとして書かれていた:
  //   「僚車間の逆転6件のうち、5周目に前へ出た4件は4件とも抜き返され、
  //     6〜7周目に出た2件は2件とも守り切った」
  // しかし判定は `o.slot != npc_slot_`(＝僚車かどうか)で書かれており、
  // **本番の相手は他チームなので slot では判別できない**。
  // 判定を物理的な条件、すなわち「その相手が現在の先頭か」に置き換える。
  //
  // 待っても順位は変わらず、待つあいだ自分は先頭と同じ速度で走れるので損が無い。
  // ただし明らかに遅い相手(壊れた車・停止車・別実装の低速車)は待たない。
  // その場合は待つほど自分が遅くなり、ハンデの理屈が成り立たない。
  if (leader_pass_last_laps_ > 0 && !clearly_slower &&
      !cur_leader_.empty() && name == cur_leader_ &&
      lap_ < race_laps_ - leader_pass_last_laps_) {
    return false;
  }

  // ここから下は「最初の数周は仕掛けない」ための周回ゲート。
  // 上のハンデの規則とは目的が別なので、こちらだけを無効にできる。
  if (!lap_gate_enable_) { return true; }
  // V2X の最初の数サンプルでは相手の slot は 0、速度平均もほぼ 0 になる。
  // 公式第6戦ではこの瞬間に P2 を「明らかに遅い車」と誤認し、グリッド確定前
  // から追越試行を開始した。確定は通常1秒未満なので、ここで待つ損失よりも
  // 誤った相手・側へコミットする損失の方がはるかに大きい。
  if (!slots_assigned_) { return false; }
  // スタート直後は全車の速度平均が小さく不安定なので、僚車を一時的に
  // 「明らかに遅い」と誤認して1周目の禁止を迂回させない。実測ではP1が
  // P2へ16秒間仕掛け、その間MPCを狙えなかった。NPCだけは初周から対象。
  if (clearly_slower && (o.slot == npc_slot_ || lap_ >= record_laps_)) { return true; }
  if (o.slot == npc_slot_) { return true; }        // 運営NPC は常に対象
  if (lap_ < record_laps_) { return false; }       // 1周目は録画に専念
  // P1 のときだけ、僚車への仕掛けを最終周まで待つ。
  //
  // 【なぜ3周目->最終周にしたか(実測 20260830-211036)】
  // 5周目 idx6 で `ペナルティ Crash(10s) 最寄り=d2 2.21m` が出て、
  // 直後に d1/d2 が同じ場所で同時に stuck した。**自作の10秒ペナルティ**。
  // これは下の最終区間ブーストのコメントに既に書いてあった実測
  //   「僚車間の逆転6件のうち、5周目に前へ出た4件は4件とも抜き返され、
  //     6〜7周目に出た2件は2件とも守り切った」
  // と矛盾していた。**先に前へ出ることに価値は無い**のに仕掛けていた。
  // さらにハンデ(1位 25km/h / 2位以下 36km/h)があるので、
  // **僚車の後ろにいるほうが速い**。追う必要すらない。
  // 同速の相手への試行は 46〜90m 必要で事実上成立せず、
  // 接触と `追越失敗` を量産するだけだった。
  // 【実測 20260830-21:27〜21:42 の3レース】これは `start_slot_==1 && o.slot==2`
  // つまり **P1 が P2 を抜く場合しか塞いでいなかった**。
  // 実際には `d2 Crash(10s)@d1`(P2 が P1 に突っ込む)も出ていた。
  // 僚車どうしの接触は左右どちらの向きでも起きるので、対称にする。
  // ここへ来る時点で NPC は上で return 済みなので、残りは必ず僚車。
  if (o.slot != npc_slot_ && lap_ < teammate_pass_lap_) { return false; }

  return true;
}

// ===================================================================
// 追い越しの状態機械 (2026-09-02)
// ===================================================================
//
// 【なぜ入れたか(実測3レース)】
//  - 追越試行は起きる(20 / 6 / 2回)が、2.4〜6.9秒続いた後すべて失敗。
//  - 中断理由の最多が「安全中断 理由=停止車回避」(1レース10件中4件)。
//    **抜こうとしている相手そのもの**を停止車と判定して回避し、中断していた。
//  - 速度上限は全周期の大半で「追従」(平均12.8km/h)が決めていた。
//    横へ出るには速度差が要るのに、横へ出るまで速度が出ない循環。
//
// ここは新しい判定を発明しない。既存の値(spot_valid_ / attempt_active_ /
// pass_sep_ / commit_sep_ / my_prog_ / o.prog)から状態を導出するだけ。
// 状態の役目は「誰が速度を決めてよいか」と「何を理由に中止するか」を
// 1箇所で決めること。
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

  // --- 横の余地が物理的に消えているか(唯一の新規計測)
  // 帯幅が車幅 band_car_w_ を下回る状態が spot_abort_sec_ 以上続いたら、
  // そこは物理的に並走できない。瞬間値では中断しない。
  if (ovPassing()) {
    if (c.avail_width < band_car_w_) {
      if (ov_narrow_since_ < 0.0) { ov_narrow_since_ = t; }
    } else {
      ov_narrow_since_ = -1.0;
    }
  } else {
    ov_narrow_since_ = -1.0;
  }

  // --- 中断してよい理由は安全条件だけ ---
  // 緊急TTC: avoidCollision が出した減速上限。applyAvoidance が dbg_avoid_cap_
  //          へ控えているので、その値をそのまま読む(判定は変更していない)。
  // 【修正 2026-09-02】緊急TTC(dbg_avoid_cap_ >= 0.0)を中断理由から外した。
  // 実体は「衝突回避層が何らかの速度上限を出したか」であり、並走すれば当然
  // 出る。実測(20260902-2014/2022/2029)では中断理由がこれだけになり、PASS へ
  // 到達した試行 3/3/1 件が全てこれで潰されていた。抜こうとしている相手を
  // 脅威として扱う、今日6件目の同型の誤り。
  //
  // さらに重要な点として、この中断は安全性に寄与していない。衝突回避層は
  // 自分で速度上限を requestCap し(min 調停なので必ず勝つ)、横方向も
  // kCollision(最高優先度)で退避を要求する。状態機械が上から中断しても
  // 安全側の挙動は1つも増えず、追い越しを壊すだけだった。
  // 中断は「物理的に横の余地が消えた」「時間切れ」「停滞」に限る。
  const bool emergency_ttc = false;
  (void)dbg_avoid_cap_;
  // 【修正 2026-09-02】コース外予測(avoidWall の pushed_to_wall)を中断理由から
  // 外した。その実体は `std::abs(want - safe) > 1e-3`、つまり「横目標が壁帯で
  // 1mm でもクリップされたか」であり、危険の指標ではなく日常的に起きる正常動作。
  // 横位置は band クランプで物理的にコース外へ出られないので、これを中断の
  // 根拠にできない。実測(20260902-200453)では中断19件が全件この理由で、
  // 追越試行が8〜9回あるのに MOVE_OUT へ到達したのは1〜3回しかなかった
  // (COOLDOWN 中に試行が始まり、追従キャップの解除が発動しなかった)。
  // 値そのものは avoidWall 側の減速判断で引き続き使われている。
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
        // 【修正 2026-09-02】試行が始まっているなら、計画の有無に関係なく
        // 直接 MOVE_OUT へ入る。
        //
        // 【何が問題だったか】入口が「計画された抜きどころがある」ことを
        // 要求していたため、「今ここで抜く」経路(spot_here)で始まった試行は
        // PREPARE に入れず、MOVE_OUT にも入れなかった。その結果、状態機械の
        // 主目的である**追従キャップの解除が発動しなかった**。
        // 実測(20260902-221308 / 222119)では追越試行 31回・27回に対し
        // PREPARE→MOVE_OUT は 2回・4回しかなく、大半の追い越しが
        // 速度制限を掛けられたまま行われていた。
        if (attempt_active_ && !attempt_target_.empty()) {
          ov_target_ = attempt_target_;
          next = OvState::kMoveOut;
          reason = "試行開始(計画なし)";
        } else if (spot_valid_ && target_ahead(spot_target_)) {
          ov_target_ = spot_target_;
          next = OvState::kPrepare;
          reason = "計画あり";
        }
        break;
      case OvState::kPrepare:
        if (attempt_active_) {
          if (!attempt_target_.empty()) { ov_target_ = attempt_target_; }
          next = OvState::kMoveOut;
          reason = "試行開始";
        } else if (!spot_valid_) {
          next = OvState::kFollow;
          reason = "計画消失";
        }
        break;
      case OvState::kMoveOut:
        if (std::abs(pass_sep_) >= commit_sep_) {
          next = OvState::kPass;
          reason = "横へ出切った";
        }
        break;
      case OvState::kPass:
        if (passed_target(ov_target_)) { next = OvState::kMerge; reason = "抜き切り"; }
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


// 指定した相手に対して、いま追い越しが進行中で、かつ横方向にその相手の
// 進路から十分外れているか。
//
// 【なぜ作ったか】追従(followAndCommit)と追突防止(preventRearEnd)が
// 「いま追い越し中か」を別々の条件で判定していた。追従の commit_now は
// attempt_active_ を見ず、追突防止の緩和は attempt_active_ 必須。速度上限は
// 両者の min を採るため、片方だけが緩んでももう片方が押さえ込み、実測では
// 全周期の96%で 8〜12km/h に張り付いていた。判定を1本にして揃える。
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
  if (enable_) {
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

    // --- 試行中の横位置は、試行の状態が持つ ---
    //
    // 【何が壊れていたか(実測 2026-09-04、4レース・試行236本)】
    // 追越試行の最中に、横位置の調停で勝っていた意図は
    //   追越 43.0% / **基準(=ラインへ戻る) 39.5%** / 衝突回避 6.0% /
    //   停止車回避 4.9% / 壁回避 3.1% / 近接車反発 2.9%
    // だった。**約4割の周期で横目標が基準ライン(0.0m)へ戻されていた。**
    //
    // 【なぜ戻るか】追越の意図は evaluateOpponent の中でしか出ない。
    // その関数は冒頭で `isNearestBlocker`(いま自分の前をふさぐ最も近い1台か)が
    // 偽なら即 return する。**横に並んだ瞬間、相手は「前」ではなくなるので
    // 追越の意図が出なくなる。** 意図を出す層が1つも無い周期は、既定値の
    // 基準ライン(0.0m)がそのまま勝つ。つまり並走に入った途端、
    // 横目標が相手側へ引き戻される。
    //
    // これは「並走26% -> 先行1%」で崩れることと、走行中の接触14件のうち
    // 13件が「車両+壁」(=並走中に挟まれた)であることの両方を説明する。
    //
    // 【直し方】層を上書きし返す対症療法(旧 holdAttemptSide)には戻さない。
    // 所有者を1つに決める:「試行中の横位置は試行が持つ」。
    // evaluateOpponent は値を**作る**だけで、**保持する**のはここ。
    // 同じ周期で追越の意図が既に出ていれば同じ値なので二重にならない。
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
//
// 相手の横位置と走行可能領域から左右それぞれの余地を出し、学習した
// 相手のラインも加味して side_sign_ を決める。一度決めた側は
// 抜き切るか失敗が確定するまで保持する(ユーザー方針)。余地が無いままなら
// 一度だけ反対側へ回る。
//
// 相手の横位置 olat と、判定に使う相手速度 ospeed_for_gate を返す。
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

  // 相手と反対側へ、必要な間隔ぶん寄せる。
  // ただし相手の横位置は揺れるので、毎周期で左右を決め直すと目標が反転し続ける。
  // 一度どちらに抜けるか決めたら、対象車が変わるか一定時間経つまで側を保持する。
  // 側は対象車が変わったときだけ決め直す。時間で決め直すと
  // 追い越し中に左右が反転して危険なため。
  // 相手と反対側が基本だが、その側にコリドアの余地が無いなら反対へ回る。
  // どちらにも余地が無ければ side_fits_ を偽にして追い越し自体をやめる。
  // 「pass_gap ぶん丸ごと寄れるか」で判定すると厳しすぎる。
  // pass_gap=1.7m に対しコリドアの片側の余地が 1.35〜1.85m しかなく、
  // 幅も時間も足りている場面が side_fits_=false で全部却下されていた
  // (実測: zone=1 幅=4.0(要3.4) 所要=6.1s なのに feasible=0)。
  // 実際に必要なのは「並んだときに車体が当たらない横間隔」なので、
  // 寄れる範囲まで寄った結果の間隔が min_pass_sep 以上あれば良しとする。
  const double reach_left = std::min(olat + pass_gap_, room_hi_);
  const double reach_right = std::max(olat - pass_gap_, room_lo_);
  bool fit_left = (reach_left - olat) >= min_pass_sep_;
  bool fit_right = (olat - reach_right) >= min_pass_sep_;
  // 学習した相手のラインで、抜き切るまでの区間を丸ごと見て側を決める。
  // 瞬間値だけだと、相手がラインを横切っている最中の一瞬を見て
  // 余地の無い側を選んでしまう(実測: 側OK=0 の 48% は反対側なら成立)。
  double map_left = 0.0, map_right = 0.0;
  int map_known = 0;
  double room_l_mean = 0.0, room_r_mean = 0.0;
  if (lane_map_side_) {
    sideRoomMap(o, in, n, oi, lane_map_stretch_, olat, map_left, map_right, map_known,
                &room_l_mean, &room_r_mean);
    // 学習データが区間の大半にある場合だけ信用する。
    // map_left/right は「足りている区間が連続で何m続くか」。
    // 抜き切るのに要る長さ(pass_len)を満たしていれば、その側は成立。
    const double need = pass_len_ * lane_map_need_gain_;
    if (map_known >= lane_map_min_pts_) {
      fit_left = fit_left || (map_left >= need);
      fit_right = fit_right || (map_right >= need);
    }
  }
  dbg_map_l_ = map_left; dbg_map_r_ = map_right; dbg_map_n_ = map_known;
  dbg_room_l_ = room_l_mean; dbg_room_r_ = room_r_mean;
  if (c.blocker != side_blocker_) {
    side_blocker_ = c.blocker;
    side_decided_at_ = now.seconds();
    side_flip_cnt_ = 0;           // 対象車が変わったら側の変更枠を戻す
    side_flip_at_ = now.seconds();
    side_unfit_since_ = -1.0;
    // 側の優先順位: イン > 相手の反対側。
    // イン側を先に押さえると相手は避けざるを得ず、アウトから被せるより
    // 成立しやすい(ユーザー方針: 攻められるならイン、無理なら反対側)。
    // 直線(curve_sign_=0)では従来どおり相手の反対側。
    double want = (olat >= 0.0) ? -1.0 : +1.0;   // 相手が左なら右へ
    // --- 空いているほうから抜く(ユーザー指示 2026-08-29)
    //
    // 以前はここに **区間を焼き込んだ指定**(right_zones = idx220->30 は右)が
    // あった。本番は相手が変わるので、区間で決め打ちすると外れる。
    // 代わりに**録画した相手のライン**から、抜き切るまでの区間で
    // 左右それぞれの空き幅を平均し、大きいほうを選ぶ。
    // 例: 相手がメインストレートで左に寄っていれば room_right が大きくなり、
    // 指定なしで「右から抜く」が導かれる。
    // --- 側は「相手が平均して寄っている方の反対」(ユーザー指示 2026-08-29)
    //
    //   ・使うのは **idx220 -> 25 の直線だけ**(side_pick_zones)。
    //     他の区間はコーナーが絡むので、従来のイン優先・余地の判断に任せる。
    //   ・相手の**区間平均の横位置**を録画から求め、その反対側から抜く。
    //   ・差が小さい(side_pick_tie 以内)ときは右から抜く。
    //
    // 平均で見るのは、相手がラインを横切っている一瞬を捉えて
    // 逆側を選ばないようにするため。
    bool by_room = false;
    if (lane_map_side_ && inSidePickZone(ei)) {
      int zknown = 0;
      const double mlat = zoneMeanLat(o, n, zknown);
      if (mlat < 1e8) {
        // 【修正 2026-09-04】判断の基準を「レースラインからのずれ」から
        // 「相手と左右の壁との空き」へ変える。
        //
        // 【なぜ誤りだったか】zoneMeanLat が返すのは相手の**レースラインからの
        // 横オフセット**。1位は前方に誰もいないのでほぼレースライン上を走り、
        // mlat ≈ 0 になる。すると「相手は中央にいる」と判定される。
        //
        // しかし**レースライン自体がコースの中央にはいない**。
        // corridor_ten.csv の実測:
        //   idx220-233 は lo -0.35〜-1.55 / hi +2.95〜+4.65 → ラインは右壁寄り
        //   idx 10-20  は lo -3.60〜-4.35 / hi +0.35〜+1.00 → ラインは左壁寄り
        // つまり idx10-20 でレースライン上を走る1位は、**コース上では左端に
        // 張り付いている**。空いているのは右で、右から抜くのが正しい。
        // それを「中央にいる」と読んでいた。
        //
        // 正しい量は sideRoomMap が既に計算している room_l_mean / room_r_mean
        // (相手の録画ラインからコリドアの縁までの距離を、抜き切る区間で平均)。
        // 同関数のコメント自身が「相手が左に寄っていれば room_right が大きくなり、
        // 空いているほうから抜くが成り立つ」と書いており、その値を使っていなかった。
        // side_pick_by_room=false で従来の判断に戻せる。
        if (side_pick_by_room_ && map_known >= lane_map_min_pts_) {
          want = (room_r_mean > room_l_mean + side_pick_tie_) ? -1.0   // 右が空く -> 右から
               : (room_l_mean > room_r_mean + side_pick_tie_) ? +1.0   // 左が空く -> 左から
               :                                                -1.0;  // 同じくらい -> 右から
        } else {
          want = (mlat > side_pick_tie_) ? -1.0        // 相手が左 -> 右から
               : (mlat < -side_pick_tie_) ? +1.0       // 相手が右 -> 左から
               :                            -1.0;      // 同じぐらい -> 右から
        }
        by_room = true;
        if ((this->now() - last_wallpick_log_).seconds() > 3.0) {
          last_wallpick_log_ = this->now();
          RCLCPP_INFO(get_logger(),
            "側を録画で決定 target=%s 側=%s 相手の区間平均横=%.2fm(点%d) "
            "空き 左%.2fm 右%.2fm idx=%zu",
            c.blocker.c_str(), (want > 0.0) ? "左" : "右", mlat, zknown,
            room_l_mean, room_r_mean, ei);
        }
      }
    }
    // --- 公式オーバーテイクレーンでは側は右で確定する ---
    //
    // レーンは自車ラインの**右** 2.15〜5.0m の帯(AWSIM のシーンから実測)。
    // ところが idx220-241 の曲率半径は 8.7〜37m なので下の曲率ブロックが
    // 必ず発火し、イン側=左へ書き換える。その左は idx10-20 で左壁まで
    // 1.5〜2.0m しかない。**運営が右に追い越し車線を引いた直線で、
    // わざわざ左の壁際へ潜る**動きになっていた(ユーザーが繰り返し報告)。
    //
    // ここだけ側を固定する。側の反転を全区間で許すと成績が落ちることは
    // 実測済み(2026-09-04: 反転0%->21%で追い越し 0.50->0.00)だが、
    // レーンは242点中34点しかなく、しかも運営が側を決めている区間なので
    // 「場所ごとに側を選び直す」ことにはならない。
    const bool ot_lane_side = ot_lane_side_right_ && otLaneUsable(ei);
    if (ot_lane_side) { want = -1.0; }        // 右
    const bool in_right_zone = by_room || ot_lane_side;   // 録画で決めた側は曲率より優先する
    // 【修正 2026-09-04】上のコメント「録画で決めた側は曲率より優先する」が
    // **実装されていなかった**。in_right_zone は 3881行の別の判定でしか
    // 使われておらず、ここの曲率によるイン優先が録画の判断を無条件に
    // 上書きしていた。
    //
    // idx220-241 は曲率半径 8.7〜37m なので curve_sign_ != 0 が常に成立し、
    // メインストレートで「録画から右と決めた」直後にイン側(左)へ書き換えられる。
    // その左は idx10-20 で 0.35〜1.00m しか無く、相手と左壁に挟まれる。
    // ユーザーが以前から報告していた「右が空いているのに左から抜こうとして
    // 壁に当たる」はこれで説明できる。
    //
    // コメントどおり、録画で側を決めた区間では曲率で上書きしない。
    // side_pick_over_curve=false で従来の挙動に戻せる。
    // レーン内は side_pick_over_curve に関係なく曲率で上書きしない。
    const bool curve_may_override =
      !ot_lane_side && !(side_pick_over_curve_ && in_right_zone);
    if (curve_may_override && curve_sign_ != 0.0) {
      const double inside_sign = (curve_sign_ > 0.0) ? +1.0 : -1.0;
      const bool inside_fit = (inside_sign > 0.0) ? fit_left : fit_right;
      if (inside_fit) { want = inside_sign; }
    }
    // --- 録画から決めた抜きどころの側を最優先する(ユーザー指示) ---
    // 「抜く側(左か右か、経路が大きく空いているほう)を録画から決める」。
    // 瞬間値やイン優先より、抜き切るまでの区間を丸ごと見た判断のほうが強い。
    // その地点に近づいているときだけ効かせる(遠い地点の側で今寄ると危ない)。
    // 速度と必要横移動量から、予測ラインへ展開し始める距離を決める。
    const double side_spot_gate = spotGateDistance(f, o);
    if (spot_enable_ && spot_valid_ && c.blocker == spot_target_ &&
        spot_dist_ <= side_spot_gate && spot_side_ != 0.0) {
      const bool spot_fit = (spot_side_ > 0.0) ? fit_left : fit_right;
      if (spot_fit) { want = spot_side_; }
    }
    // --- 抜く側を予測バンドで決める(ユーザー指示「相手の動きを予想して」)
    //
    // buildBand は相手を「自分がその地点へ着く時刻」まで参照ラインの
    // 速度プロファイルで進め、その**予測経路に沿って** 40m 先まで
    // 左右の通しの空き幅を測って側を決めている。
    // 瞬間位置(従来のイン優先)や録画の区間平均より、
    // 時間をそろえた予測のほうが根拠が強いので、ここで最優先する。
    // ただし fit(実際に寄れるか)は従来どおり尊重する。
    const bool spot_controls_side = spot_enable_ && spot_valid_ &&
      c.blocker == spot_target_ &&
      (spot_dist_ <= side_spot_gate ||
       (attempt_active_ && attempt_target_ == spot_target_));
    // band は短期の安全確認には使うが、ロック済み時空間計画の側を
    // 上書きしない。旧実装は spot の直後に band が走り、計画した側が
    // 毎周期反転していた。
    if (band_side_lead_ && band_enable_ && band_predict_ && !spot_controls_side) {
      const int bs = bandSide(c.blocker);
      if (bs != 0) {
        const bool bfit = (bs > 0) ? fit_left : fit_right;
        if (bfit) { want = (bs > 0) ? +1.0 : -1.0; }
      }
    }
    if (want < 0.0) {
      side_sign_ = fit_right ? -1.0 : (fit_left ? +1.0 : -1.0);
    } else {
      side_sign_ = fit_left ? +1.0 : (fit_right ? -1.0 : +1.0);
    }
    // 両側とも成立するなら、並走できる区間が長いほうを選ぶ。
    // イン優先は「相手が避けざるを得ない」ための策だが、
    // 抜き切るまでの区間で明らかに狭ければ意味がない。
    if (!in_right_zone && lane_map_side_ && map_known >= lane_map_min_pts_ &&
        fit_left && fit_right) {
      if (std::abs(map_left - map_right) > lane_map_margin_) {
        side_sign_ = (map_left > map_right) ? +1.0 : -1.0;
      }
    }
  }
  // --- 対象が同じでも、場所によって空いている側は変わる(ユーザー指示 2026-08-29)
  //
  // 【直した構造の問題】側を決める処理は上の `if (c.blocker != side_blocker_)`、
  // つまり **対象車が入れ替わった瞬間にしか実行されない**。一度決めた側を
  // 追走の間ずっと持ち続けるので、
  //   「最初に決めた場所では左が空いていた」-> その後ずっと左から行こうとする
  // という挙動になる(ユーザー報告「毎回左から抜こうとしているように見えた」)。
  //
  // 実測(録画): メインストレートの終わり(区間22-23 ≒ idx212-242)で
  // NPC は横 +0.84 -> +1.39 と**左へ寄る**。そこでの右の空きは 2.94m あり、
  // 並走に要る 1.15m を大きく上回る。決め直せば右が選べる。
  //
  // まだ踏み切っていない間(試行中でない・横に出ていない)だけ選び直す。
  // 踏み切ってからの反転は左右に振られて危険なので従来どおり禁止。
  if (lane_map_side_ && inSidePickZone(ei) && !attempt_active_ &&
      std::abs(offset_) < pass_gap_ * 0.5)
  {
    // 区間平均の横位置の反対側。同じぐらいなら右(ユーザー指示)。
    int zknown2 = 0;
    const double mlat2 = zoneMeanLat(o, n, zknown2);
    // diff の符号を「左を選びたいときに正」にそろえる。
    const double diff = (mlat2 > 1e8) ? 0.0
                      : ((mlat2 < -side_pick_tie_) ? +1.0 : -1.0);
    // --- 反転のヒステリシス ---
    // 空き幅は先読み区間の平均なので、進むにつれて滑らかに入れ替わる。
    // 閾値を跨いだ瞬間に反転させると、境目で左右に振られて横目標が定まらず、
    // どちらにも並べないまま壁に寄る(実測3レース: 壁 1/5/1・stuck 2/3/3)。
    // 「反対側が side_room_hold 秒continuously 勝っている」ことを要求する。
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
    if (planned_fits) { side_sign_ = spot_side_; }
  }

  side_fits_ = (side_sign_ > 0.0) ? fit_left : fit_right;

  // --- 選んだ側に余地が無いままなら、一度だけ反対側へ回り直す
  //
  // 側は対象車が変わったときしか決め直していなかった。実測(3レース)では
  // side_sign_ が t=0.04s に決まったきり最後まで変わらず、
  // 却下の 83% が「幅・時間・距離は足りているのに side_fits_=false」だった。
  //
  // 【試して却下した版】無制限に回り直す実装は3台走行で悪化した
  // (側の変更12回・追越成功0・stuck 2->7・復帰タイムアウト 1->6)。
  // 左右に振られて壁に当たっていた。そこで制限を3つ入れてある。
  //   (1) 対象車1台につき変更は side_flip_max 回まで
  //   (2) 余地なしが side_flip_hold 秒連続で続いたときだけ
  //   (3) まだ本当に踏み切っていない(|offset| < pass_gap*0.8)ときだけ
  //
  // 実測(3レース): 1台1回の枠は却下 128 件のうち 98 件(77%)が
  // 「反対側なら成立していた」場面で使い果たされていた。枠を 4 回に増やし、
  // 保持時間も 0.6s に縮めてある。左右に振られないための担保は
  // side_room_ahead=15m(先読みを伸ばして側の判断が古くならないようにした)と
  // 上の(3)。committed の判定を pass_gap*0.8(約1.4m)にしたので、
  // 「まだ寄り始めただけ」の段階なら側を直せる。
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
  // 試行開始後に側を反転すると、横目標が左右へ1〜3秒周期で振られ、
  // どちら側にも必要な横間隔を作れない。実測3レースでは側変更38/31/61回、
  // 失敗66件中36件が allow/feasible/latch 全成立なのに横間隔0.34m未満だった。
  // 狭区間では後段の latch_width_ok が試行を終了させるので、ここでは
  // 試行が失敗・終了するまで選んだ側を固定する。
  const bool committed = attempt_active_;
  if (!side_fits_ && side_flip_cnt_ < side_flip_max_ && other_fits && !committed &&
      side_unfit_since_ >= 0.0 &&
      (now.seconds() - side_unfit_since_) >= side_flip_hold_)
  {
    side_sign_ = -side_sign_;
    side_fits_ = true;            // 回った先は余地があると確認済み
    side_flip_cnt_++;
    side_flip_at_ = now.seconds();
    side_unfit_since_ = -1.0;
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

// この相手を抜きにいってよいかを決める。evaluateOpponent の中心。
//
// 見るもの: 幅が足りるか / 抜き切るのに要る距離が使える距離に収まるか /
// 相手が実測で明らかに遅いか / 同速の相手を無理に攻めていないか /
// 自分が1位ハンデ中でないか / 追い越し禁止区間でないか。
// それらを ev.allow に畳み込み、後段(chargeBoost / followAndCommit)が使う。
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
  const double need = gap + pass_len_;             // 抜き切るのに詰める距離

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
  auto pass_dist = [&](double ta) {
    const double t = pass_time(ta);
    if (t >= 1e8) {
      return 1e9;
    }
    // その間に自車が進む距離
    return my_speed * t + 0.5 * accel_eff * std::min(t, ta) * std::min(t, ta);
  };

  // 使える距離。
  // ゾーン外だと zone_remain=0 になり、zone_exit_margin(25m) だけが使える距離になる。
  // 周回遅れの遅い車を抜くには 34〜36m 必要なので、これでは却下されてしまう
  // (実測: 相手12km/h で所要6.6s 距離36m が 25m 制限で却下されていた)。
  // 速度差が十分大きい相手は、抜き切るまでの間ずっと有利なので距離を緩める。
  const double closing_kmh = closing_max * 3.6;
  double usable;
  if (c.slow_leader || closing_kmh >= big_gap_closing_) {
    usable = 1e9;                       // 相手が明らかに遅い。距離で縛らない
  } else {
    usable = c.zone_remain + zone_exit_margin_;
  }

  // イン(旋回内側)から抜く場合は判定を緩める。
  // イン側を先に押さえられた相手は避ける動作を取らざるを得ないため、
  // アウトから被せるより成立しやすい。
  // 曲率の向きと抜こうとしている側が一致していればイン。
  // ただし幅の割引はゾーン内に限る。
  // ゾーンは幅を検証済みの区間なので割り引いても壁に寄らないが、
  // ゾーン外で割り引くと 3.0m 幅の場所へ入り込んで壁に当たる
  // (実測: 割引をゾーン外にも掛けたら d1 の壁接触が 3 -> 16 に増えた)。
  const bool inside = (curve_sign_ != 0.0) && (side_sign_ * curve_sign_ > 0.0);
  const bool inside_ok = inside && c.in_zone;
  // --- 最下位のときは積極的に抜く ---
  //
  // 順位が下なら、抜かない限り結果は変わらない。多少の失敗より
  // 「仕掛けないまま終わる」ほうが損。
  // 実測(21:59版で2位だったレース): 3位スタートから 4倍遅い相手の
  // 後ろで60秒を潰し、その間に勝者は 213m 先へ行った。
  // 一方で1位・2位のときは、無理をして接触すると順位を落とすので
  // 従来どおりの慎重さを保つ。
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
  // --- 実測データから「明らかに遅い相手」を判定する ---
  // 溜めた走行データ(区間ごとの平均速度・ラップタイム)で相手の実力を見る。
  // 瞬間の速度差(closing)ではなく実績で判定するのが重要:
  // 自車が1位でハンデ(25km/h)を受けると、遅い MPC に対する closing が
  // 12km/h を割って「同速」と誤分類され、周回67秒の相手を最後まで
  // 抜けなくなる(実測: P1 のまま MPC の後ろで周回差を付けられた)。
  bool clearly_slower = false;
  {
    const double om = o.meanSpeed();
    const double mm = (my_speed_cnt_ > 20) ? my_speed_sum_ / my_speed_cnt_ : -1.0;
    if (om > 0.0 && mm > 0.0 && om < mm * slow_rival_ratio_) {
      clearly_slower = true;
    }
    // その区間での実績も見る。全体が遅くても、その場所だけ速いことがある。
    if (clearly_slower && !line_x_.empty()) {
      const int sec = static_cast<int>(oi * OtherState::kSections / n);
      const double os = o.sectionSpeed(sec);
      if (os > 0.0 && mm > 0.0 && os > mm * slow_rival_ratio_) {
        clearly_slower = false;
      }
    }
  }
  // --- 同速の相手を攻めない ---
  // 実測(3レース): 同じコードの僚車(自分と同じ速度)への試行は
  // 抜き切るのに 46〜90m 必要で、直線の長さでは足りない。
  // 対して遅い MPC は 42〜46m で足りる。
  // 止まっている・壊れている相手(slow_leader)と、実績で明らかに遅い相手
  // (clearly_slower)はこの足切りの対象外。
  // 先頭車は 25km/h のハンデを受けており、2位以下(36km/h)から見ると
  // 最高速で構造的に上回れる。瞬間の closing が小さくても抜きにいってよい
  // (実測: 2位のとき先頭を「同速」と誤却下 60件/レース、抜けずに終了)。
  const bool capped_leader = (rank_ >= 2) && !cur_leader_.empty() &&
                             (name == cur_leader_);
  const bool closing_ok = (closing_max * 3.6 >= min_closing_kmh_)
                          && (pass_dist(t_accel) <= pass_dist_max_);
  // --- 自分が1位でハンデを受けている間の足切り ---
  //
  // 1位の速度上限は 25km/h。前にいるのが周回遅れの 17〜24km/h の車でも
  // closing は 1〜8km/h にしかならず、min_closing_kmh(12km/h)には
  // 構造的に届かない。つまり「1位の間は誰も抜けない」設定になっていた。
  // 実測(3レース・却下471件): 却下の 365件(77%)が rank=1。
  // 直線手前 idx215-241 では却下53件のうち48件(91%)が
  // 「zone=1・幅OK・側OK で、同速(closing不足)だけが理由」だった。
  // ユーザー報告「220-240 でインから行けるのに相手の後ろを走っている」の正体。
  //
  // 速度差そのものが小さいのは事実なので、時間と距離では従来どおり縛る
  // (pass_time <= t_limit / pass_dist <= usable は self_ok に残っている)。
  // ここで見るのは「そもそも相手より速いか」と「抜き切る距離が現実的か」の2つ。
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
    // ブーストで初めて成立する場合のみ「役に立つ」と判断する。
    // 自力でも成立するなら温存する(ユーザー方針: 使わなくても抜けるなら使わない)。
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
  // 速度差や順位だけを根拠にzone外を許可すると、横へ出る途中で帯が狭まり、
  // 相手の正面へ戻される。公式submit_11の敗戦とローカル再現では、遅いNPCへ
  // zone外から12回仕掛けて成功0、平均速度11.3km/hまで低下した。
  // 新規試行は検証済みzoneか、録画から選んだ抜きどころの直前に限定する。
  // 一度抜いたNPCは別途ブーストと助走を禁止する。一方、展開開始距離そのものは
  // 相手種別の決め打ちでなく、自車速度・横移動量・制動距離から算出する。
  const bool lap_traffic = o.slot == npc_slot_ && o.passed_cnt > 0;
  const double spot_gate = spotGateDistance(f, o);
  const bool spot_ready_planned = spot_enable_ && spot_valid_ && name == spot_target_ &&
                                  spot_dist_ >= 0.0 && spot_dist_ <= spot_gate;
  // 【修正Q 2026-09-02】「今ここから抜き切れる」と予測が示したなら、そこも
  // 正当な抜きどころとして認める。
  //
  // 【何が問題だったか】仕掛けられるのは計画した1点だけで、そこへ着くまでは
  // 「抜きどころ待ち」、待つ間に車間が詰まって「開始車間不足」になっていた。
  // 実測(遅い相手<=13km/h が 25m 以内にいる場面)では、この2つで却下の
  // 36〜58% を占めていた。相手が 8km/h、自車が 36km/h 出せる状況で、
  // その場で抜けるのに「より良い場所」を待って結局抜かない。
  //
  // これは旧方式(静的な pass_ok ゾーン)の復活ではない。判断は引き続き
  // latestPassAccelDelay() の予測シミュレーション(相手の地点別学習速度と
  // 自車の加速・速度上限を 0.1 秒刻みで積分)が行う。区間長には、学習した
  // 相手のラインに対して必要な横間隔が連続して確保できる距離(map_left/right)
  // を使う。計画器は引き続き「最良の抜きどころ」を探すが、それが見つかるまで
  // 抜けないという拒否権は持たなくなる。予測は拒否権ではなく加点である。
  // map_left / map_right / map_known は chooseSide() が同一周期の直前(L3039)で
  // 計算し、dbg_map_* へ保持している。decideAllow からはそちらを読む。
  const double here_room = (side_sign_ > 0.0) ? dbg_map_l_ : dbg_map_r_;
  double here_delay = -1.0;
  // 【修正 2026-09-02】即時の抜きどころは「いま自分の進路を塞いでいる車」に
  // 対してだけ認める。
  //
  // 計画経路には「抜きどころの対象が現在の blocker と一致すること」という条件が
  // あったが(planPassSpot が c_blocker_ != spot_target_ で計画を捨てる)、即時
  // 経路にはそれが無く、進路を塞いでいない車を対象にできてしまっていた。
  // 実測(20260902-2348〜20260903-0003)では PASS 中の 187 周期で、抜く相手が
  // 21.1km/h で走っているのに自車が 9.1km/h に抑えられており、その決め手は
  // 追従(69回)と追突防止(113回)、すなわち**対象ではない第三の車**だった。
  // 進路上に別の車がいるのに、その車ではなく別の車を抜こうとしていた。
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
  const double planned_look = std::min(
    spot_range_, std::max(spot_dist_, 0.0) + std::min(spot_len_, pass_need));
  const bool check_spot_path = using_spot &&
    (spot_ready || (attempt_active_ && attempt_target_ == name));
  const bool predicted_path_ok = !check_spot_path ||
                                 spotPathSafe(f, o, planned_look);
  double spot_exit_distance = 0.0;
  if (check_spot_path) {
    double to_end = f.s[spot_end_] - f.s[ei];
    if (to_end < 0.0) { to_end += f.total; }
    spot_exit_distance = spot_in_now_
      ? to_end : std::max(spot_dist_, 0.0) + spot_len_;
  }
  const double base_accel_delay = check_spot_path
    ? latestPassAccelDelay(
        f, name, o, gap, spot_exit_distance, false,
        std::max(spot_dist_, 0.0)) : 0.0;
  const bool timed_boost_available = is_boosting_ ||
    (boost_remaining_ > 0 && boostLapOk() && !lap_traffic);
  const double boost_accel_delay =
    (check_spot_path && base_accel_delay < 0.0 && timed_boost_available)
      ? latestPassAccelDelay(
          f, name, o, gap, spot_exit_distance, true,
          std::max(spot_dist_, 0.0)) : -1.0;
  const bool timed_boost = check_spot_path && base_accel_delay < 0.0 &&
                           boost_accel_delay >= 0.0 && !is_boosting_;
  const double accel_delay = (base_accel_delay >= 0.0)
    ? base_accel_delay : boost_accel_delay;
  const bool predictive_timing_ok = !check_spot_path || accel_delay >= 0.0;
  // 旧式の瞬間速度式で拾えなくても、地点別シミュレーションが成立を証明した
  // 候補は通常加速・ブーストのどちらでも許可する。以前は timed_boost だけを
  // 救済しており、通常加速の予測が成立しても旧式判定で落ちていた。
  const bool timed_feasible = check_spot_path && accel_delay >= 0.0 &&
                              side_fits_ && width_ok;
  const bool feasible =
    (kinematic_feasible || timed_feasible) &&
    predicted_path_ok && predictive_timing_ok;
  // --- 追い越し禁止区間 ---
  // 実測: idx78-92 は幅 2.3m しかなく、必要幅 2.7m を満たせない。
  // 「側の余地あり」と「幅あり」が同時に成立しないので、ここで仕掛けても
  // latch の時間と側の変更枠を食い潰すだけで終わる。
  bool in_no_pass = false;
  for (const auto & z : no_pass_zones_) {
    const bool inside = (z.first <= z.second)
                          ? (ei >= z.first && ei <= z.second)
                          : (ei >= z.first || ei <= z.second);
    if (inside) { in_no_pass = true; break; }
  }
  // 計画があるときは、その対象・地点だけを使う。従来は別の相手用の
  // 計画が有効でも、現在地が汎用zoneなら目の前の車へ場当たり的に
  // 仕掛けられ、選んだ側・到達時刻が使われない試行が発生していた。
  // P1の学習後は、予測計画が無い場当たり的な試行を禁止する。公式敗戦の
  // 37回試行の多くと、直前の再現でのP2失敗2回はこの fallback から始まった。
  // 1周目のNPC発進追越と、5戦5勝のP2経路は従来どおり汎用zoneを使う。
  // 初回のP3発進追越は汎用zoneで成功しているので維持する。一度抜いた周回遅れ
  // P3は速度差が大きくても、計画なしでは今回9秒の失敗を2回繰り返した。
  // 2回目以降は競技車と同じく、予測区間内で抜き切れる計画を必須にする。
  const bool require_spot_plan = slots_assigned_ && start_slot_ == 1 &&
                                 lap_ >= record_laps_ &&
                                 (o.slot != npc_slot_ || lap_traffic);
  // 【修正 2026-09-02 ユーザー指示】予測を用いた抜き方に一本化するため、
  // デバッグ目的で旧方式の汎用ゾーン fallback を完全に切っていた。
  //
  // 【2026-09-04 実測】却下理由を相手の走行順位別に集計すると、
  // **走行順位1位への却下886件のうち 535件(60%)が「ゾーン外」**で、
  // しかも **661件すべてが self_ok=1**(幅・時間・距離は足りていると自分で
  // 判定済み)だった。速度差不足・幅不足・時間超過はいずれも0件。
  // つまり「抜ける」と計算しておきながら、抜きどころ計画が出ていない
  // という理由だけで却下していた。
  //
  // 【訂正】当初ここには「相手順位別の成功率は P1 が最低(2.7%)」と書いたが、
  // これは誤り。`相手順位=` を含む行には **被追越の記録も混ざっており**、
  // それを成功数として数えていた。`追越記録 成功` に絞ると
  // **走行順位1位を抜いた成功は12レースで0件**である。
  //
  // 【重要】この理屈で汎用ゾーンを復活させる A/B を行ったが、**悪化した**。
  // 同一レース内比較(4レース)で 接触 4.0 -> 9.2件/車レース、
  // ペナルティ 5.9 -> 11.2秒、追い越しは 0.38 のまま変化なし。
  // 機会を増やしても抜けず、並走時間が延びて接触だけが増える。
  // 既定は false のまま維持すること。
  //
  // 汎用ゾーン(corridor の pass_ok)は make_corridor.py が幅と曲率で選んだ
  // 検証済みの区間なので、そこで仕掛けること自体は危険側ではない。
  // 予測計画が出ているときは従来どおりそれを使い、出ていないときだけ使う。
  // zone_fallback_enable=false で 2026-09-02 以降の挙動(予測のみ)に戻せる。
  // 公式のオーバーテイクレーンは**運営が「ここで抜け」と指定した区間**なので、
  // 録画から学習した抜きどころと同格に扱う。
  // 実測(20260905-003852、4台・決勝条件): レーン内(idx234-21)で追い越しが
  // 始まらない理由は全車とも「開始車間不足」12件が最多で、
  // 却下理由の全体1位は「ゾーン外」だった。レーンにいても許可が下りていない。
  const bool ot_lane_here = otLaneUsable(ei);
  const bool zone_ok = spot_ready || ot_lane_here ||
                       (zone_fallback_enable_ && c.in_zone && !require_spot_plan);

  // 計画開始後に相手がこちらの側へ動いた場合、または実測速度を反映した
  // 縦シミュレーションで区間内に抜き切れなくなった場合、古い20秒ロックへ
  // 固執しない。瞬間ノイズでは中止しないよう0.3秒継続を要求する。
  if (attempt_active_ && attempt_target_ == name && using_spot) {
    const bool predicted_plan_ok = predicted_path_ok && predictive_timing_ok;
    if (!predicted_plan_ok) {
      if (spot_unsafe_since_ < 0.0) { spot_unsafe_since_ = now.seconds(); }
      // 【修正L 2026-09-02】0.3秒は短すぎた。並走に入る瞬間は帯も予測も大きく
      // 揺れるため、瞬間的な不成立で中断すると横移動の途中で引き返すことになる。
      // 実測では開始 0.4 秒で中断していた。実際に危険なら TTC と追突防止が
      // 別の層で即座に効くので、ここは判断が固まるまで待つ。
      // 【修正 2026-09-02】ここでの中断(旧「追越試行 予測中断」)を削除した。
      //
      // 外した理由: predicted_path_ok / predictive_timing_ok が言っているのは
      // 「この抜きどころが理想的でなくなった」「計画した区間内に抜き切れなく
      // なった」であって、**危険ではない**。実測(3レース)では追越試行が
      // 20 / 6 / 2 回起きて 2.4〜6.9秒続いた後すべて失敗しており、横へ出た
      // 途中で「理想的でなくなった」だけを理由に引き返していた。危険な場合は
      // 緊急TTC(avoidCollision)・追突防止(preventRearEnd)・壁回避
      // (avoidWall)が別の層で必ず効く。横の余地が物理的に消えた場合は
      // 状態機械が「帯幅 < band_car_w_ が spot_abort_sec_ 継続」で中断する。
      //
      // この2つの条件は削除ではなく **「新規に開始しない」条件としてだけ残す**。
      //   feasible(= ... && predicted_path_ok && predictive_timing_ok) -> allow
      //   latched(= attempt_active_ && predicted_path_ok && ...)
      // は下でそのまま使っており、成立しない間は新規の試行が始まらない。
      // 継続中の試行を attempt_active_ = false で降ろすのをやめただけ。
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
  // 一度始めた試行は、条件が多少揺らいでも続行する。
  // 揺らぐたびに追従制御が車間を詰め直すので、車間 4.1m のまま
  // 12 秒粘って打切りになる、という現象が起きていた。
  // 継続は「幅がある間だけ」。幅が無くなったら降りる。
  // 幅を見ずに継続すると、狭い区間へ横オフセットを保ったまま進入して壁に当たる。
  // ただし開始時と同じ厳しさで見ると、幅がわずかに揺らいだだけで
  // 並走の途中で降りてしまう。並走中に急に戻るほうが危ないので、
  // 継続中だけ latch_width_gain 分だけ緩める
  // (実測: 試行15回すべて途中で降りて成功0回)。
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
  // 横移動を始めるには、少なくとも相手の後端から安全余白までの距離が要る。
  // 既に rear-end guard が制動へ入る 2〜3m で新規に側へ出ると、横へ寄せ切る
  // 前に相手の正面へ食い込み、回避/復帰が壁側へ押し出される。submit_13 の
  // Wall(6周目 idx52)は、停止しかけた d3 の 2.9m 後方から新規試行を開始した
  // 直後にこの形になった。継続中の並走はここで落とさない。
  // 予測経路・縦シミュレーションの両方が成立した試行は、横移動中も
  // followAndCommit の相手速度キャップと最終段の追突防止が残る。
  // 助走目標5.0mに対し通常閾値4.5mを厳密比較すると、制御誤差で実測
  // 4.2～4.5mを往復して永遠に開始できなかったため、予測時だけ0.7m余裕を使う。
  const bool predictive_start_safe = check_spot_path && predicted_path_ok &&
                                     predictive_timing_ok;
  const double start_gap_need = rear_end_margin_ +
                                (predictive_start_safe ? 0.3 : 1.0);
  // 【修正L 2026-09-02】「追い越しを開始するのに最低 4.5m の車間が必要」という
  // 要求そのものをやめる。近いから抜くのであって、横移動は相手から離れる方向
  // なので前へ食い込まない。縦方向の追突は TTC(avoidCollision)と追突防止
  // (preventRearEnd)が別の層で担保している。
  //
  // 【この判定が壊れていた経緯】助走は車間 5.0m を目標に詰めるのに開始条件が
  // 4.5m で、制御誤差により実測 gap が 4.1〜4.4m に収束して永久に開始できな
  // かった。接近速度による緩和を足したが、同じ日に「最も遅い加速」を撤去した
  // ことで接近速度が常に閾値超えとなり、緩和が一度も効かなくなった
  // (実測 20260902-145021: 開始車間不足の gap が再び 4.1〜4.4m に張り付き)。
  //
  // 計画した抜きどころに着いているなら、残すのは「相手の直後に貼り付いた
  // 状態から新規に横へ出ない」ための最小限(start_gap_floor)だけにする。
  // レーン内では横に 2.2〜2.5m の走行可能幅が確保されている(実測)ので、
  // 「相手の直後から新規に横へ出ない」ための最小限(start_gap_floor)まで許す。
  // 4.5m を要求していると、レーンに着いた時点で既に車間が足りない。
  const bool start_gap_settled = (spot_ready || ot_lane_here) && gap >= start_gap_floor_;
  const bool start_gap_ok =
    attempt_active_ || gap >= start_gap_need || start_gap_settled;
  // --- 周回による解禁(ユーザー指示) ---
  // 1周目は録画のために NPC 以外を抜かない。P1 が僚車を抜くのは3周目以降。
  // 「相手が想定より遅かったら抜いてよい」ので、実測で遅い相手は対象外。
  const bool lap_ok = passAllowedThisLap(name, o, clearly_slower || c.slow_leader);

  // --- 決めた抜きどころまで待つ(ユーザー指示) ---
  // 録画から決めた地点の手前 spot_gate_slack まで来ていなければ仕掛けない。
  // 待っている間は followAndCommit の助走が車間を詰めるので、
  // 入口に着いたときにはちょうど pass_gap まで縮んでいる。
  // 既に仕掛けている最中だけは縛らない。相手が遅いことは、狭い場所で
  // 横移動してよい根拠にはならない（むしろ接近速度が大きく危険）。
  bool spot_block = false;
  if (spot_enable_ && spot_gate_ && !attempt_active_ && lap_ >= record_laps_) {
    if (spot_valid_ && name == spot_target_) {
      spot_block = (spot_dist_ > spot_gate);
    }
    // 【待ちすぎを止める(実測 2026-08-29)】d1 の却下 72 件が
    // 「抜きどころ待ち」で、その間ずっと前の車の後ろに留まり、
    // 3台とも 67秒/周のまま「抜いた」が 0 回だった。
    // 計画した地点に着かないまま待ち続けるくらいなら、
    // 手前で成立する機会を使うほうがよい。
    if (spot_block) {
      if (spot_wait_since_ < 0.0) { spot_wait_since_ = now.seconds(); }
      // 予測を必須にしたP1は、待ち時間を理由に汎用追越へ落とさない。
      // 実測では入口49m手前でこのフォールバックが発火し、狭窄へ並走進入した。
      if (!require_spot_plan && spot_gate_max_wait_ > 0.0 &&
          (now.seconds() - spot_wait_since_) > spot_gate_max_wait_) {
        spot_block = false;
      }
    } else {
      spot_wait_since_ = -1.0;
    }
  }
  // --- 直線で仕掛けるなら、直線の中で抜き切れること(ユーザー方針の裏付け)
  //
  // コーナー入口(idx18以降)は右の幅が 4.10m -> 0.75m へ縮む。
  // 並走のまま入ると相手の正面へ押し込まれ、Crash(10秒 5km/h)になる。
  // 実測では追越試行 19 に対し成功 0・失敗 14 で、失敗のほとんどが
  // 「直線で並びかけたがコーナーまでに抜き切れない」形だった。
  // 抜き切れないなら横に出ない。次の周でやり直すほうが速い。
  bool straight_short = false;
  {
    const double d_end = distToSidePickZoneEnd(f);
    if (d_end >= 0.0 && !c.slow_leader && !clearly_slower && !attempt_active_) {
      straight_short = pass_dist(t_accel) > d_end + straight_finish_margin_;
    }
  }
  // latch は「横に出続けてよい」だけを許す。速度上限の解除(commit)まで
  // latch で救うと、直線 232:17 で開いた試行がコーナー入口 idx18 で
  // 上限解除を発火させ、右の余地が 3.45m -> 0.10m へ消える先で
  // 相手の正面へ押し込まれる(実測3レース 3/3 で再現、2/3 で Crash)。
  // latch 本来の目的(狭所で急にラインへ戻らない)は allow 側で不変。
  const bool allow_base = (zone_ok && feasible && lap_ok && !spot_block &&
                           !straight_short && !in_no_pass && !stall_block &&
                           !stop_avoid_block && start_gap_ok);
  // 【修正 2026-09-03 夜】latch に「許可」を持たせるのをやめる。
  //
  // 【latch は何のために入ったか】「狭所で急にラインへ戻るほうが危ない」。
  // つまり守りたかったのは**横位置の保持**であって、仕掛け続ける許可ではない。
  // その横位置の保持は、下の横位置ブロックが `attempt_active_ && 対象一致` で
  // 既に allow と無関係に行っている(修正P 2026-09-02)。**二重になっている。**
  //
  // 【実測(2026-09-03 夜、2レース、2秒窓1808)】
  //   latch で継続(allow_base が偽) 451窓 = 全走行時間の 24.9%
  //   その内訳: ゾーン外185 / 側の余地なし111 / 速度差不足88 / 禁止区間55
  //   接触50件のうち 68% が、直前に「latch で継続」の窓を持っていた。
  // **「側の余地なし」111窓と「禁止区間」55窓は安全に直結する。**
  // 横に出る余地が無いと判定されているのに追い越しを続け、
  // 禁止区間に入っているのに続けていた。
  //
  // latch を外すと can_pass_now_ が偽になり、追従の安全車間が 1.3倍に広がって
  // 助走側へ回る。これは 外部レビュー の言う「abort は offset=0 ではなく、
  // 横位置を保ったまま縦の車間を作って相手を先に行かせる」そのものになる。
  //
  // latch_allow_enable=true で従来の挙動に戻せる(A/B の対照)。
  // latch で越えてはいけない理由(安全に直結する2つ)。
  // ここを越えると、許可が下りていない追い越しを最狭部や禁止区間へ運び込む。
  const bool latch_hard_block = latch_never_no_pass_ && (in_no_pass || !side_fits_);
  const bool allow = allow_base ||
                     (latch_allow_enable_ && latched && !latch_hard_block);
  // 追い越しが途中で降りる原因を追うため、判定の中身を残しておく。
  dbg_allow_ = allow; dbg_width_ = w_avail; dbg_zone_ = c.in_zone;
  // 【追加 2026-09-03 夜】継続の可否を試行の終了判断へ渡す。
  // 従来、試行の終了は 打切16秒 / 進展なし / 横に出られない だけが持っており、
  // **実現可能性は終了に一切関与していなかった**。開始を止めた条件が、
  // 同じ試行の継続を止められないのは基準が二つあるのと同じ。同じ量を使う。
  //
  // 【修正 2026-09-03 深夜 ユーザー指示】打切りの条件を `allow_base` から
  // **自分の壁余裕**へ差し替える。
  //
  // 【なぜ変えたか】allow_base で打ち切った実測(6レース)では、
  // Crash/並走100秒 0.19→0.13、車両接触/並走100秒 0.94→0.70 と安全側は改善したが、
  // **走行中の相手を抜いた回数が 0.42→0.00 になった**(成功38→3)。
  // 試行数は 18.5→33.8/車レースへ倍増し、始めては切るだけになっていた。
  // allow_base には「ゾーン外」「速度差不足」「側の余地なし」が入っており、
  // *自分が危ないわけではない*理由で抜くのをやめていた。
  //
  // 見るべきは「そのまま進んだら自分が壁に当たるか」だけ。相手に当たる側の
  // リスクは相手の回避と既存の追突防止・衝突回避が持つ。
  if (attempt_active_ && attempt_target_ == name) {
    const double look = std::max(std::abs(f.ev), 1.0) * attempt_wall_look_time_;
    const double edge = minEdgeClearAhead(f, my_lat_for_target_, look);
    if (edge >= attempt_wall_abort_clear_) { attempt_infeasible_since_ = -1.0; }
    else if (attempt_infeasible_since_ < 0.0) {
      attempt_infeasible_since_ = now.seconds();
    }
  }
  dbg_latched_ = latched; dbg_feasible_ = feasible; dbg_zone_ok_ = zone_ok;

  // 却下された理由を残す(パラメータ調整のため)
  const char * why =
      straight_short                 ? "直線内に抜き切れない"
    : !lap_ok                        ? "周回で禁止"
    : spot_block                     ? "抜きどころ待ち"
    : in_no_pass                     ? "禁止区間"
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
    : !zone_ok                       ? "ゾーン外"
    :                                  "その他";

  // 【観測のみ 2026-09-03 夜】allow = allow_base || latched なので、
  // 一度試行が始まると allow_base の判定(幅・時間・距離・速度差・禁止区間)は
  // latch に迂回されうる。**どれだけ迂回しているかを測っていなかった。**
  // 「試行終了時に latch=1 が96%」は迂回の頻度ではない(終了時点の一断面)。
  // 迂回そのものを数えるため、latch で救われている周期だけを別に残す。
  // ここは記録だけで、制御は一切変えない。
  if (latched && !allow_base && (now - last_latch_log_).seconds() > 2.0) {
    last_latch_log_ = now;
    diagLog("追越継続", "追越継続 latchで継続 target=%s allow_base落ち=%s "
            "幅=%.2f(要%.2f) 先読幅=%.2f 経過=%.1fs idx=%zu 自車=%.1fkm/h",
            name.c_str(), why, c.avail_width, w_need, w_avail,
            now.seconds() - attempt_start_, f.ei, std::abs(f.ev) * 3.6);
  }

  if (!allow && (now - last_reject_log_).seconds() > 2.0) {
    last_reject_log_ = now;
    // 側が理由の却下を直接読めるようにする。
    // 実測(3レース)では却下の 83% が「幅・時間・距離は足りていて
    // side_fits_ だけが偽」だったが、このログに側が出ていなかったため
    // 幅や時間の不足を疑って対策を外し続けていた。
    // 却下の「決め手」を1語で出す。従来は zone/側/幅/同速 の各フラグしか
    // 出しておらず、capped_self や boost_would_help で救われたかどうかが
    // 読めなかった。実測(5レース471件)を集計したとき、
    // 「同速=1」が立っていても実際には別の条件で落ちている行が混ざり、
    // 原因の切り分けを誤りかけた。

    diagLog("追越却下", "追越却下 決め手=%s self_ok=%d capped自=%d capped先=%d 遅相手=%d ブ助=%d "
      "gap=%.1f zone=%d %s 幅=%.1f(要%.1f) 残距離=%.0f "
      "v_reach=%.1f 相手=%.1f closing=%.1f 所要=%.1fs 距離=%.0f rank=%d "
      "側OK=%d 側=%s 相手横=%.2f 余地=[%.2f,%.2f] idx=%zu 同速=%d 禁止区=%d レーン=%d "
      "枠=%d/%d 不成立=%.1fs 学習連続=[%.1f,%.1f]m/%d点 空き=[%.2f,%.2f]m "
      "抜きどころ=%s/%s/%.0fm 展開距離=%.0fm 加速待=%.1fs 周回=%d 相手P%d"
      " 帯幅min=%.2f 閉塞位置=%.0fm 閉塞幅=%.2f"
      " 助走要車間=%.1fm 加速開始=%.1fm",
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
      runup_need_gap_, runup_accel_at_);
  }
  // --- 追い越し中は「相手の端と壁のちょうど真ん中」を走る(ユーザー指示 2026-08-31)
  //
  // 【これまで】目標は常に `相手の横位置 + 側 x pass_gap` だった。
  // pass_gap は 1.75m 固定なので、壁までまだ 4m 空いていても
  // **相手から 1.75m の位置に貼り付く**。ユーザー報告
  // 「壁と相手の間に十分スペースがあるのに相手の方へ寄っていく」の原因。
  //
  // 【これから】自車の中心が取りうる範囲の両端、
  //   近い端 = 相手の中心から接触しない最小間隔(band_car_w / min_pass_sep の大きいほう)
  //   遠い端 = コリドアと安全余裕で決まる壁側の限界(room_hi_ / room_lo_)
  // の中点を狙う。壁に寄りすぎる場合は room_* のクランプが効く。
  // 余地が最小間隔しか無いときは従来どおり最小間隔へ寄る。
  // 【調停 2026-09-02】抜かないときは「ラインへ戻せ」と主張しない。
  // 主張しなければ基準ライン(kBase の 0.0)が残るので挙動は同じで、
  // かつ他の意図(停止車回避など)を潰さない。
  double tgt_lat = 0.0;
  // 【修正P 2026-09-02】試行中は allow の瞬間値に関わらず追越の意図を出し続ける。
  //
  // 調停器では「意図を出さない = 基準ライン(0.0m)が残る」ので、allow が一瞬でも
  // false に落ちると横目標が 0.0m へ跳ね、並走中の車がラインへ引き戻される。
  // 実測(20260902-190845)では `追越試行 失敗 横間隔=0.00 最大実測横間隔=0.84
  // allow=0` のように、横に出た直後 allow が落ちて失敗していた。
  //
  // 無効化した holdAttemptSide / holdSideBySide は、この跳ね返りを後段で
  // 上書きし返すための対症療法だった。層の上書き合戦は正しく撤去したが、
  // その裏にあった「試行中は横位置を保持する」という要件はここへ移す。
  // 試行の終了判断は allow ではなく試行側の中断・成功・失敗判定が持つ。
  // 【修正R 2026-09-03】接近中(PREPARE)から横へ出始める。
  //
  // 【何が問題だったか】横位置の要求は allow(=仕掛けてよい)が立つまで出して
  // いなかったため、抜きどころへ着くまで基準ライン上を相手の真後ろで走る。
  // 真後ろにいる限り追従キャップが効くので、**相手と同じ速度=相対速度ゼロ**の
  // まま抜きどころに到達し、そこから加速を始めることになる。しかも横へ出ると
  // 経路速度が落ちる(実測: 制限なしの周期で FOLLOW 29.1km/h に対し PASS
  // 24.7km/h)ので、優位が 1km/h しか作れず 16 秒で抜き切れない。
  //
  // 抜く場所には**速度を持って**到達する必要がある。そのために接近中から
  // 相手の進路の外へ出ておく。横に離れれば追突の対象でなくなり、追従キャップが
  // 外れて加速できる(下の followAndCommit 側の解除条件と対になっている)。
  if (allow || (attempt_active_ && attempt_target_ == name) ||
      ovPreparingTarget(name)) {
    const double sep_min = std::max(band_car_w_, min_pass_sep_);
    const double near_p = olat + side_sign_ * sep_min;
    const double far_p = (side_sign_ > 0.0) ? room_hi_ : room_lo_;
    const bool has_room = (side_sign_ > 0.0) ? (far_p > near_p) : (far_p < near_p);
    const bool execute_spot = spot_ready ||
      (spot_valid_ && name == spot_target_ && attempt_active_ &&
       attempt_target_ == spot_target_);
    // 予測地点で選んだ固定ラインを最後まで使う。現在の相手位置を基準に
    // 毎周期目標を作ると、相手が同じ側へ動いたときにその後を追いかけ、
    // 広い側を選んだはずなのに横間隔が増えない。
    const double tgt = execute_spot
                         ? spot_offset_
                         : ((pass_center_ && has_room)
                              ? 0.5 * (near_p + far_p)
                              : (olat + side_sign_ * pass_gap_));
    // コリドアの余地(room_lo_/room_hi_)は先読み区間の交差であり、
    // **この意図の内部の丸め**として掛ける。全層に効く制約にすると、
    // 停止車回避や発進レーンまで先読みの狭さで縛ってしまう。
    tgt_lat = std::clamp(tgt, room_lo_, room_hi_);
    c.requestLat(tgt_lat, PlanCtx::LatPrio::kOvertake, "追越");
    // 試行中の横位置は「試行」が持つ。ここは値を作る場所であって、
    // 保持する場所ではない。作った値を試行の状態へ預ける。
    if (attempt_active_ && attempt_target_ == name) {
      attempt_lat_ = tgt_lat;
      attempt_lat_valid_ = true;
      attempt_lat_fresh_ = true;
    }
  }
  // 追い越しが成立しているかは「走行ラインからどれだけ離れたか」ではなく
  // 「相手からどれだけ横に離れたか」で見る。
  // 相手がラインから外れている場合、正しい追い越し位置が
  // ライン上(オフセット約0)になることがあり、
  // |target_offset| で判定すると横に出た瞬間に失敗と数えてしまう
  // (実測: 相手が +1.55m にいて目標 -0.15m、間隔は 1.7m 取れているのに失敗扱い)。
  // 【修正 2026-09-02】計画上の目標(tgt_lat)ではなく、いま実際に離れている量を
  // 入れる。しかも allow で 0 に潰さない。
  //
  // 【これが何を壊していたか】pass_sep_ は失敗判定
  // (|pass_sep_| < min_pass_sep_*0.3 が 0.5 秒続いたら失敗)と、状態機械の
  // MOVE_OUT→PASS / MERGE→FOLLOW 判定に使われる。旧実装は allow が false に
  // なった瞬間に 0.0 を代入していたため、**許可フラグが一瞬落ちただけで
  // 0.5 秒後に自動的に「失敗」が記録されていた**。車は横に出たままである。
  // 実測(20260902-203904 / 205348)では
  //   `追越試行 失敗 横間隔=0.00(要0.48) 最大実測横間隔=2.41`
  // のように、同じログの中で実測 2.4m 並走している事実が併記されていた。
  // 「失敗」は追い越しの失敗ではなく、許可フラグが落ちたことの記録だった。
  //
  // my_lat_for_target_ は findFrontCar() が同一周期の先頭で更新しており
  // (onTimer の呼び出し順: findFrontCar → planOvertake)、ここでは最新値。
  // followAndCommit も同じ量を lat_sep_now として使い、そこには既に
  // 「目標値 pass_sep_ ではなく、いま実際に離れている量で見る」と書かれていた。
  // 判定に使う量を実測へ揃える。
  pass_sep_ = my_lat_for_target_ - olat;
  can_pass_now_ = allow;

  // 【修正 2026-09-02】抜きどころに着いているのに仕掛けられない状態が続く
  // 計画は死んでいる。実測(20260902-122015-s0)では残距離0mのまま16秒
  // 却下され続け、その間まったく別の候補を探せなかった。旧方式の汎用ゾーンを
  // 切った以上、機会の数は「死んだ計画を早く捨てて次を出す」ことで確保する。
  // ここでは決め手(why)は前段のログブロックにスコープが閉じており使えない
  // ため、破棄ログには含めない。
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
  ev.accel_delay = accel_delay;
  ev.timed_boost = timed_boost;
  ev.allow = allow;
  ev.allow_commit = latch_commit_ ? allow : allow_base;
}

// 追い越しのための助走ブーストを撃つか決める。
//
// ユーザー方針: 「抜くために少し手前からどんどん加速していき、
// 相手が 25km/h しか出せないところを追い抜く」。
// ブーストは 10 秒持続するので並んでから撃つのでは遅い。
// ゾーンが射程に入った時点で、並ぶ前に撃つ。
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
  // 通常加速とブーストの開始時刻を同じ予測へ揃える。まだ待てる段階や、
  // 今すぐ加速しても間に合わない計画で先にブーストを消費しない。
  // 【修正 2026-09-02】accel_delay > 0.5 での足切りを外した。値は「最も遅い
  // 開始時刻」であって危険度ではなく、大きいほど余裕がある計画を意味する。
  // 余裕のある計画ほど落とすという逆の判定になっていた。
  if (ev.timed_plan && ev.accel_delay < 0.0) { return; }
  if (ev.timed_boost &&
      std::abs(my_lat_for_target_ - ev.olat) < rear_end_free_min_) { return; }

  // ブーストの発火判定。
  // 使わなくても抜けるなら使わない。1個で足りなければ2個目を使うが、
  // 最初から2個使いにいくことはしない(1個目の効果を見てから)。
  // 真後ろにいる状態(オフセット0)でブーストを撃っても、下の追従制御が
  // 前車速度で頭打ちにするので完全に無駄になる(実測で2個とも空撃ちした)。
  // 実際に横へ出て並びかけているときだけ使う。
  // 横へ出てから撃つと、直線を半分使ってから加速し始めることになり、
  // 直線の終わり(コーナー入口)で並んだまま突っ込んで失敗する。
  // 仕掛ける直前(まだ真後ろ)でも、これから直線に入るなら先に撃つ。
  // --- 助走ブースト: 抜くために「手前から」加速しておく
  //
  // ユーザー方針: 「抜くために少し手前からどんどん加速していき、
  // 相手が 25km/h しか出せないところを追い抜く」。
  //
  // 【なぜ手前から撃つのか】
  // ブーストは **10秒持続**する。並んでから撃つ従来の条件
  // (moved_out = 横に出てから / commit_boost_time = 並走2.5秒)では、
  // 加速し始めた時点で既に相手の真横におり、10秒のうち有効に使えるのは
  // ごく一部。実戦では並走の継続が中央値 0.9秒しかなく、
  // 3個中1個を残したままレースが終わっていた。
  // 追い越しゾーンが射程に入った時点で撃てば、ゾーンに入るときには
  // すでに速度が乗っている。
  //
  // 【なぜ「相手が25km/hしか出せないところ」なのか】
  // 1位は driveFadeSpeed が 25km/h に制限される(handicap)。
  // 2位以下は 36km/h。**先頭を追うときだけ、構造的に 11km/h 速い。**
  // 同じ速度の相手はコーナー速度でも差が出ないので、
  // この速度上限の差が同格の相手を抜く唯一の確実な手段になる。
  // 実測: 自コード同士(完全に同速)では 57回試行して成功 0回。
  // 横間隔は 91% が車幅以上に達しているのに前へ出られない。
  // 並走できても速度が同じなら永久に抜けないという当たり前の帰結。
  if (boost_runup_enable_ && !want_boost_ && boost_remaining_ > 0 &&
      !is_boosting_ && start_merge_done_ && my_speed > boost_min_speed_ &&
      headroom > boost_min_headroom_ && boostLapOk() &&
      (c.in_zone || (spot_valid_ && spot_dist_ <= spot_gate_slack_)) &&
      gap >= boost_runup_gap_min_ && gap < boost_runup_gap_ &&
      (capped_leader || clearly_slower || c.slow_leader))
  {
    const double since = (now - last_boost_time_).seconds();
    if (boost_used_ == 0 || since > boost_retry_sec_) {
      want_boost_ = true;
      RCLCPP_INFO(get_logger(),
        "ブースト使用(助走) target=%s 車間=%.1fm 相手=%.1fkm/h "
        "自車上限=%.1fkm/h 余地=%.1f 先頭ハンデ=%d 遅相手=%d "
        "%d周目 残り%d rank=%d",
        c.blocker.c_str(), gap, ospeed_for_gate * 3.6, v_reach * 3.6,
        headroom, capped_leader ? 1 : 0, clearly_slower ? 1 : 0,
        lap_ + 1, boost_remaining_, rank_);
    }
  }

  const bool moved_out = std::abs(offset_) > pass_gap_ * 0.5 || straight_ahead_;
  // スタート直後は全車が数m以内に密集しており、しかも追い越しゾーンが
  // メインストレート(スタート/フィニッシュ直線)なので in_zone が真になる。
  // 全員が加速中で誰も抜けないのに「抜ける」と誤判定してブーストを2個とも
  // 撃ってしまっていた(実測: 開始直後に 10.1 秒間隔で2個消費)。
  //  (1) スタートの合流が終わるまで撃たない
  //  (2) 十分な速度が出ていないと撃たない(低速ではブーストの効果も薄い)
  const bool start_phase_over = start_merge_done_;
  const bool fast_enough = my_speed > boost_min_speed_;
  // 序盤に使うと、抜いた後にまた抜き返されてブーストが無駄になる。
  // 終盤まで温存して、そこで確実に仕掛ける。
  // ただし最終ラップまで待つと失敗したときに取り返せないので、
  // boost_hold_laps で「何周を終えたら使ってよいか」を決める。
  // 終盤まで温存する制限は無効化した(boost_hold_laps=0)。
  // 同ランク帯との勝負では、序盤に詰まって失う時間のほうが
  // 「抜き返される」危険より大きい。実測でも 21:59版のレースで
  // 序盤60秒を遅い車の後ろで潰し、その間に勝者は213m先へ行った。
  // 温存が必要になったら boost_hold_laps を戻す。
  // ブーストの解禁は3周目以降(ユーザー指示)。boostLapOk がその判断。
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

  // --- 相手がこれから落とす速度を先読みする ---
  //
  // カーブでは相手はほぼ確実に減速する。それに気づかず今の速度だけを見て
  // 追従すると、後ろから加速していって追突する(実測で頻発)。
  // 相手の少し先の速度プロファイルを見て、そこまで落ちる前提で合わせる。
  //
  // ただし「遅く見積もる」方向にしか使わない。速く見積もると
  // 追い越しの判定が甘くなって危険なため。
  // また追い越し中(passing_now)には効かせない。効かせると
  // 相手の減速に合わせて自分も落とし、並んだまま抜けなくなる。
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
      // --- 録画した「その地点で相手が実際に出している速度」も使う
      //
      // 経路の速度プロファイルは**自分が出せる速度**であって、
      // 相手が出す速度ではない。相手が NPC の場合、コーナーでの減速は
      // 自分より遥かに大きい(実測: ラップ 66.8s 対 47.2s)。
      // プロファイルだけで先読みすると相手の減速を見落とし、
      // コーナーで後ろから突っ込む。実測の Crash は
      // **すべて相手が NPC**で、idx57 / 150 / 164 とコーナーに集中していた。
      // 録画には地点ごとの相手の速度があるので、それを使う。
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
  // 助走モード。空けた車間を「目標車間の縮小を追いかける」だけでは
  // 実際の加速が始まらない(実測: ゾーン入口で車間 9m 残り、所要9秒級の
  // 失敗が多発)。車間が目標より大きい間は追従キャップ自体を外し、
  // 速度プロファイルどおり加速して詰める。TTC・緊急減速は別段で効く。
  bool charge_now = false;
  const bool predictive_approach = spot_enable_ && spot_valid_ &&
                                   c.blocker == spot_target_;
  // 周回遅れNPCには助走で車間を詰めない。予測した側へ出られない周期でも
  // 安全車間を維持し、ラインが空いた周期に横移動を先行させる。
  if (approach_enable_ && !can_pass_now_ && !ev.lap_traffic) {
    // 次に仕掛けられる場所までの距離を測る。
    // 録画から抜きどころを決めてあるなら**そこ**を目標にする(ユーザー指示)。
    // 「決めた抜く地点では、抜き始める前までに十分加速しておき、
    //  抜き始められるところでちょうど車間が縮まるように」。
    // 決めていないときだけ、従来どおり手前のゾーンを探す。
    double d_zone = -1.0;
    if (spot_enable_ && spot_valid_ && c.blocker == spot_target_ &&
        spot_dist_ >= 0.0 && spot_dist_ <= approach_range_) {
      d_zone = std::max(spot_dist_, 0.0);
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
      // 詰める速度差は相手の「その先の区間での実績」で見積もる。
      // 相手の瞬間速度(コーナーで遅い)を使うと closing が過大になり、
      // want が上限まで張り付いて空けすぎる(実測: 入口で 9m 残り)。
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
      // 【追加 2026-09-03】助走: 抜きどころへ「速度を持って」到達する。
      //
      // 【何が問題だったか】既存の助走は車間を**詰める**ためのもので、
      // 相対速度を作る計算が無かった。相手の真後ろで同じ速度のまま抜きどころへ
      // 着き、そこから加速を始めるため、実測で PASS 中の自車 22.6km/h に対し
      // 相手 23.5km/h と優位が作れず、16秒のタイムアウトで中断していた。
      //
      // 加速には距離が要り、加速中は相手との車間が縮む。その縮む量を
      // あらかじめ開けておき、抜きどころの手前で全開にすれば、到達時に
      // 相対速度を持っている。
      //
      // 【charge_now を直接立てない理由】この直後の既存行
      //   charge_now = gap > eff_safe + 0.3;
      // が無条件に代入するため、ここで true にしても上書きされる。
      // 意図(全開)は runup_charge に持ち、既存の判定がすべて終わったあと、
      // **制動距離による打ち切りより前**で反映する。安全側の打ち切りは残る。
      bool runup_charge = false;
      if (runup_enable_) {
        const double a_eff = std::max(vehicle_accel_ * 0.60, 0.05);
        const double v_self_now = std::max(std::abs(f.ev), 0.5);
        const double v_opp = std::max(o_ref, 0.5);      // 既存の相手速度見積り
        const double v_tgt = std::min(v_opp + runup_dv_, rank_cap);
        if (v_tgt > v_self_now) {
          const double t_acc = (v_tgt - v_self_now) / a_eff;
          const double d_self = v_self_now * t_acc + 0.5 * a_eff * t_acc * t_acc;
          const double d_opp  = v_opp * t_acc;
          const double shrink = std::max(d_self - d_opp, 0.0);
          const double need_gap = std::min(shrink + pass_gap_, runup_gap_max_);
          const double accel_at = d_self + runup_margin_;
          if (d_zone > accel_at) {
            // まだ加速するには早い。車間を開けて待つ。
            eff_safe = std::clamp(std::max(eff_safe, need_gap),
                                  safe_gap_min_, approach_gap_max_);
          } else {
            // 加速開始点に達した。全開で速度を作る。
            runup_charge = true;
          }
          runup_need_gap_ = need_gap;      // 観測用
          runup_accel_at_ = accel_at;
          // 状態が変わったときだけ2秒に1回出す。
          const char * st = runup_charge ? "加速" : "待機";
          const double since = (now - runup_log_last_).seconds();
          if (runup_log_state_ != st && since >= 2.0) {
            runup_log_state_ = st;
            runup_log_last_ = now;
            diagLog("助走",
              "助走 target=%s 抜きどころまで=%.1fm 車間=%.1fm 要車間=%.1fm "
              "加速開始=%.1fm 自車=%.1fkm/h 相手=%.1fkm/h 目標=%.1fkm/h 状態=%s",
              c.blocker.c_str(), d_zone, gap, need_gap, accel_at,
              v_self_now * 3.6, v_opp * 3.6, v_tgt * 3.6, st);
          }
        }
      }
      // 予測入口へ横移動するまでは、新規試行条件(rear_end_margin+1m)を
      // 下回らない。旧実装は入口でpass_gapまで詰める一方、試行開始には
      // 4.5mを要求したため、必ず「開始車間不足」になっていた。
      const double predictive_start_gap = rear_end_margin_ + 1.5;
      if (predictive_approach) {
        eff_safe = std::max(eff_safe, predictive_start_gap);
      }
      // 車間が目標より大きければ助走(全開で詰める)。
      charge_now = gap > eff_safe + 0.3;
      // 入口が目前なら、逆算値ではなく「抜くのに要る車間」まで詰めきる。
      // want は入口到達時に pass_gap になる想定だが、追従則の遅れで
      // 実測では入口に 5-6m 残ったまま入っていた。ここだけ目標を
      // pass_gap*k に下げ、追従キャップを外す時間を伸ばす。
      // 追突(Crash)は前方接触のみ罰なので TTC・緊急減速(avoid_speed_cap)
      // が最後の砦になる。これらはチャージ中も別段で効く。
      if (d_zone < charge_close_dist_) {
        const double close_goal = predictive_approach
          ? predictive_start_gap : pass_gap_ * charge_close_gap_k_;
        charge_now = gap > close_goal;
      }
      // --- 助走は「止まれる距離」を割ったらやめる
      //
      // 【直したバグ(ユーザー報告「そもそもぶつかった原因は?」)】
      // 打ち切り条件が **固定の車間**(pass_gap*1.2 = 2.28m)だけで、
      // **接近速度を見ていなかった**。
      // 実測(20260828-215915-s0 の d1):
      //   gap=1.6m 相手=12.9km/h closing=23.1km/h v_reach=36.0km/h
      // 接近 6.4m/s から a_min=2.5m/s^2 で止まるには **8.2m** 要る。
      // 2.28m から減速を始めても間に合わず必ず当たる。
      // 実際 `前方 d3 まで 1.6m -> 0.0m` と詰まって接触した。
      //
      // **相手が遅いほど接近速度が大きくなり助走が危険になる**という
      // 逆説的な関係になっていた。制動距離で縛る。
      // 【追加 2026-09-03】助走の全開要求を反映する。
      // 直後の制動距離による打ち切りが**この後**に来るため、安全側の
      // 打ち切りはそのまま効く(殺していない)。
      if (runup_charge) { charge_now = true; }
      // 【追加 2026-09-03 観測のみ】助走が一度でも立ったかをファネルへ残す。
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
  // 【修正 2026-09-02 ユーザー指示】「最も遅い加速開始時刻まで相手速度で待つ」
  // を撤去した。狙いは狭い区間で相手へ追いつきすぎないことだったが、正しい
  // 制約は時刻ではなく車間であり、車間は追突防止層が別途受け持っている。
  // 実測では accel_delay が 174 回中 171 回で 0.0 秒(=今すぐ全開でぎりぎり)
  // であり、同期の余裕は存在せず、速度差という唯一の武器を捨てるだけだった。
  // 抜けると判定できたら待たずに加速する。
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
  // 横へ出て抜きにいっている最中は速度を抑えない。
  // 抑えると前車と同じ速度に張り付いて永久に抜けない(ブーストも無駄になる)。
  // 相手が遅い(止まりかけている)なら「追い越し中だから制限しない」を
  // 適用しない。実測(3レース中2レース): 減速中の相手に対し passing_now が
  // 立ち、車間 2.3〜4.0m で「上限=-1.0(制限なし)」のまま接触した。
  bool passing_now = allow && std::abs(offset_) > pass_gap_ * 0.5;
  if (ospeed_for_gate < slow_leader_speed_) { passing_now = false; }
  // --- 第3段: 発進の一発追い越し(ユーザー要望)
  //
  // 【実測 20260830-131907/d1】相手(MPC)の速度が
  //   stopped_speed(1.0m/s=3.6km/h) 〜 slow_leader_speed(2.5m/s=9km/h)
  // の帯にある間、直上の行で passing_now が落ち、下の commit も立たない。
  // 結果 上限 = 相手速度 + follow_kp*(車間 - eff_safe) に固定され、
  // グリッド車間 4.3m では相手より 1.2km/h しか出せない。
  // 実際 相手 3.8-7.5km/h の 20秒間、弧長差は +4.1〜+4.4m のまま一度も詰まらず、
  // 上限が外れたのは相手が 9.1km/h(=2.528m/s)に達した瞬間だった。
  // 横間隔が取れている間だけ、この帯を明示的に埋める。
  // 追突は preventRearEnd(層の最後)がそのまま見るので安全網は残る。
  const bool launch_pass_now =
      launchPassActive() && (&o == launchNpc()) &&
      std::abs(my_lat_for_target_ - olat) >= launch_p1_pass_sep_ && gap > 1.0;
  if (launch_pass_now) { passing_now = true; }
  // 直線で抜きにいっている間は速度を抑えない(ユーザー指示 2026-08-29)。
  // 追突は preventRearEnd が最後に見るので、ここで抑える必要はない。
  if (straight_pass_now_ && allow && ospeed_for_gate >= stopped_speed_) {
    passing_now = true;
  }
  // 【止まっている車の脇を、横に離れて通るとき】(ユーザー報告 2026-08-29:
  //  「P2 が P3 の後ろにわざわざ行ってしまう」)
  //
  // 上の行は「減速中の相手に張り付いて接触した」実測から入れたもので、
  // **完全に止まっている車**まで巻き込んで速度を抑えていた。
  // スタートでは最前列の運営NPC が数秒動かないことがあり、その間
  // 後続は 10.8km/h に抑えられたまま真後ろに並ぶ。
  //
  // 前から当てなければ Crash は付かない。横間隔が車幅(commit_sep)以上
  // 取れていて、相手が完全に止まっているなら、抑える理由が無い。
  // 車間の条件も付けて、真後ろに近い場面では従来どおり抑える。
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
  // 追い越し中も追従を完全には切らない。切ると相手が止まっても
  // 減速指令が一切出ないまま突っ込む。並走中は詰めてよいので、
  // 目標車間を 0.6 倍に縮めたうえで同じ制御を掛ける。
  // TTC・緊急の減速(avoid_speed_cap)はこれとは別に後段でかかる。
  const double follow_safe = passing_now ? eff_safe * 0.6 : eff_safe;

  // --- 横に出切ったら追従キャップを外し、加速して抜き切る ---
  //
  // 【原因】追従則は相手を最後まで「前の車」として扱う。横に並んでも
  // 上限は ospeed + follow_kp*(gap - follow_safe) のままで、
  // follow_safe = eff_safe*0.6 = 3.0m なので、車間が 3m を割った瞬間に
  // 上限が相手より遅くなる。釣り合うのは「車間 3m・相手と同速」の点。
  // つまり相手の斜め後ろ 3m に貼り付き、相手とぴったり同じ速度で
  // 走り続ける。ユーザー報告「相手の横まで来たのにゆっくり並走している」
  // はこの釣り合い点そのもので、自分から抜け出せる経路が無い。
  // 実測(3レース): 試行228回のうち90回が 16.0s ちょうどの時間切れ。
  //
  // 【対策】横間隔が min_lat_sep 以上あるなら、前から当てる経路が無い。
  // Crash(10秒・5km/h固定)は前方接触にしか付かないので、追従で
  // 速度を抑える理由がそもそも無い。上限を外して速度プロファイルどおり
  // 加速し、抜き切る。
  //
  // 外す条件は3つ。
  //   (1) 追い越しが許可されている(allow / latch)
  //   (2) 実測の横間隔が commit_sep(=min_lat_sep) 以上ある
  //       ※ 目標値 pass_sep_ ではなく、いま実際に離れている量で見る
  //   (3) 前後の車間が commit_gap 以内(本当に並びかけている)
  // 相手が止まりかけているとき(slow_leader 相当)は対象外にする。
  // 減速中の相手に上限なしで突っ込む過去の失敗を繰り返さないため。
  const double lat_sep_now = my_lat_for_target_ - olat;
  bool commit_now = false;
  if (commit_pass_ && ev.allow_commit &&
      (ospeed_for_gate > slow_leader_speed_ || launch_pass_now)) {
    // 解除側でも**車幅を下回らせない**。重なった状態で加速を続けないため。
    const double need_sep = commit_now_
      ? std::max(commit_sep_ * commit_release_, kCarWidth)
      : commit_sep_;
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

  // --- 並走が続いているのにに抜き切れないならブーストを使う
  //
  // 実測(2レース・自コード4台): 打切36件のうち **33件がブースト0個**。
  // しかも1台2個持ちで4台=8個あるうち、レース中に使われたのは **4個だけ**。
  // 横間隔の中央値は 1.96m あり(要1.15m)、**ちゃんと横には出ている**。
  // つまり「並んだのに抜けないまま16秒使い、ブーストは温存したまま
  // レースが終わる」という最悪の形になっていた。
  // ブーストは持ち越せないので、使わずに終わるのは丸損。
  //
  // 従来の発動条件 boost_would_help は「自力で抜けないと判断できるとき」で、
  // 仕掛ける前の見積りに基づく。見積りで「抜ける」と出ていても実際に
  // 抜けないのがここで見えているので、**実際に並走が続いた事実**を根拠に撃つ。
  // 【2026-08-31】commit_now は「車間 < commit_gap(5.0m)」で成立するので、
  // **相手の 4m 後ろ**でも「並走」と見なしてブーストを撃っていた。
  // 実測 (20260831-045422 d2 idx221):
  //   ブースト要求 横間隔=-1.60m 車間=4.2m 相手=18.0km/h -> 2秒後に Crash
  // 実測減速度 0.48m/s^2 ではブーストで足した速度を捨てられないので、
  // 横がわずかに崩れただけで追突が確定する。
  // 本当に横に並んでいる(接触は中心間 約2.6m で起きる)ときだけ撃つ。
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
  // 試行中に実際どこまで横に離れられたかを覚える。
  // 打切の原因を「並走したが抜けなかった」と
  // 「そもそも横に出られなかった」に分けるために要る。
  if (attempt_active_ && std::abs(lat_sep_now) > attempt_max_sep_) {
    attempt_max_sep_ = std::abs(lat_sep_now);
  }
  // 【ユーザー報告「スタートでP1がMPCを抜けず1位に周回差をつけられた」の主因】
  // 実測 20260830-200445: グリッドで **味方の d2 が 1.2m 前**にいるだけで
  // `前方 d2 まで 1.2m / 速度上限 3.3km/h` が **16秒**続き、
  // 30秒で 80m 離された。発進は全車が同時に加速するので、
  // 停止している前車に gap の P 制御を掛けるのは意味がない。
  // 安全側は wouldRearEnd(接近速度と制動距離で判定)に任せる。
  // 接近速度がある場合は従来どおり効くので、走行中には影響しない。
  const bool launch_free =
      launch_free_sec_ > 0.0 && launch_since_ >= 0.0 &&
      (now.seconds() - launch_since_) < launch_free_sec_ &&
      gap > launch_free_gap_ && [&]() {
        // wouldRearEnd は使えない。あれは stop_hold_margin という
        // **固定の車間マージン**で判定するので、グリッドの車間 1.2m では
        // 接近速度が 0 でも必ず真になり、この解除が一度も発火しなかった
        // (実測 20260830-201956: レース開始後も 3.3km/h が 6 秒続いた)。
        // ここでは実測の減速度(0.48m/s^2。開発メモ の a_min=2.5 は 5 倍過大)
        // で「今の接近速度なら車間内で止まれるか」を直接見る。
        // 【ユーザー指摘】相手が壊れて止まっていても、こちらがぶつかって
        // よい理由にはならない。実測 20260831-011257 で MPC が一度も動かず
        // 停止したままだったが、そのとき後続が追突していた。
        // 空走(指令が効くまで)と、余裕そのものを厚くする。
        const double closing = my_speed_for_gap_ - std::max(ospeed, 0.0);
        if (closing <= 0.0) { return true; }
        const double react = closing * launch_free_react_;
        const double brake = closing * closing / (2.0 * std::max(launch_free_decel_, 0.1));
        return (react + brake) < gap - launch_free_room_;
      }();
  if (gap < eff_follow && !charge_now && !commit_now && !launch_free) {
    double v_target = ospeed + follow_kp_ * (gap - follow_safe);
    // 完全に止まらないよう下限を設ける。本当に近い(接触寸前)ときは 0 まで許すが、
    // それは相手が動いている場合に限る。
    // 実測(3レース中2レース): 止まっている車の後ろで
    // 「車間 < follow_safe*0.5 -> 下限 0」となり、上の停止車両ブロックが
    // 横へよける目標を出しても速度が 0 のままで 24〜28 秒膠着した。
    // 相手が止まっているなら止まっても解決しないので、下限を残す。
    // 下限速度の扱い。
    //
    // 【元の実装】相手が止まっているときは下限を min_follow_speed(2.2m/s)に
    // 残していた。理由は「止まっている車の後ろで下限0にすると
    // 24〜28秒膠着した。相手が止まっているなら止まっても解決しない」。
    //
    // 【それが起こしていた問題(ユーザー報告)】
    // **スタート直後は全車が停止している**ので必ずこの分岐に入り、
    // 車間 1.2m で前車が完全停止していても 2.2m/s (8km/h) を出せと指令する。
    // その結果、スタートした瞬間に前の車へ追突していた。
    // 実測ログ: `停止車両 2台 先頭 d2 まで 1.2m 空き幅 0.62m -> 通過
    // (上限 10.8km/h)` のまま前進し、`膠着 ... 1.2m` に至る。
    //
    // 【直し方】膠着対策の下限は「詰まった状態がしばらく続いてから」
    // 効かせれば足りる。ぶつかる距離にいる間は止まってよい。
    // 近すぎる(stop_hold_gap 未満)ときは、詰まりが
    // stop_hold_sec 続くまで下限を 0 にする。
    double floor = min_follow_speed_;
    if (gap < follow_safe * 0.5 && ospeed > stopped_speed_) {
      floor = 0.0;                       // 相手は動いている。従来どおり
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
    // 落とすと車間が開いて仕掛けられないまま、後ろの車に抜かれる。
    // 接触寸前(gap < follow_safe*0.5)では従来どおり 0 まで落とす。
    if (c.pressed_from_behind && gap >= follow_safe * 0.5) {
      floor = std::max(floor, std::max(ospeed, 0.0));
    }
    // --- 車間が足りている間は、相手より遅い指令を出さない
    //
    // 【実測 2026-08-29(ユーザー報告「直線で一気に抜くはずが、ゆっくり
    //   追従しているだけ」「MPC を抜けずに周回遅れにされる」)】
    // 追従則は  上限 = 相手速度 + follow_kp * (車間 - follow_safe)  で、
    // **車間が目標より詰まっている間は相手より遅い指令**になる。
    // 実測(相手 18km/h): 車間 2.8m -> 上限 7.9km/h、車間 2.2m -> 7.7km/h。
    // 相手より 10km/h 遅い。下がる -> 車間が開く -> 加速 -> 詰まる、を繰り返し、
    // **平均すると相手より遅くなる**。これが「抜けない」だけでなく
    // 「周回遅れにされる」の正体で、助走も成立しない
    // (空けた瞬間に上限が上がって突っ込み、また 7km/h に落ちる)。
    //
    // 車間を詰めすぎないための減速は必要だが、**相手より遅くする必要は無い**。
    // 相手と同じ速度で走れば車間はそれ以上詰まらない。
    // 本当に追突が近い場面は preventRearEnd が層の最後で別に見ているので、
    // ここで二重に落とすと上のような釣り合い点に落ちるだけ。
    // 接触位置(中心間 2.6m)に余裕を足した距離を上回っている間は、
    // 相手速度を床にする。
    if (gap > follow_keep_gap_ && ospeed > 0.0) {
      floor = std::max(floor, ospeed);
    }
    // 【修正 2026-09-02】H-2a と同じ理由。追従則は「相手の速度 + 係数×車間誤差」
    // なので、車間が目標より詰まっていると相手より遅い上限を出す。横へ出て
    // 追い越している最中にそれをやると、抜くための速度差が消える。
    if (passUnderway(c.blocker, std::abs(lat_sep_now))) {
      v_target = std::max(v_target, ospeed);
    }
    // 【修正 2026-09-02】追い越し実行中(MOVE_OUT/PASS/MERGE)は、前車に快適に
    // 追従するための速度制限を外す。外すのは追従制御であって安全装置ではない。
    // 追突防止(preventRearEnd)と緊急TTC(avoidCollision)は状態に関係なく
    // 効き続ける。実測では全周期の大半でこの追従キャップが 12.8km/h を出して
    // おり、「横へ出るには速度が要るが、横へ出るまで速度が出ない」循環に
    // なっていた。
    // 【2026-09-02】speed_cap への直接代入をやめ、調停器へ要求として積む。
    // 【修正R 2026-09-03】接近中でも、横へ出てその相手の進路から外れていれば
    // 追従キャップを外す。真後ろ(pass_side_clear_ 未満)では従来どおり効く。
    // これが助走であり、抜きどころへ相対速度を持って到達するための条件。
    const bool clear_of_prepare_target =
      ovPreparingTarget(c.blocker) &&
      std::abs(lat_sep_now) >= pass_side_clear_;
    if (!ovPassingTarget(c.blocker) && !clear_of_prepare_target) {
      c.requestCap(std::max(floor, v_target), "追従");
    }
  }
}


// ===================================================================
// 当たらないようにする層
// ===================================================================

// 自分の進路上にいる車への追突を、**対象車かどうかに関係なく**防ぐ。
//
// 【なぜ要るのか(実測 2026-08-29、ユーザー報告「毎回スタートで P1 が P3 に
//   ぶつかって Crash を食らう」)】
// 追従と抜き切りの判断は `evaluateOpponent` の中で **1台ずつ**行われる。
// 抜き切りモードは「対象車と横に commit_sep 以上離れた」ことを根拠に
// **速度上限を外す**が、その判断に**対象でない車は入っていない**。
//
// 実測ログ(d1, レース開始から19.9秒):
//   並走から抜き切りへ target=d2 車間=3.2m 横間隔=2.86m 上限解除
//   前方 d3 まで 2.1 m / 速度上限 7.9 km/h
// d2 とは 2.86m 離れているので上限を外したが、**真正面 2.1m に d3(5.5km/h)**が
// いた。そのまま加速して追突し、Crash(10秒 5km/h 固定)になっている。
// 同じレースで3回、いずれも相手は d3(2.26m / 1.82m / 2.16m)。
//
// Crash は前方接触にしか付かないぶん、**前を見てさえいれば必ず防げる**。
// ここでは全車を見て、自分の車体が通る帯に入っている車のうち最も近いものに対し、
// 「止まれる速度」まで上限を落とす。抜き切りの判断より後に置いてあるので、
// どの層が上限を外していても最後にここで抑えられる。
// 指定 idx から先 look[m] の範囲にレーンがあるか。
//
// 【なぜ先読みが要るか(実測 2026-09-05)】BLOCK が1件出た(ハンデ中の先頭)。
// ガードは「レーンの idx にいて 28km/h 未満なら右へ出ない」だが、
// **横位置は指令から約20m走ってから実現する**ので、ゾーンに入ってから
// 引き戻しても間に合わない。入る前から寄せておく必要がある。
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

// 指定 idx が公式オーバーテイクレーンの中か。0 またぎは no_pass_zones と同じ扱い。
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

// いま公式オーバーテイクレーンを使ってよいか。
//
// 判定は「レーンの中の idx にいる」かつ「27km/h(+余裕) 以上出ている」だけ。
// 順位は見ない。1位はハンデで 25km/h に固定されるので速度条件を満たせず、
// 自動的にレーンを使わない側に回る。順位推定はずれることが分かっているので、
// 推定に依存しないこの形にしてある(ユーザー指摘: 左寄りのおかげで
// 1位は今まで BLOCK を受けていない。その性質は壊さない)。
bool V2XOvertaker::otLaneUsable(std::size_t idx) const
{
  if (!ot_lane_enable_ || ot_lane_zones_.empty()) { return false; }
  if (!inOtLane(idx)) { return false; }
  return my_speed_for_gap_ * 3.6 >= ot_lane_min_kmh_;
}

void V2XOvertaker::preventRearEnd(const Frame & f, PlanCtx & c)
{
  if (!rear_end_guard_) { return; }
  const Trajectory & in = f.in;
  const std::vector<double> & s = f.s;
  const size_t n = f.n;
  const size_t ei = f.ei;
  const double total = f.total;

  // 自分がこれから通る横位置。いまの位置と、寄せようとしている先の両方を見て
  // 「どちらかで重なる」なら進路上とみなす(寄せている最中に当てないため)。
  const double my_now = my_lat_for_target_;
  // 【調停 2026-09-02】target_offset は applyLatDecision までゼロのままなので、
  // 「これから寄せる先」は調停の途中値(採用中の意図を既存の制約で丸めた値)で見る。
  const double my_want = c.latWant();

  // --- 直線での追い越しは通しきる(ユーザー指示 2026-08-29)
  //
  // 「直線に限り、狭くなる側へ出ないクランプを無効にする。
  //  直線で追い越しモードになったらアクセル全開で、
  //  空いている側の壁とカートの間を駆け抜ける」。
  //
  // 直線は速度差が最も付く場所で、しかも並走時間が短い。
  // ここで詰まると1周ぶん失うので、多少壁に寄っても通す。
  const bool straight_pass = straight_pass_now_;
  // 通すときは「本当に車体が重なるか」だけで見る(車幅 1.30m + わずかな余裕)。
  const double sep_th = straight_pass ? straight_pass_sep_ : rear_end_sep_;

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
    // 通る帯に入っているか。いまの横位置と目標の横位置の近いほうで見る。
    const double sep = std::min(std::abs(my_now - olat), std::abs(my_want - olat));
    // 「走行ラインからの横ずれ」で進路上かを見るのは、直線では正しいが
    // **コーナーでは破綻する**(ラインが曲がっているので、横ずれが大きくても
    // 自分の鼻先が相手を向いていることがある)。
    // 実測 2026-08-29: 残った Crash は idx57 / 150 / 164 と
    // コーナーに集中し、相手はいずれも遅い NPC(18km/h)で距離 2.2〜3.2m。
    // 近距離では直線距離でも見る。ただし**並走して抜いている最中**まで
    // 抑えると追い越しが成立しないので、
    // 「相手が明らかに遅くて詰まっている」ときに限る。
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
      best_gap = gap; best_name = kv.first; best_olat = olat;
      best_speed = std::max(o.vx * 0.0 + std::hypot(o.vx, o.vy), 0.0);
      best_sep = sep;
    }
  }
  // --- 先で狭くなる側へは出ない(ユーザー報告の idx25-27 の Crash 対策)
  //
  // 【実測 2026-08-29】メインストレートで右へ 2.54m 出て抜きにいった直後、
  // コリドアの右限界が -2.55m(idx238)から -1.00m(idx25)へ縮む。
  // 壁のクランプに押し戻されて**相手の正面へ入り**、追突して Crash。
  // 横目標は「いまの地点の帯」でしか丸めていなかったのが原因。
  // 前に車がいる間だけ、**これから通る区間で最も狭い帯**に丸める。
  // 前が空いているときは従来どおり(レースラインの自由度を残す)。
  // 直線ではクランプを掛けない(ユーザー指示)。壁ぎりぎりまで使って通す。
  // 【範囲を絞った(実測 2026-08-29)】このクランプは「前の車と横に重なっている
  // 間」に効く。それは**これから横へ出ようとする瞬間**でもあるので、
  // 距離を問わず掛けると追い越しの入口そのものを塞ぐ。
  // 実測: 1レースで 116 回発火し、3台とも 67秒/周の団子のまま
  // 「抜いた」が 0 回、追越失敗 25 回。
  // 押し込まれる危険が現実にあるのは車間が詰まってからなので、
  // squeeze_gap 以内に限る。
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
      // 【調停 2026-09-02】これは「ここへ行きたい」ではなく
      // 「この範囲から出るな」なので、意図ではなく制約として積む。
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

  // --- 車間が詰まっているときは相手の横位置をまたがない
  //
  // 【なぜ(実測 2026-08-29)】横に 2.6m 渡るには offset_rate(1.2m/s)で
  // 最短 2.17秒かかる。観測された接近速度 3〜4.5m/s では、その間に車間が
  // **6.5〜9.8m** 縮む。つまり **車間 8m 未満で始めた側の入れ替えは、
  // 定義上すべて接触範囲(車間 2.6m 以下・横間隔 1.3m 以下)の中で実行される**。
  // 成功しようがないうえ、相手の正面を横切ることになる。
  // 実測では側変更 12件のうち車間 5.7m 以下が4件あり、**その4件すべてが
  // Crash / Wall の窓の中**だった(残る8件は 13.5〜24.9m で無害)。
  //
  // 横へ出る量そのものは制限しない(棄却済みの squeeze_ahead とは別物)。
  // 禁じるのは「相手を貫通して反対側へ移ること」だけ。
  if (cross_gap_ > 0.0 && !best_name.empty() && best_gap < cross_gap_) {
    const double my_lat = my_lat_for_target_;
    if (std::abs(my_lat - best_olat) > cross_dead_) {
      // 【調停 2026-09-02】「相手の横位置をまたぐな」も範囲の制限なので制約。
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
    // 【ここでリセットしてはいけない(実測 2026-08-29)】
    // 追い越しで横に出ている間は対象が外れる。そこで guard_cap_ を消すと、
    // ラインへ戻った瞬間に**自車速度から**やり直すことになり、
    // そこから 2.5m/s^2 でしか下げられないので間に合わない。
    // 実測: 追突防止 32件のうち 14件が「上限 99.0 ->」(直前は無制限)で、
    // うち7件は出した上限が**自車速度以上**だった。
    //   846.8 gap=1.5 横間隔1.44 相手17.6 上限 99.0->19.5 自車19.9 -> 直後に Crash
    //   953.5 gap=2.0 横間隔1.28 相手27.3 上限 99.0->35.4 自車35.9 -> 直後に Crash
    // 値は残す。状況が安全なら次の周期で v_allow が上回って即座に戻る
    // (下の「上げるのは即時」の分岐)。
    return;
  }

  // その車間で止まれる速度。相手も動いているので相手速度を足す。
  // 【カートには制動力がほとんど無い(実測 2026-08-29)】
  // 追突防止が減速を掛けている区間から実測した減速度は
  //   中央値 0.48 m/s^2 / 平均 0.61 / p90 1.09  (1.3 超は 69件中5件だけ)
  // 例: 上限15km/h を3秒維持して 26.0 -> 24.3 -> 21.6 km/h = 0.35〜0.43 m/s^2。
  // pure_pursuit は 1.0*(v_target - v_now) を出し AWSIM が -1.37 に丸めるので
  // 指令としては十分なはずだが、実車がその減速を出していない。
  //
  // ここで |a_min|(2.5) をそのまま使うと**制動距離を3.1倍過小評価**する。
  // rear_end_brake_k はその補正で、実測の中央値の少し上に置く。
  // 定数(rear_end_margin)を足すのと違い、こちらは Δv^2 に比例して効くので、
  // 速度差が大きい危険な場面ほど早く落とし始める:
  //   自車30/相手18 (Δv3.3m/s): 発動車間 7.7m -> 14.0m
  //   自車25/相手23 (Δv0.55m/s): 3.9m -> 4.0m (後ろで待つ場面はほぼ不変)
  // 「待っているだけの場面」を縛らずに「詰まっている場面」だけ早める形になる。
  // --- 詰めてよいのは「抜けると分かっているとき」だけ(実測 2026-09-05) ---
  //
  // 【実測で分かった構造】
  //  ・`rear_end_brake_k=0.22`(制動 0.55m/s^2 前提)では、実測能力 1.66 の 1/3 なので
  //    **必要より約3倍手前から速度を落とす。** 先頭に 1.8m までしか近づけず抜けない。
  //  ・0.45(1.09m/s^2 前提)に上げると **0.4m まで詰められ、追い越しも 1→3〜4件/レース**
  //    に増えた。診断は正しかった。
  //  ・ただしペナルティ回数が 1.67 → 3.25/車レース、リタイアが 0/12 → 5/16 に悪化した。
  //    **詰めた後に安全に横へ出る手段が無いので、詰めた分そのままぶつかる。**
  //
  // したがって制動の前提は状況で切り替える。
  //  ・**抜く算段が付いている**(試行中で、選んだ側に余地があり、幅も足りている)
  //    → 実測に近い値で詰めてよい。抜くための速度を捨てない。
  //  ・そうでない(ただ後ろに付いているだけ)
  //    → 従来の保守的な値。詰める意味が無いのでリスクだけ取らない。
  const bool pass_underway = attempt_active_ && side_fits_ && dbg_width_ >= min_pass_width_;
  const double brake_k = pass_underway ? rear_end_brake_k_pass_ : rear_end_brake_k_;
  const double brake_a = std::max(std::abs(a_min_) * brake_k, 0.3);
  // 接触は**中心間 約2.6m**で起きる(自車前端は原点から1.6m、相手の後端まで約1.0m)。
  // rear_end_margin はこの接触位置に残す余裕。
  // 【3.5 にした理由】接触の余裕ではなく **best_gap の量子化**の吸収。
  // best_gap は s[oi]-s[ei] とインデックスに丸めた値なので **1.45m 刻み**に
  // 量子化される(実測: ログの gap は 1.1-1.5 / 2.7-3.2 / 3.6-4.1 に離散化し、
  // 4.2〜6.0 の値が1件も無い)。誤差 ±0.7m は「余裕 0.2m」より大きい。
  // これ以上上げると、抜き切り 182件(車間は全件 5m 以内)がすべて
  // 後退枝に入り、追い越しが成立しなくなる。
  // 反応と減速の立ち上がりで進む距離を引く。
  // 【実測 2026-08-29】idx25-26 で、右へ 2.54m 出て抜きにいった直後に
  // コリドアの右限界が -2.55m(idx238)から -1.00m(idx25)へ縮み、
  // 壁のクランプで相手の正面へ押し戻されて追突した。
  // 横に重なった時点から落とし始めたのでは 21km/h では間に合わない。
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
  // 【レート制限はここに置かない(バグ修正 2026-08-29)】
  // 以前ここに `guard_cap_` の減衰(2.5m/s^2)を置いていたが、
  // **これが上限を最大 +24.8km/h 水増ししていた**。
  // 実測(1レース・追突防止19行を式と突き合わせ):
  //   車間2.9m 横間隔1.40 相手15.7km/h 自車31.4km/h
  //   -> 式の v_allow は 12.2km/h なのに、出力は 37.0km/h
  // room が負(既に食い込んでいる)のときの枝は「相手速度 - 食い込み」と
  // 正しく低い値を出しているのに、前の周期から引きずった guard_cap_ が
  // それを上書きしていた。横に出て対象が外れ、戻った瞬間に古い高い値が残る。
  //
  // 減速率の制限は publishTrajectory 側にある(cap_rate_limit_)。
  // あちらは**毎周期の実速度に錨を打っている**ので陳腐化せず、
  // 直前が無制限だった場合も守られる。車が出せる減速率は変わらないまま、
  // 目標値だけが正しく下がる。
  // 【2026-08-31 膠着の真因】停止車の 3.1m 後ろで **自車 -0.00m/s が 3.0秒** という
  // ログが1レースあたり約40件出ていた。停止車回避は「空き幅 1.83m -> 通過
  // (上限 10.8km/h)」と出しているのに一歩も動かない。
  // 原因はここ: 相手が止まっている(best_speed=0)ので
  //   room = best_gap(3.1) - rear_end_margin(3.5) - react < 0
  // となり v_allow = 0 になる。つまり停止車の 3.5m 手前で必ず停止する。
  // そして **速度 0 のカートは横に動けない**(横移動は前進でしか起きない)。
  // 脇に通れる帯があるのに、そこへ寄るための前進が禁じられる循環になっていた。
  // 通れる帯があるあいだは、寄るのに必要なだけの微速を許す。
  if (c.stop_avoid_active && c.stop_avoid_have_gap &&
      best_gap > stop_creep_gap_ && v_allow < stop_creep_speed_) {
    v_allow = stop_creep_speed_;
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

  // --- 横にずれている分だけ追突防止を緩める(2026-08-31)
  //
  // 【ユーザー報告】「低速のMPCを、壁との間も十分空いているのに抜けない」。
  //
  // 【実測】
  //   追越試行 打切 所要=16.0s 理由=時間切れ 最大横間隔=5.14(要1.30)
  //   追突防止 d2 まで 2.8m 横間隔 1.15m 相手 10.4km/h 上限 8.7 -> 7.8km/h 自車 5.0km/h
  // 横に 1.15m ずれて並びかけているのに「自分の進路上」と判定され、
  // 速度が 7.8km/h に抑えられていた。相手は 10.4km/h なので
  // **抜こうとしている最中に相手より遅くなり、永久に前へ出られない**。
  // 16秒の打切りはこれ。横間隔の判定が min_lat_sep を境にした二値で、
  // 1.15m は「進路上」に倒れる。
  //
  // 【対策】横間隔が min_lat_sep の rear_end_free_frac 倍を超えたら、
  // そこから min_lat_sep までの間で上限を線形に開放する。
  // 完全に外れる前でも、ずれた分だけ前へ出られるようになる。
  // 追い越し試行中に限る(通常の追従では従来どおり厳しく効かせる)。
  // 開放しても相手速度は下回らせない(それでは抜けないため)。
  //
  // 【2026-08-31 15:xx の誤りと訂正】最初は min_lat_sep(1.15m)の 55% から
  // 緩め始めるようにした。**これは物理的に誤りだった。**
  // カートの車幅は 1.46m なので、横間隔 0.73m は **車体が 0.73m 重なっている**
  // 状態であり、そこで加速を許せば当たる。実測でも
  //   `追突防止を緩めた 横間隔0.73m / 0.83m / 0.94m` が61回出ており、
  // ユーザー報告「衝突回数が増えた」と一致した。自車平均も 23.5 -> 17.1km/h に落ちた。
  // 基準にした min_lat_sep(1.15m)自体が車幅より小さい。
  //
  // 正しくは **車体が触れない距離** から緩め始める:
  //   車幅 1.46m + 余裕 = rear_end_free_min(既定 1.66m)
  // それ未満では一切緩めない(従来どおりの追突防止)。
  if (rear_end_lat_release_ && attempt_active_ && best_sep > rear_end_free_min_) {
    {
      const double t = std::clamp(
          (best_sep - rear_end_free_min_) /
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
  // 【修正 2026-09-02】横へ出て追い越している相手に対しては、その相手より
  // 遅い上限を出さない。横に並んだ相手と同速で走ることはその相手への追突に
  // ならない。真後ろ(横間隔 pass_side_clear 未満)と第三者車に対しては
  // 従来どおり制動側の上限をそのまま出す。
  // 【修正 2026-09-02】状態機械が追い越し実行中と判断している相手にも同じ床を
  // 与える。passUnderway は横間隔 pass_side_clear_ 以上を要求するため、横へ
  // 出ている途中(MOVE_OUT)では成立せず、まさに速度が要る場面で床が無かった。
  // 上限そのもの(c.requestCap の "追突防止")は残すので安全網は変わらない。
  // 【修正 2026-09-02】床(相手の速度まで)では追い越せない。
  //
  // 追い越すには相手より**速く**走る必要があるのに、床は「相手と同じ速度まで
  // は許す」でしかなかった。実測(20260902-2230/2237/2244)では PASS 状態
  // 476 周期の 55% で追突防止が 17.4km/h を出しており、相手 d2 が 24km/h で
  // 走るため物理的に抜き切れず、並走したまま 16 秒でタイムアウトしていた
  // (中断理由は全件「時間切れ」、所要ちょうど 16.0 秒)。
  //
  // 横に並んでいる相手はそもそも追突の対象ではない。band から追い越し対象を
  // 除外したのと同じ理由で、ここでも**計算から外す**のが正しい。
  // 真後ろ(横間隔が pass_side_clear_ 未満)のときは従来どおり制動側が効く。
  const bool passing_beside =
    (passUnderway(best_name, best_sep) ||
     (ovPassingTarget(best_name) && std::abs(best_sep) >= pass_side_clear_));
  if (passing_beside) {
    // 並走している対象に対しては上限を出さない。
    v_allow = -1.0;
  } else if (ovPassingTarget(best_name)) {
    // まだ横へ出切っていない追い越し対象。相手速度は下回らせない。
    v_allow = std::max(v_allow, best_speed);
  }
  // 【2026-09-02 ユーザー指示】ここは「最後の砦」として他層の値を上書き
  // 代入していたが、min 合成の調停器なら層の順序に関係なく最も厳しい値が
  // 勝つので、要求にしても安全性は下がらない。代入をやめて要求にする。
  // v_allow が負のときは要求そのものを出さない(requestCap は負値を無視する
  // 実装だが、意図を明示しておく)。
  if (v_allow >= 0.0) { c.requestCap(v_allow, "追突防止"); }
  // 近いときは、効いていなくても状況を残す(原因の切り分けのため)。
  // 「効いているか」は最終確定(applyCapRequests 後)でないと分からないため、
  // ここでは v_allow が現在の自車速度を下回っているか/車間が詰まっているかで
  // 記録の要否だけ判断する(従来の before/after 比較から変更)。
  if ((f.now - last_rearend_log_).seconds() > 1.0 &&
      (my_speed_for_gap_ > v_allow + 0.5 || best_gap < 4.0))
  {
    last_rearend_log_ = f.now;
    RCLCPP_INFO(get_logger(),
      "追突防止 %s まで %.1fm 横間隔 %.2fm 相手 %.1fkm/h "
      "要求上限 %.1fkm/h 自車 %.1fkm/h",
      best_name.c_str(), best_gap, best_sep, best_speed * 3.6,
      v_allow * 3.6, my_speed_for_gap_ * 3.6);
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

  // --- 停止車両への突入防止(複数台が同じ場所で止まっている場合を含む)
  // 大会での実測: 相手がスタックして止まっていると、そこへ突っ込んでいた。
  // 原因は3つ重なっていた。
  //  (1) 追い越し中(passing_now)は速度制限を一切かけない作りだった。
  //      止まっている相手は slow_leader となり距離制限も外れるので、
  //      抜けるかどうかに関わらず全開で近づいていた。
  //  (2) 追従制御の下限 min_follow_speed(2.2 m/s) が、
  //      車間 2m を切るまで解除されない。止まっている相手には遅すぎる。
  //  (3) 回避層の探索範囲が 8m・TTC 1.0 秒しかない。35km/h では 0.8 秒前で、
  //      制動距離(約19m)にまるで足りない。
  //
  // 1台ずつ「横にずれているか」で判定すると、複数台が並んで止まっている場合に
  // 破綻する。A が左、B が右にいるとき、それぞれとの横ずれは足りていても
  // 2台の「間」が通れるとは限らず、逆に間を狙って両方に当たる。
  // そこで、止まっている車の集団に対して「空いている横位置の区間」を計算し、
  //   通れる区間がある -> そこへ寄せる
  //   無い            -> 手前で止まる
  // とする。
  // 停止車回避 -> 壁回避 へ渡す情報。
  // この2つのブロックは今まで会話しておらず、停止車回避が「ここへ寄れば通れる」と
  // 決めて速度上限を 10.8km/h に引き上げた直後に、壁回避が横目標だけを黙って
  // 潰していた。避けられない位置のまま全開で前進する、という最悪の組み合わせ。
  // 潰されたかの判定には、壁回避の時点での横目標(want)をそのまま使う。
  // 停止車回避の答えを別に持ち回るより、「最終的に出したい横位置が壁で
  // 潰されたか」を見るほうが、途中の層が書き換えた場合も正しく効く。
  {
    // 【カートには制動力がほとんど無い(実測 2026-08-29)】
    // 実測減速度は中央値 0.48 m/s^2 / p90 1.09。|a_min|(2.5) をそのまま使うと
    // 制動距離を約5倍過小評価し、回避できない停止車へ全開で近づく。
    // ここで出る v_stop の消費先は「横に回避できない枝」と
    // 「壁クランプで横目標が潰された枝」だけで、通れる枝の速度は
    // stopped_thread_speed_ から出る。よって下げても追い越し能力は落ちない
    // (rear_end_brake_k と同じ流儀)。
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
    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid || (now - o.stamp).seconds() > v2x_timeout_) {
        continue;
      }
      if (std::hypot(o.vx, o.vy) > stopped_speed_) {
        continue;              // 動いている。通常の追従制御に任せる
      }
      // 【修正 2026-09-02】いま追い越している相手は障害物ではない。相手が遅い
      // ことは追い越す理由であって避ける理由ではない。実測(3レース)で
      // 「安全中断 理由=停止車回避」が中断理由の最多(10件中4件)を占め、
      // 抜こうとしている当の相手を停止車として回避していた。
      // 第三者の停止車に対する回避は従来どおり効く。
      if (ovPassingTarget(kv.first)) { continue; }
      const size_t oi = nearest(in, o.x, o.y);
      double gap = s[oi] - s[ei];
      if (gap < 0) {
        gap += total;
      }
      if (gap > stopped_look_ahead_ || gap < 0.5) {
        continue;
      }
      double nxo, nyo;
      normalAt(in, oi, nxo, nyo);
      const auto & lpo = in.points[oi].pose.position;
      const double olat_s = (o.x - lpo.x) * nxo + (o.y - lpo.y) * nyo;
      stopped.push_back({gap, olat_s, oi, std::hypot(o.vx, o.vy), kv.first});
    }

    // --- ラッチの「抜け切り」解除(外部レビュー レビュー 2026-09-05) ---
    //
    // 抜き切ると対象の gap が周回長近くになり探索対象から外れる。
    // その周期は stopped が空なので、内側に置いた解除処理は走らない。
    // 結果、同じ名前の車が次周も停止していると古いラッチがそのまま効く。
    // 「一定時間その対象を見ていない」を解除条件にする。
    if (stop_nopass_latched_ && stop_nopass_seen_.nanoseconds() > 0 &&
        (now - stop_nopass_seen_).seconds() > 1.0)
    {
      stop_nopass_latched_ = false;
      RCLCPP_INFO(get_logger(),
        "停止車 %s を見なくなったので「通れない」を解除する",
        stop_nopass_target_.c_str());
      stop_nopass_target_.clear();
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
      // --- (1) 壁帯は「抜け切るまでの区間の最狭」で取る(2026-09-05) ---
      //
      // 停止車の1点だけで見ていたため、入口は広くて出口で壁が迫る場所に
      // 全開で入っていた。実測では 11.3m 手前で「空き幅3.25m→通過」と出した
      // 直後、6.8m 手前で壁帯が [-1.60,0.40] に潰れている。
      // 停止車の手前から抜け切るまでを覆う区間で最も狭い帯を使う。
      if (stop_avoid_fix_ && corridor_.lo.size() == n) {
        // 【外部レビュー レビュー 2026-09-05】6m では停止車クラスタ(8m先まで同じ
        // 障害物群として扱う)を覆えない。+3〜+8m の2台目が横を塞ぐのに、
        // その地点の壁帯を評価していなかった。クラスタ + 車体通過長まで見る。
        // 【計測で戻した 2026-09-05】クラスタ(8m)まで広げると「通れない」が増え、
        // 停止時間が伸びて退行した。span_eff はパラメータで広げられるようにしておく。
        const double span_eff = std::max(stop_avoid_span_, stop_avoid_span_);
        double acc = 0.0;
        // 手前側は半区間ぶん戻ってから、前方へ span ぶん見る。
        size_t k0 = bi;
        double back = 0.0;
        while (back < stop_avoid_span_ * 0.5) {
          const size_t prev = (k0 + n - 1) % n;
          back += std::hypot(in.points[k0].pose.position.x - in.points[prev].pose.position.x,
                             in.points[k0].pose.position.y - in.points[prev].pose.position.y);
          k0 = prev;
        }
        for (size_t k = 0; k < n; ++k) {
          const size_t a = (k0 + k) % n, b = (k0 + k + 1) % n;
          lo = std::max(lo, corridor_.lo[a] + safetyAt(a));
          hi = std::min(hi, corridor_.hi[a] - safetyAt(a));
          acc += std::hypot(in.points[b].pose.position.x - in.points[a].pose.position.x,
                            in.points[b].pose.position.y - in.points[a].pose.position.y);
          if (acc > span_eff) { break; }
        }
      }

      // 手前の集団(先頭から stopped_cluster_span 以内)が塞ぐ横位置
      std::vector<std::pair<double, double>> blocked;
      int group = 0;
      for (const auto & st : stopped) {
        if (st.gap - base > stopped_cluster_span_) {
          break;
        }
        // 停止車の周囲の「通れない帯」は **車幅**で取る。
        //
        // 【直したバグ(ユーザー報告: スタート直後に前の車へ追突)】
        // ここは min_pass_sep(1.15m)を使っていた。カート幅は 1.30m なので
        // **0.15m 足りない**。その結果「空き幅がある = 横へ避けて通れる」と
        // 判定して横へ寄せても、車体が当たる。
        // 実測ログ: `停止車両 2台 先頭 d2 まで 1.2m 空き幅 0.62m -> 通過
        // (上限 10.8km/h)` のまま前進して接触。
        // **横に避ける処理はあったが、避け先の計算が物理的に足りていなかった。**
        //
        // min_pass_sep が車幅を下回っているのは「抜けるかを**計画**する」ための
        // 意図的な値(開発メモ)。実際に車体を通す判定に流用してはいけない。
        blocked.emplace_back(st.lat - kCarWidth, st.lat + kCarWidth);
        group++;
      }
      std::sort(blocked.begin(), blocked.end());

      // 空いている区間のうち最も広いものを探す
      double best_w = -1.0, best_a = 0.0, best_b = 0.0;
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

      const double d = base - stop_margin_;
      const double v_stop = (d > 0.0) ? std::sqrt(2.0 * brake_a * d) : 0.0;

      // 通す位置は**帯の中央**にする(ユーザー指示)。
      //
      // 【直したバグ(ユーザー報告: 右に余裕があるのに避けずに当たる)】
      // ここは `std::clamp(my_lat_for_target_, best_a, best_b)` で
      // 「今の横位置から一番近い端」を狙っていた。ところが帯の端は
      // `st.lat ± kCarWidth` の境界、つまり **相手と中心間ちょうど 1.30m =
      // 車体が触れる位置**そのもの。低速の追従誤差は ±0.2〜0.3m あるので、
      // 狙った時点で当たる。「動く量は最小に」した結果が接触ぎりぎりだった。
      // 帯の中央なら左右に best_w/2 の余裕が残る。
      const double band_center = (best_a + best_b) * 0.5;
      std::string act;

      // --- (2)(3) 到達可能性の判定と「通れない」のラッチ(2026-09-05) ---
      //
      // (2) 横位置は指令から約20m走ってから実現する。狙う横位置まで
      //     寄せ切るのに要る距離を先に見て、残距離が足りないなら
      //     「通れる」と判断してはいけない。
      //     必要距離 = 遅れ(20m) + |Δ横| / offset_rate * 速度
      // (3) 一度「通れない」と決めたら、抜け切るか止まるまで解除しない。
      //     従来は毎周期やり直していたので上限が「なし→8.6→なし」と振動し、
      //     2.6m 手前で上限なしに戻って正面衝突の緊急回避に至った。
      bool reach_ok = true;
      double need_dist = 0.0;
      if (stop_avoid_fix_) {
        const double dy = std::abs(band_center - my_lat_for_target_);
        const double v = std::max(my_speed_for_gap_, 0.5);
        need_dist = stop_avoid_lat_lag_ + (dy / std::max(offset_rate_, 0.1)) * v;
        reach_ok = (base >= need_dist) ||
                   (dy < 0.15) ||   // すでにその横位置にいる
                   (my_speed_for_gap_ < 1.0);  // ほぼ止まっているなら遅れは効かない
        // ラッチの解除条件: 対象が変わった / ほぼ止まった。
        // 「抜け切った」の解除は下(stopped が空の場合を含む)で時刻で見る。
        if (stop_nopass_latched_ &&
            (stop_nopass_target_ != stopped.front().name || my_speed_for_gap_ < 0.5))
        {
          stop_nopass_latched_ = false;
        }
        // 対象を今この周期で見たことを記録する。
        for (const auto & st : stopped) {
          if (st.name == stop_nopass_target_) { stop_nopass_seen_ = now; break; }
        }
      }
      const bool pass_geom_ok = (best_w >= stopped_slack_);
      const bool pass_ok = stop_avoid_fix_
                             ? (pass_geom_ok && reach_ok && !stop_nopass_latched_)
                             : pass_geom_ok;
      if (stop_avoid_fix_ && !pass_ok && !stop_nopass_latched_) {
        stop_nopass_latched_ = true;
        stop_nopass_target_ = stopped.front().name;
        if ((now - last_stop_fix_log_).seconds() > 1.0) {
          last_stop_fix_log_ = now;
          RCLCPP_WARN(get_logger(),
            "停止車 %s まで %.1fm 通れないと決めた(抜けるか止まるまで解除しない) "
            "空き幅=%.2f(要%.2f) 壁帯=[%.2f,%.2f] 横目標=%.2f 自車横=%.2f "
            "寄せ切るのに要る距離=%.1fm",
            stopped.front().name.c_str(), base, best_w, stopped_slack_,
            lo, hi, band_center, my_lat_for_target_, need_dist);
        }
      }
      if (pass_ok) {
        // 余裕をもって通れる。帯の中央へ寄せる。
        c.requestLat(band_center, PlanCtx::LatPrio::kStoppedCar, "停止車回避");
        c.stop_avoid_active = true;
        c.stop_avoid_v_stop = v_stop;
        c.stop_avoid_have_gap = true;
        c.stop_avoid_lo = best_a;
        c.stop_avoid_hi = best_b;
        c.stop_avoid_dist = base;
        // 横へよける目標を出しても、上の追従制御が「相手が止まっている =
        // 車間が近い」で上限 0 を出していると一歩も動けない。
        // 実測(3レース中2レース): この状態で 24〜28 秒膠着した。
        // 通れると判断したのだから、最低でも徐行できる速度は残す。
        // speed_cap < 0 は「制限なし」なのでそのまま(下げてはいけない)。
        //
        // 【ただし近すぎるうちは引き上げない(ユーザー報告: スタート直後の追突)】
        // 膠着対策がこことは別に追従制御の下限にも入っており、
        // **2箇所で別々に「止まるな」と強制**していた。追従側の下限を
        // 0 にしても、この `std::max` が 3.0m/s (10.8km/h) へ引き上げるので
        // 効かなかった。スタート時は全車が停止しているので必ずここを通る。
        // 実測ログ: `停止車両 2台 先頭 d2 まで 1.2m 空き幅 0.62m ->
        // 通過 (上限 10.8km/h)` のまま前進して追突。
        //
        // 膠着対策は「詰まりがしばらく続いてから」効かせれば足りる。
        // ぶつかる距離にいる間(stop_hold_gap 未満)は、stop_hold_sec 秒だけ
        // 引き上げを見送る。時間が過ぎれば従来どおり引き上げるので
        // 24〜28秒膠着した問題も再発しない。
        // 【スタートが遅くなった原因(ユーザー報告)】
        // スタート時は全車が停止しているので必ずここに入り、
        // 4秒間まるごと速度を上げずに待っていた。2位スタートでも
        // MPC を抜けないほど出遅れる。
        // 待つ必要があるのは「自分が動いていて、止まっている車に
        // 突っ込む」場合だけ。自分が止まっているなら待つ意味がない
        // (前の車も同時に発進するので、追従制御に任せれば足りる)。
        // 固定車間 + タイマーをやめ、制動距離で判定する(wouldRearEnd 参照)。
        // 155 の教訓どおり、同じ意図の対策が追従側にもあるので両方そろえる。
        bool hold_now = false;
        if (wouldRearEnd(base, stopped.front().speed)) {
          if (stop_hold_since_ < 0.0) { stop_hold_since_ = now.seconds(); }
          hold_now = (now.seconds() - stop_hold_since_) < stop_hold_sec_;
        } else {
          stop_hold_since_ = -1.0;
        }
        // 【実測 20260830-181222 / ユーザー報告「P3がスタックしていて
        //   それを避けるために過度にスピードが落ちている」】
        // 止まった d3 の脇を空き幅 2.05〜2.74m で通れるのに、
        // 上限 10.8km/h (= stopped_thread_speed 3.0m/s) に張り付いたまま
        // 65 回。帯は相手の中心から ±kCarWidth(1.30m) を除いてあるので、
        // 帯に入りきっていれば車体間は 0.5m 以上空く。
        // **止まっている物体の脇を徐行する理由はない**ので、空き幅に
        // 応じて上限を上げる。ただし帯にまだ入りきっていない間
        // (横に移動中)は従来どおり徐行のままにする。
        double v_thread = stopped_thread_speed_;
        const double room = best_w - stopped_slack_;
        const bool in_band = (my_lat_for_target_ >= best_a + stopped_clear_in_) &&
                             (my_lat_for_target_ <= best_b - stopped_clear_in_);
        if (room > 0.0 && in_band) {
          v_thread = std::min(stopped_clear_speed_,
                              stopped_thread_speed_ + room * stopped_clear_gain_);
        }
        // 【2026-09-02 ユーザー指示】ここは他層が落とした上限を std::max で
        // 上げ直していた(上限を戻す = 調停の後勝ちに順序依存で負ける事故の
        // 原因の一つ)。調停器の下では「上げ直す」要求は意味を持たない
        // (min 合成なので他層の要求より緩い値は採用されない)ため撤去する。
        act = hold_now ? "通過(近いので待つ)"
                       : (in_band && v_thread > stopped_thread_speed_ + 1e-6
                            ? "通過(帯の中・加速)" : "通過");
      } else if (best_w > 0.0) {
        // かろうじて通れる。速度を落として通す。ここも帯の中央を狙う。
        c.requestLat(band_center, PlanCtx::LatPrio::kStoppedCar, "停止車回避");
        c.stop_avoid_active = true;
        c.stop_avoid_v_stop = v_stop;
        // 【2026-09-05】「通れない」とラッチされている間は have_gap を主張しない。
        // これは preventRearEnd の「通れる帯があるので微速を許す」免除に効く。
        // 通れないのに微速で寄っていくと、壁と相手の間へ押し込まれる。
        c.stop_avoid_have_gap = !(stop_avoid_fix_ && stop_nopass_latched_);
        c.stop_avoid_lo = best_a;
        c.stop_avoid_hi = best_b;
        c.stop_avoid_dist = base;
        c.requestCap(stopped_thread_speed_, "停止車回避");
        act = "徐行通過";
      } else {
        // 通れない。手前で止まる。
        c.requestCap(v_stop, "停止車回避");
        act = "停止";
      }

      // --- (4) 最大制動の監督(2026-09-05) ---
      //
      // 速度上限は「目標」であって、下位制御がどれだけの遅れで実現するかに
      // 依存する。避けられないと分かった時点で残距離が制動距離を割っているなら、
      // 上限を下げるだけでは足りないので**最大制動を明示的に要求する**。
      //
      // 発火は TTC ではなく「回避経路が無い かつ 残距離 <= 制動距離 + 余裕」。
      // 36km/h(10m/s)から 2.4m/s^2 で止まるだけで 20.8m 要る。TTC 0.68s で
      // 気づく設計では原理的に間に合わない。
      //
      // 後方から追突されるリスクはあるが、追突の罰は当てた側にしか付かない
      // (公式のペナルティ表: 自車の前バンパーが他車の後バンパーに接触したとき)。
      // 自分が正面衝突を選ぶ理由にはならないので、ここでは後方を理由に
      // 最大制動を拒否しない。早めの弱い減速で境界に入らないのが本筋。
      if (stop_avoid_fix_ && !pass_ok) {
        const double v = std::max(my_speed_for_gap_, 0.0);
        // 【外部レビュー レビュー 2026-09-05】ここは a_min(2.5) を使っており楽観的だった。
        // **加速度指令は AWSIM 側で 1.37 に切り捨てられる**(開発メモ の実測)ので、
        // 要求できる減速度はそれが上限。発火距離もそれで計算する。
        // 10m/s なら制動距離は 20m ではなく約 36m。
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
                    "停止車両 %d台 先頭 %s まで %.1fm 空き幅 %.2fm -> %s (上限 %.1fkm/h)",
                    group, stopped.front().name.c_str(), base, best_w, act.c_str(),
                    (c.speed_cap < 0.0 ? 99.0 : c.speed_cap) * 3.6);
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

  // --- 衝突回避層(他車 + 壁)
  // 追い越しロジックとは独立に、常に働く。追い越し中かどうかに関係なく
  // 「当たりそうなら必ず避ける/減速する」を最優先で行う。
  // 従来は追い越し可能な区間でしか反発が働かず、狭い区間や複数台、
  // 後方からの接近に対して無防備だった。
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
      }
      // 横方向にどれだけ離れているかを先に見る。
      // 追い越しは「横に避けて隣を通り抜ける」動作なので、正面から接近して
      // いるように見えても横に十分離れていれば当たらない。
      // ここを見ないと、追い越しのたびに回避層が減速をかけて抜けなくなる
      // (実測: 試行38回で成功0回)。
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
      // --- 逃げる向きは**相手の反対側**(バグ修正 2026-08-29)
      //
      // 【直したバグ】ここは `room_left >= room_right` とコリドアの空きだけで
      // 向きを決めており、**相手がどちら側にいるかを見ていなかった**。
      // すぐ上の通常の push は sep の符号で正しく決めているのに、
      // wedge がそれを上書きしてしまう。
      // 実測(1レース・Crash 7件): **7件中6件で wedge が相手側へ押していた**。
      // しかも wedge は意図的に減速しない(avoid_speed_cap = -1)ので、
      // 相手へ向かって減速せずに寄せることになる。
      //   60.9s 右へ-1.15(空き 左0.27/右3.18) 相手横 -0.56 = 右  -> 相手側
      //   194.0s 右へ-1.15(左0.49/右2.76) 相手横 0.04 自車 1.33 -> 相手側
      // 空きの比較は「相手が真後ろ(|sep|<0.15)でどちらへ逃げても同じ」
      // ときのタイブレークに降格する。
      const double dir = (std::abs(worst_sep) > 0.15)
                           ? ((worst_sep > 0.0) ? 1.0 : -1.0)
                           : ((room_left >= room_right) ? 1.0 : -1.0);
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
      // 【直したバグ 2026-08-30: 8/29 の wedge の向き修正が、ここで打ち消されていた】
      // push は定義上「相手の反対側」を向いているので、下の cand_b は**必ず相手側**。
      // このブロックは「空いているほうへ寄る」だけで相手を見ておらず、
      // すぐ上で直したはずの「コリドアの空きだけで向きを決める」バグが
      // 1ブロック下にそのまま残っていた。
      // 実測(8レース・wedge 発火61件): **42件(69%)がここで相手側へ 1.15m 反転**。
      // Crash に2秒以内で先行した10件は **10件とも反転していた**。
      // 8/29 の検証(42件中38件が正しい向き)は上書き**前**の push を見ており、
      // 実際に指令へ載る値ではなかった。
      //
      // wedge_keep_room: wedge は wedge_room(0.25)まで縁を使う前提で量を決めるのに、
      // ここで corridor_safety(0.65)へ締め直すため両側 0.40m ぶん逃げ量を失い、
      // wedge_room が事実上無効になっていた。
      const double edge = (wedge_active_ && wedge_keep_room_) ? wedge_room_
                                                             : safetyAt(ei);
      const double lo = corridor_.lo[ei] + edge;
      const double hi = corridor_.hi[ei] - edge;
      // 必要なぶん丸ごと入らないときに横移動を諦めると、狭い区間では
      // まったく避けずに減速だけになる(実測: 接触したのに 横=0.00m)。
      // 入るところまで寄せる。半分でも寄せたほうが当たり方は軽くなる。
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
    // 反発は追い越し要求より優先する。接触は crash ペナルティで最も重い。
    //
    // 【2026-08-31 修正】ここは長く can_pass_now_ で門番していたが、これは誤り。
    // can_pass_now_ は「追い越してよいか」であって「並走する幅があるか」ではない。
    // そのため *追い越し試行が失敗した瞬間* に押しのけが止まり、
    // 相手と横に重なったまま寄っていって接触していた
    //   (20260831-012108 d2 Crash@d3 idx223 5周目:
    //    追越試行 失敗 最大実測横間隔=1.92 -> 横間隔=-0.09、直後に Crash)。
    // 幅の判定は走行可能帯(buildBand)で直接できるので、そちらで見る。
    const bool launch_phase =
        launch_since_ >= 0.0 && (now.seconds() - launch_since_) < launch_free_sec_;
    double want = my_lat_for_target_ + c.repulse;
    bool room_ok;
    if (!repulse_need_allow_ && !launch_phase &&
        ei < band_lo_.size() && band_lo_[ei] < band_hi_[ei]) {
      // 帯に収まる範囲まで逃げる。帯の端に張り付いていて動けないなら
      // 横退避は諦める(壁に当たるだけ)。その場合は preventRearEnd が車間で守る。
      // 帯の端に張り付くと壁に当たる(実測: 反発を解放したら壁接触が
      // 0.33 -> 1.67件/レースに増えた)。端に余白を残す。
      // 帯がその余白を取れないほど狭いときは中央に寄せる。
      double lo = band_lo_[ei] + repulse_band_margin_;
      double hi = band_hi_[ei] - repulse_band_margin_;
      if (lo > hi) {
        const double mid = 0.5 * (band_lo_[ei] + band_hi_[ei]);
        lo = hi = mid;
      }
      want = std::clamp(want, lo, hi);
      room_ok = std::abs(want - my_lat_for_target_) > 0.10;
    } else {
      // 帯が無い区間・発進直後は従来どおりの条件を使う
      // (発進は holdGridLane が最終決定するので挙動を変えない)。
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
  // 【実測 20260830-135051】グリッドはフィニッシュラインの約7m手前にあり、
  // 発進して 6.5m 進んだ t=11.5s でラインを越えて lap_ が 0->1 になる。
  // 「1周目だけ」を lap_==0 で書くと、抜き切る前にゲートが落ちる
  // (実際 失効 t=11.5s 弧長差=+5.6m で打ち切られた)。
  // 窓は launch_p1_pass_sec(25秒)で閉じるので、周回は保険として 1 まで許す。
  if (lap_ > 1) { return false; }
  if (launch_since_ < 0.0) { return false; }
  return (this->now().seconds() - launch_since_) < launch_p1_pass_sec_;
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
                f.now.seconds() - launch_since_, d_prog, launch_run_,
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
                f.now.seconds() - launch_since_, d_prog, launch_run_);
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

// 発進フェーズだけ、横目標を「自分のグリッドの横位置」で最終決定する。
//
// 【ユーザー報告: スタートした瞬間に P1・P2 ともにハンドルが左を向く】
// 実測5レースで原因は3つ、いずれも別の層だった(横位置は 正=左 / 負=右):
//   d2(P2, 自車横 -2.48): avoidStoppedCars が停止中の MPC の脇の空き帯の
//       **中央** -1.59 を狙う(3591行)。0.89m 左へ寄れという指令になる。
//   d1(P1, 自車横 +1.29): repulseFromNearCars が P3(横 +0.61)から離れる向き、
//       つまり**左**へ +1.78 を出す(3899行)。can_pass_now_ ガードがあるため
//       合図の瞬間にきっかり跳ぶ。ユーザーの望む右とは真逆。
//   d2 の持続的な左: holdStartLane の 70m 減衰(4069行台)。
//
// 【holdStartLane では直せない理由】
//   (1) start_captured_ は速度 0.3m/s 超で初めて真になる。合図から約5.3秒は
//       全車が動けないので、その窓を丸ごと素通りする。ハンドルが左を向くのは
//       まさにこの窓で、HANDOVER のコリドア基準の丸めとは無関係。
//   (2) holdStartLane は層の5番目で、applyAvoidance / holdAttemptSide に
//       上書きされる。
// そこで holdAttemptSide の後・avoidWall の前、つまり最終決定段に置く。
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
  if (!pass_hold && launch_since_ >= 0.0 &&
      (f.now.seconds() - launch_since_) > launch_hold_sec_) { return; }

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

  // --- 第2段: P1 だけ右へ出して MPC(P3)を即座に抜きにいく(ユーザー要望)
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
    // 抜き切るまで層を降ろさない。
    // 【実測】launch_hold_sec で降りた瞬間、目標が 0 付近へ戻り、
    // preventRearEnd の sep = min(|実横-相手横|, |目標横-相手横|) が
    // 4.11m -> 0.59m に落ちて上限が掛かった
    // (「追突防止 d3 まで 3.7m 横間隔 0.59m 上限 11.6 -> 6.8km/h」)。
    if (pass_hold) { span = 1e9; }
    // 横目標を動かす速さに上限をかける。pure_pursuit は低速で lookahead を
    // 指数的に縮めるので、大きな横ずれを一気に与えると行き過ぎる
    // (目標 -1.70 に対し実測 -3.79 でコリドアを 0.5m 逸脱した)。
    //
    // 【直したバグ(ユーザー報告「P1が無駄に何度も切り返している」)】
    // ここは `clamp(want, my_lat_for_target_ ± step)` = **実横位置基準**
    // だった。これは正のフィードバックになる。実測 20260830-181222:
    //   実横 +1.29 のとき 目標 -0.90 -> +0.09 (逆向きに出る)
    //   実横 -3.33 のとき 目標 -1.70 -> -2.13 (行き過ぎをさらに助長)
    //   実横 -3.44 のとき 目標 -1.70 -> -2.24
    // 行き過ぎるほど目標が行き過ぎ側へ引きずられ、振動する。
    // 基準は**直前に自分が出した指令**でなければならない(素直なレート制限)。
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
  // 【調停 2026-09-02】重み w で「今決まりかけている横目標」と混ぜてから、
  // グリッド保持の意図として出す。優先度は最下位に近いので、
  // 追越・停止車回避・衝突回避が出ていればそちらが勝つ。
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
      // --- 【ユーザー報告 2026-08-29「P2 が左により、P1 とぶつかる」の原因】
      //
      // ここは固定値 start_lat_max(0.9) で横位置を丸めていた。
      // P2 のグリッドは **横 -2.48m(右端)**なので、保持する横位置が
      // -0.90 に丸められ、**発進した瞬間に 1.6m 左へ寄る指令**になる。
      // その先には最前列の P3(横 +0.62)と、右へ切ってくる P1 がいる。
      // 実測(発進計測ログ): P2 の横が -2.48 -> -1.83 -> -1.44 -> -1.33 と
      // 左へ動き、P3 の後ろに入って 2〜4km/h まで落ち、
      // P1 との進行度差が 0.0m(真横)になっていた。
      //
      // 丸める目的は「壁に張り付いたまま走らないこと」なので、
      // **固定値ではなくコリドア(走行可能領域)で丸める**のが正しい。
      // 壁から wall_margin だけ離れていれば、グリッドの横位置は保ってよい。
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
      // --- 3位スタートなら右へ寄って2位に付いていく(ユーザー方針)
      //
      // 本番は **1位が運営コンピュータ(NPC)で確定**しており、しかも
      // 1位は 25km/h のハンデを受けるので遅い。3位から1位のラインに
      // 付いていくと、その遅い車の後ろに並ぶことになって損。
      // 抜くべき相手は2位(プレイヤー)なので、そちらの側へ寄せて出る。
      //
      // 実測で d3=最前列 / d2=中間 / d1=最後尾。3位スタート = d1。
      // グリッドは進行方向に対して左右に振られているので、
      // 「2位に付く」= 2位のグリッド側へ寄せる、と読み替えて実装する。
      // 2位の横位置が取れているならそちらへ、取れていなければ
      // start_p3_lat(既定 +0.6m = 右)へ寄せる。
      // --- スタート時に全車の位置を記録する(ユーザー指示)
      //
      // 「スタートした瞬間の P1,P2 の位置を記録し、その記録をもとに
      //  自分が今 P1 なのか P2 なのかを求め、P1 は P2 のほう(右)へ寄る」。
      //
      // **P1 が最後尾、P3 が最前列(MPC)**。ユーザー確認済み。
      // AWSIM の P 番号はグリッドの後ろから振られている。
      // したがって進行度(prog)の**小さい順**に P1, P2, P3 となる。
      // (最初は逆に並べており、P1 を最前列と誤って扱っていた)
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

        // --- 記録済みのグリッド座標と照合して自分のスロットを決める(ユーザー指示)
        //
        // 「スタートする瞬間の P1,P2,P3 の場所を保存して一旦終了。
        //  記録した場所をもとに、実際のレースのスタート位置を照合して
        //  自分が P1/P2/P3 のどれかを求める」。
        //
        // 他車の進行度から毎回その場で並べる方法は、V2X がまだ届いていない、
        // あるいは prog の初期化が済んでいないと崩れる。
        // グリッドは毎回同じ場所なので、**記録した座標に最も近いスロット**を
        // 自分の位置とみなすのが確実。
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

        // 【削除(ユーザー指示 2026-08-29)】
        // ここには「P1 なら P2 の側(右)へ寄せる」分岐があったが、
        // 実走で機能しなかったため削除した。P1 も自分のグリッドの
        // 横位置をそのまま保つ。
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
        const double w = 1.0 - run / start_merge_dist_;
        // 他車回避の要求が無いときだけレーン保持。回避が必要ならそちらを優先
        // 【調停 2026-09-02】スタートのレーン保持もグリッド保持と同じ層。
        // 回避要求があるときは混ぜるだけにして、優先度で潰されるに任せる。
        const double sl = c.blocker.empty()
                            ? (start_lat_ * w)
                            : (c.latWant() * (1 - w) + start_lat_ * w);
        c.requestLat(sl, PlanCtx::LatPrio::kGridLane, "スタートレーン保持");
      }
    }
  }
}

// 並走中は横オフセットを保持する。
// 横に並ぶと相手が前方帯から外れて検出されなくなり、目標が 0 に戻ってしまうため。
void V2XOvertaker::holdSideBySide(const Frame & f, PlanCtx & c)
{
  const size_t n = f.n;
  const size_t ei = f.ei;

  // --- 並走中は横オフセットを保持する
  // 横に並びかけると、相手は「前方の帯(front_lane_half)」から外れるので
  // 前方車として検出されなくなる。すると target_offset が 0 に戻り、
  // ちょうど並んだ瞬間にラインへ戻ってしまう。
  // 実測: 失敗ログが全て `横目標=0.00 allow=1 幅=4.6〜5.4` で、
  // 判定は通っているのに横へ出る指令だけが消えていた(成功 0 回の直接原因)。
  // 試行中は、抜き切るか打ち切るまで出した側の offset を保持する。
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

// ===================================================================
// 【追加 2026-09-03】追越ファネルを1試行1行で残す(観測専用)。
//
// 機会→試行→横移動完了→並走→先行→維持→復帰 のどこまで進んだか(到達段階)と、
// **最初に進めなくなった理由を一つだけ**書く。試行の出口は
//   (1) 停止車回避による安全中断 (onTimer)
//   (2) 成功 (3) 失敗 (4) 打切 (recordAttempt)
// の4つある。片方に書くと必ず抜けるので、すべてここを通す。
// 制御はこの関数を呼ばないし、この関数は指令を一切書き換えない。
// ===================================================================
void V2XOvertaker::logAttemptFunnel(const Frame & f, const char * result, double elapsed)
{
  static const char * const kStageName[] = {
    "未到達", "試行開始", "横移動完了", "並走", "先行", "維持", "復帰"};
  int st = attempt_stage_;
  if (st < 0) { st = 0; }
  if (st > 6) { st = 6; }
  // 失敗したのに理由が付いていない試行は「その他」で数える(取りこぼさない)。
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
    // 【修正 2026-09-03】横ずれの量だけで試行を数えると、追い越すつもりの
    // ない横ずれまで「追越試行」になる。
    //
    // 【実測(11レース・881試行)】「最初の失敗=その他」が385件(44%)あり、
    // その所要中央値は 2.1秒、69%が3秒未満で終わっていた。最大横間隔の
    // 中央値は 1.53m で、開始しきい値 pass_gap*0.7 = 1.33m のすぐ上。
    // つまりこれらは**コーナーや相手の横移動で横間隔が 1.33m を跨いだだけ**の
    // 事象を試行として数え、0.48m を割った時点で「失敗」にしていた。
    // 分母が水増しされ、ファネルの割合も成功率も信用できない値になっていた。
    //
    // 【対策】横位置の調停で「追越」の意図が実際に採用されている周期でだけ
    // 試行の開始を認める。lat_intent は planOvertake まででほぼ確定しており、
    // この後に走る wallGuard は意図を出さない(制約と上限のみ)ので、
    // ここで読む値は最終値と一致する。
    const bool overtake_intent =
      (c.lat_intent.why != nullptr && std::strcmp(c.lat_intent.why, "追越") == 0);
    const bool start_ok = moving_out && (overtake_intent || !attempt_require_intent_);
    if (!attempt_active_ && start_ok && !c.blocker.empty()) {
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
      // 【追加 2026-09-03 観測のみ】追越ファネルの状態を試行ごとに初期化する。
      attempt_stage_ = 1;                 // 1 = 試行開始
      attempt_fail_first_.clear();
      attempt_runup_used_ = false;
      attempt_rel_v_since_ = -1.0;
      attempt_third_since_ = -1.0;
      // 【追加 2026-09-03】成功/失敗の記録に残す観測値。
      // gap / ei はこの関数のスコープに無い。相手の進行度は findFrontCar が
      // o.prog = my_prog_ + 前方距離 として入れているので、その差が車間になる。
      {
        const auto itb = others_.find(c.blocker);
        attempt_gap0_ = (itb != others_.end() && itb->second.valid)
                          ? (itb->second.prog - my_prog_) : -1.0;
      }
      attempt_idx0_ = f.ei;
      // 【追加 2026-09-03】開始時の順位。成功の記録に P?->P? として残す。
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
      // 対象車が自分より後ろに回ったら成功。
      // 累積進行度の差をそのまま見ると、周回のまたぎで一時的に大きく振れて
      // 開始直後に「成功」と誤判定していた(所要0.0秒の記録が大量に出た)。
      // 差が妥当な範囲(1周未満)にあるときだけ判定する。
      // 【追加 2026-09-03】試行中に一度でもペナルティ中だったら記録に残す。
      // 「止まっている車を抜いた」だけの成功を、実力の証拠と取り違えないため。
      if (isPenalized(attempt_target_)) { attempt_tgt_pen_ = true; }
      if (selfPenalized()) { attempt_self_pen_ = true; }
      bool passed = false;
      bool stalled = false;
      // 【追加 2026-09-03 観測のみ】ファネル判定に使う進行度差。
      // 周回のまたぎで大きく振れるので、妥当な範囲のときだけ有効とする。
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
        // 【修正 2026-09-02】完全に前へ出たと認める距離を pass_done_len_ にした。
        // pass_len_*0.5 は本日 pass_len を 8.0→4.0 にしたため 2.0m まで下がり、
        // 車1台ぶんも離れていない状態を「抜き切った」と扱っていた。
        // 【修正 2026-09-02】旧実装は `diff > 0.0` が3周期(20Hz なので 0.15秒)
        // 続けば成功としていた。並走中は進行度差がゼロ付近で振動するので、
        // 1cm 前に出ただけで成立する。しかも成功判定は attempt_active_ を
        // 落とすため、**横に並んだ瞬間に試行を終えてラインへ戻り、また後ろへ
        // 落ちる**という挙動になっていた(ユーザー報告「抜けそうで抜けない」)。
        // ログ上の「成功」も実体の無いものを数えていた。
        //
        // 完全に前へ出た(pass_done_len_)状態が pass_done_sec_ 続いたときだけ
        // 成功とする。20Hz 前提で周期数へ換算する。
        const int need_cycles =
          std::max(1, static_cast<int>(pass_done_sec_ * 20.0 + 0.5));
        // 【修正 2026-09-03 実測】所要時間の下限を足した。
        // 成功記録 72 件のうち **19% が所要 1 秒未満**だった。追い越しは横へ出て
        // 並走して抜き切る動作なので、1 秒未満で完了することはない。相手が
        // ペナルティで急減速したりコースを外れたりして進行度が不連続に動いた
        // 瞬間を拾った偽陽性である。7% は「自分のほうが遅いのに抜いた」記録で、
        // これも同じ原因。実際に前へ出ていない試行を成功と数えると、改善の
        // 効果測定そのものが狂う。
        const double elapsed_now = now.seconds() - attempt_start_;
        if (diff > pass_done_len_ && diff < total * 0.5 &&
            elapsed_now >= pass_done_min_sec_) {
          if (++attempt_lead_cnt_ >= need_cycles) { passed = true; }
        } else {
          attempt_lead_cnt_ = 0;
        }
      }
      const double elapsed = now.seconds() - attempt_start_;

      // ================= 追越ファネル(観測のみ / 2026-09-03) =================
      // ここは c を読むだけで、指令(requestCap / requestLat / target_offset /
      // charge_now / 状態遷移)には一切触れない。閾値も既存のものしか使わない。
      {
        const double sep_now = std::abs(pass_sep_);
        const double veh_len =
          veh_wheel_base_ + veh_front_overhang_ + veh_rear_overhang_;

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

        // --- 継続的に見る条件の経過時間を更新する(理由の判定より前)。
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

        // --- 最初の失敗理由を一つだけ持つ。一度入ったら上書きしない。
        // 評価の順序がそのまま優先順位。「相手ペナ」を最優先にするのは、
        // 止まっている相手の試行を成功にも失敗にも数えないためである。
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

      // --- 横に出られないまま粘るのをやめる(既定は無効: 0.0)
      //
      // 実測(5レース): 打切 110 件のうち **62 件(56%)は、その試行中に
      // 一度も抜き切りモードの条件(実測の横間隔 >= min_lat_sep)を
      // 満たしていなかった**。つまり16秒かけて一度も横に出られていない。
      // 抜けるための前提が成立していないので、粘っても車間を詰めるだけ損。
      //
      // 【過去に棄却した早期打切との違い】attempt_stall_time は
      // 「進行度差が増えたか」で見ており、4秒/0.5m で降りると
      // 17-27回/レース発動して追越成功が3レースとも0になった(devnote 3節)。
      // こちらが見るのは進展ではなく**横に出られたかどうか**という前提条件。
      // 一度でも横間隔を確保できた試行は対象外にする(attempt_latok_)。
      bool lat_stalled = false;
      if (attempt_latfail_time_ > 0.0) {
        if (attempt_max_sep_ >= commit_sep_) { attempt_latok_ = true; }
        if (!attempt_latok_ && elapsed > attempt_latfail_time_) {
          lat_stalled = true;
        }
      }
      // そのまま進むと自分が壁に当たる状態が続いたら試行を終える。
      // ここで見るのは自分の壁余裕だけで、速度差やゾーンでは降りない
      // (それで降りると抜けなくなる。実測: 成功 0.42 -> 0.00)。0 で無効。
      const bool infeasible_stop =
        attempt_infeasible_time_ > 0.0 && attempt_infeasible_since_ >= 0.0 &&
        (now.seconds() - attempt_infeasible_since_) >= attempt_infeasible_time_;
      // 失敗は「ラインへ戻った」ときだけ。
      // moving_out (pass_gap*0.5) で見ると、コリドアに丸められて
      // わずかに届かないだけで失敗扱いになり、成功が一度も記録されない
      // (実測: 横目標 -0.89 に対ししきい値 0.95 で失敗)。
      // さらに、1周期(0.05s)だけ見て失敗にすると、前方車が1周期見えなかった
      // だけで失敗になる。抜き切った直後がまさにその状態なので、
      // 成功した追い越しが失敗として記録されていた。
      // 0.5 秒連続で戻ったままのときだけ失敗とする。
      if (!passed && std::abs(pass_sep_) < min_pass_sep_ * 0.3) {
        if (attempt_fail_since_ < 0.0) { attempt_fail_since_ = now.seconds(); }
      } else {
        attempt_fail_since_ = -1.0;
      }
      if (passed) {
        attempt_active_ = false;
        attempt_ok_++;
        // 【拡充 2026-09-03】後から「なぜ抜けたか」を再構成できるだけの事実を残す。
        // 相手/自車のペナルティ状態を必ず併記する。止まっている車を抜いた成功を
        // 実力の証拠と取り違えると、改善の方向を誤る。
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
        // 【追加 2026-09-03】接触の局面判定「追越直後」に使う時刻。
        last_pass_ok_t_ = now.seconds();
        logAttemptFunnel(f, "成功", elapsed);
      } else if (attempt_fail_since_ >= 0.0 && elapsed > 1.0 &&
                 (now.seconds() - attempt_fail_since_) >= 0.5) {
        attempt_active_ = false;
        attempt_ng_++;
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
                 infeasible_stop) {
        attempt_active_ = false;
        attempt_ng_++;
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

// 追い越し試行中は「寄ると決めた側」を保持しきる(ユーザー方針)。
// 一度寄る方向を決めたら、抜き切るか失敗が確定するまで戻さない。
void V2XOvertaker::holdAttemptSide(PlanCtx & c)
{
  // --- 試行中は「寄ると決めた側」を保持しきる(ユーザー方針)
  //
  // 「抜こうとするときは左右どちらかに寄っておく。一度寄る方向を決めたら
  //  抜き切るか、失敗が確定するまで寄るのをやめない」。
  //
  // 【なぜ必要か】横目標は複数の層が上書きする(回避 / 近接車の反発 /
  // 壁回避の相手側へ寄る / スタートのレーン保持)。そのたびに横位置が
  // ラインへ戻され、**相手の真後ろに落ちる**。真後ろは
  //   (a) 抜けない (b) 追突して Crash になる
  // という最悪の位置。実戦解析では追従キャップが外れていたのは 27% だけで、
  // 解除の継続は中央値 0.9 秒しかなかった。
  // 実測でも自コード同士は 57回試行して成功0回、しかも横間隔の
  // 91% は車幅以上に達している。**寄れているのに保てていない**。
  //
  // ここでは「試行中は side_sign_ の側へ最低 attempt_hold_sep だけ寄せる」
  // を最後に上書きする。試行が終わる(成功・失敗・打切)まで解かない。
  // 壁は下の帯クランプが必ず効くので、寄せ続けても壁には当たらない。
  // 現在観測から衝突回避が横方向を指定した周期は、その指令を追越保持で
  // 上書きしない。予測の外れを検知した最後の安全層を残す。
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

  // ===================================================================
  // --- 壁を最優先で避ける。避けきれないときだけ相手側へ寄る
  //
  // ユーザー方針:
  //   (1) 単独では絶対に壁に当たらない。
  //   (2) 追い越し中に壁に当たりそうなら、相手との位置関係を見て、
  //       Crash にならないなら相手側へ寄って壁を避ける。
  //   (3) 相手へ寄ると Crash になる位置なら、壁にも相手にも当たらないようにする。
  //
  // 罰則の非対称性(バイナリ実測。開発メモ):
  //   Crash(P1) = 10秒 5km/h 固定、Wall(P2) = 5秒 5km/h 固定。
  //   レイヤーが BumperFront / BumperRear / Vehicle に分かれ
  //   HandleRearEndOverlap があることから、**Crash は自分の前で当てたときに付く**。
  //   横から触れる分には付かない。
  // したがって優先順位は  前から当てる(10秒) > 壁(5秒) > 横で触れる(0秒)。
  // 「壁に当たるくらいなら、横に並んでいる相手へ寄る」が正しい。
  //
  // ここは横目標を決める最後の段。以降はレート制限を掛けて出すだけなので、
  // この判断が最終的な指令になる。
  if (corridor_.lo.size() == n && !wedge_active_) {
    // 帯は「走行ラインを必ず含む」ようにする。
    //
    // 【最初の実装の誤り】corridor_.lo/hi は make_corridor.py の時点で
    // 車体半幅 0.73 + 余裕 0.45 = 1.18m を引いた値になっている。
    // そこへさらに wall_margin(0.65)を引くと帯が過剰に狭くなり、
    // **走行ライン(offset=0)自体が帯の外に出る点が 242 中 57 点(24%)**あった。
    // その結果、他車が居ない単独走行でも「壁へ押されている」と誤判定し、
    // 下の減速が発火して極端に遅くなった(ユーザー報告)。
    // 走行ラインは定義上走れる線なので、帯は必ず 0 を含める。
    // きついコーナーほど壁の余裕を増やす。
    //
    // 実測(3台走行4レース): 残った壁接触は **idx 130-131 に集中**していて、
    // ここは曲率半径 4.7〜5.8m のヘアピン。自車は 横=-0.52〜-0.72 で
    // 停止しており、最寄りの他車は 50m 先。つまり単独で壁に当たっている。
    // コリドアの lo/hi は「車体中心に半幅0.73+余裕0.45」を見込んだ値だが、
    // 半径5mの旋回では車体の四隅が中心より外を通るので、中心が帯の内側でも
    // 角が接触しうる。きつい所だけ余裕を増やす。
    double margin = wall_margin_;
    if (!corridor_.radius.empty() && tight_radius_ > 0.0) {
      const double r = corridor_.radius[ei];
      if (r < tight_radius_) {
        margin += wall_margin_tight_ *
                  std::clamp((tight_radius_ - r) / tight_radius_, 0.0, 1.0);
      }
    }
    // --- 停止車を避けている間だけ壁の余裕を縮める(ユーザー指示)
    //
    // wall_margin(0.65) + wall_margin_tight(0.35) = 1.00m は
    // **単独走行でヘアピンの壁に当たらないため**の値(142 の経緯)。
    // これを停止車の脇を抜ける場面にもそのまま適用すると、余裕が三重に積まれる。
    // 実測(d2 が d1 に詰まった地点):
    //   物理的な壁 -2.85 / csv の lo -1.67(半幅0.73+余裕0.45を控除済み)
    //   / wall_lo -0.67(さらに margin 1.00 を控除)
    // 右側に 2.85m あるのに車体中心が 0.67m しか動けず、
    // d1 を避けるのに必要な -1.20 が「壁に近すぎる」として却下されていた。
    //
    // 罰則で比べれば答えは明らか。Wall は 5秒 5km/h、Crash は 10秒 5km/h。
    // しかも壁は掠める程度なら当たらずに済むが、正面の停止車には確実に当たる。
    // **壁ぎりぎりを通るほうが安い。**
    if (c.stop_avoid_active) {
      margin = std::min(margin, wall_margin_stopped_);
    }
    // --- 直線で追い越しにいく間は壁ぎりぎりまで使う(ユーザー指示 2026-08-29)
    //
    // 「直線で追い越しモードになったらアクセル全開で、空いている側の
    //  壁とカートの間を駆け抜ける」。
    // 直線は速度差が最も付く場所で、並走時間も短い。ここで通しきれないと
    // 1周ぶん失う。Wall(5秒)は Crash(10秒)の半分なので、
    // 壁を掠めてでも相手の正面へ押し戻されないほうが安い。
    if (straight_pass_now_) {
      margin = std::min(margin, straight_pass_wall_);
    }
    // 帯は必ず走行ラインを含める(142 の失敗を繰り返さない)
    double wall_lo = std::min(corridor_.lo[ei] + margin, 0.0);
    double wall_hi = std::max(corridor_.hi[ei] - margin, 0.0);
    if (wall_hi > wall_lo) {
      const double want = c.latWant();
      const double safe = std::clamp(want, wall_lo, wall_hi);
      // まず横目標は必ず帯の中に収める。これだけで「壁へ寄せる指令」は出なくなる。
      // 単独走行(他車なし)ではここで終わり。減速は一切しない。
      // 【調停 2026-09-02】ここは「範囲の制限」なので制約として積む。
      // 意図を潰さずに交差させるので、追越の横目標は壁帯の中で最大限残る。
      c.boundLat(wall_lo, wall_hi, "壁回避");
      const bool pushed_to_wall = std::abs(want - safe) > 1e-3;
      // 【追加 2026-09-02】判定はそのまま。状態機械が「コース外予測」の
      // 安全条件として読むために控えるだけ(avoidWall のロジックは不変)。
      wall_push_now_ = pushed_to_wall;

      // --- 停止車を避けきれないなら減速する(ユーザー指示)
      //
      // 【直したバグ(ユーザー報告: 回避しないまま突っ込む)】
      // 停止車回避が出した「ここへ寄れば通れる」という横目標を、この壁帯が
      // 潰していた。にもかかわらず停止車回避は「通過できる」前提のまま
      // 速度上限を stopped_thread_speed(10.8km/h)へ引き上げており、
      // **避けられない位置で全開前進**していた。
      // 実測: 横目標 -1.28 が wall_lo -0.67 に潰され、上限 10.8km/h のまま
      // 前方 1.5m の d1 へ押し付け続けた。
      //
      // 潰されたということは、その停止車の脇は通れないということ。
      // 「通れない -> 手前で止まる」の経路(act="停止")と同じ扱いにする。
      // 【2026-08-31 修正】判定が「横目標がどれだけ動かされたか」だったため、
      // **動かされた後でも通れるか**を見ていなかった。
      // 実測 (20260831-073512 d2, NPC_SIM で大会NPC相当の停止を再現):
      //   停止車両 1台 先頭 d3 まで 3.1m 空き幅 1.83m -> 通過 (上限 10.8km/h)
      //   停止車を避けきれない 横目標 -2.16 が壁帯[-2.00,2.50]で -2.00 に潰された
      //     -> 上限 2.2km/h へ減速
      //   膠着 停止車両 d3 まで 3.1m 自車 -0.00m/s が 3.0秒 空き幅 1.83m
      // 1.83m の空きがあり、削られたのは 0.16m だけなのに完全停止していた。
      // 空き帯 [stop_avoid_lo, stop_avoid_hi] は既に相手の中心から
      // ±kCarWidth を除いてあるので、**潰された後の位置がその中に入っていれば
      // 車体は通る**。入っていないときだけ「通れない」とする。
      bool cannot_pass = std::abs(want - safe) > stop_avoid_crush_;
      if (cannot_pass && stop_avoid_fit_ && c.stop_avoid_have_gap &&
          safe >= c.stop_avoid_lo && safe <= c.stop_avoid_hi) {
        cannot_pass = false;
      }
      // --- 遠方での「潰された」を減速の理由にしない(2026-08-31)
      //
      // 上の判定は **自車の現在位置 ei** の壁帯で横目標を切っている。だが
      // その横目標は **停止車の位置 bi** で満たしていればよい値であり、
      // 今いる場所で満たす必要はない。コリドアの幅は地点ごとに違うので、
      // 手前が狭いだけで「通れない」と誤判定する。
      //
      // 実測(大会NPC相当 8レース、16標本): この減速は 290回/16台 発生し、
      // **停止車までの距離の中央値は 26.5m**(20m以遠が 66%)。
      // 20km/h なら 26m は 4.6秒先で、横に 1.6m 寄るには十分すぎる。
      // 実際ログでは壁帯が [-0.55,1.75] -> [-0.45,3.30] -> [-0.35,3.80] と
      // 進むほど広がっており、手前の狭さは通過可否と無関係だった。
      // その間ずっと 26.5 -> 23.7 -> 20.8km/h と落とし続けた結果、
      // 停止車の手前 4m で 4.4km/h まで落ちて詰まり、復帰動作(平均11秒)へ入る。
      //
      // 元の不具合(横目標が潰されたまま前方 1.5m の停止車へ全開で押し付ける)は
      // **近距離**で起きたものなので、近距離では従来どおり減速する。
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
          // 横に並んでいる相手なので寄っても Crash にならない。
          //
          // 【直したバグ(ユーザー報告: 壁に余裕があるのに相手へ寄っていく)】
          // 旧実装は「相手から crash_safe_sep(1.45m)の位置」を目標にしていた。
          // 発火条件 pushed_to_wall は「横目標が帯でクリップされたか」だけで、
          // 壁までの実距離を一切見ていない。corridor_.lo/hi は make_corridor.py
          // の時点で 半幅0.73 + corridor_extra(0.45) を引いた値なので、
          // 帯端に居る時点で車体端-物理壁は必ず 0.70m 以上空いている。
          // 実測5レース168件では余裕 最小0.70 / 中央値1.10m、
          // 「壁が本当に近かった件」は 0 件。それでも相手が 3.7m 離れていても
          // 1.45m まで詰めており、最大 2.55m を相手側へ振っていた。
          // その結果、横間隔 3.9m あった並走が潰れて追越試行が失敗している。
          // さらに preventRearEnd はこの横目標を「これから通る位置」として
          // 読む(sep_th = rear_end_sep = 1.45)ので、1.45m に置いた瞬間
          // 前方車扱いの減速が掛かる。自他同一コードなので相互減速になる。
          //
          // 直し方: 寄るのは「壁の余裕が実際に足りない分だけ」。
          // crash_safe_sep_ は目標ではなく「これ以上は近づかない」上限に降格。
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
          // 相手へ寄ると前から当てる。壁にも相手にも当たらないよう減速する。
          //
          // 【最初の実装の誤り】上限を「現在速度 x 0.6」にしていた。
          // 毎周期(20Hz)現在速度に掛かるので幾何級数的に落ち、
          // min_follow_speed(7.9km/h)まで落ち切ってしまう。
          // 速度プロファイルの値を基準にして、掛け算が累積しないようにする。
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

// ===================================================================
// 壁衝突の予測監視 (2026-09-03 ユーザー指示)
// ===================================================================
// 「今の速度・今の横目標のまま走ったら、この先で車体が走行可能領域から
// はみ出す」ことを先に見つける層。
//
// 【なぜ要るか】既存の avoidWall は **今いる点** のコリドアへ横目標を
// 収めるだけで、先の点は見ていない。そのため
//   (1) pure_pursuit の目標点がコーナーの壁の向こう側に来る
//   (2) 他車に当てられた / 追い越そうとして寄せすぎた
// のどちらでも、「今は帯の中だが 1 秒後にははみ出す」状態を止められない。
//
// 【他層と喧嘩しないための約束】
//   - 出力は c.boundLat() と c.requestCap() の **2 つだけ**。
//   - 意図(requestLat)は出さない。範囲を狭める / 上限を下げる方向にしか
//     働かないので、後勝ちの上書きも発振も起きない。
//   - c.target_offset / c.speed_cap へ直接代入しない。
//   - wall_guard_enable=false なら関数の頭で return するので、挙動は完全に
//     元へ戻る(この関数は他に一切の副作用を持たない。書き換えるのは
//     観測用の wall_guard_min_room_ と、ログの時刻だけ)。
//
// 【車体の張り出し】
//   - 内輪差(off-tracking): 半径 R の旋回では後軸が前軸より内側を通る。
//     近似 wheel_base^2 / (2R) を **内側** へ加算する。
//   - 前オーバーハングの振り出し: 旋回で前端外側が外へ膨らむ。
//     厳密には(後軸基準で)おおよそ (wheel_base + front_overhang)^2 / (2R)
//     だが、基準点が後軸か車体中心かの確証が無いので、ユーザー指示どおり
//     front_overhang * (wheel_base / R) の **近似** を使う。
//     後端外側の角の膨らみ rear_overhang^2 / (2R) も外側へ足す。
//     いずれも近似であり、実測で足りなければパラメータで振ること。
//   - 内 / 外の向きは corridor_.radius に符号が無い(常に正、既定 1e9)ので
//     経路の 3 点から外積で求める。外積がほぼ 0(直線)で向きが取れない
//     ときは **両側に大きいほうを加算** して保守側へ倒す。
//
// 【半幅の二重計上を避ける】corridor_ten.csv の lo/hi は make_corridor.py が
// 「境界 - (半幅 + 余裕)」で作っており、すでに半幅が控除済み
// (実測メモ: 物理的な壁 -2.85 / csv の lo -1.67 = 差 1.18 = 0.73 + 0.45)。
// ここで veh_half_width_ を丸ごと引くと 0.73m ぶん二重に効き、コーナーで
// 常時作動して極端に遅くなる。控除済みぶん wall_guard_corridor_half_ を
// 差し引いた **超過ぶんだけ** を足す(既定では 0 になる)。
// 舵角の許容範囲を publish する。lo が右いっぱい側、hi が左いっぱい側[rad]。
// viol=true のとき「壁予測が違反を検知している」。wall_guard_enable_=false の
// ときは呼ばれない(購読側は「メッセージが来ない = 制限なし」で動く)。
// テキストログと診断トピックの両方へ同じ内容を出す。
// 片方だけに記録が増えることを防ぐため、記録の追加は必ずこの関数を通す。
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

// ===================================================================
// 占有格子による舵角ガード (2026-09-03 ユーザー指示)
// ===================================================================
// 上の CSV(corridor_ten.csv)ベースの判定は、内輪差や前端の振り出しを
// 近似式で **横位置** に足す間接的なものだった。占有格子を直接引けば
// 車体の四隅の座標をそのまま地図に当てられるので、近似が要らない。
//
// 【この層が出すもの】/control/wall_guard/steer_limit の舵角範囲 **だけ**。
// requestCap / boundLat は呼ばない(2026-09-03 ユーザー訂正)。
// 縦方向は preventRearEnd と TTC の担当。追突の正しい対処は減速であって
// 転舵ではないので、他車を舵角の除外条件に入れてはいけない。
//
// 【車体寸法】ユーザー指示により **実測ぴったり(余裕ゼロ)** で判定する。
// 後軸中心を基準に、前端 = veh_wheel_base_ + veh_front_overhang_ (2.14+0.47)、
// 後端 = veh_rear_overhang_ (0.51)、半幅 = veh_half_width_ (0.73)。
// マージンのパラメータは作らない。
// なお公式 vehicle_info.param.yaml の wheel_base 1.087 と食い違うが、
// この寸法(前端 2.61m)は実測より **長め** = 保守側で、オフライン検証では
// traj_mincurv.csv の 350 点すべてで車体が占有セルに刺さらないことを確認済み。

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
  const double hw = veh_half_width_;
  const double fr = veh_wheel_base_ + veh_front_overhang_;   // 後軸 -> 前端
  const double re = veh_rear_overhang_;                     // 後軸 -> 後端
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

// t_ahead 秒後の他車(等速直線予測)と車体矩形が重なるか。
// 【判定方法】2 つの有向境界矩形(OBB)の分離軸判定。軸は両車の前後軸・左右軸の
// 計 4 本。1 本でも分離できれば重なっていない。相手も自車と同じ寸法とみなす。
// 【重要】他車は舵角の除外条件に **しない**。ここは「前方が塞がっているか」を
// 記録するためだけに使う。
bool V2XOvertaker::otherHits(double cx, double cy, double yaw, double t_ahead) const
{
  const double hw = veh_half_width_;
  const double fr = veh_wheel_base_ + veh_front_overhang_;
  const double re = veh_rear_overhang_;
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
    // 【修正 2026-09-03 A/B実測】生存した舵角の範囲へ常時クランプするのをやめた。
    //
    // 3構成 × 3レース × 4台の比較:
    //   導入前(全部OFF)   走行中相手を抜いた 9 / ペナルティ125
    //   全部ON            走行中相手 2 / ペナルティ199  ← 大幅悪化
    //   舵角クランプのみOFF 走行中相手 6 / ペナルティ133  ← ほぼ回復
    // 舵角上書きは 3レース×4台で 1023 回発動し、生存本数は 41 本中 26〜33 本、
    // つまり常時 2〜4 割の舵角を禁止していた。「最終手段」の頻度ではない。
    // 接触の最多局面が「追越中」・種別の最多が「壁」で、舵角を制限された状態で
    // 追い越そうとして壁へ寄せられていた。
    //
    // ユーザー指示は「本当にぶつかりそうになったら、ぶつからないギリギリで
    // 壁に沿う舵角にする」。逃げ場がある間は何も制限しないのが正しい。
    // 通常時は制限なしを出し、生存 0 のときだけ強制する(下の else)。
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

void V2XOvertaker::wallGuard(const Frame & f, PlanCtx & c)
{
  // 作動しなかったときも「予測した最小余裕」は毎周期残す(観測用)。
  wall_guard_min_room_ = 1e9;

  // 【修正 2026-09-03】占有格子の評価を wall_guard_enable から独立させた。
  // 以前は CSV 版を切ると占有格子の評価とログも止まり、「制御に介入せず記録だけ
  // 取る」構成が作れなかった(実測しようとして生存サンプルが 0 件になった)。
  // 占有格子は occ_enable だけで制御し、舵角範囲の publish もここで行う。
  // 実際に舵角へ効くかどうかは制御側の wall_guard_clamp_enable が決めるので、
  // ここを動かしても clamp が false なら挙動は変わらない。
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

  // --- 占有格子による舵角ガード(2026-09-03 ユーザー指示)
  // CSV ベースの判定は下でそのまま走らせる。占有格子が使えるときは
  // publish する舵角範囲だけを格子版へ差し替える。requestCap / boundLat は
  // 一切呼ばない(この層は舵角のみ)。
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

    // --- 曲率半径。点ごとの値は単発の外れ値が出る(ヘアピンの途中で
    // 257m と出た実測がある)ので、前後の最小をとった radius_min_ と
    // 小さいほうを採る。保守側。
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

  // --- 作動しない周期の観測ログ。
  // 「ぎりぎりが適切か、余裕を増やすべきか」を実測で判断するために、
  // 平常時にどれだけ余裕が残っているかを残す。走りには影響しない。
  // 1 点だけの瞬間的な逸脱では作動しない。
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

  // --- 出力 1: 横位置の制約。
  // 予測に使った lo_need / hi_need の最も厳しい値をそのまま現在地点の
  // 制約にする(ユーザー指示の簡略化)。
  // ただし **今いる点のコリドアより外へ出る要求は作らない**。
  // そこは既存の avoidWall の担当で、先の点の都合で今この瞬間に壁側へ
  // 押し出したら本末転倒になる。
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

  // --- 出力 3(追加): 壁に当たらない舵角の範囲。
  // 【近似】予測時間 t のあいだ曲率 κ を一定で走ると、横方向の変位は
  //   Δy ~= 0.5 * a_lat * t^2 = 0.5 * (v^2 * κ) * t^2
  // (自車座標での二次近似。ヨー角の変化と経路の曲率は無視している)。
  // これが左へ寄れる余裕 (hi_ok - offset_) を超えない最大の κ が左の限界。
  //   κ_max = 2 * room / (v^2 * t^2),  steer = atan(wheel_base * κ)
  // 余裕は 0 で下限を切る。負(すでにはみ出している)のときにそのまま使うと
  // 上限が負になり「反対へ切れ」という **能動的な操舵指示** になってしまう。
  // ここは範囲を狭める安全網であって舵を作る層ではないので、常に
  // lo <= 0 <= hi(直進はいつでも許す)を保つ。
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
  // --- オフセットをレート制限つきで目標へ動かす
  const double dt = 0.05;
  // スタートの合流が終わるまではレートを上げる。
  // 【なぜ(実測 2026-08-29)】P1 はグリッド(横 +1.29)から右(-1.7)へ
  // 2.9m 動く必要がある。通常のレート 1.2m/s では 2.4 秒かかり、その間ずっと
  // 前の P3(横 +0.62)と横に重なっているので追従キャップ(7.9km/h)が外れず、
  // 5.0km/h のまま 4 秒を失っていた。
  // 開始直後は全車が低速なので、横へ速く動いても危険は小さい。
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
    // --- 壁の余裕を「場面」と「曲率」で変える(2026-08-31・ユーザー指示)
    //
    // 【なぜ要るか(実測)】
    // corridor_ten.csv の lo/hi は **既に車体半幅 0.73 + 余裕 0.45 = 1.18m** を
    // 壁から控除してある(車体中心を置いてよい範囲)。そこから更に
    // corridor_safety(0.65)を引くと、片側 1.83m・両側 3.66m を捨てることになり、
    // **車体半幅 0.73m を二重に数えている**。
    //   物理的な壁の間隔  平均 6.76m / 最小 4.96m -> 全地点で2台並べる
    //   実際に使う帯      平均 3.10m / 最小 1.30m -> 36% の地点で「並べない」判定
    // 実測ログ: 相手が右 0.94m にいる直線で 余地=[-2.00,+0.45] となり、
    // 右へ抜くのに要る -2.24m が入らず、**ぎりぎりの左(余裕0.09m)が選ばれていた**。
    // 右の壁までは実際には 3.2〜4.35m ある。
    //
    // 【方針(ユーザー指示)】
    //  - 通常走行はこれまでどおり(安全側)。
    //  - **追い越し・回避をしようとしている間だけ**ぎりぎりを攻める。
    //  - 直線ほど攻め、カーブでは従来の余裕を残す。
    //    低速・大半径の直線は追従誤差が小さく、壁を掠めても Wall(5秒)で済む。
    //    一方コーナーは横加速度が高く、外へ膨らむと復帰できない。
    // 曲率による調整は safetyAt() へ移し、全域(通常走行を含む)へ適用した。
    // ここでは追い越し中にさらに詰める分だけを残す。
    const bool pass_mode = attempt_active_ || c_stop_avoid_active_ || straight_pass_now_;
    for (size_t i = 0; i < n; ++i) {
      double safety = safetyAt(i);
      if (pass_mode) { safety = std::min(safety, corridor_safety_pass_); }
      lo[i] = corridor_.lo[i] + safety;
      hi[i] = corridor_.hi[i] - safety;
      if (lo[i] > hi[i]) { const double m = 0.5 * (lo[i] + hi[i]); lo[i] = hi[i] = m; }
    }
  }

  // 【修正K 2026-09-02】追い越し対象だけを除いた band。コリドア+安全余裕・
  // 壁クランプまではここまでの lo/hi と完全に同じ処理を経ているので、
  // ここで複製すれば以降の「相手を削る」処理だけを分離できる。
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

    // 【修正L 2026-09-02】除外する相手は「いま抜いている相手」。試行中に計画が
    // 破棄されると spot_target_ が変わり/空になり、その瞬間に対象が障害物として
    // 復活して band が潰れ、`相手予測経路が計画ラインへ侵入` で中断していた(実測
    // 20260902-145021: 追越試行2回のうち1回が開始0.4秒で中断)。
    const std::string & band_pass_target =
      (attempt_active_ && !attempt_target_.empty()) ? attempt_target_ : spot_target_;

    for (const auto & kv : others_) {
      const OtherState & o = kv.second;
      if (!o.valid) { continue; }
      // 停止車回避がこの対象の通過帯を既に選んでいる周期は、通常の予測
      // バンドを重ねない。公式とローカルの停止車通過では、回避目標とは別に
      // band clamp が経路を2〜4m動かし、通過ではなくOver/膠着を起こした。
      // 壁クランプ・近接回避・追突防止は後段に残る。
      if (c_stop_avoid_active_ && kv.first == c_blocker_) { continue; }
      const size_t oi = nearest(f.in, o.x, o.y);
      double nx = 0.0, ny = 0.0;
      normalAt(f.in, oi, nx, ny);
      const auto & lp = f.in.points[oi].pose.position;
      const double olat = (o.x - lp.x) * nx + (o.y - lp.y) * ny;
      const double ov = std::hypot(o.vx, o.vy);
      // --- 相手ごとに「どんな MPC で走っているか」を仮定する(2026-08-31)
      //
      // 【ユーザー指示】P1/P2 は速い MPC、P3(運営NPC) は遅い NPC 用 MPC の設定。
      // さらに **相手の順位によって最高速度が変わる**(実測: 1位は 25km/h、
      // 2位以下は 36km/h で駆動が頭打ち)ので、それも上限に入れる。
      //
      // ただし固定値で上書きし続けるのは危険。相手チームのコードが想定より
      // 速い/遅い場合や、スロット判定が外れた場合に、全レースにわたって
      // 間違った予測を使い続けることになる。
      // そこで **走り出しは仮定値、実測が溜まったら実測値** に移す。
      const bool op_is_npc = (o.slot == npc_slot_);
      const double slot_cap_kmh =
          op_is_npc ? predict_speed_slow_ : predict_speed_fast_;
      // 順位のハンデ。先頭は 25km/h で頭打ちになる。
      const double rank_cap_kmh =
          (!cur_leader_.empty() && kv.first == cur_leader_)
            ? leader_speed_cap_ : rank2_speed_cap_;
      const double cap_op = std::min(slot_cap_kmh, rank_cap_kmh) / 3.6;
      // 実測が足りないうちは仮定値を相手の速度とみなして比を作る。
      const bool have_meas = (o.speed_cnt >= predict_prior_samples_);
      const double k_op =
          have_meas ? ratio(oi, std::max(ov, 0.2)) : ratio(oi, cap_op);
      // --- 止まっている相手は「止まったまま」と予測する(2026-08-31)
      //
      // 【ユーザー報告】「止まっている相手に対して、pure_pursuit の経路は
      //   直前まで完全にぶつかっていく経路で、数メートル手前でやっと避ける」。
      //
      // 【原因】上の `ratio` は下限が **0.2** で固定されている。
      // 速度 0 の相手でも k_op = 0.2 になり、さらに下の `vop` は
      // `max(k_op * v_ref, 0.3)` なので、参照ラインが 8m/s の区間なら
      // **止まっている車が 1.6m/s で走り続ける**と予測される。
      // バンドが塞ぐのは予測位置の前後 band_long_ の範囲だけなので、
      // 予測上の相手が前へ滑っていき、**実際に止まっている場所は塞がれない**。
      // 自車は 5.6m/s で近づくので、塞ぎ位置は逃げ続け、
      // 経路は相手へ真っ直ぐ向かったまま接近する。
      // 数メートル手前で avoidStoppedCars や正面衝突回避がやっと反応するが、
      // そのときには寄せ切る時間が無く、詰まって復帰動作に入る。
      //
      // 【対策】速度が band_stop_speed 未満なら相手を進めない。
      // 横位置の減衰(band_lat_tau)も run_o が 0 のままなので効かず、
      // 「今いる場所に居座る」という正しい予測になる。
      const bool op_stopped = (ov < band_stop_speed_);

      const double a = olat - band_car_w_;
      const double b = olat + band_car_w_;
      // 通す側は相手ごとに覚えておき、**明確に差がついたときだけ**入れ替える。
      // 毎周期で広いほうを選び直すと、相手が少し動くだけで左右が入れ替わり、
      // バンドが 2.6m 跳ねて経路がガタつく(ユーザー報告)。
      int & side = band_side_[kv.first];
      const bool is_side_target =
          !side_target_.empty() && kv.first == side_target_;
      // 【棄却した案(実測 20260830-232823)】ここを「相手の予測経路に沿って
      // band_side_look(40m)先まで見て、通しで空いている最小幅」で決める
      // ようにしたところ、発進区間で**左**が選ばれた。あの区間のコリドアは
      // 先へ行くほど左が +3.25〜4.00m に開き、右は -0.75m まで閉じるため。
      // その結果 band_clamp が経路を左へ縛り、右へ出て MPC を抜く発進追抜が
      // 潰れた(4/4 成功 -> `発進追抜 失効 弧長差=+5.9m`)。
      // 相手の今いる 1 点で決めるほうが実測で強い。**再提案しないこと。**
      const double room_l = hi[oi] - b;
      const double room_r = a - lo[oi];
      // --- 抜く側を「相手の予測経路に沿って」決める(2026-08-31・既定オフ)
      //
      // 【ユーザー指摘】「先を読めば絶対に右から抜くべき直線で左を選ぶ」。
      // そのとおりで、ここは **相手の今の1点 oi** の左右の余地しか見ていない。
      // 予測経路(band_pred_)は「どのセルを塞ぐか」にしか使っていなかった。
      //
      // 【前回の棄却について】2026-08-30 に同じ趣旨の実装(band_side_lead)を試し、
      // 発進区間で左が選ばれて発進追抜が潰れたため棄却されている。
      // ただしその判定は **帯の二重計上・停止車の予測の誤り・順位別の速度上限なし**
      // という前提で行われたもので、いずれも本日修正した。前提が変わったので
      // 再評価の価値がある。ただし既定はオフにして、有効化は計測してから。
      //
      // 見方: 相手が「自車がそこへ着く時刻」に居る位置を予測経路上でたどり、
      // その各点で左右に残る幅の **最小値** を取る。通しで通れる側を選ぶ。
      // 発進区間だけは従来どおり「相手の今の位置」で決める。
      //
      // 【ユーザー指摘 2026-08-31】前回の棄却理由(発進区間で左が選ばれ、
      // 右へ出て抜く発進追抜が潰れた)は **スタート時限定の話** であり、
      // そこだけ例外にすれば残りの区間では使える。そのとおりなので、
      // 合流が終わるまで、かつ合図から launch_p1_pass_sec 秒の間は無効にする。
      // あの区間のコリドアは先へ行くほど左が開き右が閉じるので、
      // 先読みで最小幅を取ると必ず左が勝ってしまう。
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
      // 対象車について、ここで再び「瞬間的に広い側」を選ぶと、直後の
      // band_side_follow が追越側へ戻すまで最大 band_side_hold 秒だけ反対側を
      // 開ける。この再選択は試行前にも毎周期起き、公式submit_11では往復が
      // 1戦800〜1300周期、最初の局所修正版でも2周目までに400周期に達した。
      // 対象車の側はヒステリシスを持つ chooseSide だけに決めさせる。
      // バンドはその決定に常時従い、他車だけ従来の余地判定を使う。
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

      // --- 追い越しの側とバンドの側を一本化する(2026-08-31)
      //
      // 【ユーザー報告】「後ろでどっちから抜こうか迷って左右に振られる」。
      //
      // 【原因】同じ「どちら側から抜くか」を **2箇所が別々に決めていた**。
      //  - chooseSide の `side_sign_`  … 追い越しの側。独自のヒステリシス
      //    (side_flip_max=2 / side_flip_hold=0.6s)を持つ。
      //  - ここの `band_side_[相手]`   … バンドをどちら側に開けるか。
      //    独自のヒステリシス(band_side_hyst=0.60)を持ち、
      //    **side_sign_ を一切参照していなかった**(参照は 5146行の1箇所のみ)。
      //
      // 食い違うと、追い越し側が左へ寄せる目標を出す一方でバンドが左を塞ぐため、
      // publishTrajectory の clamp が目標を右へ引き戻す。次の周期も追い越し側は
      // 左を出す。これが左右への振れの正体。
      // 実測(1レース): 追越試行 107回中 成功3回。失敗の多くは
      // `最大実測横間隔=1.5〜2.6m` まで出られているのに feasible=0 だった。
      //
      // 【対策】いま追い越そうとしている相手についてはバンドも同じ側を開ける。
      // それ以外の相手は従来どおり空きの広い側を選ぶ。
      // 窓は **前の周期に** この相手について求めたものを使う。
      // (窓の計算は下の予測ループの中で行うため、同じ周期では間に合わない)
      const auto wit = win_map_.find(kv.first);
      win_found_ = (wit != win_map_.end()) && wit->second.found;
      if (win_found_) {
        win_side_ = wit->second.side; win_t_ = wit->second.t;
        win_d_ = wit->second.d; win_gap_ = wit->second.gap; win_dur_ = wit->second.dur;
      }
      // 窓が見つかったなら、側はその窓の側にする。
      // 「幅が広いほう」よりも「実際に抜き切れる区間があるほう」が強い根拠。
      if (win_found_) {
        // 追越対象の側は chooseSide が、現在の余地・学習した相手ライン・
        // 抜きどころをまとめて決めている。予測窓は最初に見つかった1.2秒の
        // 窓なので、相手が左右へ動くと周期ごとに左/右が反転する。
        // submit_11 ではメイン直線で録画判断が右を選んだ直後にも、この窓が
        // side_sign_ を左へ上書きしていた。対象車については選択済みの側の
        // バンドを補強するだけにし、非対象車は従来どおり窓の側を使う。
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
        // --- 一本化は双方向にする(2026-08-31)
        //
        // 【実測で見つけた欠陥】追い越し側が「左」を選んでいるのに
        //   `空き 左-1.80 右2.60` という場面が出た。左は**入る余地が無い**。
        //   にもかかわらずバンドを左に合わせると、通れない側を塞ぎ続ける。
        //   `追越 側を変更(2/2)` が頻出しており、側の変更枠(side_flip_max=2)を
        //   使い切ると余地の無い側に張り付いたままになる。
        //
        // 【対策】選んでいる側に余地が無く、反対側には車体が通る幅があるなら、
        // **追い越し側のほうを直す**。バンドは幅を直接測っているので、
        // 「入るかどうか」についてはバンド側の判断のほうが確か。
        // 変更枠は使わない(枠は「迷い」を止めるためのもので、
        // ここは迷いではなく明白な誤り)。
        const double room_want = (want > 0) ? room_l : room_r;
        const double room_other = (want > 0) ? room_r : room_l;
        // --- 修正にも保持時間と再反転の待ちを入れる(2026-08-31)
        //
        // 【ユーザー報告】「追い越すのかと思ったら急に経路が切り替わって抜けない」。
        // 診断ログにも `経路の跳び 最大0.70m/周期` が出ており、横オフセットが
        // -0.78 -> 0.00 -> -1.15 -> -0.50 -> 0.00 と往復していた。
        //
        // 【原因】この「余地の無い側を選んでいたら直す」処理に保持時間が無かった。
        // 相手が動くと `空き` の符号は簡単に入れ替わるので、
        // 「側の不一致を解消」と交互に働いて**毎周期でも反転しうる**。
        // バンド側には band_side_hold(0.4s)を入れたのに、こちらは
        // 「明白な誤りだから即座に直す」として付けなかったのが誤りだった。
        //
        // 【対策】(1) 条件が band_side_hold 秒続いてから直す
        //         (2) 一度直したら side_fix_cool 秒は再反転しない
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
        if (!attempt_active_ && room_want < 0.0 &&
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
        // --- 側の切替に保持時間を入れる(2026-08-31)
        //
        // 元々このバンドには band_side_hyst(0.60m)のヒステリシスが入っていた。
        // 理由もコメントに残っている:「毎周期で広いほうを選び直すと、相手が
        // 少し動くだけで左右が入れ替わり、バンドが 2.6m 跳ねて経路がガタつく」。
        // ところが上の一本化で **追い越し側へ強制的に合わせた**ため、
        // そのヒステリシスが効かなくなった。
        // ユーザー報告「追い越し直前で経路がガタガタする」はこれが原因の疑いが濃い。
        // 側を変えるときは、同じ側が band_side_hold 秒続いてからにする。
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
        // 1周分の実測がある地点は相手自身のラインを使う。旧実装は相手が
        // 自車の基準線へ指数的に戻ると仮定していたため、実際には右を保つ相手を
        // 中央へ戻ると予測し、左右の空きと衝突帯を取り違えていた。
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
        // --- 塞ぐ範囲は「予測経路に沿った掃引領域」にする(2026-08-31)
        //
        // 【ユーザー指示】縦方向は予測経路を元に曲線で設定する。
        //
        // 位置は既に予測経路上の点 oc なので曲線に沿っている。長さのほうが
        // 固定 4.0m の一定値で、幾何とも予測の確からしさとも無関係だった。
        // 正しくは
        //   (a) 幾何としての最小値 = 相手の半長 1.3m + 自車の半長 1.3m = 2.6m
        //   (b) 予測誤差ぶんの余裕 = 先を見るほど大きい
        // の和になる。t_me は「自車がこの点へ着くまでの時間」なので、
        // それに比例させれば **近くは細く、遠くは太い** 正しい形になる。
        // 横も同じ理由でわずかに広げる。
        const double lat_half = band_car_w_ + band_lat_grow_ * t_me;
        const double long_half = band_long_ + band_long_grow_ * t_me;
        // ================= 追い越し窓の探索 (2026-08-31) =================
        //
        // 【ユーザー指示】「予測経路において、壁と相手の車の距離がいい感じの時間
        //  続く区間を見つけ、そのタイミングで抜けるようにし、間隔が開く場所で仕掛ける」。
        //
        // 【これまで何をしていなかったか】追い越し可能ゾーン(pass_ok)は
        // make_corridor.py が **壁の幅と曲率だけ** から事前計算した静的な列で、
        // **相手がどこにいるかを一切見ていない**。つまり評価していたのは
        // 「壁と壁の距離」であって「壁と相手の距離」ではなかった。
        //
        // 【ここで使うもの】この時点で揃っている量:
        //   t_me   自車がこの地点へ着くまでの時間[s](自分の到達可能速度で積分)
        //   lat_p  そのとき相手がいる横位置(相手の予測速度・最高速度・順位ハンデ込み)
        //   lo[i]/hi[i] そのときの壁(と、既に処理した他車)の内側の限界
        // よって「自車が着いた瞬間に、壁と相手の間にどれだけ空くか」が直接出る。
        //
        // 【窓の条件】
        //  (1) 幅: 左右いずれかの隙間が pass_window_gap 以上
        //  (2) 継続: その状態が pass_window_sec 以上つづく(一瞬すれ違うだけでは抜けない)
        //  (3) 到達可能性: 自車が相手に追いつく地点より先であること。
        //      追いついていない場所の窓は意味がない。
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
        // 【修正K】除外 band は追い越し対象(band_pass_target)以外の相手だけを削る。
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

  // --- 予測が当たっているかを実測する(診断・2026-08-31)
  //
  // 「相手をMPCと仮定して予測経路を出す」仕組みは入っているが、
  // **当たっているかを一度も測っていなかった**。推測で良し悪しを語らないために、
  // predict_check_sec 秒後の予測位置を覚えておき、その時刻に実測位置と比べる。
  if (predict_check_sec_ > 0.0) {
    const double tnow = f.now.seconds();
    // 期限の来たものから順に、実測と突き合わせて誤差を出す
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
  // 【修正K】除外 band にも同じ時間平滑化を適用する。
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
  // 表示の間引き。0.2s(5Hz)では動きが追えないという指摘があったので詰める。
  // 内部の計算は 20Hz(経路と同レート)で回っており、ここは表示専用。
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

  // --- 経路の横移動に速度制限を掛ける(2026-08-31)
  //
  // 【ユーザー報告】「追い越すのかと思ったら急に経路が切り替わって抜けない」。
  // 【実測】診断ログ `経路の跳び` が 1周期(50ms)で最大 1.65m。
  //         横方向に 33m/s に相当する非物理的な値。
  //
  // 【順序が重要】バンドのクランプ(すぐ上)より **後** に置くこと。
// 最初に入れたときは band_slope_ の再クランプより前に置いてしまい、
// せっかく制限した値がその後の clamp で破られていた。
// 既存の offset_rate(1.2m/s) が効いていなかったのも同じ理由。
//
// 【原因】バンドの塞ぐ側が入れ替わると lo/hi が約2.6m入れ替わり、
  // クランプを通じて経路にそのまま出る。側の判断に保持時間を入れても、
  // 上流には side_sign_ / band_side_ / 追越側の修正 と複数の経路があり、
  // どれか1つが揺れれば跳ぶ。**出口で速度制限を掛けるのが確実。**
  //
  // pure_pursuit は経路を追うだけなので、経路が跳べば操舵も跳ぶ。
  // 横に動ける速さは物理的に決まっている(旋回で作るしかない)ので、
  // それを超える経路を出しても追従できず、ガタつくだけで得はない。
  if (path_rate_ > 0.0 && prev_offs_.size() == n) {
    const double dt = std::clamp((now - last_offs_time_).seconds(), 0.01, 0.2);
    const double lim = path_rate_ * dt;
    for (size_t i = 0; i < n; ++i) {
      offs[i] = std::clamp(offs[i], prev_offs_[i] - lim, prev_offs_[i] + lim);
    }
  }
  last_offs_time_ = now;

  // 診断: 前の周期からの横目標の跳び。ガタつきの実測用。
  // **レート制限の後**に置くこと。prev_offs_ は「実際に publish した値」で
  // なければ、次の周期の制限が別の基準と比べることになり効かなくなる。
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

  // --- 順位に応じた速度プロファイルの切替
  // バイナリ解析で確定した仕様(reference/parameter.md):
  //   1位のみ driveFadeSpeed が Min(base, 25.0 km/h) に制限される。2位以下は制限なし。
  //   加速度への handicap は無い。
  // 1位では直線が 25 km/h で頭打ちになるので、そこを狙って高い目標速度を置いても
  // 到達しないうえ、加速指令が大きいまま維持されて無駄になる。
  // 代わりにコーナー速度を上げて稼ぐ。
  // 順位ごとに速度プロファイルを変える。
  //   1位: driveFadeSpeed が 25 km/h に制限される(parameter.md)。
  //        直線は伸びないのでコーナーで稼ぐ。
  //   2位以下: 制限が無いので直線も使える。
  // 係数は sweep.sh の順位別計測から決める(rank1_gain / rank2_gain)。
  //
  // 注意: 25.0 は「駆動力が抜け始める速度」であって「到達できる速度」ではない。
  // 目標速度をそこへ丸めると実際にはそれ未満(実測22km/h)しか出ず、かえって遅くなる。
  // よって目標は下げず、fade 未満の区間にだけ係数を掛ける。
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

  // --- 速度上限の調停を確定する(2026-09-02 ユーザー指示)
  // 各層が requestCap() で積んだ要求から最小値を採り、c.speed_cap を確定する。
  // 以降のレート制限・trajectory への適用は、確定後のこの値を使う。
  c.applyCapRequests();
  if ((this->now() - last_cap_arbitration_log_).seconds() > 2.0) {
    last_cap_arbitration_log_ = this->now();
    // 【追加 2026-09-02】上限だけでなく**実速度**と相手速度を出す。
    // 上限が 24km/h でも実際に何km/h 出ているかが分からず、
    // 「並走したまま16秒抜けない」原因を上限側か制御側か切り分けられなかった。
    double ov_sp = -1.0;
    if (!ov_target_.empty()) {
      auto it = others_.find(ov_target_);
      if (it != others_.end() && it->second.valid) {
        ov_sp = std::hypot(it->second.vx, it->second.vy) * 3.6;
      }
    }
    // 【追加 2026-09-03】経路がその地点で指示している速度と、自車の横位置も出す。
    // 上限が当たっていないのに PASS 中だけ実速度が 4.4km/h 低い(FOLLOW 29.1 に
    // 対し PASS 24.7)理由が、横へ出たラインの経路速度なのか、追い越しが起きる
    // 区間が元々低速なだけなのかを区別できなかった。
    const double ref_kmh =
      (f.ei < f.in.points.size())
        ? f.in.points[f.ei].longitudinal_velocity_mps * 3.6 : -1.0;
    diagLog("速度上限", "速度上限 %.1fkm/h 決め手=%s 要求数=%zu 状態=%s 自車=%.1fkm/h "
                "相手=%.1fkm/h 経路速度=%.1fkm/h 横=%.2fm idx=%zu",
                (c.speed_cap < 0.0) ? -1.0 : c.speed_cap * 3.6,
                c.cap_why, c.cap_reqs.size(), ovStateName(),
                std::abs(f.ev) * 3.6, ov_sp, ref_kmh, offset_, f.ei);
  }

  // --- 速度上限の下げ方に、全層まとめてレート制限を掛ける(調停の後の後処理)
  //
  // 【なぜ要るのか(実測 2026-08-29)】
  // Over ペナルティ(|加速度| > 3.0 m/s^2 で 2秒間 5km/h)を
  // **接触ではなく自分の急減速**で受けている。1レースで最大4件。
  // 実測: `Over(2s) idx=22 最寄り=d3 21.13m` — 他車は 21m 先。
  //
  // 速度上限を出す層は5つあり(追従 / 停止車回避 / 衝突回避 / 壁回避 /
  // 追突防止)、それぞれが独立に上限を落とす。追突防止の中だけレート制限を
  // 掛けても、他の層が階段状に落とせば同じことになる。
  // **最後にまとめて掛ける**のが正しい。
  //
  // 車が出せる減速は高々 a_min なので、それ以上の下げ方は指令として
  // 意味が無いうえ、Over を招くだけ。安全性は落ちない
  // (各層の上限は制動距離から作った値で、a_min で追従できる)。
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

  if (c.speed_cap >= 0.0) {
    // 自車の前方だけ速度を抑える。後方まで下げるとレース全体が遅くなる
    for (size_t k = 0; k < n; ++k) {
      const size_t i = (ei + k) % n;
      double gap = s[i] - s[ei];
      if (gap < 0) {
        gap += total;
      }
      if (gap > detect_range_) {
        break;
      }
      out.points[i].longitudinal_velocity_mps =
        std::min<double>(out.points[i].longitudinal_velocity_mps, c.speed_cap);
    }
  }

  // 次の層(manageBoost)が使うので、自車地点の目標速度を残す。
  c.ego_target_speed = out.points[ei].longitudinal_velocity_mps;
  // 次の周期の計測ログ用に、最終的な速度上限を控える。
  last_speed_cap_ = c.speed_cap;

  out.header.stamp = this->now();
  pub_->publish(out);
  {
    std_msgs::msg::Bool ov;
    // 追い越し試行中は pure_pursuit の目標点を近づけ、横オフセットへ
    // 素早く追従させる。
    ov.data = attempt_active_;
    overtaking_pub_->publish(ov);
  }
}

// 残ったブーストの使い道を決めて発射する。
// ブーストは 0.0 に戻してから 1.0 に立ち上げる必要がある(公式仕様)ので
// 2周期かけて撃つ。
void V2XOvertaker::manageBoost(const Frame & f, PlanCtx & c)
{
  const Trajectory & in = f.in;
  const size_t n = f.n;
  const double ex = f.ex;
  const double ey = f.ey;
  const size_t ei = f.ei;
  const rclcpp::Time now = f.now;

  // --- 残ったブーストの使い道 ---
  // ブーストは最高速ではなく「加速度 +0.5 m/s^2 を10秒」上げるだけなので、
  // 既に上限速度に達している場面では効果がない。特に1位は 25km/h で
  // 頭打ちになるため無駄になりやすい。加速余地があることを必須にする。
  //
  // 温存しすぎても価値はゼロ(実測では1レース2個とも未使用だった)。
  // 使ってよいのは次の2つの場面に限る:
  //   (a) 後ろから詰められている  … 抜かれないための加速
  //   (b) 最終ラップ              … もう温存する意味がない
  //
  // --- スタート直後の1本 ---
  //
  // 実測(21:59版で2位だったレース): スタートで3位につけ、最初の60秒を
  // 4倍遅い相手(ラップ153秒)の後ろで潰した。その間に勝者は 213m 先へ行き、
  // 以後 330〜440m 差のまま最後まで届かなかった。
  // このときブーストは2つとも温存され、**1つも使われなかった**。
  //
  // 温存の理由は「抜き返される」ことだったが、抜き返してくるのは
  // 自分より速い相手だけ。序盤で詰まる相手は遅い車なので、
  // 一度抜けば抜き返されない。序盤の損失のほうが遥かに大きい。
  //
  // ただしスタートが1位なら前が空いているので使う必要がない。
  // 2位・3位で始まったときだけ、1つ使って前に出る。
  //
  // 判定はこのブロックの外で作る。run_dist_ は合流が終わるまでしか
  // 進まない(70m で凍る)ので、start_merge_done_ を条件にした中では
  // run_dist_ < start_boost_dist_(40m) は決して成立しない。
  //
  // ただし「グリッドで止まったまま撃つ」のは丸損になる。
  // ブーストの効果は 10 秒だが、実測では 1.2m 前のグリッド車の後ろで
  // 止まったまま約6秒を消費していた(効果の6割が停止中に流れた)。
  // 動き出しているか、前が空いていることを確かめてから撃つ。
  // ブーストの解禁は3周目以降(ユーザー指示 2026-08-29)。
  // 1〜2周目は録画と隊列の整理に使い、ブーストは温存する。
  bool start_push = false;
  if (start_boost_enable_ && boostLapOk() && !start_boost_used_ && start_rank_ >= 2 &&
      boost_used_ == 0 && lap_ < start_boost_laps_ &&
      run_dist_ < start_boost_dist_)
  {
    const double v_now_push = odom_->twist.twist.linear.x;
    // OR にしていたため、実測では 速度1.5m/s・前が1.2m の状態で成立し、
    // 前がつかえたまま撃って丸損した。動き出していて、かつ前が空いている
    // ことの両方を要求する。
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
    // ハンデが効いていない場面(handicap off の計測など)では、現在速度が
    // 上限を超えている。そのときは上限として扱わない。
    // これを入れないと加速余地が負になり、ブーストが一度も撃たれない
    // (実測: 単独走行で発動0回、タイムがブースト無しと同じ 213.18 秒)。
    if (my_v > rank_cap) { rank_cap = 1e9; }
    const double v_reach = std::min<double>(want_v, rank_cap);
    const bool power_limited = my_v > free_boost_min_speed_ &&
                               v_reach - my_v > free_boost_headroom_;

    // (a) 後ろから詰められているか(このサイクルの頭で求めてある)
    const bool pressed = c.pressed_from_behind;
    // (b) 終盤か
    // 「最終ラップだけ」にすると、2個を使い切る前にレースが終わる。
    // 実測では 2個目が7周目(=完走後)に撃たれて丸ごと無駄になっていた。
    // 残り2周から使えるようにして、確実に使い切る。
    // 「終盤まで待つ」制限も外した。free_boost_laps_left を周回数以上に
    // すれば最初から使える。抜けるときに抜くのが最優先。
    const bool last_lap = lap_ >= race_laps_ - free_boost_laps_left_;
    // 自由発射の機会は最終ラップだけに限る(ユーザー指示)。
    // 後方から詰められている・スタート直後、といった理由での発射は、
    // 抜くことに結びつかないまま消えるのでやめる。
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
    // スタート直後の1本は、前が詰まっていても撃つ。
    // 「前が空いていること」を条件にすると、遅い車に詰まっている
    // まさにその場面で撃てない(今回の敗因)。
    // 前に抜くべき相手がいる状態では、自由発射を一切しない。
    // ブーストは「抜くために撃つ」か「前に誰もいないから素直に速く走る」かの
    // どちらかにする。実測では前が詰まったまま(前方空き0)撃って丸損していた。
    // 一方で完全に切ると単独走行でブーストが1発も出ず、
    // タイムが 212.63 -> 214.42 秒に落ちた(ベスト周 34.83 -> 35.37)。
    const bool clear_ok = ahead_clear && c.blocker.empty();
    // スタート直後の1本は「直線か」「加速余地があるか」も問わない。
    //
    // 狙いは合図と同時に前へ出ることなので、条件を待った時点で意味を失う。
    // 実測では、直線判定を待った結果コースを少し進んでから発射しており、
    // 出し抜く効果が無くなっていた。
    // 停止からの加速なので加速余地は必ずあり、スタート地点は
    // グリッドから真っ直ぐ出る区間なので直線判定も本来は不要。
    const bool place_ok = straight || in_boost_zone;
    const bool power_ok = power_limited || start_push;
    // なぜ撃てないのかを条件ごとに残す。
    // 「撃つと決めた」ログだけでは、その後どの条件で落ちたか分からない。
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

  // ================= 最終区間の決めうち =================
  //
  // 【一次情報】parallel.sh: --boosts 2 --laps 6 --handicap on --ranking on。
  //   評価は docs/interface/evaluation-interface.md より final_position のみ。
  // 【AWSIM 実測】VehicleHandicapController.ResolveDriveFadeSpeed は
  //   rank==1 のとき Min(base, 25.0km/h)。2位以下は 36.0。判定は瞬間順位。
  // 【実測 13レース】1位でいた車のベストラップ中央値 47.2s、2位 35.7s。
  //   周長 334.5m なので 334.5/47.2 = 25.5km/h = 上限に貼り付いている。
  //   先頭は 1周あたり 11.5秒(81m)を失う。
  // 【実測】僚車間の逆転6件のうち、5周目に前へ出た4件は4件とも抜き返され、
  //   6〜7周目に出た2件は2件とも守り切った。
  // したがって「先に前へ出る」ことに価値は無く、
  // 「最後にラインを先に切る」ことだけに価値がある。
  // ブーストは有限なので最後に集中させる。追い越しの条件には一切触れない。
  const double dist_to_line   = f.total - f.s[ei];
  const int    laps_left      = std::max(0, race_laps_ - 1 - lap_);
  const double dist_to_finish = laps_left * f.total + dist_to_line;
  const bool   final_dash     = final_dash_enable_ &&
                                (dist_to_finish <= final_dash_dist_);
  // 最終70mだけでは、前車との距離が大きいと追いつくための加速が間に合わない。
  // 実測ではP1が最後の1個を温存したまま最終2周をP2の後ろで走り、発動は
  // 競技の完走推定時刻より後だった。最終2周・2位以下・前車25m以内・直線に
  // 限り、温存分を「追いつくため」に解放する。カーブや前が空いた場面では撃たない。
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
      // 1位は 25km/h で頭打ち。加速余地が無いのに撃つのは丸損。
      // 実測: 43発中13発がこの状態(自車上限=25.0 / 余地1.0〜2.4)で撃たれていた。
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

  // ブーストは 0.0 に戻してから 1.0 に立ち上げる必要がある(公式仕様)
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
