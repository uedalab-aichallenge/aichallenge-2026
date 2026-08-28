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

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using autoware_auto_planning_msgs::msg::Trajectory;
using nav_msgs::msg::Odometry;
using v2x_msgs::msg::V2XVehiclePositionArray;

// 後方この距離[m]より後ろの車は回避の対象にしない。
// 完全に 0 にすると真横の車が前後の判定で揺れるので、少しだけ後ろを許す。
constexpr double kRearIgnore = 1.0;
// カートの車幅[m](実測)。2台が触れずに並ぶには中心間でこれだけ要る。
// 「横に離れている」の判断がこれを下回ると、重なっているのに安全と誤判定する。
constexpr double kCarWidth = 1.30;
// 走行可能領域の境界からこれだけ[m]内側にいるなら、壁には当たっていないとみなす。
constexpr double kContactMargin = 0.35;
// 相手の走行データの要約を出す間隔[s]
constexpr double kStatsLogSec = 20.0;

// --- AWSIM のペナルティ判定(Assembly-CSharp.dll の
//     AIChallenge2026.Penalty.VehiclePenaltyController から実測) ---
//
//   ClampedSpeedMps          = 1.38889  (= 5 km/h)
//   CooldownSecondsP1        = 10.0     Crash
//   CooldownSecondsP2        = 5.0      Wall
//   CooldownSecondsP3        = 2.0      Over
//   WallNormalDotThreshold   = 0.6      壁か車かを接触面の法線で判別
//   OverAccelerationThreshold= 3.0
//   ReverseSpeedThresholdMps = 0.1
//   レイヤー: BumperFront / BumperRear / Vehicle
//
// ペナルティは「時間を足される」のではなく **その秒数だけ 5km/h に固定される**。
// Crash なら 10 秒間 5km/h なので、失う距離は 30m 以上になる。
//
// 前バンパーと後バンパーがレイヤーで分かれており、追突専用の処理
// (HandleRearEndOverlap)がある。つまり Crash は「自分の前で当てた」ときに付き、
// 後ろから当てられた側には付かない。横からの接触は Vehicle レイヤーの
// 衝突として扱われ、追突の判定には入らない。
//
// この構造から、取るべき方針は次のとおり:
//   - 前から突っ込むのは最悪(10秒 5km/h)。止まりきれないなら横へ逃げる
//   - 後ろから当てられるのは無罰。後方の車を避ける必要はない
//   - 横に並んでの軽い接触は追突より安い。抜けるなら多少詰めてよい
using std_msgs::msg::Float32MultiArray;

namespace
{
struct Corridor
{
  std::vector<double> lo;   // 右方向の限界 (負)
  std::vector<double> hi;   // 左方向の限界 (正)
  std::vector<bool> pass_ok;  // ここで追い越してよいか(幅と曲率から事前算出)
  std::vector<double> radius; // その地点の曲率半径[m]。きつい所で壁の余裕を増やすのに使う
};

struct OtherState
{
  double x{0.0}, y{0.0};
  double vx{0.0}, vy{0.0};
  rclcpp::Time stamp;
  bool valid{false};
  // 順位計算用の累積進行度[m]。周回をまたいだら total を足していく
  double prog{0.0};
  double last_s{0.0};
  bool prog_init{false};

  // --- 走行データの蓄積 ---
  // 相手が遅いのか(NPCか、性能が低いのか)を、推測ではなく実測で判断する。
  // 相手を抜けるかどうかの判断は、今の瞬間の速度差だけでは当てにならない
  // (コーナーで一時的に落ちているだけかもしれない)。区間ごとに平均を取る。
  static constexpr int kSections = 24;      // 走行ラインを24分割して集計
  double sec_sum[kSections]{};              // 区間ごとの速度の合計
  int    sec_cnt[kSections]{};              // 同 サンプル数
  double speed_sum{0.0};                    // 全体の平均用
  int    speed_cnt{0};
  double lap_start_time{0.0};               // 周回計測用
  int    last_sec{-1};
  int    laps{0};
  double last_lap_time{0.0};
  double best_lap_time{0.0};

  // --- 相手の走行ラインの学習 ---
  // 相手(既定MPC)はほぼ同じラインを毎周なぞる。1周目に「どの地点で
  // 走行ラインからどれだけ横にいるか」を記録しておけば、
  // 2周目以降は「その地点で自分がどちら側から並べるか」を先に決められる。
  // 目の前の瞬間値だけで側を決めると、相手がラインを横切る場面で
  // 余地の無い側を選んでしまう(実測: 側OK=0 の 48% は反対側なら成立していた)。
  // --- 追い越しの実測 ---
  // 「追越試行 成功」は試行として記録できた分しか数えない。
  // 抜いた/抜かれたを実験の指標にするなら、試行の記録とは切り離して
  // 「前後関係が入れ替わった回数」そのものを数える必要がある。
  int rel_sign{0};        // +1 = 自分が前、-1 = 自分が後ろ、0 = 未確定
  int rel_hold{0};        // 反転を確定させるまでの連続周期数
  int passed_cnt{0};      // この相手を抜いた回数
  int overtaken_cnt{0};   // この相手に抜かれた回数

  static constexpr int kLatBins = 256;      // 走行ラインを256分割して横位置を集計
  double lat_sum[kLatBins]{};
  int    lat_cnt[kLatBins]{};

  // その地点で相手が普段いる横位置[m]。データが足りなければ 1e9 を返す。
  double laneLat(int bin) const
  {
    if (bin < 0 || bin >= kLatBins || lat_cnt[bin] < 3) { return 1e9; }
    return lat_sum[bin] / lat_cnt[bin];
  }
  void noteLat(int bin, double lat)
  {
    if (bin < 0 || bin >= kLatBins) { return; }
    // 単純平均だと1周目の外乱(接触・復帰)が最後まで残る。
    // 十分たまったら指数移動平均に切り替えて直近の走りに追従させる。
    if (lat_cnt[bin] >= 40) {
      lat_sum[bin] = lat_sum[bin] * (39.0 / 40.0) + lat;
      return;
    }
    lat_sum[bin] += lat;
    lat_cnt[bin]++;
  }

  // その区間で相手が普段出している速度[m/s]。データが無ければ負を返す。
  double sectionSpeed(int sec) const
  {
    if (sec < 0 || sec >= kSections || sec_cnt[sec] < 5) { return -1.0; }
    return sec_sum[sec] / sec_cnt[sec];
  }
  double meanSpeed() const
  {
    return speed_cnt >= 20 ? speed_sum / speed_cnt : -1.0;
  }
};
}  // namespace

class V2XOvertaker : public rclcpp::Node
{
public:
  V2XOvertaker()
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
    side_flip_max_(declare_parameter<int>("side_flip_max", 4)),
    // 枠を時間で回復させる。総量制のままだと、同じ相手が長く前にいるレースで
    // 序盤に枠を使い切り、以降ずっと余地の無い側に張り付いたままになる。
    // 実測(6レース、却下ログ476件): 側OK=0 のうち 228件(48%)は
    // 「反対側なら成立していた」場面だった。
    // この秒数につき1回ぶん回復。バースト4回・以降は 1回/この秒数 に制限される。
    side_flip_regen_(declare_parameter<double>("side_flip_regen", 3.0)),
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
    pass_len_(declare_parameter<double>("pass_len", 8.0)),
    pass_time_limit_(declare_parameter<double>("pass_time_limit", 18.0)),
    boost_gain_(declare_parameter<double>("boost_gain", 3.0)),
    boost_retry_sec_(declare_parameter<double>("boost_retry_sec", 6.0)),
    boost_min_speed_(declare_parameter<double>("boost_min_speed", 4.5)),
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
    no_pass_zone_spec_(declare_parameter<std::string>("no_pass_zones", "89:95")),
    // 右側から抜くと決めている区間(ユーザー指示)。書式は boost_zones と同じで
    // 0 をまたぐ指定もできる。既定はメインストレート idx220 -> 30。
    right_zone_spec_(declare_parameter<std::string>("right_zones", "220:30")),
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
    predict_floor_(declare_parameter<double>("predict_floor", 0.55)),
    near_radius_(declare_parameter<double>("near_radius", 6.0)),
    min_lat_sep_(declare_parameter<double>("min_lat_sep", 1.15)),
    min_pass_width_(declare_parameter<double>("min_pass_width", 3.2)),
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
    // 追突の待ちを判定するときに、制動距離へ足す余裕[m]。
    stop_hold_margin_(declare_parameter<double>("stop_hold_margin", 0.5)),
    stopped_cluster_span_(declare_parameter<double>("stopped_cluster_span", 8.0)),
    stopped_slack_(declare_parameter<double>("stopped_slack", 0.5)),
    stopped_thread_speed_(declare_parameter<double>("stopped_thread_speed", 3.0)),
    look_width_ahead_(declare_parameter<double>("look_width_ahead", 20.0)),
    v2x_timeout_(declare_parameter<double>("v2x_timeout", 1.0)),
    safe_gap_(declare_parameter<double>("safe_gap", 3.0)),
    safe_gap_min_(declare_parameter<double>("safe_gap_min", 3.0)),
    safe_gap_max_(declare_parameter<double>("safe_gap_max", 5.0)),
    gap_brake_ratio_(declare_parameter<double>("gap_brake_ratio", 0.5)),
    a_min_(declare_parameter<double>("a_min", 2.5)),
    follow_kp_(declare_parameter<double>("follow_kp", 0.8)),
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
    // 試行中に保持する横オフセットの最小値[m]。決めた側へこれだけは寄せ続ける。
    attempt_hold_side_(declare_parameter<bool>("attempt_hold_side", true)),
    attempt_hold_sep_(declare_parameter<double>("attempt_hold_sep", 1.30)),
    // 横に出られないまま粘る試行を打ち切るまでの秒数。0 で無効(既定)。
    // 有効にするときは 6.0 あたりから。棄却済みの attempt_stall_time とは
    // 見ているものが違う(進展ではなく「横に出られたか」)。
    attempt_latfail_time_(declare_parameter<double>("attempt_latfail_time", 0.0)),
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
    // 停止車回避の横目標が壁帯でこれ[m]以上潰されたら「通れない」と判断する。
    stop_avoid_crush_(declare_parameter<double>("stop_avoid_crush", 0.10)),
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
    // 相手の車体が自分の前にあるとみなす前後距離[m]。この範囲なら寄せると Crash。
    crash_front_near_(declare_parameter<double>("crash_front_near", -0.5)),
    crash_front_far_(declare_parameter<double>("crash_front_far", 3.0)),
    // 壁にも相手にも寄れないときの減速率(現在速度に対する比)
    wall_brake_ratio_(declare_parameter<double>("wall_brake_ratio", 0.6)),
    // きついコーナーで壁の余裕を増やす。半径がこの値を下回ると効き始める。
    tight_radius_(declare_parameter<double>("tight_radius", 10.0)),
    wall_margin_tight_(declare_parameter<double>("wall_margin_tight", 0.35)),
    enable_(declare_parameter<bool>("enable", true))
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
    boost_pub_ = create_publisher<Float32MultiArray>("output/awsim_cmd", rclcpp::QoS(10));
    // 追い越しを試行中かどうかを制御側へ伝える。
    // pure_pursuit は「横にずらした軌道」を受け取るだけで、それが
    // 追い越しのためのものかどうかを知らない。試行中は目標点を近づけて
    // オフセットへ素早く追従させたいので、状態を明示的に渡す。
    overtaking_pub_ = create_publisher<std_msgs::msg::Bool>(
      "output/overtaking", rclcpp::QoS(1));
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

private:
  bool loadCorridor(const std::string & path)
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
    return !corridor_.lo.empty();
  }

  void onV2X(const V2XVehiclePositionArray::SharedPtr msg)
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
  static size_t nearest(const Trajectory & t, double x, double y)
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
  static void normalAt(const Trajectory & t, size_t i, double & nx, double & ny)
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
  void sideRoomMap(const OtherState & o, const Trajectory & in, size_t n,
                   size_t from, double stretch, double olat_now,
                   double & run_left, double & run_right, int & known) const
  {
    run_left = 0.0;
    run_right = 0.0;
    known = 0;
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
      double sf = corridor_safety_;
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
      if (!broke_l) {
        if (sep_l >= min_pass_sep_) { cur_l += step; } else { broke_l = true; }
      }
      if (!broke_r) {
        if (sep_r >= min_pass_sep_) { cur_r += step; } else { broke_r = true; }
      }
      run_left = cur_l;
      run_right = cur_r;
      if (broke_l && broke_r) { break; }
    }
  }

  // ===================================================================
  // onTimer() が使う文脈
  //
  // もともと onTimer() は 2,500 行の単一関数で、22 のブロックが 18 個の
  // ローカル変数を共有して上書きし合っていた。どの層がどれを書き換えるのかが
  // 読めず、実際に「停止車回避が出した横目標を壁回避が黙って潰す」
  // 「同じ意図の対策が 2 箇所にあり片方を直しても効かない」という不具合を生んだ。
  //
  // そこで状態を 2 つに分ける。
  //   Frame   : その周期の観測。**読むだけ**。層は書き換えない。
  //   PlanCtx : 層が積み上げていく指令と中間結果。**書き換わる**。
  //
  // 層のシグネチャを見れば、その層が観測だけ使うのか指令を触るのかが分かる。
  // ===================================================================

  // その周期の観測。全層で共通、書き換えない。
  struct Frame
  {
    const Trajectory & in;          // 入力軌道
    const std::vector<double> & s;  // 各点までの累積距離[m]
    size_t n;                       // 点数
    double total;                   // 周回長[m]
    double ex, ey;                  // 自車位置
    double ev;                      // 自車速度[m/s]
    size_t ei;                      // 自車に最も近い点の添字
    rclcpp::Time now;
  };

  // 層が積み上げる指令と中間結果。
  struct PlanCtx
  {
    // --- 最終的な指令。各層がこの 2 つを詰めていく
    double target_offset{0.0};   // 走行ラインからの横オフセット目標[m]
    double speed_cap{-1.0};      // 速度上限[m/s]。負なら制限なし

    // --- 前方の状況
    std::string blocker;         // 前をふさいでいる相手の名前。空なら前は空き
    double best_gap{0.0};        // その相手までの車間[m]
    bool slow_leader{false};     // 前の相手が明らかに遅い
    bool pressed_from_behind{false};  // 後ろから詰められている

    // --- この地点で並走できるか
    bool in_zone{true};
    double avail_width{1e9};
    double zone_remain{1e9};

    // --- 層ごとの中間結果。次の層が判断に使う
    bool stop_avoid_active{false};  // 停止車を避けようとしている
    double stop_avoid_v_stop{0.0};  // 避けられないときに落とす速度[m/s]
    double avoid_offset{0.0};       // 衝突回避層が出した横オフセット
    double avoid_speed_cap{-1.0};   // 衝突回避層が出した速度上限
    double repulse{0.0};            // 近接車からの横方向の反発

    // publishTrajectory が作った最終軌道の、自車地点での目標速度[m/s]。
    // ブーストの判断が「速度上限とハンデを掛けたあとの目標」を必要とするため、
    // 軌道そのものではなくこの 1 点だけを次の層へ渡す。
    double ego_target_speed{0.0};
  };

  // evaluateOpponent の段の間で受け渡す値。
  // 前半(前方判定 / 側の選択 / 抜けるかの判定)で決まり、後半が読む。
  struct OppEval
  {
    size_t oi{0};          // 相手に最も近い経路点の添字
    double gap{0.0};       // 車間[m]
    double olat{0.0};      // 相手の横位置[m]
    double ospeed_for_gate{0.0};  // 判定に使う相手速度[m/s](予測を含む)
    double my_speed{0.0};      // 自車速度[m/s]
    double v_reach{0.0};       // 自分が出せる上限速度[m/s](順位ハンデ込み)
    double headroom{0.0};      // まだ伸ばせる速度[m/s]
    bool clearly_slower{false};   // 実測で明らかに遅い相手
    bool capped_leader{false};    // 前が1位ハンデで頭打ちになっている
    bool boost_would_help{false}; // ブーストを使えば抜ける
    bool self_ok{false};          // 自力(ブーストなし)で抜ける
    bool allow{false};     // 抜きにいってよいか
  };

  // ===================================================================
  // 20Hz の本体。
  //
  // やることは「観測(Frame)を作り、層を順に呼んで指令(PlanCtx)を積み上げ、
  // 最後に publish する」だけ。層の順序がそのまま優先順位になっている。
  //
  //   観測と記録   logDrivingStats / estimateRank / evaluateZone
  //                checkPressedFromBehind / logStopCause / findFrontCar
  //                learnOpponentLine
  //   走り方を決める planOvertake            追う / 抜く
  //   当たらないようにする
  //                avoidStoppedCars         止まっている車の脇を通す
  //                avoidCollision           他車と壁の回避
  //                repulseFromNearCars      近接車から離れる
  //   横位置を保つ  holdStartLane            スタートのレーン
  //                holdSideBySide           並走中
  //                holdAttemptSide          寄ると決めた側
  //   最終判断     avoidWall                壁が最優先。ここが最後の砦
  //   出力         applyOffsetRateLimit / publishTrajectory / manageBoost
  //
  // **後の層ほど強い。** avoidWall が横目標を潰したら、それが最終指令になる。
  // 潰されたことを前の層へ知らせないと「避けられない位置で全開前進」になるので、
  // 必要なものは PlanCtx を通して受け渡すこと(stop_avoid_active がその例)。
  // ===================================================================
  void onTimer()
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

    logDrivingStats(f);
    estimateRank(f);
    evaluateZone(f, c);
    checkPressedFromBehind(f, c);
    logStopCause(f);
    findFrontCar(f);

    learnOpponentLine(f);

    planOvertake(f, c);

    avoidStoppedCars(f, c);

    avoidCollision(f, c);

    repulseFromNearCars(f, c);

    holdStartLane(f, c);

    holdSideBySide(f, c);

    recordAttempt(f, c);
    applyAvoidance(f, c);

    holdAttemptSide(c);

    avoidWall(f, c);
    applyOffsetRateLimit(c);
    publishTrajectory(f, c);

    manageBoost(f, c);

    logBlocker(f, c);
  }

  // ---- ここから onTimer から切り出した層 ----

  // この相手を抜きにいってよいかを決める。evaluateOpponent の中心。
  //
  // 見るもの: 幅が足りるか / 抜き切るのに要る距離が使える距離に収まるか /
  // 相手が実測で明らかに遅いか / 同速の相手を無理に攻めていないか /
  // 自分が1位ハンデ中でないか / 追い越し禁止区間でないか。
  // それらを ev.allow に畳み込み、後段(chargeBoost / followAndCommit)が使う。
  void decideAllow(const Frame & f, PlanCtx & c, const std::string & name,
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
    const bool width_ok = c.avail_width >= w_need;
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
    const bool feasible = side_fits_ &&
                         ((c.slow_leader && width_ok) || self_ok || boost_would_help);
    // 速度差が十分大きければゾーン外でも抜く。
    // ゾーンは「並走しても安全な区間」の目安だが、相手が明らかに遅ければ
    // 並走時間そのものが短いのでゾーンで縛る必要がない。
    // これが無いと周回遅れを直線で抜けない(実測: 試行47回 成功0回)。
    // (clearly_slower は closing の足切り免除にも使うため、self_ok の前で算出済み)
    // 明らかに遅い相手には、直線の手前(加速区間)からでも仕掛けてよい。
    // ブーストを使わずに抜けるので、終盤まで温存する必要がない。
    bool in_accel_zone = false;
    for (const auto & z : boost_zones_) {
      const bool inside = (z.first <= z.second)
                            ? (ei >= z.first && ei <= z.second)
                            : (ei >= z.first || ei <= z.second);
      if (inside) { in_accel_zone = true; break; }
    }
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
    // 最下位なら、追い越し可能ゾーンの外でも仕掛けてよい
    const bool zone_ok = c.in_zone || c.slow_leader || boost_would_help
                         || (aggressive && in_accel_zone)
                         || (clearly_slower && in_accel_zone)
                         || (closing_max * 3.6 >= big_gap_closing_);
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
    const bool latched = attempt_active_ && latch_width_ok &&
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
    const bool allow = (zone_ok && feasible && !in_no_pass && !stall_block) || latched;
    // 追い越しが途中で降りる原因を追うため、判定の中身を残しておく。
    dbg_allow_ = allow; dbg_width_ = c.avail_width; dbg_zone_ = c.in_zone;
    dbg_latched_ = latched; dbg_feasible_ = feasible; dbg_zone_ok_ = zone_ok;

    // 却下された理由を残す(パラメータ調整のため)
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
      const char * why =
          in_no_pass                     ? "禁止区間"
        : stall_block                    ? "打切直後"
        : !side_fits_                    ? "側の余地なし"
        : !width_ok                      ? "幅不足"
        : (pass_time(t_accel) > t_limit) ? "時間超過"
        : (pass_dist(t_accel) > usable)  ? "ゾーン残距離不足"
        : !(c.slow_leader || clearly_slower || capped_leader ||
            capped_self || closing_ok)   ? "速度差不足"
        : !zone_ok                       ? "ゾーン外"
        :                                  "その他";
      RCLCPP_INFO(get_logger(),
        "追越却下 決め手=%s self_ok=%d capped自=%d capped先=%d 遅相手=%d ブ助=%d "
        "gap=%.1f zone=%d %s 幅=%.1f(要%.1f) 残距離=%.0f "
        "v_reach=%.1f 相手=%.1f closing=%.1f 所要=%.1fs 距離=%.0f rank=%d "
        "側OK=%d 側=%s 相手横=%.2f 余地=[%.2f,%.2f] idx=%zu 同速=%d 禁止区=%d "
        "枠=%d/%d 不成立=%.1fs 学習連続=[%.1f,%.1f]m/%d点",
        why, self_ok ? 1 : 0, capped_self ? 1 : 0, capped_leader ? 1 : 0,
        clearly_slower ? 1 : 0, boost_would_help ? 1 : 0,
        gap, c.in_zone ? 1 : 0, inside ? "イン" : "アウト", c.avail_width, w_need, usable,
        v_reach * 3.6, ospeed_for_gate * 3.6, closing_max * 3.6,
        pass_time(t_accel), pass_dist(t_accel), rank_,
        side_fits_ ? 1 : 0, (side_sign_ > 0.0) ? "左" : "右",
        olat, room_lo_, room_hi_, ei,
        (!c.slow_leader && !closing_ok) ? 1 : 0, in_no_pass ? 1 : 0,
        side_flip_cnt_, side_flip_max_,
        (side_unfit_since_ >= 0.0) ? (now.seconds() - side_unfit_since_) : -1.0,
        dbg_map_l_, dbg_map_r_, dbg_map_n_);
    }
    c.target_offset =
      allow ? std::clamp(olat + side_sign_ * pass_gap_, room_lo_, room_hi_) : 0.0;
    // 追い越しが成立しているかは「走行ラインからどれだけ離れたか」ではなく
    // 「相手からどれだけ横に離れたか」で見る。
    // 相手がラインから外れている場合、正しい追い越し位置が
    // ライン上(オフセット約0)になることがあり、
    // |target_offset| で判定すると横に出た瞬間に失敗と数えてしまう
    // (実測: 相手が +1.55m にいて目標 -0.15m、間隔は 1.7m 取れているのに失敗扱い)。
    pass_sep_ = allow ? (c.target_offset - olat) : 0.0;
    can_pass_now_ = allow;

    ev.oi = oi; ev.gap = gap; ev.olat = olat;
    ev.ospeed_for_gate = ospeed_for_gate;
    ev.my_speed = my_speed; ev.v_reach = v_reach; ev.headroom = headroom;
    ev.clearly_slower = clearly_slower; ev.capped_leader = capped_leader;
    ev.boost_would_help = boost_would_help; ev.self_ok = self_ok;
    ev.allow = allow;
  }


  // どちら側から抜くかを決める。
  //
  // 相手の横位置と走行可能領域から左右それぞれの余地を出し、学習した
  // 相手のラインも加味して side_sign_ を決める。一度決めた側は
  // 抜き切るか失敗が確定するまで保持する(ユーザー方針)。余地が無いままなら
  // 一度だけ反対側へ回る。
  //
  // 相手の横位置 olat と、判定に使う相手速度 ospeed_for_gate を返す。
  void chooseSide(const Frame & f, PlanCtx & c, const OtherState & o,
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
    if (lane_map_side_) {
      sideRoomMap(o, in, n, oi, lane_map_stretch_, olat, map_left, map_right, map_known);
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
      // --- 右側から抜くと決めている区間(ユーザー指示)
      // メインストレート(idx220 -> 30)は右から抜く。
      // side_sign_ は +1 が左、-1 が右(ログの表記と同じ)。
      bool in_right_zone = false;
      for (const auto & z : right_zones_) {
        const bool inside = (z.first <= z.second)
                              ? (ei >= z.first && ei <= z.second)
                              : (ei >= z.first || ei <= z.second);
        if (inside) { in_right_zone = true; break; }
      }
      if (in_right_zone) { want = -1.0; }
      if (curve_sign_ != 0.0) {
        const double inside_sign = (curve_sign_ > 0.0) ? +1.0 : -1.0;
        const bool inside_fit = (inside_sign > 0.0) ? fit_left : fit_right;
        if (inside_fit) { want = inside_sign; }
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
  }


  // この相手が「いま自分の前をふさいでいる最も近い1台」かを判定する。
  //
  // 進行度差だけで前後を判定すると、スタートのグリッドのように横に並んだ車が
  // 「差ほぼ0」で前車として検出されず、速度制限が掛からないまま追突する。
  // 車体の向きから見た実際の前方距離でも判定し、小さい方を車間として採る。
  //
  // 該当すれば c.best_gap / c.blocker を更新して true。そうでなければ false
  // (呼び出し側はこの相手の評価を打ち切る)。
  bool isNearestBlocker(const Frame & f, PlanCtx & c, const std::string & name,
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


  // 追い越しのための助走ブーストを撃つか決める。
  //
  // ユーザー方針: 「抜くために少し手前からどんどん加速していき、
  // 相手が 25km/h しか出せないところを追い抜く」。
  // ブーストは 10 秒持続するので並んでから撃つのでは遅い。
  // ゾーンが射程に入った時点で、並ぶ前に撃つ。
  void chargeBoost(const Frame & f, PlanCtx & c, const OppEval & ev)
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
        headroom > boost_min_headroom_ && c.in_zone &&
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
    const bool late_enough = lap_ >= boost_hold_laps_;
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
  void followAndCommit(const Frame & f, PlanCtx & c,
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
    if (approach_enable_ && !can_pass_now_) {
      // 次に仕掛けられる場所までの距離を測る
      double d_zone = -1.0;
      {
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
        const double o_zone_v = o.sectionSpeed(zsec);
        const double o_ref = (o_zone_v > 0.0) ? std::max(ospeed, o_zone_v) : ospeed;
        const double closing = std::max(rank_cap - o_ref, 0.0);
        const double t_zone = d_zone / std::max(v_now, 1.0);
        // 仕掛けどころで pass_gap になるように、今は余分に空けておく
        const double want = pass_gap_ + closing * t_zone;
        eff_safe = std::clamp(std::max(eff_safe, want), safe_gap_min_, approach_gap_max_);
        // 車間が目標より大きければ助走(全開で詰める)。
        charge_now = gap > eff_safe + 0.3;
        // 入口が目前なら、逆算値ではなく「抜くのに要る車間」まで詰めきる。
        // want は入口到達時に pass_gap になる想定だが、追従則の遅れで
        // 実測では入口に 5-6m 残ったまま入っていた。ここだけ目標を
        // pass_gap*k に下げ、追従キャップを外す時間を伸ばす。
        // 追突(Crash)は前方接触のみ罰なので TTC・緊急減速(avoid_speed_cap)
        // が最後の砦になる。これらはチャージ中も別段で効く。
        if (d_zone < charge_close_dist_) {
          charge_now = gap > pass_gap_ * charge_close_gap_k_;
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
        const double closing_now = std::max(my_speed_for_gap_ - ospeed, 0.0);
        const double brake_need =
          closing_now * closing_now / (2.0 * std::max(std::abs(a_min_), 0.5));
        if (gap < brake_need + charge_brake_margin_) {
          charge_now = false;
        }
      }
    }
    const double eff_follow = std::min(dyn_safe * 1.8, safe_gap_max_ * 1.8);
    // 横へ出て抜きにいっている最中は速度を抑えない。
    // 抑えると前車と同じ速度に張り付いて永久に抜けない(ブーストも無駄になる)。
    // 相手が遅い(止まりかけている)なら「追い越し中だから制限しない」を
    // 適用しない。実測(3レース中2レース): 減速中の相手に対し passing_now が
    // 立ち、車間 2.3〜4.0m で「上限=-1.0(制限なし)」のまま接触した。
    bool passing_now = allow && std::abs(offset_) > pass_gap_ * 0.5;
    if (ospeed_for_gate < slow_leader_speed_) { passing_now = false; }
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
    if (commit_pass_ && allow && ospeed_for_gate > slow_leader_speed_) {
      // 解除側でも**車幅を下回らせない**。重なった状態で加速を続けないため。
      const double need_sep = commit_now_
        ? std::max(commit_sep_ * commit_release_, kCarWidth)
        : commit_sep_;
      const double need_gap = commit_now_ ? commit_gap_ * 1.5 : commit_gap_;
      commit_now = (std::abs(lat_sep_now) >= need_sep) && (gap < need_gap);
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
    if (commit_now && commit_boost_time_ > 0.0 && commit_since_ > 0.0 &&
        (now.seconds() - commit_since_) >= commit_boost_time_ &&
        boost_remaining_ > 0 && !is_boosting_ && !want_boost_)
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
    if (gap < eff_follow && !charge_now && !commit_now) {
      const double v_target = ospeed + follow_kp_ * (gap - follow_safe);
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
      c.speed_cap = std::max(floor, v_target);
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
  void evaluateOpponent(const Frame & f, PlanCtx & c,
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


  // 溜めた走行データの要約を定期的に出す。
  void logDrivingStats(const Frame & f)
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
  void estimateRank(const Frame & f)
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

  // この区間で並走できるかを判定する(使える幅と、その区間の残り距離)。
  void evaluateZone(const Frame & f, PlanCtx & c)
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
        double safety = attempt_active_ ? corridor_safety_pass_ : corridor_safety_;
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
  void checkPressedFromBehind(const Frame & f, PlanCtx & c)
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
  void logStopCause(const Frame & f)
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
  void findFrontCar(const Frame & f)
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
  void learnOpponentLine(const Frame & f)
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
        os.noteLat(static_cast<int>(li * OtherState::kLatBins / n), llat);

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

  // 前方の相手を追う / 抜くかを決める中心の層。
  // 追従の車間制御・追い越しの可否判定・側の選択・助走ブーストを含む。
  void planOvertake(const Frame & f, PlanCtx & c)
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
    }
  }

  // 停止車両への突入を防ぐ。複数台が同じ場所で止まっている場合を含む。
  // 止まっている車の集団に対して空いている横位置の区間を求め、
  // 通れる区間があればそこへ、無ければ手前で止まる。
  void avoidStoppedCars(const Frame & f, PlanCtx & c)
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
      const double brake_a = std::max(std::abs(a_min_), 0.5);
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

      if (!stopped.empty()) {
        std::sort(stopped.begin(), stopped.end(),
                  [](const StoppedCar & a, const StoppedCar & b) { return a.gap < b.gap; });
        const double base = stopped.front().gap;
        const size_t bi = stopped.front().idx;

        // その地点で使える横方向の範囲
        double lo = -3.0, hi = 3.0;
        if (corridor_.lo.size() == n) {
          lo = corridor_.lo[bi] + corridor_safety_;
          hi = corridor_.hi[bi] - corridor_safety_;
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
        if (best_w >= stopped_slack_) {
          // 余裕をもって通れる。帯の中央へ寄せる。
          c.target_offset = band_center;
          c.stop_avoid_active = true;
          c.stop_avoid_v_stop = v_stop;
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
          if (c.speed_cap >= 0.0 && !hold_now) {
            c.speed_cap = std::max(c.speed_cap, stopped_thread_speed_);
          }
          act = hold_now ? "通過(近いので待つ)" : "通過";
        } else if (best_w > 0.0) {
          // かろうじて通れる。速度を落として通す。ここも帯の中央を狙う。
          c.target_offset = band_center;
          c.stop_avoid_active = true;
          c.stop_avoid_v_stop = v_stop;
          c.speed_cap = (c.speed_cap < 0.0) ? stopped_thread_speed_
                                        : std::min(c.speed_cap, stopped_thread_speed_);
          act = "徐行通過";
        } else {
          // 通れない。手前で止まる。
          c.speed_cap = (c.speed_cap < 0.0) ? v_stop : std::min(c.speed_cap, v_stop);
          act = "停止";
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
                        (c.speed_cap < 0.0 ? 99.0 : c.speed_cap) * 3.6, c.target_offset);
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
  void avoidCollision(const Frame & f, PlanCtx & c)
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
        const double dir = (room_left >= room_right) ? 1.0 : -1.0;
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
                        "正面衝突を回避 TTC%.2fs 横へ%.2fm(%s) 空き[左%.2f 右%.2f]",
                        worst_ttc, push, dir > 0 ? "左" : "右", room_left, room_right);
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
        const double lo = corridor_.lo[ei] + corridor_safety_;
        const double hi = corridor_.hi[ei] - corridor_safety_;
        // 必要なぶん丸ごと入らないときに横移動を諦めると、狭い区間では
        // まったく避けずに減速だけになる(実測: 接触したのに 横=0.00m)。
        // 入るところまで寄せる。半分でも寄せたほうが当たり方は軽くなる。
        const double want_cand = my_lat_for_target_ + push;
        const double cand_a = std::clamp(want_cand, lo, hi);
        const double cand_b = std::clamp(my_lat_for_target_ - push, lo, hi);
        const double got_a = cand_a - my_lat_for_target_;   // 望んだ側で実際に寄れる量
        const double got_b = cand_b - my_lat_for_target_;   // 逆側で寄れる量
        double cand = cand_a;
        if (std::abs(got_b) > std::abs(got_a) + 1e-3) {
          cand = cand_b;                                    // 逆側のほうが寄れる
        }
        const double got = cand - my_lat_for_target_;
        if (std::abs(got) < std::abs(push) * 0.5) {
          // 半分も寄れないなら、横移動だけでは足りない。減速も併用する。
          c.avoid_speed_cap = std::max(my_speed_for_gap_ * 0.5, min_follow_speed_);
        }
        push = got;
        if (std::abs(push) > 1e-3) {
          c.avoid_offset = cand;
        }
      }
    }
  }

  // 近接車から横方向へ反発する。
  void repulseFromNearCars(const Frame & f, PlanCtx & c)
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
    if (std::abs(c.repulse) > 1e-3 && can_pass_now_) {
      // 反発は追い越し要求より優先する。接触は crash ペナルティで最も重い。
      // ただし並走できない幅しかない区間では横に逃げても壁に当たるだけなので行わない
      // (その場合は下の追従制御で車間を空ける)。
      c.target_offset = my_lat_for_target_ + c.repulse;
      if (c.blocker.empty()) {
        c.blocker = "近接車";
      }
    }
  }

  // スタート直後はグリッドの横位置を保持し、距離とともに 0 へ減衰させる。
  // 全車が一斉に同じレースラインへ収束して団子になるのを防ぐ。
  void holdStartLane(const Frame & f, PlanCtx & c)
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
        // グリッドは壁際にも置かれる(スタート地点の左限界は +1.7 m しかないのに
        // +1.30 m のスロットがある)。その位置をそのまま保持すると車体が壁に接し続け、
        // 一度当たると抜け出せなくなる。保持量に上限を設けて壁から離す。
        start_lat_ = std::clamp(start_lat_, -start_lat_max_, start_lat_max_);
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

          // **P1(最後尾)なら P2 の側 = 右へ寄る**(ユーザー指示)。
          // P3(最前列)は運営コンピュータで確定しており 25km/h のハンデで遅い。
          // P1 から P3 のラインに付くとその遅い車の後ろに並ぶことになって損なので、
          // 抜くべき相手である P2 の側へ出る。
          //
          // 横位置は **負が右**(コリドアの lo が右方向の限界)。
          // P2 の実際の横位置と「右」の既定値のうち、より右側を採る。
          if (start_slot_ == 1 && start_slot_lat_.size() >= 2) {
            const double p2_lat = start_slot_lat_[1];
            const double want = std::min(p2_lat, -std::abs(start_p3_lat_));
            start_lat_ = std::clamp(want, -start_lat_max_, start_lat_max_);
            RCLCPP_INFO(get_logger(),
                        "P1スタート: P2側(右)へ寄せる 横=%.2f (P2の横=%.2f)",
                        start_lat_, p2_lat);
          }
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
          if (c.blocker.empty()) {
            c.target_offset = start_lat_ * w;
          } else {
            c.target_offset = c.target_offset * (1 - w) + start_lat_ * w;
          }
        }
      }
    }
  }

  // 並走中は横オフセットを保持する。
  // 横に並ぶと相手が前方帯から外れて検出されなくなり、目標が 0 に戻ってしまうため。
  void holdSideBySide(const Frame & f, PlanCtx & c)
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
        const double lo = corridor_.lo[ei] + corridor_safety_;
        const double hi = corridor_.hi[ei] - corridor_safety_;
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

  // 追い越しの試行・成功・失敗を数えてログに残す。sweep.sh がこれを集計する。
  void recordAttempt(const Frame & f, PlanCtx & c)
  {
    const double total = f.total;
    const rclcpp::Time now = f.now;

    // --- 追い越しの試行と結果を記録する
    // 「横に出て抜きにいった」を試行開始、「相手を前後で追い越した」を成功、
    // 「横に出たが抜けずにラインへ戻った」を失敗として数える。
    // sweep.sh がこのログを集計してパラメータの良し悪しを判断する。
    {
      // 試行の開始は「本当に横へ出る指令が出たとき」にする。
      // min_pass_sep*0.5(0.58m)では、追従中のわずかな横ずれまで試行と数えて
      // 「試行47回・成功0回」のような数字になり、何が本当の仕掛けか読めなかった。
      // pass_gap の 70%(1.9m -> 1.33m)まで寄せる指令が出て初めて試行とみなす。
      const bool moving_out = std::abs(pass_sep_) > pass_gap_ * 0.7;
      if (!attempt_active_ && moving_out && !c.blocker.empty()) {
        attempt_active_ = true;
        attempt_target_ = c.blocker;
        attempt_start_ = now.seconds();
        attempt_boosts_ = boost_used_;
        attempt_offset_ = c.target_offset;
        attempt_fail_since_ = -1.0;
        attempt_lead_cnt_ = 0;
        attempt_diff0_ = 1e18;
        attempt_max_sep_ = 0.0;
        attempt_latok_ = false;
        RCLCPP_INFO(get_logger(), "追越試行 開始 target=%s 車間=%.1fm rank=%d",
                    c.blocker.c_str(), c.best_gap, rank_);
      } else if (attempt_active_) {
        // 前方車が見えている間は保持値を最新の指令で更新する
        if (moving_out) {
          attempt_offset_ = c.target_offset;
        }
        // 対象車が自分より後ろに回ったら成功。
        // 累積進行度の差をそのまま見ると、周回のまたぎで一時的に大きく振れて
        // 開始直後に「成功」と誤判定していた(所要0.0秒の記録が大量に出た)。
        // 差が妥当な範囲(1周未満)にあるときだけ判定する。
        bool passed = false;
        bool stalled = false;
        auto it = others_.find(attempt_target_);
        if (it != others_.end() && it->second.valid) {
          const double diff = my_prog_ - it->second.prog;
          // 進行度差は周回のまたぎで大きく振れる。妥当な範囲のときだけ使う。
          if (attempt_stall_time_ > 0.0 && std::abs(diff) < total * 0.5) {
            if (attempt_diff0_ > 1e17) { attempt_diff0_ = diff; }
            else if ((now.seconds() - attempt_start_) > attempt_stall_time_ &&
                     (diff - attempt_diff0_) < attempt_stall_gain_) {
              stalled = true;
            }
          }
          if (diff > pass_len_ * 0.5 && diff < total * 0.5) {
            passed = true;
          }
          // pass_len*0.5(+4m)まで離れる前に相手が消える/減速が終わることがあり、
          // 実際に前に出ているのに成功として記録されなかった。
          // 「前に出ている」が3周期続いたらそれも成功とみなす。
          if (diff > 0.0 && diff < total * 0.5) {
            if (++attempt_lead_cnt_ >= 3) { passed = true; }
          } else {
            attempt_lead_cnt_ = 0;
          }
        }
        const double elapsed = now.seconds() - attempt_start_;
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
          RCLCPP_INFO(get_logger(), "追越試行 成功 target=%s 所要=%.1fs ブースト=%d rank=%d",
                      attempt_target_.c_str(), elapsed,
                      boost_used_ - attempt_boosts_, rank_);
        } else if (attempt_fail_since_ >= 0.0 && elapsed > 1.0 &&
                   (now.seconds() - attempt_fail_since_) >= 0.5) {
          attempt_active_ = false;
          attempt_ng_++;
          RCLCPP_INFO(get_logger(),
                      "追越試行 失敗 target=%s 所要=%.1fs ブースト=%d rank=%d "
                      "横間隔=%.2f(要%.2f) 最大実測横間隔=%.2f allow=%d zone=%d "
                      "zone_ok=%d feasible=%d "
                      "latch=%d 幅=%.1f",
                      attempt_target_.c_str(), elapsed,
                      boost_used_ - attempt_boosts_, rank_,
                      pass_sep_, min_pass_sep_ * 0.3, attempt_max_sep_, dbg_allow_ ? 1 : 0,
                      dbg_zone_ ? 1 : 0, dbg_zone_ok_ ? 1 : 0,
                      dbg_feasible_ ? 1 : 0, dbg_latched_ ? 1 : 0, dbg_width_);
        } else if (stalled || lat_stalled || elapsed > attempt_timeout_) {
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
                      stalled ? "進展なし" : (lat_stalled ? "横に出られない" : "時間切れ"),
                      attempt_max_sep_, commit_sep_);
        }
      }
    }

  }

  // 衝突回避層の結果を最優先で指令へ反映する。
  // 追い越しの都合より当たらないことを優先する(Crash 10秒 / Wall 5秒)。
  void applyAvoidance(const Frame & f, PlanCtx & c)
  {
    const rclcpp::Time now = f.now;

    // --- 衝突回避を最優先で適用する
    // 追い越しの都合より、当たらないことを優先する。
    // Crash は 10 秒 5km/h、Wall は 5 秒 5km/h と罰則が重く、
    // 追い越し1回の利得より損失が大きい。
    dbg_avoid_offset_ = c.avoid_offset;
    dbg_avoid_cap_ = c.avoid_speed_cap;
    if (std::abs(c.avoid_offset) > 1e-3) {
      c.target_offset = c.avoid_offset;
      if (c.blocker.empty()) {
        c.blocker = "回避";
      }
    }
    if (c.avoid_speed_cap >= 0.0) {
      c.speed_cap = (c.speed_cap < 0.0) ? c.avoid_speed_cap
                                    : std::min(c.speed_cap, c.avoid_speed_cap);
      if ((this->now() - last_avoid_log_).seconds() > 2.0) {
        last_avoid_log_ = this->now();
        RCLCPP_INFO(get_logger(), "衝突回避 減速=%.1fkm/h 横=%.2fm",
                    c.avoid_speed_cap * 3.6, c.avoid_offset);
      }
    }
  }

  // 追い越し試行中は「寄ると決めた側」を保持しきる(ユーザー方針)。
  // 一度寄る方向を決めたら、抜き切るか失敗が確定するまで戻さない。
  void holdAttemptSide(PlanCtx & c)
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
    if (attempt_hold_side_ && attempt_active_ && !c.blocker.empty()) {
      const double want = side_sign_ * attempt_hold_sep_;
      // コリドアの余地には従う(壁側へは出ない)
      const double held = std::clamp(want, room_lo_, room_hi_);
      // すでに同じ側へ十分寄っているならそのまま。足りないときだけ引き上げる。
      if (side_sign_ * c.target_offset < side_sign_ * held) {
        c.target_offset = held;
      }
    }
  }

  // 壁を最優先で避ける。避けきれないときだけ相手側へ寄る。
  // ここは横目標を決める最後の段。以降はレート制限を掛けて出すだけ。
  void avoidWall(const Frame & f, PlanCtx & c)
  {
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
      // 帯は必ず走行ラインを含める(142 の失敗を繰り返さない)
      double wall_lo = std::min(corridor_.lo[ei] + margin, 0.0);
      double wall_hi = std::max(corridor_.hi[ei] - margin, 0.0);
      if (wall_hi > wall_lo) {
        const double want = c.target_offset;
        const double safe = std::clamp(want, wall_lo, wall_hi);
        // まず横目標は必ず帯の中に収める。これだけで「壁へ寄せる指令」は出なくなる。
        // 単独走行(他車なし)ではここで終わり。減速は一切しない。
        c.target_offset = safe;
        const bool pushed_to_wall = std::abs(want - safe) > 1e-3;

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
        if (c.stop_avoid_active && std::abs(want - safe) > stop_avoid_crush_) {
          c.speed_cap = (c.speed_cap < 0.0) ? c.stop_avoid_v_stop
                                        : std::min(c.speed_cap, c.stop_avoid_v_stop);
          if ((this->now() - last_crush_log_).seconds() > 1.0) {
            last_crush_log_ = this->now();
            RCLCPP_WARN(get_logger(),
                        "停止車を避けきれない 横目標 %.2f が壁帯[%.2f,%.2f]で "
                        "%.2f に潰された -> 上限 %.1fkm/h へ減速",
                        want, wall_lo, wall_hi, safe, c.stop_avoid_v_stop * 3.6);
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
            // 壁から離れる向きへ、相手との間隔を crash_safe_sep まで詰めてよい。
            const double toward = nearest_lat + crash_safe_sep_ * wall_dir;
            c.target_offset = std::clamp(toward, wall_lo, wall_hi);
            if ((this->now() - last_wallpick_log_).seconds() > 1.0) {
              last_wallpick_log_ = this->now();
              RCLCPP_INFO(get_logger(),
                          "壁回避 相手側へ寄る 横目標 %.2f -> %.2f (壁側=%s "
                          "相手横=%.2f 自車横=%.2f 帯=[%.2f,%.2f])",
                          want, c.target_offset, wall_dir > 0 ? "左" : "右",
                          nearest_lat, my_lat_for_target_, wall_lo, wall_hi);
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
            c.speed_cap = (c.speed_cap < 0.0) ? slow : std::min(c.speed_cap, slow);
            if ((this->now() - last_wallpick_log_).seconds() > 1.0) {
              last_wallpick_log_ = this->now();
              RCLCPP_INFO(get_logger(),
                          "壁回避 減速で両方避ける 横目標 %.2f -> %.2f 上限 %.1fkm/h "
                          "(壁側=%s 前に相手あり)",
                          want, c.target_offset, slow * 3.6, wall_dir > 0 ? "左" : "右");
            }
          }
        }
      }
    }

  }

  // 決まった横目標へ、レート制限つきで現在のオフセットを近づける。
  void applyOffsetRateLimit(PlanCtx & c)
  {
    // --- オフセットをレート制限つきで目標へ動かす
    const double dt = 0.05;
    const double step = offset_rate_ * dt;
    if (c.target_offset > offset_) {
      offset_ = std::min(c.target_offset, offset_ + step);
    } else {
      offset_ = std::max(c.target_offset, offset_ - step);
    }

  }

  // 横オフセットを乗せた軌道を作り、順位に応じた速度上限を掛けて publish する。
  void publishTrajectory(const Frame & f, PlanCtx & c)
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
      if (have_corr) {
        off = std::clamp(off, corridor_.lo[i] + corridor_safety_,
                         corridor_.hi[i] - corridor_safety_);
      }
      double nx, ny;
      normalAt(in, i, nx, ny);
      out.points[i].pose.position.x += off * nx;
      out.points[i].pose.position.y += off * ny;
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

    out.header.stamp = this->now();
    pub_->publish(out);
    {
      std_msgs::msg::Bool ov;
      ov.data = attempt_active_;
      overtaking_pub_->publish(ov);
    }
  }

  // 残ったブーストの使い道を決めて発射する。
  // ブーストは 0.0 に戻してから 1.0 に立ち上げる必要がある(公式仕様)ので
  // 2周期かけて撃つ。
  void manageBoost(const Frame & f, PlanCtx & c)
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
    bool start_push = false;
    if (start_boost_enable_ && !start_boost_used_ && start_rank_ >= 2 &&
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
    if (!want_boost_ && free_boost_enable_ && boost_remaining_ > 0 && !is_boosting_ &&
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
  void logBlocker(const Frame & f, PlanCtx & c)
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

  const double detect_range_;
  const double front_lane_half_;   // 自分の進路とみなす横幅の半分[m]
  const double contact_vehicle_dist_;
  const double contact_log_hold_;  // これ以内に他車がいれば車両接触とみなす[m]
  const double avoid_range_;       // 衝突回避で見る範囲[m]
  const double collision_radius_;  // 衝突とみなす半径[m](車体2台分)
  const double ttc_threshold_;     // この時間[s]以内に衝突しそうなら回避する
  const double big_gap_closing_;   // 速度差[km/h]がこれ以上なら距離制限を外す
  const double inside_time_gain_;  // イン側から抜くときの所要時間の許容倍率
  const double inside_width_gain_;
  const double latch_width_gain_; // イン側から抜くときの必要幅の倍率
  double curve_sign_{0.0};         // 現在地点の曲率の向き (+1=左旋回)
  const double pass_gap_;
  const double follow_gap_;
  const double offset_rate_;
  const double corridor_safety_;
  const double corridor_safety_pass_;  // 追い越し試行中に使う縁からの余裕[m]
  const double corridor_safety_zone_;  // 追い越し可能ゾーンで使う縁からの余裕[m]
  const double side_room_ahead_;   // 左右の余地を見る先読み距離[m]
  const double side_flip_hold_;    // 余地なしがこの秒数続いたら反対側へ回る[s]
  const int side_flip_max_;        // 対象車1台につき側を変更してよい回数(バースト)
  const double side_flip_regen_;   // 変更の枠がこの秒数につき1回ぶん回復する
  const bool lane_learn_;          // 相手の横位置を地点ごとに学習するか
  const bool lane_map_side_;       // 学習結果を側の判断に使うか
  const double lane_map_stretch_;  // 側の判断で見る先の距離[m]
  const int lane_map_min_pts_;     // 学習点がこの数以上あるときだけ信用する
  const double lane_map_margin_;   // 足りている連続区間がこの差[m]を超えたら長いほうを選ぶ
  const double lane_map_need_gain_; // 成立に要する連続区間 = pass_len * これ
  const double zone_look_ahead_;   // 追い越しゾーンを探す先読み距離[m]
  const double window_full_;
  const double window_end_;
  const double window_back_;
  const double start_merge_dist_;
  const double start_lat_max_;
  const bool start_p3_follow_;   // 3位スタートで2位側へ寄せるか
  const double start_p3_lat_;    // 3位スタートで寄せる横位置[m](正=左, 負=右は符号次第)
  const double stop_hold_gap_;   // 停止車がこれ[m]より近ければ下限速度を課さない
  const double stop_hold_sec_;   // その猶予[s]。過ぎたら膠着対策の下限を戻す
  const double stop_hold_move_;  // 自車がこれ[m/s]以上で動いているときだけ待つ
  double stop_hold_since_{-1.0}; // 停止車の直後で待ち始めた時刻[s]
  const double slow_leader_speed_;   // 前車がこれ未満[m/s]ならゾーン外でも抜く
  const double attempt_timeout_;     // 追越試行をこの時間[s]で打ち切る
  const double attempt_stall_time_;  // 試行開始からこの時間[s]で進展を見る
  const double attempt_stall_gain_;  // その間に進行度差がこの量[m]増えなければ降りる
  const double attempt_stall_cool_;  // 進展なしで降りた相手へ再挑戦しない時間[s]
  const double pass_len_;            // 抜き切るのに必要な相対距離[m]
  const double pass_time_limit_;     // この時間[s]以内に抜けるなら実行する。
                                     // 実測: 2位で前車20km/h・車間8m のとき 5.3秒、
                                     // 前車24km/h なら 6.5秒かかる。4秒では全て却下される。
  const double boost_gain_;          // ブーストで得られる接近速度の上乗せ[m/s]
  const double boost_retry_sec_;     // 1個目が効かなかったと判断するまでの時間[s]
  const double boost_min_speed_;     // これ未満[m/s]では撃たない(密集した発進直後を避ける)
  const double boost_accel_;         // ブーストの加速度上乗せ[m/s^2] (実装値 0.5)
  const double boost_duration_;      // ブーストの継続時間[s] (実装値 10.0)
  const double boost_min_headroom_;  // 加速余地[m/s]がこれ未満なら撃たない
  const double vehicle_accel_;       // 実効加速度[m/s^2]。AWSIM のクランプは1.0だが
                                     // 抵抗があるので実測 0.9 程度を使う
  const double zone_exit_margin_;    // ゾーン出口を越えても許す距離[m]。
                                     // 追い越しには 40〜50m 要るがゾーンは 14〜24m しかない。
                                     // ゾーン内で並びかけ、出口を越えて完了する形を許す。
  const double leader_speed_cap_;    // 1位の速度制限[km/h] (実装値 25.0)
  // 順位別のコーナー速度係数。sweep.sh の順位別計測から決める
  const double rank1_corner_gain_;   // 1位(25km/h制限あり)のとき
  const double rank2_corner_gain_;   // 2位以下(制限なし)のとき
  const double rank2_speed_cap_;     // 2位以下の上限[km/h] (driveFadeSpeed 36.0)
  const bool rank_shape_enable_;
  const double near_radius_;
  const double min_lat_sep_;
  const double min_pass_width_;
  const double min_pass_sep_;   // 並走時に必要な横間隔[m](カート幅 1.45)
  const double min_closing_kmh_;  // これ未満[km/h]の速度差では仕掛けない(同速対策)
  const double pass_dist_max_;    // 抜き切るのにこれ以上[m]要るなら仕掛けない
  const double stopped_speed_;       // これ以下なら「止まっている」[m/s]
  const double stopped_look_ahead_;  // 停止車両を探す前方距離[m]
  const double stop_margin_;         // 停止車両の手前に空ける距離[m]
  const double stop_hold_margin_;    // 追突判定で制動距離へ足す余裕[m]

  // 「このまま行くと前の停止車へ追突する」か。
  //
  // 【直したバグ(ユーザー報告: スタートがとても遅い / P2 が1位になれない)】
  // ここは `gap < stop_hold_gap_(3.0) && my_speed > stop_hold_move_(0.5)` という
  // **固定車間 + 自分が動いているか**で判定していた。
  // スタートではグリッド間隔が 2.0〜3.1m しかなく、青信号で発進した瞬間に
  // 「自分は動いている・前車はまだ 2.0m 先で停止中」が成立するため**必ず発動**し、
  // 速度上限が 10.8km/h から 0.0km/h へ落ちる。
  // 実測(d2): t=602.9 に 0.0km/h、そこから 8 秒間 0〜8km/h。
  // devnote 159 で足した `my_speed > stop_hold_move_` は、発進した瞬間に
  // 自分が動く以上まったく効いていなかった。
  //
  // 待つべきなのは「接近速度があって、その車間では止まりきれない」ときだけ。
  // スタートは前車も同時に加速するので接近速度がほぼ 0 になり、待たなくなる。
  // 走行中に止まっている車へ突っ込む場面は接近速度が大きいので従来どおり待つ。
  bool wouldRearEnd(double gap, double other_speed) const
  {
    const double closing = my_speed_for_gap_ - std::max(other_speed, 0.0);
    if (closing <= 0.0) { return false; }
    const double brake_a = std::max(std::abs(a_min_), 0.5);
    const double need = closing * closing / (2.0 * brake_a);
    return gap < need + stop_hold_margin_;
  }
  const double stopped_cluster_span_;  // 同じ場所で止まっているとみなす前後差[m]
  const double stopped_slack_;         // 余裕をもって通れるとみなす空き幅[m]
  const double stopped_thread_speed_;  // かろうじて通るときの速度[m/s]
  const double look_width_ahead_;
  const double v2x_timeout_;
  bool race_started_{false};       // /awsim/state が Start になったか
  const double safe_gap_;
  const double safe_gap_min_;      // 車間の下限[m]
  const double safe_gap_max_;      // 車間の上限[m]。開けすぎると追い越せず順位を落とす
  const double gap_brake_ratio_;   // 制動距離を車間にどれだけ反映するか
  const double a_min_;             // 想定減速度[m/s^2]
  const double follow_kp_;
  const double min_follow_speed_;
  const double side_hold_time_;
  const bool capped_self_enable_;   // 1位ハンデ中の closing 足切り緩和を使うか
  const double capped_self_closing_;// そのときに要る最低速度差[km/h]
  const double capped_self_dist_;   // そのときに許す抜き切り距離[m]
  const bool commit_pass_;          // 横に出切ったら追従キャップを外すか
  const double commit_sep_;         // 外すのに要る実測の横間隔[m]
  const double commit_gap_;         // 外すのに要る前後の車間[m](これ以内)
  const double commit_release_;     // 解除のヒステリシス(commit_sep への倍率)
  const bool attempt_hold_side_;    // 試行中は寄ると決めた側を保持しきるか
  const double attempt_hold_sep_;   // そのとき保持する横オフセットの最小値[m]
  const double attempt_latfail_time_; // 横に出られない試行を打ち切る秒数(0で無効)
  const double commit_boost_time_;  // 並走が続いたらブーストを撃つまでの秒数(0で無効)
  const bool boost_runup_enable_;   // 並ぶ前(助走段階)にブーストを撃つか
  const double boost_runup_gap_;    // 助走ブーストを撃つ車間の上限[m]
  const double boost_runup_gap_min_;// 同 下限[m]。近すぎると助走にならず追突する
  const double wall_margin_;        // 横目標を壁から必ず離す量[m]
  const double wall_margin_stopped_;  // 停止車を避けるときの壁の余裕[m]
  const double stop_avoid_crush_;     // 横目標がこれ[m]以上潰されたら通れない扱い
  const double crash_safe_sep_;     // 相手へ寄るときに許す横間隔[m]
  const double crash_front_near_;   // 相手が「自分の前」とみなす前後距離の下限[m]
  const double crash_front_far_;    // 同 上限[m]
  const double wall_brake_ratio_;   // 両方避けられないときの減速率
  const double tight_radius_;       // これ未満の曲率半径[m]で壁の余裕を増やす
  const double wall_margin_tight_;  // きついコーナーで足す余裕の最大[m]
  const bool enable_;

  Corridor corridor_;
  std::map<std::string, OtherState> others_;
  Trajectory::SharedPtr traj_;
  Odometry::SharedPtr odom_;
  double offset_{0.0};
  // 回避方向のヒステリシス用
  std::string side_blocker_;
  std::string cur_leader_;   // 現在の先頭車(自分が先頭なら空)
  double side_sign_{1.0};
  bool side_fits_{true};   // 選んだ側にコリドアの余地があるか
  double pass_sep_{0.0};   // 相手との横間隔の目標[m]
  double room_hi_{1e9};    // 先読み区間で出られる左の限界[m]
  double room_lo_{-1e9};   // 同じく右の限界[m]
  double side_decided_at_{0.0};
  int side_flip_cnt_{0};           // この対象車で側を変更した回数(side_flip_max まで)
  double side_flip_at_{0.0};       // 枠の回復を数える基準時刻[s]
  double side_unfit_since_{-1.0};  // 余地なしが始まった時刻[s]。負なら余地あり
  double deadlock_since_{-1.0};    // 停止車両の前で動けなくなった時刻[s]。負なら動いている
  // スタート時のグリッド横位置の保持
  int start_slot_{0};                  // スタート時の並び順(1=P1)
  std::vector<double> start_slot_lat_; // スタート時の各スロットの横位置[m]
  bool start_captured_{false};
  double start_lat_{0.0};
  double start_s_{0.0};
  bool start_merge_done_{false};
  double run_dist_{0.0};
  double last_x_{0.0};
  double last_y_{0.0};
  bool last_valid_{false};
  double my_lat_for_target_{0.0};
  double my_speed_for_gap_{0.0};
  double boost_gain_time_{0.0};   // ブーストで短縮できる追い越し時間[s]
  rclcpp::Time stall_since_{0, 0, RCL_ROS_TIME};
  bool stall_logged_{false};
  bool can_pass_now_{false};
  bool commit_now_{false};         // 横に出切って追従キャップを外している最中か
  bool wedge_active_{false};       // この周期で正面衝突回避(wedge)が働いたか
  rclcpp::Time last_wallpick_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_crush_log_{0, 0, RCL_ROS_TIME};
  double commit_since_{-1.0};      // 抜き切りモードに入った時刻[s]。負なら入っていない
  double attempt_max_sep_{0.0};    // この試行で実際に取れた横間隔の最大値[m]
  bool attempt_latok_{false};      // この試行で一度でも commit_sep を満たしたか
  // 順位推定
  int rank_{1};
  double my_prog_{0.0};
  // 追い越し試行の記録
  bool attempt_active_{false};
  double attempt_offset_{0.0};  // 試行中に保持する横オフセット
  double attempt_fail_since_{-1.0};  // ラインへ戻った状態が始まった時刻[s]。負なら戻っていない
  int attempt_lead_cnt_{0};          // 相手より前に出ている周期数(3周期で成功)
  double attempt_diff0_{1e18};       // 試行開始時の進行度差[m]。1e18 は未取得
  double attempt_stall_until_{-1.0}; // この時刻[s]まで下の相手へは仕掛けない
  std::string attempt_stall_name_;   // 進展なしで降りた相手
  // 追越試行が降りた理由を追うための直近の判定内容
  // 接触した瞬間に「回避層が何をしていたか」を振り返るための記録。
  // 接触の原因が「回避が働かなかった」のか「働いたが足りなかった」のかを
  // 区別できないと直しようがない。
  double dbg_avoid_offset_{0.0};   // 横に逃げた量[m]
  double dbg_avoid_cap_{-1.0};     // 減速の上限[m/s]。負なら減速していない
  double dbg_avoid_ttc_{-1.0};     // 最も近い相手との衝突までの時間[s]
  double dbg_avoid_sep_{-1.0};     // その相手との横間隔[m]
  double dbg_near_veh_{-1.0};      // 最も近い他車までの距離[m]
  bool dbg_allow_{false};
  bool dbg_zone_{false};
  bool dbg_zone_ok_{false};
  bool dbg_feasible_{false};
  bool dbg_latched_{false};
  double dbg_width_{0.0};
  double dbg_map_l_{0.0};   // 学習ラインで左に並走できる連続区間[m]
  double dbg_map_r_{0.0};   // 同 右
  int dbg_map_n_{0};        // 学習点の数
  std::string attempt_target_;
  double attempt_start_{0.0};
  int attempt_boosts_{0};
  int attempt_ok_{0};
  int attempt_ng_{0};
  double my_last_s_{0.0};
  bool my_prog_init_{false};
  int boost_remaining_{0};
  bool is_boosting_{false};
  bool want_boost_{false};
  bool free_boost_enable_{true};
  double free_boost_gap_{20.0};          // 2個目までこの秒数あける
  double free_boost_min_speed_{4.0};     // これ以下[m/s]では撃たない
  double free_boost_clear_ahead_{15.0};  // 前方この距離[m]に他車がいたら撃たない
  double free_boost_straight_{25.0};     // 先の曲率半径がこれ[m]以上なら直線とみなす
  double free_boost_headroom_{1.5};      // 加速余地[m/s]。これ未満なら撃っても無駄
  double free_boost_skip_{6.0};          // 直線判定でこの距離[m]先から見る(立ち上がりで撃つため)
  bool wedge_enable_{true};
  double wedge_ttc_{0.7};                // これ[s]を切ったら横へねじ込む
  double wedge_room_{0.25};              // コリドアの縁までこれ[m]まで詰めてよい
  double boost_gain_min_{1.5};           // 自力でも抜けるとき、これだけ[s]短縮するなら使う
  int boost_hold_laps_{3};               // この周回を終えるまでは温存する(抜き返され対策)
  bool straight_ahead_{false};           // これから直線に入るか
  double free_boost_defend_dist_{15.0};  // 後方この距離[m]に詰められたら防衛で使う
  int race_laps_{6};
  int free_boost_laps_left_{6};          // 周回数以上にすると最初から使える
  std::string boost_zone_spec_;
  // 相手の走行データを区間ごとに集計するために、経路の点を控えておく。
  std::vector<double> line_x_, line_y_;
  double my_speed_sum_{0.0};
  int my_speed_cnt_{0};
  rclcpp::Time last_stats_log_{0, 0, RCL_ROS_TIME};
  std::vector<std::pair<std::size_t, std::size_t>> boost_zones_;
  std::string no_pass_zone_spec_;
  std::string right_zone_spec_;
  std::string grid_slot_spec_;
  std::vector<std::pair<double, double>> grid_slots_;  // 記録したグリッド座標
  std::vector<std::pair<std::size_t, std::size_t>> no_pass_zones_;  // 追い越し禁止区間
  std::vector<std::pair<std::size_t, std::size_t>> right_zones_;   // 右から抜く区間
  double slow_rival_ratio_{0.85};
  bool start_boost_enable_{true};
  int start_boost_laps_{2};       // この周回数以内なら「序盤」とみなす
  double start_boost_dist_{40.0}; // スタートからこの距離[m]以内で撃つ
  double aggressive_time_gain_{1.5};   // 最下位のとき、所要時間の上限を伸ばす
  double aggressive_width_gain_{0.85}; // 最下位のとき、必要な幅を緩める
  bool start_boost_used_{false};
  bool start_boost_pending_{false};  // 今の周期の要求がスタートの1本か
  int start_rank_{0};             // スタート時の順位(0=未確定)
  bool approach_enable_{true};
  double approach_range_{80.0};    // 仕掛けどころをこの距離[m]先まで探す
  double approach_gap_max_{14.0};  // 逆算で開ける車間の上限[m]
  double charge_close_dist_{20.0};   // ゾーン入口までこの距離[m]を切ったら詰めきる
  double charge_close_gap_k_{1.2};   // 詰めきる先の車間 = pass_gap * この係数
  double charge_brake_margin_{1.5};  // 助走を打ち切る制動距離への上乗せ[m]
  bool predict_enable_{true};
  double predict_ahead_{12.0};    // 相手の何m先まで見るか
  double predict_floor_{0.55};    // 予測値の下限(今の速度に対する比)
  int lap_{0};
  std::size_t prev_ei_{0};
  bool prev_ei_set_{false};
  bool boost_armed_{false};
  int boost_used_{0};
  rclcpp::Time last_boost_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<Float32MultiArray>::SharedPtr boost_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr overtaking_pub_;
  rclcpp::Subscription<Float32MultiArray>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Time last_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reject_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_avoid_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wedge_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_push_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_contact_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stopped_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_deadlock_log_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<V2XVehiclePositionArray>::SharedPtr sub_v2x_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<V2XOvertaker>());
  rclcpp::shutdown();
  return 0;
}
