// V2X の他車位置を見て走行ラインを横にずらし、追い越し・追従を行うノード。
//
// 実装は src/v2x_overtaker.cpp。ここには**状態と操作の一覧**だけを置く。
// どんな値を持ち、どんな段で処理するのかをここだけで把握できるようにするため。
#ifndef V2X_OVERTAKER__V2X_OVERTAKER_HPP_
#define V2X_OVERTAKER__V2X_OVERTAKER_HPP_

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

class V2XOvertaker : public rclcpp::Node
{
public:
  V2XOvertaker();

private:

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

  // ---- 20Hz の本体と、そこから呼ばれる層 ----
  // それぞれの意図と、過去にどんな不具合を出したかは cpp の定義の直前にある。
  // 層の順序と優先順位は onTimer() の定義の直前にまとめてある。
  bool loadCorridor(const std::string & path);
  void onV2X(const V2XVehiclePositionArray::SharedPtr msg);
  static size_t nearest(const Trajectory & t, double x, double y);
  static void normalAt(const Trajectory & t, size_t i, double & nx, double & ny);
  void sideRoomMap(const OtherState & o, const Trajectory & in, size_t n,
                   size_t from, double stretch, double olat_now,
                   double & run_left, double & run_right, int & known) const;
  void onTimer();
  void decideAllow(const Frame & f, PlanCtx & c, const std::string & name,
                   const OtherState & o, size_t oi, double gap, double olat,
                   double ospeed_for_gate, OppEval & ev);
  void chooseSide(const Frame & f, PlanCtx & c, const OtherState & o,
                  size_t oi, double & olat, double & ospeed_for_gate);
  bool isNearestBlocker(const Frame & f, PlanCtx & c, const std::string & name,
                        const OtherState & o, size_t & oi, double & gap);
  void chargeBoost(const Frame & f, PlanCtx & c, const OppEval & ev);
  void followAndCommit(const Frame & f, PlanCtx & c,
                       const OtherState & o, const OppEval & ev);
  void evaluateOpponent(const Frame & f, PlanCtx & c,
                        const std::string & name, const OtherState & o);
  void logDrivingStats(const Frame & f);
  void estimateRank(const Frame & f);
  void evaluateZone(const Frame & f, PlanCtx & c);
  void checkPressedFromBehind(const Frame & f, PlanCtx & c);
  void logStopCause(const Frame & f);
  void findFrontCar(const Frame & f);
  void learnOpponentLine(const Frame & f);
  void planOvertake(const Frame & f, PlanCtx & c);
  void avoidStoppedCars(const Frame & f, PlanCtx & c);
  void avoidCollision(const Frame & f, PlanCtx & c);
  void repulseFromNearCars(const Frame & f, PlanCtx & c);
  void holdStartLane(const Frame & f, PlanCtx & c);
  void holdSideBySide(const Frame & f, PlanCtx & c);
  void recordAttempt(const Frame & f, PlanCtx & c);
  void applyAvoidance(const Frame & f, PlanCtx & c);
  void holdAttemptSide(PlanCtx & c);
  void avoidWall(const Frame & f, PlanCtx & c);
  void applyOffsetRateLimit(PlanCtx & c);
  void publishTrajectory(const Frame & f, PlanCtx & c);
  void manageBoost(const Frame & f, PlanCtx & c);
  void logBlocker(const Frame & f, PlanCtx & c);

  // ---- 状態 ----
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

#endif  // V2X_OVERTAKER__V2X_OVERTAKER_HPP_
