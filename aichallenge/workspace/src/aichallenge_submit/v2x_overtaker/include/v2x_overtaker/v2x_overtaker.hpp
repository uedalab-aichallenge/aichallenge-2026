// V2X の他車位置を見て走行ラインを横にずらし、追い越し・追従を行うノード。
//
// 実装は src/v2x_overtaker.cpp。ここには**状態と操作の一覧**だけを置く。
// どんな値を持ち、どんな段で処理するのかをここだけで把握できるようにするため。
#ifndef V2X_OVERTAKER__V2X_OVERTAKER_HPP_
#define V2X_OVERTAKER__V2X_OVERTAKER_HPP_

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <v2x_msgs/msg/v2_x_vehicle_position_array.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <deque>
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
constexpr double kBandAbs = 6.0;  // バンドの既定の広さ[m]
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

  // スタートグリッドの P 番号(1=最後尾 ... 3=最前列=運営NPC)。0=未確定。
  // 「1周目は NPC 以外を抜かない」「P1 のとき僚車を抜くのは3周目以降」を
  // 判断するために、レース開始時にグリッド座標と照合して1度だけ決める。
  int slot{0};

  static constexpr int kLatBins = 256;      // 走行ラインを256分割して横位置を集計
  double lat_sum[kLatBins]{};
  int    lat_cnt[kLatBins]{};
  // --- 相手の走り方の録画(ユーザー指示) ---
  // 横位置だけでなく**その地点で相手が出している速度**も地点ごとに覚える。
  // 2周目以降に「どこで抜くか」を決めるとき、
  //   ・相手が遅い場所ほど詰めやすい
  //   ・相手の横位置が片側に寄っている場所ほど反対側が空いている
  // の2つが要る。横位置(lat)だけでは前者が分からない。
  double spd_sum[kLatBins]{};
  int    spd_cnt[kLatBins]{};

  // その地点で相手が普段いる横位置[m]。データが足りなければ 1e9 を返す。
  double laneLat(int bin) const
  {
    if (bin < 0 || bin >= kLatBins || lat_cnt[bin] < 3) { return 1e9; }
    return lat_sum[bin] / lat_cnt[bin];
  }
  // 抜きどころ探索用の暫定値。V2X は地点ごとに1回程度しか届かないことが
  // あり、3サンプル必須だと6周レースの前半を丸ごと追従に費やしてしまう。
  // 単点の誤差は、探索側が15m以上の連続した空間を要求して除外する。
  double laneLatProvisional(int bin) const
  {
    if (bin < 0 || bin >= kLatBins || lat_cnt[bin] < 1) { return 1e9; }
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

  // その地点で相手が普段出している速度[m/s]。データが足りなければ負を返す。
  double laneSpd(int bin) const
  {
    if (bin < 0 || bin >= kLatBins || spd_cnt[bin] < 3) { return -1.0; }
    return spd_sum[bin] / spd_cnt[bin];
  }
  double laneSpdProvisional(int bin) const
  {
    if (bin < 0 || bin >= kLatBins || spd_cnt[bin] < 1) { return -1.0; }
    return spd_sum[bin] / spd_cnt[bin];
  }
  void noteSpd(int bin, double v)
  {
    if (bin < 0 || bin >= kLatBins) { return; }
    if (spd_cnt[bin] >= 40) {
      spd_sum[bin] = spd_sum[bin] * (39.0 / 40.0) + v;
      return;
    }
    spd_sum[bin] += v;
    spd_cnt[bin]++;
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
    // 追突までの余裕が尽き、通常の上限レート制限を待てない状態。
    // このときだけ publishTrajectory 側の減速平滑化を通さない。
    bool emergency_brake{false};

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
    bool stop_avoid_have_gap{false};   // 下の空き帯が有効か
    double stop_avoid_lo{0.0};         // 停止車の脇の空き帯(車幅を除いた後)の右限
    double stop_avoid_hi{0.0};         // 同 左限
    // 先頭の停止車までの距離[m]。-1 は未設定。
    // 「今いる場所の壁帯に横目標が入らない」ことを「停止車の脇を通れない」と
    // 解釈して減速してよいのは、寄せ切る時間がもう無いときだけ。その判定に使う。
    double stop_avoid_dist{-1.0};
    double avoid_offset{0.0};       // 衝突回避層が出した横オフセット
    double avoid_speed_cap{-1.0};   // 衝突回避層が出した速度上限
    double repulse{0.0};            // 近接車からの横方向の反発

    // publishTrajectory が作った最終軌道の、自車地点での目標速度[m/s]。
    // ブーストの判断が「速度上限とハンデを掛けたあとの目標」を必要とするため、
    // 軌道そのものではなくこの 1 点だけを次の層へ渡す。
    double ego_target_speed{0.0};

    // --- 速度上限の調停(2026-09-02 ユーザー指示で統一) ---
    // 各層は speed_cap を直接書き換えず、requestCap() で「上限[m/s]と理由」を
    // 積む。最後に applyCapRequests() が最小値を採り、縛った層の名前を残す。
    // 以前は9層が後勝ちで書き換え、うち2層が std::max で上げ直していたため、
    // 層の順序に依存した事故が起きていた(onTimer のコメント参照)。
    // --- 横位置の調停(2026-09-02 ユーザー指示で統一) ---
    // 【なぜ要るか】従来は11の層が target_offset を後勝ちで上書きしており、
    // 追い越しで横間隔 2.89m まで出た直後に別の層が 0.00m へ引き戻して失敗して
    // いた(実測 20260902-150954)。holdSideBySide / holdAttemptSide は
    // 「上書きされた自分の指令を上書きし返す」ための対症療法だった。
    // 意図は優先度で1つだけ選び、制約は区間として交差させ、最後にクランプする。
    enum class LatPrio : int {
      kBase        = 0,   // 基準ライン
      kGridLane    = 10,  // 発進時のグリッド保持
      kRepulse     = 20,  // 近接車からの反発
      kOvertake    = 30,  // 追い越しの目標横位置
      kStoppedCar  = 40,  // 停止車集団の回避
      kCollision   = 50,  // TTC 切迫の緊急回避
    };
    struct LatIntent { double v; LatPrio prio; const char * why; };
    struct LatBound  { double lo; double hi; const char * why; };

    LatIntent lat_intent{0.0, LatPrio::kBase, "基準"};
    double lat_lo{-1e9};
    double lat_hi{ 1e9};
    const char * lat_lo_why{"なし"};
    const char * lat_hi_why{"なし"};
    const char * lat_bound_why{"なし"};   // 最終的にクランプした側の理由

    // 意図を出す。優先度が今のものより高いときだけ置き換える。
    // 同じ優先度なら**後から出したほうが勝つ**(同一層の内部で値を更新できる)。
    void requestLat(double v, LatPrio prio, const char * why)
    {
      if (!std::isfinite(v)) { return; }
      if (static_cast<int>(prio) >= static_cast<int>(lat_intent.prio)) {
        lat_intent = {v, prio, why};
      }
    }
    // 制約を出す。区間を狭める方向にだけ効く。
    void boundLat(double lo, double hi, const char * why)
    {
      if (std::isfinite(lo) && lo > lat_lo) { lat_lo = lo; lat_lo_why = why; }
      if (std::isfinite(hi) && hi < lat_hi) { lat_hi = hi; lat_hi_why = why; }
    }
    // 途中の層が「いま決まりかけている横目標」を読むための値。
    // 【なぜ必要か】調停を入れると target_offset は最後まで確定しない。
    // 従来 c.target_offset を読んでいた層(追突防止の進路判定、壁回避の
    // want、試行の保持値)がそのまま読むと常に 0 になり、判定が壊れる。
    // 採用中の意図を、そこまでに積まれた制約で丸めた値を返す。
    double latWant() const
    {
      double lo = lat_lo, hi = lat_hi;
      if (lo > hi) { const double m = 0.5 * (lo + hi); lo = hi = m; }
      return std::clamp(lat_intent.v, lo, hi);
    }
    // 採用した意図を制約へクランプして target_offset を確定する。
    void applyLatDecision()
    {
      double lo = lat_lo, hi = lat_hi;
      if (lo > hi) { const double m = 0.5 * (lo + hi); lo = hi = m; }
      const double want = lat_intent.v;
      target_offset = std::clamp(want, lo, hi);
      lat_bound_why = (target_offset > want + 1e-6) ? lat_lo_why
                    : (target_offset < want - 1e-6) ? lat_hi_why
                    : "なし";
    }

    struct CapReq { double v; const char * why; };
    std::vector<CapReq> cap_reqs;
    const char * cap_why{"なし"};

    // 上限を要求する。負値・非有限は無視する。
    void requestCap(double v, const char * why)
    {
      if (!std::isfinite(v) || v < 0.0) { return; }
      cap_reqs.push_back({v, why});
    }
    // 積まれた要求の最小値を speed_cap へ確定し、cap_why に理由を入れる。
    void applyCapRequests()
    {
      double best = -1.0;
      const char * bw = "なし";
      for (const auto & r : cap_reqs) {
        if (best < 0.0 || r.v < best) { best = r.v; bw = r.why; }
      }
      speed_cap = best;
      cap_why = bw;
    }
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
    bool lap_traffic{false};      // 一度抜いた運営NPC（周回遅れ）
    bool timed_plan{false};       // 予測区間出口までの前後運動を計算済み
    double accel_delay{0.0};      // 全開加速を開始できる最も遅い時刻[s]
    bool timed_boost{false};      // この成立時刻には新規ブーストが必要
    bool allow{false};     // 抜きにいってよいか
  // 速度上限の解除(commit)を許すか。allow との違いは latch を含めない点。
  // latch は「横に出続けてよい」だけを許し、上限解除までは救わない。
  bool allow_commit{false};
  };

  // ---- 20Hz の本体と、そこから呼ばれる層 ----
  // それぞれの意図と、過去にどんな不具合を出したかは cpp の定義の直前にある。
  // 層の順序と優先順位は onTimer() の定義の直前にまとめてある。
  bool loadCorridor(const std::string & path);
  void buildBand(const Frame & f);
  void publishBand(const Frame & f);
  void publishStatus(const Frame & f, const PlanCtx & c);

  // 予測バンドが決めた「相手のどちら側を通るか」。+1=左 / -1=右 / 0=未定
  int bandSide(const std::string & name) const
  {
    const auto it = band_side_.find(name);
    return (it == band_side_.end()) ? 0 : it->second;
  }
  void onV2X(const V2XVehiclePositionArray::SharedPtr msg);
  static size_t nearest(const Trajectory & t, double x, double y);
  // 曲率で決まるコリドアの余裕[m](カーブ=corridor_safety / 直線=pass_margin_min)。
  double safetyAt(size_t i) const;
  static void normalAt(const Trajectory & t, size_t i, double & nx, double & ny);
  void sideRoomMap(const OtherState & o, const Trajectory & in, size_t n,
                   size_t from, double stretch, double olat_now,
                   double & run_left, double & run_right, int & known,
                   double * room_left = nullptr,
                   double * room_right = nullptr) const;
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
  void assignStartSlots(const Frame & f);
  void planPassSpot(const Frame & f);
  bool inSidePickZone(std::size_t idx) const;
  bool straightPassNow(const Frame & f);
  bool sideStaysOpen(const Frame & f, double off, double ahead) const;
  double spotGateDistance(const Frame & f, const OtherState & o) const;
  bool spotPathSafe(const Frame & f, const OtherState & o, double ahead) const;
  double latestPassAccelDelay(const Frame & f, const std::string & name,
                              const OtherState & o, double gap,
                              double exit_distance, bool use_boost,
                              double pass_start_dist) const;
  double distToSidePickZoneEnd(const Frame & f) const;
  double zoneMeanLat(const OtherState & o, std::size_t n, int & known) const;
  void dumpTrace(const Frame & f);
  bool passAllowedThisLap(const std::string & name, const OtherState & o,
                          bool clearly_slower) const;
  // 指定した相手に対して、いま追い越しが進行中で、かつ横方向にその相手の
  // 進路から十分外れているか。
  //
  // 【なぜ作ったか】追従(followAndCommit)と追突防止(preventRearEnd)が
  // 「いま追い越し中か」を別々の条件で判定していた。追従の commit_now は
  // attempt_active_ を見ず、追突防止の緩和は attempt_active_ 必須。速度上限は
  // 両者の min を採るため、片方だけが緩んでももう片方が押さえ込み、実測では
  // 全周期の96%で 8〜12km/h に張り付いていた。判定を1本にして揃える。
  bool passUnderway(const std::string & name, double lat_sep) const;

  // ================= 追い越しの状態機械 (2026-09-02) =================
  // 【なぜ作るか】従来は attempt_active_ という真偽値ひとつで「追い越し中か」を
  // 表し、速度制御・横位置・中断判定がそれぞれ別の条件で「今追い越しているか」を
  // 判断していた。そのため追従用の速度制限が追い越し中も効き続け(実測で全周期の
  // 大半が 12.8km/h に制限)、横へ出るには速度が要るのに横へ出るまで速度が出ない
  // という循環が起きていた。また「抜こうとしている相手が遅い」ことを理由に
  // 停止車回避が試行を中断していた(実測で中断理由の最多)。
  // 状態を1つ持ち、状態ごとに「誰が速度を決めてよいか」「何を理由に中止するか」を
  // 明示する。
  //
  //   FOLLOW   : 追わない/追従する。通常の追従速度制限が効く
  //   PREPARE  : 計画した抜きどころへ接近中。追従制限は効くが目標車間を詰める
  //   MOVE_OUT : 横へ出ている最中。追従制限は外れ、衝突安全だけが残る
  //   PASS     : 並走〜追い抜き中。同上
  //   MERGE    : 抜き切ってラインへ戻る最中。同上
  //   COOLDOWN : 中止直後の休止。次の計画を作るまで仕掛けない
  enum class OvState { kFollow, kPrepare, kMoveOut, kPass, kMerge, kCooldown };
  OvState ov_state_{OvState::kFollow};
  std::string ov_target_;        // 状態機械が対象としている相手
  double ov_state_since_{-1.0};  // 現在の状態に入った時刻[s]
  double ov_cooldown_until_{-1.0};
  // 「横の余地が物理的に消えた」を数えるための計時。帯幅 < band_car_w_ が
  // spot_abort_sec_ 以上続いたときだけ中断する。負なら余地はある。
  double ov_narrow_since_{-1.0};
  // kMerge から kFollow へ戻す2条件の計時。負なら成立していない。
  double ov_merge_back_since_{-1.0};   // |pass_sep_| がラインへ戻った時刻[s]
  double ov_attempt_off_since_{-1.0};  // attempt_active_ が偽になった時刻[s]
  // avoidWall の「壁に押し出される」判定(pushed_to_wall)をそのまま控えた値。
  // 状態機械は avoidWall より前に走るので、前周期(50ms前)の値を読む。
  // avoidWall のロジック自体は変えていない。記録するだけ。
  bool wall_push_now_{false};
  static const char * ovStateName(OvState s);
  const char * ovStateName() const { return ovStateName(ov_state_); }
  // 追い越しの実行中(追従制限を外してよい状態)か
  // 【追加 2026-09-03】接近中(PREPARE)にその相手を対象にしているか。
  // 抜く場所へ「速度を持って」到達するための助走に使う。
  bool ovPreparingTarget(const std::string & name) const {
    return ov_state_ == OvState::kPrepare && !ov_target_.empty() &&
           ov_target_ == name;
  }
  bool ovPassing() const {
    return ov_state_ == OvState::kMoveOut || ov_state_ == OvState::kPass ||
           ov_state_ == OvState::kMerge;
  }
  // 指定した相手に対して追い越し実行中か
  bool ovPassingTarget(const std::string & name) const {
    return ovPassing() && !ov_target_.empty() && ov_target_ == name;
  }
  // 状態遷移。planOvertake の後、実行層の前で毎周期呼ぶ。
  void updateOvertakeState(const Frame & f, PlanCtx & c);
  // ブーストを解禁してよい周回か(ユーザー指示: 3周目以降)。
  bool boostLapOk() const { return !lap_gate_enable_ || lap_ >= boost_min_lap_; }
  void planOvertake(const Frame & f, PlanCtx & c);
  void preventRearEnd(const Frame & f, PlanCtx & c);
  void avoidStoppedCars(const Frame & f, PlanCtx & c);
  void avoidCollision(const Frame & f, PlanCtx & c);
  void repulseFromNearCars(const Frame & f, PlanCtx & c);
  void holdStartLane(const Frame & f, PlanCtx & c);
  // 発進フェーズだけ、横目標を「自分のグリッドの横位置」で最終決定する層。
  void holdGridLane(const Frame & f, PlanCtx & c);
  // 発進の一発追い越し(第3段)。P1・1周目・発進直後だけ MPC を抜きにいく。
  bool launchPassActive() const;
  const OtherState * launchNpc() const;
  void updateLaunchPass(const Frame & f);
  double gridLat(const Frame & f, int slot) const;
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
  const double pass_side_clear_;  // 追い越し中とみなす最小の横間隔[m]
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
  // 【追加 2026-09-02】追い越しを「完了した」と認めるための相対距離と継続時間。
  // 旧実装は diff>0 が3周期(0.15秒)続けば成功としており、並走中に差がゼロ
  // 付近で振動するだけで成立していた。成功と判定すると attempt_active_ を
  // 落とすため、横に並んだ瞬間に試行を終えてラインへ戻り、また後ろへ落ちる。
  const double pass_done_len_;       // 完全に前へ出たと認める相対距離[m]
  // 【追加 2026-09-02】「今ここから抜き切れる」と予測が示したとき、そこも
  // 抜きどころとして認めるか。計画した1点でしか仕掛けられない設計では、
  // 相手が8km/h・自車36km/hでも「より良い場所」を待って結局抜かなかった。
  const bool spot_here_enable_;      // 即時の抜きどころを認めるか
  const double spot_here_range_;     // 即時判定を行う車間の上限[m]
  const double pass_done_sec_;       // その状態を維持すべき時間[s]
  const double pass_done_min_sec_;   // 追い越し1件の所要時間の下限[s]
  // ================= ペナルティの推定 (2026-09-03) =================
  // AWSIM のペナルティは「その秒数だけ最高速度を 5km/h に固定」する形で効き、
  // 通知トピックは無い(単独走行の評価 JSON にしか出ない)。したがって
  // 速度が 5km/h に張り付いていることからしか推定できない。
  // 誤検出を減らすため、許容幅の内側に一定時間入り続けた場合だけ「ペナルティ中」
  // とみなす。通常走行で 5.0km/h ちょうどを維持し続けることは稀。
  const double pen_speed_;           // ペナルティ時の固定速度[m/s] (5km/h)
  const double pen_tol_;             // 許容幅[m/s]
  const double pen_hold_;            // この秒数continuousで確定[s]
  // ================= 壁衝突の予測監視 (2026-09-03) =================
  // 「今の速度・今の横目標のまま走ると、この先で車体が走行可能領域から
  // はみ出す」ことを先に見つけて、横位置の制約と速度上限として出す層。
  // 意図(requestLat)は一切出さない。範囲を狭める / 上限を下げる方向にしか
  // 働かないので、他層と喧嘩したり発振したりしない。
  //
  // 【寸法について】公式 vehicle_info.param.yaml は wheel_base 1.087 /
  // 車幅 1.30m だが、このプロジェクトの実測はホイールベース 2.14m /
  // 実幅 1.46m(半幅 0.73)で、比 1.97 を steering_tire_angle_gain が
  // 吸収している。予測は実測値を既定にするが、どちらが正しいか確証は
  // 無いのでパラメータで振れるようにしてある。
  const bool wall_guard_enable_;         // 有効化
  const double wall_guard_horizon_;      // 予測する秒数[s]
  const double wall_guard_dt_;           // 刻み[s]
  const double veh_half_width_;          // 車体半幅[m](実測 1.46m の半分)
  const double veh_wheel_base_;          // ホイールベース[m](実測値)
  const double veh_front_overhang_;      // 前オーバーハング[m]
  const double veh_rear_overhang_;       // 後オーバーハング[m]
  const double veh_max_steer_;           // 最大舵角[rad]
  const double wall_guard_margin_;       // 走行可能領域の内側に残す余裕[m]
  const double wall_guard_run_;          // この長さ[m]以上連続で違反したら作動
  const double wall_guard_ay_max_;       // 速度制限に使う横加速度上限[m/s^2]
  // corridor_ten.csv の lo/hi は make_corridor.py が
  // 「境界 - (半幅 + 余裕)」で作っており、**すでに半幅が控除済み**。
  // (コード内の実測メモ: 物理的な壁 -2.85 / csv の lo -1.67
  //  = 差 1.18 = 半幅 0.73 + 余裕 0.45)
  // よって予測でもう一度 veh_half_width_ を丸ごと引くと二重計上になり、
  // 常時作動して極端に遅くなる。控除済みの半幅をここで宣言し、
  // 車体の張り出しのうち**超過したぶんだけ**を足す。
  const double wall_guard_corridor_half_;

  // ================= 占有格子による舵角ガード (2026-09-03) =================
  // 【なぜ要るか】上の CSV(corridor_ten.csv)ベースの判定は、車体の内輪差や
  // 前端の振り出しを **近似式で横位置に足す** 間接的なものだった。占有格子を
  // 直接引けば、車体の四隅の座標をそのまま地図に当てられるので近似が要らない。
  //
  // 【出力は舵角の範囲だけ】(2026-09-03 ユーザー訂正)
  // この層は /control/wall_guard/steer_limit へ流す舵角の許容範囲しか作らない。
  // requestCap も boundLat も呼ばない。縦方向(減速)は preventRearEnd と TTC の
  // 担当で、追突の正しい対処は減速であって転舵ではないため。
  //
  // 【他車を除外条件にしない】前方に車がいるだけで「直進が禁止」になると、
  // 強制的に横へ切る挙動になってしまう。他車は予測位置として評価し、
  // ログには残すが候補からは落とさない。
  //
  // 【逃げ場が無いとき】(2026-09-03 ユーザー訂正)
  // 余裕 >= 0 の候補が1つも無いときはフェイルオープンせず、
  // 「最も食い込みが浅い舵角」= 幾何的に最も壁と平行に近い舵角へ固定する。
  // 正面から突っ込むより接触が浅くなる(= 壁に沿う)。
  const bool occ_enable_;            // 占有格子ガードの有効化
  const std::string occ_map_yaml_;   // 占有格子地図の yaml(絶対パス)
  const double occ_sample_step_;     // 車体の辺を刻む間隔[m]
  const int occ_steer_bins_;         // 候補舵角の本数
  const double occ_clear_search_;    // 余裕を測る上限[m](これ以上は飽和させる)

  // 読み込んだ占有格子と、その符号付き距離場。
  //   sd > 0 : 自由セル。最寄りの占有セルまでの距離[m]
  //   sd < 0 : 占有セル。最寄りの自由セルまでの距離[m](食い込みの深さ)
  // 距離場は起動時に一度だけ作る(Felzenszwalb の厳密 EDT)。
  struct OccGrid
  {
    bool ok{false};
    int w{0}, h{0};
    double res{0.1};
    double ox{0.0}, oy{0.0};
    std::vector<float> sd;    // 行は画像の上から。要素数 = w*h
    std::size_t n_occ{0};     // 占有セル数(検証ログ用)
  };
  OccGrid occ_;
  bool occ_warned_{false};
  rclcpp::Time last_occ_log_{0, 0, RCL_ROS_TIME};

  bool loadOccGrid();
  // (x, y) の符号付き余裕[m]。地図外は「コース外」なので最大の食い込み扱い。
  double occSignedDist(double x, double y) const;
  // 後軸中心 (cx, cy)・向き yaw の車体矩形の、壁までの最小余裕[m]。
  // 正 = 接触しない / 負 = 食い込み量。t_ahead は静的地図では使わないが、
  // 呼び出し側の意味を揃えるために署名に残してある。
  double bodyClearance(double cx, double cy, double yaw, double t_ahead) const;
  bool bodyHits(double cx, double cy, double yaw, double t_ahead) const;
  // t_ahead 秒後の他車(等速直線予測)と車体矩形が重なるか。分離軸判定。
  bool otherHits(double cx, double cy, double yaw, double t_ahead) const;

  struct OccSteerResult
  {
    bool used{false};       // 占有格子で評価できたか
    int bins{0};            // 候補の本数
    int surv{0};            // 壁に当たらなかった候補の数
    double lo{0.0}, hi{0.0};// 許容する舵角の範囲[rad]
    bool blocked{false};    // 全候補が他車と衝突(= 前方閉塞)
    bool forced{false};     // 逃げ場が無く1本へ固定した
    double forced_depth{0.0};  // そのときの食い込み[m](負値)
    double best_clear{0.0};    // 生存候補の中での最良の余裕[m]
  };
  // 占有格子で舵角の許容範囲を求める。副作用は無い(PlanCtx を触らない)。
  OccSteerResult occSteerGuard(const Frame & f) const;

  void wallGuard(const Frame & f, PlanCtx & c);
  // 【舵角の許容範囲を制御側へ渡す】
  // wallGuard は横位置の制約(boundLat)と速度上限(requestCap)しか出さないので、
  // 「経路がどうであれ制御が壁へ向かう」場合を止められない。そこで、現在地点で
  // 壁に当たらない舵角の範囲を毎周期 /control/wall_guard/steer_limit へ流し、
  // simple_pure_pursuit 側で最後にクランプさせる。ここは publish するだけで、
  // wallGuard の既存の出力(boundLat / requestCap)には一切触れない。
  void publishSteerLimit(double lo, double hi, bool viol);
  // ================= 記録を rosbag へも載せる (2026-09-03 ユーザー指示) =================
  // 【なぜ必要か】車両状態・制御指令・他車位置は既に rosbag に入っているが、
  // 「なぜその判断をしたか」(却下の決め手・速度上限を決めた層・横位置の意図と制約・
  // 状態遷移・占有格子の判定・接触の局面)はどのトピックにも流れておらず、
  // bag からは復元できない。テキストログにしか無いため、後処理が grep 頼みになり、
  // レート制限のせいで発生回数を誤読する事故も起きた(「最終回避35回」は
  // 2秒制限のログ行数で、実際は数百回だった)。
  //
  // 【同期を構造で保証する】diagLog / diagWarn は「1回のフォーマットで
  // テキストログと診断トピックの両方へ出す」。記録を追加するときはこの関数を
  // 使う限り、両方へ自動的に載る。片方だけ追加されることが起こらない。
  //
  // 形式は JSON Lines。1 メッセージ = 1 行の JSON で、後処理が容易。
  //   {"t":123.456,"node":"v2x_overtaker","type":"追越記録","msg":"..."}
  void diagEmit(const char * type, bool warn, const char * text);
  void diagLog(const char * type, const char * fmt, ...)
    __attribute__((format(printf, 3, 4)));
  void diagWarn(const char * type, const char * fmt, ...)
    __attribute__((format(printf, 3, 4)));
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr diag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr steer_limit_pub_;
  // 制御側が実際にクランプしたかどうか(/control/wall_guard/override)。
  // ログへ併記するだけで、走りには影響しない。
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_steer_override_;
  bool steer_override_active_{false};
  // 作動していないときも残す「予測した最小余裕[m]」。壁接触の推定ログへ
  // 併記できるようにするための観測値(走りには影響しない)。
  double wall_guard_min_room_{1e9};
  rclcpp::Time last_wall_guard_log_{0, 0, RCL_ROS_TIME};
  struct PenState { double since{-1.0}; bool on{false}; double began{-1.0}; };
  std::map<std::string, PenState> pen_;   // 相手ごと
  PenState pen_self_;
  void updatePenalty(const Frame & f);
  bool isPenalized(const std::string & n) const {
    const auto it = pen_.find(n);
    return it != pen_.end() && it->second.on;
  }
  bool selfPenalized() const { return pen_self_.on; }
  // 試行中に観測した事実(成功/失敗の記録に残す)
  double attempt_gap0_{-1.0};        // 開始時の車間[m]
  bool attempt_tgt_pen_{false};      // 試行中に相手がペナルティ中だった
  bool attempt_self_pen_{false};     // 試行中に自車がペナルティ中だった
  std::size_t attempt_idx0_{0};      // 開始地点のインデックス
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
  // --- 最終区間の決めうち(ブーストの温存とタイミング) ---
  // 【一次情報】parallel.sh は --boosts 2 --laps 6。ブーストは有限資源。
  // 評価は docs/interface/evaluation-interface.md より final_position のみ。
  // 【AWSIM実測】rank==1 のとき駆動の頭打ちが 25km/h、2位以下は 36km/h。
  // 【実測】1位でいた車のベストラップ中央値 47.2s / 2位 35.7s。
  // 周長 334.5m なので先頭は 1周あたり 11.5秒(81m)を失う。
  // よってレース中の順位に価値は無く、最後に先にラインを切ることだけに価値がある。
  const bool   final_dash_enable_;   // マスタ退避スイッチ
  const int    boost_reserve_final_; // 最終区間用に残す個数
  const double final_dash_dist_;     // フィニッシュまでの残り[m]。以下を最終区間とする
  const double final_dash_gap_;      // 最終区間で前車がこの距離以内なら撃つ
  const bool   leader_boost_block_;  // 1位で加速余地が無ければ撃たない
  const double leader_boost_headroom_;
  const bool   final_dash_boost_when_leading_;  // 検証用。1位でも最終区間で撃つ
  const double near_radius_;
  const double min_lat_sep_;
  // true にすると反発力を追い越し許可(can_pass_now_)で門番する旧挙動に戻る。
  // A/B 計測用。既定は false(帯で幅を判定する新挙動)。
  const bool repulse_need_allow_;
  // 反発で逃げるとき、帯の端に残す余白[m]。0 だと端に張り付いて壁接触が増えた。
  const double repulse_band_margin_;
  // ブーストを撃ってよい最大の車間[m]。これより後ろで撃つと追突する。
  const double boost_side_gap_;
  const double min_pass_width_;
  const bool rear_end_lat_release_;   // 横にずれた分だけ追突防止を緩めるか
  const double rear_end_free_min_;    // 車体が触れない横間隔[m]。これ未満は緩めない
  const double rear_end_free_full_;   // ここまで離れたら完全に開放[m]
  const double rear_end_free_speed_;  // 完全に外れたときに許す速度[m/s]
  rclcpp::Time last_release_log_{0, 0, RCL_ROS_TIME};
  const double min_pass_sep_;   // 並走時に必要な横間隔[m](カート幅 1.45)
  const double min_closing_kmh_;  // これ未満[km/h]の速度差では仕掛けない(同速対策)
  const double pass_dist_max_;    // 抜き切るのにこれ以上[m]要るなら仕掛けない
  const double stopped_speed_;       // これ以下なら「止まっている」[m/s]
  const double stopped_look_ahead_;  // 停止車両を探す前方距離[m]
  const double stop_margin_;         // 停止車両の手前に空ける距離[m]
  // 停止車回避の制動距離に掛ける係数。実測減速度は中央値0.48 m/s^2 しかなく、
  // a_min(2.5) をそのまま使うと制動距離を約5倍過小評価する。
  const double stop_brake_k_;
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
  const double stopped_clear_speed_;   // 十分な空き幅があるときに許す速度[m/s]
  const double stopped_clear_gain_;    // 空き幅1mあたりの上限引き上げ[m/s]
  const double stopped_clear_in_;      // 帯の端からこれだけ内側に入っていること[m]

  // ---- 全域バンド(壁 + 他車の予測位置)----
  // 【なぜ作ったか(ユーザー指示)】
  // これまでは「今この瞬間どこへ寄るか」を表す **スカラー1個** (offset_) を
  // 12 層が上書きし合い、それを自車前後 30m の窓だけに掛けていた
  // (window_full=15 / window_end=30)。先読みが原理的に不可能で、
  // rviz でも自車の近くしか経路が変わらない。
  // ここでは走行可能な横方向の範囲を **周回全域** で持ち、
  //  (a) rviz に出してデバッグできるようにし、
  //  (b) 軌道オフセットを全域でそこへ収める。
  const bool   band_enable_;      // バンドを計算して rviz に出す
  const bool   band_clamp_;       // 軌道オフセットをバンドへ収める
  const bool   band_predict_;     // 他車を「自分が着く時刻」まで進めて塞ぐ
  const double band_horizon_;     // 先読みする距離[m]
  const double band_long_;        // 縦にこれだけ近ければ塞ぐ[m]
  const double band_car_w_;       // 相手の中心から塞ぐ横幅[m]
  const double band_v_floor_;     // 到達時刻を出すときの自車速度の下限[m/s]
  const double band_slope_;       // 横オフセットの傾きの上限[m/m]
  const double band_smooth_;      // バンドの時間方向の平滑化係数(0-1)
  const double band_side_hyst_;   // 通す側を入れ替えるのに必要な差[m]
  // 予測した他車の横位置がラインへ収束する距離定数[m]。
  // MPC は参照ラインを追うので、いま外れていても走るうちに戻る。
  const double band_lat_tau_;
  // 相手の速度がこれ[m/s]未満なら、予測経路の中で相手を進めない。
  // 止まっている車を「参照ラインの20%で走り続ける」と予測していた不具合の対策。
  const double band_stop_speed_;
  // バンドの「どちら側を開けるか」を、追い越しの側(side_sign_)に合わせるか。
  // 既定 true。false で従来の独立判断に戻せる(A/B 用)。
  const bool band_side_follow_;
  const double band_side_hold_;         // バンドの側を変えるのに要る保持時間[s]
  const double side_fix_cool_;          // 側を直したあと再反転しない時間[s]
  const double path_rate_;              // 経路の横移動の上限[m/s](バンド適用後)
  rclcpp::Time last_offs_time_{0, 0, RCL_ROS_TIME};
  double side_fix_since_{-1.0};         // 修正条件が続いている開始時刻[s]
  double side_fix_at_{-1e9};            // 最後に修正した時刻[s]
  const bool band_side_pred_;           // 側を相手の予測経路に沿って決めるか
  const bool pass_window_enable_;       // 追い越し窓の探索を使うか
  const double pass_window_gap_;        // 窓とみなす壁と相手の最小隙間[m]
  const bool pass_center_;              // 追越中は相手の端と壁の中点を狙う
  const double pass_window_sec_;        // 窓とみなす最小の継続時間[s]
  // --- 追い越し窓の探索結果(buildBand が毎周期作る)
  bool win_found_{false};
  int win_side_{0};        // +1 左 / -1 右
  double win_t_{0.0};      // 窓の入口へ着くまでの時間[s]
  double win_d_{0.0};      // 窓の入口までの距離[m]
  double win_gap_{0.0};    // 窓の最小隙間[m]
  double win_dur_{0.0};    // 窓の継続時間[s]
  double win_gap0_{0.0};   // 探索開始時の自車から相手までの弧長[m]
  int win_side_run_{0};    // 探索中の連続区間の側
  double win_run_start_t_{0.0}, win_run_end_t_{0.0}, win_run_start_d_{0.0};
  double win_run_min_gap_{0.0};
  rclcpp::Time last_window_log_{0, 0, RCL_ROS_TIME};
  struct PassWindow { bool found{false}; int side{0}; double t{0}, d{0}, gap{0}, dur{0}; };
  std::map<std::string, PassWindow> win_map_;   // 相手ごとの窓(前周期の結果)
  std::string c_blocker_;                       // 直近周期の前をふさぐ相手
                                        // (先読み距離は既存の band_side_look_ を使う)
  const double pass_margin_min_;        // 追い越し中・直線で詰める壁の余裕[m]
  const double pass_margin_r_curve_;    // これ以下の半径は従来どおり[m]
  const double pass_margin_r_straight_; // これ以上の半径は詰めきる[m]
  bool c_stop_avoid_active_{false};     // 直近の周期で停止車回避が働いていたか
  // 予測の当たり具合を測る先読み時間[s]。0 で無効。
  const double predict_check_sec_;
  // 相手ごとに仮定する MPC の最高速度[km/h]。実測が溜まるまでの初期値に使う。
  const double predict_speed_fast_;   // P1/P2(相手チームのコード)
  const double predict_speed_slow_;   // 運営NPC のスロット
  const int predict_prior_samples_;   // これだけ標本が溜まったら実測へ移る
  const double band_long_grow_;       // 塞ぐ長さの伸び[m/s](予測誤差ぶん)
  const double band_lat_grow_;        // 同 横幅の伸び[m/s]
  const bool   band_side_lead_;   // 抜く側の判断を予測バンドに任せる
  const double band_side_look_;   // 側を決めるとき先まで見る距離[m]


  // 発進直後、停止している前車への追従キャップを外す秒数
  const double launch_free_sec_;
  const double launch_free_gap_;  // ただしこの車間より近ければ外さない[m]
  const double launch_free_decel_;  // 判定に使う実測の減速度[m/s^2]
  const double launch_free_react_;  // 指令が効くまでの空走時間[s]
  const double launch_free_room_;   // 止まりきったあとに残す車間[m]
  const double look_width_ahead_;
  const double v2x_timeout_;
  bool race_started_{false};       // /awsim/state が Start になったか
  const double safe_gap_;
  const double safe_gap_min_;      // 車間の下限[m]
  const double safe_gap_max_;      // 車間の上限[m]。開けすぎると追い越せず順位を落とす
  const double gap_brake_ratio_;   // 制動距離を車間にどれだけ反映するか
  const double a_min_;             // 想定減速度[m/s^2]
  const double follow_kp_;
  const double follow_keep_gap_;  // これ以上の車間なら相手より遅くしない[m]
  const double min_follow_speed_;
  const double side_hold_time_;
  const bool capped_self_enable_;   // 1位ハンデ中の closing 足切り緩和を使うか
  const double capped_self_closing_;// そのときに要る最低速度差[km/h]
  const double capped_self_dist_;   // そのときに許す抜き切り距離[m]
  const bool commit_pass_;          // 横に出切ったら追従キャップを外すか
  // true で従来挙動(latch が速度上限の解除まで救う)に戻す退避スイッチ。
  const bool latch_commit_;
  const double commit_sep_;         // 外すのに要る実測の横間隔[m]
  const double commit_gap_;         // 外すのに要る前後の車間[m](これ以内)
  const double commit_release_;     // 解除のヒステリシス(commit_sep への倍率)
  const double commit_look_;        // 抜き切り前に横位置を保てるか見る距離[m]
  const double commit_crush_;       // 横位置がこれ[m]潰されるなら保てない
  const bool attempt_hold_side_;    // 試行中は寄ると決めた側を保持しきるか
  const double attempt_hold_sep_;   // そのとき保持する横オフセットの最小値[m]
  const double attempt_latfail_time_; // 横に出られない試行を打ち切る秒数(0で無効)
  // 追越試行の開始に「横位置の調停で追越の意図が採用されていること」を要求する。
  // false にすると従来どおり横間隔のしきい値だけで数える(A/B 用)。
  const bool attempt_require_intent_;
  // latch に追い越しの「許可」を持たせるか。false なら latch は横位置の保持だけ。
  const bool latch_allow_enable_;
  const double attempt_infeasible_time_;   // 継続不能が続いたら打ち切る秒数(0で無効)
  const double attempt_wall_look_time_;    // 壁余裕を見る先読み時間[s]
  const double attempt_wall_abort_clear_;  // これを下回ったら「壁に当たる」[m]
  double attempt_infeasible_since_{-1.0};  // 継続不能になった時刻[s]
  // 幅の判定を見る距離の設定。詳細は minWidthAhead() の説明を参照。
  const double pass_width_dist_gain_;  // 幅を見る距離 = 抜き切る距離 x これ(0で従来)
  const double pass_width_dist_max_;   // 幅を見る距離の上限[m]
  const double commit_boost_time_;  // 並走が続いたらブーストを撃つまでの秒数(0で無効)
  const bool boost_runup_enable_;   // 並ぶ前(助走段階)にブーストを撃つか
  const double boost_runup_gap_;    // 助走ブーストを撃つ車間の上限[m]
  const double boost_runup_gap_min_;// 同 下限[m]。近すぎると助走にならず追突する
  const double wall_margin_;        // 横目標を壁から必ず離す量[m]
  const double wall_margin_stopped_;  // 停止車を避けるときの壁の余裕[m]
  const bool straight_pass_enable_;   // 直線の追い越しを通しきるか
  const double straight_pass_wall_;   // そのとき壁に残す余裕[m]
  const double straight_pass_sep_;    // そのとき「重なり」とみなす横間隔[m]
  const double straight_pass_hold_;   // 区間を出てから維持する時間[s]
  const double straight_finish_margin_;  // 直線の残りに足して使える距離[m]
  const double stop_avoid_crush_;     // 横目標がこれ[m]以上潰されたら通れない扱い
  // true にすると、潰された「量」ではなく **潰された後の位置が空き帯に入るか**
  // で通れるかを判定する。A/B 計測用。
  const bool stop_avoid_fit_;
  // 停止車までこの距離[m]以内のときだけ、横目標が壁帯で潰されたことを
  // 「通れない」とみなして減速する。遠方では手前の狭さは通過可否と無関係。
  const double stop_avoid_crush_range_;
  // 停止車の脇に通れる帯があるとき、追突防止に許させる最低速度[m/s]。
  // 0 だと横へ寄るための前進ができず膠着する。
  const double stop_creep_speed_;
  // これより車間が近いときは creep しない[m]。
  const double stop_creep_gap_;
  const double crash_safe_sep_;     // 相手へ寄るときに許す横間隔[m]
  // --- wedge(正面衝突回避)の逃げ先の直し ---
  // 相手が特定できているとき、逃げ先を相手側へ反転させない。
  const bool wedge_no_flip_;
  // 逃げ場が無い並走で、減速ではなく前へ出る(Crash は自分の前で当てた側に付く)。
  const bool wedge_forward_;
  // wedge 発火中は再クランプに corridor_safety ではなく wedge_room を使う。
  const bool wedge_keep_room_;
  // 車体端-物理壁の余裕がこれ[m]未満のときだけ相手側へ寄る。
  // 実測168件では帯端でも余裕は最小0.70/中央値1.10m あり、
  // 「壁が本当に近い」件は1件も無かった。
  const double wall_pick_need_;
  // make_corridor.py --margin の値。帯端から実壁までの追加余裕[m]。
  // コリドアCSVを作り直したらこの値も合わせること。
  const double corridor_extra_;
  // 退避スイッチ。true で「相手から crash_safe_sep まで詰める」旧挙動へ戻す。
  const bool wall_pick_legacy_;
  const double crash_front_near_;   // 相手が「自分の前」とみなす前後距離の下限[m]
  const double crash_front_far_;    // 同 上限[m]
  const double wall_brake_ratio_;   // 両方避けられないときの減速率
  const double tight_radius_;       // これ未満の曲率半径[m]で壁の余裕を増やす
  const double wall_margin_tight_;  // きついコーナーで足す余裕の最大[m]
  const bool enable_;

  // ===================================================================
  // スタートの発進制御 / 追い越し地点の計画 / 周回による解禁
  // (ユーザー指示 2026-08-29)
  // ===================================================================
  //
  // 【スタート】発進待ちと P1 の右寄せは実走で機能せず削除した(2026-08-29)。
  //   横位置をコリドアで丸める処理だけ残してある。
  const double start_lat_abs_;       // スタート横位置の絶対上限[m](保険)
  const double start_wall_margin_;   // スタートの横位置を壁から空ける量[m]
  // --- 発進レーンの保持(holdGridLane)。既定 off の第2段つき ---
  const bool   launch_hold_grid_;    // 第1段: 発進中はグリッドの横位置を保つ
  const double launch_hold_dist_;    // 第1段の権限が及ぶ走行距離[m]
  const double launch_hold_sec_;     // 保険。合図からこの秒数で降りる
  const bool   launch_p1_right_;     // 第2段: P1 だけ右へ出して MPC を即抜く
  const double launch_p1_lat_near_;  // P2 がまだ横に居る間の右目標[m]
  const double launch_p1_lat_far_;   // P2 が前へ抜けた後の右目標[m]
  const double launch_p1_clear_;     // P2 の前方弧長差[m]がこれを超えたら far へ
  const double launch_p1_dist_;      // 第2段の権限が及ぶ走行距離[m]
  // --- 第3段: 発進の一発追い越し ---
  const bool   launch_p1_pass_;          // 親スイッチ(退避スイッチ)
  const double launch_p1_pass_sec_;      // 合図からこの秒数だけ許す
  const double launch_p1_pass_sep_;      // この横間隔[m]がある間だけ上限を外す
  const double launch_p1_pass_ahead_;    // 弧長差がこれだけ負になったら完了
  const double launch_p1_lat_step_;      // 横目標を動かす速さの上限[m/s]
  const bool   launch_p1_hold_until_pass_;  // 抜き切るまで層を降ろさない
  const double launch_no_pass_guard_dist_;  // 禁止区間の手前で発進追越を諦める距離[m]
  const double start_offset_rate_;   // 合流までの横方向のレート[m/s]
  //
  // 【追い越し地点の計画】録画した相手のラインと速度から、
  // 「どこで・どちら側から抜くか」を先に決める。決めた地点までは詰めるだけで
  // 仕掛けず、地点の入口でちょうど pass_gap になるように助走する。
  const bool spot_enable_;
  const double spot_range_;      // 前方これだけ[m]先まで候補を探す
  const double spot_min_len_;    // 候補として認める連続区間の下限[m]
  const double spot_recalc_sec_; // 計画を作り直す間隔[s]
  const double spot_slow_bonus_; // 相手が遅い区間を優先する重み
  const double spot_margin_;     // 相手の録画ラインから空けたい横間隔[m]
  const double spot_run_gap_;    // 幾何条件を割ってもまたいで区間を継続してよい長さ[m]
  const bool spot_gate_;         // 計画した地点の外では仕掛けないか
  const double spot_gate_slack_; // 地点の手前これだけ[m]から仕掛けてよい
  const double spot_gate_max_wait_;  // 抜きどころ待ちの上限[s]
  const double spot_path_tol_;    // spotPathSafeで許容するクランプ量[m]
  const double spot_path_bad_len_;  // 連続して超過してよい区間長の上限[m]
  const double spot_path_min_w_;  // 線が外れても帯としてこの幅[m]以上あれば通行可とみなす
  const double spot_stuck_max_;   // 抜きどころに着いて仕掛けられない状態の許容[s]
  const double spot_avoid_sec_;   // 破棄した地点を候補から外す時間[s]
  const double spot_abort_sec_;   // 予測不成立が継続したら中断するまでの時間[s]
  const double side_room_margin_; // 左右の空き幅の差がこれ[m]を超えたら広いほうへ
  const double side_room_hold_;   // 反対側が連続で勝つべき時間[s]
  const std::string side_pick_zone_spec_;  // 側を録画で決める区間
  const double side_pick_tie_;    // 相手の平均横位置がこれ[m]以内なら右から抜く
  const bool rear_end_guard_;     // 対象以外への追突を止めるか
  const double rear_end_range_;   // 前方これだけ[m]の車を見る
  const double rear_end_sep_;     // 横間隔がこれ[m]未満なら自分の進路上
  const double rear_end_margin_;  // 止まりきる位置に残す余裕[m]
  const double start_gap_closing_max_;  // 開始車間緩和を許す接近速度の上限[m/s]
  const double start_gap_floor_;  // 開始車間の最小限(貼り付き直後の新規横出しを防ぐ)[m]
  const double rear_end_brake_k_; // 減速の見積りを割り引く係数
  const double rear_end_back_k_;  // 食い込んだとき相手より遅くする割合[1/s]
  const double rear_end_time_;    // 反応の遅れとして見込む時間[s]
  const double squeeze_ahead_;    // 前に車がいるとき帯を見る先の距離[m]
  const double squeeze_gap_;      // この車間[m]以内のときだけ帯で丸める
  const double guard_decel_;      // 追突防止で上限を下げる最大の率[m/s^2]
  const bool cap_rate_limit_;     // 速度上限の下げ方に全層まとめて制限を掛けるか
  const double cap_decel_;        // その最大の減速率[m/s^2]
  const double rear_end_near_;    // この距離[m]以内は横ずれに関係なく見る
  const double rear_end_near_closing_;  // そのとき要る接近速度[m/s]
  const double cross_gap_;        // この車間[m]未満では相手をまたがない
  const double cross_dead_;       // 真後ろ扱いにする横間隔[m]
  //
  // 【周回による解禁】1周目は録画のために NPC 以外を抜かない。
  // P1 が僚車(P2)を抜き始めるのは3周目以降。ブーストも3周目以降。
  const bool lap_gate_enable_;
  const int npc_slot_;           // 運営NPC のグリッド番号(既定 3 = 最前列)
  const int record_laps_;        // この周回数(0起点)の間は NPC 以外を抜かない
  const int teammate_pass_lap_;  // P1 が僚車を抜き始める周回(0起点)
  const int leader_pass_last_laps_;  // 先頭を抜いてよい残り周回数(0で無効)
  const bool zone_fallback_enable_;  // 予測計画が無いとき汎用ゾーンで仕掛けてよいか
  const bool side_pick_over_curve_;  // 録画で決めた側を曲率より優先するか
  const bool side_pick_by_room_;     // 側を「相手と壁の空き」で決めるか
  const bool side_room_use_min_;     // 側の空きを区間の最小で見るか(falseで平均)
  const bool attempt_lat_hold_enable_;  // 試行中の横位置を試行が保持するか
  double attempt_lat_{0.0};             // 試行が持つ横目標[m]
  bool attempt_lat_valid_{false};       // その値が有効か
  bool attempt_lat_fresh_{false};       // 今周期に evaluateOpponent が出したか
  rclcpp::Time last_lat_hold_log_{0, 0, RCL_ROS_TIME};
  // 抜きどころの成立条件の調整幅。大きいほど慎重(抜けなくなる)、小さいほど強気(当たる)。
  const double spot_entry_gap_;      // 横へ出る前に取り返す縦の安全車間[m](負で従来値)
  const double spot_pass_len_gain_;  // 前へ出切る量に掛ける係数
  const double spot_need_margin_;    // 必要距離に掛ける安全率
  const int boost_min_lap_;      // ブーストを解禁する周回(0起点)
  const double trace_dump_sec_;  // 録画を要約してログへ出す間隔[s]
  // スタート直後は何が起きたのかがログから読めなかった(実測: 速度 0 のまま
  // 数秒が過ぎるのに、速度上限のログは 10.8km/h と出ていた)。
  // 合図からこの秒数[s]の間だけ、自車と他車の速度を 2Hz で残す。
  const double telem_sec_;

  // ===================================================================
  // 【追加 2026-09-03】観測ログ専用の状態(ユーザー指示)
  //
  // ここから下は **一切制御に使わない**。requestCap / requestLat / boundLat /
  // target_offset / speed_cap / 状態遷移のどれからも参照しない。
  // 目的は「何位が何位を抜いたか」「抜かれたか」「接触の要因は何か」を
  // 後からログだけで再構成できるようにすること。
  // ===================================================================
  // 被追越の確定に必要な進行度差[m]。これ未満は並走の揺れとみなす。
  const double overtaken_margin_;
  // その差が続くべき時間[s]。V2X の位置の飛びで1周期だけ前に出ても数えない。
  const double overtaken_hold_;
  // 1周期でこれだけ[m/s]速度が落ちたら接触の疑いとする。
  const double contact_decel_;
  // この距離[m]以内に他車がいなければ「単独」の接触とみなす。
  const double contact_alone_dist_;
  // 追越成功からこの時間[s]以内の接触は「追越直後」とする。
  const double contact_after_pass_sec_;

  // ===================================================================
  // 【追加 2026-09-03】助走(run-up)
  // 抜きどころへ「速度を持って」到達するためのパラメータ。
  // 既存の助走(車間を詰めるだけ)には相対速度を作る計算が無かった。
  // ===================================================================
  const bool runup_enable_;      // 有効化。false で完全に従来動作へ戻る
  const double runup_dv_;        // 抜き切るのに要る相対速度[m/s]
  const double runup_gap_max_;   // 開ける車間の上限[m]。離れすぎない
  const double runup_margin_;    // 加速開始距離に足す余裕[m]
  // 観測用(制御には使わない)。ログへ出すだけ。
  double runup_need_gap_{-1.0};
  double runup_accel_at_{-1.0};
  std::string runup_log_state_;                        // 直近に出した状態
  rclcpp::Time runup_log_last_{0, 0, RCL_ROS_TIME};    // 直近に出した時刻

  // 全車の現在順位。累積進行度 prog の降順(= 周回数 -> 周回内進行度の辞書式)。
  // rank_ (制御が使う自車順位) とは別に、記録用に自前で持つ。
  std::map<std::string, int> rank_of_;
  int my_rank_obs_{1};            // 記録用に数え直した自車の順位
  // 相手ごとの「抜かれ監視」。前後関係が入れ替わったことを進行度差で見る。
  struct OvtWatch
  {
    bool was_behind{false};   // 一度でも自分より後ろで確定していたか
    double since{-1.0};       // margin 以上前に出た状態が始まった時刻[s]
    int rank0{0};             // そのときの自車順位
  };
  std::map<std::string, OvtWatch> ovt_;
  // 相手の速度履歴(0.5秒ぶん = 20Hz で10点)。接触時の「相手急減速」に使う。
  std::map<std::string, std::deque<double>> spd_hist_;
  double last_overtaken_t_{-1e9};   // 最後に被追越を確定した時刻[s]
  double last_pass_ok_t_{-1e9};     // 最後に「追越記録 成功」を出した時刻[s]
  int attempt_rank0_{0};            // 追越試行の開始時の自車順位
  double contact_prev_speed_{-1.0}; // 前周期の自車速度[m/s](負なら未取得)
  rclcpp::Time last_contact_ctx_log_{0, 0, RCL_ROS_TIME};

  // 全車の順位を数え直し、被追越を検出する。観測のみ。
  void trackRanks(const Frame & f);
  // 接触を検出したとき、その要因の文脈を1行で残す。観測のみ。
  void logContactContext(const Frame & f, PlanCtx & c);
  // 相手の順位。分からなければ 0。
  int oppRank(const std::string & name) const
  {
    const auto it = rank_of_.find(name);
    return it == rank_of_.end() ? 0 : it->second;
  }
  // 追い越し / 被追越の難易度の分類。
  const char * passKind(const std::string & name) const
  {
    const auto io = others_.find(name);
    if (io != others_.end() && io->second.laps < lap_) { return "周回遅れ"; }
    const int r = oppRank(name);
    if (r == 1) { return "ハンデ有(2位→1位)"; }
    if (r >= 2 && my_rank_obs_ >= 2) { return "対等"; }
    return "その他";
  }
  // 相手の直近0.5秒の速度変化率[km/h/s]。データが無ければ 0。
  double oppAccelKmh(const std::string & name) const
  {
    const auto it = spd_hist_.find(name);
    if (it == spd_hist_.end() || it->second.size() < 2) { return 0.0; }
    const double dt = 0.05 * static_cast<double>(it->second.size() - 1);
    if (dt <= 1e-6) { return 0.0; }
    return (it->second.back() - it->second.front()) * 3.6 / dt;
  }

  // --- 発進制御の状態
  double launch_since_{-1.0};     // レース開始を検知した時刻[s](計測ログの基準)
  double telem_at_{-1.0};         // スタート直後の計測ログを出した時刻[s]
  double last_speed_cap_{-1.0};   // 前の周期で最終的に出した速度上限[m/s]
  // 公式ペナルティ(速度が 5km/h に固定される)の検出
  double penalty_since_{-1.0};    // 貼り付き始めた時刻[s]
  std::size_t penalty_at_idx_{0}; // その地点
  std::string penalty_near_;      // そのとき最も近かった車
  double penalty_near_dist_{0.0};
  bool slots_assigned_{false};    // グリッド番号を確定したか
  double launch_run_{0.0};        // holdGridLane 専用の走行距離[m]
  double launch_lx_{0.0}, launch_ly_{0.0};
  bool   launch_lv_{false};
  double launch_want_prev_{0.0};  // 直前に出した横目標(レート制限の基準)
  double launch_want_t_{-1.0};    // その時刻[s]
  bool   launch_want_valid_{false};

  std::vector<double> band_lo_;   // 各点で許される横オフセットの右限(負)
  std::vector<double> band_hi_;   // 同 左限(正)
  // 追い越し対象だけを障害物から除いた band。
  // 【なぜ要るか】band は others_ の全車を削るので、追い越し対象そのものも
  // 障害物として削られる。その band に対して「計画ラインが通れるか」を検査
  // すると「今から抜く相手を含めて道が空いているか」という自己矛盾になり、
  // 相手が狭い区間にいる限り帯幅が0になって追い越しが構造的に不可能だった
  // (実測 20260902-143812: 閉塞地点の帯幅 0.00m が5件)。
  // 相手との必要横間隔は spot_margin_ で別途担保している。
  std::vector<double> band_lo_ex_;
  std::vector<double> band_hi_ex_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr band_pub_;
  // 【必ず時計種別を指定する】既定構築するとシミュレータ時刻と食い違い、
  // 引き算した瞬間に `can't subtract times with different time sources` を
  // 投げてノードが死ぬ(実測 20260830-231433。起動直後に落ちて素の経路が
  // 素通りし、回避も追い越しもしない状態になった)。
  rclcpp::Time last_band_pub_{0, 0, RCL_ROS_TIME};
  // デバッグ表示(GUI)へ流す状態。key=value を改行で並べただけの文字列。
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Time last_status_pub_{0, 0, RCL_ROS_TIME};
  std::map<std::string, int> band_side_;  // 相手ごとに「左を通る(+1)/右(-1)」
  // デバッグ表示用。相手ごとの予測経路(世界座標)
  std::map<std::string, std::vector<std::pair<double, double>>> band_pred_;
  rclcpp::Time last_launch_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wedge_fix_log_{0, 0, RCL_ROS_TIME};
  bool launch_pass_done_{false};  // 発進の一発追い越しが完了/失効したか
  rclcpp::Time last_hold_log_{0, 0, RCL_ROS_TIME};
  double race_start_time_{-1.0};  // レース開始を検知した時刻[s]
  // --- 追い越し地点の計画
  std::string spot_target_;       // 計画の対象車
  bool spot_valid_{false};
  std::size_t spot_begin_{0};     // 地点の入口の経路点
  std::size_t spot_end_{0};       // 出口
  double spot_side_{0.0};         // +1=左 -1=右
  double spot_offset_{0.0};       // 予測経路から決めた固定の横目標[m]
  double spot_dist_{-1.0};        // 入口までの距離[m]。負なら地点の中
  double spot_len_{0.0};          // 地点の長さ[m]
  double spot_ospeed_{-1.0};      // その地点での相手の録画速度[m/s]
  double spot_calc_at_{-1.0};
  double spot_lock_until_{-1.0};  // 対象・側・地点を再計画しない時刻[s]
  double spot_unsafe_since_{-1.0};  // 予測経路が計画ラインを塞ぎ始めた時刻[s]
  bool spot_accel_active_{false};   // 予測で算出した全開開始時刻に到達したか
  std::string spot_accel_target_;   // 加速開始ログを出した計画対象
  double trace_dump_at_{-1.0};
  bool spot_in_now_{false};       // いま計画した地点の中にいるか
  double spot_wait_since_{-1.0};  // 抜きどころ待ちが始まった時刻[s]
  double guard_cap_{-1.0};        // 追突防止の上限(レート制限つき)[m/s]
  double last_cap_out_{-1.0};     // 前の周期で出した速度上限[m/s]
  double spot_dbg_l_{0.0};
  double spot_dbg_need_{0.0};      // 抜き切るのに要る距離[m](最小の候補)
  double spot_dbg_need_len_{0.0};  // そのときの連続区間長[m]
  double spot_dbg_need_vo_{-1.0};  // そのときの相手速度[m/s]        // 見つかった連続区間の最長(左)[m]
  double spot_dbg_r_{0.0};        // 同(右)
  int spot_dbg_known_{0};         // 先読み区間で録画があった点の数
  double spot_stuck_since_{-1.0};   // 抜きどころに着いて仕掛けられない状態が始まった時刻[s]
  std::size_t spot_avoid_begin_{0}; // 直前に破棄した地点の入口の経路点
  double spot_avoid_until_{-1.0};   // その地点を候補から外す期限[s]
  // 【修正J 2026-09-02】spotPathSafe() が false を返す理由を診断するための値。
  // spotPathSafe() は const メンバ関数なので mutable にする。
  mutable double spot_path_min_w_seen_{-1.0};  // 直近の検査で見た最小の帯幅[m]
  mutable double spot_path_fail_at_{-1.0};     // 落ちた地点までの前方距離[m]
  mutable double spot_path_fail_w_{-1.0};      // 落ちた地点の帯幅[m]

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
  bool attempt_predictive_{false}; // 予測地点へ接続して開始した試行か
  double attempt_plan_dist_{-1.0}; // 試行開始時の予測入口までの距離[m]
  // ================= 追越ファネル(観測のみ / 2026-09-03) =================
  // 【なぜ必要か】従来の記録は「追越却下」が2秒ごとの周期単位で出るだけで、
  // 1回の試行がどこまで到達し、どこで最初に止まったかが分からなかった。
  // そのため頻度の小さい理由(全試行の 2.5%)を先に直してしまった。
  // ここは**観測専用**。制御はこれらを一切読まない。
  int  attempt_stage_{0};              // 到達した最大の段階(1..6)
  std::string attempt_fail_first_;     // 最初の失敗理由(後から上書きしない)
  bool attempt_runup_used_{false};     // 助走(runup_charge)が一度でも立ったか
  double attempt_rel_v_since_{-1.0};   // 相対速度が小さい状態の開始時刻[s]
  double attempt_third_since_{-1.0};   // 第三者が blocker の状態の開始時刻[s]
  // 試行の全ての出口(成功/失敗/打切/中断)から必ず呼ぶ。1行で1試行を残す。
  void logAttemptFunnel(const Frame & f, const char * result, double elapsed);
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
  // 停止車回避で中断した直後の同一相手には、横に出直す前に状況を安定させる。
  // 「停止/低速」の観測が数周期揺れても、試行開始と安全中断を繰り返さないため。
  double stop_avoid_retry_until_{-1.0};
  std::string stop_avoid_target_;
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
  double dbg_room_l_{0.0};  // 録画から見た左の平均空き幅[m]
  double dbg_room_r_{0.0};  // 同 右
  double side_room_want_{0.0};   // 空き幅から見て入れ替えたい側
  double side_room_since_{-1.0}; // その側が勝ち始めた時刻[s]
  bool straight_pass_latch_{false};  // 直線の追い越しを通しきっている最中か
  double straight_pass_since_{0.0}; // 区間内で最後に仕掛けていた時刻[s]
  bool straight_pass_now_{false};   // その周期の判定(層をまたいで使う)
  std::string attempt_target_;
  // いま side_sign_ がどの相手について決めた側なのか。buildBand が参照する。
  std::string side_target_;
  int band_side_conflict_{0};              // 側の不一致を解消した累計回数
  double band_dbg_moved_{0.0};             // バンドが目標を動かした最大量[m]
  double band_dbg_at_{0.0};                // その地点の前方距離[m]
  rclcpp::Time last_band_dbg_log_{0, 0, RCL_ROS_TIME};
  // 予測誤差の実測用
  struct PredCheck { std::string id; double due; double x; double y; };
  std::deque<PredCheck> pred_checks_;
  std::map<std::string, double> pred_push_at_;
  double pred_err_sum_{0.0};
  double pred_err_max_{0.0};
  int pred_err_cnt_{0};
  rclcpp::Time last_pred_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_conflict_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_fix_log_{0, 0, RCL_ROS_TIME};
  std::map<std::string, std::pair<int, double>> band_side_pending_;
  std::vector<double> radius_min_;      // 前後の最小をとった曲率半径[m]
  std::vector<double> prev_offs_;       // 前周期の横目標(跳びの計測用)
  double path_jump_max_{0.0};
  std::size_t path_jump_at_{0};
  rclcpp::Time last_jump_log_{0, 0, RCL_ROS_TIME};
  void buildRadiusMin();
  double minWidthAhead(const Frame & f, double dist) const;
  double minEdgeClearAhead(const Frame & f, double lat, double dist) const;
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
  std::vector<std::pair<std::size_t, std::size_t>> side_pick_zones_;  // 側を録画で決める区間
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
  bool predict_lane_speed_{true}; // 先読みに録画した相手の速度を使うか
  double predict_op_margin_{1.10}; // 相手の学習速度に掛ける安全率
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
  rclcpp::Time last_latch_log_{0, 0, RCL_ROS_TIME};   // latch で継続した周期の記録用
  // 予測が不成立になったが中断はしない、という記録のレート制限用。
  rclcpp::Time last_spot_unsafe_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_avoid_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wedge_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_push_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_contact_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_stopped_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rearend_log_{0, 0, RCL_ROS_TIME};
  // 横位置の調停結果を 2秒に1回だけ出すための時刻
  rclcpp::Time last_lat_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_deadlock_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cap_arbitration_log_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<V2XVehiclePositionArray>::SharedPtr sub_v2x_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // V2X_OVERTAKER__V2X_OVERTAKER_HPP_
