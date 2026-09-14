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

#include "v2x_overtaker/lateral_interval.hpp"
#include "v2x_overtaker/launch_gate.hpp"
#include "v2x_overtaker/alongside_guard.hpp"
#include "v2x_overtaker/lateral_lag.hpp"
#include "v2x_overtaker/pass_completion_side.hpp"
#include "v2x_overtaker/corridor_funnel.hpp"
#include "v2x_overtaker/rear_end_inpath.hpp"
#include "v2x_overtaker/rear_end_release.hpp"
#include "v2x_overtaker/overtake_start.hpp"
#include "v2x_overtaker/stopped_pass_reachability.hpp"
#include "v2x_overtaker/stop_nopass_latch.hpp"
#include "v2x_overtaker/trajectory_speed_cap.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <deque>
#include <array>
#include <map>
#include <unordered_map>
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
constexpr double kCarWidth = 1.30;
constexpr double kBandAbs = 6.0;  // バンドの既定の広さ[m]
// 走行可能領域の境界からこれだけ[m]内側にいるなら、壁には当たっていないとみなす。
constexpr double kContactMargin = 0.35;
// 相手の走行データの要約を出す間隔[s]
constexpr double kStatsLogSec = 20.0;

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

  static constexpr int kSections = 24;      // 走行ラインを24分割して集計
  double sec_sum[kSections]{};              // 区間ごとの速度の合計
  int    sec_cnt[kSections]{};              // 同 サンプル数
  double prev_sec_sum[kSections]{};
  int    prev_sec_cnt[kSections]{};
  bool   prev_sec_valid{false};
  double speed_sum{0.0};                    // 全体の平均用
  int    speed_cnt{0};
  double lap_start_time{0.0};               // 周回計測用
  int    last_sec{-1};
  int    laps{0};
  double last_lap_time{0.0};
  double best_lap_time{0.0};

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

  double m_v_top{-1.0};      // 観測した最高速[m/s](緩やかに減衰させる)
  double m_ay_max{-1.0};     // 観測した最大横加速度[m/s^2](同上)
  // lat = bias + gain * inside の最小二乗用
  double m_n{0.0}, m_sx{0.0}, m_sy{0.0}, m_sxx{0.0}, m_sxy{0.0};
  int    m_samples{0};

  // 1サンプル取り込む。radius はその地点の曲率半径[m]、
  // inside は「イン側がどちらでどれだけ強いか」(-1..+1)。
  void noteModel(double v, double radius, double lat, double inside)
  {
    if (v > 0.3 && v < 30.0) {
      m_v_top = (m_v_top < 0.0) ? v : std::max(v, m_v_top - 0.0002);
      if (radius > 1.0 && radius < 1e6) {
        const double ay = v * v / radius;
        if (ay < 40.0) {
          m_ay_max = (m_ay_max < 0.0) ? ay : std::max(ay, m_ay_max - 0.0004);
        }
      }
    }
    if (std::abs(lat) <= 6.0 && std::abs(inside) <= 1.0) {
      // 直近を重く見るため、たまってきたら古い寄与を薄める。
      if (m_n > 400.0) {
        const double k = 0.995;
        m_n *= k; m_sx *= k; m_sy *= k; m_sxx *= k; m_sxy *= k;
      }
      m_n += 1.0; m_sx += inside; m_sy += lat;
      m_sxx += inside * inside; m_sxy += inside * lat;
      m_samples++;
    }
  }

  bool modelReady() const { return m_samples >= 30 && m_v_top > 0.0; }

  // その地点で相手が出せると見込む速度[m/s]。当てはめ前は負を返す。
  double modelSpd(double radius) const
  {
    if (m_v_top <= 0.0) { return -1.0; }
    double v = m_v_top;
    if (m_ay_max > 0.0 && radius > 1.0 && radius < 1e6) {
      v = std::min(v, std::sqrt(m_ay_max * radius));
    }
    return std::max(v, 0.5);
  }

  // その地点で相手がいると見込む横位置[m]。当てはめ前は 1e9 を返す。
  double modelLat(double inside) const
  {
    if (m_samples < 30 || m_n < 5.0) { return 1e9; }
    const double det = m_n * m_sxx - m_sx * m_sx;
    const double bias = (std::abs(det) < 1e-6)
                          ? (m_sy / m_n)
                          : (m_sxx * m_sy - m_sx * m_sxy) / det;
    const double gain = (std::abs(det) < 1e-6)
                          ? 0.0
                          : (m_n * m_sxy - m_sx * m_sy) / det;
    // 当てはめが暴れても現実的な範囲に収める。
    return std::clamp(bias + std::clamp(gain, -3.0, 3.0) * inside, -5.0, 5.0);
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
  double topSectionSpeedRecent() const
  {
    const double * su = prev_sec_valid ? prev_sec_sum : sec_sum;
    const int    * cn = prev_sec_valid ? prev_sec_cnt : sec_cnt;
    double best = -1.0;
    for (int i = 0; i < kSections; ++i) {
      if (cn[i] < 5) { continue; }
      best = std::max(best, su[i] / cn[i]);
    }
    return best;
  }
  double topSectionSpeed() const
  {
    double best = -1.0;
    for (int i = 0; i < kSections; ++i) {
      if (sec_cnt[i] < 5) { continue; }
      best = std::max(best, sec_sum[i] / sec_cnt[i]);
    }
    return best;
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
    double lat_fallback{0.0};
    bool lat_feasible{true};
    double speed_cap{-1.0};      // 速度上限[m/s]。負なら制限なし
    // 追突までの余裕が尽き、通常の上限レート制限を待てない状態。
    // このときだけ publishTrajectory 側の減速平滑化を通さない。
    bool emergency_brake{false};

    // --- 前方の状況
    std::string blocker;         // 前をふさいでいる相手の名前。空なら前は空き
    // 今周期の decideAllow が追越開始を許可した対象。blocker と同じIDの
    // 許可証があるときだけ recordAttempt が状態遷移を開始できる。
    std::string pass_authorized_target;
    double best_gap{0.0};        // その相手までの車間[m]
    bool slow_leader{false};     // 前の相手が明らかに遅い
    bool pressed_from_behind{false};  // 後ろから詰められている
    double rear_gap{1e9};             // 後方車までの距離[m](無ければ 1e9)

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

    enum class LatPrio : int {
      kBase        = 0,   // 基準ライン
      kGridLane    = 10,  // 発進時のグリッド保持
      // 追い越す側が既に決まっているとき、相手に着く前から寄せておく。
      kPrePosition = 15,
      kRepulse     = 20,  // 近接車からの反発
      kOvertake    = 30,  // 追い越しの目標横位置
      kStartHold   = 35,
      kStoppedCar  = 40,  // 停止車集団の回避
      kCollision   = 50,  // TTC 切迫の緊急回避
      // の車体位置がコリドア/相手に食い込んだときの引き戻し。
      kBodyGuard   = 60,
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
    struct LatTrace { const char * why; double a; double b; char kind; bool won; };
    static constexpr int kLatTraceMax = 24;
    LatTrace lat_trace[kLatTraceMax];
    int lat_trace_n{0};
    void noteLat(char kind, const char * why, double a, double b, bool won)
    {
      if (lat_trace_n >= kLatTraceMax) { return; }
      lat_trace[lat_trace_n++] = {why ? why : "?", a, b, kind, won};
    }
    void requestLat(double v, LatPrio prio, const char * why)
    {
      if (!std::isfinite(v)) { return; }
      const bool won = static_cast<int>(prio) >= static_cast<int>(lat_intent.prio);
      // kind 'R' = 要求(採用) / 'r' = 要求(優先度で却下)
      noteLat(won ? 'R' : 'r', why, v, static_cast<double>(prio), won);
      if (won) { lat_intent = {v, prio, why}; }
    }
    // 制約を出す。区間を狭める方向にだけ効く。
    void boundLat(double lo, double hi, const char * why)
    {
      const bool nlo = std::isfinite(lo) && lo > lat_lo;
      const bool nhi = std::isfinite(hi) && hi < lat_hi;
      // kind 'B' = 制約が実際に狭めた / 'b' = 効かなかった
      noteLat((nlo || nhi) ? 'B' : 'b', why, lo, hi, nlo || nhi);
      if (nlo) { lat_lo = lo; lat_lo_why = why; }
      if (nhi) { lat_hi = hi; lat_hi_why = why; }
    }
    double latWant() const
    {
      auto hard = v2x_overtaker::Hard::kNone;
      if (hardBound(lat_lo_why) && !hardBound(lat_hi_why)) {
        hard = v2x_overtaker::Hard::kLo;
      } else if (hardBound(lat_hi_why) && !hardBound(lat_lo_why)) {
        hard = v2x_overtaker::Hard::kHi;
      }
      return v2x_overtaker::resolveLateralInterval(
        lat_intent.v, lat_fallback, lat_lo, lat_hi, hard).target;
    }
    // 境界が「物理的に譲れないもの」か判定する。
    // 壁・コリドアは越えれば当たるので譲れない。相手との余裕は譲れる。
    static bool hardBound(const char * why)
    {
      if (why == nullptr) { return false; }
      const std::string w(why);
      // 「姿勢ぶんの余裕」は車体の張り出しぶんを壁から引いたもので、
      // これも越えれば当たる。壁と同じ扱いにする。
      return w.find("壁") != std::string::npos ||
             w.find("コリドア") != std::string::npos ||
             w.find("姿勢") != std::string::npos;
    }

    // 空集合を壁側で解いた回数。効果を数えられないと判定もできない。
    std::size_t lat_relaxed_n{0};
    bool lat_relaxed{false};

    // 採用した意図を制約へクランプして target_offset を確定する。
    void applyLatDecision()
    {
      const double want = lat_intent.v;
      auto hard = v2x_overtaker::Hard::kNone;
      if (hardBound(lat_lo_why) && !hardBound(lat_hi_why)) {
        hard = v2x_overtaker::Hard::kLo;
      } else if (hardBound(lat_hi_why) && !hardBound(lat_lo_why)) {
        hard = v2x_overtaker::Hard::kHi;
      } else if (!hardBound(lat_lo_why) && !hardBound(lat_hi_why)) {
        // 両側とも「相手との余裕」= 譲れる制約。左右を車に挟まれた形。
        hard = v2x_overtaker::Hard::kNeither;
      }
      const bool was_empty = (lat_lo > lat_hi);
      const auto decision = v2x_overtaker::resolveLateralInterval(
        want, lat_fallback, lat_lo, lat_hi, hard);
      target_offset = decision.target;
      lat_feasible = decision.feasible;
      lat_relaxed = was_empty && decision.feasible;
      if (lat_relaxed) { ++lat_relaxed_n; }
      if (!lat_feasible) {
        lat_bound_why = "横制約空集合";
        emergency_brake = true;
        requestCap(0.0, "横制約空集合");
        return;
      }
      if (lat_relaxed) {
        // 壁側で解いた。縦は止めないが、由来は残す。
        lat_bound_why = "空集合を壁側で解決";
        return;
      }
      lat_bound_why = (target_offset > want + 1e-6) ? lat_lo_why
                    : (target_offset < want - 1e-6) ? lat_hi_why
                    : "なし";
    }

    struct CapReq { double v; const char * why; };
    std::vector<CapReq> cap_reqs;
    const char * cap_why{"なし"};
    std::string cap_all;

    // 上限を要求する。負値・非有限は無視する。
    void requestCap(double v, const char * why)
    {
      if (!std::isfinite(v) || v < 0.0) { return; }
      cap_reqs.push_back({v, why});
    }
    void applyCapRequests()
    {
      double best = -1.0;
      const char * bw = "なし";
      cap_all.clear();
      char buf[96];
      for (const auto & r : cap_reqs) {
        if (best < 0.0 || r.v < best) { best = r.v; bw = r.why; }
        std::snprintf(buf, sizeof(buf), "%s%s=%.1f",
                      cap_all.empty() ? "" : " ", r.why ? r.why : "-", r.v * 3.6);
        cap_all += buf;
      }
      if (cap_all.empty()) { cap_all = "なし"; }
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
    bool planned_spot_ready{false}; // 動的展開距離内にいる対象別の計画
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
                   double * room_right = nullptr,
                   // 決め直しゾーンの番号。>=0 ならその区間の外で窓を打ち切る。
                   int zone_clip = -1,
                   bool * zone_clipped = nullptr) const;
  int mapMinPts(bool zone_clipped) const;
  bool mapKnownOk(int known, bool zone_clipped) const;
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
  // --- 能力だけで「抜き切れるか」を判定する(ゾーンを使わない) ---。
  bool passPathCapable(const Frame & f, double ahead, double w_need,
                       double & fail_at, double & min_w) const;
  double latestPassAccelDelay(const Frame & f, const std::string & name,
                              const OtherState & o, double gap,
                              double exit_distance, bool use_boost,
                              double pass_start_dist) const;
  double distToSidePickZoneEnd(const Frame & f) const;
  double zoneMeanLat(const OtherState & o, std::size_t n, int & known,
                     int zone = -1) const;
  // idx が属する決め直しゾーンの番号(属さないなら -1)
  int sidePickZoneIndex(std::size_t idx) const;
  void dumpTrace(const Frame & f);
  bool passAllowedThisLap(const std::string & name, const OtherState & o,
                          bool clearly_slower) const;
  bool passUnderway(const std::string & name, double lat_sep) const;

  enum class OvState { kFollow, kPrepare, kMoveOut, kPass, kMerge, kCooldown };
  OvState ov_state_{OvState::kFollow};
  std::string ov_target_;        // 状態機械が対象としている相手
  double ov_state_since_{-1.0};  // 現在の状態に入った時刻[s]
  double ov_cooldown_until_{-1.0};
  std::string prepare_wish_;        // この周期に「接近準備してよい」と判定した相手
  double prepare_wish_gap_{1e18};   // その相手までの車間(最も近い車を選ぶため)
  // 準備状態のばたつき対策。
  bool prepare_target_seen_{false};    // この周期に対象を評価したか
  bool prepare_target_hard_ok_{true};  // 対象に対し 側/幅/禁止区間 が成立
  double prepare_lost_since_{-1.0};    // 希望が対象を指さなくなった時刻[s]
  bool prepare_free_active_{false}; // 録画なしの入口で PREPARE に入っているか
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
  bool boostLapOk() const { return !lap_gate_enable_ || lap_ >= boost_min_lap_; }
  void planOvertake(const Frame & f, PlanCtx & c);
  void preventRearEnd(const Frame & f, PlanCtx & c);
  bool inOtLane(std::size_t idx) const;
  // 車体を丸ごとレーンへ入れられる区間の中か。
  bool inOtLaneUse(std::size_t idx) const;
  // idx から先 look[m] の範囲にレーンがあるか(横移動の遅れを織り込むため)。
  bool otLaneAhead(std::size_t idx, double look) const;
  bool otLaneUsable(std::size_t idx) const;
  bool laneGuardSuppressed(const Frame & f) const;
  double otLaneEntryDistance(std::size_t idx, double look) const;
  double otLaneSpeedDeadlineDistance(std::size_t idx, double look) const;
  // 止まっている車の占有に足す余裕[m]。max(stopped_size_pad_, size_pad_)。
  double stoppedPad() const;
  // 助走が使う加速度[m/s^2]の唯一の計算箇所。ブースト分の加算も含む。
  double runupAccelMps2() const;
  // ブースト抜きの助走加速度[m/s^2]。ブースト有無を両方比較したい
  // 呼び出し側(ブーストを撃つべきか判定する側)はこちらを使い、
  // 自分で `+ boost_accel_` する。
  double runupAccelBase() const;
  bool otLaneAimWindow(std::size_t idx, double & lo_out, double & hi_out) const;
  // レーンの中にいる、または「入口までに攻撃側の速度へ到達できる位置にいる」か。
  // 横位置は指令から約20m先で実現するので、レーンに入ってから許可を出しても
  // その許可ではレーンへ入れない。入口の手前から許可する必要がある。
  bool otLaneApproach(std::size_t idx, double v_now, double v_cap) const;
  // いまの順位で許される速度上限[m/s]。
  double rankSpeedCap() const;

  void buildInsideTable(const Trajectory & in);
  double insideAt(std::size_t idx) const;
  // その地点で相手がいると見込む横位置[m]。録画があればそれを優先し、
  // 無ければモデルで埋める。どちらも無ければ 1e9。
  double oppLatAt(const OtherState & o, std::size_t idx, std::size_t n) const;
  // その地点で相手が出すと見込む速度[m/s]。同様に録画優先・モデル受け皿。負で無効。
  double oppSpdAt(const OtherState & o, std::size_t idx, std::size_t n) const;
  // その車の前方 look[m] 以内にいる車の速度[m/s]。いなければ負。
  // 「その車が遅いのは能力ではなく前が詰まっているから」を判定するのに使う。
  double queueCapFor(const OtherState & o, double look) const;
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
  double avoid_range_;       // 衝突回避で見る範囲[m]
  const double collision_radius_;  // 衝突とみなす半径[m](車体2台分)
  const double avoid_min_lon_;   // これより前に出ていない相手は衝突回避の対象外[m]
  const double ttc_threshold_;     // この時間[s]以内に衝突しそうなら回避する
  const double big_gap_closing_;   // 速度差[km/h]がこれ以上なら距離制限を外す
  const double inside_time_gain_;  // イン側から抜くときの所要時間の許容倍率
  const double inside_width_gain_;
  const double latch_width_gain_; // イン側から抜くときの必要幅の倍率
  double curve_sign_{0.0};         // 現在地点の曲率の向き (+1=左旋回)
  const double pass_gap_;
  const double pass_gap_clear_ratio_;  // 相手との車体の隙間を車幅の何倍取るか
  const double pass_side_clear_;  // 追い越し中とみなす最小の横間隔[m]
  const bool gate_sidestep_after_merge_;  // 門への横逃がしは合流後だけ
  const bool predict_all_targets_;  // 予測判定を全相手に適用する
  // 追突防止を完全に外してよい横間隔[m]。既定は車体の全幅 1.45m。
  double pass_beside_sep_;
  const double offset_rate_;
  double corridor_safety_;
  double corridor_safety_pass_;  // 追い越し試行中に使う縁からの余裕[m]
  double corridor_safety_zone_;  // 追い越し可能ゾーンで使う縁からの余裕[m]
  const double side_room_ahead_;   // 左右の余地を見る先読み距離[m]
  const double side_flip_hold_;    // 余地なしがこの秒数続いたら反対側へ回る[s]
  const int side_flip_max_;        // 対象車1台につき側を変更してよい回数(バースト)
  const double side_flip_regen_;   // 変更の枠がこの秒数につき1回ぶん回復する
  const bool lane_learn_;          // 相手の横位置を地点ごとに学習するか
  const bool lane_map_side_;
  const bool side_model_fill_;  // 側の選択で地図の穴をモデルで埋めるか
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
  const double pass_done_len_;       // 完全に前へ出たと認める相対距離[m]
  const bool spot_here_enable_;      // 即時の抜きどころを認めるか
  const double spot_here_range_;     // 即時判定を行う車間の上限[m]
  const double pass_done_sec_;       // その状態を維持すべき時間[s]
  const double pass_done_min_sec_;   // 追い越し1件の所要時間の下限[s]
  const double pen_speed_;           // ペナルティ時の固定速度[m/s] (5km/h)
  const double pen_tol_;             // 許容幅[m/s]
  const double pen_hold_;            // この秒数continuousで確定[s]
  const bool wall_guard_enable_;         // 有効化
  const double wall_guard_horizon_;      // 予測する秒数[s]
  const double wall_guard_dt_;           // 刻み[s]
  const double veh_half_width_;          // 車体半幅[m](実測 1.46m の半分)
  const double veh_wheel_base_;          // 制御モデル上の実効ホイールベース[m]
  const double veh_front_overhang_;      // 前オーバーハング[m]
  const double veh_rear_overhang_;       // 後オーバーハング[m]
  const double veh_max_steer_;           // 最大舵角[rad]
  const double wall_guard_margin_;       // 走行可能領域の内側に残す余裕[m]
  const double wall_guard_run_;          // この長さ[m]以上連続で違反したら作動
  const double wall_guard_ay_max_;       // 速度制限に使う横加速度上限[m/s^2]
  const double wall_guard_corridor_half_;

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
  // 方位差 e[rad](正=左を向く)のときの車体の左右の張り出し[m]。
  // その地点の曲率半径[m]。取れなければ -1。
  double curveRadiusAt(std::size_t idx) const;
  void bodyExtent(double e, double & ext_left, double & ext_right,
                  double curve_radius = -1.0, int curve_sign = 0) const;
  // 車体が縦に重なっている相手へ、横に近づく動きを禁じる。
  void holdSideAlongside(const Frame & f, PlanCtx & c);
  void bodyGuardByMeasured(const Frame & f, PlanCtx & c);
  void publishSteerLimit(double lo, double hi, bool viol);
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
  const double boost_retry_sec_;     // 1個目が効かなかったと判断するまでの時間[s]
  const double boost_min_speed_;     // これ未満[m/s]では撃たない(密集した発進直後を避ける)
  const double boost_accel_;         // ブーストの加速度上乗せ[m/s^2] (実装値 0.5)
  const double boost_min_headroom_;  // 加速余地[m/s]がこれ未満なら撃たない
  const double vehicle_accel_;       // 実効加速度[m/s^2]。AWSIM のクランプは1.0だが
  const double zone_exit_margin_;    // ゾーン出口を越えても許す距離[m]。
                                     // 追い越しには 40〜50m 要るがゾーンは 14〜24m しかない。
                                     // ゾーン内で並びかけ、出口を越えて完了する形を許す。
  const double leader_speed_cap_;    // 1位の速度制限[km/h] (実装値 25.0)
  const double rank1_corner_gain_;   // 1位(25km/h制限あり)のとき
  const double rank2_corner_gain_;   // 2位以下(制限なし)のとき
  const double rank2_speed_cap_;     // 2位以下の上限[km/h] (driveFadeSpeed 36.0)
  const bool rank_shape_enable_;
  const bool   final_dash_enable_;   // マスタ退避スイッチ
  const int    boost_reserve_final_; // 最終区間用に残す個数
  const double final_dash_dist_;     // フィニッシュまでの残り[m]。以下を最終区間とする
  const double final_dash_gap_;      // 最終区間で前車がこの距離以内なら撃つ
  const bool   leader_boost_block_;  // 1位で加速余地が無ければ撃たない
  const double leader_boost_headroom_;
  const bool   final_dash_boost_when_leading_;  // 検証用。1位でも最終区間で撃つ
  const double near_radius_;
  const double min_lat_sep_;
  const bool repulse_need_allow_;
  // 反発で逃げるとき、帯の端に残す余白[m]。0 だと端に張り付いて壁接触が増えた。
  const double repulse_band_margin_;
  // ブーストを撃ってよい最大の車間[m]。これより後ろで撃つと追突する。
  const double boost_side_gap_;
  double min_pass_width_;
  const bool rear_end_lat_release_;   // 横にずれた分だけ追突防止を緩めるか
  const double rear_end_free_min_;
  const bool lat_lag_by_speed_;
  const double lat_lag_sec_;
  const bool stop_nopass_release_on_pass_;  // 通れる判定に戻ったらラッチを解く
  const double lat_lag_base_;
  const double lat_lag_max_;
  // 準備(横へ出始める)を開始する車間を、必要距離から決める。
  const bool prepare_early_;
  const double prepare_early_margin_;
  rclcpp::Time last_prepare_gate_log_{0, 0, RCL_ROS_TIME};
  std::size_t prepare_early_count_{0};
  const bool spot_look_local_;
  rclcpp::Time last_look_short_log_{0, 0, RCL_ROS_TIME};
  std::size_t look_short_count_{0};
  const std::string caution_zone_spec_;
  std::vector<std::pair<std::size_t, std::size_t>> caution_zones_;
  const bool side_recheck_;
  const double side_recheck_sec_;
  const bool side_by_completion_;
  // 側を「広いほう」ではなく「早く通せるほう」で決めるか。
  const bool side_plan_enable_;
  const double side_plan_reach_gain_;   // 到達時間に掛ける安全率
  const double side_plan_room_margin_;  // 通れると認める最小の空き[m]
  const double side_plan_ay_max_;       // 追従可能性の判定に使う横加速度の限界[m/s^2]
  // 計算が「抜ける」と答えた側を、失敗するまで決め直さないか。
  const bool side_commit_;
  bool side_committed_{false};
  std::string side_commit_target_;
  // 計画のときに見る窓の範囲(抜き切り距離に対する比)。0 で全域。
  const double side_plan_window_;
  // 側の判定に使う窓の最低の長さ[m]。短いと『開いている側が反転する』手前で切れる。
  const double side_window_min_m_;
  // 側の判定に要る横間隔へ rear_end_free_min を含めるか。
  const bool side_need_rear_free_;
  // どちらの側でも必要間隔に届かないとき、横へ出るのをやめるか。
  const bool side_hold_when_none_;
  const bool allow_need_full_;
  // 「どちらの側も抜けない」がこの秒数続いたら試行を降りる。0 で無効。
  const double attempt_giveup_time_;
  double attempt_nopass_since_{-1.0};
  std::size_t attempt_nopass_n_{0};
  // 同上の判定結果(その周期で「どちらでも抜けない」か)。
  bool no_pass_side_{false};
  std::string no_pass_side_target_;   // その判定が誰についてのものか
  rclcpp::Time last_no_side_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_no_start_log_{0, 0, RCL_ROS_TIME};
  std::size_t no_side_hold_n_{0};
  // 窓の探索を、いま走っている直線の中に閉じ込めるか。
  const bool side_search_in_zone_;
  const bool side_zone_end_clamp_;
  // plan_mode で縮尺 0.85(窓の短縮)を使わない
  const bool side_plan_full_scale_;
  const double pass_need_room_m_;
  // 計画が側を決められなかったとき、**直線の幾何そのもの**を既定にする。
  const bool side_zone_geom_default_;
  // 停止車の横を通るときの要る横移動を「帯に入るまで」で測る。
  const bool stopped_band_edge_;
  // 先の閉まり方から逆算して、戻れない縁へ横目標を出さない(idx170 対策)。
  const bool reachable_lat_clamp_;
  const double reachable_lat_ahead_;
  const bool side_default_by_room_;
  const double side_default_room_hyst_;
  // その判定に使う、抜き切り地点の手前の長さ[m]。
  const double side_geom_tail_m_;
  // 直線の残りがこれ未満なら幾何の既定も出さない[m](端切れで決めない)。
  const double side_geom_min_m_;
  // 助走の基準を、録画の抜きどころではなく計画の「抜き始める地点」にする。
  const bool runup_use_plan_;
  // 助走の加速度にブーストぶんを足す。
  const bool runup_boost_accel_;
  // 抜き切り距離に加速フェーズを入れる。
  const bool pass_dist_accel_;
  // 抜き切り計算が「ブーストが要る」と答えたら撃つ。
  const bool pass_boost_gate_;
  // 停止車の占有帯にコーナーの張り出しを足す。
  const bool occupied_yaw_pad_;
  // 車体の張り出しにコーナーの振り出しを足す。
  const bool body_curve_pad_;
  const bool occupied_real_width_;
  const bool side_meanlat_live_;
  // 相手の横位置だけで決める枝にもコリドアを見させる。
  const bool side_fallback_room_;
  // 側を相手と壁の間の空きで決める。
  const bool side_room_first_;
  const bool ot_lane_log_;
  const double side_run_tie_;
  // 既定の側を通せる連続長で決める。
  const bool side_default_run_;
  // 抜ける判定が出ないとき試行の開始を止める / 横移動まで止めるか。
  const bool no_pass_attempt_hold_;
  const bool no_pass_lat_hold_;
  // 「隙間が続く距離 >= 抜き切り距離」で側を決める。
  const bool side_run_decide_;
  mutable rclcpp::Time last_run_decide_log_{0, 0, RCL_ROS_TIME};
  mutable rclcpp::Time last_run_short_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_creep_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_lane_aim_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gap_open_log_{0, 0, RCL_ROS_TIME};
  // 判定→ブーストの接続。どちらも「その周期の計算結果」で毎周期入れ替える。
  bool gate_boost_want_{false};      // 門に間に合わせるにはブーストが要る
  bool straight_need_boost_{false};  // 直線内に抜き切るにはブーストが要る
  rclcpp::Time last_gate_boost_log_{0, 0, RCL_ROS_TIME};
  bool   dbg_runup_reached_{false};   // 助走ブロックに入ったか
  bool   dbg_runup_charge_{false};    // 助走が「全開」と答えたか
  bool   dbg_gate_runup_{false};      // レーン入口を基準にしたか
  double dbg_d_gate_{-1.0};           // レーン入口までの距離[m]
  double dbg_runup_vtgt_{-1.0};       // 助走の目標速度[m/s]
  double dbg_runup_accel_at_{-1.0};   // 加速開始点[m]
  double dbg_runup_dref_{-1.0};       // 加速開始の基準距離[m]
  double dbg_eff_safe_{-1.0};         // 助走が要求した車間[m]
  double runup_holding_gap_{0.0};     // 助走がいま保とうとしている車間[m]
  double dbg_gap_{-1.0};              // そのときの車間[m]
  bool   dbg_charge_now_{false};      // 最終的に全開になったか
  rclcpp::Time last_runup_audit_log_{0, 0, RCL_ROS_TIME};
  std::map<std::string, size_t> reject_why_n_;
  std::map<std::string, size_t> reject_why_straight_n_;
  size_t reject_total_n_{0};
  rclcpp::Time last_reject_sum_log_{0, 0, RCL_ROS_TIME};
  mutable rclcpp::Time last_otlane_log_{0, 0, RCL_ROS_TIME};
  const double side_run_need_;
  // 相手と自車を少し大きく見る余裕[m]。経路が無ければ外す。
  double size_pad_;
  // 横間隔のパラメータに「物理の下限 + size_pad」の床を張る。
  const bool sep_floor_enable_;
  mutable rclcpp::Time last_occ_relax_log_{0, 0, RCL_ROS_TIME};
  mutable rclcpp::Time last_meanlat_log_{0, 0, RCL_ROS_TIME};
  // 相手の中心からこれだけ離れないと当たる、という半幅[m]。
  double occupiedHalfWidth(std::size_t idx, bool pad = true) const;
  // 「ブーストを使えば直線の中で抜き切れる」と判定したか。
  mutable bool pass_need_boost_{false};
  mutable rclcpp::Time last_pass_accel_log_{0, 0, RCL_ROS_TIME};
  // plan_mode の出力(監査ログ用)
  // 相手の生座標と最近傍点の idx(位置の問題か変換の問題かを切り分ける)
  mutable double audit_opp_x_{0.0}, audit_opp_y_{0.0};
  mutable int audit_opp_idx_{-1};
  mutable double audit_plan_start_{-1.0};
  mutable double audit_plan_y_{0.0};
  mutable int audit_plan_feas_{0};      // bit0=左 bit1=右
  mutable double audit_early_l_{-1.0};
  mutable double audit_early_r_{-1.0};
  mutable int audit_rej_l_[4]{0, 0, 0, 0};
  mutable int audit_rej_r_[4]{0, 0, 0, 0};
  // 相手の観測が何秒前のものか。**これが無いとログの古さと実際のずれを。
  mutable double audit_opp_age_{-1.0};
  // completionSide をこの周期で実際に呼んだか。0 なら下の値は前周期の残り。
  mutable int audit_cs_fresh_{0};
  // 側が幾何の既定から出たか(1)、計画から出たか(0)。
  mutable int audit_geom_{0};

  mutable std::size_t plan_start_idx_{0};
  mutable bool plan_start_valid_{false};
  mutable std::string plan_start_target_;
  mutable double plan_y_target_{0.0};
  const double side_completion_window_;   // 抜き切り窓の比(0.5 = 後半半分)
  // 抜き切り地点の空きで側を決めたか / そこに入れるか。
  // side_fits_ の判定(別スコープ)まで持ち越すのでメンバにする。
  bool want_from_completion_{false};
  bool completion_room_ok_{false};
  bool ot_lane_side_{false};
  std::size_t ot_lane_side_kept_n_{0};
  rclcpp::Time last_ot_lane_keep_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_latgate_log_{0, 0, RCL_ROS_TIME};
  std::size_t latgate_block_count_{0};
  rclcpp::Time last_completion_side_log_{0, 0, RCL_ROS_TIME};
  std::size_t completion_side_count_{0};
  // 抜き切る地点の空きから側を返す。決められなければ 0。
  int completionSide(const Frame & f, const OtherState & o, double pass_dist,
                     double need_sep, double olat_now,
                     double & room_l, double & room_r) const;
  const bool rear_end_predict_release_;
  const double rear_end_predict_floor_;    // 真後ろ扱いにする横間隔[m]
  const double rear_end_predict_max_sec_;  // 外挿してよい時間[s]
  // 横間隔の変化率を作るための追跡(相手ごと)。V2X も自車も横速度を直接は持たない。
  struct SepTrack
  {
    double sep{0.0};
    double rate{0.0};                          // なました変化率[m/s]
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool init{false};
  };
  std::map<std::string, SepTrack> sep_track_;
  rclcpp::Time last_rear_predict_log_{0, 0, RCL_ROS_TIME};
  std::size_t rear_predict_release_count_{0};
  double sepRate(const std::string & name, double sep_now, const rclcpp::Time & now);    // 車体が触れない横間隔[m]。これ未満は緩めない
  const double rear_end_free_full_;   // ここまで離れたら完全に開放[m]
  const double rear_end_free_speed_;  // 完全に外れたときに許す速度[m/s]
  rclcpp::Time last_release_log_{0, 0, RCL_ROS_TIME};
  double min_pass_sep_;
  const double pass_sep_floor_;   // 追い越しで確保する横間隔の下限[m]   // 並走時に必要な横間隔[m](カート幅 1.45)
  const double min_closing_kmh_;  // これ未満[km/h]の速度差では仕掛けない(同速対策)
  const double pass_dist_max_;    // 抜き切るのにこれ以上[m]要るなら仕掛けない
  const double stopped_speed_;       // これ以下なら「止まっている」[m/s]
  // 停止判定(<= stopped_speed)への入口と、ラッチからの出口を分ける。
  // 速度推定が 1.0m/s 前後で揺れても、1.5m/s を超えて 1秒続くまでは
  // 「通れない」を保持する。
  const double stop_nopass_exit_speed_;
  const double stop_nopass_release_sec_;
  double stopped_look_ahead_;  // 停止車両を探す前方距離[m]
  double stop_margin_;         // 停止車両の手前に空ける距離[m]
  const double stop_brake_k_;
  const double stop_hold_margin_;    // 追突判定で制動距離へ足す余裕[m]

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

  const bool   band_enable_;      // バンドを計算して rviz に出す
  const bool   band_clamp_;       // 軌道オフセットをバンドへ収める
  const bool   band_predict_;     // 他車を「自分が着く時刻」まで進めて塞ぐ
  const double band_horizon_;     // 先読みする距離[m]
  const double band_long_;        // 縦にこれだけ近ければ塞ぐ[m]
  double band_car_w_;       // 相手の中心から塞ぐ横幅[m]
  const double band_slope_;       // 横オフセットの傾きの上限[m/m]
  const double band_smooth_;      // バンドの時間方向の平滑化係数(0-1)
  const double band_side_hyst_;   // 通す側を入れ替えるのに必要な差[m]
  // 予測した他車の横位置がラインへ収束する距離定数[m]。
  // MPC は参照ラインを追うので、いま外れていても走るうちに戻る。
  const double band_lat_tau_;
  // 相手の速度がこれ[m/s]未満なら、予測経路の中で相手を進めない。
  // 止まっている車を「参照ラインの20%で走り続ける」と予測していた不具合の対策。
  const double band_stop_speed_;
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
  // 前周期の停止車回避の結果。追突防止(層の順序では先に走る)が。
  bool   c_stop_avoid_pass_{false};     // 通過/減速して通過 と判定していた
  double c_stop_avoid_lo_{0.0};         // その空き帯の右限
  double c_stop_avoid_hi_{0.0};         // その空き帯の左限
  std::string c_stop_avoid_target_;     // 対象の停止車
  // 予測の当たり具合を測る先読み時間[s]。0 で無効。
  const double predict_check_sec_;
  const double predict_speed_fast_;   // P1/P2(相手チームのコード)
  const double predict_speed_slow_;   // 運営NPC のスロット
  const int predict_prior_samples_;   // これだけ標本が溜まったら実測へ移る
  const double band_long_grow_;       // 塞ぐ長さの伸び[m/s](予測誤差ぶん)
  const double band_lat_grow_;        // 同 横幅の伸び[m/s]
  const bool   band_side_lead_;   // 抜く側の判断を予測バンドに任せる
  const double cap_slow_log_kmh_; // これ未満の上限で「遅い原因」を必ず残す[km/h]
  const double band_side_look_;   // 側を決めるとき先まで見る距離[m]


  // 発進直後、停止している前車への追従キャップを外す秒数
  const double launch_free_sec_;
  const double launch_free_gap_;  // ただしこの車間より近ければ外さない[m]
  const double launch_free_decel_;  // 判定に使う実測の減速度[m/s^2]
  const double launch_free_react_;  // 指令が効くまでの空走時間[s]
  const double launch_free_room_;   // 止まりきったあとに残す車間[m]
  const double launch_stopped_grace_; // 実移動開始後、停止車分類を待つ時間[s]
  const double look_width_ahead_;
  const double v2x_timeout_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr ten_param_cb_;
  bool stopped_pad_relax_{false};   // true で「余裕込みで帯が無ければ余裕を外して測り直す」(旧挙動)
  bool race_started_{false};       // /awsim/state が Start になったか
  const double safe_gap_;
  const double safe_gap_min_;      // 車間の下限[m]
  const double safe_gap_max_;      // 車間の上限[m]。開けすぎると追い越せず順位を落とす
  const double gap_brake_ratio_;   // 制動距離を車間にどれだけ反映するか
  const double a_min_;             // 想定減速度[m/s^2]
  const double follow_kp_;
  double follow_keep_gap_;  // これ以上の車間なら相手より遅くしない[m]
  const double min_follow_speed_;
  const bool capped_self_enable_;   // 1位ハンデ中の closing 足切り緩和を使うか
  const double capped_self_closing_;// そのときに要る最低速度差[km/h]
  const double capped_self_dist_;   // そのときに許す抜き切り距離[m]
  const bool commit_pass_;          // 横に出切ったら追従キャップを外すか
  const bool latch_commit_;
  double commit_sep_;         // 外すのに要る実測の横間隔[m]
  const double commit_gap_;         // 外すのに要る前後の車間[m](これ以内)
  const double commit_release_;     // 解除のヒステリシス(commit_sep への倍率)
  const double commit_look_;        // 抜き切り前に横位置を保てるか見る距離[m]
  const double commit_crush_;       // 横位置がこれ[m]潰されるなら保てない
  const bool attempt_hold_side_;    // 試行中は寄ると決めた側を保持しきるか
  double attempt_hold_sep_;   // そのとき保持する横オフセットの最小値[m]
  const double attempt_latfail_time_; // 横に出られない試行を打ち切る秒数(0で無効)
  const bool attempt_require_intent_;
  // latch に追い越しの「許可」を持たせるか。false なら latch は横位置の保持だけ。
  const bool latch_allow_enable_;
  const bool latch_never_no_pass_;   // 禁止区間と側の余地なしは latch で越えない

  const bool stop_avoid_fix_;        // この作り直しを使うか
  const bool stop_avoid_band_local_;
  const double stop_avoid_band_span_;
  const double stop_avoid_span_;     // 壁帯を最狭で見る s 区間の長さ[m]
  const double stop_avoid_lat_lag_;  // 横位置が実現するまでの距離[m]
  const double stop_avoid_emg_margin_;  // 緊急制動の判定に足す余裕[m]
  const double stop_avoid_emg_accel_;   // 緊急制動で見込む減速度[m/s^2]
  // 「通れない」と決めた状態のラッチ。対象IDで保持し、現在の先頭車の
  // 入替りでは解除しない。
  v2x_overtaker::StopNoPassLatch stop_nopass_latch_;
  rclcpp::Time last_stop_fix_log_{0, 0, RCL_ROS_TIME};
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
  double wall_margin_;        // 横目標を壁から必ず離す量[m]
  const double wall_margin_stopped_;  // 停止車を避けるときの壁の余裕[m]
  const bool straight_pass_enable_;   // 直線の追い越しを通しきるか
  const double straight_pass_wall_;   // そのとき壁に残す余裕[m]
  const double straight_pass_sep_;    // そのとき「重なり」とみなす横間隔[m]
  const double straight_pass_hold_;   // 区間を出てから維持する時間[s]
  const double straight_finish_margin_;  // 直線の残りに足して使える距離[m]
  const double stop_avoid_crush_;     // 横目標がこれ[m]以上潰されたら通れない扱い
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
  const double wall_pick_need_;
  const double corridor_extra_;
  const bool wall_pick_legacy_;
  const double crash_front_near_;   // 相手が「自分の前」とみなす前後距離の下限[m]
  const double crash_front_far_;    // 同 上限[m]
  const double wall_brake_ratio_;   // 両方避けられないときの減速率
  const double tight_radius_;       // これ未満の曲率半径[m]で壁の余裕を増やす
  const double wall_margin_tight_;  // きついコーナーで足す余裕の最大[m]
  const bool enable_;

  // ===================================================================。
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
  bool zone_free_;                // ゾーンをやめ、能力計算だけで許可する
  double zone_free_brake_decel_;  // 中止して後ろへ戻るときの減速度[m/s^2]
  double zone_free_keep_;         // 中止時に相手の後ろへ残す距離[m]
  const double spot_path_bad_len_;  // 連続して超過してよい区間長の上限[m]
  const double spot_path_min_w_;  // 線が外れても帯としてこの幅[m]以上あれば通行可とみなす
  bool spot_w_ref_;               // 帯の幅判定を帯の基準に合わせる
  double spot_w_ref_min_;         // そのときの要求幅[m](帯が空でない余裕)
  const double spot_stuck_max_;   // 抜きどころに着いて仕掛けられない状態の許容[s]
  const double spot_avoid_sec_;   // 破棄した地点を候補から外す時間[s]
  const double spot_abort_sec_;   // 予測不成立が継続したら中断するまでの時間[s]
  const double side_room_hold_;   // 反対側が連続で勝つべき時間[s]
  const std::string side_pick_zone_spec_;  // 側を録画で決める区間
  const double side_pick_tie_;    // 相手の平均横位置がこれ[m]以内なら右から抜く
  const bool rear_end_guard_;     // 対象以外への追突を止めるか
  const double rear_end_range_;   // 前方これだけ[m]の車を見る
  const double rear_end_sep_;     // 横間隔がこれ[m]未満なら自分の進路上
  const double rear_end_margin_;  // 止まりきる位置に残す余裕[m]
  const double start_gap_floor_;  // 開始車間の最小限(貼り付き直後の新規横出しを防ぐ)[m]
  const double rear_end_brake_k_; // 減速の見積りを割り引く係数
  const double rear_end_brake_k_pass_;  // 抜く算段が付いているときの制動係数
  const double rear_end_back_k_;  // 食い込んだとき相手より遅くする割合[1/s]
  const double rear_end_time_;    // 反応の遅れとして見込む時間[s]
  const double squeeze_ahead_;    // 前に車がいるとき帯を見る先の距離[m]
  const double squeeze_gap_;      // この車間[m]以内のときだけ帯で丸める
  const bool cap_rate_limit_;     // 速度上限の下げ方に全層まとめて制限を掛けるか
  const double cap_decel_;        // その最大の減速率[m/s^2]
  const double rear_end_near_;    // この距離[m]以内は横ずれに関係なく見る
  const double rear_end_near_closing_;  // そのとき要る接近速度[m/s]
  const double cross_gap_;        // この車間[m]未満では相手をまたがない
  const double cross_dead_;       // 真後ろ扱いにする横間隔[m]
  const bool lap_gate_enable_;
  const int npc_slot_;           // 運営NPC のグリッド番号(既定 3 = 最前列)
  const int record_laps_;        // この周回数(0起点)の間は NPC 以外を抜かない
  const int teammate_pass_lap_;  // P1 が僚車を抜き始める周回(0起点)
  const int leader_pass_last_laps_;  // 先頭を抜いてよい残り周回数(0で無効)
  const bool zone_fallback_enable_;  // 予測計画が無いとき汎用ゾーンで仕掛けてよいか
  const bool ot_lane_enable_;      // オーバーテイクレーンを使うか
  const bool ot_lane_guard_;       // 低速でレーンへ入らないガード(常時有効)
  const bool yaw_margin_enable_;      // 姿勢ぶん横の許容範囲を狭めるか
  const double yaw_margin_half_len_;  // 回転で横へ張り出す長さ[m]
  const double ot_lane_guard_look_;  // ガードを効かせ始める先読み距離[m]
  const double ot_lane_guard_time_;  // 同、速度に比例して足す時間[s]
  const double ot_lane_min_kmh_;   // レーンを使うのに要る自車速度[km/h]
  const bool prepare_free_enable_;     // 録画なしで PREPARE に入るか
  const double prepare_free_gap_;      // 相手がこの距離[m]以内なら準備してよい
  const double prepare_free_vgain_;    // 自由走行の到達速度が相手+これ[m/s]以上
  const double prepare_free_grace_;    // 希望が消えてから降りるまでの猶予[s]
  const bool ot_lane_prepare_;         // レーンの手前から許可を出すか
  const double ot_lane_prepare_look_;  // 入口の何m手前から許可するかの基本値
  const double ot_lane_prepare_time_;  // それに足す「速度×この秒数」
  const double ot_lane_inset_;
  const bool body_margin_asym_;
  const double geom_front_;      // 後軸中心から前端[m]
  const double geom_rear_;       // 後軸中心から後端[m]
  const double geom_half_width_; // 車体の半幅[m]
  // 壁の境界を「車体が占める区間」で最も狭いところで取るか。
  // ヘアピンでは方位差が小さくても車体の前隅が内側の壁を削る。
  const bool wall_body_span_;
  // 車体が縦に重なっている相手へ、横に近づく動きを禁じるか。
  const bool hold_side_alongside_;
  // 上を効かせる横間隔の上限に足す余裕[m]。
  const double alongside_extra_;
  const std::string ot_lane_use_zone_spec_;
  const bool side_window_search_;
  const double side_window_search_m_;   // 何m 先まで探すか
  mutable double audit_found_at_{-1.0};
  // completionSide が**実際に使った**相手の横位置(監査ログ用)。
  mutable double audit_used_olat_{9.99};
  mutable bool audit_used_set_{false};
  mutable double audit_win_from_{0.0};
  mutable double audit_win_to_{0.0};
  mutable int audit_win_used_{0};
  mutable double audit_win_scale_{1.0};
  mutable double audit_win_span_{0.0};
  std::string audit_room_target_{};
  bool rear_end_inpath_predict_{false};
  double rear_end_inpath_max_sec_{1.5};
  rclcpp::Time last_stall_log_{0, 0, RCL_ROS_TIME};
  bool stopped_funnel_{true};
  // 停止車を「同時に避ける」ではなく「順に抜ける」として空き幅を出すか。
  bool stopped_seq_pass_{true};
  bool pass_need_room_{true};
  std::size_t side_swap_n_{0};
  std::size_t side_none_n_{0};
  rclcpp::Time last_side_swap_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_side_none_log_{0, 0, RCL_ROS_TIME};
  bool lat_relaxed_prev_{false};
  bool lat_relax_yield_{true};
  double lat_relax_drop_mps_{1.5};
  std::size_t lat_relax_yield_n_{0};
  rclcpp::Time last_relax_yield_log_{0, 0, RCL_ROS_TIME};
  bool corridor_funnel_{true};
  double funnel_ahead_m_{20.0};
  double funnel_speed_floor_{2.0};
  double funnel_ds_acc_{-1.0};
  int funnel_empty_at_{-1};
  bool spot_keep_completion_side_{false};
  std::size_t spot_side_kept_n_{0};
  rclcpp::Time last_spot_keep_log_{0, 0, RCL_ROS_TIME};
  bool pre_position_enable_{true};
  double pre_position_range_{45.0};
  // 側が決まっていれば、抜く気が無くても抜けそうな側へ寄せておくか。
  bool pre_position_always_{true};
  // 射程のどれだけを詰めた時点で寄せ量が満額になるか(0.6 = 60%)。
  double pre_position_gain_{0.6};
  bool   pre_position_pass_ok_{true};   // 抜けると判断している間も寄せ続ける
  bool   rear_end_sep_plan_ok_{true};   // 計画後が足りていれば現在値で判定
  bool   rear_end_stop_band_{true};     // 停止車は回避帯での横間隔で判定する
  bool   follow_skip_stop_pass_{true};  // 通せる停止車には追従の上限を出さない
  rclcpp::Time last_follow_skip_log_{0, 0, RCL_ROS_TIME};
  bool   stopped_bound_enable_{true};   // 停止車の側へは寄らない(制約)
  double stopped_bound_range_{25.0};    // 前方この距離[m]の停止車を見る
  rclcpp::Time last_stopped_bound_log_{0, 0, RCL_ROS_TIME};
  bool   fwd_clear_floor_{true};        // 前方が空いていれば止めない
  double fwd_clear_need_m_{6.0};        // 空いているとみなす距離[m]
  double fwd_clear_floor_kmh_{8.0};     // そのときの速度の下限[km/h]
  rclcpp::Time last_fwd_clear_log_{0, 0, RCL_ROS_TIME};
  bool   rear_audit_{true};             // 追突防止の入力を1行で残す
  double rear_audit_sec_{0.5};
  rclcpp::Time last_rear_audit_log_{0, 0, RCL_ROS_TIME};
  double runup_hold_gap_k_{1.35};       // 助走の目標車間の倍率
  bool   runup_hold_until_gap_{true};   // 目標車間に届くまで点火しない
  double runup_hold_ratio_{0.9};        // 目標のこの割合で点火を許す
  double runup_hold_margin_k_{1.15};    // 点火に要る距離の余裕係数
  double runup_react_sec_{0.35};        // 指令から実速度までの遅れ[s]
  bool   runup_hold_lat_{true};         // 横へ寄る距離も点火時期に入れる
  bool   runup_hold_block_charge_{true};// 保留中は通常の全開判定も止める
  rclcpp::Time last_runup_time_log_{0, 0, RCL_ROS_TIME};
  double runup_hold_gap_abs_{12.0};     // 目標車間の絶対的な下限[m]
  bool   dbg_hold_for_gap_{false};
  double dbg_want_hold_{0.0};
  double runup_hold_min_room_m_{12.0};  // 入口までこれ未満なら保留を解く
  int    runup_open_from_idx_{190};     // 車間を開け始める idx
  int    runup_open_to_idx_{241};       // 開けるのをやめる idx
  bool   runup_open_leader_only_{true}; // 1位に対してだけ開ける
  bool   runup_open_keep_reach_{true};  // 門に間に合う速度より遅くしない
  bool   runup_open_dv_auto_{true};     // 減速量を残り距離から逆算する
  double runup_open_dv_max_{2.0};       // その上限[m/s]
  // 「車間を開ける」dv_need 計算専用の入口探索地平[m]。ot_lane_runup_look_。
  double runup_open_look_m_{90.0};
  double stopped_size_pad_{0.25};   // 止まっている車にだけ使う余裕[m]
  bool   runup_gap_open_cmd_{true};     // 車間を開けるのを指令として出す
  bool   runup_charge_relax_{true};     // 点火中は追従/追突防止を緩める
  double runup_charge_follow_gap_{3.5}; // 点火中の追従の安全車間[m]
  bool   start_hold_priority_{true};    // 合流までグリッド列の保持を優先
  bool   npz_look_by_lat_{true};        // 禁止区間を横復帰距離ぶん先読みする
  double npz_look_max_m_{25.0};
  double ot_lane_win_safety_{0.15};     // レーンの窓を出すときの余裕[m]
  bool   ot_lane_aim_seek_{true};       // 狙い点に窓が無ければ前方を探す
  std::size_t ot_lane_aim_seek_max_{30};
  double pass_gap_margin_{0.15};        // 相手の輪郭に足す安全マージン[m]
  double pass_wall_keep_{0.25};         // 壁側の帯の縁に残す量[m]
  rclcpp::Time last_pass_gap_log_{0, 0, RCL_ROS_TIME};
  bool   ot_abort_in_nopass_{true};     // 禁止区間では追越の横要求を出さない
  rclcpp::Time last_npz_abort_log_{0, 0, RCL_ROS_TIME};
  bool   body_guard_enable_{true};      // 実測位置での引き戻しを使う(round125)
  bool   body_guard_after_merge_{true}; // 合流が終わるまで実位置ガードを止める
  bool   lane_guard_start_off_{true};   // スタート直後はレーン回避を止める
  int    lane_guard_off_from_idx_{230};
  int    lane_guard_off_to_idx_{30};
  bool   start_merge_smooth_{true};     // 合流の減衰を smoothstep にする
  double body_guard_react_m_{0.0};      // この余裕[m]を切ったら横を引き戻す
  double body_guard_hard_m_{0.35};      // これ以上はみ出したときだけ減速する
  double body_guard_cap_kmh_{14.0};     // はみ出している間の速度上限
  rclcpp::Time last_body_guard_log_{0, 0, RCL_ROS_TIME};
  bool   pre_position_sep_{true};       // 追突防止が外れる横間隔を下限にする
  double pre_position_sep_extra_{0.15}; // 解除境界に足す余裕[m]
  bool   pre_position_in_prepare_{true};// 準備中(PREPARE)でも事前寄せを続ける
  bool   pre_reject_log_{true};         // 事前寄せが発火しない理由を数える(観測専用)
  bool   accel_audit_{true};            // 加速側の判断を毎秒残す(観測専用)
  bool   start_gap_by_rearend_{true};   // 開始車間を追突防止と同じ式で出す
  bool   rear_end_target_release_{true}; // 横「目標」で追突防止を先読み解除する
  bool   rear_end_lat_plan_{true};       // 横の計画で回避できるなら減速しない
  rclcpp::Time last_latplan_log_{0, 0, RCL_ROS_TIME};
  double rear_end_target_blend_{0.5};    // 実測と目標の混ぜ具合(0=実測のみ)
  bool   runup_fuel_{true};              // 点火距離を「車間という滑走路」で決める
  bool   runup_fuel_body_{true};         // 滑走路の計算から車体長を引く
  bool   runup_sim_{true};               // 点火判定を前向きシミュレーションで行う
  bool   runup_sim_opp_model_{true};     // 相手の将来速度を modelSpd で推定する
  double opp_accel_mps2_{0.8};           // 相手の加速能力[m/s^2](予測の変化率の上限)
  double best_sep_true_{0.0};            // 追突防止の対象との真の横間隔[m](観測用)
  double best_d3_{0.0};                  // 同 直線距離[m](観測用)
  bool   gate_runup_in_lane_{true};      // レーン内でも門の目標速度を保つ
  bool   runup_sim_reach_{false};
  double runup_sim_v_{0.0};
  double runup_sim_min_gap_{0.0};
  rclcpp::Time last_sim_log_{0, 0, RCL_ROS_TIME};
  bool   gate_runup_ref_gate_{true};     // 点火の基準をレーン入口までの距離にする
  bool   rear_end_sep_true_{true};       // 横間隔を自車法線への射影で正しく測る
  bool   normal_gap_by_rearend_{true};   // 通常走行の目標車間も追突防止の式で出す
  double normal_gap_max_{14.0};          // その上限[m]
  rclcpp::Time last_normal_gap_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_fuel_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_target_release_log_{0, 0, RCL_ROS_TIME};
  double start_gap_need_max_{14.0};     // その上限[m]
  rclcpp::Time last_accel_audit_log_{0, 0, RCL_ROS_TIME};
  std::map<std::string, size_t> pre_reject_n_;
  rclcpp::Time last_pre_reject_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_pre_position_log_{0, 0, RCL_ROS_TIME};
  std::size_t pre_position_count_{0};
  int audit_cs_{0};
  double audit_need_here_{0.0};
  bool audit_cfit_now_{false};
  bool audit_cfit_ahead_{false};
  // 追い越し実行中は壁余裕を緩めるか。
  const bool ov_pass_wall_relax_;
  // レーン封鎖をアタッカーがいるときだけにするか。
  const bool ot_lane_hot_enable_;
  const double ot_lane_hot_range_;   // 後方何m まで見るか
  bool otLaneHot(const Frame & f) const;
  const bool lat_audit_;
  const double lat_audit_sec_;   // 横監査ログの間隔[s]
  double audit_olat_{9.99};
  double audit_pass_len_{-1.0};
  double audit_room_l_{-99.0};
  double audit_room_r_{-99.0};
  rclcpp::Time last_lat_audit_log_{0, 0, RCL_ROS_TIME};
  // 衝突回避の逃げる向きを追い越しの計画側に合わせるか。
  const bool ov_collide_follow_side_;
  // 前にいない相手を追い越し対象から外すか。
  const bool ov_release_stale_;
  const int lat_pred_mode_;
  const bool lane_record_enable_;
  const bool opp_model_enable_;
  // 抜きどころ計画をグリッド slot 1 以外でも作るか。
  const bool spot_all_slots_;
  std::vector<double> inside_at_;      // 地点ごとのイン側の向きと強さ(-1..+1)
  rclcpp::Time last_opp_model_log_{0, 0, RCL_ROS_TIME};
  const double ot_lane_guard_lat_; // レーン低速ガードで許す右への最大量[m](ot_lane_lat_ が空のとき)
  const bool ot_lane_side_right_;  // レーン内では側を右に固定するか
  // 低速ガードを「いまの速度」ではなく「レーンに触れる時点の速度」で判定するか。
  const bool ot_lane_guard_predict_;
  // レーンで決めた右を下流の層に上書きさせないか。
  const bool ot_lane_side_sticky_;
  // レーン区間では抜きどころの左指定に従わないか。
  const bool ot_lane_side_over_spot_;
  // 低速ガードの右限に ot_lane_lat_ の幾何を使うか。
  const bool ot_lane_guard_geom_;
  // レーンの横範囲 "idx:lo:hi,..."(レースライン基準・左が正)。
  const std::string ot_lane_lat_spec_;
  const double ot_lane_touch_margin_;   // 触れ判定に足す余裕[m](ヨーで横幅が増えるぶん)
  // idx -> (lo, hi)。無い idx はレーンが無い。
  std::unordered_map<std::size_t, std::pair<double, double>> ot_lane_lat_;
  // その idx で車体がレーンに触れない中心の右限[m](レーン外は -1e9)。
  double otLaneNoTouchLat(std::size_t idx) const;
  // idx から look[m] 先までの区間で最も厳しい右限を返す。
  double otLaneNoTouchAhead(std::size_t idx, double look) const;
  const bool side_pick_over_curve_;  // 録画で決めた側を曲率より優先するか
  const bool side_pick_by_room_;
  const bool side_pick_by_run_;   // 側を連続長で決めるか(平均ではなく)
  const bool side_zone_repick_;   // 区間が変わったら側を決め直すか
  const bool side_zone_mean_split_; // 相手の区間平均横を区間ごとに取るか
  int side_pick_zone_seen_{-2};   // 前周期に属していた決め直しゾーンの番号
  bool pick_zone_changed_now_{false}; // この周期に区間が変わったか(毎周期更新)
  bool side_zone_repicked_{false};// この対象で区間変化の決め直しを使ったか
  const char * side_src_{"未"};   // 側を最後に決めた分岐(計測のみ)
  const bool curve_side_enable_;
  const bool curve_side_in_zone_;     // 側を「相手と壁の空き」で決めるか
  const bool side_room_use_min_;     // 側の空きを区間の最小で見るか(falseで平均)
  const bool attempt_lat_hold_enable_;  // 試行中の横位置を試行が保持するか
  double attempt_lat_{0.0};             // 試行が持つ横目標[m]
  bool attempt_lat_valid_{false};       // その値が有効か
  bool attempt_lat_fresh_{false};       // 今周期に evaluateOpponent が出したか
  rclcpp::Time last_lat_hold_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_attempt_auth_log_{0, 0, RCL_ROS_TIME};
  // 抜きどころ成立条件の調整値(大きいほど慎重)。
  const double spot_entry_gap_;      // 横へ出る前に取り返す縦の安全車間[m](負で従来値)
  const double spot_pass_len_gain_;  // 前へ出切る量に掛ける係数
  const double spot_need_margin_;    // 必要距離に掛ける安全率
  const int boost_min_lap_;      // ブーストを解禁する周回(0起点)
  const double trace_dump_sec_;  // 録画を要約してログへ出す間隔[s]
  // スタート合図からこの秒数[s]、自車と他車の速度を 2Hz でログに出す。
  const double telem_sec_;

  // 以下は観測ログ専用で制御には使わない。
  // 被追越の確定に必要な進行度差[m]。これ未満は並走の揺れとみなす。
  const double overtaken_margin_;
  // その差が続くべき時間[s]。
  const double overtaken_hold_;
  // 1周期でこれだけ[m/s]速度が落ちたら接触の疑いとする。
  const double contact_decel_;
  // この距離[m]以内に他車がいなければ「単独」の接触とみなす。
  const double contact_alone_dist_;
  // 追越成功からこの時間[s]以内の接触は「追越直後」とする。
  const double contact_after_pass_sec_;

  // 助走: 抜きどころへ速度を持って到達するためのパラメータ。
  const bool runup_enable_;      // 助走を有効にする
  const double runup_dv_;        // 抜き切るのに要る相対速度[m/s]
  const double runup_gap_max_;   // 開ける車間の上限[m]。離れすぎない
  const double runup_margin_;
  const double runup_accel_mps2_;   // 助走が使う加速度[m/s^2](0以下で vehicle_accel*0.60)
  const bool   ot_lane_runup_;             // レーン入口で門を超えるよう助走する
  const double ot_lane_runup_look_;        // 入口を探す先読み距離[m]
  const double ot_lane_runup_margin_kmh_;  // 門に上乗せする余裕[km/h]
  const double runup_gate_slack_kmh_;  // 門にこれ[km/h]まで届かなくても点火
  const double ot_lane_entry_pre_m_;       // レーン入口の何m手前を目標にするか
  const bool   gate_runup_charge_;   // 入口までの残距離から逆算して全開にする
  const double gate_runup_margin_m_; // 逆算距離に足す余裕[m]
  const bool   boost_only_if_decisive_;  // ブーストが結果を変えるときだけ撃つ
  const bool   gate_runup_sidestep_;   // 門への助走中は横へ逃がして追突防止を外す
  const double gate_sidestep_extra_;   // 解除境界に足す余裕[m]
  const bool   stopped_aim_edge_;      // 停止車回避は帯の中央でなく手前の端を狙う
  const double stopped_edge_inset_;    // 端から中へ入れる量[m]
  const bool   stopped_keep_pass_exclude_;  // true=追い越し対象を停止車から除外する
  const bool   ot_lane_aim_;           // 追越レーンの窓へ実際に横位置を寄せる
  const bool   runup_gap_hold_;        // 助走の要車間を「その速度を保てる車間」で決める
  const bool   runup_gap_cap_;         // 助走の要車間に追従の上限(8m)を掛けない
  const bool   runup_gap_open_;        // 助走中は後方車がいても相手より遅くしてよい
  const double runup_open_dv_;         // そのとき相手速度から下げてよい量[m/s]
  const double runup_open_rear_min_;   // 後方車がこれより遠いときだけ緩める[m]
  const bool   gate_runup_target_gate_;// 入口への助走の目標を門+余裕に抑える
  const bool   gate_runup_late_;       // 加速開始点だけを門基準にする(目標は据え置き)
  const double ot_lane_aim_look_;      // 窓を取る先読み距離[m]
  const double ot_lane_aim_inset_;     // 窓の内側の端から中へ入れる量[m]
  const bool   stop_creep_by_band_;    // にじり出しの可否を車間でなく空き帯で決める
  const double stop_creep_slow_;       // にじり出しの速度[m/s]
  const double stop_creep_gap_min_;    // これ以下の車間では前へ出さない[m]
  // 観測用(制御には使わない)。ログへ出すだけ。
  double runup_need_gap_{-1.0};
  double runup_accel_at_{-1.0};
  std::string runup_log_state_;                        // 直近に出した状態
  rclcpp::Time runup_log_last_{0, 0, RCL_ROS_TIME};    // 直近に出した時刻

  // 全車の順位(累積進行度 prog の降順、記録用)。
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
  double launch_motion_since_{-1.0}; // 最初にいずれかの車の実移動を見た時刻[s]
  double launch_wait_log_at_{-1.0}; // Start後・物理発進前の診断ログ時刻[s]
  double telem_at_{-1.0};         // スタート直後の計測ログを出した時刻[s]
  double last_speed_cap_{-1.0};   // 前の周期で最終的に出した速度上限[m/s]
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
  std::vector<double> band_lo_ex_;
  std::vector<double> band_hi_ex_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr band_pub_;
  // 時計種別を指定しないとシミュレータ時刻との引き算で例外になる。
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
  double spot_dbg_l_{0.0};        // 見つかった連続区間の最長(左)[m]
  double spot_dbg_need_{0.0};      // 抜き切るのに要る距離[m](最小の候補)
  double spot_dbg_need_len_{0.0};  // そのときの連続区間長[m]
  double spot_dbg_need_vo_{-1.0};  // そのときの相手速度[m/s]
  double spot_dbg_r_{0.0};        // 同(右)
  int spot_dbg_known_{0};         // 先読み区間で録画があった点の数
  double spot_stuck_since_{-1.0};   // 抜きどころに着いて仕掛けられない状態が始まった時刻[s]
  std::size_t spot_avoid_begin_{0}; // 直前に破棄した地点の入口の経路点
  double spot_avoid_until_{-1.0};   // その地点を候補から外す期限[s]
  mutable double spot_path_min_w_seen_{-1.0};  // 直近の検査で見た最小の帯幅[m]
  mutable double spot_path_fail_at_{-1.0};     // 落ちた地点までの前方距離[m]
  mutable double spot_path_fail_w_{-1.0};      // 落ちた地点の帯幅[m]
  // 閉塞判定地点の「車がいない帯幅」と帯の端(診断用)。
  mutable double spot_path_fail_free_w_ = -1.0;   // その地点の「車がいない帯幅」
  mutable double spot_path_fail_lo_ = 0.0;        // 帯の下端(交差していれば lo>hi)
  mutable double spot_path_fail_hi_ = 0.0;
  mutable int    spot_path_fail_idx_ = -1;

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
  double side_unfit_since_{-1.0};
  double side_other_fit_since_{-1.0};  // 反対側に余地が続き始めた時刻[s]。負なら無し
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
  // 停止車回避で中断した直後の同一相手へ再試行しない期限[s]。
  double stop_avoid_retry_until_{-1.0};
  std::string stop_avoid_target_;
  std::string attempt_stall_name_;   // 進展なしで降りた相手
  // 接触時に回避層が何をしていたかの記録(観測用)。
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
  // 自車の区間ごとの速度(相手の sec_sum/sec_cnt と同じ bin・更新則)。
  double my_sec_sum_[OtherState::kSections]{};
  int    my_sec_cnt_[OtherState::kSections]{};
  // 自車の直近1周ぶんの区間速度。
  double my_prev_sec_sum_[OtherState::kSections]{};
  int    my_prev_sec_cnt_[OtherState::kSections]{};
  bool   my_prev_sec_valid_ = false;
  int    my_last_sec_ = -1;
  double mySectionTop() const
  {
    double best = -1.0;
    for (int i = 0; i < OtherState::kSections; ++i) {
      if (my_sec_cnt_[i] < 5) { continue; }
      best = std::max(best, my_sec_sum_[i] / my_sec_cnt_[i]);
    }
    return best;
  }
  // 直近1周ぶんの自車の区間平均の最大[m/s]。
  double mySectionTopRecent() const
  {
    const double * su = my_prev_sec_valid_ ? my_prev_sec_sum_ : my_sec_sum_;
    const int    * cn = my_prev_sec_valid_ ? my_prev_sec_cnt_ : my_sec_cnt_;
    double best = -1.0;
    for (int i = 0; i < OtherState::kSections; ++i) {
      if (cn[i] < 5) { continue; }
      best = std::max(best, su[i] / cn[i]);
    }
    return best;
  }
  bool slow_rival_by_top_;
  bool slow_rival_recent_;   // 最大を直近1周だけで取る
  double slow_rival_top_ratio_;
  rclcpp::Time last_stats_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ot_lane_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_alongside_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_yaw_margin_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_sep_floor_log_{0, 0, RCL_ROS_TIME};
  std::vector<std::pair<std::size_t, std::size_t>> boost_zones_;
  std::string no_pass_zone_spec_;
  std::string ot_lane_zone_spec_;
  std::string right_zone_spec_;
  std::string grid_slot_spec_;
  std::vector<std::pair<double, double>> grid_slots_;  // 記録したグリッド座標
  std::vector<std::pair<std::size_t, std::size_t>> no_pass_zones_;  // 追い越し禁止区間
  std::vector<std::pair<std::size_t, std::size_t>> ot_lane_zones_;  // 公式オーバーテイクレーン
  std::vector<std::pair<std::size_t, std::size_t>> ot_lane_use_zones_;
  std::vector<std::pair<std::size_t, std::size_t>> right_zones_;   // 右から抜く区間
  std::vector<std::pair<std::size_t, std::size_t>> side_pick_zones_;  // 側を録画で決める区間
  // 直線の終わりを表す区間(抜き切り判定用)。
  const std::string pass_finish_zone_spec_;
  std::vector<std::pair<std::size_t, std::size_t>> pass_finish_zones_;
  bool inPassFinishZone(std::size_t idx) const;
  double distToPassFinishZoneEnd(const Frame & f) const;
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
  rclcpp::Time last_lat_conflict_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_deadlock_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_slow_cap_log_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_cap_arbitration_log_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<Trajectory>::SharedPtr pub_;
  rclcpp::Subscription<Trajectory>::SharedPtr sub_traj_;
  rclcpp::Subscription<Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<V2XVehiclePositionArray>::SharedPtr sub_v2x_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // V2X_OVERTAKER__V2X_OVERTAKER_HPP_
