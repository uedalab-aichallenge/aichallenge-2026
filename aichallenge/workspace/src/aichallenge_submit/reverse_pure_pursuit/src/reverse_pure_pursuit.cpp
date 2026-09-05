#include "reverse_pure_pursuit/reverse_pure_pursuit.hpp"

#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <tf2/utils.h>

#include <algorithm>
#include <sstream>
#include <string>

namespace reverse_pure_pursuit
{

using motion_utils::findNearestIndex;
using tier4_autoware_utils::calcLateralDeviation;
using tier4_autoware_utils::calcYawDeviation;

ReversePurePursuit::ReversePurePursuit()
: Node("reverse_pure_pursuit"),
  // initialize parameters
  wheel_base_(declare_parameter<float>("wheel_base", 2.14)),
  lookahead_gain_(declare_parameter<float>("lookahead_gain", 1.0)),
  lookahead_min_distance_(declare_parameter<float>("lookahead_min_distance", 1.0)),
  speed_proportional_gain_(declare_parameter<float>("speed_proportional_gain", 1.0)),
  use_external_target_vel_(declare_parameter<bool>("use_external_target_vel", false)),
  external_target_vel_(declare_parameter<float>("external_target_vel", 0.0)),
  steering_tire_angle_gain_(declare_parameter<float>("steering_tire_angle_gain", 1.0)),
  lookahead_cte_gain_(declare_parameter<float>("lookahead_cte_gain", 3.0)),
  lookahead_curve_ref_(declare_parameter<float>("lookahead_curve_ref", 12.0)),
  lookahead_curve_min_(declare_parameter<float>("lookahead_curve_min", 0.5)),
  // --- 低速でだけ曲率に応じて目標点を近づける ---
  // 既定 2.78 m/s = 10 km/h。これ以上の速度では一切変更しない。
  lookahead_slow_speed_(declare_parameter<float>("lookahead_slow_speed", 2.78)),
  lookahead_slow_full_(declare_parameter<float>("lookahead_slow_full", 1.0)),
  // 縮める先 = 曲率半径 x この係数。
  // 0.8 では半径4.8mのヘアピンで 3.84m となり、基準の 3.5m より**長く**なって
  // まったく縮まっていなかった。実際に短くなる値にする。
  lookahead_curve_k_(declare_parameter<float>("lookahead_curve_k", 0.35)),
  lookahead_slow_min_(declare_parameter<float>("lookahead_slow_min", 1.5)),
  // 低速での縮め方の鋭さ。大きいほど停止に近い側で急激に短くなる。
  lookahead_slow_exp_(declare_parameter<float>("lookahead_slow_exp", 2.0)),
  // 低速でも直線ならここまでしか縮めない[m]
  lookahead_slow_far_(declare_parameter<float>("lookahead_slow_far", 3.0)),
  // 追い越し試行中に lookahead へ掛ける倍率。
  // pure pursuit は lookahead が長いほど目標線を内側へ切り込むので、
  // 横にずらした軌道に対しては「オフセットへ届くのが遅れる」形で出る。
  // 実測(自コード同士): 打切・失敗時の最大横間隔は 91% が車幅以上に
  // 達しているのに一度も抜けなかった。横へ出るのが遅いぶん、
  // 抜き切るまでに要る距離が伸びていた。試行中だけ目標点を近づける。
  lookahead_overtake_scale_(declare_parameter<float>("lookahead_overtake_scale", 0.9)),
  lookahead_zone_spec_(declare_parameter<std::string>("lookahead_scale_zones", "")),
  lookahead_curve_ahead_(declare_parameter<float>("lookahead_curve_ahead", 6.0)),
  start_steer_speed_(declare_parameter<float>("start_steer_speed", 3.0)),
  start_steer_limit_(declare_parameter<float>("start_steer_limit", 0.21)),
  stuck_steer_free_speed_(declare_parameter<float>("stuck_steer_free_speed", 0.4)),
  max_acceleration_(declare_parameter<float>("max_acceleration", 3.0)),
  // 後退時の速度上限。経路の速度(前進用にチューニングされている)をそのまま使うと
  // 速すぎるので、絶対値をここで頭打ちにする。
  reverse_max_speed_(declare_parameter<float>("reverse_max_speed", 1.5))
{
  // "165:185:0.35,10:20:0.5" の形を解析する
  {
    std::stringstream ss(lookahead_zone_spec_);
    std::string item;
    while (std::getline(ss, item, ',')) {
      std::size_t a = item.find(':');
      std::size_t b = item.rfind(':');
      if (a == std::string::npos || b == a) { continue; }
      LookaheadZone z;
      z.from = static_cast<std::size_t>(std::stoul(item.substr(0, a)));
      z.to = static_cast<std::size_t>(std::stoul(item.substr(a + 1, b - a - 1)));
      z.scale = std::stod(item.substr(b + 1));
      lookahead_zones_.push_back(z);
      RCLCPP_INFO(get_logger(), "lookahead 縮小区間 idx%zu-%zu x%.2f", z.from, z.to, z.scale);
    }
  }
  pub_cmd_ = create_publisher<AckermannControlCommand>("output/control_cmd", 1);
  pub_raw_cmd_ = create_publisher<AckermannControlCommand>("output/raw_control_cmd", 1);
  // デバッグ用トピック名が前進用(simple_pure_pursuit)と衝突しないよう reverse_ を付ける
  pub_lookahead_point_ = create_publisher<PointStamped>("/control/debug/reverse_lookahead_point", 1);

  const auto bv_qos = rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().best_effort();
  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", bv_qos, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_overtaking_ = create_subscription<std_msgs::msg::Bool>(
    "input/overtaking", rclcpp::QoS(1),
    [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { overtaking_ = msg->data; });
  sub_trajectory_ = create_subscription<Trajectory>(
    "input/trajectory", bv_qos, [this](const Trajectory::SharedPtr msg) { trajectory_ = msg; });

  using namespace std::literals::chrono_literals;
  timer_ = create_wall_timer(10ms, std::bind(&ReversePurePursuit::onTimer, this));
}

AckermannControlCommand zeroAckermannControlCommand(rclcpp::Time stamp)
{
  AckermannControlCommand cmd;
  cmd.stamp = stamp;
  cmd.longitudinal.stamp = stamp;
  cmd.longitudinal.speed = 0.0;
  cmd.longitudinal.acceleration = 0.0;
  cmd.lateral.stamp = stamp;
  cmd.lateral.steering_tire_angle = 0.0;
  return cmd;
}

// 自車の少し先(lookahead_curve_ahead_ 手前まで)の軌道の曲率半径[m]を返す。
// 3点の外接円で見る。軌道点の間隔は約2.7mなので、3点で 5m 程度の区間を見ることになる。
// 直線では半径が発散するので、上限を切って返す。
double ReversePurePursuit::localTurnRadius(size_t closest_idx) const
{
  constexpr double kStraight = 1e4;
  if (!trajectory_) {
    return kStraight;
  }
  const auto & pts = trajectory_->points;
  const size_t n = pts.size();
  if (n < 3) {
    return kStraight;
  }
  const auto at = [&](size_t i) { return pts.at(i % n).pose.position; };

  // 外接円を測る点の間隔[点]。点間隔が細かいほど、隣り合う3点で測ると
  // わずかなジグザグを曲率として拾ってしまう。約2.7m 離れた3点で測る。
  size_t span = 1;
  {
    double total = 0.0;
    for (size_t i = 0; i < n; ++i) {
      const auto & a = at(i);
      const auto & b = at(i + 1);
      total += std::hypot(b.x - a.x, b.y - a.y);
    }
    const double spacing = total / static_cast<double>(n);
    if (spacing > 1e-6) {
      span = std::max<size_t>(1, static_cast<size_t>(std::lround(2.7 / spacing)));
    }
  }

  // 現在地から curve_ahead_ [m] 先までを走査し、いちばんきつい曲率を採る。
  // 平均を採るとコーナー入口で緩く出てしまい、縮めたい場所で縮まらない。
  double tightest = kStraight;
  double travelled = 0.0;
  for (size_t k = 0; k + 2 < n; ++k) {
    const auto a = at(closest_idx + k);
    const auto b = at(closest_idx + k + span);
    const auto c = at(closest_idx + k + 2 * span);
    if (k > 0) {
      const auto prev = at(closest_idx + k - 1);
      travelled += std::hypot(a.x - prev.x, a.y - prev.y);
      if (travelled > lookahead_curve_ahead_) {
        break;
      }
    }
    const double ab = std::hypot(b.x - a.x, b.y - a.y);
    const double bc = std::hypot(c.x - b.x, c.y - b.y);
    const double ca = std::hypot(a.x - c.x, a.y - c.y);
    // 三角形の面積(外積)。潰れていれば直線とみなす。
    const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    if (std::abs(cross) < 1e-6 || ab < 1e-6 || bc < 1e-6 || ca < 1e-6) {
      continue;
    }
    const double r = (ab * bc * ca) / (2.0 * std::abs(cross));
    tightest = std::min(tightest, r);
  }
  return tightest;
}

void ReversePurePursuit::onTimer()
{
  // check data
  if (!subscribeMessageAvailable()) {
    return;
  }

  size_t closet_traj_point_idx =
    findNearestIndex(trajectory_->points, odometry_->pose.pose.position);

  // publish zero command
  AckermannControlCommand cmd = zeroAckermannControlCommand(get_clock()->now());

  // get closest trajectory point from current position
  TrajectoryPoint closet_traj_point = trajectory_->points.at(closet_traj_point_idx);

  // calc longitudinal speed and acceleration
  // --- 後退用に変更 ---
  // 経路の速度は前進用にチューニングされた値なので、絶対値を reverse_max_speed_ で
  // 頭打ちにしたうえで符号を反転し、後退方向の指令にする。
  double target_longitudinal_vel =
    use_external_target_vel_ ? external_target_vel_ : closet_traj_point.longitudinal_velocity_mps;
  target_longitudinal_vel = -std::min(std::abs(target_longitudinal_vel), reverse_max_speed_);
  double current_longitudinal_vel = odometry_->twist.twist.linear.x;

  cmd.longitudinal.speed = target_longitudinal_vel;
  // --- 加速度は「ギアの向きへのペダル」として出す(2026-09-06) ---
  //
  // 【ユーザー報告】「AWSIM 上ではリバースの表記なのに実際は前進し、その後
  // 壁にぶつかる」「P2 が復帰処理できず、リバースのまま その場に留まる」。
  //
  // 【公式仕様】docs/specifications/interface.ja.md の
  // `/control/command/control_cmd`:
  //     longitudinal.speed        -> **未使用**
  //     longitudinal.acceleration -> 目標加速度 (m/s^2)
  // AWSIM は speed を見ない。**進む向きはギアが決め、acceleration は
  // そのギアの向きへのアクセル(正)/ブレーキ(負)** である。
  // 復帰の直接制御が「REVERSE + 加速度 +1.0」で実際に後退できている
  // (実測 速度 -0.84m/s)ことからも、この規約で確定している。
  //
  // 【何が間違っていたか】ここは車両前方を正とした速度差
  //     gain * (target - current)
  // をそのまま加速度にしていた。target は後退なので負である。
  // 停止状態(current=0)で target=-1.5 なら **-1.5、つまり負**。
  // **REVERSE ギアで負の加速度はブレーキ**なので、後退の推力にならない。
  //
  // 結果として、
  //   ・経路による後退は一度も車を動かせない
  //     (ログ「復帰 後退を経路で渡しても 2.0s 動かない。直接制御に戻す」が毎回出る)
  //   ・前向きの慣性が残っていると、ギア表示は R のまま減速しながら前進し、
  //     そのまま壁へ当たる(ユーザーが目撃した挙動)
  // となっていた。
  //
  // ギアの向きで測った速度 s = -v で考えると、
  //     ペダル = gain * (s_target - s_current)
  //            = gain * (-target - (-current))
  //            = gain * (current - target)
  // つまり従来式の符号を反転したものが正しい。
  //   停止中に target=-1.5 -> +1.5*gain (後退へ踏む)
  //   -1.5 で走行中      -> 0          (維持)
  //   -2.5 で走りすぎ    -> -1.0*gain  (ブレーキ)
  cmd.longitudinal.acceleration =
    speed_proportional_gain_ * (current_longitudinal_vel - target_longitudinal_vel);
  // 加減速とも ±max_acceleration_ に収める。
  // 大会ルール(OVER ペナルティ): 加速度指令の絶対値が ±3 m/s^2 を超えるか
  // 250 Hz 以上で publish すると 2 秒間 5 km/h に制限される。
  // 元の実装は上限側しか clamp しておらず、減速側が無制限だった
  // (目標速度が急に下がると -6 m/s^2 級の指令が出てペナルティを踏む)。
  // AWSIM 側は入力を ±1.37 m/s^2 に clamp するので、2.0 に抑えても性能は落ちない。
  cmd.longitudinal.acceleration =
    std::clamp<double>(cmd.longitudinal.acceleration, -max_acceleration_, max_acceleration_);

  // calc lateral control
  //// calc lookahead distance
  //// ラインから離れているときは lookahead を伸ばして緩やかに合流させる。
  //// 固定 lookahead だと、横ずれ e に対して必要な操舵が asin(e/L) 相当まで
  //// 跳ね上がり、スタートのグリッド位置(ライン外)から復帰するときに大きく行き過ぎる。
  //// 追従できているとき(e が小さいとき)は base 側が勝つので走行中の挙動は変わらない。
  const double cross_track_error = std::hypot(
    closet_traj_point.pose.position.x - odometry_->pose.pose.position.x,
    closet_traj_point.pose.position.y - odometry_->pose.pose.position.y);
  //// 曲率に応じて lookahead を縮める。
  //// pure pursuit は lookahead が長いほどコーナーを内側へ切り込む。
  //// 直線で有利な長い lookahead を、余裕の無いコーナーでそのまま使うと
  //// 壁に届いてしまう。実測(単独走行)でコース最小半径の idx82(R=4.7m)は
  //// 外側の余裕が 0.16m しかなく、追従誤差(±0.24m)のほうが大きい。
  //// 速度による分は残し、下限側(min_distance)だけを曲率で縮める。
  // 曲率で lookahead を縮めるのはやめた。
  //
  // 実測で、狙って直したいコーナー(idx165-185)の半径は 7.0m だが、
  // コース上にはもっときつい 4.0〜4.8m のコーナーが5箇所ある。
  // 半径で縮めると、そちらのほうが強く縮まって左右に発振する。
  // 問題のコーナーが特別なのは半径ではなく、内側の余地が 0.90m しか
  // ないこと。だから場所で指定する。
  //
  // lookahead_scale_zones = "開始:終了:倍率" をカンマ区切りで並べる。
  // 例 "165:185:0.35" は idx165-185 で lookahead_min_distance を 0.35 倍。
  double curve_scale = 1.0;
  for (const auto & z : lookahead_zones_) {
    const bool inside = (z.from <= z.to)
                          ? (closet_traj_point_idx >= z.from && closet_traj_point_idx <= z.to)
                          : (closet_traj_point_idx >= z.from || closet_traj_point_idx <= z.to);
    if (inside) { curve_scale = std::min(curve_scale, z.scale); }
  }
  // 急に切り替えると舵が跳ねるので、なまして入れる
  lookahead_scale_now_ += (curve_scale - lookahead_scale_now_) * kScaleSmooth;
  curve_scale = lookahead_scale_now_;
  // --- 追い越し試行中は目標点を近づける(ユーザー指示)
  //
  // 横にずらした軌道を渡しても、lookahead が長いと遠くの点を向くので
  // オフセットへ寄るのが遅れる。試行中だけ縮めて素早く横へ出す。
  //
  // 【蛇行させた初版の誤り】合成後の lookahead **全体**に 0.6 を掛けていた。
  // 速度の項(lookahead_gain * v)まで縮むので 36km/h で 5.5m -> 3.3m と
  // 高速域が 40% 短くなり、舵が発振して蛇行した(ユーザー報告)。
  // **速度の項は高速での直進安定性そのもの**なので触ってはいけない。
  // 縮めてよいのは速度に依らない項(lookahead_min_distance)だけ。
  {
    const double want = overtaking_ ? lookahead_overtake_scale_ : 1.0;
    overtake_scale_now_ += (want - overtake_scale_now_) * kScaleSmooth;
  }
  double lookahead_distance = std::max(
    lookahead_gain_ * target_longitudinal_vel +
      lookahead_min_distance_ * curve_scale * overtake_scale_now_,
    lookahead_cte_gain_ * cross_track_error);

  // --- 低速のときだけ、曲率と現在速度で目標点を近づける(ユーザー指示)
  //
  // 【なぜ低速で当たるか】
  // lookahead = max(0.20*目標速度 + 3.5, 3.0*横ずれ)。
  // 低速では速度の項がほぼ消えるので **ほぼ 3.5m 固定**になる。
  // R=4.7m のコーナーに対して 3.5m 先の目標点は円弧を大きく横切る位置に来るため、
  // 車は内側へ切り込んで壁に当たる(lookahead が長いほど切り込むのは既知)。
  // さらに一度当たると横ずれが増え、`3.0*横ずれ` の項が勝って **もっと長くなり**、
  // いっそう切り込む。低速で何度も同じコーナーに当たるのはこの正のフィードバック。
  //
  // 【過去に棄却した曲率縮小との違い】
  // 以前の実装は**全速度域**で曲率に応じて縮めたため、コース上の R=4.0-4.8m の
  // 5箇所が強く縮まって左右に発振し棄却された(上のコメント参照)。
  // ここは **lookahead_slow_speed(既定 10km/h)未満でだけ**効かせるので、
  // レース速度での挙動は一切変わらない。発振した速度域には掛からない。
  //
  // 【なぜ curve_scale ではなく合成後に上限を掛けるか】
  // curve_scale は max() の左側にしか効かず、**横ずれの項を抑えられない**。
  // 当たった後は横ずれ項が勝っているので、そこを抑えないと連鎖が切れない。
  {
    const double v_now = std::abs(current_longitudinal_vel);
    double cap = 1e9;
    if (v_now < lookahead_slow_speed_) {
      const double radius = localTurnRadius(closet_traj_point_idx);
      // 効き方を線形から**指数関数的**に変えた(ユーザー指示)。
      //
      // 「一定以上の速度なら変更しない」はそのまま
      // (lookahead_slow_speed = 2.78m/s = 10km/h 以上では何もしない)。
      // その下では、遅くなるほど**加速度的に**目標点を近づける。
      //
      //   x = 0 (基準速度)  ... 1 (停止)
      //   w = (exp(k*x) - 1) / (exp(k) - 1)     k = lookahead_slow_exp
      // k を大きくするほど、停止に近い側で急激に短くなる。
      // 線形(k->0 相当)では、ぶつかる直前の極低速でも十分に短くならず、
      // 目標点が遠いまま切り込んで壁や相手に当たっていた。
      // **距離そのものを指数関数で減衰させる**(ユーザー指示)。
      //   x = (基準速度 - 速度)/基準速度   … 0(基準速度)〜1(停止)
      //   倍率 = exp(-k * x)
      // 基準速度(lookahead_slow_speed)で倍率1 = 変更なし。
      // そこから下は速度が落ちるほど**指数関数的**に短くなる。
      //
      // 【初版が弱すぎた点】重み w を線形補間の係数として使っていたため、
      // **6km/h で効果が 12% しか出ていなかった**。
      // 指示は「6km/h くらいの低速で指数関数的に短くする」なので、
      // 距離そのものを exp で減衰させる形に改めた。
      // k=2.0 で 8km/h 0.67倍 / 6km/h 0.45倍 / 4km/h 0.30倍。
      const double x =
        std::clamp((lookahead_slow_speed_ - v_now) / lookahead_slow_speed_, 0.0, 1.0);
      const double k = std::max(lookahead_slow_exp_, 1e-3);
      const double by_speed = lookahead_distance * std::exp(-k * x);
      // コーナーでは「円弧から外れない長さ」でも抑える。
      // 短すぎると舵が発振するので下限を切る。
      const double by_curve = radius * lookahead_curve_k_;
      cap = std::max(std::min(by_speed, by_curve), lookahead_slow_min_);
    }
    // 急に切り替えると舵が跳ねるので、なましてから掛ける。
    if (lookahead_slow_now_ > 1e8 && cap > 1e8) {
      lookahead_slow_now_ = cap;                    // どちらも無効。そのまま
    } else {
      const double target = (cap > 1e8) ? lookahead_distance * 4.0 : cap;
      if (lookahead_slow_now_ > 1e8) { lookahead_slow_now_ = target; }
      lookahead_slow_now_ += (target - lookahead_slow_now_) * kScaleSmooth;
    }
    if (lookahead_slow_now_ < 1e8) {
      lookahead_distance = std::min(lookahead_distance, lookahead_slow_now_);
    }
    lookahead_distance = std::max(lookahead_distance, lookahead_slow_min_);
  }
  //// calc center coordinate of front wheel (後退用に変更)
  //// orientation.z はクォータニオンの z 成分であって yaw ではない。
  //// ここを取り違えると基準点が最大 wheel_base/2 だけ明後日の方向へずれる。
  ////
  //// 前進の pure pursuit は「後軸を基準に前方の目標点を追う」。
  //// 後退はその鏡像で、「前軸を基準に後方の目標点を追う」ことになるため、
  //// 元の rear_x/rear_y の式(position - wheel_base/2 * cos/sin(yaw))の符号を
  //// 反転して前軸位置(front_x/front_y)を基準点として使う。
  //// 変数名は以降の式(steering 計算など)を極力変えずに済むよう rear_x/rear_y の
  //// ままにしているが、実体は前軸位置である点に注意。
  const double yaw = tf2::getYaw(odometry_->pose.pose.orientation);
  double rear_x = odometry_->pose.pose.position.x + wheel_base_ / 2.0 * std::cos(yaw);
  double rear_y = odometry_->pose.pose.position.y + wheel_base_ / 2.0 * std::sin(yaw);
  //// search lookahead point
  //// 閉ループ軌道では末尾で探索が尽きるため、先頭へ回り込んで探す。
  //// 回り込まずに end() をそのまま参照すると未定義動作になり、操舵指令が壊れる。
  ////
  //// 【後退用の前提】この探索は「最近傍点からインデックスが増える方向」へ進む。
  //// 前進用ではそれが「経路上で自車より前方」を意味したが、後退用の経路は
  //// 「後退の進行方向(=自車が向かう向き)に沿ってインデックスが並んでいる」
  //// ことを前提とする。この前提が満たされていれば探索ロジックは変更不要。
  const auto & traj_points = trajectory_->points;
  const size_t n_points = traj_points.size();
  // 閉ループ判定。
  // 固定閾値(1.0m)だと点間隔より小さくなり、実際は閉じている軌道を「開いている」と
  // 誤判定する。raceline_ten_v2 は先頭-末尾が 1.978m(=通常の点間隔)あり、
  // その結果 idx120(最終点)で lookahead 探索が打ち切られ、
  // lookahead 点が自車位置とほぼ同じになって舵角が -42度/900deg/s に暴れていた。
  // 点間隔を基準にすることで、リサンプル間隔が変わっても正しく判定できる。
  bool is_closed_loop = false;
  if (n_points > 3) {
    double span = 0.0;
    for (size_t i = 0; i + 1 < n_points; ++i) {
      span += std::hypot(
        traj_points[i + 1].pose.position.x - traj_points[i].pose.position.x,
        traj_points[i + 1].pose.position.y - traj_points[i].pose.position.y);
    }
    const double mean_spacing = span / static_cast<double>(n_points - 1);
    const double end_gap = std::hypot(
      traj_points.front().pose.position.x - traj_points.back().pose.position.x,
      traj_points.front().pose.position.y - traj_points.back().pose.position.y);
    // 先頭と末尾の隙間が「通常の点間隔の2倍」以内なら閉じているとみなす
    is_closed_loop = end_gap < mean_spacing * 2.0;
  }

  size_t lookahead_idx = closet_traj_point_idx;
  bool lookahead_found = false;
  const size_t search_count = is_closed_loop ? n_points : (n_points - closet_traj_point_idx);
  for (size_t k = 0; k < search_count; ++k) {
    const size_t i =
      is_closed_loop ? (closet_traj_point_idx + k) % n_points : (closet_traj_point_idx + k);
    const auto & p = traj_points.at(i).pose.position;
    if (std::hypot(p.x - rear_x, p.y - rear_y) >= lookahead_distance) {
      lookahead_idx = i;
      lookahead_found = true;
      break;
    }
  }
  //// 全点が lookahead 以内（開いた軌道の終端など）のときは終端を使う
  if (!lookahead_found) {
    lookahead_idx = n_points - 1;
  }
  double lookahead_point_x = traj_points.at(lookahead_idx).pose.position.x;
  double lookahead_point_y = traj_points.at(lookahead_idx).pose.position.y;

  geometry_msgs::msg::PointStamped lookahead_point_msg;
  lookahead_point_msg.header.stamp = get_clock()->now();
  lookahead_point_msg.header.frame_id = "map";
  lookahead_point_msg.point.x = lookahead_point_x;
  lookahead_point_msg.point.y = lookahead_point_y;
  lookahead_point_msg.point.z = closet_traj_point.pose.position.z;
  pub_lookahead_point_->publish(lookahead_point_msg);

  // calc steering angle for lateral control
  double alpha = std::atan2(lookahead_point_y - rear_y, lookahead_point_x - rear_x) -
                 yaw;
  cmd.lateral.steering_tire_angle =
    steering_tire_angle_gain_ * std::atan2(2.0 * wheel_base_ * std::sin(alpha), lookahead_distance);
  // 後退時は同じ舵角でも車体の振られ方が前進と逆になるため、符号を反転する。
  cmd.lateral.steering_tire_angle = -cmd.lateral.steering_tire_angle;

  // 低速時は操舵角に上限を掛ける。
  // スタート時、車両はグリッド位置(レースラインから最大 1.3 m ずれる)に置かれる。
  // 低速だと lookahead が最小値まで縮むため、この横ずれに対して asin(e/L) 相当の
  // 巨大な操舵角が出て、左右に大きく振られてから復帰する挙動になっていた。
  // 速度が乗れば lookahead も伸びて自然に収まるので、低速域だけ抑える。
  // ヘアピンでも 24 km/h(6.7 m/s)は出ているので、通常走行には掛からない。
  {
    const double v_abs = std::abs(current_longitudinal_vel);
    // ほぼ停止しているときは制限を外す。
    // 壁に当たって止まった状態では、脱出のために大きく切る必要がある。
    // ここを制限したままにすると舵角が上限に張り付き、いつまでも抜け出せない
    // (実際に d1 が 46 回リカバリーを繰り返してスタート地点から動けなくなった)。
    if (v_abs > stuck_steer_free_speed_ && v_abs < start_steer_speed_) {
      // 停止時 start_steer_limit_ から、しきい速度で通常制限まで線形に開放する
      const double ratio = (start_steer_speed_ > 1e-6) ? (v_abs / start_steer_speed_) : 1.0;
      const double limit = start_steer_limit_ + ratio * (M_PI_2 - start_steer_limit_);
      cmd.lateral.steering_tire_angle =
        std::clamp<double>(cmd.lateral.steering_tire_angle, -limit, limit);
    }
  }

  pub_cmd_->publish(cmd);
  cmd.lateral.steering_tire_angle /=  steering_tire_angle_gain_;
  pub_raw_cmd_->publish(cmd);
}

bool ReversePurePursuit::subscribeMessageAvailable()
{
  if (!odometry_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "odometry is not available");
    return false;
  }
  if (!trajectory_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/, "trajectory is not available");
    return false;
  }
  if (trajectory_->points.empty()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000 /*ms*/,  "trajectory points is empty");
      return false;
    }
  return true;
}
}  // namespace reverse_pure_pursuit

int main(int argc, char const * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<reverse_pure_pursuit::ReversePurePursuit>());
  rclcpp::shutdown();
  return 0;
}
