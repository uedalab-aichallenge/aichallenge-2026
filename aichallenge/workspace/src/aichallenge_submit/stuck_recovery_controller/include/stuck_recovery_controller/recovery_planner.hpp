// 壁と他車を避けて車を目標経路へ戻す操作を計算する。
#ifndef STUCK_RECOVERY_CONTROLLER__RECOVERY_PLANNER_HPP_
#define STUCK_RECOVERY_CONTROLLER__RECOVERY_PLANNER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace recovery
{

// 占有格子から作った、各点の壁までの符号付き距離の地図。
class ObstacleMap
{
public:
  // map_server 形式の yaml と同じ場所の pgm を読む。
  bool load(const std::string & yaml_path);
  bool valid() const { return !dist_.empty(); }
  // (x,y) の符号付き距離[m]。走行可能側で正、壁の中で負、地図の外は大きな値。
  double clearance(double x, double y) const;

  // rviz 表示用に地図の寸法と原点を返す。
  int width() const { return w_; }
  int height() const { return h_; }
  double resolution() const { return res_; }
  double originX() const { return ox_; }
  double originY() const { return oy_; }
  // 格子 (ix, iy) の符号付き距離[m](iy は OccupancyGrid の下端基準、範囲外は 0、表示専用)。
  double distAt(int ix, int iy) const
  {
    if (ix < 0 || iy < 0 || ix >= w_ || iy >= h_) { return 0.0; }
    const int r = h_ - 1 - iy;
    return dist_[static_cast<std::size_t>(r) * static_cast<std::size_t>(w_) + ix];
  }

private:
  std::vector<float> dist_;
  int w_{0};
  int h_{0};
  double res_{0.1};
  double ox_{0.0};
  double oy_{0.0};
};

struct Pose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

// 走行可能領域。lat は左向き法線が正で、lo <= lat <= hi なら領域内。
struct Corridor
{
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> lo;
  std::vector<double> hi;

  bool valid() const
  {
    return x.size() >= 3 && x.size() == y.size() &&
           x.size() == lo.size() && x.size() == hi.size();
  }
  // 最近傍点の添字とそこでの横位置を返す。
  bool locate(double px, double py, std::size_t & idx, double & lat) const;
};

struct VehicleParams
{
  double wheel_base{2.14};      // [m] 呼び出し側が 1.087 を渡す
  double max_steer{0.6109};     // [rad] 呼び出し側が 0.31 を渡す
  double half_width{0.6};       // 車体半幅[m]
  // base_link から前端まで[m]。
  double front_overhang{1.6};
  double rear_overhang{0.55};   // base_link から後端まで[m]
  double bound_inflate{1.18};   // コリドア lo/hi から実際の壁までの距離[m]
  double wall_margin{0.12};     // 車体外周と壁の最小間隔[m]
};

// ギアと舵角が一定の1つの操作区間。
struct Phase
{
  bool forward{true};
  double steer{0.0};     // [rad] 左が正
  double length{0.0};    // 走行距離[m](正)
  // path の半開区間。切り返し点は隣り合う両区間に含める。
  std::size_t path_begin{0};
  std::size_t path_end{0};
};

struct Plan
{
  std::vector<Phase> phases;
  std::vector<Pose> path;      // 積分した経路
  double cost{0.0};
  bool valid{false};
  // 前進区間で見込む壁・他車との最小余裕[m]。
  double min_wall_clear{1e9};
  double min_car_clear{1e9};
  // path の先頭から後退区間の点数。
  std::size_t rev_points{0};
};

// 姿勢 p に車体を置いたときの車体外周から壁までの最小距離[m]。
double wallClearanceAt(const ObstacleMap & map, const VehicleParams & veh, const Pose & p);

// 車体の前半分・後半分それぞれの壁までの最小余裕[m]。
void wallClearanceSplit(
  const ObstacleMap & map, const VehicleParams & veh, const Pose & p,
  double & front_clear, double & rear_clear);

// 復帰経路が避ける他車。
struct CarObstacle
{
  double x{0.0};
  double y{0.0};
  std::string id;
  // 動いている相手は向き付き矩形、止まっている相手は円で扱う。
  bool moving{false};
  double yaw{0.0};      // moving のときの速度ベクトルの向き
  // 向きの不確かさぶんの横方向の上乗せ[m]。
  double extra_half_width{0.0};
};

// 相手の車体寸法。V2X の位置は後軸中心とみなす。
struct OpponentShape
{
  double front{1.554};      // 基準点から前端[m]
  double rear{0.510};       // 基準点から後端[m]
  double half_width{0.725}; // 半幅[m]
  double margin{0.10};      // 余裕[m]
};

// 参照経路上の少し先を目標に、共通地点までの所要時間が最短の前後進経路を A* で探す設定。
struct GoalPlanParams
{
  double goal_ahead_min{4.0};    // 目標を置く距離の下限[m]
  double goal_ahead_max{10.0};   // 目標を置く距離の上限[m]
  double goal_ahead_step{1.0};   // 目標候補の間隔[m]
  double step{0.5};              // 素片の長さ[m]
  int max_switch{4};             // 切り返しの上限回数
  // 最初の区間の向き。0 で自由、-1 で後退から、+1 で前進から。
  int first_phase{0};
  double pos_tol{0.6};           // 目標に着いたとみなす位置の許容[m]
  double yaw_tol{0.26};          // 同 方位の許容[rad] (15deg)
  double grid_xy{0.25};          // 状態の離散化[m]
  double grid_yaw{0.175};        // 方位の離散化[rad]
  double grid_v{0.5};            // 速度の離散化[m/s]
  int max_expand{200000};        // 展開の上限
  double v_fwd_max{10.0};        // 前進で見込む上限速度[m/s]
  double v_rev_max{1.5};         // 後退の上限速度[m/s]
  double a_accel{1.37};          // 加速[m/s^2]
  double a_brake{1.66};          // 減速[m/s^2]
  double ay_max{12.0};           // 旋回で許す横加速度[m/s^2]
  double t_gear{0.30};           // 前後の切り替えに要る時間[s]
  double tail_len{40.0};         // 共通の評価地点までの距離[m]
};

// 目標へ到達する経路を返す(到達不能なら valid=false、goal_idx に到達した目標の中心線 index)。
Plan planToGoal(const Corridor & corridor, const ObstacleMap & obstacles,
                const VehicleParams & veh, const Pose & start,
                const std::vector<CarObstacle> & cars,
                const GoalPlanParams & prm,
                std::size_t * goal_idx = nullptr);

// 後退1区間と前進1区間の組を列挙して、経路へ戻れる最良の操作を返す。
Plan plan(const Corridor & corridor, const ObstacleMap & obstacles,
          const VehicleParams & veh, const Pose & start,
          double ahead_min = 4.0, double ahead_max = 16.0,
          int first_phase = 0, double min_gain = 0.0,
          double min_escape = 2.5, double min_reverse = 0.0,
          double max_reverse = 8.0,
          const std::vector<CarObstacle> & cars = {},
          double req_wall_clear = 0.0,   // 前進区間で確保したい壁との余裕[m]
          double req_car_clear = 0.0,    // 前進区間で確保したい他車との余裕[m]
          // true なら解が無いとき棄却条件を外して最も離れられる案を返す
          bool best_effort = false
          );

// 姿勢 p での他車との重なりの深さ[m](常に 0 以上、棄却判定専用)。
double carViolation(const std::vector<CarObstacle> & cars,
                    const VehicleParams & veh, const Pose & p);

// 姿勢 p での他車までの余裕[m](負なら重なり、他車なしは大きな値)。
double carClearanceAt(const std::vector<CarObstacle> & cars,
                      const VehicleParams & veh, const Pose & p);
double carClearanceAt(const CarObstacle & car,
                      const VehicleParams & veh, const Pose & p);

}  // namespace recovery

#endif  // STUCK_RECOVERY_CONTROLLER__RECOVERY_PLANNER_HPP_
