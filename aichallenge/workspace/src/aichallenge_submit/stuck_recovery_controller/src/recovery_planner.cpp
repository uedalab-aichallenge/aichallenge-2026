#include "stuck_recovery_controller/recovery_planner.hpp"
#include <map>
#include <queue>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <sstream>

namespace recovery
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
// 経路の当たり判定の刻み[m]。車体長より十分細かく取る。
constexpr double kStep = 0.25;
// これ以上そろえても通常制御には効かない、とみなす線。
// ここを超えて詰めにいくと、そのぶん後退が伸びて時間を失う。
constexpr double kYawGoodEnough = 0.15;   // [rad] 8.6deg
// 総当りで方位差の改善に与える重み。0 で無効(既定)。
// 0.30 は余裕 0.94m 相当に化けて後退を選び続けたため戻した。
constexpr double kYawGainWeight = 0.0;
constexpr double kLatGoodEnough = 0.5;    // [m] 中心線からの横ずれ

double wrap(double a)
{
  while (a > kPi) { a -= 2.0 * kPi; }
  while (a < -kPi) { a += 2.0 * kPi; }
  return a;
}

// 自転車モデルで1歩進める。ds は符号付き(負で後退)。
Pose advance(const Pose & p, double ds, double steer, double wheel_base)
{
  Pose q = p;
  // ヨーレート = (v / L) * tan(舵角)。後退では ds が負なので符号も反転する。
  const double dyaw = ds / wheel_base * std::tan(steer);
  if (std::abs(dyaw) < 1e-9) {
    q.x += ds * std::cos(p.yaw);
    q.y += ds * std::sin(p.yaw);
  } else {
    // 円弧の中点方位で積分すると刻みが粗くても誤差が出にくい
    const double mid = p.yaw + dyaw * 0.5;
    q.x += ds * std::cos(mid);
    q.y += ds * std::sin(mid);
    q.yaw = wrap(p.yaw + dyaw);
  }
  return q;
}

// Felzenszwalb の 1 次元距離変換。2 回かければ厳密なユークリッド距離場になる。
void edt1d(std::vector<float> & f, std::vector<float> & d,
           std::vector<int> & v, std::vector<float> & z, int n)
{
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<float>::max();
  z[1] = std::numeric_limits<float>::max();
  for (int q = 1; q < n; ++q) {
    float s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
    while (s <= z[k]) {
      --k;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * q - 2.0f * v[k]);
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<float>::max();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) { ++k; }
    const float dx = static_cast<float>(q - v[k]);
    d[q] = dx * dx + f[v[k]];
  }
}


// 中心線上の 2 点間の弧長[m]。進行方向にだけ数える。
double arcBetween(const Corridor & c, std::size_t from, std::size_t to)
{
  const std::size_t n = c.x.size();
  double acc = 0.0;
  std::size_t i = from;
  for (std::size_t k = 0; k < n; ++k) {
    if (i == to) { return acc; }
    const std::size_t j = (i + 1) % n;
    acc += std::hypot(c.x[j] - c.x[i], c.y[j] - c.y[i]);
    i = j;
  }
  return acc;
}

// 中心線の局所曲率半径[m]。3 点から求める。直線に近ければ大きな値。
double localRadius(const Corridor & c, std::size_t idx, std::size_t span)
{
  const std::size_t n = c.x.size();
  const std::size_t a = (idx + n - span) % n;
  const std::size_t b = idx;
  const std::size_t d = (idx + span) % n;
  const double ax = c.x[a], ay = c.y[a];
  const double bx = c.x[b], by = c.y[b];
  const double cx = c.x[d], cy = c.y[d];
  const double area2 = std::abs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax));
  if (area2 < 1e-6) { return 1e9; }
  const double ab = std::hypot(bx - ax, by - ay);
  const double bc = std::hypot(cx - bx, cy - by);
  const double ca = std::hypot(ax - cx, ay - cy);
  return ab * bc * ca / (2.0 * area2);
}

double tailTime(const Corridor & c, std::size_t start_idx, std::size_t gi,
                double v_end, const GoalPlanParams & prm)
{
  const std::size_t n = c.x.size();
  // 共通の評価地点(現在位置から tail_len 先)までの残り距離。
  const double done = arcBetween(c, start_idx, gi);
  double remain = prm.tail_len - done;
  if (remain <= 0.0) { return 0.0; }
  double v = std::max(v_end, 0.1);
  double t = 0.0;
  std::size_t i = gi;
  for (std::size_t k = 0; k < n && remain > 0.0; ++k) {
    const std::size_t j = (i + 1) % n;
    double ds = std::hypot(c.x[j] - c.x[i], c.y[j] - c.y[i]);
    if (ds > remain) { ds = remain; }
    if (ds < 1e-6) { i = j; continue; }
    const double r = localRadius(c, i, 3);
    const double v_lim = std::min(prm.v_fwd_max, std::sqrt(prm.ay_max * r));
    double v2;
    if (v < v_lim) {
      v2 = std::min(v_lim, std::sqrt(v * v + 2.0 * prm.a_accel * ds));
    } else {
      v2 = std::max(v_lim, std::sqrt(std::max(v * v - 2.0 * prm.a_brake * ds, 0.01)));
    }
    t += 2.0 * ds / std::max(v + v2, 0.2);
    v = v2;
    remain -= ds;
    i = j;
  }
  return t;
}

}  // namespace

bool ObstacleMap::load(const std::string & yaml_path)
{
  std::ifstream y(yaml_path);
  if (!y.is_open()) { return false; }
  std::string image;
  double res = 0.1;
  double ox = 0.0, oy = 0.0;
  int origin_seen = 0;
  std::string line;
  // 使うのは image / resolution / origin の3つだけなので、
  // yaml ライブラリを足さずにこの3つだけ拾う。
  while (std::getline(y, line)) {
    std::istringstream ss(line);
    std::string key;
    if (line.rfind("- ", 0) == 0 && origin_seen > 0 && origin_seen < 3) {
      const double v = std::atof(line.substr(2).c_str());
      if (origin_seen == 1) { ox = v; } else if (origin_seen == 2) { oy = v; }
      ++origin_seen;
      continue;
    }
    if (!(ss >> key)) { continue; }
    if (key == "image:") { ss >> image; }
    else if (key == "resolution:") { ss >> res; }
    else if (key == "origin:") { origin_seen = 1; }
  }
  if (image.empty()) { return false; }
  // pgm は yaml からの相対で解決する
  if (image.front() != '/') {
    const std::size_t slash = yaml_path.find_last_of('/');
    if (slash != std::string::npos) { image = yaml_path.substr(0, slash + 1) + image; }
  }

  std::ifstream img(image, std::ios::binary);
  if (!img.is_open()) { return false; }
  std::string magic;
  img >> magic;
  if (magic != "P5" && magic != "P2") { return false; }
  auto next_int = [&img]() {
    int v = 0;
    while (img >> std::ws && img.peek() == '#') {
      std::string skip;
      std::getline(img, skip);
    }
    img >> v;
    return v;
  };
  const int w = next_int();
  const int h = next_int();
  const int maxval = next_int();
  if (w <= 0 || h <= 0 || maxval <= 0) { return false; }

  std::vector<unsigned char> px(static_cast<std::size_t>(w) * h);
  if (magic == "P5") {
    img.get();   // ヘッダ直後の空白1文字
    img.read(reinterpret_cast<char *>(px.data()), static_cast<std::streamsize>(px.size()));
    if (!img) { return false; }
  } else {
    for (std::size_t i = 0; i < px.size(); ++i) {
      int v = 0;
      if (!(img >> v)) { return false; }
      px[i] = static_cast<unsigned char>(v);
    }
  }

  // make_corridor.py と同じ判定にそろえる(白 > 200 を走行可能とみなす)
  const float kInf = 1e12f;
  auto edt = [&](bool free_is_source) {
    std::vector<float> f(px.size());
    for (std::size_t i = 0; i < px.size(); ++i) {
      const bool free_cell = px[i] > 200;
      f[i] = (free_cell == free_is_source) ? 0.0f : kInf;
    }
    {
      std::vector<float> col(h), dcol(h), zz(h + 1);
      std::vector<int> vv(h);
      for (int c = 0; c < w; ++c) {
        for (int r = 0; r < h; ++r) { col[r] = f[static_cast<std::size_t>(r) * w + c]; }
        edt1d(col, dcol, vv, zz, h);
        for (int r = 0; r < h; ++r) { f[static_cast<std::size_t>(r) * w + c] = dcol[r]; }
      }
    }
    {
      std::vector<float> row(w), drow(w), zz(w + 1);
      std::vector<int> vv(w);
      for (int r = 0; r < h; ++r) {
        for (int c = 0; c < w; ++c) { row[c] = f[static_cast<std::size_t>(r) * w + c]; }
        edt1d(row, drow, vv, zz, w);
        for (int c = 0; c < w; ++c) {
          f[static_cast<std::size_t>(r) * w + c] =
            std::sqrt(drow[c]) * static_cast<float>(res);
        }
      }
    }
    return f;
  };
  const std::vector<float> to_wall = edt(false);   // 壁までの距離
  const std::vector<float> to_free = edt(true);    // 走行可能までの距離
  std::vector<float> signed_dist(px.size());
  for (std::size_t i = 0; i < px.size(); ++i) {
    signed_dist[i] = px[i] > 200 ? to_wall[i] : -to_free[i];
  }
  dist_ = std::move(signed_dist);
  w_ = w;
  h_ = h;
  res_ = res;
  ox_ = ox;
  oy_ = oy;
  return true;
}

// 壁の外なら正(壁までの距離)、壁の中なら負(どれだけ食い込んでいるか)。
double ObstacleMap::clearance(double x, double y) const
{
  if (dist_.empty()) { return 1e3; }
  // make_corridor.py の grid_clear と同じ画素の取り方にそろえる
  const int c = static_cast<int>((x - ox_) / res_);
  const int r = static_cast<int>(h_ - (y - oy_) / res_);
  if (r < 0 || r >= h_ || c < 0 || c >= w_) { return 1e3; }
  return dist_[static_cast<std::size_t>(r) * w_ + c];
}

bool Corridor::locate(double px, double py, std::size_t & idx, double & lat) const
{
  if (!valid()) { return false; }
  const std::size_t n = x.size();
  std::size_t best = 0;
  double bd = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < n; ++i) {
    const double d = (x[i] - px) * (x[i] - px) + (y[i] - py) * (y[i] - py);
    if (d < bd) { bd = d; best = i; }
  }
  const std::size_t nx = (best + 1) % n;
  const std::size_t pv = (best + n - 1) % n;
  double tx = x[nx] - x[pv];
  double ty = y[nx] - y[pv];
  const double tl = std::hypot(tx, ty);
  if (tl < 1e-9) { return false; }
  tx /= tl;
  ty /= tl;
  idx = best;
  lat = (px - x[best]) * (-ty) + (py - y[best]) * (tx);
  return true;
}

namespace
{

// 他車を全長約2.6m×全幅約1.46mの外接円で安全側に近似する。

// 停止中の相手を包む円の半径。向きが不定なので最も遠い隅までを取る。
double stoppedCarRadius(const OpponentShape & sh)
{
  return std::hypot(std::max(sh.front, sh.rear), sh.half_width) + sh.margin;
}

// 点 (px,py) から、基準点 (cx,cy)・向き yaw の相手の車体矩形までの距離[m]。
// 矩形の中なら負を返す。
double distToOpponentBox(
  double px, double py, double cx, double cy, double yaw, const OpponentShape & sh,
  double extra_half_width)
{
  const double dx = px - cx;
  const double dy = py - cy;
  const double cs = std::cos(yaw), sn = std::sin(yaw);
  // 相手の車体座標系へ回す(前方が +lx)
  const double lx = dx * cs + dy * sn;
  const double ly = -dx * sn + dy * cs;
  // 基準点から前へ sh.front、後ろへ sh.rear の矩形。margin ぶん外へ広げる。
  const double ex = std::max(lx - (sh.front + sh.margin), (-(sh.rear + sh.margin)) - lx);
  const double ey = std::abs(ly) - (sh.half_width + sh.margin + extra_half_width);
  if (ex <= 0.0 && ey <= 0.0) { return std::max(ex, ey); }   // 矩形の内側
  return std::hypot(std::max(ex, 0.0), std::max(ey, 0.0));
}

// 相手1台に対する距離。停止中は円、走行中は向き付き矩形。
double distToOpponent(double px, double py, const CarObstacle & c, const OpponentShape & sh)
{
  if (c.moving) {
    return distToOpponentBox(px, py, c.x, c.y, c.yaw, sh, c.extra_half_width);
  }
  return std::hypot(px - c.x, py - c.y) - stoppedCarRadius(sh);
}

// 車体の外周点を返す。四隅だけだと、隅の間にある壁の出っ張りを跨いでしまう。
int bodyPoints(const VehicleParams & v, const Pose & p, double * bx, double * by)
{
  const double cs = std::cos(p.yaw);
  const double sn = std::sin(p.yaw);
  const double fx = v.front_overhang;
  const double rx = -v.rear_overhang;
  const double mx = (fx + rx) * 0.5;
  const double hw = v.half_width;
  const double dx[8] = {fx, fx, rx, rx, mx, mx, fx, rx};
  const double dy[8] = {hw, -hw, hw, -hw, hw, -hw, 0.0, 0.0};
  for (int i = 0; i < 8; ++i) {
    bx[i] = p.x + cs * dx[i] - sn * dy[i];
    by[i] = p.y + sn * dx[i] + cs * dy[i];
  }
  return 8;
}


double wallViolation(const ObstacleMap & map, const Corridor & c,
                     const VehicleParams & v, const Pose & p)
{
  double bx[8], by[8];
  const int n = bodyPoints(v, p, bx, by);
  double worst = 0.0;
  if (map.valid()) {
    for (int i = 0; i < n; ++i) {
      worst = std::max(worst, v.wall_margin - map.clearance(bx[i], by[i]));
    }
    return std::max(0.0, worst);
  }
  // 占有格子が読めなかったときだけコリドアで代用する。
  // lo/hi は「車体中心を置いてよい範囲」で、生成時にすでに 半幅+余裕 ぶん
  // 壁の内側へ寄せてある。四隅の判定ではその分を戻して実際の壁を見る。
  for (int i = 0; i < n; ++i) {
    std::size_t idx = 0;
    double lat = 0.0;
    if (!c.locate(bx[i], by[i], idx, lat)) { return 1e3; }
    const double over = std::max(lat - (c.hi[idx] + v.bound_inflate),
                                 (c.lo[idx] - v.bound_inflate) - lat);
    worst = std::max(worst, over);
  }
  return std::max(0.0, worst);
}

}  // namespace

// 他車までの余裕[m]。負なら重なっている。他車がいなければ大きな値。
//
// `carViolation` は 0 で初期化した「重なりの深さ」なので**離れていても 0**を返す。
// それを距離と取り違えて `cv > -0.25` と書いた結果、他車が1台もいなくても
// 条件が常に真になり、前進のたびに中断して無限に切り返すバグを出した。
// 実行中の監視にはこちらの符号付きの値を使うこと。
double carClearanceAt(const std::vector<CarObstacle> & cars,
                      const VehicleParams & veh, const Pose & p)
{
  constexpr double kFar = 1e3;
  if (cars.empty()) { return kFar; }
  double bx[8], by[8];
  const int n = bodyPoints(veh, p, bx, by);
  const OpponentShape sh{};
  double best = kFar;
  for (const auto & c : cars) {
    for (int i = 0; i < n; ++i) {
      best = std::min(best, distToOpponent(bx[i], by[i], c, sh));
    }
  }
  return best;
}

double carClearanceAt(const CarObstacle & car,
                      const VehicleParams & veh, const Pose & p)
{
  double bx[8], by[8];
  const int n = bodyPoints(veh, p, bx, by);
  const OpponentShape sh{};
  double best = 1e3;
  for (int i = 0; i < n; ++i) {
    best = std::min(best, distToOpponent(bx[i], by[i], car, sh));
  }
  return best;
}

// 他車との重なりの深さ[m]。0 以下なら当たっていない。
//
// 相手は向きを持たないV2X位置から外接円で近似し、自車外周点との距離を測る。
double carViolation(const std::vector<CarObstacle> & cars,
                    const VehicleParams & veh, const Pose & p)
{
  if (cars.empty()) { return 0.0; }
  double bx[8], by[8];
  const int n = bodyPoints(veh, p, bx, by);
  const OpponentShape sh{};
  double worst = 0.0;
  for (const auto & c : cars) {
    for (int i = 0; i < n; ++i) {
      // めり込みの深さ。停止中は円、走行中は向き付き矩形(carClearanceAt と同じ形)。
      worst = std::max(worst, -distToOpponent(bx[i], by[i], c, sh));
    }
  }
  return worst;
}

// 車体の前半分・後半分それぞれの最小余裕[m]を返す。
void wallClearanceSplit(
  const ObstacleMap & map, const VehicleParams & veh, const Pose & p,
  double & front_clear, double & rear_clear)
{
  front_clear = 1e3;
  rear_clear = 1e3;
  if (!map.valid()) { return; }
  const double cs = std::cos(p.yaw);
  const double sn = std::sin(p.yaw);
  const double length = veh.front_overhang + veh.rear_overhang;
  const double mid = (veh.front_overhang - veh.rear_overhang) * 0.5;   // 車体中央の縦座標
  auto sample = [&](double longitudinal, double lateral) {
    const double x = p.x + cs * longitudinal - sn * lateral;
    const double y = p.y + sn * longitudinal + cs * lateral;
    const double d = map.clearance(x, y);
    if (longitudinal >= mid) { front_clear = std::min(front_clear, d); }
    else                     { rear_clear = std::min(rear_clear, d); }
  };
  constexpr double kEdgeStep = 0.10;
  const int longitudinal_steps = std::max(1, static_cast<int>(std::ceil(length / kEdgeStep)));
  for (int i = 0; i <= longitudinal_steps; ++i) {
    const double x = -veh.rear_overhang + length * i / longitudinal_steps;
    sample(x, veh.half_width);
    sample(x, -veh.half_width);
  }
  const int lateral_steps = std::max(
    1, static_cast<int>(std::ceil(2.0 * veh.half_width / kEdgeStep)));
  for (int i = 0; i <= lateral_steps; ++i) {
    const double y = -veh.half_width + 2.0 * veh.half_width * i / lateral_steps;
    sample(veh.front_overhang, y);
    sample(-veh.rear_overhang, y);
  }
}

double wallClearanceAt(const ObstacleMap & map, const VehicleParams & veh, const Pose & p)
{
  if (!map.valid()) { return 1e3; }
  double worst = 1e3;
  const double cs = std::cos(p.yaw);
  const double sn = std::sin(p.yaw);
  auto sample = [&](double longitudinal, double lateral) {
    const double x = p.x + cs * longitudinal - sn * lateral;
    const double y = p.y + sn * longitudinal + cs * lateral;
    worst = std::min(worst, map.clearance(x, y));
  };
  // Corners and midpoints can straddle a narrow wall protrusion. Sample every
  // body edge at map-scale spacing so a swept-path certificate covers the body.
  constexpr double kEdgeStep = 0.10;
  const double length = veh.front_overhang + veh.rear_overhang;
  const int longitudinal_steps = std::max(1, static_cast<int>(std::ceil(length / kEdgeStep)));
  for (int i = 0; i <= longitudinal_steps; ++i) {
    const double x = -veh.rear_overhang + length * i / longitudinal_steps;
    sample(x, veh.half_width);
    sample(x, -veh.half_width);
  }
  const int lateral_steps = std::max(
    1, static_cast<int>(std::ceil(2.0 * veh.half_width / kEdgeStep)));
  for (int i = 0; i <= lateral_steps; ++i) {
    const double y = -veh.half_width + 2.0 * veh.half_width * i / lateral_steps;
    sample(veh.front_overhang, y);
    sample(-veh.rear_overhang, y);
  }
  return worst;
}

namespace
{

// 中心線の idx 地点が向いている方位
double trackYaw(const Corridor & c, std::size_t idx)
{
  const std::size_t n = c.x.size();
  const std::size_t j = (idx + 1) % n;
  return std::atan2(c.y[j] - c.y[idx], c.x[j] - c.x[idx]);
}

// 中心線に沿って start_idx から dist[m] 進んだ点の姿勢
bool poseAhead(const Corridor & c, std::size_t start_idx, double dist, Pose & out)
{
  const std::size_t n = c.x.size();
  double acc = 0.0;
  std::size_t i = start_idx;
  for (std::size_t k = 0; k < n; ++k) {
    const std::size_t j = (i + 1) % n;
    const double d = std::hypot(c.x[j] - c.x[i], c.y[j] - c.y[i]);
    if (acc + d >= dist) {
      const double t = (dist - acc) / std::max(d, 1e-9);
      out.x = c.x[i] + (c.x[j] - c.x[i]) * t;
      out.y = c.y[i] + (c.y[j] - c.y[i]) * t;
      out.yaw = std::atan2(c.y[j] - c.y[i], c.x[j] - c.x[i]);
      return true;
    }
    acc += d;
    i = j;
  }
  return false;
}

}  // namespace


// ===================================================================
// 目標指向の復帰計画。設計の意図はヘッダのコメントを参照。
// ===================================================================
Plan planToGoal(const Corridor & corridor, const ObstacleMap & obstacles,
                const VehicleParams & veh, const Pose & start,
                const std::vector<CarObstacle> & cars,
                const GoalPlanParams & prm,
                std::size_t * goal_idx_out)
{
  Plan out;
  if (!corridor.valid()) { return out; }
  std::size_t start_idx = 0;
  double start_lat = 0.0;
  if (!corridor.locate(start.x, start.y, start_idx, start_lat)) { return out; }

  // --- 目標の候補を作る。参照経路上の「少し先」を等間隔に並べる ---
  // 1点だけに絞ると、そこがヘアピンの途中で到達できない向きだった場合に
  // 「解なし」になる。範囲で置いて、到達できた最も手前を採る。
  struct Goal { Pose pose; std::size_t idx; };
  std::vector<Goal> goals;
  for (double d = prm.goal_ahead_min; d <= prm.goal_ahead_max + 1e-9;
       d += prm.goal_ahead_step)
  {
    Pose g;
    if (!poseAhead(corridor, start_idx, d, g)) { break; }
    std::size_t gi = 0; double glat = 0.0;
    if (!corridor.locate(g.x, g.y, gi, glat)) { continue; }
    goals.push_back({g, gi});
  }
  if (goals.empty()) { return out; }

  // --- 当たり判定の許容。開始時点で食い込んでいるならそのぶんは許す ---
  // 許さないと「いま壁に触れている」状態から一歩も動かせない。
  const double start_wall_vio = wallViolation(obstacles, corridor, veh, start);
  const double start_car_vio = carViolation(cars, veh, start);
  // 食い込み中は素片ごとの改善、脱出後は再侵入しないことを要求する。
  constexpr double kImprove = 0.01;   // 改善とみなす最小量[m]
  auto ok_step = [&](double vio, double prev_vio) {
    if (prev_vio > 1e-6) { return vio <= prev_vio - kImprove; }
    return vio <= 1e-3;
  };
  // --- 素片。舵は最大舵角を7段階 ---。
  const double steers[7] = {-veh.max_steer, -veh.max_steer * 0.8,
                            -veh.max_steer * 0.4, 0.0,
                            veh.max_steer * 0.4, veh.max_steer * 0.8,
                            veh.max_steer};

  struct Node
  {
    Pose pose;
    double g{0.0};        // ここまでの所要時間[s]
    double f{0.0};        // g + ヒューリスティック
    double v{0.0};        // その姿勢での進行方向の速さ[m/s]
    double run{0.0};      // 開始からの走行距離[m]
    double wall_vio{0.0}; // その姿勢での壁への食い込み[m]。悪化の判定に使う
    double car_vio{0.0};  // 同 他車との重なり[m]
    std::vector<double> car_vio_by_id;  // aggregate min must not hide a different car
    int dir{0};           // 直前の素片の向き +1 前進 / -1 後退 / 0 開始
    int switches{0};      // ここまでの切り返し回数
    int parent{-1};
    double steer{0.0};    // ここへ来た素片の舵角
  };

  // ヒューリスティック: 目標までの直線距離を上限速度で割った時間。
  // どの動作もこれより速くは進めないので許容的(A* の最適性が保てる)。
  auto heuristic = [&](const Pose & p) {
    double best = 1e18;
    for (const auto & gl : goals) {
      best = std::min(best, std::hypot(gl.pose.x - p.x, gl.pose.y - p.y));
    }
    return best / std::max(prm.v_fwd_max, 0.1);
  };
  auto reached = [&](const Pose & p, std::size_t & gi) {
    for (const auto & gl : goals) {
      if (std::hypot(gl.pose.x - p.x, gl.pose.y - p.y) <= prm.pos_tol &&
          std::abs(wrap(p.yaw - gl.pose.yaw)) <= prm.yaw_tol)
      {
        gi = gl.idx;
        return true;
      }
    }
    return false;
  };
  auto key = [&](const Pose & p, int dir, double v) {
    const long ix = static_cast<long>(std::floor(p.x / prm.grid_xy));
    const long iy = static_cast<long>(std::floor(p.y / prm.grid_xy));
    long iyaw = static_cast<long>(std::floor(wrap(p.yaw) / prm.grid_yaw));
    iyaw += 64;   // 方位の索引を非負に寄せる
    const long iv = static_cast<long>(std::floor(v / std::max(prm.grid_v, 0.1)));
    return (((ix * 100003L + iy) * 251L + iyaw) * 3L + (dir + 1)) * 61L + iv;
  };

  std::vector<Node> nodes;
  nodes.reserve(4096);
  Node s0;
  s0.pose = start;
  s0.wall_vio = start_wall_vio;
  s0.car_vio = start_car_vio;
  for (const auto & car : cars) {
    s0.car_vio_by_id.push_back(
      std::max(0.0, -carClearanceAt(car, veh, start)));
  }
  s0.g = 0.0;
  s0.v = 0.0;          // 復帰に入る時点では止まっている前提
  s0.f = heuristic(start);
  nodes.push_back(s0);

  // f が小さい順に取り出す。std::priority_queue は最大取り出しなので符号を反転。
  using Item = std::pair<double, int>;
  std::priority_queue<Item, std::vector<Item>, std::greater<Item>> open;
  open.push({s0.f, 0});
  std::map<long, double> best_cost;
  best_cost[key(start, 0, 0.0)] = 0.0;

  int expanded = 0;
  int goal_node = -1;
  std::size_t goal_idx = 0;
  double best_total = 1e18;
  while (!open.empty() && expanded < prm.max_expand) {
    const auto top = open.top();
    open.pop();
    const int ci = top.second;
    if (top.first > nodes[ci].f + 1e-9) { continue; }   // 古い項目
    ++expanded;
    {
      std::size_t gi = 0;
      if (nodes[ci].dir > 0 && reached(nodes[ci].pose, gi)) {
        const double total = nodes[ci].g +
                             tailTime(corridor, start_idx, gi, nodes[ci].v, prm);
        if (total < best_total) {
          best_total = total;
          goal_node = ci;
          goal_idx = gi;
        }
        // 目標に着いた枝はそこで打ち切る(その先を伸ばす意味がない)。
        continue;
      }
    }
    for (int dir : {+1, -1}) {
      if (prm.first_phase != 0 && nodes[ci].dir == 0 &&
          dir != (prm.first_phase > 0 ? +1 : -1)) { continue; }
      const int sw = nodes[ci].switches +
                     ((nodes[ci].dir != 0 && dir != nodes[ci].dir) ? 1 : 0);
      if (sw > prm.max_switch) { continue; }
      for (double st : steers) {
        // 素片を細かく積分して当たり判定する。刻みは plan() と同じ kStep。
        Pose p = nodes[ci].pose;
        bool ok = true;
        const int sub = std::max(1, static_cast<int>(std::ceil(prm.step / kStep)));
        const double ds = (dir > 0 ? 1.0 : -1.0) * (prm.step / sub);
        // 直前の姿勢での食い込み量を基準に、悪化しないことを求める。
        double run = nodes[ci].run;
        double prev_wall = nodes[ci].wall_vio;
        double prev_car = nodes[ci].car_vio;
        std::vector<double> prev_car_by_id = nodes[ci].car_vio_by_id;
        for (int k = 0; k < sub; ++k) {
          p = advance(p, ds, st, veh.wheel_base);
          run += std::abs(ds);
          const double wv = wallViolation(obstacles, corridor, veh, p);
          const double cv = carViolation(cars, veh, p);
          // 食い込んでいる間は改善する向きだけを通す。
          if (!ok_step(wv, prev_wall)) { ok = false; break; }
          if (!ok_step(cv, prev_car)) { ok = false; break; }
          for (std::size_t car_i = 0; car_i < cars.size(); ++car_i) {
            const double car_vio =
              std::max(0.0, -carClearanceAt(cars[car_i], veh, p));
            if (!ok_step(car_vio, prev_car_by_id[car_i])) {
              ok = false;
              break;
            }
            prev_car_by_id[car_i] = car_vio;
          }
          if (!ok) { break; }
          prev_wall = wv;
          prev_car = cv;
        }
        if (!ok) { continue; }
        // --- この素片に要る時間を見積る ---
        // 向きが変わるなら、まず止まりきる時間 + ギアが入る時間を払う。
        double dt = 0.0;
        double v0 = nodes[ci].v;
        if (sw > nodes[ci].switches || nodes[ci].dir == 0) {
          dt += v0 / prm.a_brake;      // 止まりきる
          dt += prm.t_gear;            // ギアが入って動き出すまで
          v0 = 0.0;
        }
        // 素片の旋回半径から、横加速度で許される速度の上限を出す。
        // 舵を大きく切る素片は速度が出せない = 時間がかかる、が自然に入る。
        const double r_prim = (std::abs(st) > 1e-6)
                                ? std::abs(veh.wheel_base / std::tan(st)) : 1e9;
        const double v_cap = (dir > 0)
          ? std::min(prm.v_fwd_max, std::sqrt(prm.ay_max * r_prim))
          : prm.v_rev_max;
        double v1;
        if (v0 < v_cap) {
          v1 = std::min(v_cap, std::sqrt(v0 * v0 + 2.0 * prm.a_accel * prm.step));
        } else {
          v1 = std::max(v_cap,
                        std::sqrt(std::max(v0 * v0 - 2.0 * prm.a_brake * prm.step, 0.01)));
        }
        dt += 2.0 * prm.step / std::max(v0 + v1, 0.2);
        const double ng = nodes[ci].g + dt;
        const long k2 = key(p, dir, v1);
        const auto it = best_cost.find(k2);
        if (it != best_cost.end() && it->second <= ng + 1e-9) { continue; }
        best_cost[k2] = ng;
        Node nd;
        nd.pose = p;
        nd.g = ng;
        nd.v = v1;
        nd.run = run;
        nd.wall_vio = prev_wall;
        nd.car_vio = prev_car;
        nd.car_vio_by_id = std::move(prev_car_by_id);
        nd.f = ng + heuristic(p);
        nd.dir = dir;
        nd.switches = sw;
        nd.parent = ci;
        nd.steer = st;
        nodes.push_back(nd);
        open.push({nd.f, static_cast<int>(nodes.size()) - 1});
      }
    }
  }
  if (goal_node < 0) { return out; }

  // --- 経路を復元する ---
  std::vector<int> chain;
  for (int i = goal_node; i >= 0; i = nodes[i].parent) { chain.push_back(i); }
  std::reverse(chain.begin(), chain.end());

  out.path.clear();
  out.phases.clear();
  for (std::size_t k = 0; k < chain.size(); ++k) { out.path.push_back(nodes[chain[k]].pose); }
  // 素片を向きでまとめて区間にする。直接制御へ落ちたときのために
  // 各区間の舵角は「その区間の最初の素片の舵角」を代表値にする
  // (追従は経路で行うので、区間はあくまで退避用)。
  {
    int cur_dir = 0;
    double len = 0.0;
    double steer0 = 0.0;
    std::size_t phase_begin = 0;
    for (std::size_t k = 1; k < chain.size(); ++k) {
      const Node & nd = nodes[chain[k]];
      if (nd.dir != cur_dir) {
        if (cur_dir != 0) {
          out.phases.push_back({cur_dir > 0, steer0, len, phase_begin, k});
        }
        phase_begin = k - 1;
        cur_dir = nd.dir;
        steer0 = nd.steer;
        len = 0.0;
      }
      len += prm.step;
    }
    if (cur_dir != 0) {
      out.phases.push_back({cur_dir > 0, steer0, len, phase_begin, chain.size()});
    }
  }
  // 最初の方向転換までの点数。後退用と合流用で経路を分けるのに使う。
  out.rev_points = 0;
  if (!out.phases.empty() && !out.phases.front().forward) {
    std::size_t cnt = 1;   // 開始姿勢を含む
    for (std::size_t k = 1; k < chain.size(); ++k) {
      if (nodes[chain[k]].dir > 0) { break; }
      ++cnt;
    }
    out.rev_points = cnt;
  }
  // 前進区間で見込む最小の余裕。追従中の中断閾値に使う(plan() と同じ意味)。
  {
    double min_wall = 1e9, min_car = 1e9;
    for (std::size_t k = 1; k < chain.size(); ++k) {
      if (nodes[chain[k]].dir <= 0) { continue; }
      min_wall = std::min(min_wall, wallClearanceAt(obstacles, veh, nodes[chain[k]].pose));
      min_car = std::min(min_car, carClearanceAt(cars, veh, nodes[chain[k]].pose));
    }
    out.min_wall_clear = min_wall;
    out.min_car_clear = min_car;
  }
  out.cost = nodes[goal_node].g;
  if (!out.path.empty() &&
      wallViolation(obstacles, corridor, veh, out.path.back()) > 1e-3)
  {
    out.phases.clear();
    out.path.clear();
    out.valid = false;
    return out;
  }
  out.valid = true;
  if (goal_idx_out) { *goal_idx_out = goal_idx; }
  return out;
}

Plan plan(const Corridor & corridor, const ObstacleMap & obstacles,
          const VehicleParams & veh, const Pose & start,
          double ahead_min, double ahead_max, int first_phase, double min_gain,
          double min_escape, double min_reverse, double max_reverse,
          const std::vector<CarObstacle> & cars,
          double req_wall_clear, double req_car_clear, bool best_effort)
{
  Plan best;
  if (!corridor.valid()) { return best; }

  std::size_t start_idx = 0;
  double start_lat = 0.0;
  if (!corridor.locate(start.x, start.y, start_idx, start_lat)) { return best; }

  {
    Pose probe;
    if (!poseAhead(corridor, start_idx, ahead_min, probe)) { return best; }
  }
  const std::size_t n_pts = corridor.x.size();
  // 点間隔[m]。点数で数えている量を距離に直すのに使う。
  double mean_spacing = 1.0;
  {
    double total = 0.0;
    for (std::size_t i = 0; i < n_pts; ++i) {
      const std::size_t j = (i + 1) % n_pts;
      total += std::hypot(corridor.x[j] - corridor.x[i], corridor.y[j] - corridor.y[i]);
    }
    mean_spacing = total / static_cast<double>(n_pts);
  }

  // 舵角の候補。最大舵角を5段階に割る。0(直進)も入れる。
  const double st[5] = {-veh.max_steer, -veh.max_steer * 0.5, 0.0,
                        veh.max_steer * 0.5, veh.max_steer};
  // 後退距離の候補。0 は「後退せずに前進だけで戻る」場合。
  const double rev_len[16] = {0.0, 0.25, 0.4, 0.55, 0.75, 1.0, 1.5, 2.0,
                              2.5, 3.0, 3.5, 4.0, 5.0, 6.0, 7.0, 8.0};
  // 前進距離の候補。
  // A contact chain sometimes has no safe forward continuation yet: the rear
  // vehicle must first reverse away so the front vehicle can leave the wall.
  // Requiring every candidate to end with >=1.5m forward made that useful
  // partial move impossible and both controllers safe-stopped for 30 seconds.
  const double fwd_len[8] = {0.0, 1.5, 3.0, 4.5, 6.0, 8.0, 10.0, 12.0};

  // 出発時点の食い込み量。これより悪化させない範囲で動かす。
  // わずかな増加は許す(切り返しで一時的に角が出るのは避けられない)。
  const double start_violation = wallViolation(obstacles, corridor, veh, start);
  const double allow = start_violation + 0.15;

  const bool touching = start_violation > 1e-6;
  const double kEarlyLen = 1.5;

  // 他車についても壁とまったく同じ扱いにする。
  const double start_car_violation = carViolation(cars, veh, start);
  const double car_allow = start_car_violation + 0.05;
  const bool car_touching = start_car_violation > 1e-6;

  for (double rl : rev_len) {
    // 最初の区間の向きが縛られているなら、それに合わない候補は作らない
    if (first_phase < 0 && rl < 1e-6) { continue; }   // 後退から始めろ
    if (first_phase > 0 && rl > 1e-6) { continue; }   // 前進から始めろ
    // 地図の上では動けるはずなのに動けなかったぶん、後退を伸ばす
    if (min_reverse > 0.0 && rl < min_reverse - 1e-6) { continue; }
    // 後ろに他車がいるなら、空いている距離までしか下がらない
    if (rl > max_reverse + 1e-6) { continue; }
    for (double rs : st) {
      if (rl < 1e-6 && std::abs(rs) > 1e-6) { continue; }   // 後退0なら舵は無関係
      // 後退区間を積分
      Pose p = start;
      std::vector<Pose> path;
      path.push_back(p);
      bool ok = true;
      const int rn = static_cast<int>(std::ceil(rl / kStep));
      for (int i = 0; i < rn; ++i) {
        p = advance(p, -kStep, rs, veh.wheel_base);
        path.push_back(p);
        const double vio = wallViolation(obstacles, corridor, veh, p);
        if (vio > allow) { ok = false; break; }
        // 他車も壁と同じく「今より悪くしなければ可」。
        // 既に重なっている状態から抜け出せるようにするため。
        const double cvio = carViolation(cars, veh, p);
        if (cvio > car_allow) { ok = false; break; }
        // 壁に触れているなら、出だしで食い込みを増やしてはいけない
        if (touching && (i + 1) * kStep <= kEarlyLen && vio > start_violation + 1e-6) {
          ok = false; break;
        }
        // 相手に触れているなら、出だしは**重なりが減っていること**を要求する。
        // 減る方向にしか動かないので、押し付けたまま回る経路は弾ける。
        if (car_touching && (i + 1) * kStep <= kEarlyLen &&
            cvio > start_car_violation + 1e-6) {
          ok = false; break;
        }
      }
      if (!ok) { continue; }
      const Pose after_rev = p;
      const std::size_t rev_pts = path.size();

      for (double fs : st) {
        // 前進は最長まで一度だけ積分して、途中を各候補の終端として使う。
        Pose q = after_rev;
        std::vector<Pose> fpath;
        const double fmax = fwd_len[7];
        const int fnmax = static_cast<int>(std::ceil(fmax / kStep));
        // 後退から始まる計画なら前進時には既に壁から離れている。
        // 前進から始まる計画(rl==0)のときだけ、出だしの条件を課す。
        const bool fwd_is_first = (rl < 1e-6);
        // 後退で相手から離れられたなら、前進でまた重なりに戻ることは許さない。
        // car_allow(開始時の重なり + 0.05)をそのまま使うと、せっかく下がったのに
        // 元の食い込みまで突っ込み直す経路が通ってしまう。
        // 後退終了時点の重なりを基準に張り直す。
        // clearance >= req_car_clear <=> violation <= -req_car_clear
        // req_*_clear が 0 のとき(緩い段)は締めない。ここで無条件に clamp すると
        // 「開始時に既に重なっているなら +0.05 まで許す」という逃げ道を潰してしまい、
        // 相手が前方 2.6m 以内にいるだけで経路が一切見つからなくなる
        // (過去に実際に出したバグ。復帰が20秒以上働かなかった)。
        double fwd_car_allow =
          std::min(car_allow, carViolation(cars, veh, after_rev) + 0.05);
        if (req_car_clear > 0.0) { fwd_car_allow = std::min(fwd_car_allow, -req_car_clear); }
        // clearance >= req_wall_clear <=> violation <= wall_margin - req_wall_clear
        double fwd_wall_allow = allow;
        if (req_wall_clear > 0.0) {
          fwd_wall_allow = std::min(allow, veh.wall_margin - req_wall_clear);
        }
        for (int i = 0; i < fnmax; ++i) {
          q = advance(q, kStep, fs, veh.wheel_base);
          const double vio = wallViolation(obstacles, corridor, veh, q);
          if (vio > fwd_wall_allow) { break; }
          const double cvio = carViolation(cars, veh, q);
          if (cvio > fwd_car_allow) { break; }
          if (touching && fwd_is_first && (i + 1) * kStep <= kEarlyLen &&
              vio > start_violation + 1e-6) {
            break;
          }
          if (car_touching && fwd_is_first && (i + 1) * kStep <= kEarlyLen &&
              cvio > start_car_violation + 1e-6) {
            break;
          }
          fpath.push_back(q);
        }
        for (double fl : fwd_len) {
          // 狙う範囲より先まで走る案は見ない(復帰は最短で済ませる)
          if (fl > ahead_max + 1e-6) { continue; }
          const int fn = static_cast<int>(std::ceil(fl / kStep));
          if (static_cast<int>(fpath.size()) < fn) { continue; }   // 途中で壁
          if (rl < 1e-6 && fl < 1e-6) { continue; }
          const Pose e = fn > 0 ? fpath[fn - 1] : after_rev;

          // 終端がどれだけ経路へ戻れているかで評価する。
          std::size_t ei = 0;
          double elat = 0.0;
          if (!corridor.locate(e.x, e.y, ei, elat)) { continue; }
          // 終端位置における中心線の向きとのずれ。
          // これが小さくないと、通常制御に返した瞬間また壁を向いて走り出す。
          const double yaw_err = std::abs(wrap(e.yaw - trackYaw(corridor, ei)));
          // コース方向にどれだけ進めたか。後退しただけで終わる案を弾く。
          // 点数ではなく距離[m]で数える。点数だと、走行ラインの点を増やしただけで
          // 同じ後退量が2倍に罰せられてしまう(121点 -> 242点で実際に起きる)。
          double advance_idx = static_cast<double>(ei) - static_cast<double>(start_idx);
          if (advance_idx < -static_cast<double>(n_pts) / 2.0) { advance_idx += n_pts; }
          if (advance_idx > static_cast<double>(n_pts) / 2.0) { advance_idx -= n_pts; }
          const double back_penalty =
            advance_idx < 0.0 ? -advance_idx * mean_spacing : 0.0;
          if (std::hypot(e.x - start.x, e.y - start.y) < min_escape) { continue; }
          const double end_violation = wallViolation(obstacles, corridor, veh, e);
          // 「今より改善する計画だけ」を求められている場合(やり直し時)は、
          // 改善しない候補を捨てる。捨てないと同じ計画を出し続けて時間を失う。
          if (min_gain > 0.0 && end_violation > start_violation - min_gain) { continue; }

          // 最優先は「壁から離れること」、次が「コースの向きに戻ること」。
          // 以前は横ずれを重く見ていたため、96 度傾いたまま横ずれだけ縮める
          // 1.5m の前進が最良と判定され、向きが直らないまま25秒を使い切った。
          // 向きが直っていなければ通常制御に返しても即座に再スタックする。
          // 後退は割り増ししない。必要なだけ下がるのが正しいため。
          // 「十分そろっていれば、それ以上の改善より短さを優先する」。
          // 距離のペナルティが小さすぎたころは、方位差がわずかに良くなるだけで
          // 6m 下がる案が勝ち、そのぶん時間を捨てていた。
          // 通常制御は多少の方位差なら自分で吸収できるので、
          // 8.6deg・横 0.5m まで入ったら同点として扱う。
          const double yaw_over = std::max(0.0, yaw_err - kYawGoodEnough);
          const double lat_over = std::max(0.0, std::abs(elat) - kLatGoodEnough);
          const double cost = end_violation * 50.0 + yaw_over * 12.0 +
                              lat_over * 1.0 + back_penalty * 1.5 +
                              rl * 0.35 + fl * 0.15;
          if (best.valid && cost >= best.cost) { continue; }

          best.phases.clear();
          if (rl > 1e-6) { best.phases.push_back({false, rs, rl, 0, rev_pts}); }
          const std::size_t fwd_begin = rev_pts > 0 ? rev_pts - 1 : 0;
          if (fl > 1e-6) {
            best.phases.push_back({true, fs, fl, fwd_begin, rev_pts + fn});
          }
          best.path.assign(path.begin(), path.begin() + rev_pts);
          best.path.insert(best.path.end(), fpath.begin(), fpath.begin() + fn);
          best.rev_points = rev_pts;
          best.cost = cost;
          best.valid = true;
          // 採用した前進区間の最小余裕を記録する。best を更新するときだけ計算する
          // (毎候補で計算すると重い)。追従中の中断閾値をこれに合わせるために使う。
          {
            double min_wall = 1e9;
            double min_car = 1e9;
            for (int k = 0; k < fn; ++k) {
              min_wall = std::min(min_wall, wallClearanceAt(obstacles, veh, fpath[k]));
              min_car = std::min(min_car, carClearanceAt(cars, veh, fpath[k]));
            }
            best.min_wall_clear = min_wall;
            best.min_car_clear = min_car;
          }
        }
      }
    }
  }

  // ここまでの探索で「どこにも当たらない・改善する」案が。
  if (best_effort && best.valid) {
    const double start_wall_clearance = wallClearanceAt(obstacles, veh, start);
    const double wall_floor = start_wall_clearance < 0.0
      ? start_wall_clearance - 0.02 : 0.0;
    bool fully_clear = wallClearanceAt(obstacles, veh, best.path.back()) >= 0.0;
    for (const auto & pose : best.path) {
      fully_clear = fully_clear && wallClearanceAt(obstacles, veh, pose) >= wall_floor;
    }
    for (const auto & car : cars) {
      const double start_clearance = carClearanceAt(car, veh, start);
      const double floor = start_clearance < 0.0 ? start_clearance - 0.02 : 0.0;
      fully_clear = fully_clear && carClearanceAt(car, veh, best.path.back()) >= 0.0;
      for (const auto & pose : best.path) {
        fully_clear = fully_clear && carClearanceAt(car, veh, pose) >= floor;
      }
    }
    if (!fully_clear) { best = Plan{}; }
  }
  if (best_effort && !best.valid) {
    // 開始時点の方位差。総当りで「向きが直る案」を選ぶための基準。
  std::size_t syi = 0; double syl = 0.0;
  const bool start_yaw_ok = corridor.locate(start.x, start.y, syi, syl);
  const double start_yaw_err =
    start_yaw_ok ? std::abs(wrap(start.yaw - trackYaw(corridor, syi))) : 0.0;
  double best_score = -1e18;
    double best_len = 1e18;
    double best_terminal_penetration = 1e18;
    const double start_wall_clearance = wallClearanceAt(obstacles, veh, start);
    std::vector<double> start_car_clearances;
    start_car_clearances.reserve(cars.size());
    double start_car_penetration = 0.0;
    for (const auto & car : cars) {
      const double clearance = carClearanceAt(car, veh, start);
      start_car_clearances.push_back(clearance);
      start_car_penetration += std::max(0.0, -clearance);
    }

    for (double rl : rev_len) {
      if (first_phase < 0 && rl < 1e-6) { continue; }
      if (first_phase > 0 && rl > 1e-6) { continue; }
      if (min_reverse > 0.0 && rl < min_reverse - 1e-6) { continue; }
      if (rl > max_reverse + 1e-6) { continue; }
      for (double rs : st) {
        if (rl < 1e-6 && std::abs(rs) > 1e-6) { continue; }
        Pose p = start;
        std::vector<Pose> path;
        path.push_back(p);
        const int rn = static_cast<int>(std::ceil(rl / kStep));
        for (int i = 0; i < rn; ++i) {
          p = advance(p, -kStep, rs, veh.wheel_base);
          path.push_back(p);
        }
        const Pose after_rev = p;
        const std::size_t rev_pts = path.size();

        for (double fs : st) {
          Pose q = after_rev;
          std::vector<Pose> fpath;
          const double fmax = fwd_len[7];
          const int fnmax = static_cast<int>(std::ceil(fmax / kStep));
          for (int i = 0; i < fnmax; ++i) {
            q = advance(q, kStep, fs, veh.wheel_base);
            fpath.push_back(q);
          }
          for (double fl : fwd_len) {
            if (fl > ahead_max + 1e-6) { continue; }
            const int fn = static_cast<int>(std::ceil(fl / kStep));
            if (static_cast<int>(fpath.size()) < fn) { continue; }
            if (rl < 1e-6 && fl < 1e-6) { continue; }

            double score = 1e18;
            double min_wall_clearance = 1e18;
            std::vector<double> min_car_clearance(cars.size(), 1e18);
            for (std::size_t k = 0; k < rev_pts; ++k) {
              const double wall = wallClearanceAt(obstacles, veh, path[k]);
              score = std::min(score, wall);
              min_wall_clearance = std::min(min_wall_clearance, wall);
              score = std::min(score, carClearanceAt(cars, veh, path[k]));
              for (std::size_t car_i = 0; car_i < cars.size(); ++car_i) {
                min_car_clearance[car_i] = std::min(
                  min_car_clearance[car_i], carClearanceAt(cars[car_i], veh, path[k]));
              }
            }
            for (int k = 0; k < fn; ++k) {
              const double wall = wallClearanceAt(obstacles, veh, fpath[k]);
              score = std::min(score, wall);
              min_wall_clearance = std::min(min_wall_clearance, wall);
              score = std::min(score, carClearanceAt(cars, veh, fpath[k]));
              for (std::size_t car_i = 0; car_i < cars.size(); ++car_i) {
                min_car_clearance[car_i] = std::min(
                  min_car_clearance[car_i], carClearanceAt(cars[car_i], veh, fpath[k]));
              }
            }
            const Pose & ep = fn > 0 ? fpath[fn - 1] : after_rev;
            const double terminal_wall = wallClearanceAt(obstacles, veh, ep);
            const double wall_floor = start_wall_clearance < 0.0
              ? start_wall_clearance - 0.02 : 0.0;
            if (min_wall_clearance < wall_floor || terminal_wall < 0.0) { continue; }
            bool per_car_safe = true;
            double terminal_car_penetration = 0.0;
            for (std::size_t car_i = 0; car_i < cars.size(); ++car_i) {
              const double floor = start_car_clearances[car_i] < 0.0
                ? start_car_clearances[car_i] - 0.02 : 0.0;
              if (min_car_clearance[car_i] < floor) {
                per_car_safe = false;
                break;
              }
              terminal_car_penetration += std::max(
                0.0, -carClearanceAt(cars[car_i], veh, ep));
            }
            if (!per_car_safe) { continue; }
            if (start_car_penetration > 0.0 && terminal_car_penetration > 0.0 &&
                terminal_car_penetration > start_car_penetration - 0.10)
            {
              continue;
            }
            std::size_t eyi = 0; double eyl = 0.0;
            double yaw_gain = 0.0;
            if (corridor.locate(ep.x, ep.y, eyi, eyl) && start_yaw_ok) {
              const double end_yaw_err = std::abs(wrap(ep.yaw - trackYaw(corridor, eyi)));
              yaw_gain = std::max(0.0, start_yaw_err - end_yaw_err);
            }
            const double obj = score + kYawGainWeight * yaw_gain;
            const double length = rl + fl;
            const bool better =
              terminal_car_penetration < best_terminal_penetration - 1e-9 ||
              (std::abs(terminal_car_penetration - best_terminal_penetration) <= 1e-9 &&
               (obj > best_score + 1e-9 ||
                (std::abs(obj - best_score) <= 1e-9 && length < best_len)));
            if (!better) { continue; }
            score = obj;

            best_score = score;
            best_len = length;
            best_terminal_penetration = terminal_car_penetration;
            best.phases.clear();
            if (rl > 1e-6) { best.phases.push_back({false, rs, rl, 0, rev_pts}); }
            const std::size_t fwd_begin = rev_pts > 0 ? rev_pts - 1 : 0;
            if (fl > 1e-6) {
              best.phases.push_back({true, fs, fl, fwd_begin, rev_pts + fn});
            }
            best.path.assign(path.begin(), path.begin() + rev_pts);
            best.path.insert(best.path.end(), fpath.begin(), fpath.begin() + fn);
            best.rev_points = rev_pts;
            best.cost = -score;
            best.valid = true;
            {
              double min_wall = 1e9;
              double min_car = 1e9;
              for (int k = 0; k < fn; ++k) {
                min_wall = std::min(min_wall, wallClearanceAt(obstacles, veh, fpath[k]));
                min_car = std::min(min_car, carClearanceAt(cars, veh, fpath[k]));
              }
              best.min_wall_clear = min_wall;
              best.min_car_clear = min_car;
            }
          }
        }
      }
    }
  }

  return best;
}

}  // namespace recovery
