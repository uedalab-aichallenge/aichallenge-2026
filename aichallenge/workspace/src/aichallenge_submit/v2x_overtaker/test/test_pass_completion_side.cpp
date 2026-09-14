#include "v2x_overtaker/pass_completion_side.hpp"

#include <cassert>
#include <cstdio>

using v2x_overtaker::PassSideInput;
using v2x_overtaker::PassSideSample;
using v2x_overtaker::passCompletionSide;

namespace
{
// のコリドア(直線 idx230->25)を模した並び。
PassSideInput straightWithFlip()
{
  PassSideInput in;
  in.pass_dist = 60.0;
  in.need_sep = 1.66;
  in.window_ratio = 0.5;          // 後半 30〜60m で判定する
  // s, lo, hi, opp_lat, known
  const double rows[][4] = {
    { 0.0, -0.35, 4.30, 0.0},     // 入口: 左が広い
    {10.0, -1.90, 3.15, 0.0},
    {20.0, -3.15, 1.70, 0.0},     // 反転点
    {35.0, -3.85, 0.85, 0.0},     // 後半: 右が広い
    {45.0, -4.35, 0.40, 0.0},
    {55.0, -3.60, 1.00, 0.0},
  };
  for (const auto & r : rows) {
    PassSideSample s;
    s.s = r[0]; s.lo = r[1]; s.hi = r[2]; s.opp_lat = r[3]; s.opp_known = true;
    in.samples.push_back(s);
  }
  return in;
}
}  // namespace

int main()
{
  // 1. 本命: 入口は左が広いが、抜き切る後半は右が広い -> 右を選ぶ。
  {
    const auto r = passCompletionSide(straightWithFlip());
    assert(r.valid);
    assert(r.side == -1);            // 右
    assert(r.room_right > r.room_left);
  }

  // 2. 窓を入口側にずらすと左を選ぶ。
  //    「どこで抜き切るかで側が変わる」ことの確認。
  {
    auto in = straightWithFlip();
    in.pass_dist = 12.0;             // すぐ前で抜き切る想定
    in.window_ratio = 0.0;           // 0〜12m を見る
    const auto r = passCompletionSide(in);
    assert(r.valid);
    assert(r.side == +1);            // 左
  }

  // 3. 相手の横位置が分かっていない地点は、可動域そのもので評価する。
  {
    auto in = straightWithFlip();
    for (auto & s : in.samples) { s.opp_known = false; }
    const auto r = passCompletionSide(in);
    assert(r.valid);
    assert(r.side == -1);            // 後半は右が広いので変わらない
  }

  // 4. 相手が右へ寄っていれば、後半でも左を選ぶ。
  //    「場所」ではなく「相手の予測位置込み」で決めていることの確認。
  {
    auto in = straightWithFlip();
    for (auto & s : in.samples) { s.opp_lat = -2.6; }   // 相手が右端寄り
    const auto r = passCompletionSide(in);
    assert(r.valid);
    assert(r.side == +1);            // 左
  }

  {
    PassSideInput in;
    in.pass_dist = 20.0; in.need_sep = 1.66; in.window_ratio = 0.0;
    PassSideSample s;
    s.s = 10.0; s.lo = -0.5; s.hi = 0.5; s.opp_lat = 0.0; s.opp_known = true;
    in.samples.push_back(s);
    const auto r = passCompletionSide(in);
    assert(r.valid);
    assert(r.side == 0);
  }

  // 6. 窓に入る標本が無ければ「決められない」を返す(valid=false)。
  {
    PassSideInput in;
    in.pass_dist = 100.0; in.window_ratio = 0.9;
    PassSideSample s;
    s.s = 1.0; s.lo = -3.0; s.hi = 3.0; s.opp_known = false;
    in.samples.push_back(s);
    const auto r = passCompletionSide(in);
    assert(!r.valid);
    assert(r.side == 0);
  }

  // 7. 壊れた入力では決めない(fail-closed)。
  {
    PassSideInput in;
    in.pass_dist = -1.0;
    assert(passCompletionSide(in).side == 0);
    PassSideInput in2;
    in2.pass_dist = 10.0;   // samples が空
    assert(passCompletionSide(in2).side == 0);
  }

  // 8. 僅差では側を反転させない(ヒステリシス)。
  //    左右の空きがほぼ同じなら、毎周期ばたつかせないこと。
  {
    PassSideInput in;
    in.pass_dist = 20.0; in.need_sep = 1.0; in.window_ratio = 0.0;
    PassSideSample s;
    s.s = 10.0; s.lo = -3.0; s.hi = 3.05; s.opp_lat = 0.0; s.opp_known = true;
    in.samples.push_back(s);
    const auto r = passCompletionSide(in);
    // 左 3.05-1.0=2.05 / 右 -1.0-(-3.0)=2.0 で差 0.05 < 0.20 なので
    // ヒステリシス内。side は決まるが、差で選んだのではないことを確認する。
    assert(r.valid);
    assert(std::abs(r.room_left - r.room_right) < 0.20);
  }

  std::printf("test_pass_completion_side: all passed\n");
  
  // ------------------------------------------------------------------。
  {
    // のコリドアに近い並び。
    PassSideInput in;
    in.pass_dist = 10.0;
    in.need_sep = 1.66;
    in.window_ratio = 0.0;        // 窓の全域で見る
    in.plan_mode = true;
    in.y_now = -0.50;             // いま少し右にいる
    in.v_ego = 8.0;
    in.lat_rate = 1.2;
    in.room_margin = 0.0;
    const double rows[][4] = {
      { 2.0, -0.40, 4.30, 0.0},
      { 6.0, -3.20, 1.60, 0.0},   // 右 = -1.66 .. -3.20 -> 1.54m
      {11.0, -3.20, 1.60, 0.0},
      {16.0, -3.20, 1.60, 0.0},
      {21.0, -1.00, 1.60, 0.0},   // ここは右が閉じる
      {26.0, -1.00, 4.60, 0.0},   // 遠く: 左 = 1.66 .. 4.60 -> 2.94m
      {31.0, -1.00, 4.60, 0.0},
      {36.0, -1.00, 4.60, 0.0},
      {41.0, -1.00, 4.60, 0.0},
    };
    for (const auto & r : rows) {
      PassSideSample sm;
      sm.s = r[0]; sm.lo = r[1]; sm.hi = r[2]; sm.opp_lat = r[3]; sm.opp_known = true;
      in.samples.push_back(sm);
    }
    double found_at = -1.0;
    const auto r = v2x_overtaker::passCompletionSideSearch(in, 40.0, 2.0, found_at);
    std::printf("plan_mode: 側=%d 開始%.1fm 狙い%.2f 空き 左%.2f 右%.2f 到達%d\n",
                r.side, r.start_s, r.y_target, r.room_left, r.room_right,
                r.reach_ok ? 1 : 0);
    assert(r.side == -1);             // 近くの右を選ぶ
    assert(r.reach_ok);
    assert(r.start_s < 20.0);         // 遠くの左(26m 以遠)ではない
    assert(r.y_target < -1.6);        // 相手から必要間隔ぶん右

    PassSideInput legacy = in;
    legacy.plan_mode = false;
    legacy.window_ratio = 0.0;
    double found2 = -1.0;
    const auto r2 = v2x_overtaker::passCompletionSideSearch(legacy, 40.0, 2.0, found2);
    std::printf("legacy   : 側=%d ずらし%.1fm 空き 左%.2f 右%.2f\n",
                r2.side, found2, r2.room_left, r2.room_right);
    assert(r2.side == +1);            // 従来は左を選んでいた(これが不具合)
  }

  // 到達できない側は選ばない。
  {
    PassSideInput in;
    in.pass_dist = 10.0;
    in.need_sep = 1.66;
    in.window_ratio = 0.0;
    in.plan_mode = true;
    in.y_now = +3.50;            // いま大きく左にいる
    in.v_ego = 30.0;             // 速いので窓まですぐ着く = 寄る時間が無い
    in.lat_rate = 1.2;
    const double rows[][4] = {
      { 2.0, -3.20, 4.00, 0.0},
      { 6.0, -3.20, 4.00, 0.0},
      {11.0, -3.20, 4.00, 0.0},
      {16.0, -3.20, 4.00, 0.0},
    };
    for (const auto & r : rows) {
      PassSideSample sm;
      sm.s = r[0]; sm.lo = r[1]; sm.hi = r[2]; sm.opp_lat = r[3]; sm.opp_known = true;
      in.samples.push_back(sm);
    }
    double found_at = -1.0;
    const auto r = v2x_overtaker::passCompletionSideSearch(in, 8.0, 2.0, found_at);
    // 右は 0.2秒で 5.2m 寄る必要があり届かない。左(3.5 -> 1.66以上)は近い。
    std::printf("到達性  : 側=%d 開始%.1fm 狙い%.2f\n", r.side, r.start_s, r.y_target);
    assert(r.side == +1);
  }

  // ------------------------------------------------------------------。
  {
    // s[m], lo, hi  … 実コリドア idx230..25(1点 1.382m)から抜粋して線形に並べる
    const double rows[][3] = {
      { 0.0, -0.85, 3.00}, { 1.4, -0.50, 3.50}, { 2.8, -0.00, 3.75},
      { 4.1,  0.10, 3.85}, { 5.5, -0.45, 3.65}, { 6.9, -1.00, 3.25},
      { 8.3, -1.45, 2.70}, { 9.7, -1.80, 2.30}, {11.1, -2.10, 1.95},
      {12.4, -2.40, 1.65}, {13.8, -2.60, 1.40}, {15.2, -2.70, 1.25},
      {16.6, -2.75, 1.15}, {18.0, -2.85, 0.95}, {19.3, -2.85, 0.90},
      {20.7, -2.95, 0.90}, {22.1, -3.00, 0.80}, {23.5, -3.05, 0.75},
      {24.9, -3.10, 0.70}, {26.3, -3.20, 0.70}, {27.6, -3.25, 0.60},
      {29.0, -3.35, 0.55}, {30.4, -3.40, 0.40}, {31.8, -3.55, 0.20},
      {33.2, -3.55, 0.20}, {34.5, -3.75, 0.05}, {35.9, -3.80, 0.05},
      {37.3, -3.90, -0.05},
    };
    PassSideInput in;
    in.pass_dist = 25.0;
    in.need_sep = 1.6;
    in.window_ratio = 0.0;      // 全域を見る
    in.plan_mode = true;
    in.y_now = 0.0;             // 相手の真後ろ、まだ寄っていない
    in.v_ego = 8.3;             // 30km/h
    in.lat_rate = 1.2;          // 実測の横移動レート
    in.reach_gain = 1.0;
    in.room_margin = 0.30;      // 幅 0.1m のような使えない帯は採らない
    for (const auto & r : rows) {
      PassSideSample sm;
      sm.s = r[0]; sm.lo = r[1]; sm.hi = r[2];
      sm.opp_lat = 0.0; sm.opp_known = true;
      in.samples.push_back(sm);
    }
    double found_at = -1.0;
    const auto r = v2x_overtaker::passCompletionSideSearch(in, 40.0, 4.0, found_at);
    std::printf("実コース: 側=%d 開始%.1fm 狙い%.2f 空き[左%.2f 右%.2f] ずらし%.0fm\n",
                r.side, r.start_s, r.y_target, r.room_left, r.room_right, found_at);
    // **右**を選ぶこと。ここが左に戻ったら、この案件の退行。
    assert(r.side == -1);
    assert(r.reach_ok);
    assert(r.y_target < -1.0);      // 相手から必要間隔ぶん右
    assert(r.start_s > 5.0);        // いますぐではなく、寄り切れる先から
  }

  // どこにも通れない形では**側を返さない**。
  {
    PassSideInput in;
    in.pass_dist = 25.0; in.need_sep = 1.6; in.window_ratio = 0.0;
    in.plan_mode = true; in.y_now = 0.0; in.v_ego = 8.3;
    in.lat_rate = 1.2; in.reach_gain = 1.0; in.room_margin = 0.30;
    for (int k = 0; k < 20; ++k) {          // 左右とも狭く、どこも通れない
      PassSideSample sm;
      sm.s = k * 1.382; sm.lo = -1.0; sm.hi = 1.0;
      sm.opp_lat = 0.0; sm.opp_known = true;
      in.samples.push_back(sm);
    }
    double found_at = -1.0;
    const auto r = v2x_overtaker::passCompletionSideSearch(in, 40.0, 4.0, found_at);
    std::printf("通れない形: 側=%d 空き[左%.2f 右%.2f]\n",
                r.side, r.room_left, r.room_right);
    assert(r.side == 0);      // 側を決めない = 横へ出ない
  }

  // 経路の追従可能性(横加速度)。
  {
    auto mk = [](double R, double v, double ay) {
      PassSideInput in;
      in.pass_dist = 20.0; in.need_sep = 1.6; in.window_ratio = 0.0;
      in.plan_mode = true; in.y_now = 0.0; in.v_ego = v; in.lat_rate = 1.2;
      in.reach_gain = 1.0; in.room_margin = 0.30;
      in.ay_max = ay; in.curve_sign = +1;          // 左カーブ
      for (int k = 0; k < 20; ++k) {
        PassSideSample sm;
        sm.s = k * 1.382; sm.lo = -3.0; sm.hi = 3.0;
        sm.opp_lat = 0.0; sm.opp_known = true; sm.radius_m = R;
        in.samples.push_back(sm);
      }
      return in;
    };
    double f = -1.0;
    auto a = mk(9.0, 8.3, 23.0);   // 30km/h・半径9m → ay 7.7 で余裕
    auto b = mk(9.0, 10.0, 5.0);   // 曲がれない設定
    auto c = mk(9.0, 10.0, 0.0);   // 判定を無効化
    const int sa = v2x_overtaker::passCompletionSideSearch(a, 20.0, 4.0, f).side;
    const int sb = v2x_overtaker::passCompletionSideSearch(b, 20.0, 4.0, f).side;
    const int sc = v2x_overtaker::passCompletionSideSearch(c, 20.0, 4.0, f).side;
    std::printf("追従可能性: 余裕あり=%d 曲がれない=%d 判定無効=%d\n", sa, sb, sc);
    assert(sa != 0);   // 曲がれるなら側を返す
    assert(sb == 0);   // 曲がれないなら側を返さない
    assert(sc != 0);   // 判定を切れば従来どおり
  }

  // **この案件の退行検知**。
  {
    const double rows[][3] = {
      { 0.0,  0.20, 3.55}, { 1.4,  0.30, 3.65}, { 2.8, -0.25, 3.45},
      { 4.1, -0.80, 3.05}, { 5.4, -1.25, 2.50}, { 6.6, -1.60, 2.10},
      { 7.7, -1.90, 1.75}, { 8.8, -2.20, 1.45}, { 9.8, -2.40, 1.20},
      {10.8, -2.50, 1.05}, {11.8, -2.55, 0.95}, {12.8, -2.66, 0.76},
      {13.9, -2.75, 0.80}, {15.0, -3.00, 0.95}, {16.2, -3.05, 0.85},
      {17.4, -3.10, 0.80}, {18.7, -3.15, 0.75}, {20.1, -3.25, 0.75},
      {21.5, -3.30, 0.65}, {23.0, -3.40, 0.60}, {24.5, -3.45, 0.45},
      {26.0, -3.60, 0.25}, {27.5, -3.60, 0.25}, {28.9, -3.80, 0.10},
      {30.4, -3.78, 0.03}, {31.9, -3.76, -0.19}, {33.4, -3.70, -0.25},
      {34.8, -3.55, -0.30}, {36.3, -3.45, -0.10}, {37.8, -3.25, 0.10},
      {39.2, -2.95, 0.35}, {40.7, -2.50, 0.80}, {42.2, -2.00, 1.40},
      {43.6, -1.50, 2.15}, {45.1, -0.70, 2.60}, {46.4, -0.35, 3.00},
      {47.8, -0.10, 3.15},
    };
    auto mk = [&](double zone_end, bool full_scale) {
      PassSideInput in;
      in.pass_dist = 25.0; in.need_sep = 1.60; in.window_ratio = 0.0;
      in.plan_mode = true; in.y_now = 0.0; in.v_ego = 8.3;
      in.lat_rate = 1.2; in.reach_gain = 1.0; in.room_margin = 0.30;
      in.zone_end_s = zone_end; in.plan_full_scale = full_scale;
      for (const auto & r : rows) {
        PassSideSample sm;
        sm.s = r[0]; sm.lo = r[1]; sm.hi = r[2];
        sm.opp_lat = 0.0; sm.opp_known = true;
        in.samples.push_back(sm);
      }
      return in;
    };
    double fa = -1.0;
    auto no_clamp = mk(-1.0, false);
    const auto r_old = v2x_overtaker::passCompletionSideSearch(no_clamp, 40.0, 4.0, fa);
    // (2) 後端を直線(46.4m)の中に縛り、縮尺の短縮も禁じる = 修正後。
    auto clamped = mk(46.4, true);
    const auto r_new = v2x_overtaker::passCompletionSideSearch(clamped, 40.0, 4.0, fa);
    std::printf("実直線: 従来 側=%d(空き 左%.2f 右%.2f) / 修正後 側=%d 開始%.1fm "
                "狙い%.2f(空き 左%.2f 右%.2f)\n",
                r_old.side, r_old.room_left, r_old.room_right,
                r_new.side, r_new.start_s, r_new.y_target,
                r_new.room_left, r_new.room_right);
    // **右**でなければこの案件の退行。
    assert(r_new.side == -1);
    assert(r_new.reach_ok);
    assert(r_new.y_target < -1.0);          // 相手から必要間隔ぶん右
    assert(r_new.room_right > 0.30);        // 通せると答えた側は必ず空いている
    // 窓の後端が直線を出ていないこと。
    assert(r_new.start_s + 25.0 <= 46.4 + 1e-6);
  }

  // plan_mode では窓を短縮しないこと。
  {
    PassSideInput in;
    in.pass_dist = 20.0; in.need_sep = 1.60; in.window_ratio = 0.0;
    in.plan_mode = true; in.y_now = 0.0; in.v_ego = 8.3;
    in.lat_rate = 1.2; in.reach_gain = 1.0; in.room_margin = 0.30;
    in.plan_full_scale = true;
    // 手前 17m は左が広く、その先で左が閉じる形。
    for (int k = 0; k < 30; ++k) {
      PassSideSample sm;
      sm.s = k * 1.4;
      sm.lo = -0.5; sm.hi = (sm.s < 17.0) ? 4.0 : 1.0;
      sm.opp_lat = 0.0; sm.opp_known = true;
      in.samples.push_back(sm);
    }
    double fa = -1.0;
    const auto r = v2x_overtaker::passCompletionSideSearch(in, 0.0, 4.0, fa);
    std::printf("窓の短縮禁止: 側=%d 空き[左%.2f 右%.2f]\n",
                r.side, r.room_left, r.room_right);
    // 左は 17m 以降で閉じる。20m 全域を見れば左は通せない。
    assert(r.side != +1);
  }

  std::printf("test_pass_completion_side: ok\n");
  return 0;
}
