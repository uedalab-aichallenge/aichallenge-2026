#include "v2x_overtaker/rear_end_release.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>

using v2x_overtaker::RearEndReleaseInput;
using v2x_overtaker::rearEndRelease;

int main()
{
  {
    RearEndReleaseInput in;
    in.sep_now = 1.50; in.need = 1.45;
    const auto r = rearEndRelease(in);
    assert(r.release);
    assert(!r.by_prediction);   // 予測ぶんではない
  }

  // 2. 退行しない: 横間隔が閉じつつあるときは、予測で解除しない。
  {
    RearEndReleaseInput in;
    in.sep_now = 1.00; in.sep_rate = -0.5; in.gap = 8.0; in.closing = 2.0;
    in.need = 1.45; in.floor = 0.30;
    const auto r = rearEndRelease(in);
    assert(!r.release);
  }

  // 3. 本命: 横へ出つつあり、縦に並ぶころには離れているなら解除する。
  //    0.60m + 1.2m/s * 1.0s = 1.80m >= 1.45m
  {
    RearEndReleaseInput in;
    in.sep_now = 0.60; in.sep_rate = 1.2; in.gap = 10.0; in.closing = 10.0;
    in.need = 1.45; in.floor = 0.30; in.max_predict_sec = 1.5;
    const auto r = rearEndRelease(in);
    assert(r.release);
    assert(r.by_prediction);
    assert(std::abs(r.t_meet - 1.0) < 1e-6);
    assert(std::abs(r.sep_pred - 1.80) < 1e-6);
  }

  // 4. 真後ろに貼り付いた状態(floor 未満)では、開きつつあっても解除しない。
  {
    RearEndReleaseInput in;
    in.sep_now = 0.10; in.sep_rate = 3.0; in.gap = 10.0; in.closing = 10.0;
    in.need = 1.45; in.floor = 0.30;
    const auto r = rearEndRelease(in);
    assert(!r.release);
  }

  {
    RearEndReleaseInput in;
    in.sep_now = 0.60; in.sep_rate = 0.2; in.gap = 20.0; in.closing = 2.0;
    in.need = 1.45; in.floor = 0.30; in.max_predict_sec = 1.5;
    const auto r = rearEndRelease(in);
    assert(!r.release);
    assert(std::abs(r.t_meet - 1.5) < 1e-6);
  }

  // 6. 詰まっていない(closing <= 0)なら、並ぶ時刻が定まらないので解除しない。
  {
    RearEndReleaseInput in;
    in.sep_now = 0.60; in.sep_rate = 2.0; in.gap = 5.0; in.closing = 0.0;
    in.need = 1.45; in.floor = 0.30;
    const auto r = rearEndRelease(in);
    assert(!r.release);
  }

  // 7. 壊れた値では解除しない(fail-closed)。
  {
    RearEndReleaseInput in;
    in.sep_now = std::nan(""); in.need = 1.45;
    assert(!rearEndRelease(in).release);
    RearEndReleaseInput in2;
    in2.sep_now = 0.60; in2.sep_rate = std::nan("");
    in2.gap = 10.0; in2.closing = 10.0; in2.need = 1.45; in2.floor = 0.30;
    assert(!rearEndRelease(in2).release);
  }

  // 8. 符号に依らない(左右どちらでも絶対値で扱う)。
  {
    RearEndReleaseInput in;
    in.sep_now = -1.50; in.need = 1.45;
    assert(rearEndRelease(in).release);
  }

  std::printf("test_rear_end_release: all passed\n");
  return 0;
}
