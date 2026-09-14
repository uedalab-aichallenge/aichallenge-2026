#include "v2x_overtaker/overtake_start.hpp"

int main()
{
  using v2x_overtaker::overtakeStartGapOk;
  using v2x_overtaker::overtakeAttemptStartAuthorized;
  using v2x_overtaker::plannedSpotReady;

// 抜きどころは同じ相手かつ距離がゲート以内のときだけ準備完了。
  if (!plannedSpotReady(true, true, true, 35.0, 40.0)) { return 12; }
  if (plannedSpotReady(true, true, false, 10.0, 40.0)) { return 13; }
  if (plannedSpotReady(true, true, true, 41.0, 40.0)) { return 14; }

// 抜きどころが無ければ通常の開始車間を要求する。
  if (overtakeStartGapOk(false, 1.4, 4.5, false, false, 1.3)) { return 1; }
  if (!overtakeStartGapOk(false, 5.0, 4.5, false, false, 1.3)) { return 2; }

// 計画済みの抜きどころと公式レーンでは小さい下限を使える。
  if (!overtakeStartGapOk(false, 1.4, 4.5, true, false, 1.3)) { return 3; }
  if (!overtakeStartGapOk(false, 1.4, 4.5, false, true, 1.3)) { return 4; }
  if (overtakeStartGapOk(false, 1.2, 4.5, true, false, 1.3)) { return 5; }

// 実行中の追い越しは車間で打ち切らない。
  if (!overtakeStartGapOk(true, 0.5, 4.5, false, false, 1.3)) { return 6; }

// 許可対象がいまの前方車と一致しないと開始しない。
  if (overtakeAttemptStartAuthorized(
      false, true, true, true, "d1", "")) { return 7; }
  if (overtakeAttemptStartAuthorized(
      false, true, true, true, "d1", "d2")) { return 8; }
  if (!overtakeAttemptStartAuthorized(
      false, true, true, true, "d1", "d1")) { return 9; }
  if (overtakeAttemptStartAuthorized(
      false, false, true, true, "d1", "d1")) { return 10; }
// 実行中の追い越しは開始判定の対象外。
  if (!overtakeAttemptStartAuthorized(
      true, false, false, true, "", "")) { return 11; }
  return 0;
}
