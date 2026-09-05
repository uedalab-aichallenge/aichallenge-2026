// 壁に刺さった車を目標経路へ戻す操作を計算する。
//
// それまでの復帰は「決め打ちで後退 -> 静定 -> 前進」を繰り返す方式だった。
// 抜けられないまま25秒を使い切ることが多く、実測では
// 「復帰 完了」の2秒後に横4.4m・車体111度の姿勢で再スタックし、
// そこから53秒を失っていた。決め打ちの舵角では、どの向きへ
// どれだけ切れば出られるかを判断できない。
//
// ここでは車両運動学(ホイールベースと最大舵角)で候補を積分し、
// 走行可能領域(コリドア)からはみ出すものを捨てて、
// 経路へ戻れる最短の操作を選ぶ。
//
// 純粋な Reeds-Shepp 最短経路との違いは壁を見る点。壁を見ないと、
// 前進から始まる最短経路が選ばれて、壁に押し付けられた車では実行できない。
#ifndef STUCK_RECOVERY_CONTROLLER__RECOVERY_PLANNER_HPP_
#define STUCK_RECOVERY_CONTROLLER__RECOVERY_PLANNER_HPP_

#include <cstddef>
#include <string>
#include <vector>

namespace recovery
{

// 実際の壁。占有格子から「各点が壁からどれだけ離れているか[m]」を作って持つ。
//
// これが要る理由: コリドア(中心線 + 左右の余裕)は 121 点・平均間隔 2.75m の
// 粗い表現で、しかも中心線に対する「横方向」しか持たない。半径 4.8m の
// ヘアピンで鼻先が壁に当たっていても、コリドア上は「領域内」に見える。
// 実測でも、車体が 96 度傾いて まったく動けない状態で「横=-0.30 領域内」
// と出ていた。壁の判定にはコリドアではなく占有格子を使う。
class ObstacleMap
{
public:
  // map_server 形式の yaml(+ 同じ場所の pgm)を読む
  bool load(const std::string & yaml_path);
  bool valid() const { return !dist_.empty(); }
  // (x,y) の符号付き距離[m]。走行可能側で正(壁までの距離)、
  // 壁の中で負(食い込み量)。地図の外は判定しない(大きな値)。
  double clearance(double x, double y) const;

  // --- rviz へそのまま出すための読み出し(2026-08-31・ユーザー指示)
  //
  // 復帰の当たり判定が **実際に見ている地図** を publish するために要る。
  // map_server を別に立てて同じ yaml を読ませる手もあるが、それだと
  // 「判定が見ている地図」と「表示している地図」が別物になり得る。
  // ここから出せば原理的にズレない。
  int width() const { return w_; }
  int height() const { return h_; }
  double resolution() const { return res_; }
  double originX() const { return ox_; }
  double originY() const { return oy_; }
  // 格子 (ix, iy) の符号付き距離[m]。範囲外は 0 を返す。
  //
  // **iy は nav_msgs/OccupancyGrid の約束(iy=0 が原点＝下端、上へ増える)**で受ける。
  // 内部の dist_ は pgm の行順(行0 = 画像の上端 = y が最大)で持っているので、
  // ここで行を反転する。clearance() が
  //     r = h - (y - oy) / res
  // と数えているのと同じ向きにそろえるためで、これを忘れると
  // **表示だけが上下反転する**(実際に出した不具合)。
  // 当たり判定は clearance() を通るので、この関数は表示専用。
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

// 走行可能領域。中心線と、その各点における左右の余裕。
// lat は左向き法線を正とし、lo <= lat <= hi なら領域内。
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
  // pos の最近傍点の添字と、そこでの横位置を返す
  bool locate(double px, double py, std::size_t & idx, double & lat) const;
};

struct VehicleParams
{
  double wheel_base{2.14};      // [m]
  double max_steer{0.6109};     // [rad] 35deg
  double half_width{0.6};       // 車体半幅[m]。当たり判定の膨らませに使う
  // base_link から前端まで[m]。1.2 では「地図では 0.3〜0.4m 空いているのに
  // 前進できない」事例が繰り返し出た(実測4件)。実際の車体はもう少し前まで
  // あるとみて伸ばす。伸ばしすぎると経路が見つからなくなるので控えめに。
  double front_overhang{1.6};
  double rear_overhang{1.0};    // 後端まで[m]
  // コリドアの lo/hi は「車体中心を置いてよい範囲」で、生成時(make_corridor.py)に
  // すでに 半幅0.73 + 余裕0.35 = 1.08 m ぶん壁から内側に寄せてある。
  // その境界に対して車体四隅をさらに当てると、幅を二重に数えることになる。
  // 実際の壁はおよそ lo-1.08 / hi+1.08 なので、四隅の判定はここまで広げる。
  // corridor_ten.csv は make_corridor.py --margin 0.45 で作られている
  // (出荷物から逆算して確定。work/corridor_cmd.txt 参照)。
  double bound_inflate{1.18};   // lo/hi に含まれている膨らませ量[m] (0.73 + 0.45)
  double wall_margin{0.12};     // 車体外周が壁から最低限あけたい距離[m]
};

// 1つの操作区間。gear と舵角が決まっている。
struct Phase
{
  bool forward{true};
  double steer{0.0};     // [rad] 左が正
  double length{0.0};    // [m] 走行距離(正)
};

struct Plan
{
  std::vector<Phase> phases;
  std::vector<Pose> path;      // 積分した経路(表示・追従用)
  double cost{0.0};
  bool valid{false};
  // 採用した計画の「前進区間」で見込まれる最小の余裕[m]。
  // 追従中の中断閾値をこれに合わせるために持つ。計画が合格とした幾何を
  // 中断側が落とす、という食い違いを構造的に無くす。
  double min_wall_clear{1e9};
  double min_car_clear{1e9};
  // path の先頭から何点が「後退区間」か。後退と前進で別のコントローラへ
  // 経路を渡すために要る(2026-08-31)。
  std::size_t rev_points{0};
};

// 現在姿勢から、コリドア内で目標経路へ戻る操作を計算する。
//   corridor  走行可能領域
//   veh       車両パラメータ
//   start     現在姿勢
//   ahead_min/ahead_max  復帰先として狙う中心線上の距離[m]の範囲
// 候補は後退・前進それぞれの舵角と距離の組み合わせを離散化して列挙する。
// first_phase で最初の区間の向きを縛れる。
//   0 = どちらでもよい / +1 = 前進から始める / -1 = 後退から始める
//
// これが要るのは、走行可能領域だけでは「鼻先が壁に当たっていて前進できない」
// ことが分からないため。実測では前進の計画を5回続けて出し、
// 動けないまま25秒を使い切った。前進が駄目だったと分かった後は
// 後退から始めさせる、という形で失敗を反映する。
//
// min_gain を指定すると、終端のはみ出しが今より これだけ[m] 減る計画だけを採る。
//
// min_reverse は後退区間の下限[m]。自己位置推定がずれていると、地図の上では
// 前方が空いているのに実際には壁に当たって動けない、ということが起きる
// (実測で「前方 1.85m 空き」なのに前進できない姿勢があった)。
// そのときは地図を信じても仕方がないので、後退の下限を上げて物理的に離す。
// 壁の判定には obstacles(占有格子)を使う。読めていない場合だけ corridor で代用する。
// 姿勢 p に車体を置いたときの、車体外周から壁までの最小距離[m]。
// 復帰を終えてよいかの判定にも使う。
double wallClearanceAt(const ObstacleMap & map, const VehicleParams & veh, const Pose & p);

// 復帰経路が避けるべき他車。位置だけ持つ(向きは使わず円で近似する)。
//
// これが無かったため、復帰の経路計画は **壁しか見ていなかった**。
// 前方に止まっている車がいても、その車を突き抜ける円弧を平然と計画し、
// 実行してぶつかり、また stuck になって復帰が再発動する、を繰り返していた
// (ユーザー報告: 「止まっている他車に復帰動作後なんども突っ込んでいる」)。
struct CarObstacle
{
  double x{0.0};
  double y{0.0};
};

// ===================================================================
// 目標指向の復帰計画(ユーザー指示 2026-09-06)
//
// 【ご指示】「ぶつかったことを検知したら、すこし先の pure_pursuit の
// 目標地点まで他車、壁を考慮して切り返しを行なって向かえる経路を計算、
// その計算結果に基づいて後退、前進を行い復帰」
// 「経路は切り返しをするので Y 字型になる。できるだけ後退が少なくなる経路を」
// 「後退する経路に壁や他車がいない限り切り返しは1回で済む。逆に後方に何かが
//   ある場合は複数回必要な場合もある。ちゃんと考慮して設計して」
//
// 【従来の plan() との違い】
//   従来: 「後退1本 + 前進1本」を全列挙し、終端がどれだけ経路に戻れたかを
//         重み付き合計で採点する。**目標地点が存在せず、切り返しは1回に固定。**
//         前進が一定舵角の円弧なので参照経路と繋がらず、実測で合流の継ぎ目が
//         中央 1.16m ずれていた(復帰直後に横 1m から追従を始めるので再スタックする)。
//   本関数: **参照経路上の少し先の姿勢を目標に置き**、そこへ到達する経路を探す。
//         切り返し回数は固定しない。後退と切り返しにコストを掛けるので、
//         **後方が空いていれば切り返し1回(Y字)が最小コストで選ばれ、
//         後方が塞がっていれば必要な回数だけ増える。**
//         終点が参照経路上の目標そのものなので、合流の継ぎ目は構造的に 0。
//
// 【探索】状態 (x, y, yaw) を離散化した格子上の A*。
//   素片は 舵 5 通り × {前進, 後退} × step[m]。
//   コスト = 前進長 + 後退長 * w_reverse + 切り返し回数 * w_switch
//   ヒューリスティック = 目標までの直線距離(前進コスト 1.0 基準なので許容的)
//   当たり判定は plan() と同じ wallViolation / carViolation を使い、
//   開始時点の食い込みは「悪化させない範囲で」許す(でないと脱出できない)。
// 【ご指示の訂正 2026-09-06】「できるだけ後退を減らす というのは間違いでした。
// 大きく回転し、逆に速度が落ちています。復帰後の位置や加速度なども含めて
// 一番早くなる経路を計算してください。PCスペックは考える必要はありません」
//
// そこでコストを距離の重み付けから **所要時間** に置き換える。
// 評価するのは「いまから、参照経路上の**共通の地点**(現在位置から tail_len[m] 先)
// を通過するまでの時間」。共通の地点までで測るので、
//   ・早く復帰できる経路
//   ・復帰した地点が先にある経路
//   ・復帰した時点の速度が高い(大きく回転していない)経路
// が同じ尺度で比べられる。**後退量そのものは評価しない。**
// 後退が短くても大きく回頭して速度を失う経路は、tail の時間で負ける。
struct GoalPlanParams
{
  // --- 目標は近くに置く(ユーザー指示 2026-09-06) ---
  //
  // 【なぜ近くか】復帰の仕事は「壁に食い込み・大きく傾き・領域の外」から
  // 「領域の内側・向きがおおむね揃い・動ける」までを作ることだけ。
  // そこから先(ラインへ戻る途中の他車・壁の回避)は、通常走行の回避処理
  // (v2x_overtaker)が**復帰中も毎周期動いており**、そちらが担当する。
  //
  // 遠くに目標を置くと、復帰が 12〜15m の前進区間を計画することになる。
  // それは通常の回避がやる仕事を、
  //   ・他車は計画時点のスナップショットしか見ない
  //   ・一定舵角の円弧で表現する
  //   ・追従が飽和して計画と別の軌跡になる
  // という劣った条件で代行することになり、実測では失敗がすべてこの区間で
  // 起きていた(完了した例の方位差は 2〜11度で質が高い)。
  double goal_ahead_min{4.0};    // 目標をどれだけ先に置くか[m]の下限
  double goal_ahead_max{10.0};   // 同 上限(この範囲の候補すべてを目標にする)
  double goal_ahead_step{1.0};   // 目標候補の間隔[m]
  double step{0.5};              // 素片の長さ[m]
  int max_switch{4};             // 切り返しの上限回数
  double pos_tol{0.6};           // 目標に着いたとみなす位置の許容[m]
  double yaw_tol{0.26};          // 同 方位の許容[rad] (15deg)
  double grid_xy{0.25};          // 状態の離散化[m]
  double grid_yaw{0.175};        // 同 方位[rad] (10deg)
  double grid_v{0.5};            // 同 速度[m/s]
  int max_expand{200000};        // 展開の上限(PC 性能は問わないので広く取る)
  // --- 時間の見積りに使う車両の値(すべて実測または公式値) ---
  double v_fwd_max{10.0};        // 前進で見込む上限速度[m/s]
  double v_rev_max{1.5};         // 後退の上限速度[m/s] (reverse_pure_pursuit の cap)
  double a_accel{1.37};          // 加速[m/s^2] (AWSIM の指令クランプの実測値)
  double a_brake{1.66};          // 減速[m/s^2] (実測の達成値 中央)
  double ay_max{12.0};           // 旋回で許す横加速度[m/s^2](低速域は控えめに)
  double t_gear{0.30};           // 前後を入れ替えるのに要る時間[s](実測 0.23〜0.27+)
  double tail_len{40.0};         // 共通の評価地点までの距離[m]
};

// 目標へ到達する経路を返す。到達できなければ valid=false。
// goal_idx には到達した目標の中心線 index が入る(合流用の経路を作るのに使う)。
Plan planToGoal(const Corridor & corridor, const ObstacleMap & obstacles,
                const VehicleParams & veh, const Pose & start,
                const std::vector<CarObstacle> & cars,
                const GoalPlanParams & prm,
                std::size_t * goal_idx = nullptr);

Plan plan(const Corridor & corridor, const ObstacleMap & obstacles,
          const VehicleParams & veh, const Pose & start,
          double ahead_min = 4.0, double ahead_max = 16.0,
          int first_phase = 0, double min_gain = 0.0,
          double min_escape = 2.5, double min_reverse = 0.0,
          double max_reverse = 8.0,
          const std::vector<CarObstacle> & cars = {},
          double req_wall_clear = 0.0,   // 前進区間で確保したい壁との余裕[m]
          double req_car_clear = 0.0,    // 前進区間で確保したい他車との余裕[m]
          // true なら、通常探索で解が1つも無いときに棄却条件を全て外して
          // 再探索し、「どこにも当たらない案が無いなら、一番離れられる案」を
          // 返す(ユーザー指示)。kCarRadius を 1.49m に拡げた副作用で
          // 「解なし」を返す距離が伸びたぶんをここで受ける。
          bool best_effort = false
          );

// 姿勢 p に車体を置いたときの、他車との重なりの深さ[m]。
// **常に 0 以上**を返す(離れていても 0)。経路の棄却判定にだけ使うこと。
// 距離として使ってはいけない(実際にその誤用で無限に切り返すバグを出した)。
double carViolation(const std::vector<CarObstacle> & cars,
                    const VehicleParams & veh, const Pose & p);

// 姿勢 p に車体を置いたときの、他車までの余裕[m]。**負なら重なっている。**
// 他車がいなければ大きな値を返す。実行中の監視にはこちらを使う。
double carClearanceAt(const std::vector<CarObstacle> & cars,
                      const VehicleParams & veh, const Pose & p);

}  // namespace recovery

#endif  // STUCK_RECOVERY_CONTROLLER__RECOVERY_PLANNER_HPP_
