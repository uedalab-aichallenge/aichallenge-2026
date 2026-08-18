# ACT LeRobot Workspace

LeRobot の ACT で AI Challenge の模倣学習を行うための作業ディレクトリです。

## 1. LeRobotDataset へ変換

```bash
cd /aichallenge/ml_workspace/act_lerobot

python3 convert_rosbag_to_lerobot.py \
  --bags-dir /aichallenge/ml_workspace/rawdata \
  --outdir ./dataset/aichallenge_act \
  --repo-id local/aichallenge_act \
  --fps 40 \
  --state-mode none \
  --overwrite
```

速度を観測 state に入れる場合:

```bash
python3 convert_rosbag_to_lerobot.py \
  --bags-dir /aichallenge/ml_workspace/rawdata \
  --outdir ./dataset/aichallenge_act \
  --repo-id local/aichallenge_act \
  --fps 40 \
  --state-mode vehicle_status \
  --overwrite
```

出力 feature:

- `observation.images.front`: RGB画像
- `action`: `[acceleration, steering_tire_angle]`
- `observation.state`: `--state-mode` が `none` 以外のときだけ追加

## 2. ACT 学習

```bash
cd /aichallenge/ml_workspace/act_lerobot
DATASET_REPO_ID=local/aichallenge_act CHUNK_SIZE=20 N_ACTION_STEPS=20 ./train_act.bash
```

40Hz走行では `CHUNK_SIZE=10..30` から試すのが無難です。`100` は2.5秒先までのchunkになり、車両制御では長すぎる場合があります。

## 3. ROS2 推論

学習済みpolicyディレクトリを指定して起動します。

```bash
ros2 launch act_lerobot_controller act_lerobot.launch.xml \
  policy_path:=/aichallenge/ml_workspace/act_lerobot/outputs/train/act_aichallenge/checkpoints/last/pretrained_model \
  use_sim_time:=true
```

`reference.launch.xml` から使う場合:

```bash
ros2 launch aichallenge_submit_launch reference.launch.xml \
  control_method:=act_lerobot \
  simulation:=true \
  use_sim_time:=true
```
