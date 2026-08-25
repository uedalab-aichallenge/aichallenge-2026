# ACT LeRobot Workspace

AI Challenge の rosbag を LeRobotDataset に変換し、LeRobot の ACT policy で模倣学習するための作業ディレクトリです。

## 前提

LeRobot は AI Challenge の ROS/Docker 環境ではなく、ホスト側の conda 環境で使います。

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot-aichallenge

cd ~/aichallenge/aichallenge-2026/aichallenge/ml_workspace/act_lerobot
```

LeRobot v0.6.0 を使う場合、`~/aichallenge/lerobot` を editable install しておきます。

```bash
cd ~/aichallenge/lerobot
pip install -e ".[training]"
pip install rosbags opencv-python pyyaml tqdm
```

確認:

```bash
python -c "import lerobot; print(lerobot.__version__)"
which lerobot-train
```

## 1. 学習用 rosbag 収集

ACTには画像と教師制御が必要です。最低限必要なtopicは以下です。

- `/sensing/camera/image_raw`
- `/control/command/control_cmd`

速度も観測stateに入れる場合は以下も記録します。

- `/vehicle/status/velocity_status`

AWSIM のカメラがOFFだと画像topicは出ません。`aichallenge/simulator_scripts/dev.sh` の `--camera off` を `--camera cpu` または `--camera gpu` にしてから、シミュレータを起動し直してください。

```bash
make down
make dev
```

記録前に画像topicが流れているか確認します。

```bash
ROS_DOMAIN_ID=1 docker compose run --rm --no-deps autoware-command \
  bash -lc 'source /opt/ros/humble/setup.bash && source /aichallenge/workspace/install/setup.bash && ros2 topic hz /sensing/camera/image_raw'
```

記録:

```bash
ROS_DOMAIN_ID=1 docker compose run --rm --no-deps autoware-command \
  bash -lc 'source /opt/ros/humble/setup.bash && source /aichallenge/workspace/install/setup.bash && ros2 bag record /clock /sensing/camera/image_raw /control/command/control_cmd /vehicle/status/velocity_status -s mcap -o /output/d1_learning_bag'
```

終了は記録している端末で `Ctrl+C` です。`metadata.yaml` に以下があれば画像入りbagです。

```text
name: /sensing/camera/image_raw
type: sensor_msgs/msg/Image
```

## 2. LeRobotDataset へ変換

`--state-mode none` は画像のみ、`--state-mode vehicle_status` は速度stateも使います。今のACT実装では `vehicle_status` を推奨します。

```bash
cd ~/aichallenge/aichallenge-2026/aichallenge/ml_workspace/act_lerobot
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot-aichallenge

python3 convert_rosbag_to_lerobot.py \
  --bags-dir ~/aichallenge/aichallenge-2026/aichallenge/ml_workspace/dataset/d1_learning_bag \
  --outdir ./dataset/aichallenge_act \
  --repo-id local/aichallenge_act \
  --fps 40 \
  --state-mode vehicle_status \
  --overwrite
```

変換結果の確認:

```bash
python -c "from lerobot.datasets import LeRobotDataset; ds=LeRobotDataset('local/aichallenge_act', root='./dataset/aichallenge_act'); print(ds.num_frames, ds.num_episodes); print(ds.features)"
```

期待するfeature:

- `observation.images.front`: RGB画像
- `action`: `[acceleration, steering_tire_angle]`
- `observation.state`: `[longitudinal_velocity, heading_rate]`

## 3. GPU確認

GPUが見えない場合は、NVIDIAデバイスノードを作成します。

```bash
sudo nvidia-modprobe -u -c=0
nvidia-smi
```

conda環境から確認:

```bash
python -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'cpu')"
```

`True` とGPU名が出れば `DEVICE=cuda` で学習できます。

## 4. ResNet18 重みをネットから取得

ACTのデフォルトbackboneは ResNet18 です。ネット接続できる環境では、事前学習重みを先にキャッシュしておくと学習開始時に止まりません。

```bash
python -c "from torchvision.models import resnet18, ResNet18_Weights; resnet18(weights=ResNet18_Weights.IMAGENET1K_V1)"
```

保存先:

```text
~/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth
```

ネット接続がない場合は、`PRETRAINED_BACKBONE_WEIGHTS=null` のまま学習します。`train_act.bash` のデフォルトはネット不要の `null` です。

## 5. ACT 学習

スモークテスト:

```bash
DEVICE=cpu STEPS=1 BATCH_SIZE=1 NUM_WORKERS=0 SAVE_FREQ=1 LOG_FREQ=1 OUTPUT_DIR=./outputs/train/act_smoke ./train_act.bash
```

GPU学習:

```bash
DEVICE=cuda \
STEPS=100000 \
BATCH_SIZE=8 \
NUM_WORKERS=4 \
SAVE_FREQ=5000 \
LOG_FREQ=100 \
OUTPUT_DIR=./outputs/train/act_$(date +%Y%m%d_%H%M%S) \
./train_act.bash
```

ResNet18事前学習重みを使う場合:

```bash
DEVICE=cuda \
PRETRAINED_BACKBONE_WEIGHTS=ResNet18_Weights.IMAGENET1K_V1 \
STEPS=100000 \
BATCH_SIZE=8 \
NUM_WORKERS=4 \
SAVE_FREQ=5000 \
LOG_FREQ=100 \
OUTPUT_DIR=./outputs/train/act_$(date +%Y%m%d_%H%M%S) \
./train_act.bash
```

CPUで短く動かす場合:

```bash
DEVICE=cpu STEPS=5000 BATCH_SIZE=1 NUM_WORKERS=0 SAVE_FREQ=1000 LOG_FREQ=50 OUTPUT_DIR=./outputs/train/act_cpu_$(date +%Y%m%d_%H%M%S) ./train_act.bash
```

40Hz走行では `CHUNK_SIZE=10..30` から試すのが無難です。`100` は2.5秒先までのchunkになり、車両制御では長すぎる場合があります。現在のデフォルトは `20` です。

学習済みpolicyは以下のような場所に保存されます。LeRobot v0.6.0では `checkpoints/last` ではなく、ステップ番号のディレクトリを指定します。

```text
outputs/train/<run_name>/checkpoints/100000/pretrained_model
```

## 6. ROS2 推論

ホスト側で `ros2 launch` する場合、AI Challenge workspaceをsourceします。Docker内 `make autoware-build` の symlink install はホストからリンク先が壊れることがあるため、見えない場合は `act_lerobot_controller` だけホストでビルドします。

```bash
source /opt/ros/humble/setup.bash
cd ~/aichallenge/aichallenge-2026/aichallenge/workspace
colcon build --packages-select act_lerobot_controller --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/act_lerobot_controller/share/act_lerobot_controller/local_setup.bash
```

確認:

```bash
ros2 pkg list | grep act_lerobot_controller
```

起動:

```bash
ros2 launch act_lerobot_controller act_lerobot.launch.xml \
  policy_path:=/home/hirosueryoko/aichallenge/aichallenge-2026/aichallenge/ml_workspace/act_lerobot/outputs/train/<run_name>/checkpoints/100000/pretrained_model \
  use_sim_time:=true
```

`reference.launch.xml` から使う場合:

```bash
ros2 launch aichallenge_submit_launch reference.launch.xml \
  control_method:=act_lerobot \
  simulation:=true \
  use_sim_time:=true
```

## よくあるエラー

`LeRobot is not installed`:

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate lerobot-aichallenge
```

`no synchronized samples; skipped`:

rosbagに `/sensing/camera/image_raw` が入っていません。AWSIMの `--camera off` を直して、画像topicが流れていることを確認してから再記録してください。

`Extra features: {'timestamp'}`:

LeRobot v0.6.0では `timestamp` は自動生成です。このリポジトリの最新版 `convert_rosbag_to_lerobot.py` を使ってください。

`Package 'act_lerobot_controller' not found`:

AI Challenge workspaceがsourceされていません。ホスト起動なら `act_lerobot_controller` をホスト側でビルドし、`local_setup.bash` をsourceしてください。
