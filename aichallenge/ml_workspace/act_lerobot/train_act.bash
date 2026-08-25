#!/usr/bin/env bash
set -euo pipefail

DATASET_REPO_ID="${DATASET_REPO_ID:-local/aichallenge_act}"
DATASET_ROOT="${DATASET_ROOT:-./dataset/aichallenge_act}"
OUTPUT_DIR="${OUTPUT_DIR:-./outputs/train/act_aichallenge}"
POLICY_REPO_ID="${POLICY_REPO_ID:-local/act_aichallenge_policy}"
DEVICE="${DEVICE:-cuda}"
STEPS="${STEPS:-100000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
CHUNK_SIZE="${CHUNK_SIZE:-20}"
N_ACTION_STEPS="${N_ACTION_STEPS:-20}"
NUM_WORKERS="${NUM_WORKERS:-4}"
WANDB_ENABLE="${WANDB_ENABLE:-false}"
SAVE_FREQ="${SAVE_FREQ:-5000}"
LOG_FREQ="${LOG_FREQ:-100}"
PRETRAINED_BACKBONE_WEIGHTS="${PRETRAINED_BACKBONE_WEIGHTS:-null}"

exec lerobot-train \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --dataset.root="${DATASET_ROOT}" \
  --policy.type=act \
  --output_dir="${OUTPUT_DIR}" \
  --job_name=act_aichallenge \
  --policy.device="${DEVICE}" \
  --policy.repo_id="${POLICY_REPO_ID}" \
  --policy.push_to_hub=false \
  --wandb.enable="${WANDB_ENABLE}" \
  --steps="${STEPS}" \
  --batch_size="${BATCH_SIZE}" \
  --num_workers="${NUM_WORKERS}" \
  --save_freq="${SAVE_FREQ}" \
  --log_freq="${LOG_FREQ}" \
  --policy.chunk_size="${CHUNK_SIZE}" \
  --policy.n_action_steps="${N_ACTION_STEPS}" \
  --policy.pretrained_backbone_weights="${PRETRAINED_BACKBONE_WEIGHTS}"
