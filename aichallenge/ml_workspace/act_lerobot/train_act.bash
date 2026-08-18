#!/usr/bin/env bash
set -euo pipefail

DATASET_REPO_ID="${DATASET_REPO_ID:-local/aichallenge_act}"
OUTPUT_DIR="${OUTPUT_DIR:-/aichallenge/ml_workspace/act_lerobot/outputs/train/act_aichallenge}"
POLICY_REPO_ID="${POLICY_REPO_ID:-local/act_aichallenge_policy}"
DEVICE="${DEVICE:-cuda}"
STEPS="${STEPS:-100000}"
BATCH_SIZE="${BATCH_SIZE:-8}"
CHUNK_SIZE="${CHUNK_SIZE:-20}"
N_ACTION_STEPS="${N_ACTION_STEPS:-20}"
WANDB_ENABLE="${WANDB_ENABLE:-false}"

exec lerobot-train \
  --dataset.repo_id="${DATASET_REPO_ID}" \
  --policy.type=act \
  --output_dir="${OUTPUT_DIR}" \
  --job_name=act_aichallenge \
  --policy.device="${DEVICE}" \
  --policy.repo_id="${POLICY_REPO_ID}" \
  --policy.push_to_hub=false \
  --wandb.enable="${WANDB_ENABLE}" \
  --steps="${STEPS}" \
  --batch_size="${BATCH_SIZE}" \
  --policy.chunk_size="${CHUNK_SIZE}" \
  --policy.n_action_steps="${N_ACTION_STEPS}"
