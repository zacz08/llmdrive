#!/bin/bash
NUM_PROC=$1
shift
CUDA_LAUNCH_BLOCKING=1 OMP_NUM_THREADS=4 torchrun --nproc_per_node=$NUM_PROC train_pretrain.py "$@"

