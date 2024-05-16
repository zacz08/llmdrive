GPU_NUM=1
# DATASET_ROOT='/home/zc/datasets/lmdrive/LMDrive'
DATASET_ROOT='/home/zc/datasets/lb2_lmdrive'

torchrun --nproc_per_node=$GPU_NUM train_pretrain_without_velocity.py "$@" $DATASET_ROOT  --dataset carla --train-towns 13 --val-towns 13 \
    --train-weathers 0 --val-weathers  0\
    --model memfuser_baseline_e1d3 --sched cosine --epochs 1 --warmup-epochs 0 --lr 0.00075 --batch-size 8  -j 8 --no-prefetcher --eval-metric l1_error \
    --opt adamw --opt-eps 1e-8 --weight-decay 0.05 \
    --scale 0.9 1.1 --saver-decreasing --clip-grad 5 --freeze-num -1 \
    --with-backbone-lr --backbone-lr 0.0003 \
    --multi-view --with-lidar --multi-view-input-size 3 128 128 \
    --smoothed_l1 \
    --experiment memfuser_e1d3 \
    --pretrained \
