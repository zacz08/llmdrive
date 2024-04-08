import os


class GlobalConfig:
    """base architecture configurations"""

    # Controller
    turn_KP = 1.25
    turn_KI = 0.75
    turn_KD = 0.3
    turn_n = 40  # buffer size

    speed_KP = 5.0
    speed_KI = 0.5
    speed_KD = 1.0
    speed_n = 40  # buffer size

    max_throttle = 0.75  # upper limit on throttle signal value in dataset
    brake_speed = 0.1  # desired speed below which brake is triggered
    brake_ratio = 1.1  # ratio of speed to desired speed at which brake is triggered
    clip_delta = 0.35  # maximum change in speed input to logitudinal controller

    # the model architecture of the vision encoder.
    # preception_model = 'memfuser_baseline_e1d3_return_feature'
    preception_model = 'memfuser_baseline_e1d3'

    # the checkpoint path of the vision encoder (obtained in the vision encoder pretraining stage).
    preception_model_ckpt = '/home/zc/LMDrive/ckpt/vision-encoder-r50.pth.tar'

    # the checkpoint path of the llm (LLaMA/Vicuna/LLaVA).
    llm_model = '/home/zc/LMDrive/llm_model/llava-v1.5-7b'

    # the checkpoint path of the lmdrive (obtained in the instruction finetuing stage).
    lmdrive_ckpt = '/home/zc/LMDrive/ckpt/llava-v1.5-checkpoint.pth'

    agent_use_notice = False
    sample_rate = 2


    def __init__(self, **kwargs):
        for k, v in kwargs.items():
            setattr(self, k, v)
