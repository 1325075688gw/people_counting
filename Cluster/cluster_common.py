from Origin_Point_Cloud.common import evm_index

# 多帧融合，帧数，修改帧数后，记得重新训练radar_training_result
mixed_num = 3

# 聚类分割
# 分割后的分数增量，这个数越大，越难分割，建议（0，0.3）之间调整
score_offset = 0.5
# 需要判断的分割集合，里面有2，代表分成2个，3，分成3个。没有的数，系统不会分割
divide_num = [2]

# 训练结果
radar_training_result = [
    {'person_length_coef': 0.111048, 'person_length_intercept': 0.351560, 'person_width_max': 0.432569,
     'person_width_min': 0.282569, 'person_points': 128},
    {'person_length_coef': 0.078930, 'person_length_intercept': 0.422207, 'person_width_max': 0.419551,
     'person_width_min': 0.269551, 'person_points': 102},
    {'person_length_coef': 0.146146, 'person_length_intercept': 0.520052, 'person_width_max': 1.2,
     'person_width_min': 0.662638, 'person_points': 287}
]

# 噪声过滤阈值
# 最小聚类点数。如果老是把人过滤了，降低这个阈值
min_cluster_count = 20
# 最小snr值。如果老是把人过滤了，降低这个阈值
cluster_snr_limit = 120

# 雷达个数
radar_num = len(evm_index)