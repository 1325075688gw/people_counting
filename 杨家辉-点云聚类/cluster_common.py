# 多帧融合，帧数，修改帧数后，记得重新训练radar_training_result
mixed_num = 3

# 聚类分割
# 分割后的分数增量，这个数越大，越难分割，建议（0，0.3）之间调整
score_offset = 0.15
# 需要判断的分割集合，里面有2，代表分成2个，3，分成3个。没有的数，系统不会分割
divide_num = [2]

# 训练结果
radar_training_result = [
    {'person_length_coef': 0.083100, 'person_length_intercept': 0.765957, 'person_width_max': 0.610310,
     'person_width_min': 0.460310, 'person_points': 153},
    {'person_length_coef': 0.127080, 'person_length_intercept': 0.647264, 'person_width_max': 0.615996,
     'person_width_min': 0.465996, 'person_points': 263},
    {'person_length_coef': 0.146146, 'person_length_intercept': 0.520052, 'person_width_max': 1.2,
     'person_width_min': 0.662638, 'person_points': 287}
]

# 雷达个数
radar_num = 1