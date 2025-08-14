import numpy as np
import os
import random 
# Where your norm_metrics_i.npy files are
metrics_dir = '/home/mobrob/ros_ws/BARN_turtlebot/norm_metrics_files'

scores = []
for i in range(300):
    path = os.path.join(metrics_dir, f'norm_metrics_{i}.npy')
    metrics = np.load(path)

    # Example: weighted difficulty score
    # More negative = easier, more positive = harder
    score = (
        -metrics[0]  # distance to closest obstacle (lower = harder)
        -metrics[1]  # avg visibility (lower = harder)
        +metrics[2]  # dispersion (higher = harder)
        +metrics[3]  # char dimension (higher = harder)
        +metrics[4]  # tortuosity (higher = harder)
    )
    scores.append((i, score))

# Sort by score
scores.sort(key=lambda x: x[1])

# Get 3 easiest, 3 medium, 3 hardest

easy_bin = scores[0:99]
medium_bin = scores[100:199]
hard_bin = scores[200:299]


random.seed(42)  # for reproducibility
easiest = random.sample(easy_bin, 3)
medium = random.sample(medium_bin, 3)
hardest = random.sample(hard_bin, 3)

print("Easiest maps:", easiest)
print("Medium maps:", medium)
print("Hardest maps:", hardest)
