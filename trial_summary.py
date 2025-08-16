import csv
import os

import re
import math
import matplotlib.pyplot as plt

world_nums = []
data = []
summary = {}
class Trial:
    def __init__(self, world_num, trial_num, STATUS, reached_lg, total_lg):
        self.world_num = (world_num)
        self.trial_num = int(trial_num)
        self.STATUS = (STATUS)
        self.reached_lg = int(reached_lg)
        self.total_lg = int(total_lg)

# get all the world_nums and data
filename = 'baseline_half_radius.csv'
with open(filename, 'r') as file:
    csv_reader = csv.DictReader(file)
    for row in csv_reader:
        data.append(row)
        if row['world_num'] not in world_nums:
            world_nums.append(row['world_num'])
# print(data)
print(world_nums)
data_counter = 0
for world in world_nums:
    if world not in summary:

        summary[world] = []
    trial = 0
    for data_line in data[data_counter:]:
        if data_line['world_num'] == world:
            trial+=1
            data_counter+=1
            trial_ = Trial(
                data_line['world_num'],
                trial, 
                data_line['trial_result'],
                data_line['local_goal_reached'],
                data_line['num_lg'])
            print(trial_.reached_lg)
            if trial_.STATUS == 'AMCL TIMEOUT - MAX RETRIES EXCEEDED' or trial_.reached_lg == 0:
                continue
            else: 
                summary[world].append(trial_)

print(summary)


# ---- tweak this if your definition of success is different ----
def is_success(t):
    # success = reached all local goals AND not an AMCL timeout
    return (t.reached_lg >= t.total_lg) or (t.STATUS == 'SUCCESS')

def frac_progress(t):
    return (t.reached_lg / t.total_lg) if t.total_lg else 0.0

# Optional: sort worlds numerically by the number inside "world 12"
def world_key(w):
    m = re.search(r'(\d+)', str(w))
    return int(m.group(1)) if m else math.inf

worlds = sorted(summary.keys(), key=world_key)

# ---- aggregate per world ----
successes = []
failures = []
avg_frac = []
max_frac = []

for w in worlds:
    trials = summary[w]
    succ = sum(is_success(t) for t in trials)
    total = len(trials)
    fail = total - succ
    fracs = [frac_progress(t) for t in trials]
    successes.append(succ)
    failures.append(fail)
    avg_frac.append(sum(fracs)/total if total else 0.0)
    max_frac.append(max(fracs) if fracs else 0.0)

# ---- 1) Successes per world ----
plt.figure(figsize=(10, 5))
plt.bar(worlds, successes)
plt.title('Successful Trials per World')
plt.xlabel('World')
plt.ylabel('# Successes')
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
plt.savefig(f'{filename}_successes_per_world.png', dpi=200)
plt.show()

# ---- 2) Stacked: successes vs failures ----
plt.figure(figsize=(10, 6))
plt.bar(worlds, successes, label='Successes')
plt.bar(worlds, failures, bottom=successes, label='Failures')
plt.title('Trials per World (Success vs Failure)')
plt.xlabel('World')
plt.ylabel('# Trials')
plt.xticks(rotation=45, ha='right')
plt.legend()
plt.tight_layout()
plt.savefig(f'{filename}_success_vs_failure_per_world.png', dpi=200)
plt.show()

# ---- 3) “How far they go”: average fraction of LG reached ----
plt.figure(figsize=(10, 5))
plt.bar(worlds, avg_frac)
plt.title('Average Fraction of Local Goals Reached per World')
plt.xlabel('World')
plt.ylabel('Average fraction (0–1)')
plt.ylim(0, 1)
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
plt.savefig(f'{filename}_avg_fraction_per_world.png', dpi=200)
plt.show()

# ---- 4) Max fraction (optional) ----
plt.figure(figsize=(10, 5))
plt.bar(worlds, max_frac)
plt.title('Max Fraction of Local Goals Reached per World')
plt.xlabel('World')
plt.ylabel('Max fraction (0–1)')
plt.ylim(0, 1)
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
plt.savefig(f'{filename}_max_fraction_per_world.png', dpi=200)
plt.show()


