
import csv
import os
import re
import math
import matplotlib.pyplot as plt
import sys

# ---- CONFIG ----
# Pass the directory as the first argument when running:
# python script.py /path/to/dir
if len(sys.argv) < 2:
    print("Usage: python script.py <directory>")
    sys.exit(1)

input_dir = sys.argv[1]
# or 'baseline_half_radius.csv'
filename = os.path.join(input_dir, 'baseline.csv')
if not os.path.exists(filename):
    print(f"Error: {filename} not found")
    sys.exit(1)

# ---- Data loading ----
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


with open(filename, 'r') as file:
    csv_reader = csv.DictReader(file)
    for row in csv_reader:
        data.append(row)
        if row['world_num'] not in world_nums:
            world_nums.append(row['world_num'])

data_counter = 0
for world in world_nums:
    if world not in summary:
        summary[world] = []
    trial = 0
    for data_line in data[data_counter:]:
        if data_line['world_num'] == world:
            trial += 1
            data_counter += 1
            trial_ = Trial(
                data_line['world_num'],
                trial,
                data_line['trial_result'],
                data_line['local_goal_reached'],
                data_line['num_lg']
            )
            if trial_.STATUS == 'AMCL TIMEOUT - MAX RETRIES EXCEEDED' or trial_.reached_lg == 0:
                continue
            else:
                summary[world].append(trial_)

# ---- Analysis functions ----


def is_success(t):
    return (t.reached_lg >= t.total_lg) or (t.STATUS == 'SUCCESS')


def frac_progress(t):
    return (t.reached_lg / t.total_lg) if t.total_lg else 0.0


def world_key(w):
    m = re.search(r'(\d+)', str(w))
    return int(m.group(1)) if m else math.inf


worlds = sorted(summary.keys(), key=world_key)

successes, failures, avg_frac, max_frac = [], [], [], []
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

# ---- Plotting helper ----


def save_plot(fig, name):
    outpath = os.path.join(input_dir, name)
    fig.savefig(outpath, dpi=200)
    print(f"Saved {outpath}")


# ---- 1) Successes per world ----
fig = plt.figure(figsize=(10, 5))
plt.bar(worlds, successes)
plt.title('Successful Trials per World')
plt.xlabel(f'{input_dir}')
plt.ylabel('# Successes')
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
save_plot(fig, 'successes_per_world.png')
plt.close(fig)

# ---- 2) Stacked successes vs failures ----
fig = plt.figure(figsize=(10, 6))
plt.bar(worlds, successes, label='Successes')
plt.bar(worlds, failures, bottom=successes, label='Failures')
plt.title('Trials per World (Success vs Failure)')
plt.xlabel(f'{input_dir}')
plt.ylabel('# Trials')
plt.xticks(rotation=45, ha='right')
plt.legend()
plt.tight_layout()
save_plot(fig, 'success_vs_failure_per_world.png')
plt.close(fig)

# ---- 3) Average fraction of LG reached ----
fig = plt.figure(figsize=(10, 5))
plt.bar(worlds, avg_frac)
plt.title('Average Fraction of Local Goals Reached per World')
plt.xlabel(f'{input_dir}')
plt.ylabel('Average fraction (0–1)')
plt.ylim(0, 1)
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
save_plot(fig, 'avg_fraction_per_world.png')
plt.close(fig)

# ---- 4) Max fraction ----
fig = plt.figure(figsize=(10, 5))
plt.bar(worlds, max_frac)
plt.title('Max Fraction of Local Goals Reached per World')
plt.xlabel(f'{input_dir}')
plt.ylabel('Max fraction (0–1)')
plt.ylim(0, 1)
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
save_plot(fig, 'max_fraction_per_world.png')
plt.close(fig)
