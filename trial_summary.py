
from matplotlib.ticker import MultipleLocator, AutoMinorLocator
import csv
import os
import re
import math
import matplotlib.pyplot as plt
import sys

import matplotlib as mpl
mpl.rcParams.update({
    "font.size": 13,        # base size
    "axes.titlesize": 15,
    "axes.labelsize": 13,
    "xtick.labelsize": 11,
    "ytick.labelsize": 11,
    "legend.fontsize": 11,
    "pdf.fonttype": 42,     # embed fonts nicely for LaTeX
    "ps.fonttype": 42,
})
# ---- CONFIG ----
# Usage: python trial_summary.py /path/to/dir
if len(sys.argv) < 2:
    print("Usage: python script.py <directory>")
    sys.exit(1)

input_dir = sys.argv[1]
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

# ---- Helpers ----
def nice_grid(ax, y_major=None, minor_divs=2):
    """Make a subtle y-only grid behind the bars."""
    ax.set_axisbelow(True)
    ax.grid(False)
    if y_major is not None:
        ax.yaxis.set_major_locator(MultipleLocator(y_major))
        ax.yaxis.set_minor_locator(AutoMinorLocator(minor_divs))
    else:
        ax.yaxis.set_minor_locator(AutoMinorLocator())
    ax.yaxis.grid(True, which='major', linewidth=0.6, alpha=0.25)
    ax.yaxis.grid(True, which='minor', linewidth=0.4, alpha=0.12)
    ax.xaxis.grid(False)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(axis='both', length=3, width=0.6)

def is_success(t):
    return (t.reached_lg >= t.total_lg) or (t.STATUS == 'SUCCESS')

def frac_progress(t):
    return min((t.reached_lg / t.total_lg), 1) if t.total_lg else 0.0

def world_key(w):
    m = re.search(r'(\d+)', str(w))
    return int(m.group(1)) if m else math.inf

def save_plot(fig, name):
    outpath = os.path.join(input_dir, name)
    fig.savefig(outpath, dpi=300, bbox_inches='tight')
    print(f"Saved {outpath}")

# ---- Aggregate ----
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

# --- Fixed buckets you chose (make sure these strings match your CSV) ---
easy   = {'world 12','world 36','world 75'}
medium = {'world 125','world 203','world 210'}
hard   = {'world 69','world 187','world 266'}

bucket_rank = {**{w:0 for w in easy}, **{w:1 for w in medium}, **{w:2 for w in hard}}

def _rank(w):  # bucket first, then numeric world id within bucket
    m = re.search(r'(\d+)', str(w))
    wn = int(m.group(1)) if m else math.inf
    return (bucket_rank.get(w, 99), wn)

order_idx = sorted(range(len(worlds)), key=lambda i: _rank(worlds[i]))
import re

def world_id(w):
    m = re.search(r'\d+', str(w))
    return m.group(0) if m else str(w)

labels = [world_id(w) for w in worlds]
def _reord(a): return [a[i] for i in order_idx]
worlds    = _reord(worlds)
worlds = [world_id(w) for w in worlds]
successes = _reord(successes)
failures  = _reord(failures)
avg_frac  = _reord(avg_frac)
max_frac  = _reord(max_frac)
# ---- 1) Successes per world ----
fig, ax = plt.subplots(figsize=(10, 6))
ax.bar(worlds, successes, color='C0')
ax.set_title('Successful Trials per World for CNN')
ax.set_xlabel('Worlds (easy → hard)')
ax.set_ylim(0, 10)                 # top at 10
ax.set_yticks(range(0, 11, 1))     # 0,1,...,10
nice_grid(ax, y_major=1)           # optional: subtle grid every 1
ax.set_ylabel('# Successes')
plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
nice_grid(ax)  # default (no fixed major step)
fig.tight_layout(pad=1.1)
fig.subplots_adjust(top=0.90)   # leave 10% for the title
save_plot(fig, 'successes_per_world.png')
plt.close(fig)

# ---- 2) Stacked successes vs failures ----
fig, ax = plt.subplots(figsize=(10, 6))
ax.bar(worlds, successes, label='Successes', color='C0')
ax.bar(worlds, failures, bottom=successes, label='Failures', color='C1')
ax.set_title('Trials per World (Success vs Failure) for CNN')
ax.set_xlabel('Worlds (easy → hard)')
ax.set_ylim(0, 10)                 # top at 10
ax.set_yticks(range(0, 11, 1))     # 0,1,...,10
ax.set_ylabel('# Trials')
ax.legend()
plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
nice_grid(ax)
fig.tight_layout()
save_plot(fig, 'success_vs_failure_per_world.png')
plt.close(fig)

# ---- 3) Average fraction of LG reached ----
fig, ax = plt.subplots(figsize=(10, 5))
ax.bar(worlds, avg_frac, color='C0')
ax.set_title('Average Fraction of Local Goals Reached per World for CNN')
ax.set_xlabel('Worlds (easy → hard)')
ax.set_ylabel('Average fraction (0–1)')
ax.set_ylim(0, 1)
plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
nice_grid(ax, y_major=0.2)  # fixed 0.2 grid for 0–1 axis
fig.tight_layout()
save_plot(fig, 'avg_fraction_per_world.png')
plt.close(fig)

# ---- 4) Max fraction ----
fig, ax = plt.subplots(figsize=(10, 5))
ax.bar(worlds, max_frac, color='C0')
ax.set_title('Max Fraction of Local Goals Reached per World for CNN')
ax.set_xlabel('Worlds (easy → hard)')
ax.set_ylabel('Max fraction (0–1)')
ax.set_ylim(0, 1)
plt.setp(ax.get_xticklabels(), rotation=45, ha='right')
nice_grid(ax, y_major=0.2)  # fixed 0.2 grid for 0–1 axis
fig.tight_layout()
save_plot(fig, 'max_fraction_per_world.png')
plt.close(fig)
