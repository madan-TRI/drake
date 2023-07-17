import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

import numpy as np
import os
import sys

# Command-line flags determine which example (pendulum, acrobot, spinner) we're
# dealing with.
possible_example_names = "pendulum, acrobot, spinner, toy_limb"
if (len(sys.argv) != 3):
    print(f"Usage: {sys.argv[0]} {possible_example_names}")
    print("\nThe corresponding example must be run first (e.g. 'bazel run traj_opt/examples:pendulum`), with 'save_solver_stats_csv=true'")
    sys.exit(1)
example_name = sys.argv[1]

qcf = sys.argv[2]

# drake/ directory, contains drake/bazel-out symlink
drake_root = os.getcwd()

# Bazel stores files in strange places
data_file = drake_root + f"/bazel-out/k8-opt/bin/traj_opt/examples/{example_name}.runfiles/drake/force_traj_{qcf}.csv"

data_file_0 = drake_root + f"/bazel-out/k8-opt/bin/traj_opt/examples/{example_name}.runfiles/drake/force_traj_0.csv"

# Read data from the file and format nicely
data = np.genfromtxt(data_file, delimiter=',', names=True)
iters = data["t"]

data_0 = np.genfromtxt(data_file_0, delimiter=',', names=True)
iters_0 = data_0["t"]

# Make plots
fig, ax = plt.subplots(1,2,sharex=True,figsize=(16,11))

fig.suptitle(f"{example_name} contact force data")

# ax[0,0].plot(iters, data["trust_ratio"])
# ax[0,0].set_ylabel("trust ratio")
# ax[0,0].set_ylim((-1,3))

ax[0].plot(iters, data["contact_force"])
ax[0].set_ylabel("contact force")
ax[0].set_xlabel("time")
ax[0].set_ylim((0, 30))
# set title
ax[0].title.set_text(f"contact_force_penalty_weight = {qcf}")

ax[1].plot(iters_0, data_0["contact_force"])
ax[1].set_ylabel("contact force")
ax[0].set_xlabel("time")
ax[1].set_ylim((0, 30))
ax[1].title.set_text(f"contact_force_penalty_weight = 0")

plt.show()


#iters_tr_accepted = []
#tr_accepted = []
#eta = 0.0
#for i in range(len(iters)):
#    if data["trust_ratio"][i] > eta:
#        tr_accepted.append(data["trust_ratio"][i])
#        iters_tr_accepted.append(iters[i])
#ax[1,1].plot(iters_tr_accepted, tr_accepted)