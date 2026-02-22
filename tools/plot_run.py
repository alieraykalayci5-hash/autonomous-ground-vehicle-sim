import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

def load_csv(path):
    return np.genfromtxt(path, delimiter=',', names=True, dtype=None, encoding='utf-8')

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="indir", required=True)
    ap.add_argument("--out", dest="outdir", default="plots")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    state = load_csv(os.path.join(args.indir, "state.csv"))
    truth = load_csv(os.path.join(args.indir, "truth.csv"))
    est = load_csv(os.path.join(args.indir, "target_est.csv"))

    plt.figure()
    plt.plot(state["x"], state["y"], label="vehicle")
    plt.plot(truth["target_x"], truth["target_y"], label="target truth")
    plt.plot(est["xhat"], est["yhat"], label="target est")
    plt.axis("equal")
    plt.legend()
    plt.title("Trajectory")
    plt.savefig(os.path.join(args.outdir, "traj.png"), dpi=160)

    plt.figure()
    plt.plot(state["t"], state["v"], label="v")
    plt.legend()
    plt.title("Speed")
    plt.savefig(os.path.join(args.outdir, "speed.png"), dpi=160)

    print("Wrote plots to", args.outdir)

if __name__ == "__main__":
    main()