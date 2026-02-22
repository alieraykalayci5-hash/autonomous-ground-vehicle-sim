import argparse
import os
import numpy as np
import matplotlib.pyplot as plt

def load_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True, dtype=None, encoding="utf-8")

def exists(path):
    return os.path.exists(path) and os.path.isfile(path)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="indir", required=True)
    ap.add_argument("--out", dest="outdir", default="plots")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    state_path = os.path.join(args.indir, "state.csv")
    truth_path = os.path.join(args.indir, "truth.csv")
    est_path   = os.path.join(args.indir, "target_est.csv")
    gps_path   = os.path.join(args.indir, "gps.csv")
    lidar_path = os.path.join(args.indir, "lidar.csv")

    state = load_csv(state_path)
    truth = load_csv(truth_path)
    est   = load_csv(est_path)

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

    if exists(gps_path):
        gps = load_csv(gps_path)
        if "fix" in gps.dtype.names:
            mask = gps["fix"] == 1
        else:
            mask = np.ones(len(gps), dtype=bool)

        plt.figure()
        plt.plot(gps["x_truth"], gps["y_truth"], label="vehicle truth")
        plt.scatter(gps["zx"][mask], gps["zy"][mask], s=6, label="gps meas (fix)", alpha=0.7)
        plt.axis("equal")
        plt.legend()
        plt.title("GPS measurement vs truth")
        plt.savefig(os.path.join(args.outdir, "gps.png"), dpi=160)

    if exists(lidar_path):
        lidar = load_csv(lidar_path)
        plt.figure()
        plt.plot(lidar["t"], lidar["min_range"], label="min_range")
        if "ahead_blocked" in lidar.dtype.names:
            plt.plot(lidar["t"], lidar["ahead_blocked"], label="ahead_blocked")
        plt.legend()
        plt.title("Lidar diagnostics")
        plt.savefig(os.path.join(args.outdir, "lidar.png"), dpi=160)

    print("Wrote plots to", args.outdir)

if __name__ == "__main__":
    main()