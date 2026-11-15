import argparse
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt

KILOBYTES_PER_MEGABYTE = 1024


def parse_pidstat(path):
    series = defaultdict(lambda: defaultdict(list))
    columns = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("Linux"):
            continue
        if line.startswith("#"):
            columns = line.lstrip("#").split()
            continue
        row = dict(zip(columns, line.split()))
        command = row["Command"]
        for column, value in row.items():
            if column != "Command":
                series[command][column].append(float(value))
    return series


def plot_metric(series, column, scale, title, ylabel, output_path):
    figure, axis = plt.subplots(figsize=(12, 6))
    for command in sorted(series):
        values = [value * scale for value in series[command][column]]
        axis.plot(range(len(values)), values, label=command)
    axis.set_title(title)
    axis.set_xlabel("Time [s]")
    axis.set_ylabel(ylabel)
    axis.grid(True, alpha=0.3)
    axis.legend(loc="upper left", bbox_to_anchor=(1.01, 1.0))
    figure.tight_layout()
    figure.savefig(output_path, dpi=150)
    return figure


def main():
    parser = argparse.ArgumentParser(
        description="Plot per-node CPU and memory usage from pidstat logs."
    )
    parser.add_argument(
        "capture_dir",
        type=Path,
        help="capture_* directory containing cpu/ and mem/ pidstat logs",
    )
    parser.add_argument("--show", action="store_true", help="open the plots in windows")
    args = parser.parse_args()

    cpu_series = parse_pidstat(args.capture_dir / "cpu" / "pidstat.log")
    mem_series = parse_pidstat(args.capture_dir / "mem" / "pidstat.log")

    plot_metric(
        cpu_series,
        column="%CPU",
        scale=1.0,
        title="CPU usage per node",
        ylabel="CPU [%]",
        output_path=capture_dir / "cpu_usage.png",
    )
    plot_metric(
        mem_series,
        column="RSS",
        scale=1.0 / KILOBYTES_PER_MEGABYTE,
        title="Memory usage per node (RSS)",
        ylabel="RSS [MB]",
        output_path=capture_dir / "memory_usage.png",
    )

    print(f"Saved plots to {capture_dir}")
    if args.show:
        plt.show()


if __name__ == "__main__":
    main()
