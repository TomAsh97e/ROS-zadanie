# Performance analysis tools

## CPU and memory usage

Collect runtime statistics and plot CPU and memory usage per node:

1. Run `./analyze_performance.sh`.
2. In another terminal, run `launch_envhydrolab.sh`. The analysis script waits for all ROS 2 nodes to start before it begins capturing data.
3. Press `Ctrl+C` in the analysis terminal to stop the capture. The CPU and memory plots are generated automatically in `performance_analysis/capture_<timestamp>` in the repository root.
