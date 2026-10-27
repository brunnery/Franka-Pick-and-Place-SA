"""Summary of control_command_success_rate once per second, read from
`ros2 topic echo ... --field control_command_success_rate` on stdin (button "Lost control
cycles"). The value is the share of the last 100 control cycles (1 ms) that arrived in time:
1.0 = none lost, 0.99 = 1 of 100 lost.

Temporary add-on, not part of the thesis code (gui/README.md).
"""
import sys
import time

PERIOD_S = 1.0


def main():
    count, low, lowest, start = 0, 0, 1.0, time.monotonic()
    print("per second: messages | below 1.0 | lowest value", flush=True)
    for line in sys.stdin:
        try:
            value = float(line.strip())
        except ValueError:  # the '---' separators
            continue
        count += 1
        if value < 1.0:
            low += 1
            lowest = min(lowest, value)
        if time.monotonic() - start >= PERIOD_S:
            mark = "" if low == 0 else "   <-- cycles lost"
            print(f"{time.strftime('%H:%M:%S')}  {count:5d} | {low:5d} | {lowest:.2f}{mark}",
                  flush=True)
            count, low, lowest, start = 0, 0, 1.0, time.monotonic()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
