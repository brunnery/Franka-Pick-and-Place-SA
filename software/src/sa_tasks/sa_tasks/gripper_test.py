"""Try the iris and the M4 rotation by hand over the serial gripper. No ROS.

  ros2 run sa_tasks gripper_test

Values from tasks.yaml (gripper: port, open, close; place.screw: direction, duration, speed).
Commands:
  o                open the iris
  c                close the iris, then hold
  r [ms] [speed]   M4 tightening direction (defaults: place.screw)
  l [ms] [speed]   M4 loosening direction
  s                all motors off
  q                quit (the board's watchdog switches off after 2 s)
"""
import logging

from sa_tasks import config, serial_gripper


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = config.load_tasks()
    screw = cfg["place"]["screw"]
    gripper = serial_gripper.connect(cfg["gripper"])
    print(__doc__.split("Commands:")[1].rstrip())
    try:
        while True:
            parts = input("> ").split()
            if not parts:
                continue
            cmd = parts[0]
            try:
                args = [int(a) for a in parts[1:]]
                if cmd == "o":
                    gripper.open()
                elif cmd == "c":
                    gripper.close()
                elif cmd in ("r", "l"):
                    direction = screw["direction"] * (1 if cmd == "r" else -1)
                    duration_ms = args[0] if args else screw["duration_ms"]
                    speed = args[1] if len(args) > 1 else screw["speed"]
                    gripper.rotate(direction, duration_ms, speed)
                elif cmd == "s":
                    gripper.stop()
                elif cmd == "q":
                    break
                else:
                    print("unknown command")
                    continue
                print("done")
            except (ValueError, serial_gripper.GripperError) as error:
                print(f"error: {error}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        gripper.shutdown()


if __name__ == "__main__":
    main()
