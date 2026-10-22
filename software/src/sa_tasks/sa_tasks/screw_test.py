"""Screwing by hand, gripper only: close the iris, wait, turn M4, open the iris. No ROS.

  ros2 run sa_tasks screw_test

Put the nut into the open iris and hold the stud under it, then:
  Enter            one run with place.screw of tasks.yaml (duration, speed)
  <ms> [speed]     one run with this duration (and speed)
  q                quit (the board's watchdog switches off after 2 s)
Wait before turning: screw_test.wait_s in tasks.yaml.
"""
import logging
import time

from sa_tasks import config, serial_gripper


def main():
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    cfg = config.load_tasks()
    screw = cfg["place"]["screw"]
    wait_s = cfg["screw_test"]["wait_s"]
    gripper = serial_gripper.connect(cfg["gripper"])
    print(__doc__.split("\n\n", 1)[1])
    try:
        gripper.open()
        while True:
            parts = input("> ").split()
            if parts[:1] == ["q"]:
                break
            try:
                args = [int(a) for a in parts]
                duration_ms = args[0] if args else screw["duration_ms"]
                speed = args[1] if len(args) > 1 else screw["speed"]
                gripper.close()
                print(f"closed, turning in {wait_s:.0f} s ({duration_ms} ms at speed {speed})")
                time.sleep(wait_s)
                gripper.rotate(screw["direction"], duration_ms, speed)
                gripper.open()
                print("done, iris open")
            except (ValueError, serial_gripper.GripperError) as error:
                print(f"error: {error}")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        gripper.shutdown()


if __name__ == "__main__":
    main()
