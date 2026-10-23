"""Screwing by hand, gripper only: close the iris, wait, turn M4, open the iris. No ROS.

  ros2 run sa_tasks screw_test

Put the nut into the open iris and hold the stud under it, then:
  Enter            one run as place.screw in tasks.yaml: catch (all steps at once), tighten
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
                if args:
                    speed = args[1] if len(args) > 1 else screw["tighten"]["speed"]
                    phases = [{"duration_ms": args[0], "speed": speed}]
                else:  # no height feedback here: the catch steps as one turn
                    catch = screw["catch"]
                    phases = [{"duration_ms": catch["step_ms"] * catch["max_steps"],
                               "speed": catch["speed"]}, screw["tighten"]]
                gripper.close()
                print(f"closed, turning in {wait_s:.0f} s: "
                      + ", then ".join(f"{p['duration_ms']} ms at speed {p['speed']}"
                                       for p in phases))
                time.sleep(wait_s)
                for phase in phases:
                    gripper.rotate(screw["direction"], phase["duration_ms"], phase["speed"])
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
