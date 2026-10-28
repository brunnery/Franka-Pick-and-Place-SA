#!/usr/bin/env bash
# Run before every GUI command (gui.yaml environment.before). A ROS process killed while it
# created its Fast DDS shared-memory port leaves a 0-byte /dev/shm/fastrtps_port74xx; every
# later process on that port then hangs without a word (28.10.: ros2_control_node, spawners,
# even `ros2 node list`). `fastdds shm clean` does not remove such a port, so do it here.
for port in /dev/shm/fastrtps_port*; do
  case "$port" in *_el) continue ;; esac
  [ -f "$port" ] && [ ! -s "$port" ] || continue
  echo "removing broken DDS shared-memory port $port"
  rm -f "$port" "/dev/shm/sem.$(basename "$port")_mutex"
done
# segments and ports of dead processes (only those no live process holds)
if command -v fastdds > /dev/null; then fastdds shm clean > /dev/null 2>&1; fi
exit 0
