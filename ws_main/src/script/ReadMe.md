# Script
**This module contains scripts for the GNF project**

`main` is the default branch for this module!

| Name              | Function |
|----------         |-----|
| `run_planner.sh`  | Run planner node.  |
| `start_node.sh`   | Pub start msg to drone_node to start basic node.  |
| `kill_node.sh`    | Pub kill msg to drone_node to end basic node.  |
| `takeoff.sh`      | Takeoff drone. (by px4ctrl).  |
| `land.sh`         | Land drone. (by px4ctrl).  |
| `killros.sh`      | Kill all ros nodes. (by kill -9)  |
| `bat.sh`          | Output battery voltage. (by mavros).  |
| `ntpdate.sh`      | Sync time with ntp server.  |
| `updatetime.sh`   | Recursively touch all files in subdirectories, updating modification times.  |
