# p9_obj_manip

This package addresses P9: Modify existing client-server relationships so that the UR10 robot can pick up and dropoff various NIST ARIAC components to the staging area on the cafe table.

## Example usage

Running this package has been made fairly easy.  Simply run the launch file with

`roslaunch p9_obj_manip p9_obj_manip.launch`

and then run

`rosrun p9_obj_manip pulley_fetcher_client`

when ready.

### Known issues

The client does not always connect to the server properly...give it multiple tries.  There will also be moments when the the terminal will say the client gave up, but if allowed a couple seconds, the UR10 robot in simulation will begin to complete the task anyways.
