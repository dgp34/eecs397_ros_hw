# p7_baxter_playfile

This package addresses P7: Create an interesting playfile for the Baxter robot and record its motion.

## How to Run

From the command line, run

`roslaunch baxter_gazebo baxter_world.launch`

and

`roslaunch baxter_launch_files baxter_playfile_nodes.launch`

Then, invoke the playfile playback by doing the following:

-Nagivate to the package directory: `roscd p7_baxter_playfile`

-Run the joint-space file in this package. In order for the upcoming backstory to make sense, the joint-space file must be used for both the right and left arms:
`rosrun p7_baxter_playfile baxter_play shywavehug.jsp shywavehug.jsp`

If just a simple look at the joint-space file is desired, running it on the right arm will do:
`rosrun p7_baxter_playfile baxter_play shywavehug.jsp`

Note that, in order to avoid naming conflicts, the C++ code files have been given the following names in this package:

-get_and_save_jntval

-baxter_record

-baxter_play

-baxter_multitraj_play

-baxter_playfile_serv

-baxter_playfile_clien

### Backstory

The Baxter robot is having trouble. It's left arm is copying its right arm's joint angles exactly! Knowing the issue at hand and understanding how ridiculous it looks, the robot covers its face to the approaching person. It soon realizes, however, that this person can help! It immediately waves excitedly (albeit oddly due to its condition). As the person comes closer, the robot goes in for a hug out of relief. It then positions itself with arms outstretched so that the person can examine and hopefully resolve the robot's predicament.
