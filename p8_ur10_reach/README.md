# p8_ur10_reach

This package addresses P8: Calculate and display the reachability for Baxter and UR10.

## Example usage

The raw data results can be found in the _Reachability Raw Data Files_ directory.  However, should the UR10 data need to be recalculated, do the following:

From the command line, navigate to the directory containing the p8_ur10_reach package.  This is to ensure that all data files will save into this folder and not elsewhere.  Then, run

`rosrun p8_ur10_reach ur10_reach`

The reachability will be calculated and the resulting files can then be processed using the provided .m file.
