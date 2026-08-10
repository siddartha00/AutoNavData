1. Reduce the inflation of the obstcles to avoid obstructing the safe paths when found.
2. Add the docking station as a static obstacle that can be spawned at the beginning of the runtime to avoid the noisy localization caused by collisions with the docking station.
3. Implement hybid frontier search approach to reduce redundant scan of the map.
4. Rename Collector node to AutoSLAM and Implement collector node.
5. Implement mapping completion check and create a mapping action server to manage the mapping task.
6. Automate switching from mapping to data collection mode using launchh script.
