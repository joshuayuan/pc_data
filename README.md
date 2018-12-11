in one panel run
```
roslaunch pc_data pc_data.launch
```
Then when you want coordinates, run in another panel
```
rosrun pc_data collector.py
```
They get saved to `pc_data/data/` as `final_cloud.pcd` and `final_positions.txt`
