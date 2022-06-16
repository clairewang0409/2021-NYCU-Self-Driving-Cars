Competition I:
    Change the file path:
    (Line 45) string map_path ="Your map file path";
    (Line 46) string csv_path ="Your csv file path";
    
    Run the launch and bag file:
    roslaunch localization1.launch
    (It would take a while for initialization, and then you will see "Now can play the bag file!" at terminal.)
    rosbag play -r 0.01 sdc_localization_1.bag 


Competition II:
    Change the file path:
    (Line 45) string map_path ="Your map file path";
    (Line 46) string csv_path ="Your csv file path";
    
    Run the launch and bag file:
    roslaunch localization2.launch
    (It would take a while for initialization, and then you will see "Now can play the bag file!" at terminal.)
    rosbag play -r 0.01 sdc_localization_2_lite.bag 


Competition III:
    Change the file path:
    (Line 45) string map_path ="Your map file path";
    (Line 46) string csv_path ="Your csv file path";
    
    Run the launch and bag file:
    roslaunch localization3.launch
    (It would take a while for initialization, and then you will see "Now can play the bag file!" at terminal.)
    rosbag play -r 0.1 sdc_localization_3_lite.bag 





