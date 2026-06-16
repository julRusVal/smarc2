ros2 action send_goal /floatsam_usv_0/go_in_formation smarc_msgs/action/BaseAction "{goal: {data: '{\"desired_speed\": 2.0, \"tracks\": [[{\"latitude\": 58.8405163983891, \"longitude\": 17.6517780965932}, {\"latitude\": 58.8406214504201, \"longitude\": 17.6519172412154}], [{\"latitude\": 58.8407163983891, \"longitude\": 17.6517780965932}, {\"latitude\": 58.8409163983891, \"longitude\": 17.6517780965932}]]}'}}"


ros2 action send_goal /floatsam_usv_0/go_in_formation smarc_msgs/action/BaseAction \
"{goal: {data: '{\"desired_speed\": 2.0, \"track\": [{\"latitude\": 58.8404946676474, \"longitude\": 17.6518338919726}, {\"latitude\": 58.8405449682116, \"longitude\": 17.6518419859228}, {\"latitude\": 58.8405926660047, \"longitude\": 17.6518436714537}, {\"latitude\": 58.840588636238, \"longitude\": 17.651973506389}, {\"latitude\": 58.8405855492816, \"longitude\": 17.6520711146255}]}'}}"








58.8404946676474 17.6518338919726
58.8405449682116 17.6518419859228
58.8405926660047 17.6518436714537
58.840588636238 17.651973506389
58.8405855492816 17.6520711146255



58.8404973621284 17.6517269223936
58.84057615482 17.6517320009233
58.8406244825704 17.6517697066712
58.8406329057627 17.6518488029239
58.8406262750795 17.6520694593868



ros2 action send_goal /floatsam_usv_0/go_in_formation smarc_msgs/action/BaseAction \
"{goal: {data: '{\"desired_speed\": 2.0, \"track\": [{\"latitude\": 58.823066, \"longitude\": 17.634014}, 
{\"latitude\": 58.822197, \"longitude\": 17.634064}, 
{\"latitude\": 58.821418, \"longitude\": 17.634105}]}'}}"


