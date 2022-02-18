#!/bin/bash -x

LOG_DIR=$(date +"%Y%m%d_%H%M%S.%N")
mkdir -p $LOG_DIR

rosparam dump > $LOG_DIR/rosparam.yaml

topics=(
  /tf
  # panda_control
  /joint_states
  /tf_static_republished
  # rs_rgbd
  /camera/color/camera_info
  /camera/color/image_rect_color/compressed
  /camera/aligned_depth_to_color/camera_info
  /camera/aligned_depth_to_color/image_raw/compressedDepth
  # mask_rcnn_instance_segmentation
  /camera/color/image_rect_color_throttle/output
  /camera/mask_rcnn_instance_segmentation/output/label_ins
  /camera/mask_rcnn_instance_segmentation/output/class
  # octomap_server
  /camera/octomap_server/output/grids
  /camera/octomap_server/output/grids_noentry
  /camera/octomap_server/output/label_rendered
  /camera/octomap_server/output/class
  # singleview_3d_pose_estimation
  /singleview_3d_pose_estimation/output
  # object_mapping
  /object_mapping/output/poses
  /object_mapping/output/grids
  # demo_interface
  /demo_interface/debug/heightmap
  /demo_interface/debug/maskmap
)

rosbag record "${topics[@]}" -O $LOG_DIR/rosbag.bag $*
