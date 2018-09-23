# ROS Image Publisher

This node publishes all the images contained in an specified folder, in the specified topic.
The images will be published in BGR8 format.

## Setup

1. Clone this repo
1. Execute `catkin_make`
1. Source the workspace `source devel/setup.py`

If you want to integrate this into another catkin workspace, just copy the `image_folder_publisher` into the `src` target workspace.

## How to Launch

Once in a sourced terminal, execute:
`rosrun image_folder_publisher image_folder_publisher.py`

## Params

|Param name    | Type   | Description                                             | Default Value |
|------------- |--------|-------------------------------------------------------  |---------------|
|`topic_name`  | String | Name of the topic to publish the image stream           | `image_raw`   |
|`publish_rate`| Integer| Frame rate in Hz to publish the image.                  | `10`          |
|`sort_files`  | Boolean| Defines if the files will be sorted before publishing   | `True`        |
|`frame_id`    | String | Sets the frame_id contained in the Image message header | `camera`      |
|`image_folder`| String | Path to the folder containing the images to be published|               | 

## Notes

* Invalid files will be skipped. The supported image formats will correspond to the ones supported by CvBridge and OpenCV.
* Publish Rate will depend on the speed of the Hard disk. 