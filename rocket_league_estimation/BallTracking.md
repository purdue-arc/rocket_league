<h1>Ball Tracking</h1>
The launch file is <code>ball_detection_to_pose.launch</code>. This should launch the code. 


Inside this, you will find the paramaters for the color, as well as the value for the height above the field (currently not implemented).

You should not need to mess with the min/max colors, but if you do, they are not on the same scale as a normal HSV color scale. 

Normally, hue is a number out of 360, specifically the degree on the color wheel, and the saturation and vibrance are percents, represending the percentage of the respective value. 

For OpenCV, these numbers are all represented on a scale of 0-255, so when adjusting this that will need to be kept in mind.

<code>ball_detection.cpp</code> is the main file that runs the node, and it has a <code>BallDetection</code> class and a <code>PHCModel</code> class.

This node publishes a PoseWithConvarianceStamped. It subscribes to the camera info as well as the picture. 

The node publishes a coordinate with the orgin being directly below the camera.

My understanding is that this would be run once inside each camera namespace, and that should work, but this may be figidity, we may also need to update the camera launch files. There may also be another, better method. If you need to, these lines are those which may need to be modified:

<code>BallDetection.cpp</code>

```cpp
52    posePub{nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
53        "ball_pose", 1)},(
54    detectionSub{nh.subscribe((
55        "image_rect_color", 1, &BallDetection::BallCallback, this)},
```
<code>PHCMode.cpp</code><br>
```cpp
23    cameraInfoSub{model_nh.subscribe(
24        "CameraInfo", 1, &PHCModel::PinHoleCallback, this)}
```
