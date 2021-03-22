<h1>Ball Tracking</h1>
The launch file is <code>ball_detection_to_pose.launch</code>. This should launch the code. 


Inside this, you will find the paramaters for the color, as well as the value for the height above the field (currently not implemented).

You should not need to mess with the min/max colors, but if you do, they are not on the same scale as a normal HSV color scale. 

Normally, hue is a number out of 360, specifically the degree on the color wheel, and the saturation and vibrance are percents, represending the percentage of the respective value. 

For OpenCV, these numbers are all represented on a scale of 0-255, so when adjusting this that will need to be kept in mind.

<code>ball_detection.cpp</code> is the main file that runs the node, and it has a <code>BallDetection</code> class, along with a header file.

This node publishes a PoseWithConvarianceStamped. It subscribes to the camera info as well as the picture. 
<br><br>
Inside the launch file, there are a varity of paramaters, as listed below, along with their use:
<ul>
    <li>showImage, this boolean is whether or not you want OpenCV to display the camera image in a frame. Useful for debugging. </li>
    <li>cam_height, this is the height from the center of the ball to the camera sensor. The units of this number will be the units that the coordinate plane is in.</li>
    <li>The range of these are specified above, they control the color of the object we are looking for.</li>
    <ul>
        <li>min_hue</li>
        <li>min_sat</li>
        <li>min_vib</li>
        <li>max_hue</li>
        <li>max_sat</li>
        <li>max_vib</li>
    </ul>
</ul>
