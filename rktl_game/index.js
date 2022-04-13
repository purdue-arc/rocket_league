// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({
    url: 'ws://172.17.0.2:9090'
});

ros.on('connection', function () {
    console.log('Connected to websocket server.');
});

ros.on('error', function (error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function () {
    console.log('Connection to websocket server closed.');
});

// Setting cars to disabled
// ------------------

// var carEnable = new ROSLIB.Topic({
//     ros: ros,
//     name: '/cars/enable',
//     messageType: 'std_msgs/Bool'
// });

// var carBool = new ROSLIB.Message({
//     data: false
// });

// carEnable.publish(carBool);

// Subscribing to a Topic
// ----------------------
// Subscribe to the score keeper node
var scoreListener = new ROSLIB.Topic({
    ros: ros,
    name: '/score',
    messageType: 'rktl_msgs/Score'
});

var stateListener = new ROSLIB.Topic({
    ros: ros,
    name: '/match_status',
    messageType: 'rktl_msgs/MatchStatus'
});

var clockListener = new ROSLIB.Topic({
    ros: ros,
    name: '/game_clock',
    messageType: 'std_msgs/Float32'
});

// // Calling a service
// // -----------------

// var addTwoIntsClient = new ROSLIB.Service({
//     ros: ros,
//     name: '/add_two_ints',
//     serviceType: 'rospy_tutorials/AddTwoInts'
// });

// var request = new ROSLIB.ServiceRequest({
//     a: 1,
//     b: 2
// });

// addTwoIntsClient.callService(request, function (result) {
//     console.log('Result for service call on ' +
// addTwoIntsClient.name +
// ': ' +
// result.sum);
// });

// Getting and setting a param value
// ---------------------------------

ros.getParams(function (params) {
    console.log(params);
});

var maxVelX = new ROSLIB.Param({
    ros: ros,
    name: 'max_vel_y'
});

maxVelX.set(0.8);
maxVelX.get(function (value) {
    console.log('MAX VAL: ' + value);
});

// updating the score on a timer
const interval = setInterval(function () {
    listener.subscribe(function (message) {
        document.getElementById('blue-score').innerHTML = message.blue;
        document.getElementById('orange-score').innerHTML = message.orange;
    });
}, 1000);

// updating game clock while game is running
const clock = setInterval(function () {
    listener