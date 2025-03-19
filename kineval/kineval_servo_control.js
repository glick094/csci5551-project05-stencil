
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints

    // HARD CODE YOUR SETPOINTS HERE IF YOU WANT TO
    // kineval.setpoints = [];
    // kineval.setpoints: setpoint of the available dance poses of the robot;
    // kineval.params.dance_sequence_index: a sequence of indices pointing to the kineval.setpoints;
    // kineval.params.dance_pose_index: index to kineval.params.dance_sequence_index
    // kineval.params.setpoint_target: the current goal setpoint of the dance.

    // // this function should set kineval.params.setpoint_target
    kineval.setpoints = [{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":-1.7000000000000013,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0.7700000000000005},{"clavicle_right_yaw":0,"shoulder_right_yaw":-1.7000000000000013,"upperarm_right_pitch":-1.9000000000000015,"forearm_right_yaw":0,"clavicle_left_roll":0.7700000000000005},{"clavicle_right_yaw":-1.350000000000001,"shoulder_right_yaw":-1.7000000000000013,"upperarm_right_pitch":-1.9000000000000015,"forearm_right_yaw":0.7700000000000005,"clavicle_left_roll":0.7700000000000005},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0}];
    var cur_date = new Date();
    kineval.params.dance_pose_index = (cur_date.getSeconds()) % 4;

    // kineval.params.dance_pose_index = (kineval.params.dance_pose_index + 1) % kineval.params.dance_sequence_index.length;
    setpoint_index = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];
    kineval.params.setpoint_target = kineval.setpoints[setpoint_index];


}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints 
    // you should be setting the joint control attribute
    for (x in robot.joints) {
        robot.joints[x].control = (
            robot.joints[x].servo.p_gain *
            (kineval.params.setpoint_target[x] - robot.joints[x].angle)
        );
    }

}


