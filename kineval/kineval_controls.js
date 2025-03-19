
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | update robot state from controls

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

/**
 * @typedef {Object} Robot
 * @property {string} name - The name of the robot.
 * @property {Object} origin - The initial pose of the robot in the world.
 * @property {number[]} origin.xyz - The position of the robot in the world (x, y, z).
 * @property {number[]} origin.rpy - The orientation of the robot in the world (roll, pitch, yaw).
 * @property {string} base - The base link of the robot.
 * @property {Object.<string, Object>} links - The links of the robot.
 * @property {Object.<string, Joint>} joints - The joints of the robot.
 * @property {EndEffector} endeffector - The end effector of the robot.
 * @property {Object} control - The control of the robot.
 * @property {number[]} control.xyz - The control of the robot in the world (x, y, z).
 * @property {number[]} control.rpy - The control of the robot in the world (roll, pitch, yaw).
 */

/**
 * @typedef {Object} Joint
 * @property {string} parent - The parent (inboard) link of the joint.
 * @property {string} child - The child (outboard) link of the joint.
 * @property {string[]} children - The children joints of the joint.
 * @property {Object} origin - The origin of the joint relative to the parent link.
 * @property {number[]} origin.xyz - The position of the joint relative to the parent link (x, y, z).
 * @property {number[]} origin.rpy - The orientation of the joint relative to the parent link (roll, pitch, yaw).
 * @property {number[]} axis - The axis of rotation for the joint.
 * @property {number} angle - The angle of the joint.
 * @property {number} control - The control input for the joint.
 * 
 * @property {Object} servo - The servo control parameters for the joint.
 * @property {number} servo.p_gain - The proportional gain of the servo control.
 * @property {number} servo.p_desired - The desired position of the servo control.
 * @property {number} servo.d_gain - The derivative gain of the servo control.
 * 
 * @property {Object} limit - The limit of the joint.
 * @property {number} limit.lower - The lower limit of the joint.
 * @property {number} limit.upper - The upper limit of the joint.
 * 
 */

/**
 * This function is applying the controls to the robot kinematics transforms and joint angles.
 * You should enforce joint limits for prismatic and revolute joints.
 * 
 * @param {Robot} curRobot 
 */
kineval.applyControls = function robot_apply_controls(curRobot) {
    // apply robot controls to robot kinematics transforms and joint angles, then zero controls
    // includes update of camera position based on base movement

    // update robot configuration from controls
    for (x in curRobot.joints) {

        // update joint angles
        if ( (typeof curRobot.joints[x].type !== 'undefined')
             || (typeof curRobot.joints[x].type !== 'fixed') ) {  // TODO look at this

            if (isNaN(curRobot.joints[x].control))
                console.warn("kineval: control value for " + x +" is a nan");

            curRobot.joints[x].angle += curRobot.joints[x].control;
        }

        // STENCIL: enforce joint limits for prismatic and revolute joints

        if (
            (curRobot.joints[x].type === 'revolute')
            || (curRobot.joints[x].type === 'prismatic')
        ) {
            curRobot.joints[x].angle = Math.min(
                Math.max(curRobot.joints[x].angle, curRobot.joints[x].limit.lower),
                curRobot.joints[x].limit.upper
            )
        }

        // clear controls back to zero for next timestep
        curRobot.joints[x].control = 0;
    }

    // base motion
    curRobot.origin.xyz[0] += curRobot.control.xyz[0];
    curRobot.origin.xyz[1] += curRobot.control.xyz[1];
    curRobot.origin.xyz[2] += curRobot.control.xyz[2];
    curRobot.origin.rpy[0] += curRobot.control.rpy[0];
    curRobot.origin.rpy[1] += curRobot.control.rpy[1];
    curRobot.origin.rpy[2] += curRobot.control.rpy[2];

    // move camera with robot base
    camera_controls.object.position.x += curRobot.control.xyz[0];
    camera_controls.object.position.y += curRobot.control.xyz[1];
    camera_controls.object.position.z += curRobot.control.xyz[2];

    // zero controls now that they have been applied to robot
    curRobot.control = {xyz: [0,0,0], rpy:[0,0,0]}; 
}

