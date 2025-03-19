/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics 
     in HTML5/JavaScript and threejs
     
     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

     Updated and modified by coursestaff Adit Kadepurkar, Xun Tu, Mohit Yadav, Karthik Desingh of CSCI 5551 Spring Term - https://rpm-lab.github.io/CSCI5551-Spr25/
     Robotics: Perception and Manipulation Lab
     University of Minnesota

*/


///////////////////////////////////////////////////////

// Important API guide for the robot object and subobjects(may not be exhaustive):

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
 * @property {number[][]} xform - The transformation matrix of the joint.
 * 
 * @property {Object} limit - The limit of the joint.
 * @property {number} limit.lower - The lower limit of the joint.
 * @property {number} limit.upper - The upper limit of the joint.
 */

/**
 * @typedef {Object} Link
 * @property {string} name - The name of the link.
 * @property {string} parent - The parent joint of the link.
 * @property {string[]} children - The child joints of the link.
 * @property {number[][]} xform - The transformation matrix of the joint.
 */

/**
 * @typedef {Object} EndEffector
 * @property {string} frame - The frame of the end effector.
 * @property {number[][]} position - The position of the end effector.
 */

///////////////////////////////////////////////////////




kineval.initRobotJoints = function initRobotJoints() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert threejs scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not threejs

    var x,tempmat;

    for (x in robot.joints) {

        // give the joint its name as an id
        robot.joints[x].name = x;

        // initialize joint angle value and control input value
        if (typeof robot.joints[x].angle === 'undefined') {
            robot.joints[x].angle = 0;
        }
        if (typeof robot.joints[x].control === 'undefined') {
            robot.joints[x].control = 0;
        }
        if (typeof robot.joints[x].servo === 'undefined') {
            robot.joints[x].servo = {};
            //set appropriate servo gains for arm setpoint control
            robot.joints[x].servo.p_gain = 0.1; 
            robot.joints[x].servo.p_desired = 0;
            robot.joints[x].servo.d_gain = 0.01; 
        }


        /* STENCIL START */ 
        // STENCIL: complete kinematic hierarchy of robot for convenience.
        //   robot description only specifies parent and child links for joints.
        //   additionally specify parent and CHILDREN joints for each link
        //  
        // robot object is documented at the top of this file for reference
        // if the .children field is not defined for a joint, you will need to create an empty array


        robot.links[robot.joints[x].child].parent = x;
        if (robot.links[robot.joints[x].parent].children == undefined) {
            robot.links[robot.joints[x].parent].children = [];
        }
        robot.links[robot.joints[x].parent].children.push(x);



        /* STENCIL END */ 

    }  // end loop over robot.joints

}

