/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

     Updated and modified by coursestaff Adit Kadepurkar, Xun Tu, Mohit Yadav, Karthik Desingh of CSCI 5551 Spring Term - https://rpm-lab.github.io/CSCI5551-Spr25/
     Robotics: Perception and Manipulation Lab
     University of Minnesota

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/


/**
 * Computes the forward kinematics for the robot.
 * 
 * This function checks if the `buildFKTransforms` function is defined. If not, it updates the text bar to indicate that forward kinematics is not implemented and returns.
 * Otherwise, it proceeds to implement the forward kinematics by calling the `buildFKTransforms` function.
 */
kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: implement kineval.buildFKTransforms();

    kineval.buildFKTransforms();

}

// STENCIL: PLEASE READ
// 
//   reference code alternates recursive traversal over 
//   links and joints starting from base, by implementing the
//   following functions: 
//     traverseFKBase
//     traverseFKLink
//     traverseFKJoint
//
// user interface needs the heading (z-axis) and lateral (x-axis) directions
//   of robot base in world coordinates stored as 4x1 matrices in
//   global variables "robot_heading" and "robot_lateral"
//
// if geometries are imported and using ROS coordinates (e.g., fetch),
//   coordinate conversion is needed for kineval/threejs coordinates:
//

kineval.buildFKTransforms = function buildFKTransforms() {
    kineval.traverseFKBase(); // nothing to be done here
}


var robot_heading;
var robot_lateral;

/**
 * Traverses the base of the robot to compute its transformation matrix.
 * 
 * This function initializes the transformation matrix for the robot's base using its origin's position and orientation.
 * It also computes the robot's heading and lateral directions in world coordinates.
 * If the robot's geometries are imported and using ROS coordinates, it performs a coordinate conversion.
 * Finally, it calls `traverseFKLink` to continue the traversal from the base link.
 */
kineval.traverseFKBase = function traverseFKBase() {

    parent_xform = matrix_multiply(
        generate_translation_matrix(robot.origin.xyz[0], robot.origin.xyz[1], robot.origin.xyz[2]),
        matrix_multiply(
            generate_rotation_matrix_Z(robot.origin.rpy[2]),
            matrix_multiply(
                generate_rotation_matrix_Y(robot.origin.rpy[1]),
                generate_rotation_matrix_X(robot.origin.rpy[0])
            )
        )
    )
    robot_heading = matrix_multiply(parent_xform, [[0], [0], [1], [1]]);
    robot_lateral = matrix_multiply(parent_xform, [[1], [0], [0], [1]]);
    if (typeof robot.links_geom_imported !== 'undefined') {
        parent_xform = matrix_multiply(
            parent_xform,
            [
                [0, 1, 0, 0],
                [0, 0, 1, 0],
                [1, 0, 0, 0],
                [0, 0, 0, 1],
            ]
        )
    }
    kineval.traverseFKLink(robot.base, parent_xform);

}

/**
 * Traverses a link of the robot to compute its transformation matrix.
 * 
 * @param {string} linkname - The name of the link to traverse.
 * @param {number[][]} parent_xform - The transformation matrix of the parent link or joint.
 * 
 * This function sets the transformation matrix for the specified link and recursively traverses its child joints.
 */
kineval.traverseFKLink = function traverseFKLink(linkname, parent_xform) {
    robot.links[linkname].xform = parent_xform
    if (robot.links[linkname].children != undefined) {
        for (var i = 0; i < robot.links[linkname].children.length; i++) {
            kineval.traverseFKJoint(robot.links[linkname].children[i], robot.links[linkname].xform);
        }
    }
}


/**
 * Traverses a joint of the robot to compute its transformation matrix.
 * 
 * @param {string} joint_name - The name of the joint to traverse.
 * @param {number[][]} parent_xform - The transformation matrix of the parent link or joint.
 * 
 * This function computes the local transformation matrix, finds the joint_xform, 
 * and recursively traverses its child link.
 * 
 * joints have a .type property that should be used to determine the type of joint
 * 
 */

// notes: local_xform = TZYX
kineval.traverseFKJoint = function traverseFKJoint(joint_name, parent_xform) {
    local_xform = matrix_multiply(
        generate_translation_matrix(robot.joints[joint_name].origin.xyz[0], robot.joints[joint_name].origin.xyz[1], robot.joints[joint_name].origin.xyz[2]),
        matrix_multiply(
            generate_rotation_matrix_Z(robot.joints[joint_name].origin.rpy[2]),
            matrix_multiply(
                generate_rotation_matrix_Y(robot.joints[joint_name].origin.rpy[1]),
                generate_rotation_matrix_X(robot.joints[joint_name].origin.rpy[0])
            )
        )
    )
    
    // notes: joint_xform 
    if (robot.joints[joint_name].type === "prismatic") {
        joint_xform = generate_translation_matrix(
            robot.joints[joint_name].axis[0] * robot.joints[joint_name].angle,
            robot.joints[joint_name].axis[1] * robot.joints[joint_name].angle,
            robot.joints[joint_name].axis[2] * robot.joints[joint_name].angle
        );
    } else {
        joint_xform = kineval.quaternionToRotationMatrix(
            kineval.quaternionFromAxisAngle(
                robot.joints[joint_name].axis,
                robot.joints[joint_name].angle
            )
        );
    }

    // notes: xform = parent_xform(local_xform(joint_xform))
    robot.joints[joint_name].xform = matrix_multiply(
        parent_xform,
        matrix_multiply(
            local_xform,
            joint_xform
        )
    );
    kineval.traverseFKLink(robot.joints[joint_name].child, robot.joints[joint_name].xform);
}
