
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    // STENCIL: see instructor for random time trial code
    // get endeffector Cartesian position in the world
    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
            Math.pow(kineval.params.ik_target.position[0][0]-endeffector_world[0][0],2.0)
            + Math.pow(kineval.params.ik_target.position[1][0]-endeffector_world[1][0],2.0)
            + Math.pow(kineval.params.ik_target.position[2][0]-endeffector_world[2][0],2.0) );

    // if target reached, increment scoring and generate new target location
    // KE 2 : convert hardcoded constants into proper parameters
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        kineval.params.trial_ik_random.targets += 1;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }
}


/**
 * 
 * @param {{position: number[], orientation: number[]}} endeffector_target_world - a target endeffector position and orientation in world frame
 * @param {string} endeffector_joint - endeffector joint name
 * @param {number[]} endeffector_position_local - endeffector position in local frame
 */
kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    // robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    // robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    // robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length

    // get the xform of the end effector
    ee_xform = matrix_multiply(
        robot.joints[endeffector_joint].xform,
        generate_translation_matrix(
            endeffector_position_local[0],
            endeffector_position_local[1],
            endeffector_position_local[2],
        )
    );

    // get the xyz and rpy of the end effector from the xform
    ee_xyz = [ee_xform[0][3], ee_xform[1][3], ee_xform[2][3]];
    ee_rpy = matrix_to_rpy(ee_xform);
    
    // calculate robot.dx with or without orientation
    if (kineval.params.ik_orientation_included) {
        robot.dx = [
            [endeffector_target_world.position[0] - ee_xyz[0]],
            [endeffector_target_world.position[1] - ee_xyz[1]],
            [endeffector_target_world.position[2] - ee_xyz[2]],
            [endeffector_target_world.orientation[0] - ee_rpy[0]],
            [endeffector_target_world.orientation[1] - ee_rpy[1]],
            [endeffector_target_world.orientation[2] - ee_rpy[2]],
        ];
    } else {
        robot.dx = [
            [endeffector_target_world.position[0] - ee_xyz[0]],
            [endeffector_target_world.position[1] - ee_xyz[1]],
            [endeffector_target_world.position[2] - ee_xyz[2]],
            [0],
            [0],
            [0],
        ];
    }

    // traverse up the kinematic heirarchy to get the joint names that will be in our jacobian
    joint = robot.joints[endeffector_joint];
    joint_names_in_jacobian = []
    while (true) {
        // push every joint except for fixed joints
        if (joint.type != "fixed") {
            joint_names_in_jacobian.push(joint.name);
        }
        parent_link = joint.parent;

        // end the loop
        if (joint.parent == robot.base) {
            break;
        }
        joint = robot.joints[robot.links[joint.parent].parent];
    }
    // reverse so we are going base -> endeffector
    joint_names_in_jacobian = joint_names_in_jacobian.reverse();

    // initialize the robot jacobian
    robot.jacobian = [];
    for (i_joint_name in joint_names_in_jacobian) {
        joint_name = joint_names_in_jacobian[i_joint_name];
        joint = robot.joints[joint_name];

        // get the joint_axis in the global frame
        joint_axis = matrix_multiply(
            joint.xform,
            [[joint.axis[0]], [joint.axis[1]], [joint.axis[2]], [0]]
        )
        joint_axis = [joint_axis[0][0], joint_axis[1][0], joint_axis[2][0]];

        // jacobian for prismatic is just joint_xform * joint_axis(as a 4x1 matrix)
        if (joint.type == "prismatic") {
            robot.jacobian.push([
                ...joint_axis,
                0, 0, 0
            ]);
        } else {
            // for all other joint types(note you wont be iterating through fixed joints here since they shouldnt be in the jacobian)
            // you just want to multiply the joint axis with the error between the joints translation and the ee translation, and concat with joint axis
            robot.jacobian.push([
                ...vector_cross(
                    joint_axis,
                    [
                        ee_xyz[0] - joint.xform[0][3],
                        ee_xyz[1] - joint.xform[1][3],
                        ee_xyz[2] - joint.xform[2][3]
                    ]
                ),
                ...joint_axis
            ]);
        }
    }

    // jacobian
    robot.jacobian = matrix_transpose(robot.jacobian);

    // calculate dq
    if (kineval.params.ik_pseudoinverse) {
        robot.dq = matrix_multiply(matrix_pseudoinverse(robot.jacobian), robot.dx);
    } else {
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian), robot.dx);
    }

    // update the controls
    for (i_joint_name in joint_names_in_jacobian) {
        joint_name = joint_names_in_jacobian[i_joint_name];
        joint = robot.joints[joint_name];
        joint.control += kineval.params.ik_steplength * robot.dq[i_joint_name][0];
    }

}

function matrix_to_rpy(matrix) {
    var rpy = [0, 0, 0];
    if (matrix[0][2] < 1)
        if (matrix[0][2] > -1)
        {
            rpy[1] = Math.asin(matrix[0][2]);
            rpy[0] = Math.atan2(-matrix[1][2],matrix[2][2]);
            rpy[2] = Math.atan2(-matrix[0][1],matrix[0][0]);
        }
        else
        {
            rpy[1] = -Math.PI/2;
            rpy[0] = -Math.atan2(matrix[1][0],matrix[1][1]);
            rpy[2] = 0;
        }
    else
    {
        rpy[1] = Math.PI/2;
        rpy[0] = Math.atan2(matrix[1][0],matrix[1][1]);
        rpy[2] = 0;
    }
    // rpy[0] = Math.atan2(matrix[2][1], matrix[2][2]);
    // rpy[1] = Math.atan2(-matrix[2][0], Math.sqrt(matrix[2][1] * matrix[2][1] + matrix[2][2] * matrix[2][2]));
    // rpy[2] = Math.atan2(matrix[1][0], matrix[0][0]);
    return rpy;
}


