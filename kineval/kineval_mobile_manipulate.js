kineval.robotMobileManipulateIterate = function robotMobileManipulateIterate() {
    if (!kineval.params.persist_mobile_manipulate_traversal) {
        return;
    }

    if (typeof robotMobileManipulateIterate.mm_state == 'undefined') {
        robotMobileManipulateIterate.mm_state = 'rrt_to_cube_init';
    }
    
    if (robotMobileManipulateIterate.mm_state == 'rrt_to_cube_init') {
        // TODO: initialize rrt as in kineval.robotRRTPlannerInit() and move to next state
    } else if (robotMobileManipulateIterate.mm_state == 'rrt_to_cube_iterate') {
        // TODO: perform a single iteration of rrt with robot_rrt_planner_iterate()
        //   if rrt is complete, move to next state
    } else if (robotMobileManipulateIterate.mm_state == 'rrt_to_cube_traverse') {
        if (Date.now() - cur_time < 500) {
            return;
        }
        cur_time = Date.now();
        // TODO: traverse the planned motion trajectory (above code ensures you traverse slowly enough)
        //   if traversal is complete, rrt trees/motion plan and move to next state
    } else if (robotMobileManipulateIterate.mm_state == 'ik_to_cube') {
        if (Date.now() - cur_time < 50) {
            return;
        }
        cur_time = Date.now();
        // TODO: perform a single iteration of ik with kineval.iterateIK() to move the eef towards the cube
        //   if ik is complete, move to next state
    } else if (robotMobileManipulateIterate.mm_state == 'grasp_cube') {
        // TODO: call kineval.graspObject() to grasp the cube
    } else if (robotMobileManipulateIterate.mm_state == 'rrt_to_bubble_init') {
        // TODO: initialize rrt as in kineval.robotRRTPlannerInit() and move to next state
    } else if (robotMobileManipulateIterate.mm_state == 'rrt_to_bubble_iterate') {
        // TODO: perform a single iteration of rrt with robot_rrt_planner_iterate()
    } else if (robotMobileManipulateIterate.mm_state == 'rrt_to_bubble_traverse') {
        if (Date.now() - cur_time < 500) {
            return;
        }
        cur_time = Date.now();
        // TODO: traverse the planned motion trajectory (above code ensures you traverse slowly enough)
    } else if (robotMobileManipulateIterate.mm_state == 'ik_to_bubble') {
        if (Date.now() - cur_time < 50) {
            return;
        }
        cur_time = Date.now();
        // TODO: perform a single iteration of ik with kineval.iterateIK() to move the eef towards the bubble
    } else if (robotMobileManipulateIterate.mm_state == 'release_cube') {
        // TODO: call kineval.releaseObject() to release the cube
    } else if (robotMobileManipulateIterate.mm_state == 'complete') {
        // DO NOT DO ANYTHING HERE
    }
}