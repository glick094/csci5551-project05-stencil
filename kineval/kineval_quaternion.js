//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply



// **** Function stencils are provided below, please uncomment and implement them ****//


/**
 * Computes a quaternion from axis and angle
 * 
 * @param {number[]} axis - The axis of rotation (should be len 3).
 * @param {number} angle - The angle.
 * 
 * @returns {{a: number, b: number, c: number, d: number}} The quaternion.
 */
kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    // normalize axis before calculating quaternion?
    axis_norm = Math.sqrt(axis[0]*axis[0]+axis[1]*axis[1]+axis[2]*axis[2]);
    q.a = Math.cos(angle/2);
    q.b = axis[0]*Math.sin(angle/2)/axis_norm;
    q.c = axis[1]*Math.sin(angle/2)/axis_norm;
    q.d = axis[2]*Math.sin(angle/2)/axis_norm;
    return q;
}

/**
 * Normalizes a quaternion
 * 
 * @param {{a: number, b: number, c: number, d: number}} q1 - a quaternion.
 * 
 * @returns {{a: number, b: number, c: number, d: number}} The normalized quaternion
 */
kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    var norm = Math.sqrt(q1.a*q1.a+q1.b*q1.b+q1.c*q1.c+q1.d*q1.d);
    q.a = q1.a/norm;
    q.b = q1.b/norm;
    q.c = q1.c/norm;
    q.d = q1.d/norm;
    return q;
}

/**
 * 
 * @param {{a: number, b: number, c: number, d: number}} q1 - a quaternion.
 * @param {{a: number, b: number, c: number, d: number}} q2 - another quaternion.
 * 
 * @returns {{a: number, b: number, c: number, d: number}} The product of the two quaternions.
 */
kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    q.a = q1.a*q2.a - q1.b*q2.b - q1.c*q2.c - q1.d*q2.d;
    q.b = q1.a*q2.b + q1.b*q2.a + q1.c*q2.d - q1.d*q2.c;
    q.c = q1.a*q2.c - q1.b*q2.d + q1.c*q2.a + q1.d*q2.b;
    q.d = q1.a*q2.d + q1.b*q2.c - q1.c*q2.b + q1.d*q2.a;
    return q;    

}

/**
 * 
 * @param {{a: number, b: number, c: number, d: number}} q - a quaternion.
 * 
 * @returns {number[][]} The 4x4 rotation matrix.
 */
kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
    // returns 4-by-4 2D rotation matrix
    var R = [[],[],[],[]];
    R[0][0] = 1 - 2*q.c*q.c - 2*q.d*q.d;
    R[0][1] = 2*q.b*q.c - 2*q.d*q.a;
    R[0][2] = 2*q.b*q.d + 2*q.c*q.a;
    R[0][3] = 0;
    R[1][0] = 2*q.b*q.c + 2*q.d*q.a;
    R[1][1] = 1 - 2*q.b*q.b - 2*q.d*q.d;
    R[1][2] = 2*q.c*q.d - 2*q.b*q.a;
    R[1][3] = 0;
    R[2][0] = 2*q.b*q.d - 2*q.c*q.a;
    R[2][1] = 2*q.c*q.d + 2*q.b*q.a;
    R[2][2] = 1 - 2*q.b*q.b - 2*q.c*q.c;
    R[2][3] = 0;
    R[3][0] = 0;
    R[3][1] = 0;
    R[3][2] = 0;
    R[3][3] = 1;
    return R;
}