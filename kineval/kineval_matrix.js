//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i,j;

    for (i=0;i<m1.length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m1[0].length;j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//



/**
 * returns 2D array that is the result of the matrix multiplication m1*m2
 * 
 * @param {number[][]} m1 - The first matrix to multiply
 * @param {number[][]} m2 - The second matrix to multiply
 * @returns {number[][]} - The result of the matrix multiplication
 */
function matrix_multiply(m1, m2) {
    /**
     * you should make a NEW 2D array to store the result of the matrix multiplication
     * 
     * This is a stencil for project 2 CSCI 5551
     */


    // we do so much 4x4 * 4x4 that writing this out by hand does provide significant performance improvements
    // based on https://github.com/gfxfundamentals/webgl-fundamentals/blob/master/webgl/resources/m4.js#L91
    if (m1.length === 4 && m1[0].length === 4 && m2.length === 4 && m2[0].length === 4) {
        return [
            [
                m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0] + m1[0][3] * m2[3][0],
                m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1] + m1[0][3] * m2[3][1],
                m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2] + m1[0][3] * m2[3][2],
                m1[0][0] * m2[0][3] + m1[0][1] * m2[1][3] + m1[0][2] * m2[2][3] + m1[0][3] * m2[3][3],
            ],
            [
                m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0] + m1[1][3] * m2[3][0],
                m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1] + m1[1][3] * m2[3][1],
                m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2] + m1[1][3] * m2[3][2],
                m1[1][0] * m2[0][3] + m1[1][1] * m2[1][3] + m1[1][2] * m2[2][3] + m1[1][3] * m2[3][3],
            ],
            [
                m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0] + m1[2][3] * m2[3][0],
                m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1] + m1[2][3] * m2[3][1],
                m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2] + m1[2][3] * m2[3][2],
                m1[2][0] * m2[0][3] + m1[2][1] * m2[1][3] + m1[2][2] * m2[2][3] + m1[2][3] * m2[3][3],
            ],
            [
                m1[3][0] * m2[0][0] + m1[3][1] * m2[1][0] + m1[3][2] * m2[2][0] + m1[3][3] * m2[3][0],
                m1[3][0] * m2[0][1] + m1[3][1] * m2[1][1] + m1[3][2] * m2[2][1] + m1[3][3] * m2[3][1],
                m1[3][0] * m2[0][2] + m1[3][1] * m2[1][2] + m1[3][2] * m2[2][2] + m1[3][3] * m2[3][2],
                m1[3][0] * m2[0][3] + m1[3][1] * m2[1][3] + m1[3][2] * m2[2][3] + m1[3][3] * m2[3][3],
            ]
        ];
    }

    var mat = [];
    var i,j,k;

    for (i=0;i<m1.length;i++) {
        mat[i] = [];
        for (j=0;j<m2[0].length;j++) {
            mat[i][j] = 0;
            for (k=0;k<m1[0].length;k++) {
                mat[i][j] += m1[i][k] * m2[k][j];
            }
        }
    }
    return mat;


}


/**
 * Matrix transpose function
 * 
 * 
 * @param {number[][]} m - a matrix to transpose
 * @returns {number[][]} - The transpose of the input matrix
 */
function matrix_transpose(m) {
    /**
     * Here you should implement the matrix transpose function
     * 
     * The transpose of a matrix is a new matrix whose rows are the columns of the original.
     * A[i][j] of the original matrix becomes A[j][i] in the transposed matrix
     * 
     * This is a stencil for project 2 CSCI 5551
     * 
     */


    var mat = [];
    var i,j;

    for (i=0;i<m[0].length;i++) { // for each row of m1
        mat[i] = [];
        for (j=0;j<m.length;j++) { // for each column of m1
            mat[i][j] = m[j][i];
        }
    }
    return mat;


}




/**
 * Calculates and returns the pseudoinverse of a matrix
 * 
 * @param {number[][]} m - an input matrix
 * @returns {number[][]} - The pseudoinverse of the input matrix
 */
function matrix_pseudoinverse(m) {
    /**
     * Here you should implement the calculation of the pseudoinverse of the matrix m
     * 
     */

    if (m[0].length == m.length) {
        return numeric.inv(m);
    }
    else if (m[0].length < m.length) {
        var mt = matrix_transpose(m);
        var square_mat = matrix_multiply(mt,m);  // this square matrix will be inverted
        return matrix_multiply(numeric.inv( square_mat ), mt );
    }
    else {
        mt = matrix_transpose(m);
        square_mat = matrix_multiply(m,mt);  // this square matrix will be inverted
        return matrix_multiply(mt, numeric.inv( square_mat ) );
    }

}

/**
 * Calculates and returns the invert affine of a 4-by-4 matrix
 * 
 * @param {number[][]} m - an input matrix
 * @returns {number[][]} - The invert affine of 4-by-4 matrix m
 */
// function matrix_invert_affine(m) {
    /**
     * Here you should implement the invert affine of a 4-by-4 matrix
     * 
     * 
     * This function is NOT required for project 2 CSCI 5551. Motivated students are encouraged to implement it.
     * 
     */

// }

/**
 * Normalizes a vector v
 * 
 * @param {number[]} v - The input vector
 * @returns {number[]} 
 */
function vector_normalize(v) {
    /**
     * Here you should implement the normalization of a vector
     * 
     * Formula: v_normalized = v / |v|
     * 
     * This is a stencil for project 2 CSCI 5551
     */


    var norm = 0;
    for (i=0;i<v.length;i++) {
        norm += v[i]*v[i];
    }
    norm = Math.sqrt(norm);
    var vec = [];
    for (i=0;i<v.length;i++) {
        vec[i] = v[i]/norm;
    }
    return vec;
    
}

/**
 * Calculates the cross product of two vectors
 * 
 * @param {number[]} a 
 * @param {number[]} b 
 */
function vector_cross(a,b) {
    /**
     * Here you should implement the cross product of two vectors
     * 
     * This is a stencil for project 2 CSCI 5551
     */


    var vec = [];
    vec[0] = a[1]*b[2] - a[2]*b[1];
    vec[1] = a[2]*b[0] - a[0]*b[2];
    vec[2] = a[0]*b[1] - a[1]*b[0];
    return vec;
    
}

/**
 * Generates an identity matrix
 * 
 * @param {number} n - The size of the identity matrix
 * 
 */
function generate_identity(n=4) {
    /**
     * Here you should implement the generation of an identity matrix
     * 
     * This is a stencil for project 2 CSCI 5551
     */

    if (n == 4) {
        return [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ];
    }

    
    var mat = [];
    var i;

    for (i=0;i<n;i++) { // for each row of m1
        mat[i] = new Array(n).fill(0);
        mat[i][i] = 1;
    }

    return mat;

}

/**
 * Generates a translation matrix(4x4) for the given translation values
 * 
 * @param {number} tx - The translation value in x-axis
 * @param {number} ty - The translation value in y-axis
 * @param {number} tz - The translation value in z-axis
 */
function generate_translation_matrix(tx, ty, tz) {
    /**
     * Here you should implement the generation of a translation matrix
     * 
     * This is a stencil for project 2 CSCI 5551
     */


    var mat = [
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1],
    ]
    return mat
    
}

/**
 * Generates a rotation matrix around the x-axis
 * 
 * @param {number} angle - The angle of rotation in radians
 */
function generate_rotation_matrix_X(angle) {
    /**
     * Here you should implement the generation of a rotation matrix around the x-axis
     * 
     * This is a stencil for project 2 CSCI 5551
     */

    var mat = [
        [1, 0, 0, 0],
        [0, Math.cos(angle), -Math.sin(angle), 0],
        [0, Math.sin(angle), Math.cos(angle), 0],
        [0, 0, 0, 1],
    ]
    return mat
    
}


/**
 * Generates a rotation matrix around the y-axis
 * 
 * @param {number} angle - The angle of rotation in radians
 */
function generate_rotation_matrix_Y(angle) {
    /**
     * Here you should implement the generation of a rotation matrix around the y-axis
     * 
     * This is a stencil for project 2 CSCI 5551
     */

    var mat = [
        [Math.cos(angle), 0, Math.sin(angle), 0],
        [0, 1, 0, 0],
        [-Math.sin(angle), 0, Math.cos(angle), 0],
        [0, 0, 0, 1],
    ]
    return mat
    
}

/**
 * Generates a rotation matrix around the z-axis
 * 
 * @param {number} angle - The angle of rotation in radians
 */
function generate_rotation_matrix_Z(angle) {
    /**
     * Here you should implement the generation of a rotation matrix around the z-axis
     * 
     * This is a stencil for project 2 CSCI 5551
     */

    var mat = [
        [Math.cos(angle), -Math.sin(angle), 0, 0],
        [Math.sin(angle), Math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
    return mat


    
}