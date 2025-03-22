
/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


/**
 * @typedef {Object} Tree
 * 
 * @property {Vertex[]} vertices - an array of vertex configurations
 * @property {number} newest - the index of the newest vertex in the tree
 */

/**
 * @typedef {Object} Vertex
 * 
 * @property {number[]} vertex - the configuration of the vertex
 * @property {number[]} edges - an array of neighboring vertex indices
 */





/**
 * Performs a single iteration of the RRT algorithm.
 * An asynchronous timing mechanism is used instead of a for loop to avoid
 * blocking and non-responsiveness in the browser.
 * 
 * @returns {string} - "failed" if the search fails on this iteration,
 *                     "succeeded" if the search succeeds on this iteration,
 *                     "extended" otherwise.
 */
function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

/**
 * Performs a single iteration of the RRT-Connect algorithm.
 * An asynchronous timing mechanism is used instead of a for loop to avoid
 * blocking and non-responsiveness in the browser.
 * 
 * @returns {string} - "failed" if the search fails on this iteration,
 *                     "succeeded" if the search succeeds on this iteration,
 *                     "extended" otherwise.
 */
function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

/**
 * Performs a single iteration of the RRT* algorithm.
 * 
 * @returns {string} - "failed" if the search fails on this iteration,
 *                     "succeeded" if the search succeeds on this iteration,
 *                     "extended" otherwise.
 */
function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath



/**
 * RRT Extend Function, will extend the tree T towards configuration q
 * 
 * @param {Tree} T - The tree to extend
 * @param {number[]} q - The configuration to extend towards
 * @returns {string} - "failed" if the search fails on this iteration,
 *                     "succeeded" if the search succeeds on this iteration,
 *                     "extended" otherwise.
 */
function extendRRT(T, q) {
    // Find the nearest neighbor in T to q
    let nearestIdx = findNearestNeighbor(T, q);
    let q_near = T.vertices[nearestIdx].vertex;
    
    // Generate a new configuration by moving from q_near towards q
    let q_new = newConfig(q_near, q);
    
    // Check if the path to q_new is collision-free
    if (!testCollision(q_new)) {
        // Add the new vertex and edge to the tree
        insertTreeVertex(T, q_new);
        insertTreeEdge(T, nearestIdx, T.newest);
        
        // Check if we've reached q
        if (distance(q_new, q) < eps) {
            return "reached";
        } else {
            return "advanced";
        }
    }
    
    return "trapped";
}

/**
 * Connects the tree T to configuration q
 * 
 * @param {Tree} T - The tree to connect to
 * @param {number[]} q - The configuration to connect to
 * @returns {string} - "failed" if the search fails on this iteration,
 *                     "succeeded" if the search succeeds on this iteration,
 *                     "extended" otherwise.
 */
function connectRRT(T, q) {
}

/**
 * Returns a random configuration q
 * @returns {number[]} - The random configuration
 */
function randomConfig() {
    // Generate a random configuration in the 2D space
    // The range for our world is approximately [-1.8, 5.8] in both x and y
    return [
        Math.random() * 7.6 - 1.8,  // x between -1.8 and 5.8
        Math.random() * 7.6 - 1.8   // y between -1.8 and 5.8
    ];
}


/**
 * 
 * @param {number[]} q_near - The configuration to extend from
 * @param {number[]} q - The configuration to extend to
 * @returns {number[]} - The new configuration
 */
function newConfig(q_near, q) {
    // Check if the distance between q_near and q is less than eps
    let d = distance(q_near, q);

    if (d < eps) {
        return q; // Return q if it's already close enough
    } else {
        // Move from q_near towards q by distance eps
        let scale = eps / d;
        return [
            q_near[0] + (q[0] - q_near[0]) * scale,
            q_near[1] + (q[1] - q_near[1]) * scale
        ];
    }
}

/**
 * 
 * @param {Tree} T - The tree to search in
 * @param {number[]} q - The configuration to find the nearest neighbor to
 * @returns {number[]} - The nearest neighbor to q in T
 */
function findNearestNeighbor(T, q) {    
    let minDist = Infinity;
    let nearestIdx = -1;
    
    // Find the vertex in T with the minimum distance to q
    for (let i = 0; i < T.vertices.length; i++) {
        let q_near = T.vertices[i].vertex;
        let dist = distance(q_near, q);
        
        if (dist < minDist) {
            minDist = dist;
            nearestIdx = i;
        }
    }
    
    return nearestIdx;
}

// Helper function to calculate Euclidean distance between two configurations
function distance(q1, q2) {
    return Math.sqrt(
        Math.pow(q1[0] - q2[0], 2) + 
        Math.pow(q1[1] - q2[1], 2)
    );
}

/**
 * Performs a depth-first search to find a path in the tree.
 * 
 * @returns {number[][]} - The path found.
 */
function dfsPath() {

}
