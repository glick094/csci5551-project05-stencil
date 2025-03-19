
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
}


/**
 * 
 * @param {number[]} q_near - The configuration to extend from
 * @param {number[]} q - The configuration to extend to
 * @returns {number[]} - The new configuration
 */
function newConfig(q_near, q) {
}

/**
 * 
 * @param {Tree} T - The tree to search in
 * @param {number[]} q - The configuration to find the nearest neighbor to
 * @returns {number[]} - The nearest neighbor to q in T
 */
function findNearestNeighbor(T, q) {    
}

/**
 * Performs a depth-first search to find a path in the tree.
 * 
 * @returns {number[][]} - The path found.
 */
function dfsPath() {

}
