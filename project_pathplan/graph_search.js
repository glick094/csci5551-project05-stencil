/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

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

let visit_queue; 

class SearchAlgorithm {
    constructor() {
        this.visit_queue = []; 
    }

    initQueue(G, q_init, eps) {
        for (let iind = 0; iind < G.length; iind++) {
            for (let jind = 0; jind < G[iind].length; jind++) {
                let node = G[iind][jind]; 
                // check if node is on in the same cell as start
                if (Math.abs(node.x - q_init[0]) < eps/2 && Math.abs(node.y - q_init[1]) < eps/2) {
                    node.distance = 0; 
                    node.queued = true; 
                    //add node to search queue
                    this.visit_queue.push(node); 
                    // update global visit_queue
                    visit_queue = this.visit_queue; 
                    return; 
                }
            }
        }
    }

    getNextNode() {
        throw new Error('getNextNode must be implemented!!!');
    }

    addToQueue() {
        throw new Error('addToQueue must be implemented!!!');
    }
}

//below we implement our specific search algorithms
//the main difference is that we simply update the getNextNode, since algorithms differ by 

class DepthFirstSearch extends SearchAlgorithm {
    getNextNode() {
        const node = this.visit_queue.pop(); 
        // update global visit_queue
        visit_queue = this.visit_queue; 
        return node; 
    }

    addToQueue(node) {
        this.visit_queue.push(node); 
    }
}

class BreadthFirstSearch extends SearchAlgorithm {
    getNextNode() {
        const node = this.visit_queue.shift(); 
        // update global visit_queue
        visit_queue = this.visit_queue; 
        return node; 
    }

    addToQueue(node) {
        this.visit_queue.push(node); 
    }
}

class DijkstraSearch extends SearchAlgorithm {
    getNextNode() {
        let minIndex = 0; 
        for (let i = 1; i < this.visit_queue.length; i++) {
            if (this.visit_queue[i].distance < this.visit_queue[minIndex].distance) {
                minIndex = i; 
            }
        }
        const node = this.visit_queue.splice(minIndex, 1)[0]; 
        // update global visit_queue
        visit_queue = this.visit_queue; 
        return node; 
    }

    addToQueue(node) {
        let i = 0; 
        while (i < this.visit_queue.length && this.visit_queue[i] < node.distance) {
            i++;
        }
        this.visit_queue.splice(i, 0, node); 
    }
}

class AStarSearch extends SearchAlgorithm {
    getNextNode() {
        let minIndex = 0;
        for (let i = 1; i < this.visit_queue.length; i++) {
            if (this.visit_queue[i].priority < this.visit_queue[minIndex].priority) {
                minIndex = i;
            }
        }
        const node = this.visit_queue.splice(minIndex, 1)[0];
        visit_queue = this.visit_queue;
        return node;
    }

    addToQueue(node) {
        // f(n) = g(n) + h(n)
        // g(n) is node.distance (cost from start)
        // h(n) is heuristic (estimated cost to goal)
        const heuristic = this.calculateHeuristic(node);
        node.priority = node.distance + heuristic;
        
        // Insert maintaining ascending priority order
        let i = 0;
        while (i < this.visit_queue.length && this.visit_queue[i].priority < node.priority) {
            i++;
        }
        this.visit_queue.splice(i, 0, node);
        visit_queue = this.visit_queue;
    }

    calculateHeuristic(node) {
        // Using Euclidean distance as heuristic
        const dx = node.x - q_goal[0];
        const dy = node.y - q_goal[1];
        return Math.sqrt(dx*dx + dy*dy);
    }
}

let searchAlgo; 

function initSearchGraph() {
    /**
     * This function is called to initialize the search graph.
     * Graph G is initialized and you should add the start node to the visit_queue.
     * 
     */


    // this is the search queue you will use to implement the search
    // in the iterateGraphSearch function
    // visit_queue = [];
    // NOTE: we update visit_queue after initializing our search algorithm

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0,xpos=-2;xpos<7;iind++,xpos+=eps) {
        G[iind] = [];
        for (jind=0,ypos=-2;ypos<7;jind++,ypos+=eps) {

            // this is a two-dimensional array of objects
            // think of them as python dictionaries
            G[iind][jind] = {
                i:iind,j:jind, // mapping to graph array
                x:xpos,y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };

            // STENCIL: determine whether this graph node should be the start
            //   point for the search
            // refer to infrastructure.js for the start and goal variables
            // specifically initSearch() function will have everything you need
            // NOTE: this is checked in initQueue. 

        }
    }

    // initialize the search algorithm
    switch (search_alg) {
        case "depth-first": 
            searchAlgo = new DepthFirstSearch(); 
            break; 
        case "breadth-first": 
            searchAlgo = new BreadthFirstSearch(); 
            break; 
        case "greedy-best-first":
            searchAlgo = new DijkstraSearch(); 
            break; 
        case "A-star":
            searchAlgo = new AStarSearch(); 
            break; 
        default: 
            searchAlgo = new DepthFirstSearch();
            break;  
    }

    searchAlgo.initQueue(G, q_init, eps); 
}

function iterateGraphSearch() {
    /**
     * This function runs a single iteration of a graph search algorithm.
     * 
     * 
     */


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a highlighted path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    // If queue is empty, search has failed
    if (searchAlgo.visit_queue.length === 0) {
        return "failed";
    }

    // Get next node to explore (for DFS, pop from end of queue)
    const current = searchAlgo.getNextNode(); 
    
    // Skip if node has been visited
    if (current.visited) {
        return "iterating";
    }

    // Mark current node as visited
    current.visited = true;
    search_visited++;

    // Draw current node
    draw_2D_configuration([current.x, current.y], "visited");

    // Check if we've reached the goal
    if (Math.abs(current.x - q_goal[0]) < eps/2 && Math.abs(current.y - q_goal[1]) < eps/2) {
        drawHighlightedPathGraph(current);
        search_iterate = false;
        return "succeeded";
    }

    const directions = [
        [1,0], //E
        [-1,0], //W
        [0,1], //S
        [0,-1] //N
    ];

    for (let [di, dj] of directions) {
        let ni = current.i + di;
        let nj = current.j + dj;

        // Check if neighbor indices are within bounds
        if (ni >= 0 && ni < G.length && nj >= 0 && nj < G[0].length) {
            let neighbor = G[ni][nj];
            
            // Skip if neighbor has been visited or queued
            if (neighbor.visited || neighbor.queued) {
                continue;
            }

            // Check if neighbor position is in collision
            if (testCollision([neighbor.x, neighbor.y])) {
                continue;
            }

            // Calculate new distance
            let dx = neighbor.x - current.x;
            let dy = neighbor.y - current.y;
            let step_cost = Math.sqrt(dx*dx + dy*dy);
            let new_distance = current.distance + step_cost;

            // If we found a better path to this neighbor
            if (new_distance < neighbor.distance) {
                neighbor.distance = new_distance;
                neighbor.parent = current;
                neighbor.queued = true;
                
                searchAlgo.addToQueue(neighbor); 
                draw_2D_configuration([neighbor.x, neighbor.y], "queued");
            }
        }
    }

    return "iterating";

}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.
