// set rectangular boundary of robot's world as min and max locations
// collision only checked in x-z plane
robot_boundary = [[-5,0,-5],[5,0,5]];

// set spherical obstacles in robot's world
// with locations specified in homogeneous coordinates as 2D array
robot_obstacles = []; 

robot_obstacles[0] = {}; 
robot_obstacles[0].location = [[0],[0.5],[-2],[1]]; // in homg coords
robot_obstacles[0].radius = 1.0; 
robot_obstacles[1] = {}; 
robot_obstacles[1].location = [[2],[0.5],[0],[1]]; // in homg coords
robot_obstacles[1].radius = 1.0;
robot_obstacles[2] = {}; 
robot_obstacles[2].location = [[-2],[0.5],[0],[1]]; // in homg coords
robot_obstacles[2].radius = 1.0;

objs = {}
objs['red_cube'] = {
    origin: {
        xyz: [-4, 0, -3],
        rpy: [-1.57, 0, 0],
    },
    size: 0.3,
    graspable: true,
}
var collada_loader = new THREE.ColladaLoader();
var val = collada_loader.load('worlds/Table/Assets/Table.dae', 
    function ( collada ) {
        console.log(collada);
        objs['red_cube'].geom = collada.scene;
    },
    function ( xhr ) {
        console.log( ( xhr.loaded / xhr.total * 100 ) + '% loaded' );
    },
);

objs['blue_bubble'] = {
    origin: {
        xyz: [4, 0, 4],
        rpy: [0, 0, 0],
    },
    size: 1,
    color: 0x0000ff,
    graspable: false,
}
var val = collada_loader.load('worlds/Bed.dae', 
    function ( collada ) {
        console.log(collada);
        // scale
        collada.scene.scale.x = 0.004;
        collada.scene.scale.y = 0.004;
        collada.scene.scale.z = 0.004;
        objs['blue_bubble'].geom = collada.scene;
    },
    function ( xhr ) {
        console.log( ( xhr.loaded / xhr.total * 100 ) + '% loaded' );
    },
);