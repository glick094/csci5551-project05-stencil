
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
        xyz: [-4, 1, -3],
        rpy: [0, 1, 0],
    },
    size: 0.3,
    color: 0xff0000,
    graspable: true,
}
objs['red_cube'].geom = new THREE.Mesh(
    new THREE.CubeGeometry(objs['red_cube'].size, objs['red_cube'].size, objs['red_cube'].size),
    new THREE.MeshBasicMaterial({color: objs['red_cube'].color}),
)
objs['blue_bubble'] = {
    origin: {
        xyz: [4, 0, 4],
        rpy: [0, 0, 0],
    },
    size: 1,
    color: 0x0000ff,
    graspable: false,
}
objs['blue_bubble'].geom = new THREE.Mesh(
    new THREE.SphereGeometry(objs['blue_bubble'].size),
    new THREE.MeshBasicMaterial({color: objs['blue_bubble'].color}),
)

function isTaskCompleted() {
    cube_xyz = objs['red_cube'].origin.xyz
    bubble_xyz = objs['blue_bubble'].origin.xyz
    return Math.sqrt(
        Math.pow(cube_xyz[0] - bubble_xyz[0], 2) +
        Math.pow(cube_xyz[1] - bubble_xyz[1], 2) +
        Math.pow(cube_xyz[2] - bubble_xyz[2], 2)
    ) < 1
}