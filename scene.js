import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';
import * as BufferGeometryUtils from 'three/examples/jsm/utils/BufferGeometryUtils.js'
import { RectAreaLightHelper } from 'https://unpkg.com/three@0.168.0/examples/jsm/helpers/RectAreaLightHelper.js';
import { RectAreaLightUniformsLib } from 'https://unpkg.com/three@0.168.0/examples/jsm/lights/RectAreaLightUniformsLib.js';

const clock = new THREE.Clock();

// Graphics stuff
let camera, controls, scene, renderer, container;

// Physics stuff
let collisionConfiguration, dispatcher, broadphase, solver, physicsWorld, tmpTransform, softBodySolver, transformAux1, softBodyHelpers;
const gravityConstant = - 100;
const margin = 0.05;
let rigidBodies = [];
const softBodies = [];

Ammo().then( function ( AmmoLib ) {

    Ammo = AmmoLib;

    init();

} );

function init(){

    init_renderer();
    init_physics();
    init_scene();
    init_controls();
}

function init_renderer(){

    container = document.getElementById( 'container' );

    camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 10000);
    camera.position.set(0, 50, 150);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.setAnimationLoop(animate_and_physics);
    container.appendChild(renderer.domElement);

    scene = new THREE.Scene();

    window.addEventListener( 'resize', onWindowResize );

}

function init_scene(){

    // Light
    const light = new THREE.DirectionalLight(0xffffff, 10);
    light.position.set(0, 20, 20);
    light.target.position.set(0, 0, 0);
    light.castShadow = true;
    light.shadow.mapSize.width = 2048;
    light.shadow.mapSize.height = 2048;
    light.shadow.camera.left = -50;
    light.shadow.camera.right = 50;
    light.shadow.camera.top = 50;
    light.shadow.camera.bottom = -50;
    light.shadow.camera.near = 0.1;
    light.shadow.camera.far = 500;
    scene.add(light);

    const s_radius = 2;
    const s_mass = 100;
    // Create a smooth sphere geometry
    const geometry = new THREE.SphereGeometry(s_radius, 64, 64);
    const material = new THREE.MeshStandardMaterial({
        color: 0x0077ff,
        roughness: 0.2,
        metalness: 0.5
    });

    const sphere = new THREE.Mesh(geometry, material);
    sphere.position.y = 10;
    sphere.castShadow = true;
    sphere.receiveShadow = true;
    scene.add(sphere);
    let physics_sphere = create_sphere_physics(s_mass,sphere.position, s_radius);
    
    physics_sphere.body.setRestitution(0.5);
    physics_sphere.body.setFriction(1);
    physics_sphere.body.setRollingFriction(1);
    physicsWorld.addRigidBody(physics_sphere.body);
    rigidBodies.push({mesh: sphere, rigidBody: physics_sphere});

    // const sphere2 = new THREE.Mesh(geometry, material);
    // sphere2.position.set( 2, 10, 2 );
    // sphere2.castShadow = true;
    // sphere2.receiveShadow = true;
    // scene.add(sphere2);
    // let physics_sphere2 = create_sphere_physics(1.5,sphere2.position, 10);
    
    // physics_sphere2.body.setRestitution(0.75);
    // physics_sphere2.body.setFriction(1);
    // physics_sphere2.body.setRollingFriction(5);
    // physicsWorld.addRigidBody(physics_sphere2.body);
    // rigidBodies.push({mesh: sphere2, rigidBody: physics_sphere2});

    const volumeMass = 20;
    const sphereGeometry = new THREE.SphereGeometry( 1.5, 40, 25 );
    sphereGeometry.translate( 4, 10, 4 );
    const volume = new THREE.Mesh( sphereGeometry, new THREE.MeshPhongMaterial( { color: 0x613aa1 } ) );
    volume.castShadow = true;
    volume.receiveShadow = true;
    volume.frustumCulled = false;
    scene.add( volume );
    createSoftVolume( sphereGeometry, volumeMass, volume, 1000 );

    // Create the floor 

    const g_size = 20;
    const ground = new THREE.Mesh(
        new THREE.BoxGeometry(g_size, 1, g_size),
        new THREE.MeshStandardMaterial({color: 0x404040}));
    ground.castShadow = false;
    ground.receiveShadow = true;
    scene.add(ground);

    let physics_ground = create_box_physics(0, ground.position, ground.quaternion, new THREE.Vector3(g_size, 1, g_size));
    physics_ground.body.setRestitution(0.99);
    physicsWorld.addRigidBody(physics_ground.body);

}

function init_physics(){
    // Rigid body 
    collisionConfiguration = new Ammo.btSoftBodyRigidBodyCollisionConfiguration();
    dispatcher = new Ammo.btCollisionDispatcher(collisionConfiguration);
    broadphase = new Ammo.btDbvtBroadphase();
    solver = new Ammo.btSequentialImpulseConstraintSolver();
    collisionConfiguration = new Ammo.btDefaultCollisionConfiguration();
    physicsWorld = new Ammo.btSoftRigidDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration, softBodySolver );
    physicsWorld.setGravity( new Ammo.btVector3( 0, gravityConstant, 0 ) );
    tmpTransform = new Ammo.btTransform();

    softBodyHelpers = new Ammo.btSoftBodyHelpers();
}

function init_controls(){

    // Set up OrbitControls
    controls = new OrbitControls(camera, renderer.domElement);
    controls.target.set(0, 10, 0);
    controls.update();
}

function physics_step(timeElapsed){

    physicsWorld.stepSimulation(timeElapsed, 10);

    // Update soft volumes
    for ( let i = 0, il = softBodies.length; i < il; i ++ ) {

        const volume = softBodies[ i ];
        const geometry = volume.geometry;
        const softBody = volume.userData.physicsBody;
        const volumePositions = geometry.attributes.position.array;
        const volumeNormals = geometry.attributes.normal.array;
        const association = geometry.ammoIndexAssociation;
        const numVerts = association.length;
        const nodes = softBody.get_m_nodes();
        for ( let j = 0; j < numVerts; j ++ ) {

            const node = nodes.at( j );
            const nodePos = node.get_m_x();
            const x = nodePos.x();
            const y = nodePos.y();
            const z = nodePos.z();
            const nodeNormal = node.get_m_n();
            const nx = nodeNormal.x();
            const ny = nodeNormal.y();
            const nz = nodeNormal.z();

            const assocVertex = association[ j ];

            for ( let k = 0, kl = assocVertex.length; k < kl; k ++ ) {

                let indexVertex = assocVertex[ k ];
                volumePositions[ indexVertex ] = x;
                volumeNormals[ indexVertex ] = nx;
                indexVertex ++;
                volumePositions[ indexVertex ] = y;
                volumeNormals[ indexVertex ] = ny;
                indexVertex ++;
                volumePositions[ indexVertex ] = z;
                volumeNormals[ indexVertex ] = nz;

            }

        }

        geometry.attributes.position.needsUpdate = true;
        geometry.attributes.normal.needsUpdate = true;

    }

    for (let i = 0; i < rigidBodies.length; ++i) {
        rigidBodies[i].rigidBody.motionState.getWorldTransform(tmpTransform);
        const pos = tmpTransform.getOrigin();
        const quat = tmpTransform.getRotation();
        const pos3 = new THREE.Vector3(pos.x(), pos.y(), pos.z());
        const quat3 = new THREE.Quaternion(quat.x(), quat.y(), quat.z(), quat.w());
        rigidBodies[i].mesh.position.copy(pos3);
        rigidBodies[i].mesh.quaternion.copy(quat3);
      }

}
function animate_and_physics() {
    const deltaTime = clock.getDelta();
    physics_step(deltaTime)
    controls.update();
    renderer.render(scene, camera);
}

function create_box_physics(mass,pos, quat, size){
    let transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
    transform.setRotation(new Ammo.btQuaternion(quat.x, quat.y, quat.z, quat.w));
    let motionState = new Ammo.btDefaultMotionState(transform);
    
    const btSize = new Ammo.btVector3(size.x * 0.5, size.y * 0.5, size.z * 0.5);
    let shape = new Ammo.btBoxShape(btSize);
    shape.setMargin(0.05);
    
    let inertia = new Ammo.btVector3(0, 0, 0);
        if (mass > 0) {
        shape.calculateLocalInertia(mass, inertia);
        }
    
    let info = new Ammo.btRigidBodyConstructionInfo(
        mass, motionState, shape, inertia);
    let body = new Ammo.btRigidBody(info);
    Ammo.destroy(btSize);
    return { body: body, motionState: motionState};
}

function create_sphere_physics(mass, pos, size){

    let transform = new Ammo.btTransform();
    transform.setIdentity();
    transform.setOrigin(new Ammo.btVector3(pos.x, pos.y, pos.z));
    transform.setRotation(new Ammo.btQuaternion(0, 0, 0, 1));
    let motionState = new Ammo.btDefaultMotionState(transform);

    let shape = new Ammo.btSphereShape(size);
    shape.setMargin(0.05);

    let inertia = new Ammo.btVector3(0, 0, 0);
    if(mass > 0) {
      shape.calculateLocalInertia(mass, inertia);
    }

    let info = new Ammo.btRigidBodyConstructionInfo(mass, motionState, shape, inertia);
    let body = new Ammo.btRigidBody(info);

    return { body: body, motionState: motionState };
}

function createSoftVolume( bufferGeom, mass, volume, pressure ) {
    
    processGeometry( bufferGeom );

    // Volume physic object

    const volumeSoftBody = softBodyHelpers.CreateFromTriMesh(
        physicsWorld.getWorldInfo(),
        bufferGeom.ammoVertices,
        bufferGeom.ammoIndices,
        bufferGeom.ammoIndices.length / 3,
        true );

    const sbConfig = volumeSoftBody.get_m_cfg();
    sbConfig.set_viterations( 60 );
    sbConfig.set_piterations( 60 );

    // // Soft-soft and soft-rigid collisions
    sbConfig.set_collisions( 0x11 );

    // // Friction
    sbConfig.set_kDF( 0.1 );
    // // Damping
    sbConfig.set_kDP( 0.0001 );
    // // Pressure
    sbConfig.set_kPR( pressure );
    // // Stiffness
    volumeSoftBody.get_m_materials().at( 0 ).set_m_kLST( 0.99 );
    volumeSoftBody.get_m_materials().at( 0 ).set_m_kAST( 0.99 );

    volumeSoftBody.setTotalMass( mass, false );
    Ammo.castObject( volumeSoftBody, Ammo.btCollisionObject ).getCollisionShape().setMargin( margin );
    physicsWorld.addSoftBody( volumeSoftBody, 1, - 1 );
    volume.userData.physicsBody = volumeSoftBody;
    // // Disable deactivation
    volumeSoftBody.setActivationState( 4 );

    softBodies.push( volume );

}

function processGeometry( bufGeometry ) {

    // Ony consider the position values when merging the vertices
    const posOnlyBufGeometry = new THREE.BufferGeometry();
    posOnlyBufGeometry.setAttribute( 'position', bufGeometry.getAttribute( 'position' ) );
    posOnlyBufGeometry.setIndex( bufGeometry.getIndex() );

    // Merge the vertices so the triangle soup is converted to indexed triangles
    const indexedBufferGeom = BufferGeometryUtils.mergeVertices( posOnlyBufGeometry );

    // Create index arrays mapping the indexed vertices to bufGeometry vertices
    mapIndices( bufGeometry, indexedBufferGeom );

}

function mapIndices( bufGeometry, indexedBufferGeom ) {

    // Creates ammoVertices, ammoIndices and ammoIndexAssociation in bufGeometry

    const vertices = bufGeometry.attributes.position.array;
    const idxVertices = indexedBufferGeom.attributes.position.array;
    const indices = indexedBufferGeom.index.array;

    const numIdxVertices = idxVertices.length / 3;
    const numVertices = vertices.length / 3;

    bufGeometry.ammoVertices = idxVertices;
    bufGeometry.ammoIndices = indices;
    bufGeometry.ammoIndexAssociation = [];

    for ( let i = 0; i < numIdxVertices; i ++ ) {

        const association = [];
        bufGeometry.ammoIndexAssociation.push( association );

        const i3 = i * 3;

        for ( let j = 0; j < numVertices; j ++ ) {

            const j3 = j * 3;
            if ( isEqual( idxVertices[ i3 ], idxVertices[ i3 + 1 ], idxVertices[ i3 + 2 ],
                vertices[ j3 ], vertices[ j3 + 1 ], vertices[ j3 + 2 ] ) ) {

                association.push( j3 );

            }

        }

    }

}

function isEqual( x1, y1, z1, x2, y2, z2 ) {

    const delta = 0.000001;
    return Math.abs( x2 - x1 ) < delta &&
            Math.abs( y2 - y1 ) < delta &&
            Math.abs( z2 - z1 ) < delta;

}

function onWindowResize() {

    renderer.setSize(window.innerWidth, window.innerHeight);
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();

}
