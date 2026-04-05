/**
 * ESP-Drone 3D 可视化模块
 * ==========================
 * Three.js 无人机3D模型渲染
 */

let scene, camera, renderer, drone;

export function initThreeJS() {
    const container = document.getElementById('model-container');
    if (!container) {
        console.error('model-container not found');
        return;
    }

    const width = container.clientWidth || 800;
    const height = container.clientHeight || 600;

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x000000);

    // Grid
    const gridHelper = new THREE.GridHelper(20, 20, 0x333333, 0x111111);
    scene.add(gridHelper);

    camera = new THREE.PerspectiveCamera(45, width / height, 0.1, 100);
    camera.position.set(0, 2, 4);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(width, height);
    container.appendChild(renderer.domElement);

    // Lights
    const ambient = new THREE.AmbientLight(0xffffff, 0.7);
    scene.add(ambient);
    const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
    dirLight.position.set(5, 10, 5);
    scene.add(dirLight);

    // Build Drone Model
    drone = new THREE.Group();

    // Materials
    const matDark = new THREE.MeshPhongMaterial({ color: 0x222222 });
    const matArm = new THREE.MeshPhongMaterial({ color: 0x444444 });
    const matProp = new THREE.MeshPhongMaterial({ color: 0xeeeeee, transparent: true, opacity: 0.5 });
    const matInd = new THREE.MeshBasicMaterial({ color: 0x27d965 });

    // Body
    const body = new THREE.Mesh(new THREE.BoxGeometry(0.4, 0.1, 0.6), matDark);
    drone.add(body);

    // Front indicator
    const ind = new THREE.Mesh(new THREE.BoxGeometry(0.2, 0.05, 0.1), matInd);
    ind.position.set(0, 0, -0.3);
    drone.add(ind);

    // Arms
    const arm1 = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.05, 0.1), matArm);
    arm1.rotation.y = Math.PI / 4;
    drone.add(arm1);
    const arm2 = new THREE.Mesh(new THREE.BoxGeometry(1.8, 0.05, 0.1), matArm);
    arm2.rotation.y = -Math.PI / 4;
    drone.add(arm2);

    // Props
    const propGeo = new THREE.CylinderGeometry(0.6, 0.6, 0.01, 32);
    const positions = [
        { x: -0.65, z: -0.65 }, { x: 0.65, z: -0.65 },
        { x: 0.65, z: 0.65 }, { x: -0.65, z: 0.65 }
    ];

    positions.forEach(p => {
        const prop = new THREE.Mesh(propGeo, matProp);
        prop.position.set(p.x, 0.1, p.z);
        drone.add(prop);
    });

    // Axis helper
    const axesHelper = new THREE.AxesHelper(1);
    drone.add(axesHelper);

    scene.add(drone);

    animate();

    window.addEventListener('resize', onWindowResize);
}

function animate() {
    requestAnimationFrame(animate);
    renderer.render(scene, camera);
}

function onWindowResize() {
    const container = document.getElementById('model-container');
    if (camera && renderer && container) {
        const width = container.clientWidth;
        const height = container.clientHeight;

        camera.aspect = width / height;
        camera.updateProjectionMatrix();
        renderer.setSize(width, height);
    }
}

export function updateDroneOrientation(roll, pitch, yaw) {
    if (drone) {
        const toRad = Math.PI / 180;
        drone.rotation.order = 'YXZ';
        drone.rotation.x = pitch * toRad;
        drone.rotation.y = -yaw * toRad;
        drone.rotation.z = -roll * toRad;
    }
}
