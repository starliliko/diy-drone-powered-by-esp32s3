;(function () {
    'use strict';

    /* ===================== 常量 ===================== */
    var MAX_POINTS = 5000;          // 最大点云数
    var SPHERE_OPACITY = 0.15;      // 参考球透明度
    var POINT_SIZE = 3;             // 点大小(像素)
    var AXIS_LEN = 1.2;             // 坐标轴长度(gauss)

    /* ===================== 状态 ===================== */
    var isCalibrating = false;
    var points = [];                // [{x,y,z}]
    var scene, camera, renderer, controls;
    var pointCloud, pointGeometry, pointMaterial;
    var sphereMesh = null;
    var animFrameId = null;
    var connected = false;
    var latestMag = { x: 0, y: 0, z: 0, heading: 0 };

    // 统计
    var minX = Infinity, maxX = -Infinity;
    var minY = Infinity, maxY = -Infinity;
    var minZ = Infinity, maxZ = -Infinity;

    /* ===================== Three.js 初始化 ===================== */
    function initScene() {
        var container = document.getElementById('mag-3d-container');
        if (!container) return;
        if (renderer) {
            onResize();
            return;
        }

        // 页面隐藏时容器尺寸可能是 0，延后到页面可见再初始化
        var w = container.clientWidth;
        var h = container.clientHeight;
        if (w <= 0 || h <= 0) return;

        scene = new THREE.Scene();
        scene.background = new THREE.Color(0x1a1a2e);

        camera = new THREE.PerspectiveCamera(50, w / h, 0.01, 100);
        camera.position.set(1.5, 1.5, 1.5);
        camera.lookAt(0, 0, 0);

        renderer = new THREE.WebGLRenderer({ antialias: true });
        renderer.setSize(w, h);
        renderer.setPixelRatio(window.devicePixelRatio);
        container.appendChild(renderer.domElement);

        // OrbitControls
        if (THREE.OrbitControls) {
            controls = new THREE.OrbitControls(camera, renderer.domElement);
            controls.enableDamping = true;
            controls.dampingFactor = 0.1;
        }

        // 网格
        var grid = new THREE.GridHelper(2, 20, 0x333355, 0x222244);
        grid.rotation.x = Math.PI / 2; // XY 平面
        scene.add(grid);

        // 坐标轴
        addAxis(scene, new THREE.Vector3(AXIS_LEN, 0, 0), 0xff4444, 'X');
        addAxis(scene, new THREE.Vector3(0, AXIS_LEN, 0), 0x44ff44, 'Y');
        addAxis(scene, new THREE.Vector3(0, 0, AXIS_LEN), 0x4488ff, 'Z');

        // 点云
        pointGeometry = new THREE.BufferGeometry();
        pointMaterial = new THREE.PointsMaterial({
            size: POINT_SIZE,
            sizeAttenuation: false,
            vertexColors: true
        });
        pointCloud = new THREE.Points(pointGeometry, pointMaterial);
        scene.add(pointCloud);

        // 窗口 resize
        window.addEventListener('resize', onResize);

        animate();
    }

    function addAxis(sc, dir, color, label) {
        var origin = new THREE.Vector3(0, 0, 0);
        sc.add(new THREE.ArrowHelper(dir.clone().normalize(), origin, dir.length(), color, 0.06, 0.03));
    }

    function onResize() {
        var container = document.getElementById('mag-3d-container');
        if (!container || !renderer) return;
        var w = container.clientWidth;
        var h = container.clientHeight;
        if (w <= 0 || h <= 0) return;
        camera.aspect = w / h;
        camera.updateProjectionMatrix();
        renderer.setSize(w, h);
    }

    function onPageShown() {
        initScene();
        // 切页动画/布局完成后再补一次，确保完全贴合容器
        setTimeout(onResize, 0);
        setTimeout(onResize, 120);
    }

    function animate() {
        animFrameId = requestAnimationFrame(animate);
        if (controls) controls.update();
        renderer.render(scene, camera);
    }

    /* ===================== 点云更新 ===================== */
    function addPoint(x, y, z) {
        points.push({ x: x, y: y, z: z });
        if (points.length > MAX_POINTS) points.shift();

        // 统计更新
        if (x < minX) minX = x; if (x > maxX) maxX = x;
        if (y < minY) minY = y; if (y > maxY) maxY = y;
        if (z < minZ) minZ = z; if (z > maxZ) maxZ = z;

        rebuildPointCloud();
        updateStats();
    }

    function rebuildPointCloud() {
        var n = points.length;
        var positions = new Float32Array(n * 3);
        var colors = new Float32Array(n * 3);
        for (var i = 0; i < n; i++) {
            positions[i * 3]     = points[i].x;
            positions[i * 3 + 1] = points[i].y;
            positions[i * 3 + 2] = points[i].z;
            // 颜色：旧点→蓝，新点→亮绿
            var t = i / n;
            colors[i * 3]     = 0.2 * (1 - t) + 0.2 * t;
            colors[i * 3 + 1] = 0.3 * (1 - t) + 1.0 * t;
            colors[i * 3 + 2] = 0.9 * (1 - t) + 0.4 * t;
        }
        pointGeometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
        pointGeometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
        pointGeometry.computeBoundingSphere();
    }

    function clearPoints() {
        points = [];
        minX = Infinity; maxX = -Infinity;
        minY = Infinity; maxY = -Infinity;
        minZ = Infinity; maxZ = -Infinity;
        rebuildPointCloud();
        removeFitSphere();
        updateStats();
    }

    /* ===================== 拟合球 ===================== */
    function showFitSphere() {
        removeFitSphere();
        if (points.length < 50) return;
        var cx = (minX + maxX) / 2;
        var cy = (minY + maxY) / 2;
        var cz = (minZ + maxZ) / 2;
        var rx = (maxX - minX) / 2;
        var ry = (maxY - minY) / 2;
        var rz = (maxZ - minZ) / 2;
        var r = (rx + ry + rz) / 3;

        var geo = new THREE.SphereGeometry(r, 32, 32);
        var mat = new THREE.MeshBasicMaterial({
            color: 0x44aaff,
            transparent: true,
            opacity: SPHERE_OPACITY,
            wireframe: true
        });
        sphereMesh = new THREE.Mesh(geo, mat);
        sphereMesh.position.set(cx, cy, cz);
        scene.add(sphereMesh);

        return { cx: cx, cy: cy, cz: cz, r: r, rx: rx, ry: ry, rz: rz };
    }

    function removeFitSphere() {
        if (sphereMesh) {
            scene.remove(sphereMesh);
            sphereMesh.geometry.dispose();
            sphereMesh.material.dispose();
            sphereMesh = null;
        }
    }

    /* ===================== UI 统计更新 ===================== */
    function updateStats() {
        setText('mag-point-count', points.length);
        setText('mag-live-x', latestMag.x.toFixed(3));
        setText('mag-live-y', latestMag.y.toFixed(3));
        setText('mag-live-z', latestMag.z.toFixed(3));
        setText('mag-live-heading', latestMag.heading.toFixed(1) + '°');

        if (points.length > 0) {
            setText('mag-range-x', (maxX - minX).toFixed(3));
            setText('mag-range-y', (maxY - minY).toFixed(3));
            setText('mag-range-z', (maxZ - minZ).toFixed(3));
            setText('mag-offset-x', ((minX + maxX) / 2).toFixed(3));
            setText('mag-offset-y', ((minY + maxY) / 2).toFixed(3));
            setText('mag-offset-z', ((minZ + maxZ) / 2).toFixed(3));
        } else {
            setText('mag-range-x', '-');
            setText('mag-range-y', '-');
            setText('mag-range-z', '-');
            setText('mag-offset-x', '-');
            setText('mag-offset-y', '-');
            setText('mag-offset-z', '-');
        }
    }

    function setText(id, val) {
        var el = document.getElementById(id);
        if (el) el.textContent = val;
    }

    /* ===================== 校准流程 ===================== */
    function startCalibration() {
        if (!connected) {
            alert('飞控未连接');
            return;
        }
        clearPoints();
        isCalibrating = true;
        updateCalibUI();
        fetch('/api/mag/calibrate/start', { method: 'POST' })
            .then(function (r) { return r.json(); })
            .then(function (d) { console.log('[MagCalib] Start:', d); })
            .catch(function (e) { console.error('[MagCalib]', e); });
    }

    function stopCalibration() {
        isCalibrating = false;
        updateCalibUI();
        fetch('/api/mag/calibrate/stop', { method: 'POST' })
            .then(function (r) { return r.json(); })
            .then(function (d) {
                console.log('[MagCalib] Stop:', d);
                var fit = showFitSphere();
                if (fit) {
                    setText('mag-result-offX', fit.cx.toFixed(3));
                    setText('mag-result-offY', fit.cy.toFixed(3));
                    setText('mag-result-offZ', fit.cz.toFixed(3));
                    var avgR = fit.r;
                    setText('mag-result-scaleX', (avgR / fit.rx).toFixed(3));
                    setText('mag-result-scaleY', (avgR / fit.ry).toFixed(3));
                    setText('mag-result-scaleZ', (avgR / fit.rz).toFixed(3));
                    setText('mag-result-samples', points.length);
                }
            })
            .catch(function (e) { console.error('[MagCalib]', e); });
    }

    function updateCalibUI() {
        var startBtn = document.getElementById('mag-btn-start');
        var stopBtn = document.getElementById('mag-btn-stop');
        var statusEl = document.getElementById('mag-calib-status');
        if (startBtn) startBtn.disabled = isCalibrating;
        if (stopBtn) stopBtn.disabled = !isCalibrating;
        if (statusEl) {
            statusEl.textContent = isCalibrating ? '⏺ 采集中...' : '⏹ 已停止';
            statusEl.className = 'mag-status ' + (isCalibrating ? 'active' : 'idle');
        }
    }

    /* ===================== 参数写入 ===================== */
    function setMagParam(name, value) {
        return fetch('/api/mag/param', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ name: name, value: value })
        }).then(function (r) { return r.json(); });
    }

    function applyCalibResult() {
        // 读取结果面板的值并写入飞控
        var offX = parseFloat(document.getElementById('mag-result-offX').textContent);
        var offY = parseFloat(document.getElementById('mag-result-offY').textContent);
        var offZ = parseFloat(document.getElementById('mag-result-offZ').textContent);
        var scX = parseFloat(document.getElementById('mag-result-scaleX').textContent);
        var scY = parseFloat(document.getElementById('mag-result-scaleY').textContent);
        var scZ = parseFloat(document.getElementById('mag-result-scaleZ').textContent);

        if (isNaN(offX) || isNaN(scX)) {
            alert('无校准结果可应用');
            return;
        }

        Promise.all([
            setMagParam('offX', offX),
            setMagParam('offY', offY),
            setMagParam('offZ', offZ),
            setMagParam('scaleX', scX),
            setMagParam('scaleY', scY),
            setMagParam('scaleZ', scZ)
        ]).then(function () {
            alert('校准参数已写入飞控');
        }).catch(function (e) {
            alert('写入失败: ' + e.message);
        });
    }

    function setDeclination() {
        var val = parseFloat(document.getElementById('mag-declination-input').value);
        if (isNaN(val)) { alert('请输入有效数值'); return; }
        setMagParam('declination', val * Math.PI / 180.0)
            .then(function () { console.log('[MagCalib] Declination set to', val, 'deg'); });
    }

    function toggleYawFusion() {
        var checked = document.getElementById('mag-yaw-enable').checked;
        setMagParam('yawEn', checked ? 1 : 0);
    }

    /* ===================== 遥测回调 ===================== */
    function updateTelemetry(data) {
        if (data.magX === undefined) return;
        latestMag.x = data.magX;
        latestMag.y = data.magY;
        latestMag.z = data.magZ;
        latestMag.heading = data.magHeading || 0;

        if (isCalibrating && (data.magX !== 0 || data.magY !== 0 || data.magZ !== 0)) {
            addPoint(data.magX, data.magY, data.magZ);
        } else {
            // 即使不校准也更新实时数值
            updateStats();
        }
    }

    function setConnected(val) {
        connected = val;
    }

    /* ===================== 初始化 ===================== */
    function init() {
        initScene();
        updateCalibUI();
        updateStats();
    }

    /* ===================== 导出 ===================== */
    window.magCalib = {
        init: init,
        onPageShown: onPageShown,
        updateTelemetry: updateTelemetry,
        setConnected: setConnected,
        startCalibration: startCalibration,
        stopCalibration: stopCalibration,
        clearPoints: clearPoints,
        applyCalibResult: applyCalibResult,
        setDeclination: setDeclination,
        toggleYawFusion: toggleYawFusion
    };
})();
