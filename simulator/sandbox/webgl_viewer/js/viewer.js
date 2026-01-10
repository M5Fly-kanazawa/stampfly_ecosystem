/**
 * StampFly STL Viewer
 * WebGL-based 3D viewer for STL parts with visibility toggle
 *
 * STLパーツのWebGL 3Dビューア（表示/非表示切り替え機能付き）
 */

// Global state
let scene, camera, renderer, controls;
let parts = {};
let currentSource = 'classified';

// Configuration paths
const CONFIG_PATHS = {
    classified: '../assets/meshes/parts/parts_config.json',
    index_split: '../stl_splitter/parts/index_split/parts_config.json',
    hybrid: '../stl_splitter/parts/hybrid/parts_config.json',
    auto: '../stl_splitter/parts/auto/parts_config.json',
    manual: '../stl_splitter/parts/manual/parts_config.json'
};

const STL_BASE_PATHS = {
    classified: '../assets/meshes/parts/',
    index_split: '../stl_splitter/parts/index_split/',
    hybrid: '../stl_splitter/parts/hybrid/',
    auto: '../stl_splitter/parts/auto/',
    manual: '../stl_splitter/parts/manual/'
};

/**
 * Initialize the 3D scene
 * 3Dシーンを初期化
 */
function initScene() {
    const canvas = document.getElementById('canvas');
    const container = document.getElementById('viewer');

    // Scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);

    // Camera
    camera = new THREE.PerspectiveCamera(
        45,
        container.clientWidth / container.clientHeight,
        0.001,
        1000
    );
    camera.position.set(0.15, 0.15, 0.15);

    // Renderer
    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);

    // Controls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.target.set(0, 0, 0);

    // Lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.5);
    scene.add(ambientLight);

    const directionalLight1 = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight1.position.set(1, 1, 1);
    scene.add(directionalLight1);

    const directionalLight2 = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight2.position.set(-1, -1, -1);
    scene.add(directionalLight2);

    // Grid helper (for reference)
    const gridHelper = new THREE.GridHelper(0.2, 20, 0x444444, 0x333333);
    gridHelper.rotation.x = Math.PI / 2;  // Rotate to XY plane
    scene.add(gridHelper);

    // Create colored axis arrows with labels
    // 色付き座標軸矢印を作成（X=赤, Y=緑, Z=青）
    createAxisArrows();

    // Handle window resize
    window.addEventListener('resize', onWindowResize);

    // Start animation loop
    animate();
}

/**
 * Create colored axis arrows (X=red, Y=green, Z=blue)
 * 色付き座標軸矢印を作成
 */
function createAxisArrows() {
    const axisLength = 0.08;
    const arrowHeadLength = 0.015;
    const arrowHeadWidth = 0.008;

    // X axis (red) - Forward
    const xDir = new THREE.Vector3(1, 0, 0);
    const xArrow = new THREE.ArrowHelper(xDir, new THREE.Vector3(0, 0, 0), axisLength, 0xff0000, arrowHeadLength, arrowHeadWidth);
    scene.add(xArrow);

    // Y axis (green) - Right
    const yDir = new THREE.Vector3(0, 1, 0);
    const yArrow = new THREE.ArrowHelper(yDir, new THREE.Vector3(0, 0, 0), axisLength, 0x00ff00, arrowHeadLength, arrowHeadWidth);
    scene.add(yArrow);

    // Z axis (blue) - Down (NED coordinate)
    const zDir = new THREE.Vector3(0, 0, 1);
    const zArrow = new THREE.ArrowHelper(zDir, new THREE.Vector3(0, 0, 0), axisLength, 0x0088ff, arrowHeadLength, arrowHeadWidth);
    scene.add(zArrow);

    // Add axis labels using sprites
    createAxisLabel('X', new THREE.Vector3(axisLength + 0.01, 0, 0), 0xff0000);
    createAxisLabel('Y', new THREE.Vector3(0, axisLength + 0.01, 0), 0x00ff00);
    createAxisLabel('Z', new THREE.Vector3(0, 0, axisLength + 0.01), 0x0088ff);
}

/**
 * Create text label for axis
 * 軸のテキストラベルを作成
 */
function createAxisLabel(text, position, color) {
    const canvas = document.createElement('canvas');
    const size = 64;
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext('2d');

    // Draw text
    ctx.fillStyle = '#' + color.toString(16).padStart(6, '0');
    ctx.font = 'bold 48px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, size/2, size/2);

    // Create sprite
    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(material);
    sprite.position.copy(position);
    sprite.scale.set(0.02, 0.02, 1);
    scene.add(sprite);
}

/**
 * Handle window resize
 * ウィンドウリサイズ処理
 */
function onWindowResize() {
    const container = document.getElementById('viewer');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

/**
 * Animation loop
 * アニメーションループ
 */
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

/**
 * Load parts configuration from JSON
 * JSONからパーツ設定を読み込み
 */
async function loadPartsConfig(source) {
    const configPath = CONFIG_PATHS[source];
    const response = await fetch(configPath);
    if (!response.ok) {
        throw new Error(`Failed to load config: ${configPath}`);
    }
    return await response.json();
}

/**
 * Load a single STL file
 * 単一のSTLファイルを読み込み
 */
function loadSTL(url) {
    return new Promise((resolve, reject) => {
        const loader = new THREE.STLLoader();
        loader.load(
            url,
            (geometry) => resolve(geometry),
            undefined,
            (error) => reject(error)
        );
    });
}

/**
 * Create material with given color and opacity
 * 指定色と透明度でマテリアルを作成
 */
function createMaterial(color, opacity = 1) {
    const material = new THREE.MeshPhongMaterial({
        color: new THREE.Color(color[0], color[1], color[2]),
        specular: 0x111111,
        shininess: 30,
        side: THREE.DoubleSide
    });

    // Apply opacity if not fully opaque
    // 不透明でない場合は透明度を適用
    if (opacity < 1) {
        material.transparent = true;
        material.opacity = opacity;
    }

    return material;
}

/**
 * Clear all parts from scene
 * シーンから全パーツを削除
 */
function clearParts() {
    for (const partName in parts) {
        if (parts[partName].mesh) {
            scene.remove(parts[partName].mesh);
            parts[partName].mesh.geometry.dispose();
            parts[partName].mesh.material.dispose();
        }
    }
    parts = {};
}

/**
 * Load all parts for the selected source
 * 選択されたソースの全パーツを読み込み
 */
async function loadParts(source) {
    const loading = document.getElementById('loading');
    loading.style.display = 'block';

    try {
        // Clear existing parts
        clearParts();

        // Load configuration
        const config = await loadPartsConfig(source);
        const basePath = STL_BASE_PATHS[source];

        // Load each part
        for (const partConfig of config.parts) {
            const stlUrl = basePath + partConfig.file;

            try {
                const geometry = await loadSTL(stlUrl);

                // Scale from mm to m (STL is in mm)
                geometry.scale(0.001, 0.001, 0.001);

                // Center geometry
                geometry.computeBoundingBox();

                const material = createMaterial(partConfig.color, partConfig.opacity);
                const mesh = new THREE.Mesh(geometry, material);

                // Apply coordinate transform (Y axis flip for NED)
                mesh.rotation.x = 0;
                mesh.scale.y = -1;  // Flip Y axis

                scene.add(mesh);

                parts[partConfig.name] = {
                    mesh: mesh,
                    config: partConfig,
                    visible: true
                };
            } catch (error) {
                console.error(`Failed to load ${partConfig.file}:`, error);
            }
        }

        // Update parts list UI
        updatePartsList(config.parts);

        loading.style.display = 'none';

    } catch (error) {
        console.error('Failed to load parts:', error);
        loading.innerHTML = `<div style="color: #ff6b6b;">Error loading parts<br>${error.message}</div>`;
    }
}

/**
 * Update the parts list UI
 * パーツリストUIを更新
 */
function updatePartsList(partsConfig) {
    const partsList = document.getElementById('parts-list');
    partsList.innerHTML = '';

    for (const partConfig of partsConfig) {
        const item = document.createElement('div');
        item.className = 'part-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.checked = true;
        checkbox.id = `checkbox-${partConfig.name}`;
        checkbox.addEventListener('change', () => togglePart(partConfig.name, checkbox.checked));

        const colorBox = document.createElement('div');
        colorBox.className = 'part-color';
        const [r, g, b] = partConfig.color;
        colorBox.style.backgroundColor = `rgb(${Math.round(r*255)}, ${Math.round(g*255)}, ${Math.round(b*255)})`;

        const nameSpan = document.createElement('span');
        nameSpan.className = 'part-name';
        nameSpan.textContent = partConfig.name;

        const trianglesSpan = document.createElement('span');
        trianglesSpan.className = 'part-triangles';
        trianglesSpan.textContent = `${partConfig.triangles}`;

        item.appendChild(checkbox);
        item.appendChild(colorBox);
        item.appendChild(nameSpan);
        item.appendChild(trianglesSpan);

        // Click on item toggles checkbox
        item.addEventListener('click', (e) => {
            if (e.target !== checkbox) {
                checkbox.checked = !checkbox.checked;
                togglePart(partConfig.name, checkbox.checked);
            }
        });

        partsList.appendChild(item);
    }
}

/**
 * Toggle part visibility
 * パーツの表示/非表示を切り替え
 */
function togglePart(partName, visible) {
    if (parts[partName] && parts[partName].mesh) {
        parts[partName].mesh.visible = visible;
        parts[partName].visible = visible;
    }
}

/**
 * Show all parts
 * すべてのパーツを表示
 */
function showAllParts() {
    for (const partName in parts) {
        togglePart(partName, true);
        const checkbox = document.getElementById(`checkbox-${partName}`);
        if (checkbox) checkbox.checked = true;
    }
}

/**
 * Hide all parts
 * すべてのパーツを非表示
 */
function hideAllParts() {
    for (const partName in parts) {
        togglePart(partName, false);
        const checkbox = document.getElementById(`checkbox-${partName}`);
        if (checkbox) checkbox.checked = false;
    }
}

/**
 * Reset camera view
 * カメラ視点をリセット
 */
function resetView() {
    camera.position.set(0.15, 0.15, 0.15);
    controls.target.set(0, 0, 0);
    controls.update();
}

/**
 * Initialize event handlers
 * イベントハンドラを初期化
 */
function initEventHandlers() {
    // Source selector
    document.getElementById('source-selector').addEventListener('change', (e) => {
        currentSource = e.target.value;
        loadParts(currentSource);
    });

    // Control buttons
    document.getElementById('btn-show-all').addEventListener('click', showAllParts);
    document.getElementById('btn-hide-all').addEventListener('click', hideAllParts);
    document.getElementById('btn-reset-view').addEventListener('click', resetView);
}

/**
 * Main initialization
 * メイン初期化
 */
function init() {
    initScene();
    initEventHandlers();
    loadParts(currentSource);
}

// Start when DOM is ready
document.addEventListener('DOMContentLoaded', init);
