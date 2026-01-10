/**
 * STL Coordinate Transformer
 * WebGL-based tool for transforming STL coordinate systems
 *
 * STL座標変換ツール
 * WebGLベースの座標系変換ツール
 */

// Global state
let scene, camera, renderer, controls;
let originalGeometries = {};  // Store original geometry data
let transformedMeshes = {};   // Current meshes in scene
let partsConfig = null;

const CONFIG_PATH = '../assets/meshes/parts/parts_config.json';
const STL_BASE_PATH = '../assets/meshes/parts/';

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

    // Grid helper
    const gridHelper = new THREE.GridHelper(0.2, 20, 0x444444, 0x333333);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    // Create axis arrows
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
    const axisLength = 0.1;
    const arrowHeadLength = 0.02;
    const arrowHeadWidth = 0.01;

    // X axis (red)
    const xDir = new THREE.Vector3(1, 0, 0);
    const xArrow = new THREE.ArrowHelper(xDir, new THREE.Vector3(0, 0, 0), axisLength, 0xff0000, arrowHeadLength, arrowHeadWidth);
    scene.add(xArrow);

    // Y axis (green)
    const yDir = new THREE.Vector3(0, 1, 0);
    const yArrow = new THREE.ArrowHelper(yDir, new THREE.Vector3(0, 0, 0), axisLength, 0x00ff00, arrowHeadLength, arrowHeadWidth);
    scene.add(yArrow);

    // Z axis (blue)
    const zDir = new THREE.Vector3(0, 0, 1);
    const zArrow = new THREE.ArrowHelper(zDir, new THREE.Vector3(0, 0, 0), axisLength, 0x0088ff, arrowHeadLength, arrowHeadWidth);
    scene.add(zArrow);

    // Add axis labels
    createAxisLabel('X', new THREE.Vector3(axisLength + 0.015, 0, 0), 0xff0000);
    createAxisLabel('Y', new THREE.Vector3(0, axisLength + 0.015, 0), 0x00ff00);
    createAxisLabel('Z', new THREE.Vector3(0, 0, axisLength + 0.015), 0x0088ff);
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

    ctx.fillStyle = '#' + color.toString(16).padStart(6, '0');
    ctx.font = 'bold 48px Arial';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText(text, size/2, size/2);

    const texture = new THREE.CanvasTexture(canvas);
    const material = new THREE.SpriteMaterial({ map: texture });
    const sprite = new THREE.Sprite(material);
    sprite.position.copy(position);
    sprite.scale.set(0.025, 0.025, 1);
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
 * Load parts configuration
 * パーツ設定を読み込み
 */
async function loadPartsConfig() {
    const response = await fetch(CONFIG_PATH);
    if (!response.ok) {
        throw new Error(`Failed to load config: ${CONFIG_PATH}`);
    }
    return await response.json();
}

/**
 * Load a single STL file and return raw geometry data
 * 単一のSTLファイルを読み込み、生の頂点データを返す
 */
function loadSTL(url) {
    return new Promise((resolve, reject) => {
        const loader = new THREE.STLLoader();
        loader.load(
            url,
            (geometry) => {
                // Extract raw vertex data (in mm)
                const positions = geometry.attributes.position.array;
                const normals = geometry.attributes.normal ? geometry.attributes.normal.array : null;

                // Store as Float32Array copies
                resolve({
                    positions: new Float32Array(positions),
                    normals: normals ? new Float32Array(normals) : null,
                    triangleCount: positions.length / 9
                });
            },
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

    if (opacity < 1) {
        material.transparent = true;
        material.opacity = opacity;
    }

    return material;
}

/**
 * Get transformation matrix from UI settings
 * UI設定から変換行列を取得
 */
function getTransformMatrix() {
    const mapX = document.getElementById('map-x').value;
    const mapY = document.getElementById('map-y').value;
    const mapZ = document.getElementById('map-z').value;

    // Parse axis mapping to matrix rows
    const matrix = [
        parseAxisMapping(mapX),
        parseAxisMapping(mapY),
        parseAxisMapping(mapZ)
    ];

    return matrix;
}

/**
 * Parse axis mapping string to matrix row
 * 軸マッピング文字列を行列の行に変換
 */
function parseAxisMapping(mapping) {
    const sign = mapping[0] === '+' ? 1 : -1;
    const axis = mapping[1];

    switch (axis) {
        case 'X': return [sign, 0, 0];
        case 'Y': return [0, sign, 0];
        case 'Z': return [0, 0, sign];
    }
}

/**
 * Get offset values from UI
 * UIからオフセット値を取得
 */
function getOffset() {
    return {
        x: parseFloat(document.getElementById('offset-x').value) || 0,
        y: parseFloat(document.getElementById('offset-y').value) || 0,
        z: parseFloat(document.getElementById('offset-z').value) || 0
    };
}

/**
 * Transform vertex positions using matrix and offset
 * 行列とオフセットを使用して頂点位置を変換
 */
function transformPositions(positions, matrix, offset) {
    const transformed = new Float32Array(positions.length);

    for (let i = 0; i < positions.length; i += 3) {
        const x = positions[i];
        const y = positions[i + 1];
        const z = positions[i + 2];

        // Apply rotation/flip matrix
        transformed[i] = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + offset.x;
        transformed[i + 1] = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + offset.y;
        transformed[i + 2] = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + offset.z;
    }

    return transformed;
}

/**
 * Transform normals using matrix (no offset for normals)
 * 行列を使用して法線を変換（法線にはオフセットなし）
 */
function transformNormals(normals, matrix) {
    if (!normals) return null;

    const transformed = new Float32Array(normals.length);

    for (let i = 0; i < normals.length; i += 3) {
        const nx = normals[i];
        const ny = normals[i + 1];
        const nz = normals[i + 2];

        // Apply rotation/flip matrix to normals
        transformed[i] = matrix[0][0] * nx + matrix[0][1] * ny + matrix[0][2] * nz;
        transformed[i + 1] = matrix[1][0] * nx + matrix[1][1] * ny + matrix[1][2] * nz;
        transformed[i + 2] = matrix[2][0] * nx + matrix[2][1] * ny + matrix[2][2] * nz;
    }

    return transformed;
}

/**
 * Clear all part meshes from scene
 * シーンから全パーツメッシュを削除
 */
function clearMeshes() {
    for (const partName in transformedMeshes) {
        if (transformedMeshes[partName]) {
            scene.remove(transformedMeshes[partName]);
            transformedMeshes[partName].geometry.dispose();
            transformedMeshes[partName].material.dispose();
        }
    }
    transformedMeshes = {};
}

/**
 * Create mesh from transformed data
 * 変換データからメッシュを作成
 */
function createMeshFromData(positions, normals, partConfig) {
    const geometry = new THREE.BufferGeometry();

    // Scale from mm to m
    const scaledPositions = new Float32Array(positions.length);
    for (let i = 0; i < positions.length; i++) {
        scaledPositions[i] = positions[i] * 0.001;
    }

    geometry.setAttribute('position', new THREE.BufferAttribute(scaledPositions, 3));

    if (normals) {
        geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    } else {
        geometry.computeVertexNormals();
    }

    const material = createMaterial(partConfig.color, partConfig.opacity);
    const mesh = new THREE.Mesh(geometry, material);

    return mesh;
}

/**
 * Apply preview transformation to all parts
 * 全パーツにプレビュー変換を適用
 */
function applyPreview() {
    const loading = document.getElementById('loading');
    loading.style.display = 'block';

    setTimeout(() => {
        try {
            clearMeshes();

            const matrix = getTransformMatrix();
            const offset = getOffset();

            let globalMinX = Infinity, globalMaxX = -Infinity;
            let globalMinY = Infinity, globalMaxY = -Infinity;
            let globalMinZ = Infinity, globalMaxZ = -Infinity;

            for (const partConfig of partsConfig.parts) {
                const original = originalGeometries[partConfig.name];
                if (!original) continue;

                const transformedPositions = transformPositions(original.positions, matrix, offset);
                const transformedNormals = transformNormals(original.normals, matrix);

                // Calculate bounding box
                for (let i = 0; i < transformedPositions.length; i += 3) {
                    const x = transformedPositions[i];
                    const y = transformedPositions[i + 1];
                    const z = transformedPositions[i + 2];

                    globalMinX = Math.min(globalMinX, x);
                    globalMaxX = Math.max(globalMaxX, x);
                    globalMinY = Math.min(globalMinY, y);
                    globalMaxY = Math.max(globalMaxY, y);
                    globalMinZ = Math.min(globalMinZ, z);
                    globalMaxZ = Math.max(globalMaxZ, z);
                }

                const mesh = createMeshFromData(transformedPositions, transformedNormals, partConfig);
                scene.add(mesh);
                transformedMeshes[partConfig.name] = mesh;
            }

            // Update coordinate display
            document.getElementById('min-x').textContent = globalMinX.toFixed(2);
            document.getElementById('max-x').textContent = globalMaxX.toFixed(2);
            document.getElementById('min-y').textContent = globalMinY.toFixed(2);
            document.getElementById('max-y').textContent = globalMaxY.toFixed(2);
            document.getElementById('min-z').textContent = globalMinZ.toFixed(2);
            document.getElementById('max-z').textContent = globalMaxZ.toFixed(2);

            updateMatrixDisplay();

        } catch (error) {
            console.error('Preview failed:', error);
            alert('Preview failed: ' + error.message);
        }

        loading.style.display = 'none';
    }, 50);
}

/**
 * Update matrix display
 * 行列表示を更新
 */
function updateMatrixDisplay() {
    const matrix = getTransformMatrix();
    const offset = getOffset();

    const formatNum = (n) => (n >= 0 ? ' ' : '') + n.toString().padStart(1);

    let display = 'Transform Matrix:\n';
    display += `[${formatNum(matrix[0][0])} ${formatNum(matrix[0][1])} ${formatNum(matrix[0][2])} | ${offset.x.toFixed(1)}]\n`;
    display += `[${formatNum(matrix[1][0])} ${formatNum(matrix[1][1])} ${formatNum(matrix[1][2])} | ${offset.y.toFixed(1)}]\n`;
    display += `[${formatNum(matrix[2][0])} ${formatNum(matrix[2][1])} ${formatNum(matrix[2][2])} | ${offset.z.toFixed(1)}]`;

    document.getElementById('matrix-display').textContent = display;
}

/**
 * Reset to identity transform
 * 恒等変換にリセット
 */
function resetTransform() {
    document.getElementById('map-x').value = '+X';
    document.getElementById('map-y').value = '+Y';
    document.getElementById('map-z').value = '+Z';
    document.getElementById('offset-x').value = '0';
    document.getElementById('offset-y').value = '0';
    document.getElementById('offset-z').value = '0';

    applyPreview();
}

/**
 * Center model to origin
 * モデルを原点に中心合わせ
 */
function centerToOrigin() {
    const matrix = getTransformMatrix();

    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;
    let minZ = Infinity, maxZ = -Infinity;

    // Calculate bounding box with current matrix (no offset)
    for (const partConfig of partsConfig.parts) {
        const original = originalGeometries[partConfig.name];
        if (!original) continue;

        const transformed = transformPositions(original.positions, matrix, { x: 0, y: 0, z: 0 });

        for (let i = 0; i < transformed.length; i += 3) {
            minX = Math.min(minX, transformed[i]);
            maxX = Math.max(maxX, transformed[i]);
            minY = Math.min(minY, transformed[i + 1]);
            maxY = Math.max(maxY, transformed[i + 1]);
            minZ = Math.min(minZ, transformed[i + 2]);
            maxZ = Math.max(maxZ, transformed[i + 2]);
        }
    }

    // Set offset to center
    const centerX = (minX + maxX) / 2;
    const centerY = (minY + maxY) / 2;
    const centerZ = (minZ + maxZ) / 2;

    document.getElementById('offset-x').value = (-centerX).toFixed(2);
    document.getElementById('offset-y').value = (-centerY).toFixed(2);
    document.getElementById('offset-z').value = (-centerZ).toFixed(2);

    applyPreview();
}

/**
 * Generate binary STL data from positions and normals
 * 位置と法線からバイナリSTLデータを生成
 */
function generateBinarySTL(positions, normals) {
    const triangleCount = positions.length / 9;
    const bufferSize = 84 + triangleCount * 50;  // Header(80) + count(4) + triangles(50 each)
    const buffer = new ArrayBuffer(bufferSize);
    const view = new DataView(buffer);

    // Write header (80 bytes)
    const header = 'Binary STL generated by STL Coordinate Transformer';
    for (let i = 0; i < 80; i++) {
        view.setUint8(i, i < header.length ? header.charCodeAt(i) : 0);
    }

    // Write triangle count
    view.setUint32(80, triangleCount, true);

    // Write triangles
    let offset = 84;
    for (let t = 0; t < triangleCount; t++) {
        const baseIdx = t * 9;

        // Calculate face normal (average of vertex normals or compute from vertices)
        let nx, ny, nz;
        if (normals) {
            nx = (normals[baseIdx] + normals[baseIdx + 3] + normals[baseIdx + 6]) / 3;
            ny = (normals[baseIdx + 1] + normals[baseIdx + 4] + normals[baseIdx + 7]) / 3;
            nz = (normals[baseIdx + 2] + normals[baseIdx + 5] + normals[baseIdx + 8]) / 3;
        } else {
            // Compute normal from vertices
            const v0 = [positions[baseIdx], positions[baseIdx + 1], positions[baseIdx + 2]];
            const v1 = [positions[baseIdx + 3], positions[baseIdx + 4], positions[baseIdx + 5]];
            const v2 = [positions[baseIdx + 6], positions[baseIdx + 7], positions[baseIdx + 8]];

            const e1 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
            const e2 = [v2[0] - v0[0], v2[1] - v0[1], v2[2] - v0[2]];

            nx = e1[1] * e2[2] - e1[2] * e2[1];
            ny = e1[2] * e2[0] - e1[0] * e2[2];
            nz = e1[0] * e2[1] - e1[1] * e2[0];

            const len = Math.sqrt(nx * nx + ny * ny + nz * nz);
            if (len > 0) {
                nx /= len;
                ny /= len;
                nz /= len;
            }
        }

        // Write normal
        view.setFloat32(offset, nx, true); offset += 4;
        view.setFloat32(offset, ny, true); offset += 4;
        view.setFloat32(offset, nz, true); offset += 4;

        // Write vertices
        for (let v = 0; v < 3; v++) {
            const vIdx = baseIdx + v * 3;
            view.setFloat32(offset, positions[vIdx], true); offset += 4;
            view.setFloat32(offset, positions[vIdx + 1], true); offset += 4;
            view.setFloat32(offset, positions[vIdx + 2], true); offset += 4;
        }

        // Attribute byte count (unused)
        view.setUint16(offset, 0, true); offset += 2;
    }

    return buffer;
}

/**
 * Download file
 * ファイルをダウンロード
 */
function downloadFile(data, filename, mimeType) {
    const blob = new Blob([data], { type: mimeType });
    const url = URL.createObjectURL(blob);
    const a = document.createElement('a');
    a.href = url;
    a.download = filename;
    document.body.appendChild(a);
    a.click();
    document.body.removeChild(a);
    URL.revokeObjectURL(url);
}

/**
 * Export all transformed parts
 * 全変換済みパーツをエクスポート
 */
async function exportAllParts() {
    const loading = document.getElementById('loading');
    loading.style.display = 'block';

    const backupOriginal = document.getElementById('backup-original').checked;
    const matrix = getTransformMatrix();
    const offset = getOffset();

    try {
        // First, download backup files if requested
        if (backupOriginal) {
            for (const partConfig of partsConfig.parts) {
                const original = originalGeometries[partConfig.name];
                if (!original) continue;

                // Generate original STL data
                const stlData = generateBinarySTL(original.positions, original.normals);
                const bakFilename = partConfig.file.replace('.stl', '_BAK.stl');

                downloadFile(stlData, bakFilename, 'application/octet-stream');

                // Wait between downloads
                await new Promise(resolve => setTimeout(resolve, 300));
            }

            // Also backup parts_config.json
            const configBackup = JSON.stringify(partsConfig, null, 2);
            downloadFile(configBackup, 'parts_config_BAK.json', 'application/json');
            await new Promise(resolve => setTimeout(resolve, 300));
        }

        // Then download transformed files
        for (const partConfig of partsConfig.parts) {
            const original = originalGeometries[partConfig.name];
            if (!original) continue;

            // Transform positions
            const transformedPositions = transformPositions(original.positions, matrix, offset);
            const transformedNormals = transformNormals(original.normals, matrix);

            // Generate transformed STL
            const stlData = generateBinarySTL(transformedPositions, transformedNormals);
            downloadFile(stlData, partConfig.file, 'application/octet-stream');

            // Wait between downloads
            await new Promise(resolve => setTimeout(resolve, 300));
        }

        // Download updated parts_config.json (same content, transformation is in STL)
        const updatedConfig = JSON.stringify(partsConfig, null, 2);
        downloadFile(updatedConfig, 'parts_config.json', 'application/json');

        alert('Export completed! エクスポート完了！\n\nCheck your Downloads folder.\nダウンロードフォルダを確認してください。');

    } catch (error) {
        console.error('Export failed:', error);
        alert('Export failed: ' + error.message);
    }

    loading.style.display = 'none';
}

/**
 * Apply preset transformation
 * プリセット変換を適用
 */
function applyPreset(preset) {
    switch (preset) {
        case 'identity':
            document.getElementById('map-x').value = '+X';
            document.getElementById('map-y').value = '+Y';
            document.getElementById('map-z').value = '+Z';
            break;
        case 'ned':
            // Convert from typical CAD (Z-up) to NED (X-forward, Y-right, Z-down)
            document.getElementById('map-x').value = '+Y';
            document.getElementById('map-y').value = '+X';
            document.getElementById('map-z').value = '-Z';
            break;
        case 'enu':
            // Convert to ENU (East-North-Up)
            document.getElementById('map-x').value = '+Y';
            document.getElementById('map-y').value = '+X';
            document.getElementById('map-z').value = '+Z';
            break;
        case 'swap-xy':
            document.getElementById('map-x').value = '+Y';
            document.getElementById('map-y').value = '+X';
            document.getElementById('map-z').value = '+Z';
            break;
        case 'swap-xz':
            document.getElementById('map-x').value = '+Z';
            document.getElementById('map-y').value = '+Y';
            document.getElementById('map-z').value = '+X';
            break;
        case 'swap-yz':
            document.getElementById('map-x').value = '+X';
            document.getElementById('map-y').value = '+Z';
            document.getElementById('map-z').value = '+Y';
            break;
    }

    applyPreview();
}

/**
 * Load all parts
 * 全パーツを読み込み
 */
async function loadAllParts() {
    const loading = document.getElementById('loading');
    loading.style.display = 'block';

    try {
        partsConfig = await loadPartsConfig();

        for (const partConfig of partsConfig.parts) {
            const stlUrl = STL_BASE_PATH + partConfig.file;

            try {
                const geometryData = await loadSTL(stlUrl);
                originalGeometries[partConfig.name] = geometryData;
            } catch (error) {
                console.error(`Failed to load ${partConfig.file}:`, error);
            }
        }

        // Apply initial preview
        applyPreview();

        loading.style.display = 'none';

    } catch (error) {
        console.error('Failed to load parts:', error);
        loading.innerHTML = `<div style="color: #ff6b6b;">Error loading parts<br>${error.message}</div>`;
    }
}

/**
 * Initialize event handlers
 * イベントハンドラを初期化
 */
function initEventHandlers() {
    // Transform controls
    document.getElementById('map-x').addEventListener('change', updateMatrixDisplay);
    document.getElementById('map-y').addEventListener('change', updateMatrixDisplay);
    document.getElementById('map-z').addEventListener('change', updateMatrixDisplay);
    document.getElementById('offset-x').addEventListener('change', updateMatrixDisplay);
    document.getElementById('offset-y').addEventListener('change', updateMatrixDisplay);
    document.getElementById('offset-z').addEventListener('change', updateMatrixDisplay);

    // Buttons
    document.getElementById('btn-preview').addEventListener('click', applyPreview);
    document.getElementById('btn-reset').addEventListener('click', resetTransform);
    document.getElementById('btn-center').addEventListener('click', centerToOrigin);
    document.getElementById('btn-export').addEventListener('click', exportAllParts);

    // Presets
    document.getElementById('preset-identity').addEventListener('click', () => applyPreset('identity'));
    document.getElementById('preset-ned').addEventListener('click', () => applyPreset('ned'));
    document.getElementById('preset-enu').addEventListener('click', () => applyPreset('enu'));
    document.getElementById('preset-swap-xy').addEventListener('click', () => applyPreset('swap-xy'));
    document.getElementById('preset-swap-xz').addEventListener('click', () => applyPreset('swap-xz'));
    document.getElementById('preset-swap-yz').addEventListener('click', () => applyPreset('swap-yz'));
}

/**
 * Main initialization
 * メイン初期化
 */
function init() {
    initScene();
    initEventHandlers();
    loadAllParts();
}

// Start when DOM is ready
document.addEventListener('DOMContentLoaded', init);
