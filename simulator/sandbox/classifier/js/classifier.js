/**
 * STL Part Classifier
 * STLパーツ分類ツール
 *
 * Load original STL file and classify triangles by index ranges.
 * オリジナルSTLファイルを読み込み、インデックス範囲で三角形を分類。
 */

// Global state
let scene, camera, renderer, controls;
let currentGeometry = null;
let mainMesh = null;
let triangleData = [];  // Array of {vertices: [...], normal: [...]}
let totalTriangles = 0;

// Classifications: array of {partId, start, end, id}
let classifications = [];
let classificationIdCounter = 0;

// Visibility state for classified parts
let classificationVisibility = {};  // partId -> boolean

// Part definitions
const PART_DEFINITIONS = {
    frame: { name: 'Frame', color: [0.9, 0.9, 0.9] },
    m5stamps3: { name: 'M5StampS3', color: [0.3, 0.8, 0.3] },
    pcb: { name: 'PCB', color: [0.2, 0.5, 0.8] },
    motor_fr: { name: 'Motor FR', color: [0.5, 0.5, 0.5] },
    motor_rr: { name: 'Motor RR', color: [0.5, 0.5, 0.5] },
    motor_rl: { name: 'Motor RL', color: [0.5, 0.5, 0.5] },
    motor_fl: { name: 'Motor FL', color: [0.5, 0.5, 0.5] },
    propeller_fr: { name: 'Propeller FR', color: [0.2, 0.2, 0.2] },
    propeller_rr: { name: 'Propeller RR', color: [0.2, 0.2, 0.2] },
    propeller_rl: { name: 'Propeller RL', color: [0.2, 0.2, 0.2] },
    propeller_fl: { name: 'Propeller FL', color: [0.2, 0.2, 0.2] },
    battery: { name: 'Battery', color: [0.6, 0.3, 0.1] }
};

// Pre-defined classifications based on connected component analysis
// These are the CORRECT index ranges for each auto-detected part
// Analyzed using scipy connected_components on shared vertices
const AUTO_CLASSIFICATIONS = [
    // Main body (Component 0): 6096 triangles, indices 0-4519 + scattered
    // NOT contiguous - needs manual classification into frame/m5stamps3/pcb

    // Motors (208 triangles each) - positions verified by centroid analysis
    { partId: 'motor_fr', start: 4520, end: 4727 },      // Component 1: X=13.1, Z=22.8 (Right Front)
    { partId: 'motor_fl', start: 5418, end: 5625 },      // Component 3: X=-32.5, Z=22.8 (Left Front)
    { partId: 'motor_rr', start: 5626, end: 5833 },      // Component 4: X=13.1, Z=-22.8 (Right Rear)
    { partId: 'motor_rl', start: 5834, end: 6041 },      // Component 5: X=-32.5, Z=-22.8 (Left Rear)

    // Propellers (690 triangles each) - positions verified by centroid analysis
    { partId: 'propeller_fr', start: 4728, end: 5417 },  // Component 2: X=13.1, Z=22.8 (Right Front)
    { partId: 'propeller_rl', start: 6042, end: 6731 },  // Component 6: X=-32.5, Z=-22.8 (Left Rear)
    { partId: 'propeller_fl', start: 6732, end: 7421 },  // Component 7: X=-32.5, Z=22.8 (Left Front)
    { partId: 'propeller_rr', start: 7422, end: 8111 },  // Component 8: X=13.1, Z=-22.8 (Right Rear)

    // Battery (52 triangles) - center position
    { partId: 'battery', start: 9252, end: 9303 }        // Component 9: X=3.5, Z=0.0 (Center)
];

/**
 * Initialize the 3D scene
 */
function initScene() {
    const canvas = document.getElementById('canvas');
    const container = document.getElementById('viewer');

    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1a2e);

    camera = new THREE.PerspectiveCamera(
        45,
        container.clientWidth / container.clientHeight,
        0.001,
        1000
    );
    camera.position.set(0.15, 0.15, 0.15);

    renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    renderer.setPixelRatio(window.devicePixelRatio);

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

    // Grid
    const gridHelper = new THREE.GridHelper(0.2, 20, 0x444444, 0x333333);
    gridHelper.rotation.x = Math.PI / 2;
    scene.add(gridHelper);

    // Axis arrows
    createAxisArrows();

    window.addEventListener('resize', onWindowResize);
    animate();
}

/**
 * Create axis arrows
 */
function createAxisArrows() {
    const axisLength = 0.08;
    const arrowHeadLength = 0.015;
    const arrowHeadWidth = 0.008;

    const xArrow = new THREE.ArrowHelper(
        new THREE.Vector3(1, 0, 0),
        new THREE.Vector3(0, 0, 0),
        axisLength, 0xff0000, arrowHeadLength, arrowHeadWidth
    );
    scene.add(xArrow);

    const yArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 1, 0),
        new THREE.Vector3(0, 0, 0),
        axisLength, 0x00ff00, arrowHeadLength, arrowHeadWidth
    );
    scene.add(yArrow);

    const zArrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, 1),
        new THREE.Vector3(0, 0, 0),
        axisLength, 0x0088ff, arrowHeadLength, arrowHeadWidth
    );
    scene.add(zArrow);
}

/**
 * Handle window resize
 */
function onWindowResize() {
    const container = document.getElementById('viewer');
    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

/**
 * Animation loop
 */
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}

/**
 * Load STL file
 */
async function loadSTL(url) {
    const loading = document.getElementById('loading');
    loading.style.display = 'block';

    return new Promise((resolve, reject) => {
        const loader = new THREE.STLLoader();

        loader.load(
            url,
            (geometry) => {
                loading.style.display = 'none';
                resolve(geometry);
            },
            undefined,
            (error) => {
                loading.style.display = 'none';
                reject(error);
            }
        );
    });
}

/**
 * Extract triangle data from geometry
 */
function extractTriangleData(geometry) {
    const positions = geometry.attributes.position.array;
    const normals = geometry.attributes.normal ? geometry.attributes.normal.array : null;
    const triangles = [];

    for (let i = 0; i < positions.length; i += 9) {
        const vertices = [
            [positions[i], positions[i + 1], positions[i + 2]],
            [positions[i + 3], positions[i + 4], positions[i + 5]],
            [positions[i + 6], positions[i + 7], positions[i + 8]]
        ];

        let normal = [0, 0, 1];
        if (normals) {
            normal = [normals[i], normals[i + 1], normals[i + 2]];
        }

        triangles.push({ vertices, normal });
    }

    return triangles;
}

/**
 * Display mesh with classifications
 */
function displayMesh() {
    // Remove existing meshes
    if (mainMesh) {
        scene.remove(mainMesh);
        mainMesh.geometry.dispose();
        mainMesh.material.dispose();
    }

    if (triangleData.length === 0) return;

    // Create geometry with vertex colors
    const geometry = new THREE.BufferGeometry();
    const positions = [];
    const colors = [];

    const start = parseInt(document.getElementById('range-start').value) || 0;
    const end = parseInt(document.getElementById('range-end').value) || 0;

    for (let i = 0; i < triangleData.length; i++) {
        const tri = triangleData[i];

        // Check if classified
        const classification = getClassificationForIndex(i);

        // Skip if classified and hidden
        if (classification) {
            const isVisible = classificationVisibility[classification.partId] !== false;
            if (!isVisible) {
                continue;
            }
        }

        // Determine color based on classification or selection
        let color = [0.5, 0.5, 0.5];  // Default gray (unclassified)

        if (classification) {
            const partDef = PART_DEFINITIONS[classification.partId];
            if (partDef) {
                color = partDef.color;
            }
        }

        // Check if in selection range (highlight)
        const inSelection = i >= start && i <= end;
        if (inSelection && !classification) {
            color = [1.0, 0.8, 0.2];  // Yellow highlight for selection
        } else if (inSelection && classification) {
            // Brighten the color for selected classified triangles
            color = [
                Math.min(1, color[0] + 0.3),
                Math.min(1, color[1] + 0.3),
                Math.min(1, color[2] + 0.1)
            ];
        }

        // Add triangle vertices
        for (const v of tri.vertices) {
            positions.push(v[0] * 0.001, v[1] * 0.001, v[2] * 0.001);
            colors.push(color[0], color[1], color[2]);
        }
    }

    geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
    geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    geometry.computeVertexNormals();

    const material = new THREE.MeshPhongMaterial({
        vertexColors: true,
        side: THREE.DoubleSide,
        flatShading: true
    });

    mainMesh = new THREE.Mesh(geometry, material);
    mainMesh.scale.y = -1;
    scene.add(mainMesh);
}

/**
 * Get classification for a triangle index
 */
function getClassificationForIndex(index) {
    for (const c of classifications) {
        if (index >= c.start && index <= c.end) {
            return c;
        }
    }
    return null;
}

/**
 * Check if range overlaps with existing classifications (excluding given id)
 */
function hasOverlap(start, end, excludeId = null) {
    for (const c of classifications) {
        if (excludeId !== null && c.id === excludeId) continue;
        if (!(end < c.start || start > c.end)) {
            return true;
        }
    }
    return false;
}

/**
 * Update range sliders and inputs
 */
function updateRangeUI() {
    const start = parseInt(document.getElementById('range-start').value) || 0;
    const end = parseInt(document.getElementById('range-end').value) || 0;

    document.getElementById('range-slider-start').value = start;
    document.getElementById('range-slider-end').value = end;

    const count = Math.max(0, end - start + 1);
    document.getElementById('selection-info').textContent =
        `Selected: ${start} - ${end} (${count} triangles)`;

    // Check if selection overlaps with existing classification
    const overlap = hasOverlap(start, end);

    const assignBtn = document.getElementById('btn-assign');
    const selectedPart = document.querySelector('input[name="part"]:checked');
    assignBtn.disabled = !selectedPart || count === 0 || overlap;

    if (overlap) {
        document.getElementById('selection-info').textContent += ' - Already classified';
    }

    displayMesh();
}

/**
 * Update classification list UI
 */
function updateClassificationList() {
    const list = document.getElementById('classification-list');
    list.innerHTML = '';

    if (classifications.length === 0) {
        list.innerHTML = '<div style="color: #666; font-size: 0.85em;">No classifications yet</div>';
    } else {
        // Group by partId
        const partGroups = {};
        for (const c of classifications) {
            if (!partGroups[c.partId]) {
                partGroups[c.partId] = [];
            }
            partGroups[c.partId].push(c);
        }

        for (const [partId, ranges] of Object.entries(partGroups)) {
            const partDef = PART_DEFINITIONS[partId];
            const color = partDef.color;
            const isVisible = classificationVisibility[partId] !== false;

            // Calculate total triangles
            const totalTris = ranges.reduce((sum, r) => sum + (r.end - r.start + 1), 0);

            // Sort ranges
            ranges.sort((a, b) => a.start - b.start);

            const item = document.createElement('div');
            item.className = 'classification-item';
            item.innerHTML = `
                <input type="checkbox" class="visibility-toggle" data-part="${partId}" ${isVisible ? 'checked' : ''}>
                <span class="name">
                    <div class="part-color" style="background: rgb(${Math.round(color[0]*255)},${Math.round(color[1]*255)},${Math.round(color[2]*255)})"></div>
                    ${partDef.name} (${totalTris})
                </span>
                <button class="edit-btn" data-part="${partId}" title="Edit ranges">&#9998;</button>
                <button class="delete-btn" data-part="${partId}" title="Delete all">&times;</button>
            `;

            list.appendChild(item);

            // Add sub-items for each range
            for (const range of ranges) {
                const rangeItem = document.createElement('div');
                rangeItem.className = 'classification-range';
                rangeItem.innerHTML = `
                    <span>${range.start} - ${range.end}</span>
                    <button class="range-select-btn" data-start="${range.start}" data-end="${range.end}" title="Select this range">&#x2192;</button>
                    <button class="range-delete-btn" data-id="${range.id}" title="Delete this range">&times;</button>
                `;
                list.appendChild(rangeItem);
            }
        }

        // Add event handlers
        list.querySelectorAll('.visibility-toggle').forEach(checkbox => {
            checkbox.addEventListener('change', (e) => {
                classificationVisibility[e.target.dataset.part] = e.target.checked;
                displayMesh();
            });
        });

        list.querySelectorAll('.delete-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const partId = btn.dataset.part;
                if (confirm(`Delete all ${PART_DEFINITIONS[partId].name} classifications?`)) {
                    classifications = classifications.filter(c => c.partId !== partId);
                    delete classificationVisibility[partId];
                    updateClassificationList();
                    updateProgress();
                    updateRangeUI();
                }
            });
        });

        list.querySelectorAll('.range-select-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                document.getElementById('range-start').value = btn.dataset.start;
                document.getElementById('range-end').value = btn.dataset.end;
                updateRangeUI();
            });
        });

        list.querySelectorAll('.range-delete-btn').forEach(btn => {
            btn.addEventListener('click', () => {
                const id = parseInt(btn.dataset.id);
                classifications = classifications.filter(c => c.id !== id);
                updateClassificationList();
                updateProgress();
                updateRangeUI();
            });
        });
    }
}

/**
 * Update progress bar and counts
 */
function updateProgress() {
    let classified = 0;
    for (const c of classifications) {
        classified += c.end - c.start + 1;
    }

    const unclassified = totalTriangles - classified;
    const percent = totalTriangles > 0 ? (classified / totalTriangles * 100) : 0;

    document.getElementById('progress-fill').style.width = `${percent}%`;
    document.getElementById('classified-count').textContent =
        `Classified: ${classified} / ${totalTriangles}`;
    document.getElementById('unclassified-count').textContent =
        `Unclassified: ${unclassified}`;
}

/**
 * Find next unclassified range
 */
function findNextUnclassifiedRange(fromIndex = 0) {
    // Sort classifications by start
    const sorted = [...classifications].sort((a, b) => a.start - b.start);

    let searchStart = fromIndex;

    for (const c of sorted) {
        if (c.start > searchStart) {
            // Found a gap before this classification
            return { start: searchStart, end: c.start - 1 };
        }
        searchStart = Math.max(searchStart, c.end + 1);
    }

    // Check if there's unclassified space after all classifications
    if (searchStart < totalTriangles) {
        return { start: searchStart, end: totalTriangles - 1 };
    }

    return null;
}

/**
 * Export all classified parts as STL files
 */
async function exportAllParts() {
    if (classifications.length === 0) {
        alert('No classifications to export');
        return;
    }

    // Group by partId
    const partGroups = {};
    for (const c of classifications) {
        if (!partGroups[c.partId]) {
            partGroups[c.partId] = [];
        }
        partGroups[c.partId].push(c);
    }

    // Export each part
    let exportCount = 0;
    for (const [partId, ranges] of Object.entries(partGroups)) {
        const partDef = PART_DEFINITIONS[partId];
        const triangles = [];

        for (const range of ranges) {
            for (let i = range.start; i <= range.end; i++) {
                if (triangleData[i]) {
                    triangles.push(triangleData[i]);
                }
            }
        }

        if (triangles.length > 0) {
            const stlContent = createSTLBinary(triangles, partDef.name);
            downloadBlob(stlContent, `${partId}.stl`, 'application/octet-stream');
            exportCount++;
        }
    }

    // Create parts_config.json
    const config = {
        method: 'manual_classification',
        source: 'stampfly_v1.stl',
        parts: Object.entries(partGroups)
            .map(([partId, ranges]) => {
                const totalTris = ranges.reduce((sum, r) => sum + (r.end - r.start + 1), 0);
                return {
                    name: partId,
                    file: `${partId}.stl`,
                    triangles: totalTris,
                    color: PART_DEFINITIONS[partId].color,
                    ranges: ranges.map(r => ({ start: r.start, end: r.end }))
                };
            })
    };

    const configBlob = new Blob([JSON.stringify(config, null, 2)], { type: 'application/json' });
    downloadBlob(configBlob, 'parts_config.json', 'application/json');

    alert(`Exported ${exportCount} STL files and parts_config.json`);
}

/**
 * Create binary STL content
 */
function createSTLBinary(triangles, name) {
    const headerSize = 80;
    const triangleSize = 50;
    const bufferSize = headerSize + 4 + triangles.length * triangleSize;
    const buffer = new ArrayBuffer(bufferSize);
    const view = new DataView(buffer);

    // Header
    const header = `Binary STL - ${name}`;
    for (let i = 0; i < 80; i++) {
        view.setUint8(i, i < header.length ? header.charCodeAt(i) : 0);
    }

    // Triangle count
    view.setUint32(80, triangles.length, true);

    // Triangles
    let offset = 84;
    for (const tri of triangles) {
        // Normal
        view.setFloat32(offset, tri.normal[0], true); offset += 4;
        view.setFloat32(offset, tri.normal[1], true); offset += 4;
        view.setFloat32(offset, tri.normal[2], true); offset += 4;

        // Vertices
        for (const v of tri.vertices) {
            view.setFloat32(offset, v[0], true); offset += 4;
            view.setFloat32(offset, v[1], true); offset += 4;
            view.setFloat32(offset, v[2], true); offset += 4;
        }

        // Attribute byte count
        view.setUint16(offset, 0, true); offset += 2;
    }

    return new Blob([buffer], { type: 'application/octet-stream' });
}

/**
 * Download blob as file
 */
function downloadBlob(blob, filename, mimeType) {
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
 * Initialize event handlers
 */
function initEventHandlers() {
    // Range inputs
    document.getElementById('range-start').addEventListener('input', updateRangeUI);
    document.getElementById('range-end').addEventListener('input', updateRangeUI);

    // Range sliders
    document.getElementById('range-slider-start').addEventListener('input', (e) => {
        document.getElementById('range-start').value = e.target.value;
        updateRangeUI();
    });
    document.getElementById('range-slider-end').addEventListener('input', (e) => {
        document.getElementById('range-end').value = e.target.value;
        updateRangeUI();
    });

    // Part category selection
    document.querySelectorAll('.part-category').forEach(el => {
        el.addEventListener('click', () => {
            const radio = el.querySelector('input[type="radio"]');
            radio.checked = true;

            document.querySelectorAll('.part-category').forEach(p => p.classList.remove('selected'));
            el.classList.add('selected');

            updateRangeUI();
        });
    });

    // Assign button
    document.getElementById('btn-assign').addEventListener('click', () => {
        const start = parseInt(document.getElementById('range-start').value);
        const end = parseInt(document.getElementById('range-end').value);
        const selectedPart = document.querySelector('input[name="part"]:checked');

        if (selectedPart && start <= end && !hasOverlap(start, end)) {
            classifications.push({
                id: classificationIdCounter++,
                partId: selectedPart.value,
                start: start,
                end: end
            });

            // Move to next unclassified range
            const nextRange = findNextUnclassifiedRange(end + 1);
            if (nextRange) {
                document.getElementById('range-start').value = nextRange.start;
                document.getElementById('range-end').value = Math.min(nextRange.start + 500, nextRange.end);
            }

            updateClassificationList();
            updateProgress();
            updateRangeUI();
        }
    });

    // Export button
    document.getElementById('btn-export').addEventListener('click', exportAllParts);

    // Reset button
    document.getElementById('btn-reset').addEventListener('click', () => {
        if (confirm('Reset all classifications? (Auto-detected parts will be restored)')) {
            initAutoClassifications();
            updateClassificationList();
            updateProgress();
            updateRangeUI();
        }
    });

    // Show All button
    document.getElementById('btn-show-all').addEventListener('click', () => {
        for (const partId of Object.keys(PART_DEFINITIONS)) {
            classificationVisibility[partId] = true;
        }
        updateClassificationList();
        displayMesh();
    });

    // Hide All button
    document.getElementById('btn-hide-all').addEventListener('click', () => {
        for (const partId of Object.keys(PART_DEFINITIONS)) {
            classificationVisibility[partId] = false;
        }
        updateClassificationList();
        displayMesh();
    });
}

/**
 * Initialize auto classifications
 */
function initAutoClassifications() {
    classifications = [];
    classificationIdCounter = 0;
    classificationVisibility = {};

    for (const auto of AUTO_CLASSIFICATIONS) {
        classifications.push({
            id: classificationIdCounter++,
            partId: auto.partId,
            start: auto.start,
            end: auto.end
        });
    }
}

/**
 * Load and initialize
 */
async function loadAndInit() {
    try {
        // Load original STL file
        const geometry = await loadSTL('../assets/meshes/stampfly_v1.stl');
        currentGeometry = geometry;
        triangleData = extractTriangleData(geometry);
        totalTriangles = triangleData.length;

        document.getElementById('triangle-count').textContent = `Total: ${totalTriangles} triangles`;

        // Update slider max values
        document.getElementById('range-slider-start').max = totalTriangles - 1;
        document.getElementById('range-slider-end').max = totalTriangles - 1;
        document.getElementById('range-start').max = totalTriangles - 1;
        document.getElementById('range-end').max = totalTriangles - 1;

        // Initialize auto classifications
        initAutoClassifications();

        // Set initial range to first unclassified area
        const firstUnclassified = findNextUnclassifiedRange(0);
        if (firstUnclassified) {
            document.getElementById('range-start').value = firstUnclassified.start;
            document.getElementById('range-end').value = Math.min(firstUnclassified.start + 500, firstUnclassified.end);
        }

        updateClassificationList();
        updateProgress();
        updateRangeUI();

    } catch (error) {
        console.error('Failed to load STL:', error);
        document.getElementById('loading').innerHTML =
            `<div style="color: #ff6b6b;">Failed to load STL<br>${error.message}</div>`;
    }
}

/**
 * Main initialization
 */
function init() {
    initScene();
    initEventHandlers();
    loadAndInit();
}

// Start when DOM is ready
document.addEventListener('DOMContentLoaded', init);
