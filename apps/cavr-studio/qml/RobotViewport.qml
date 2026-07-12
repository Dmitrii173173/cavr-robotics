import QtQuick
import QtQuick3D
import QtQuick3D.Helpers
import QtQuick3D.AssetUtils

// Live 3D viewport for the articulated Yaskawa GP25 asset. Loads gp25.glb,
// walks its joint chain, drives the six revolute joints, and re-skins the links
// with tuned PBR materials (industrial-orange paint, machined-steel flange,
// cast-metal base). Joint axes (joint-local frame): S:+Y L:+X U:+X R:+Z B:+X T:+Z,
// matching cavr::visualization::yaskawa_gp25().
Item {
    id: root

    property url robotSource: (typeof robotUrl !== "undefined") ? robotUrl : ""
    property bool animate: true
    // when a `robot` controller context property is present, joints follow live
    // telemetry; otherwise the standalone demo animation runs.
    property bool useTelemetry: (typeof robot !== "undefined") && robot !== null

    property var joints: []
    property var jointAxis: ["y", "x", "x", "z", "x", "z"]
    property var tcpNode: null
    property real phase: 0
    property var visualFeatures: root.useTelemetry ? robot.visualFeatures : []
    property var visualZones: root.useTelemetry ? robot.visualZones : []
    property var visualPointCloud: root.useTelemetry ? robot.visualPointCloud : []
    property var visualPathSegments: root.useTelemetry ? robot.visualPathSegments : []
    property var visualDiagnostics: root.useTelemetry ? robot.visualDiagnostics : []

    function primitiveScale(v) { return Math.max(v / 100.0, 0.00025) }

    // studio backdrop (View3D draws transparently over this)
    Rectangle {
        anchors.fill: parent
        gradient: Gradient {
            GradientStop { position: 0.0; color: "#1b2530" }
            GradientStop { position: 0.55; color: "#121a23" }
            GradientStop { position: 1.0; color: "#0b0f15" }
        }
    }

    View3D {
        id: view
        anchors.fill: parent

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.SkyBox
            lightProbe: Texture {
                textureData: ProceduralSkyTextureData {
                    skyTopColor: "#7d93ab"
                    skyHorizonColor: "#b9c8d8"
                    groundBottomColor: "#2b333d"
                    groundHorizonColor: "#5a6675"
                    skyCurve: 0.1
                    groundCurve: 0.05
                    sunColor: "#fff3e0"
                }
            }
            probeExposure: 1.0
            skyboxBlurAmount: 0.5
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.VeryHigh
            specularAAEnabled: true        // tame shimmer on the metal parts
            aoStrength: 40
            aoDistance: 0.6
            aoSoftness: 20
            InfiniteGrid { gridInterval: 0.5 }
        }

        Node {
            id: camOrigin
            position: Qt.vector3d(0, 0.80, 0)
            PerspectiveCamera {
                id: camera
                position: Qt.vector3d(0, 0, 5.6)
                clipNear: 0.1            // tight range = better depth precision (less z-fighting)
                clipFar: 60
                fieldOfView: 45
            }
        }

        // key light with soft shadows; IBL provides ambient fill
        DirectionalLight {
            eulerRotation: Qt.vector3d(-52, -38, 0)
            brightness: 1.7
            color: "#fff4e6"
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityVeryHigh
            shadowFactor: 78
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(-8, 135, 0)
            brightness: 0.55
            color: "#cfe0ff"
        }
        DirectionalLight {
            eulerRotation: Qt.vector3d(35, 20, 0)
            brightness: 0.4
            color: "#ffffff"
        }

        // metals need something to reflect
        ReflectionProbe {
            position: Qt.vector3d(0, 0.8, 0)
            boxSize: Qt.vector3d(5, 5, 5)
            quality: ReflectionProbe.High
            refreshMode: ReflectionProbe.FirstFrame
            parallaxCorrection: true
        }

        // shadow-catching floor under the grid
        Model {
            source: "#Rectangle"
            eulerRotation.x: -90
            scale: Qt.vector3d(0.4, 0.4, 0.4)
            position: Qt.vector3d(0, -0.002, 0)
            receivesShadows: true
            materials: PrincipledMaterial {
                baseColor: "#0d141c"
                roughness: 0.55
                metalness: 0.0
            }
        }

        // Demo workpiece: simple welded steel fixture used when there is no scan.
        Node {
            id: workpiece
            Model {
                source: "#Cube"
                position: Qt.vector3d(0.64, -0.07, 0.38)
                scale: Qt.vector3d(root.primitiveScale(0.46), root.primitiveScale(0.30), root.primitiveScale(0.10))
                receivesShadows: true
                castsShadows: true
                materials: PrincipledMaterial {
                    baseColor: "#586270"
                    metalness: 0.85
                    roughness: 0.42
                }
            }
            Model {
                source: "#Cube"
                position: Qt.vector3d(0.64, 0.05, 0.46)
                scale: Qt.vector3d(root.primitiveScale(0.38), root.primitiveScale(0.05), root.primitiveScale(0.16))
                receivesShadows: true
                castsShadows: true
                materials: PrincipledMaterial {
                    baseColor: "#4b5563"
                    metalness: 0.8
                    roughness: 0.38
                }
            }
        }

        // Transparent process zones: work envelope, slowdown, weld window,
        // forbidden clamp volume, safety boundary.
        Repeater3D {
            model: root.visualZones
            delegate: Model {
                source: "#Cube"
                position: Qt.vector3d(modelData.x, modelData.y, modelData.z)
                scale: Qt.vector3d(root.primitiveScale(modelData.sx),
                                   root.primitiveScale(modelData.sy),
                                   root.primitiveScale(modelData.sz))
                pickable: false
                materials: PrincipledMaterial {
                    baseColor: modelData.color
                    alphaMode: PrincipledMaterial.Blend
                    opacity: modelData.type === "forbidden" ? 0.28 : 0.15
                    roughness: 0.65
                    metalness: 0.0
                }
            }
        }

        // Imported/demo point cloud, kept lightweight for interactive editing.
        Repeater3D {
            model: root.visualPointCloud
            delegate: Model {
                source: "#Sphere"
                position: Qt.vector3d(modelData.x, modelData.y, modelData.z)
                scale: Qt.vector3d(root.primitiveScale(0.012),
                                   root.primitiveScale(0.012),
                                   root.primitiveScale(0.012))
                pickable: false
                materials: PrincipledMaterial {
                    baseColor: "#8fb8ff"
                    alphaMode: PrincipledMaterial.Blend
                    opacity: 0.55
                    roughness: 0.3
                }
            }
        }

        // Selectable weld features: lines, edges, contours and face-derived paths.
        Repeater3D {
            model: root.visualFeatures
            delegate: Model {
                property int featureIndex: modelData.index
                source: "#Cube"
                position: Qt.vector3d(modelData.cx, modelData.cy, modelData.cz + 0.006)
                eulerRotation: Qt.vector3d(0, 0, modelData.yaw)
                scale: Qt.vector3d(root.primitiveScale(modelData.length),
                                   root.primitiveScale(modelData.selected ? 0.030 : modelData.width),
                                   root.primitiveScale(modelData.selected ? 0.030 : modelData.width))
                pickable: true
                materials: PrincipledMaterial {
                    baseColor: modelData.selected ? "#fff06a"
                              : modelData.hasOperation ? "#29d47d"
                              : modelData.kind === "face" ? "#b45cff" : "#65b7ff"
                    emissiveFactor: modelData.selected ? Qt.vector3d(0.45, 0.36, 0.05)
                                                        : Qt.vector3d(0.02, 0.05, 0.08)
                    roughness: 0.22
                    metalness: 0.0
                }
            }
        }

        // Compiled robot path phases. Timeline is a readout of these generated
        // phases: move, approach, weld/process, retract.
        Repeater3D {
            model: root.visualPathSegments
            delegate: Model {
                source: "#Cube"
                position: Qt.vector3d(modelData.x, modelData.y, modelData.z + 0.018)
                eulerRotation: Qt.vector3d(0, -modelData.pitch, modelData.yaw)
                scale: Qt.vector3d(root.primitiveScale(modelData.length),
                                   root.primitiveScale(modelData.selected ? 0.020 : 0.012),
                                   root.primitiveScale(modelData.selected ? 0.020 : 0.012))
                pickable: false
                materials: PrincipledMaterial {
                    baseColor: modelData.color
                    emissiveFactor: modelData.phase === "weld"
                                    ? Qt.vector3d(0.02, 0.22, 0.08)
                                    : Qt.vector3d(0.03, 0.05, 0.08)
                    roughness: 0.25
                }
            }
        }

        // Diagnostic markers are spatial, so the operator can see why validation
        // failed without hunting through a log.
        Repeater3D {
            model: root.visualDiagnostics
            delegate: Model {
                source: "#Sphere"
                position: Qt.vector3d(modelData.x, modelData.y, modelData.z + 0.045)
                scale: Qt.vector3d(root.primitiveScale(modelData.severity === "error" ? 0.040 : 0.030),
                                   root.primitiveScale(modelData.severity === "error" ? 0.040 : 0.030),
                                   root.primitiveScale(modelData.severity === "error" ? 0.040 : 0.030))
                pickable: false
                materials: PrincipledMaterial {
                    baseColor: modelData.severity === "error" ? "#ff4d5e" : "#ffb020"
                    emissiveFactor: modelData.severity === "error"
                                    ? Qt.vector3d(0.35, 0.03, 0.05)
                                    : Qt.vector3d(0.28, 0.12, 0.02)
                }
            }
        }

        // ---- tuned materials (replace the glTF-baked ones) ----
        PrincipledMaterial {
            id: paintOrange
            baseColor: "#e85f12"          // industrial orange paint
            metalness: 0.0
            roughness: 0.34
            clearcoatAmount: 0.7          // glossy painted lacquer
            clearcoatRoughnessAmount: 0.12
        }
        PrincipledMaterial {
            id: machinedSteel
            baseColor: "#c9ccd1"          // bright machined steel
            metalness: 1.0
            roughness: 0.34               // a touch rougher to stop specular shimmer
        }
        PrincipledMaterial {
            id: castMetal
            baseColor: "#3b3f45"          // dark cast-aluminium base
            metalness: 1.0
            roughness: 0.55
        }
        PrincipledMaterial {
            id: darkTrim
            baseColor: "#15181c"          // motor caps / wrist drive
            metalness: 0.85
            roughness: 0.35
        }

        RuntimeLoader {
            id: importer
            source: root.robotSource
            onStatusChanged: {
                if (status === RuntimeLoader.Success)
                    Qt.callLater(root.setup)
                else if (status === RuntimeLoader.Error)
                    console.warn("GP25 load error:", errorString)
            }
        }

        OrbitCameraController {
            origin: camOrigin
            camera: camera
            panEnabled: true
        }

        MouseArea {
            anchors.fill: parent
            acceptedButtons: Qt.LeftButton
            propagateComposedEvents: true
            onClicked: function(mouse) {
                if (!root.useTelemetry) {
                    mouse.accepted = false
                    return
                }
                var hit = view.pick(mouse.x, mouse.y)
                if (hit.objectHit && hit.objectHit.featureIndex !== undefined) {
                    robot.selectVisualFeature(hit.objectHit.featureIndex)
                    mouse.accepted = true
                } else {
                    mouse.accepted = false
                }
            }
        }
    }

    // ---- TCP coordinate read-out, overlaid at the top-centre of the view ----
    Rectangle {
        id: tcpOverlay
        visible: root.useTelemetry
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.top: parent.top
        anchors.topMargin: 12
        radius: 6
        color: "#d90b1016"
        border.color: "#2b3947"
        border.width: 1
        implicitWidth: coordColumn.implicitWidth + 26
        implicitHeight: coordColumn.implicitHeight + 14

        property var c: root.useTelemetry ? robot.tcpCoords : [0, 0, 0, 0, 0, 0]
        function val(i) { return (c && c.length > i) ? c[i] : 0 }
        function fmt(v) { return (v >= 0 ? " " : "") + v.toFixed(1) }

        Column {
            id: coordColumn
            anchors.centerIn: parent
            spacing: 4

            Text {
                anchors.horizontalCenter: parent.horizontalCenter
                text: "TCP  ·  " + (root.useTelemetry ? robot.coordSystem : "Base")
                      + " frame  ·  mm / deg"
                color: "#7fa9d8"
                font.pixelSize: 11
                font.family: "Menlo, monospace"
            }
            Row {
                anchors.horizontalCenter: parent.horizontalCenter
                spacing: 16
                Text { text: "X" + tcpOverlay.fmt(tcpOverlay.val(0)); color: "#e6ecf2"; font.pixelSize: 14; font.family: "Menlo, monospace" }
                Text { text: "Y" + tcpOverlay.fmt(tcpOverlay.val(1)); color: "#e6ecf2"; font.pixelSize: 14; font.family: "Menlo, monospace" }
                Text { text: "Z" + tcpOverlay.fmt(tcpOverlay.val(2)); color: "#e6ecf2"; font.pixelSize: 14; font.family: "Menlo, monospace" }
                Text { text: "Rx" + tcpOverlay.fmt(tcpOverlay.val(3)); color: "#f0b866"; font.pixelSize: 14; font.family: "Menlo, monospace" }
                Text { text: "Ry" + tcpOverlay.fmt(tcpOverlay.val(4)); color: "#f0b866"; font.pixelSize: 14; font.family: "Menlo, monospace" }
                Text { text: "Rz" + tcpOverlay.fmt(tcpOverlay.val(5)); color: "#f0b866"; font.pixelSize: 14; font.family: "Menlo, monospace" }
            }
        }
    }

    // ---- scene-graph helpers (RuntimeLoader drops glTF node names) ----
    function isNode(o) { return o.toString().indexOf("QQuick3DNode") === 0 }
    function isModel(o) { return o.toString().indexOf("QQuick3DModel") === 0 }
    function firstNodeChild(n) {
        for (var i = 0; i < n.children.length; ++i)
            if (isNode(n.children[i])) return n.children[i]
        return null
    }
    function firstModelChild(n) {
        for (var i = 0; i < n.children.length; ++i)
            if (isModel(n.children[i])) return n.children[i]
        return null
    }
    function findBaseModel(n) {
        if (isModel(n) && firstNodeChild(n)) return n
        for (var i = 0; i < n.children.length; ++i) {
            var r = findBaseModel(n.children[i])
            if (r) return r
        }
        return null
    }

    function setup() {
        var base = findBaseModel(importer)
        if (!base) { console.warn("GP25: base link not found"); return }
        base.materials = [castMetal]

        var j = firstNodeChild(base)      // joint_s
        var found = []
        for (var k = 0; k < 6 && j; ++k) {
            found.push(j)
            var mesh = firstModelChild(j)
            if (mesh)
                mesh.materials = [ (k === 5) ? machinedSteel : paintOrange ]
            j = firstNodeChild(j)
        }
        joints = found
        tcpNode = j
        console.log("GP25 ready: joints =", joints.length)
        if (useTelemetry) applyTelemetry()
        else applyPose()
    }

    // joint angles (degrees) pushed from the controller's live telemetry
    function applyTelemetry() {
        if (!useTelemetry || joints.length < 6) return
        var d = robot.jointDegrees
        if (!d || d.length < 6) return
        for (var i = 0; i < 6; ++i) setJoint(i, d[i])
    }

    Connections {
        target: root.useTelemetry ? robot : null
        function onTelemetryChanged() { root.applyTelemetry() }
        function onVisualProgramChanged() {
            root.visualFeatures = robot.visualFeatures
            root.visualZones = robot.visualZones
            root.visualPointCloud = robot.visualPointCloud
            root.visualPathSegments = robot.visualPathSegments
            root.visualDiagnostics = robot.visualDiagnostics
        }
    }

    function setJoint(i, deg) {
        var n = joints[i]
        if (!n) return
        switch (jointAxis[i]) {
        case "x": n.eulerRotation = Qt.vector3d(deg, 0, 0); break
        case "y": n.eulerRotation = Qt.vector3d(0, deg, 0); break
        case "z": n.eulerRotation = Qt.vector3d(0, 0, deg); break
        }
    }

    function applyPose() {
        if (joints.length < 6) return
        var p = phase
        setJoint(0, 35 * Math.sin(p))
        setJoint(1, 18 * Math.sin(p * 0.8 + 0.5))
        setJoint(2, -22 * Math.sin(p * 0.9))
        setJoint(3, 55 * Math.sin(p * 1.3))
        setJoint(4, 28 * Math.sin(p * 1.1 + 1.0))
        setJoint(5, 80 * Math.sin(p * 1.5))
    }

    onPhaseChanged: if (!useTelemetry) applyPose()

    NumberAnimation on phase {
        running: root.animate && !root.useTelemetry
        from: 0; to: 2 * Math.PI
        duration: 9000
        loops: Animation.Infinite
    }

    Rectangle {
        anchors { left: parent.left; top: parent.top; margins: 12 }
        color: "#cc111821"; border.color: "#2b3947"; radius: 6
        width: label.implicitWidth + 24; height: label.implicitHeight + 16
        Text {
            id: label
            anchors.centerIn: parent
            color: "#e6ecf2"
            font.pixelSize: 13
            text: root.useTelemetry
                  ? ("GP25  ·  " + robot.programState + "  ·  " + robot.stepLabel +
                     (robot.weldActive ? "  ·  ◆ WELD" : ""))
                  : "Yaskawa GP25  ·  6-axis  ·  drag to orbit"
        }
    }

    Rectangle {
        anchors { right: parent.right; top: parent.top; margins: 12 }
        color: "#cc111821"; border.color: "#2b3947"; radius: 6
        width: 320; height: sceneLegend.implicitHeight + 22
        Column {
            id: sceneLegend
            anchors { left: parent.left; right: parent.right; verticalCenter: parent.verticalCenter; margins: 12 }
            spacing: 5
            Text {
                color: "#e6ecf2"
                font.pixelSize: 13
                font.bold: true
                text: "Visual OLP editor"
            }
            Text {
                color: "#9fb0c0"
                font.pixelSize: 12
                wrapMode: Text.WordWrap
                width: parent.width
                text: root.useTelemetry
                      ? "Click a highlighted seam/edge/face, tune operation parameters, then create a compiled robot path."
                      : "Load a controller to enable geometry selection."
            }
            Text {
                color: "#79d98f"
                font.pixelSize: 12
                text: "green path = weld/process, blue = approach, amber = retract"
            }
        }
    }
}
