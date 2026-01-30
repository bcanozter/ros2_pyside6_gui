// Copyright (C) 2024 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

import QtQuick
import QtQuick.Window
import QtQuick3D
import QtQuick3D.Helpers
import OSMBuildings

Window {
    width: 1024
    height: 768
    visible: true
    title: qsTr("Pyside6 GUI")

    QtObject {
        id: mapTileCache
        property var cachedTiles: ({})

        function getTileKey(tileX, tileY, zoomLevel) {
            return `${zoomLevel}_${tileX}_${tileY}`;
        }

        function ensureMapModel(mapData, tileX, tileY, zoomLevel) {
            var key = mapTileCache.getTileKey(tileX, tileY, zoomLevel);

            if (mapTileCache.cachedTiles[key]) {
                return mapTileCache.cachedTiles[key];
            } else {
                var newModel = chunkModelMap.createObject(mapModels, {
                    "mapData": mapData,
                    "tileX": tileX,
                    "tileY": tileY,
                    "zoomLevel": zoomLevel
                });

                if (newModel) {
                    mapTileCache.cachedTiles[key] = newModel;
                    return newModel;
                }
            }
        }

        function removeOffscreenTiles(visibleTileKeys) {
            var keysToRemove = [];
            for (var key in mapTileCache.cachedTiles) {
                if (mapTileCache.cachedTiles.hasOwnProperty(key)) {
                    if (!visibleTileKeys.includes(key)) {
                        keysToRemove.push(key);
                    }
                }
            }

            for (var i = 0; i < keysToRemove.length; i++) {
                var key = keysToRemove[i];
                mapTileCache.cachedTiles[key].destroy();
                delete mapTileCache.cachedTiles[key];
            }
        }
    }
    OSMManager {
        id: osmManager

        onMapsDataReady: function (mapData, tileX, tileY, zoomLevel) {
            mapTileCache.ensureMapModel(mapData, tileX, tileY, zoomLevel);
        //mapModels.addModel(mapData, tileX, tileY, zoomLevel)
        }
    }

    Component {
        id: chunkModelMap
        Node {
            property variant mapData: null
            property int tileX: 0
            property int tileY: 0
            property int zoomLevel: 0
            Model {
                id: basePlane
                position: Qt.vector3d(osmManager && osmManager.tileSizeX * tileX, osmManager && osmManager.tileSizeY * -tileY, 0.0)
                scale: Qt.vector3d(osmManager && osmManager.tileSizeX / 100., osmManager && osmManager.tileSizeY / 100., 0.5)
                source: "#Rectangle"
                materials: [
                    CustomMaterial {
                        property TextureInput tileTexture: TextureInput {
                            enabled: true
                            texture: Texture {
                                textureData: CustomTextureData {
                                    Component.onCompleted: setImageData(mapData)
                                }
                            }
                        }
                        shadingMode: CustomMaterial.Shaded
                        cullMode: Material.BackFaceCulling
                        fragmentShader: "customshadertiles.frag"
                    }
                ]
            }
        }

    }

    View3D {
        id: v3d
        anchors.fill: parent

        environment: ExtendedSceneEnvironment {
            id: env
            backgroundMode: SceneEnvironment.Color
            clearColor: "#8099b3"
            fxaaEnabled: true
            fog: Fog {
                id: theFog
                color: "#8099b3"
                enabled: true
                depthEnabled: true
                depthFar: 600
            }
        }

        Node {
            id: originNode
            eulerRotation: Qt.vector3d(50.0, 0.0, 0.0)
            PerspectiveCamera {
                id: cameraNode
                frustumCullingEnabled: true
                clipFar: 600
                clipNear: 100
                fieldOfView: 90
                z: 100

                onZChanged: originNode.updateManagerCamera()
            }
            Component.onCompleted: updateManagerCamera()

            onPositionChanged: updateManagerCamera()

            onRotationChanged: updateManagerCamera()
            function updateVisibleTiles() {
            }//TODO
            function updateManagerCamera() {
                osmManager.setCameraProperties(originNode.position, originNode.right, cameraNode.z, cameraController.minimumZoom, cameraController.maximumZoom, originNode.eulerRotation.x, cameraController.minimumTilt, cameraController.maximumTilt);
            }
        }

        DirectionalLight {
            color: Qt.rgba(1.0, 1.0, 0.95, 1.0)
            ambientColor: Qt.rgba(0.5, 0.45, 0.45, 1.0)
            rotation: Quaternion.fromEulerAngles(-10, -45, 0)
        }

        Node {
            id: mapModels
            function addModel(mapData, tileX, tileY, zoomLevel) {
                chunkModelMap.createObject(mapModels, {
                    "mapData": mapData,
                    "tileX": tileX,
                    "tileY": tileY,
                    "zoomLevel": zoomLevel
                });
            }
            Model {
                id: robotPlaceHolder
                source: "#Cube" //TODO change model..
                materials: DefaultMaterial {
                    diffuseColor: "red"
                }
                position: Qt.vector3d(0, 0, 0)
                scale: Qt.vector3d(0.04, 0.04, 0.04)

                Connections {
                    target: mavrosHandler
                    function onPositionChanged(latitude, longitude, altitude) {
                        robotPlaceHolder.updatePosition(latitude, longitude, altitude);
                    }
                }

                function updatePosition(latitude, longitude, altitude) {
                    var res = osmManager.deg2num_f(latitude, longitude, 15);
                    // console.log(latitude,longitude)
                    //0.5 is the magic number.. centers it within the tile,
                    //otherwise, the model is referenced to the top left corner of a tile
                    var x = ((res[0]-0.5) - osmManager.startBuildingTileX) * osmManager.tileSizeX
                    var y = -((res[1]-0.5) - osmManager.startBuildingTileY)  * osmManager.tileSizeY
                    robotPlaceHolder.position = Qt.vector3d(x,y,altitude)
                }
            }
        }

        OSMCameraController {
            id: cameraController
            origin: originNode
            camera: cameraNode
        }
    }

    Text {
        id: attributionText
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        anchors.margins: 10
        color: "white"
        font.pixelSize: 12
        text: "© OpenStreetMap contributors"
        MouseArea {
            anchors.fill: parent
            onClicked: osmManager.requestOpenUrl("https://www.openstreetmap.org/copyright")
        }
    }
    // Item {
    //     id: tokenArea
    //     anchors.left: parent.left
    //     anchors.bottom: parent.bottom
    //     anchors.margins: 10
    //     Text {
    //         id: tokenInputArea
    //         visible: false
    //         anchors.left: parent.left
    //         anchors.bottom: parent.bottom
    //         color: "white"
    //         styleColor: "black"
    //         style: Text.Outline
    //         text: "Open street map tile token: "
    //         Rectangle {
    //             border.width: 1
    //             border.color: "black"
    //             anchors.fill: tokenTxtInput
    //             anchors.rightMargin: -30
    //             Text {
    //                 anchors.right: parent.right
    //                 anchors.top: parent.top
    //                 anchors.topMargin: 2
    //                 anchors.rightMargin: 8
    //                 color: "blue"
    //                 styleColor: "white"
    //                 style: Text.Outline
    //                 text: "OK"
    //                 Behavior on scale {
    //                     NumberAnimation {
    //                         easing.type: Easing.OutBack
    //                     }
    //                 }
    //                 MouseArea {
    //                     anchors.fill: parent
    //                     anchors.margins: -10
    //                     onPressedChanged: {
    //                         if (pressed)
    //                             parent.scale = 0.9
    //                         else
    //                             parent.scale = 1.0
    //                     }
    //                     onClicked: {
    //                         tokenInputArea.visible = false
    //                         osmManager.setToken(tokenTxtInput.text)
    //                         tokenWarning.demoToken = osmManager.isDemoToken()
    //                         tokenWarning.visible = true
    //                     }
    //                 }
    //             }
    //         }
    //         TextInput {
    //             id: tokenTxtInput
    //             clip: true
    //             anchors.left: parent.right
    //             anchors.bottom: parent.bottom
    //             anchors.bottomMargin: -3
    //             height: tokenTxtInput.contentHeight + 5
    //             width: 110
    //             leftPadding: 5
    //             rightPadding: 5
    //         }
    //     }

    //     Text {
    //         id: tokenWarning
    //         property bool demoToken: true
    //         anchors.left: parent.left
    //         anchors.bottom: parent.bottom
    //         color: "white"
    //         styleColor: "black"
    //         style: Text.Outline
    //         text: demoToken ? "You are using the OSM limited demo token " :
    //                           "You are using a token "
    //         Text {
    //             anchors.left: parent.right
    //             color: "blue"
    //             styleColor: "white"
    //             style: Text.Outline
    //             text: "click here to change"
    //             Behavior on scale {
    //                 NumberAnimation {
    //                     easing.type: Easing.OutBack
    //                 }
    //             }
    //             MouseArea {
    //                 anchors.fill: parent
    //                 onPressedChanged: {
    //                     if (pressed)
    //                         parent.scale = 0.9
    //                     else
    //                         parent.scale = 1.0
    //                 }
    //                 onClicked: {
    //                     tokenWarning.visible = false
    //                     tokenTxtInput.text = osmManager.token()
    //                     tokenInputArea.visible = true
    //                 }
    //             }
    //         }
    //     }
    // }
}
