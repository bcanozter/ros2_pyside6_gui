# Copyright (C) 2024 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

from PySide6.QtQuick3D import QQuick3DTextureData
from PySide6.QtQml import QmlElement
from PySide6.QtGui import QImage, QVector3D, QDesktopServices
from PySide6.QtCore import QByteArray, QObject, Property, Slot, Signal
import math

from .request import OSMTileData, OSMRequest

# To be used on the @QmlElement decorator
# (QML_IMPORT_MINOR_VERSION is optional)
QML_IMPORT_NAME = "OSMBuildings"
QML_IMPORT_MAJOR_VERSION = 1


@QmlElement
class OSMManager(QObject):

    mapsDataReady = Signal(QByteArray, int, int, int)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.m_request = OSMRequest(self)
        self.home_latitude = 44.64750
        self.home_longitude = -63.58028
        result = self.deg2num(
            self.home_latitude, self.home_longitude, zoom=15
        )
        self.m_startBuildingTileX = result[0]
        self.m_startBuildingTileY = result[1]
        self.m_tileSizeX = 37
        self.m_tileSizeY = 37
        self.m_request.mapsDataReady.connect(self._slotMapsDataReady)

    # https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    @Slot(float, float, int, result=list)
    def deg2num(self, lat_deg: float, lon_deg: float, zoom: int = 15):
        lat_rad = math.radians(lat_deg)
        n = 1 << zoom
        xtile = int((lon_deg + 180.0) / 360.0 * n)
        ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
        return [xtile, ytile]

    @Slot(float, float, int, result=list)
    def deg2num_f(self, lat_deg: float, lon_deg: float, zoom: int = 15):
        lat_rad = math.radians(lat_deg)
        n = 1 << zoom
        xtile = (lon_deg + 180.0) / 360.0 * n
        ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
        return [xtile, ytile]

    def num2deg(self, xtile, ytile, zoom):
        n = 1 << zoom
        lon_deg = xtile / n * 360.0 - 180.0
        lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
        lat_deg = math.degrees(lat_rad)
        return lat_deg, lon_deg

    def tileSizeX(self):
        return self.m_tileSizeX

    def tileSizeY(self):
        return self.m_tileSizeY

    def startBuildingTileX(self):
        return self.m_startBuildingTileX

    def startBuildingTileY(self):
        return self.m_startBuildingTileY

    @Slot(QByteArray, int, int, int)
    def _slotMapsDataReady(self, mapData, tileX, tileY, zoomLevel):
        self.mapsDataReady.emit(
            mapData, tileX - self.m_startBuildingTileX, tileY - self.m_startBuildingTileY, zoomLevel
        )

    @Slot(QVector3D, QVector3D, float, float, float, float, float, float)
    def setCameraProperties(
        self, position, right, cameraZoom, minimumZoom, maximumZoom, cameraTilt, minimumTilt, maximumTilt
    ):

        tiltFactor = (cameraTilt - minimumTilt) / max(maximumTilt - minimumTilt, 1.0)
        zoomFactor = (cameraZoom - minimumZoom) / max(maximumZoom - minimumZoom, 1.0)

        # Forward vector align to the XY plane
        forwardVector = QVector3D.crossProduct(right, QVector3D(0.0, 0.0, -1.0)).normalized()
        projectionOfForwardOnXY = position + forwardVector * tiltFactor * zoomFactor * 50.0

        queue = []
        for forwardIndex in range(-20, 21):
            for sidewardIndex in range(-20, 21):
                vx = float(self.m_tileSizeX * sidewardIndex)
                vy = float(self.m_tileSizeY * forwardIndex)
                transferredPosition = projectionOfForwardOnXY + QVector3D(vx, vy, 0)
                tile_x = self.m_startBuildingTileX + int(transferredPosition.x() / self.m_tileSizeX)
                tile_y = self.m_startBuildingTileY - int(transferredPosition.y() / self.m_tileSizeY)
                self.addBuildingRequestToQueue(queue, tile_x, tile_y)

        projectedTileX = self.m_startBuildingTileX + int(projectionOfForwardOnXY.x() / self.m_tileSizeX)
        projectedTileY = self.m_startBuildingTileY - int(projectionOfForwardOnXY.y() / self.m_tileSizeY)

        def tile_sort_key(tile_data):
            return tile_data.distanceTo(projectedTileX, projectedTileY)

        queue.sort(key=tile_sort_key)

        self.m_request.getMapsData(queue.copy())

    def addBuildingRequestToQueue(self, queue, tileX, tileY, zoomLevel=15):
        queue.append(OSMTileData(tileX, tileY, zoomLevel))

    @Slot(result=bool)
    def isDemoToken(self):
        return self.m_request.isDemoToken()

    @Slot(str)
    def setToken(self, token):
        self.m_request.setToken(token)

    @Slot(result=str)
    def token(self):
        return self.m_request.token()

    @Slot(str)
    def requestOpenUrl(self, url: str):
        try:
            QDesktopServices.openUrl(url)
        except Exception as e:
            self.logger.error(f"Failed to open URL '{url}': {e}")

    tileSizeX = Property(int, tileSizeX, constant=True)
    tileSizeY = Property(int, tileSizeY, constant=True)
    startBuildingTileX = Property(int, startBuildingTileX, constant=True)
    startBuildingTileY = Property(int, startBuildingTileY, constant=True)


@QmlElement
class CustomTextureData(QQuick3DTextureData):

    @Slot(QByteArray)
    def setImageData(self, data):
        image = QImage.fromData(data).convertToFormat(QImage.Format.Format_RGBA8888)
        self.setTextureData(QByteArray(bytearray(image.constBits())))
        self.setSize(image.size())
        self.setHasTransparency(False)
        self.setFormat(QQuick3DTextureData.Format.RGBA8)
