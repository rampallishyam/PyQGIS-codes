# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets
from qgis.core import *
from qgis.core import QgsVectorFileWriter, QgsWkbTypes
import ntpath
import os
from scipy.spatial import ConvexHull
import numpy as np
from math import sqrt, pow

class Ui_dlgDbscan(QDialog):
    def __init__(self):
        QDialog.__init__(self)
        self.setObjectName("self")
        self.resize(400, 188)
        self.setAutoFillBackground(False)

        self.lnEditCsvPath = QtWidgets.QLineEdit(self)
        self.lnEditCsvPath.setGeometry(QtCore.QRect(10, 30, 381, 20))
        self.lnEditCsvPath.setText("C:/")
        self.lnEditCsvPath.setObjectName("lnEditCsvPath")

        self.btnOpen = QtWidgets.QPushButton("Open",self)
        self.btnOpen.setGeometry(QtCore.QRect(300, 60, 92, 25))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnOpen.sizePolicy().hasHeightForWidth())
        self.btnOpen.setSizePolicy(sizePolicy)
        self.btnOpen.setMinimumSize(QtCore.QSize(92, 0))
        self.btnOpen.setObjectName("btnOpen")

        self.label_3 = QtWidgets.QLabel("Import CSV data:",self)
        self.label_3.setGeometry(QtCore.QRect(10, 10, 141, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setObjectName("label_3")

        self.label_4 = QtWidgets.QLabel("Epsilon value:",self)
        self.label_4.setGeometry(QtCore.QRect(10, 90, 151, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setObjectName("label_4")

        self.lnEdEpsilon = QtWidgets.QLineEdit(self)
        self.lnEdEpsilon.setGeometry(QtCore.QRect(10, 110, 113, 20))
        self.lnEdEpsilon.setObjectName("lnEdEpsilon")

        self.label_5 = QtWidgets.QLabel("Min. samples:",self)
        self.label_5.setGeometry(QtCore.QRect(160, 90, 151, 16))
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setObjectName("label_5")

        self.lnEdMinSamples = QtWidgets.QLineEdit(self)
        self.lnEdMinSamples.setGeometry(QtCore.QRect(160, 110, 113, 20))
        self.lnEdMinSamples.setObjectName("lnEdMinSamples")

        self.btnRUN = QtWidgets.QPushButton("RUN",self)
        self.btnRUN.setGeometry(QtCore.QRect(120, 150, 71, 28))
        font = QtGui.QFont()
        font.setPointSize(-1)
        self.btnRUN.setFont(font)

        self.btnRUN.setObjectName("btnRUN")
        self.btnCancel = QtWidgets.QPushButton("CANCEL",self)
        self.btnCancel.setGeometry(QtCore.QRect(210, 150, 71, 28))
        font = QtGui.QFont()
        font.setPointSize(-1)
        self.btnCancel.setFont(font)

        self.btnCancel.setObjectName("btnCancel")

        self.btnRUN.clicked.connect(self.run)
        self.btnCancel.clicked.connect(self.cancel)
        self.btnOpen.clicked.connect(self.open)

        QtCore.QMetaObject.connectSlotsByName(self)

    def open(self):
        initPath = self.lnEditCsvPath.text()
        path, _ = QFileDialog.getOpenFileName(self,"Open GPS data",initPath,"*.csv")
                
        import os

        filePath = path.rstrip(os.sep)
        
        self.lnEditCsvPath.setText(str(filePath))

    def loadCsv(self):
        Input_Table = self.lnEditCsvPath.text()  # set the filepath for the input CSV
        lon_field = 'origin_lon'  # set the name for the field containing the longitude
        lat_field = 'origin_lat'  # set the name for the field containing the latitude
        lon_dest = 'dest_lon'
        lat_dest = 'dest_lat'

        crs = 4326  # WGS 84 (GPS data)
        strCRS = "EPSG" + str(4326)

        directory = os.path.dirname(Input_Table) + "/output/dbscan"

        import time
        # timestamp = time.time()
        ts = time.gmtime()
        ts = time.strftime("%Y%m%dT%H%M%S", ts)

        filename = ntpath.basename(self.lnEditCsvPath.text()).replace(".csv", "") + "_" + ts + "_" + strCRS + ".shp"

        if not os.path.exists(directory):
            os.makedirs(directory)

        outputLayerPath = directory + "//" + filename  # set the filepath for the output shapefile

        #print (outputLayerPath)

        spatRef = QgsCoordinateReferenceSystem(crs, QgsCoordinateReferenceSystem.EpsgCrsId)

        inp_tab = QgsVectorLayer(Input_Table, 'Input_Table', 'ogr')
        prov = inp_tab.dataProvider()
        fields = inp_tab.fields()
        outLayer = QgsVectorFileWriter(outputLayerPath, None, fields, QgsWkbTypes.Point, spatRef, "ESRI Shapefile")
        # outLayer = QgsVectorFileWriter(Output_Layer, None, fields, QGis.WKBPoint, spatRef)

        # reprojecting to metric datum system for k-means clustering purposes

        pt = QgsPointXY()
        pt_dest = QgsPointXY()

        outFeature = QgsFeature()
        outFeature_dest = QgsFeature()

        for feat in inp_tab.getFeatures():
            attrs = feat.attributes()
            pt.setX(float(feat[lon_field]))
            pt.setY(float(feat[lat_field]))
            outFeature.setAttributes(attrs)
            outFeature.setGeometry(QgsGeometry.fromPointXY(pt))
            outLayer.addFeature(outFeature)

            pt_dest.setX(float(feat[lon_dest]))
            pt_dest.setY(float(feat[lat_dest]))
            outFeature_dest.setAttributes(attrs)
            outFeature_dest.setGeometry(QgsGeometry.fromPointXY(pt_dest))
            outLayer.addFeature(outFeature_dest)

        del outLayer
        return outputLayerPath
        # import processing

    def changeLayerCrs(self, outputLayerPath):
        rootPath=os.path.dirname(outputLayerPath)

        crs = 4326  # WGS 84 (GPS data)
        strCRS = "EPSG" + str(4326)
        destCRS = 3857
        spatRefDest = QgsCoordinateReferenceSystem(destCRS, QgsCoordinateReferenceSystem.EpsgCrsId)

        # print (spatRefDest)

        strDestCRS = "EPSG" + str(destCRS)  # PSEUDO MERCATOR PROJECTION

        # outputLayerDestPath = outputLayerPath.replace(strCRS, strDestCRS)
        # print (outputLayerDestPath)
        filename=ntpath.basename(outputLayerPath)


        filename_dest = filename.replace(strCRS, strDestCRS)

        # self.reprojectLyr(Output_Layer,Output_Layer_dest, strDestCRS)

        # parameter = {'INPUT': Output_Layer,'TARGET_CRS': "EPSG:3857",'OUTPUT':Output_Layer_dest}

        # processing.run('qgis:reprojectlayer', parameter)
        layerName = filename.replace(".shp", "")
        outputLayerDestPath=rootPath+'//'+layerName+'_NEW_3857.shp'

        outputLoad = QgsVectorLayer(outputLayerPath, layerName, 'ogr')
        # if not layer.isValid():
        # raise IOError, "Failed to open the layer"

        # add layer to the registry
        QgsProject.instance().addMapLayer(outputLoad)
        canvas = QgsMapCanvas()
        # set extent to the extent of our layer
        canvas.setExtent(outputLoad.extent())
        canvas.setLayers([outputLoad])
        canvas.refresh()

        exp_crs = QgsCoordinateReferenceSystem(3857, QgsCoordinateReferenceSystem.EpsgCrsId)

        # canvas = qgis.utils.iface.mapCanvas()
        #allLayers = canvas.layers()
        layer = QgsProject.instance().mapLayersByName(layerName)[0]

        qgis.core.QgsVectorFileWriter.writeAsVectorFormat(layer, outputLayerDestPath, 'utf-8', exp_crs, "ESRI Shapefile")

        layerName_3857 = layerName + '_NEW_3857.shp'

        outputLoad3857 = QgsVectorLayer(outputLayerDestPath, layerName_3857.replace('.shp',''), 'ogr')

        id = layer.id()

        QgsProject.instance().removeMapLayer(id)

        QgsProject.instance().addMapLayer(outputLoad3857)

        # set extent to the extent of our layer
        canvas.setExtent(outputLoad3857.extent())
        canvas.setLayers([outputLoad3857])

        selectedcrs = "EPSG:3857"
        target_crs = QgsCoordinateReferenceSystem()
        target_crs.createFromUserInput(selectedcrs)
        canvas.setDestinationCrs(target_crs)

        canvas.refresh()

        return outputLoad3857

    def getCoords(self, layer):
        import numpy as np
        print ("getting cords from layer: " + layer)
        outputFieldName = 'cluster_no'
        layerProc = QgsProject.instance().mapLayersByName(layer)[0]

        coordsList = []

        from PyQt5.QtCore import QVariant
        layer_provider = layerProc.dataProvider()
        layer_provider.addAttributes([QgsField(outputFieldName, QVariant.Int)])

        layerProc.updateFields()

        for feature in layerProc.getFeatures():
            geom = feature.geometry()
            x = geom.centroid().asPoint().x()
            y = geom.centroid().asPoint().y()
            coordsList.append([x, y])

        data = np.array(coordsList)
        return data

    def dbScan(self, layer, data, eps, minSamples):
        print ("Dbscan clustering. Processing layer: " + layer + ". Eps: k1 = " + str(eps)+ ". Param2: k2 = " + str(minSamples)+"\n")
        #outputFieldName = 'cluster_no'

        layerProc = QgsProject.instance().mapLayersByName(layer)[0]

        from sklearn.cluster import DBSCAN
        clustering = DBSCAN(eps=eps, min_samples=minSamples).fit(data)

        labels=clustering.labels_
        noCluster=[]
        for label in labels:
            if not int(label)+1 in noCluster:
                noCluster.append(int(label)+1)
            else:
                pass

        layerProc.startEditing()

        i = 0

        print ("Wait until DONE.........")
        for f in layerProc.getFeatures():
            f.setAttribute(len(f.attributes())-1,int(labels[i])+1)
            layerProc.updateFeature(f)

            i += 1
        print ("DONE")

        #layerProc.updateFields()
        layerProc.commitChanges()

        return noCluster

    def createConvHullLayer(self,path):
        rootPath = os.path.dirname(path)
        filename = ntpath.basename(path)
        filename=filename.replace('.shp','_conv.shp')
        convLayerPath = os.path.join(rootPath, filename)
        crs1=3857

        vl = QgsVectorLayer("Polygon?crs=epsg:" + str(crs1), "convexHull", "memory")
        pr = vl.dataProvider()

        # Enter editing mode
        vl.startEditing()

        pr.addAttributes([QgsField('No.', QVariant.Int), QgsField('Area[m]', QVariant.Double, 'double', 12, 2),
                          QgsField('pointsNum', QVariant.Int), QgsField('Density', QVariant.Double, 'double', 12, 2),
                          QgsField('AvgDist', QVariant.Double, 'double', 12, 2)])

        # Commit changes
        vl.commitChanges()

        crs_writer = QgsCoordinateReferenceSystem("epsg:" + str(crs1))

        # zapis
        _writer = QgsVectorFileWriter.writeAsVectorFormat(vl, convLayerPath, "utf-8", crs_writer, "ESRI Shapefile")


        return convLayerPath

    def calculateAverageDistance(self,hullTmp, layer):
        cx = np.mean(hullTmp.points[hullTmp.vertices, 0])
        cy = np.mean(hullTmp.points[hullTmp.vertices, 1])
        cPoint = QgsPointXY(cx, cy)
        total=0.0
        n=0
        for feat in layer.getSelectedFeatures():
            geom = feat.geometry()
            px = geom.centroid().asPoint().x()
            py = geom.centroid().asPoint().y()
            pPoint=QgsPointXY(px, py)


            # Create a measure object
            #distance = QgsDistanceArea()
            #distance.setEllipsoidalMode(True)
            #distance.setEllipsoid('WGS84')
            # Measure the distance
            length=sqrt(pow((cx-px),2)+pow((cy-py),2))#distance.measureLine(cPoint, pPoint)
            total+=length
            n+=1

        avgDist=total/n
        return avgDist


    def convHulls(self, layer, noClusters):


        outputFieldName = 'cluster_no'
        layerProc = QgsProject.instance().mapLayersByName(layer)[0]
        print ("CONV HULLS", layerProc.name(), noClusters, outputFieldName)

        convLayerPath = self.createConvHullLayer(layerProc.source())

        vectorLyr = QgsVectorLayer(convLayerPath, 'convexhull_'+layerProc.name(),"ogr")

        QgsProject.instance().addMapLayer(vectorLyr)

        vpr = vectorLyr.dataProvider()

        vectorLyr.startEditing()

        for i in noClusters:

            if i==0:
                print ("Cluster number 0 is noise.")
            else:
                try:
                    # exp = "'" + '"cluster_no\"=' + str(i) + "'"
                    exp = '"cluster_no\"=' + str(i)
                    print (exp)

                    layerProc.selectByExpression(exp, QgsVectorLayer.SetSelection)

                    coordListTmp = []
                    #
                    for feat in layerProc.getSelectedFeatures():
                        geom = feat.geometry()
                        x = geom.centroid().asPoint().x()
                        y = geom.centroid().asPoint().y()
                        # print (feat.id(),x,y)

                        coordListTmp.append([x, y])

                    points2 = coordListTmp  # np.array(coordListTmp)
                    pointsNumber=len(coordListTmp)
                    listAr = np.array(points2)

                    hullTmp = ConvexHull(listAr)
                    avgDist=self.calculateAverageDistance(hullTmp, layerProc)

                    hullPointsTmp = hullTmp.points[hullTmp.vertices]


                    wktGeometry='POLYGON (('
                    for point in hullPointsTmp:
                        wktPoint=str(point[0])+' '+str(point[1])
                        wktGeometry+=wktPoint+','

                    wktGeometry=wktGeometry[:-1]+'))'
                    poly = QgsGeometry.fromWkt(wktGeometry)

                    f = QgsFeature()
                    f.setGeometry(poly)
                    area=f.geometry().area()
                    density=area/pointsNumber
                    f.setAttributes([i,area,pointsNumber,density,avgDist])
                    vpr.addFeatures([f])


                    i += 1
                except:
                    print ("Change parameters value. ConvexHull can't be created for cluster number %s." %(str(i)))

            vectorLyr.commitChanges()
            iface.mapCanvas().refresh()

    def saveReport(self,layerPath, epsilon, minSamples):
        reportPath=layerPath.replace('.shp','_report.txt')
        name= (ntpath.basename(layerPath)).replace('.shp','')
        content='Name: '+name+'\nMethod: Dbscan\nEpsilon value: '+str(epsilon)+'\nMin. samples value: '+str(minSamples)
        with open(reportPath,'w',encoding='utf-8') as file:
            file.write(content)

    def run(self):
        try:
            epsilon=float(self.lnEdEpsilon.text())
            minSamples=float(self.lnEdMinSamples.text())
        except:
            print ("Epsilon value and min. sample value must be a number.")
            return


        outputLayerPath=self.loadCsv()
        print ("CSV data has been imported")
        mainLayer=self.changeLayerCrs(outputLayerPath)
        corArray = self.getCoords(mainLayer.name())
        noClusters=self.dbScan(mainLayer.name(), corArray, epsilon,minSamples)
        self.convHulls(mainLayer.name(),noClusters)
        self.saveReport(mainLayer.source(),epsilon, minSamples)
        dialog.hide()

    def cancel(self):
        print ("cancel")
        dialog.hide()


dialog = Ui_dlgDbscan()

if dialog.exec_() == QDialog.Accepted:

    print ("ACC")
