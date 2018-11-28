#!/usr/bin/env python3
# -*- coding=utf-8 -*-
from lxml import etree
# from pykml.parser import Schema
from pykml.factory import KML_ElementMaker as KML
from pykml import parser
# from pykml.factory import GX_ElementMaker as GX
import os


def addTitle(fileName):
    with open(fileName, "r+") as f:
        old = f.read()
        f.seek(0)
        title = 'num lon lat alt attr\n'
        f.write(title)
        f.write(old)

    print('Added title')


def readTXT(fileName):
    points = []
    with open(fileName, 'r') as f:
        for line in f.readlines():
            line = line.strip().split(' ')
            # line[0] is task point's number, multiple 10 for adding point
            # added point's name should plus 1
            point = [int(line[0]) * 10, float(line[1]),
                     float(line[2]), float(line[3]), int(line[4])]
            points.append(point)
            print(point)

    return points


def creatPlacemark(point, colorTag):
    defaultView = {'lon': 116.1120793889676,
                   'lat': 40.15788423720766,
                   'alt': 0,
                   'range': 3108.303488980141,
                   'tilt': 29.76964560740583,
                   'heading': 0}
    colorTag = '#' + colorTag
    name = str(point[0])
    coor = '%.8f,%.8f,%.8f' % (point[1], point[2], point[3])
    placemark = KML.Placemark(
        KML.name(name),
        KML.Lookat(
            KML.longitude(defaultView['lon']),
            KML.latitude(defaultView['lat']),
            KML.altitude(defaultView['alt']),
            KML.range(defaultView['range']),
            KML.tilt(defaultView['tilt']),
            KML.heading(defaultView['heading'])
        ),
        KML.styleUrl(colorTag),
        KML.Point(
            KML.coordinates(coor)
        )
    )

    return placemark


def creatStyle(attr, urlDic):
    if attr == 0:
        colorTag = 'ylw'
        url = urlDic[colorTag]
    elif attr == 1:
        colorTag = 'blue'
        url = urlDic[colorTag]
    elif attr == 2:
        colorTag = '2'
        url = urlDic[colorTag]
    elif attr == 3:
        colorTag = '3'
        url = urlDic[colorTag]
    elif attr == 4:
        colorTag = '4'
        url = urlDic[colorTag]
    elif attr == 5:
        colorTag = '5'
        url = urlDic[colorTag]
    else:
        colorTag = 'else'
        url = urlDic[colorTag]

    style = KML.Style(
        KML.IconStyle(
            KML.scale(1.2),
            KML.Icon(
                KML.href(url),
            ),
        ),
        id=colorTag
    )

    return style, colorTag


def writeKML(points, urlDic, path):
    kmlDoc = KML.kml()
    doc = KML.Document()

    for point in points:
        style, colorTag = creatStyle(point[-1], urlDic)
        placemark = creatPlacemark(point, colorTag)
        doc.append(style)
        doc.append(placemark)

    kmlDoc.append(doc)

    print(etree.tostring(kmlDoc, pretty_print=True))

    kmlFileName = path + '/KYXZ.kml'
    txtFileName = path + '/KYXZ.txt'  # for inspect

    with open(kmlFileName, 'wb') as f:
        f.write(etree.tostring(kmlDoc, pretty_print=True))

    with open(txtFileName, 'wb') as f:
        f.write(etree.tostring(kmlDoc, pretty_print=True))  # for inspect

    os.system('kill $(ps -A | grep earth | awk \'{print $1}\')')
    command = 'google-earth-pro ' + kmlFileName
    os.system(command)


def txt2kml(fileName, urlDic):
    points = readTXT(fileName)
    path = os.path.dirname(fileName)
    writeKML(points, urlDic, path)


def parseXML(fileName):
    path = os.path.dirname(fileName)
    kmlFileName = path + '/1.kml'

    placemarksDic = []  # each element is a dictionaty

    with open(kmlFileName, 'r') as f:
        kml = parser.parse(f).getroot()
        placemarks = kml.findall('.//{http://www.opengis.net/kml/2.2}Placemark')
        print('File <%s> has %d placemarks.' % (kmlFileName, len(placemarks)))
        for each in placemarks:
            placemark = {'name': each.name,
                         'styleUrl': each.styleUrl,
                         'coor': each.Point.coordinates}

            placemarksDic.append(placemark)

    return placemarksDic


def kml2txt(fileName):
    placemarksDic = parseXML(fileName)

    placemarksDic = sorted(placemarksDic, key=lambda i: i['name'])
    for each in placemarksDic:
        print(each)

    count = 1
    points = []
    for each in placemarksDic:
        point = getPointWithType(each, count)
        count += 1
        points.append(point)
        print(point)

    with open(fileName, 'w') as f:
        for each in points:
            line = '%d %.8f %.8f %.8f %d\n' % (int(each[0]), float(each[1]),
                                               float(each[2]), float(each[3]), int(each[4]))
            f.write(line)


def getPointWithType(placemark, count):
    point = []
    point.append(count)
    pointStr = str(placemark['coor'])
    point.append(float(pointStr.split(',')[0]))
    point.append(float(pointStr.split(',')[1]))
    point.append(float(pointStr.split(',')[2]))

    key = str(placemark['styleUrl'])[1]

    if 'y' == key:
        point.append(0)
    elif 'b' == key:
        point.append(1)
    elif '2' == key:
        point.append(2)
    elif '3' == key:
        point.append(3)
    elif '4' == key:
        point.append(4)
    elif '5' == key:
        point.append(5)
    else:
        print(key)
        print('please checkout.')

    return point


if __name__ == '__main__':
    path = os.path.expanduser('~')
    fileName = path + '/taskfile/KYXZ2018A.txt'
    # key == type, ylw == start, blue == end
    urlDic = {'ylw': 'http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png',
              'blue': 'http://maps.google.com/mapfiles/kml/pushpin/blue-pushpin.png',
              '2': 'http://maps.google.com/mapfiles/kml/paddle/2.png',
              '3': 'http://maps.google.com/mapfiles/kml/paddle/3.png',
              '4': 'http://maps.google.com/mapfiles/kml/paddle/4.png',
              '5': 'http://maps.google.com/mapfiles/kml/paddle/5.png',
              'else': 'http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png'}

    txt2kml(fileName, urlDic)
    kml2txt(fileName)
