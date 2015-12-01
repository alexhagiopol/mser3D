//
// Created by Alex Hagiopol on 11/30/15.
//

#ifndef MSER_3D_MEASUREMENTFUNCTION_H
#define MSER_3D_MEASUREMENTFUNCTION_H

#include "geometryFunctions.h"
#include "mserClasses.h"
#include "boost/optional.hpp"
#include <gtsam/geometry/OrientedPlane3.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <math.h>
#include <string>
#include <random>
#include <vector>
#include <iostream>

using namespace gtsam;
using namespace std;

std::vector<Point3> convertObjectToObjectPoint3s(mserObject& object, OptionalJacobian<6,8> Dobject = boost::none){
    //Point3 center(object.first.x(),object.first.y(),object.first.z());
    Point3 majAxisTip(object.second.x(),0,0);
    Point3 minAxisTip(0,object.second.y(),0);
    std::vector<Point3> pointsInObjectFrame;
    //pointsInObjectFrame.push_back(center);
    pointsInObjectFrame.push_back(majAxisTip);
    pointsInObjectFrame.push_back(minAxisTip);
    if (Dobject) *Dobject << 0,0,0,0,0,0,1,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,0,
                             0,0,0,0,0,0,0,1,
                             0,0,0,0,0,0,0,0;
    return pointsInObjectFrame;
}

std::vector<Point3> convertObjectPoint3sToWorldPoint3s(mserObject& object, std::vector<Point3> pointsInObjectFrame, OptionalJacobian<9,8> Dobject = boost::none, OptionalJacobian<9,6> Dpoints = boost::none) {
    Matrix36 centerDpose, majAxisDpose, minAxisDpose; //Matrices to store results from optional jacobians
    Matrix33 majAxisDpoint, minAxisDpoint; //Matrices to store results from optional jacobians
    Point3 objectCenter = object.first.translation(centerDpose);
    Point3 majAxisInObjectFrame = pointsInObjectFrame[0];
    Point3 minAxisInObjectFrame = pointsInObjectFrame[1];

    Point3 majAxisInWorldFrame = object.first.transform_from(majAxisInObjectFrame,majAxisDpose,majAxisDpoint);
    Point3 minAxisInWorldFrame = object.first.transform_from(minAxisInObjectFrame,minAxisDpose,minAxisDpoint);
    std::vector<Point3> pointRepresentation;
    pointRepresentation.push_back(objectCenter);
    pointRepresentation.push_back(majAxisInWorldFrame);
    pointRepresentation.push_back(minAxisInWorldFrame);
    //cout << "centerDpose\n" << centerDpose << endl;
    //cout << "majAxisDpose\n" << majAxisDpose << endl;
    //cout << "majAxisDpoint\n" << majAxisDpoint << endl;
    //cout << "minAxisDpose\n" << minAxisDpose << endl;
    //cout << "minAxisDpoint\n" << minAxisDpoint << endl;
    if (Dobject) *Dobject <<    centerDpose(0,0),   centerDpose(0,1),   centerDpose(0,2),   centerDpose(0,3),   centerDpose(0,4),   centerDpose(0,4), 0, 0,
                                centerDpose(1,0),   centerDpose(1,1),   centerDpose(1,2),   centerDpose(1,3),   centerDpose(1,4),   centerDpose(1,4), 0, 0,
                                centerDpose(2,0),   centerDpose(2,1),   centerDpose(2,2),   centerDpose(2,3),   centerDpose(2,4),   centerDpose(2,4), 0, 0,
                                majAxisDpose(0,0),  majAxisDpose(0,1),  majAxisDpose(0,2),  majAxisDpose(0,3),  majAxisDpose(0,4),  majAxisDpose(0,4), 0, 0,
                                majAxisDpose(1,0),  majAxisDpose(1,1),  majAxisDpose(1,2),  majAxisDpose(1,3),  majAxisDpose(1,4),  majAxisDpose(1,4), 0, 0,
                                majAxisDpose(2,0),  majAxisDpose(2,1),  majAxisDpose(2,2),  majAxisDpose(2,3),  majAxisDpose(2,4),  majAxisDpose(2,4), 0, 0,
                                minAxisDpose(0,0),  minAxisDpose(0,1),  minAxisDpose(0,2),  minAxisDpose(0,3),  minAxisDpose(0,4),  minAxisDpose(0,4), 0, 0,
                                minAxisDpose(1,0),  minAxisDpose(1,1),  minAxisDpose(1,2),  minAxisDpose(1,3),  minAxisDpose(1,4),  minAxisDpose(1,4), 0, 0,
                                minAxisDpose(2,0),  minAxisDpose(2,1),  minAxisDpose(2,2),  minAxisDpose(2,3),  minAxisDpose(2,4),  minAxisDpose(2,4), 0, 0;
    if (Dpoints) *Dpoints <<                0,                  0,                  0,                  0,                  0,                  0,
                                            0,                  0,                  0,                  0,                  0,                  0,
                                            0,                  0,                  0,                  0,                  0,                  0,
                           majAxisDpoint(0,0), majAxisDpoint(0,1), majAxisDpoint(0,2),                  0,                  0,                  0,
                           majAxisDpoint(1,0), majAxisDpoint(1,1), majAxisDpoint(1,2),                  0,                  0,                  0,
                           majAxisDpoint(2,0), majAxisDpoint(2,1), majAxisDpoint(2,2),                  0,                  0,                  0,
                                            0,                  0,                  0, minAxisDpoint(0,0), minAxisDpoint(0,1), minAxisDpoint(0,2),
                                            0,                  0,                  0, minAxisDpoint(1,0), minAxisDpoint(1,1), minAxisDpoint(1,2),
                                            0,                  0,                  0, minAxisDpoint(2,0), minAxisDpoint(2,1), minAxisDpoint(2,2);

    return pointRepresentation;
}

std::vector<Point2> convertWorldPoint3sToCameraPoint2s(SimpleCamera& camera, std::vector<Point3>& points, OptionalJacobian<6,6> Dpose = boost::none, OptionalJacobian<6,5> Dcal = boost::none, OptionalJacobian<6,9> Dpoints = boost::none){
    Point3 objectCenter = points[0];
    Point3 majorAxisTip = points[1];
    Point3 minorAxisTip = points[2];
    Matrix26 centerDpose, majorDpose, minorDpose;
    Matrix23 centerDpoint, majorDpoint, minorDpoint;
    Matrix25 centerDcal, majorDcal, minorDcal;
    Point2 projectedObjectCenter = camera.project(objectCenter, centerDpose, centerDpoint, centerDcal);
    Point2 projectedMajorAxisTip = camera.project(majorAxisTip, majorDpose, majorDpoint, majorDcal);
    Point2 projectedMinorAxisTip = camera.project(minorAxisTip, minorDpose, minorDpoint, minorDcal);
    std::vector<Point2> cameraPoint2s;
    cameraPoint2s.push_back(projectedObjectCenter);
    cameraPoint2s.push_back(projectedMajorAxisTip);
    cameraPoint2s.push_back(projectedMinorAxisTip);

    if (Dpose) {
        Eigen::MatrixXd Dpose_(6,6);
        Dpose_ << centerDpose,
                majorDpose,
                minorDpose;
        *Dpose << Dpose_;
    }
    if (Dcal){
        Eigen::MatrixXd Dcal_(6,5);
        Dcal_ << centerDcal,
                 majorDcal,
                 minorDcal;
        *Dcal <<  Dcal_;
    }
    if (Dpoints){
        Eigen::MatrixXd Dpoints_(6,9);
        Eigen::MatrixXd zeros23 = Eigen::MatrixXd::Zero(2,3);
        Eigen::MatrixXd top(2,9);
        Eigen::MatrixXd middle(2,9);
        Eigen::MatrixXd bottom(2,9);
        top << centerDpoint,zeros23, zeros23;
        middle << zeros23,majorDpoint,zeros23;
        bottom << zeros23, zeros23, minorDpoint;
        Dpoints_ << top,
                    middle,
                    bottom;
        *Dpoints << Dpoints_;
    }
    return cameraPoint2s;
}

mserMeasurement convertCameraPoint2sToMeasurement(std::vector<Point2> cameraPoint2s, OptionalJacobian<5,6> Dpoints = boost::none){
    Point2 measurementCenter = cameraPoint2s[0]; //Jacobian for this is 2x2 identity matrix
    Matrix12 thetaDcenter, thetaDmajor, A1Dmajor, A1Dcenter, A2Dminor, A2Dcenter;
    double theta = ellipse2DOrientation(measurementCenter, cameraPoint2s[1], thetaDcenter, thetaDmajor);
    double A1 = cameraPoint2s[1].distance(measurementCenter, A1Dmajor, A1Dcenter);
    double A2 = cameraPoint2s[2].distance(measurementCenter, A2Dminor, A2Dcenter);
    Pose2 ellipsePose(measurementCenter.x(),measurementCenter.y(),theta);
    Point2 axisLengths(A1,A2);
    mserMeasurement measurement(ellipsePose,axisLengths);
    if (Dpoints){
        *Dpoints <<                 1,                 0,                0,                0,             0,             0,
                                    0,                 1,                0,                0,             0,             0,
                    thetaDcenter(0,0), thetaDcenter(0,1), thetaDmajor(0,0), thetaDmajor(0,1),             0,             0,
                       A1Dcenter(0,0),    A1Dcenter(0,1),    A1Dmajor(0,0),    A1Dmajor(0,1),             0,             0,
                       A2Dcenter(0,0),    A2Dcenter(0,1),                0,                0, A2Dminor(0,0), A2Dminor(0,1);
    }
    return measurement;
}

#endif //MSER_3D_MEASUREMENTFUNCTION_H
