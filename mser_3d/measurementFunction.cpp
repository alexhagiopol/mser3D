//
// Created by alex on 2/16/16.
//

#include "measurementFunction.h"

PointsPose convertObjectToObjectPointsPose(const MserObject& object, OptionalJacobian<12,8> Dobject){
    PointsPose myPointsPose(object.first, Point3(object.second.x(),0,0), Point3(0,object.second.y(),0));
    if (Dobject) *Dobject <<1,0,0,0,0,0,0,0,
                            0,1,0,0,0,0,0,0,
                            0,0,1,0,0,0,0,0,
                            0,0,0,1,0,0,0,0,
                            0,0,0,0,1,0,0,0,
                            0,0,0,0,0,1,0,0,
                            0,0,0,0,0,0,1,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,0,
                            0,0,0,0,0,0,0,1,
                            0,0,0,0,0,0,0,0;
    return myPointsPose;
}

std::vector<Point3> convertObjectPointsPoseToWorldPoint3s(const PointsPose& objectPointsPose, OptionalJacobian<9,12> Dpointspose){
    Matrix36 centerDpose, majAxisDpose, minAxisDpose; //Matrices to store results from optional jacobians
    Matrix33 majAxisDpoint, minAxisDpoint; //Matrices to store results from optional jacobians
    const Pose3 objectPose = gtsam::traits<PointsPose>::objectPose(objectPointsPose);
    const Point3 objectCenter = objectPose.translation(centerDpose);
    const Point3 majAxisInObjectFrame = gtsam::traits<PointsPose>::majAxisTip(objectPointsPose);
    const Point3 minAxisInObjectFrame = gtsam::traits<PointsPose>::minAxisTip(objectPointsPose);
    const Point3 majAxisInWorldFrame = objectPose.transform_from(majAxisInObjectFrame,majAxisDpose,majAxisDpoint);
    const Point3 minAxisInWorldFrame = objectPose.transform_from(minAxisInObjectFrame,minAxisDpose,minAxisDpoint);
    const std::vector<Point3> pointRepresentation = {objectCenter, majAxisInWorldFrame, minAxisInWorldFrame};
    //pointRepresentation.push_back(objectCenter);
    //pointRepresentation.push_back(majAxisInWorldFrame);
    //pointRepresentation.push_back(minAxisInWorldFrame);
    if (Dpointspose){
        const Eigen::MatrixXd zeros33 = Eigen::MatrixXd::Zero(3,3);
        const Eigen::MatrixXd topLeft = Eigen::MatrixXd::Zero(3,6);
        Eigen::MatrixXd middleLeft(3,6);
        Eigen::MatrixXd bottomLeft(3,6);
        Eigen::MatrixXd left(9,6);
        Eigen::MatrixXd right(9,6);
        Eigen::MatrixXd Dpointspose_(9,12);
        middleLeft << majAxisDpoint, zeros33;
        bottomLeft << zeros33, minAxisDpoint;
        left << topLeft,
                middleLeft,
                bottomLeft;
        right << centerDpose,
                majAxisDpose,
                minAxisDpose;
        Dpointspose_ << left, right;
        *Dpointspose << Dpointspose_;
    }
    return pointRepresentation;
}

std::vector<Point2> convertWorldPoint3sToCameraPoint2s(const SimpleCamera& camera, std::vector<Point3>& points, OptionalJacobian<6,6> Dpose, OptionalJacobian<6,5> Dcal, OptionalJacobian<6,9> Dpoints){
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

MserMeasurement convertCameraPoint2sToMeasurement(std::vector<Point2>& cameraPoint2s, OptionalJacobian<5,6> Dpoints){
    Point2 measurementCenter = cameraPoint2s[0]; //Jacobian for this is 2x2 identity matrix
    Matrix12 thetaDcenter, thetaDmajor, A1Dmajor, A1Dcenter, A2Dminor, A2Dcenter;
    double theta = ellipse2DOrientation(measurementCenter, cameraPoint2s[1], thetaDcenter, thetaDmajor);
    double A1 = cameraPoint2s[1].distance(measurementCenter, A1Dmajor, A1Dcenter);
    double A2 = cameraPoint2s[2].distance(measurementCenter, A2Dminor, A2Dcenter);
    Pose2 ellipsePose(measurementCenter.x(),measurementCenter.y(),theta);
    Point2 axisLengths(A1,A2);
    MserMeasurement measurement(ellipsePose,axisLengths);
    if (Dpoints){
        *Dpoints <<                 1,                 0,                0,                0,             0,             0,
                0,                 1,                0,                0,             0,             0,
                thetaDcenter(0,0), thetaDcenter(0,1), thetaDmajor(0,0), thetaDmajor(0,1),             0,             0,
                A1Dcenter(0,0),    A1Dcenter(0,1),    A1Dmajor(0,0),    A1Dmajor(0,1),             0,             0,
                A2Dcenter(0,0),    A2Dcenter(0,1),                0,                0, A2Dminor(0,0), A2Dminor(0,1);
    }
    return measurement;
}

MserMeasurement measurementFunction(const SimpleCamera& camera, const MserObject& object, OptionalJacobian<5,11> Dcamera, OptionalJacobian<5,8> Dobject){
    //Part 1: object -> 2X Point3 in object Frame + 1X Pose3
    Eigen::MatrixXd objectpointsposeDobject12_8(12,8);
    PointsPose objectPointsPose = convertObjectToObjectPointsPose(object, objectpointsposeDobject12_8);

    //Part 2: 2X Point3s in object frame + Pose 3 -> 3X Point3s in world frame
    Eigen::MatrixXd worldpointsDobjectpointspose9_12(9,12);
    std::vector<Point3> worldPoints = convertObjectPointsPoseToWorldPoint3s(objectPointsPose, worldpointsDobjectpointspose9_12);

    //Part 3: 3X Point3s in world frame -> 3X Point2s in camera frame
    Matrix66 campointsDpose66;
    Matrix65 campointsDcal65;
    Matrix69 campointsDworldpoints69;
    std::vector<Point2> cameraPoints = convertWorldPoint3sToCameraPoint2s(camera, worldPoints, campointsDpose66, campointsDcal65, campointsDworldpoints69);

    //Part 4: 3X Point2s in camera frame -> 1X measurement
    Matrix56 msmtDcampoints56;
    MserMeasurement measurement = convertCameraPoint2sToMeasurement(cameraPoints, msmtDcampoints56);

    //Provide Jacobians
    //if (Dpose) *Dpose << msmtDcampoints56*campointsDpose66;
    //if (Dcal) *Dcal << msmtDcampoints56*campointsDcal65;
    if (Dcamera) {
        Eigen::MatrixXd Dcamera_(5,11);
        Dcamera_ << msmtDcampoints56*campointsDpose66, msmtDcampoints56*campointsDcal65;
        *Dcamera << Dcamera_;
    }
    if (Dobject) *Dobject << msmtDcampoints56*campointsDworldpoints69*worldpointsDobjectpointspose9_12*objectpointsposeDobject12_8;

    /*
    //Print out Jacobians for debugging
    cout << "objectpointsposeDobject12_8\n" << objectpointsposeDobject12_8 << endl;
    cout << "worldpointsDobjectpointspose9_12\n" << worldpointsDobjectpointspose9_12 << endl;
    cout << "campointsDpose66\n" << campointsDpose66 << endl;
    cout << "campointsDcal65\n" << campointsDcal65 << endl;
    cout << "campointsDworldpoints69\n" << campointsDworldpoints69 << endl;
    cout << "msmtDcampoints56\n" << msmtDcampoints56 << endl;
    cout << "Dpose\n" << *Dpose << endl;
    cout << "Dcal\n" << *Dcal << endl;
    cout << "Dobject\n" << *Dobject << endl;
    */
    return measurement;
}

std::vector<MserMeasurement> createIdealMeasurements(std::vector<SimpleCamera>& cameras, MserObject& object){
    std::vector<MserMeasurement> measurements;
    for (size_t i = 0; i < cameras.size(); i++){
        MserMeasurement measurement = measurementFunction(cameras[i], object);
        measurements.push_back(measurement);
    }
    return measurements;
}