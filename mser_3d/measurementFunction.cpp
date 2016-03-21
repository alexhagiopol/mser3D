//
// Created by alex on 2/16/16.
//

#include "measurementFunction.h"

PointsPose convertObjectToPointsPose(const MserObject& object, OptionalJacobian<12,8> Dobject){
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

WorldPoints convertPointsPoseToWorldPoints(const PointsPose& objectPointsPose, OptionalJacobian<9,12> Dpointspose){
    Matrix36 centerDpose, majAxisDpose, minAxisDpose; //Matrices to store results from optional jacobians
    Matrix33 majAxisDpoint, minAxisDpoint; //Matrices to store results from optional jacobians
    const Pose3 objectPose = gtsam::traits<PointsPose>::objectPose(objectPointsPose);
    const Point3 objectCenter = objectPose.translation(centerDpose);
    const Point3 majAxisInObjectFrame = gtsam::traits<PointsPose>::majAxisTip(objectPointsPose);
    const Point3 minAxisInObjectFrame = gtsam::traits<PointsPose>::minAxisTip(objectPointsPose);
    const Point3 majAxisInWorldFrame = objectPose.transform_from(majAxisInObjectFrame,majAxisDpose,majAxisDpoint);
    const Point3 minAxisInWorldFrame = objectPose.transform_from(minAxisInObjectFrame,minAxisDpose,minAxisDpoint);
    WorldPoints myWorldPoints = WorldPoints(objectCenter,majAxisInWorldFrame,minAxisInWorldFrame);

    if (Dpointspose){ //compute Jacobian
        const Eigen::MatrixXd zeros33 = Eigen::MatrixXd::Zero(3,3);
        const Eigen::MatrixXd eye33 = Eigen::MatrixXd::Identity(3,3);

        Eigen::MatrixXd worldCenterDPointsPose(3,12); //top third of output Jacobian
        worldCenterDPointsPose << zeros33, eye33, zeros33, zeros33;

        Eigen::MatrixXd worldMajAxisTipDPointsPose(3,12);
        worldMajAxisTipDPointsPose << majAxisDpose, majAxisDpoint, zeros33; //middle third of output Jacobian

        Eigen::MatrixXd worldMinAxisTipDPointsPose(3,12);
        worldMinAxisTipDPointsPose << minAxisDpose, zeros33, minAxisDpoint; //bottom third of output Jacobian

        Eigen::MatrixXd Dpointspose_(9,12);
        Dpointspose_ << worldCenterDPointsPose,
                        worldMajAxisTipDPointsPose,
                        worldMinAxisTipDPointsPose;
        *Dpointspose << Dpointspose_;
    }
    return myWorldPoints;
}

CameraPoints convertWorldPointsToCameraPoints(const SimpleCamera& camera, const WorldPoints& points, OptionalJacobian<6,11> Dcamera, OptionalJacobian<6,9> Dpoints){
    const Point3 objectCenter = gtsam::traits<WorldPoints>::centroid(points);
    const Point3 majorAxisTip = gtsam::traits<WorldPoints>::majAxisTip(points);
    const Point3 minorAxisTip = gtsam::traits<WorldPoints>::minAxisTip(points);
    Matrix26 centerDpose, majorDpose, minorDpose;
    Matrix23 centerDpoint, majorDpoint, minorDpoint;
    Matrix25 centerDcal, majorDcal, minorDcal;
    const Point2 projectedObjectCenter = camera.project(objectCenter, centerDpose, centerDpoint, centerDcal);
    const Point2 projectedMajorAxisTip = camera.project(majorAxisTip, majorDpose, majorDpoint, majorDcal);
    const Point2 projectedMinorAxisTip = camera.project(minorAxisTip, minorDpose, minorDpoint, minorDcal);
    CameraPoints myCameraPoints = CameraPoints(projectedObjectCenter,projectedMajorAxisTip,projectedMinorAxisTip);

    if (Dcamera) {
        Eigen::MatrixXd Dpose(6,6);
        Dpose << centerDpose,
                majorDpose,
                minorDpose;

        Eigen::MatrixXd Dcal(6,5);
        Dcal << centerDcal,
                majorDcal,
                minorDcal;

        Eigen::MatrixXd Dcamera_(6,11);
        Dcamera_ << Dpose, Dcal;
        *Dcamera <<  Dcamera_;
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
    return myCameraPoints;
}

MserMeasurement convertCameraPointsToMeasurement(const CameraPoints& cameraPoints, OptionalJacobian<5,6> Dpoints){
    Point2 measurementCenter = gtsam::traits<CameraPoints>::centroid(cameraPoints); //Jacobian for this is 2x2 identity matrix
    Point2 majAxisTip = gtsam::traits<CameraPoints>::majAxisTip(cameraPoints);
    Point2 minAxisTip = gtsam::traits<CameraPoints>::minAxisTip(cameraPoints);
    Matrix12 thetaDcenter, thetaDmajor, A1Dmajor, A1Dcenter, A2Dminor, A2Dcenter;
    double theta = ellipse2DOrientation(measurementCenter, majAxisTip, thetaDcenter, thetaDmajor);
    double A1 = majAxisTip.distance(measurementCenter, A1Dmajor, A1Dcenter);
    double A2 = minAxisTip.distance(measurementCenter, A2Dminor, A2Dcenter);
    Pose2 ellipsePose(theta,measurementCenter);
    Point2 axisLengths(A1,A2);
    MserMeasurement measurement(ellipsePose,axisLengths);
    if (Dpoints){
        Eigen::MatrixXd Dpoints_(5,6);

        Dpoints_<<        cos(theta),        sin(theta),                0,                0,             0,             0,
                       -1*sin(theta),        cos(theta),                0,                0,             0,             0,
                   thetaDcenter(0,0), thetaDcenter(0,1), thetaDmajor(0,0), thetaDmajor(0,1),             0,             0,
                      A1Dcenter(0,0),    A1Dcenter(0,1),    A1Dmajor(0,0),    A1Dmajor(0,1),             0,             0,
                      A2Dcenter(0,0),    A2Dcenter(0,1),                0,                0, A2Dminor(0,0), A2Dminor(0,1);
        *Dpoints << Dpoints_;
    }
    return measurement;
}

//If we can fix toy experiment, then we can fix the entire pipeline.
Pose2 toyExperiment(const Point2& center, const double& theta, OptionalJacobian<3,2> Dcenter, OptionalJacobian<3,1> Dtheta){
    if(Dcenter){
        Eigen::MatrixXd Dcenter_(3,2);
        Dcenter_ <<    cos(theta),  sin(theta),
                    -1*sin(theta), cos(theta),
                                0,          0;
        *Dcenter << Dcenter_;
    }
    if(Dtheta){
        Eigen::MatrixXd Dtheta_(3,1);
        Dtheta_ << 0,
                   0,
                   1;
        *Dtheta << Dtheta_;
    }
    return Pose2(theta,center);
}

MserMeasurement measurementFunction(const SimpleCamera& camera, const MserObject& object, OptionalJacobian<5,11> Dcamera, OptionalJacobian<5,8> Dobject){
    //Part 1: object -> 2X Point3 in object Frame + 1X Pose3
    Eigen::MatrixXd objectpointsposeDobject12_8(12,8);
    PointsPose objectPointsPose = convertObjectToPointsPose(object, objectpointsposeDobject12_8);

    //Part 2: 2X Point3s in object frame + Pose 3 -> 3X Point3s in world frame
    Eigen::MatrixXd worldpointsDobjectpointspose9_12(9,12);
    WorldPoints worldPoints = convertPointsPoseToWorldPoints(objectPointsPose, worldpointsDobjectpointspose9_12);

    //Part 3: 3X Point3s in world frame -> 3X Point2s in camera frame
    Eigen::MatrixXd campointsDcamera(6,11);
    Matrix69 campointsDworldpoints69;
    CameraPoints cameraPoints = convertWorldPointsToCameraPoints(camera, worldPoints, campointsDcamera, campointsDworldpoints69);

    //Part 4: 3X Point2s in camera frame -> 1X measurement
    Matrix56 msmtDcampoints56;
    MserMeasurement measurement = convertCameraPointsToMeasurement(cameraPoints, msmtDcampoints56);

    //Provide Jacobians
    if (Dcamera) {
        Eigen::MatrixXd Dcamera_(5,11);
        Dcamera_ << msmtDcampoints56*campointsDcamera; //5x6 x 6x11 = 5x11
        cout << "msmtDcampoints \n" << msmtDcampoints56 << endl;
        cout << "campointsDcamera \n" << campointsDcamera << endl;
        *Dcamera << Dcamera_;
    }
    if (Dobject) *Dobject << msmtDcampoints56*campointsDworldpoints69*worldpointsDobjectpointspose9_12*objectpointsposeDobject12_8;
    return measurement;
}

std::vector<MserMeasurement> createIdealMeasurements(const std::vector<SimpleCamera>& cameras, MserObject& object){
    std::vector<MserMeasurement> measurements;
    for (size_t i = 0; i < cameras.size(); i++){
        MserMeasurement measurement = measurementFunction(cameras[i], object);
        measurements.push_back(measurement);
    }
    return measurements;
}