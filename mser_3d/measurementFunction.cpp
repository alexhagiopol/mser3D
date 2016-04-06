//
// Created by alex on 2/16/16.
//

#include "measurementFunction.h"
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Testable.h>

PointsPose convertObjectToPointsPose(const MserObject& object, OptionalJacobian<12,8> Dobject){
    PointsPose myPointsPose(object.first, Point3(object.second.x(),0,0), Point3(0,object.second.y(),0));
    //gtsam::traits<PointsPose>::Print(myPointsPose, "POINTS POSE \n");
    if (Dobject){
        Eigen::MatrixXd Dobject_(12,8);
        Dobject_ << 1,0,0,0,0,0,0,0,
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
        *Dobject << Dobject_;
        boost::function<PointsPose(const MserObject&)> f = boost::bind(&convertObjectToPointsPose, _1, boost::none);
        //printf("Computing: convertObjectToPointsPose(): Dobject \n");
        //assert_equal(numericalDerivative11(f,object),Dobject_,1e-5);
    }
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
    //gtsam::traits<WorldPoints>::Print(myWorldPoints, "WORLD POINTS \n");
    if (Dpointspose){ //compute Jacobian
        const Eigen::MatrixXd zeros33 = Eigen::MatrixXd::Zero(3,3);
        const Eigen::MatrixXd eye33 = Eigen::MatrixXd::Identity(3,3);

        Eigen::MatrixXd worldCenterDPointsPose(3,12); //top third of output Jacobian
        worldCenterDPointsPose << centerDpose, zeros33, zeros33;

        Eigen::MatrixXd worldMajAxisTipDPointsPose(3,12);
        worldMajAxisTipDPointsPose << majAxisDpose, majAxisDpoint, zeros33; //middle third of output Jacobian

        Eigen::MatrixXd worldMinAxisTipDPointsPose(3,12);
        worldMinAxisTipDPointsPose << minAxisDpose, zeros33, minAxisDpoint; //bottom third of output Jacobian

        Eigen::MatrixXd Dpointspose_(9,12);
        Dpointspose_ << worldCenterDPointsPose,
                        worldMajAxisTipDPointsPose,
                        worldMinAxisTipDPointsPose;
        *Dpointspose << Dpointspose_;
        boost::function<WorldPoints(const PointsPose&)> f = boost::bind(&convertPointsPoseToWorldPoints, _1, boost::none);
        //printf("Computing: convertPoiintsPoseToWorldPoints(): Dpointspose \n");
        //assert_equal(numericalDerivative11(f,objectPointsPose),Dpointspose_,1e-5);
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
    boost::function<CameraPoints(const SimpleCamera&, const WorldPoints&)> f = boost::bind(&convertWorldPointsToCameraPoints, _1, _2, boost::none, boost::none);
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
        //printf("Computing: convertWorldPointsToCameraPoints(): Dcamera \n");
        //assert_equal(numericalDerivative21(f,camera,points),Dcamera_,1e-5);
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
        //printf("Computing: convertWorldPointsToCameraPoints(): Dpoints \n");
        //assert_equal(numericalDerivative22(f,camera,points),Dpoints_,1e-5);
    }
    return myCameraPoints;
}

//******This is the most problematic function. Derivatives are wrong when object is viewed from the side and major axis appears to have length zero.*****
MserMeasurement convertCameraPointsToMeasurement(const CameraPoints& cameraPoints, OptionalJacobian<5,6> Dpoints){
    Point2 measurementCenter = gtsam::traits<CameraPoints>::centroid(cameraPoints); //Jacobian for this is 2x2 identity matrix
    Point2 majAxisTip = gtsam::traits<CameraPoints>::majAxisTip(cameraPoints);
    Point2 minAxisTip = gtsam::traits<CameraPoints>::minAxisTip(cameraPoints);

    //derivatives matrices
    Matrix12 thetaDdifference, A1Dmajor, A1Dcenter, A2Dminor, A2Dcenter;
    const Point2 difference = majAxisTip - measurementCenter;
    const Rot2 theta = Rot2::relativeBearing(difference,thetaDdifference);
    Pose2 ellipsePose = Pose2(theta,measurementCenter);
    double A1 = majAxisTip.distance(measurementCenter, A1Dmajor, A1Dcenter);
    double A2 = minAxisTip.distance(measurementCenter, A2Dminor, A2Dcenter);
    Point2 axisLengths(A1,A2);
    MserMeasurement measurement(ellipsePose,axisLengths);
    if (Dpoints){
        Eigen::MatrixXd Dpoints_(5,6);
        Matrix22 differenceDcenter, differenceDmajor, differenceDminor;
        Matrix26 differenceDpoints;
        Matrix16 thetaDpoints;
        Matrix26 msmtctrDpoints;
        Matrix26 axesDpoints;

        differenceDcenter = -1*eye(2);
        differenceDmajor = eye(2);
        differenceDminor = zeros(2,2);
        differenceDpoints << differenceDcenter, differenceDmajor, differenceDminor;
        double t = theta.theta();
        //first 2 rows
        msmtctrDpoints << cos(t),        sin(t),                0,                0,             0,             0,
                       -1*sin(t),        cos(t),                0,                0,             0,             0;
        //third row
        thetaDpoints << thetaDdifference*differenceDpoints;
        //last 2 rows
        axesDpoints << A1Dcenter(0,0),    A1Dcenter(0,1),    A1Dmajor(0,0),    A1Dmajor(0,1),             0,             0,
                       A2Dcenter(0,0),    A2Dcenter(0,1),                0,                0, A2Dminor(0,0), A2Dminor(0,1);

        Dpoints_<< msmtctrDpoints, thetaDpoints, axesDpoints;
        *Dpoints << Dpoints_;
        /*MOST COMMONLY NEEDED TEST*/
        //boost::function<MserMeasurement(const CameraPoints&)> f = boost::bind(&convertCameraPointsToMeasurement, _1, boost::none);
        //assert_equal(numericalDerivative11(f,cameraPoints),Dpoints_,1e-2);
    }
    return measurement;
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
    //Auto test for correct derivatives
    boost::function<MserMeasurement(const SimpleCamera&, const MserObject&)> f = boost::bind(&measurementFunction, _1, _2, boost::none, boost::none);

    if (Dcamera) {
        //std::cerr << "MSMT FCN: VERIFYING msmtDcamera" << std::endl;
        Eigen::MatrixXd Dcamera_(5,11);
        Dcamera_ << msmtDcampoints56*campointsDcamera; //5x6 x 6x11 = 5x11
        *Dcamera << Dcamera_;
        //printf("Computing: measurementFunction(): Dcamera \n");
        //assert_equal(numericalDerivative21(f,camera,object),Dcamera_,1e-5);
    }
    if (Dobject){
        //std::cerr << "MSMT FCN: VERIFYING msmtDobject" << std::endl;
        Eigen::MatrixXd Dobject_(5,8);
        Dobject_ << msmtDcampoints56*campointsDworldpoints69*worldpointsDobjectpointspose9_12*objectpointsposeDobject12_8;
        *Dobject << Dobject_;
        //printf("Computing: measurementFunction(): Dobject \n");
        //assert_equal(numericalDerivative22(f,camera,object),Dobject_,1e-5);
    }
    return measurement;
}

std::vector<MserMeasurement> createIdealMeasurements(const std::vector<SimpleCamera>& cameras, const MserObject& object){
    std::vector<MserMeasurement> measurements;
    for (size_t i = 0; i < cameras.size(); i++){
        MserMeasurement measurement = measurementFunction(cameras[i], object);
        measurements.push_back(measurement);
    }
    return measurements;
}

std::vector<MserMeasurement> createNoisyMeasurements(const std::vector<SimpleCamera>& cameras, const MserObject& object){
    std::vector<MserMeasurement> measurements;
    std::default_random_engine generator;
    std::normal_distribution<double> pixelDistribution(0.0,15); //15 pixel stdev
    std::normal_distribution<double> thetaDistribution(0.0,0.5); //0.5 radian stdev

    for (size_t i = 0; i < cameras.size(); i++){
        MserMeasurement measurement = measurementFunction(cameras[i], object);
        Pose2 idealPose2 = measurement.first;
        Point2 idealPoint2 = measurement.second;
        Pose2 noisyPose2 = Pose2(idealPose2.x() + pixelDistribution(generator), idealPose2.y() + pixelDistribution(generator), idealPose2.theta() + thetaDistribution(generator));
        Point2 noisyPoint2 = Point2(idealPoint2.x() + pixelDistribution(generator), idealPoint2.y() + pixelDistribution(generator));
        MserMeasurement noisyMeasurement = MserMeasurement(noisyPose2,noisyPoint2);
        measurements.push_back(noisyMeasurement);
    }
    return measurements;
}
