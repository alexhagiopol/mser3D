//
// Created by alex on 2/16/16.
//

#include "oldTests.h"
#include "InputManager.h"
#include "optimization.h"


using namespace gtsam;
using namespace std;
using namespace noiseModel;

//Write video images with superimposed MSER measurements to disk in order to test/verify
void testPrintSuperimposedMeasurementImages(const InputManager& input){
    std::vector<MserTrack> tracks;// = input.MSERmeasurementTracks;
    std::vector<Pose3> poses;// = getPosesFromBAL();
    input.getMSERMeasurementTracks(tracks);
    input.getVOCameraPoses(poses);
    //std::pair<std::vector<MserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks, poses);
    //drawMserObjects(pair.first,pair.second);

    //Extract video frames and store in memory
    cv::VideoCapture capture("/home/alex/mser/datasets/through_the_cracks_jing.mov");
    std::vector<cv::Mat> allVideoFrames;
    int f = 0;
    if (!capture.isOpened()) {
        cerr << "The video file could not be opened successfully!!!" << endl;
    } else {
        bool readSuccess = true;
        while (readSuccess == true) {
            cv::Mat videoFrame;
            readSuccess = capture.read(videoFrame);
            if (f > 13) { //remove first 14 frames because Matlab and OpenCV don't open the same video in the same way :(
                allVideoFrames.push_back(videoFrame);
            }
            f++;
        }
    }
    capture.release();

    //Draw MserMeasurement ellipses on each video frame
    for (int t = 0; t < tracks.size(); t++){
        for (int f = 0; f < tracks[t].frameNumbers.size(); f++){
            int frameNum = tracks[t].frameNumbers[f] - 1;
            MserMeasurement msmt = tracks[t].measurements[f];
            double majAxisLength = msmt.second.x();
            double minAxisLength = msmt.second.y();
            double theta = msmt.first.theta();
            double cvTheta = msmt.first.theta()*180/M_PI; //opencv wants angles in degrees

            cv::Point center = cv::Point(msmt.first.x(),msmt.first.y());
            cv::Size axes = cv::Size(majAxisLength,minAxisLength);
            cv::Point majAxisTip = cv::Point(center.x + majAxisLength*cos(theta),center.y + majAxisLength*sin(theta));
            cv::Point minAxisTip = cv::Point(center.x + minAxisLength*cos(theta + M_PI/2), center.y + minAxisLength*sin(theta + M_PI/2));
            /* double majX = ctrX + majAxis*cos(theta);
            double majY = ctrY + majAxis*sin(theta); */
            cv::Scalar color = cv::Scalar(tracks[t].colorB,tracks[t].colorG,tracks[t].colorR);
            int thickness = 5;
            int startAngle = 0;
            int endAngle = 360;//draw entire ellipse
            cv::ellipse(allVideoFrames[frameNum],center,axes,cvTheta,startAngle,endAngle,color,thickness);
            cv::line(allVideoFrames[frameNum],center,majAxisTip,color,thickness);
            cv::line(allVideoFrames[frameNum],center,minAxisTip,color,thickness);
        }
    }

    //Store images to disk
    string outputDir = "/home/alex/mser/mser_3d/unit_test_output/";
    int imgDirectorySuccess = mkdir(outputDir.c_str(), 0777);
    for (int f = 0; f < allVideoFrames.size(); f++){
        char imgFileName[200];
        cv::Mat videoFrame = allVideoFrames[f];
        strcpy(imgFileName,outputDir.c_str());
        strcat(imgFileName,"Frame%04i.bmp");
        sprintf(imgFileName, imgFileName,f);
        cv::imwrite(imgFileName, videoFrame);
    }
}

void syntheticTestOptimization(bool visualize, bool showEachStep, bool noisy, int levMarIterations){
    //Make correct object
    const Point3 objectCenter(12,12,12);
    const Rot3 objectOrientation(1,0,0,
              0,1,0,
              0,0,1);
    //Rot3 objectOrientation = zero.Yaw(0.3);
    //objectOrientation = objectOrientation.Roll(0.1);
    //objectOrientation = objectOrientation.Pitch(-0.3);
    const Point2 objectAxes(3,1);
    const Pose3 objectPose(objectOrientation, objectCenter);
    const MserObject correctObject(objectPose,objectAxes); //ground truth object

    //Make slightly wrong initial guess
    const Point3 initialGuessCenter = objectCenter + Point3(4,4,-4);
    Rot3 initialGuessOrientation = objectOrientation.Yaw(0.8);
    initialGuessOrientation = initialGuessOrientation.Roll(0.8);
    initialGuessOrientation = initialGuessOrientation.Pitch(-0.8);
    const Point2 initialGuessAxes(0.7,7.1);
    const Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    const MserObject initialGuess(initialGuessPose, initialGuessAxes);

    //Create cameras looking directly at correct object
    Cal3_S2::shared_ptr K(new Cal3_S2(857.483, 876.718, 0.1, 1280/2, 720/2)); //gopro camera calibration from http://www.theeminentcodfish.com/gopro-calibration/
    std::vector<SimpleCamera> cameras;
    std::vector<Pose3> camPoses;
    double maxTheta = M_PI/2;
    double theta = 0.0;
    Point3 up = Point3(0,0,1);
    int numCams = 20;
    double radius = 10;
    for(int i = 0; i < numCams; i++){
        Point3 position = Point3(objectCenter.x() + radius*cos(theta), objectCenter.y(), objectCenter.z()  + radius*sin(theta)); //moving in XZ plane
        SimpleCamera tempCam = SimpleCamera::Lookat(position, objectCenter, up, *K);
        cameras.push_back(tempCam);
        camPoses.push_back(tempCam.pose());
        theta += maxTheta/numCams;
    }

    //Create noisy or ideal measurements taken by cameras of correct object
    std::vector<MserMeasurement> measurements;
    if (noisy) {
        measurements = createNoisyMeasurements(cameras,correctObject);
    } else {
        measurements = createIdealMeasurements(cameras,correctObject);
    }

    //Create a single MserTrack for all of the measurements
    std::vector<MserTrack> tracks;
    MserTrack track;
    for (int i = 0; i < numCams; i++){
        track.frameNumbers.push_back(i);
        track.measurements.push_back(measurements[i]);
    }
    track.colorB = 100; //arbitrary colors
    track.colorG = 150;
    track.colorR = 200;
    tracks.push_back(track);

    //Infer object using my function for real MSER measurements; note this uses its own initial guess and requires tracks as input
    //std::pair<std::vector<MserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks,camPoses);

    //Infer objects using multiple expressions optimizations with increasing Levenberg Marquardt iterations
    std::vector<MserObject> objects;
    std::vector<Vector3> colors;

    //Store optimization data to file
    std::string csvFileName = "syntheticOptimizationTestResults.csv";
    ofstream csvData;
    csvData.open(csvFileName);

    csvData << "CORRECT VALUES" << endl;
    csvData << "X" << "," << "Y" << "," << "Z" << "," << "Roll" << "," << "Pitch" << "," << "Yaw" << "," << "MajAxis" << "," << "MinAxis" << endl;

    double correctX = correctObject.first.x();
    double correctY = correctObject.first.y();
    double correctZ = correctObject.first.z();
    Vector3 correctAngles = correctObject.first.rotation().rpy();
    double correctRoll = correctAngles[0];
    double correctPitch = correctAngles[1];
    double correctYaw = correctAngles[2];
    double correctMajorAxis = correctObject.second.x();
    double correctMinorAxis = correctObject.second.y();
    csvData << correctX << "," << correctY << "," << correctZ << "," << correctRoll << "," << correctPitch << "," << correctYaw << "," << correctMajorAxis << "," << correctMinorAxis << endl;
    csvData << "ERRORS" << endl;
    csvData << "Iteration #" << "," << "Xerror" << "," << "YError" << "," << "ZError" << "," << "RollError" << "," << "PitchError" << "," << "YawError" << "," << "MajAxisError" << "," << "MinAxisError" << endl;

    int startingPoint;
    if (showEachStep){
        startingPoint = 0;
    } else {
        startingPoint = levMarIterations - 1;
    }

    for (int i = startingPoint; i < levMarIterations; i++){
        Values result = expressionsOptimization(initialGuess,measurements,cameras,i); //Note number of Lev Mar iterations increases with each loop. This is to see how error changes over time.
        MserObject retObject = result.at<MserObject>(Symbol('o',0));
        Vector3 objectColor = Vector3(255-i*(255 / levMarIterations),255-i*(255 / levMarIterations),255-i*(255 / levMarIterations)); //deeper shades mean more optimal objects
        objects.push_back(retObject);
        colors.push_back(objectColor);

        //Extract data from object structure
        double retX = retObject.first.x();
        double retY = retObject.first.y();
        double retZ = retObject.first.z();
        Vector3 retAngles = retObject.first.rotation().rpy();
        double retRoll = retAngles[0];
        double retPitch = retAngles[1];
        double retYaw = retAngles[2];
        double retMajorAxis = retObject.second.x();
        double retMinorAxis = retObject.second.y();

        //compute error in the form of doubles to plot in Matlab
        double Xerror = abs(correctX - retX);
        double Yerror = abs(correctY - retY);
        double Zerror = abs(correctZ - retZ);
        double rollError = abs(correctRoll - retRoll);
        double pitchError = abs(correctPitch - retPitch);
        double yawError = abs(correctYaw - retYaw);
        double majorAxisError = abs(correctMajorAxis - retMajorAxis);
        double minorAxisError = abs(correctMinorAxis - retMinorAxis);
        csvData << i << "," << Xerror << "," << Yerror << "," << Zerror << "," << rollError << "," << pitchError << "," << yawError << "," << majorAxisError << "," << minorAxisError << "," << endl;
    }

    MserObject finalEstimatedObject = objects[objects.size() - 1]; //last object in list is our best estimate

    if (visualize){
        //add correct object in green for visualization
        objects.push_back(correctObject);
        colors.push_back(Vector3(0,255,0));

        //Visualization
        std::vector<std::pair<Point3,Point3>> rays = makeRayTracingPairs(tracks,camPoses);
        drawMserObjects(camPoses,objects,colors,rays);
    }
    csvData.close();

    if (gtsam::traits<MserObject>::Equals(finalEstimatedObject,correctObject,0.01)){
        cerr << "TEST: SYNTHETIC TEST PASSED" << endl;
    } else {
        cerr << "TEST: SYNTHETIC TEST FAILED" << endl;
        gtsam::traits<MserObject>::Print(correctObject,"CORRECT OBJECT PARAMETERS \n");
        gtsam::traits<MserObject>::Print(finalEstimatedObject,"RESULT OBJECT PARAMETERS \n");
    }
}

void realWorldTestOptimization(const InputManager& input, bool showEachStep, int levMarIterations){
    std::vector<MserTrack> tracks;
    std::vector<Pose3> allCameraPoses;
    input.getMSERMeasurementTracks(tracks);
    input.getVOCameraPoses(allCameraPoses);
    //shows only relevant camera poses; VERY INEFFICIENT!!!
    std::vector<Pose3> relevantCameraPoses;
    for (int t = 0; t < tracks.size(); t++){
        for (int f = 0; f < tracks[t].frameNumbers.size(); f++){ //the frame number corresponds to the pose index??
            relevantCameraPoses.push_back(allCameraPoses[tracks[t].frameNumbers[f]]);
        }
    }
    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks, allCameraPoses, showEachStep, levMarIterations); //use all poses because this function expects to look through everything from getPosesFromBAL()
    std::vector<std::pair<Point3,Point3>> rays;
    if (input.showRays()) rays = makeRayTracingPairs(tracks, allCameraPoses);
    //Draw robot axes: causes seg fault for big track datasets because we cross the vertex memory limit.
    //myVisualizer.addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(poses, pair.first,pair.second);
    cerr << "TEST 3D: # of cam poses " << relevantCameraPoses.size() << endl;
    cerr << "TEST 3D: # of objects " << pair.first.size() << endl;
    cerr << "TEST 3D: # of colors " << pair.second.size() << endl;
    cerr << "TEST 3D: # of rays " << rays.size() << endl;
    drawMserObjects(relevantCameraPoses, pair.first, pair.second, rays); //only display relevant poses
}