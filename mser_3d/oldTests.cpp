//
// Created by alex on 2/16/16.
//

#include "oldTests.h"
#include "InputManager.h"
#include "optimization.h"


using namespace gtsam;
using namespace std;
using namespace noiseModel;


//Unit test for 3D object location
void testLocateObject(){
    //test for object localization via back projection
    Point3 target = Point3(1.5,1.5,1.5);
    Point3 guess = Point3(1.1,1.1,1.1);
    int numCameras = 8;
    double cameraMotionRadius = 10.0;
    Values correct1;
    correct1.insert(1,target);
    Values result1 = locateObject(target, guess, numCameras, cameraMotionRadius);
    if (result1.equals(correct1,0.0001)){
        cout << "Localization PASSED." << endl;
    }
    else{
        cout << "Localization FAILED." << endl;
    }
}

void testEllipse2DOrientation(){
    Point2 center(5,5);
    Point2 majorAxisTip(10,10);
    Matrix12 H1, H2, correctCenterJacobian, correctAxisPointJacobian;
    double orientation = ellipse2DOrientation(center, majorAxisTip, H1, H2);
    //cout << orientation << H1 << H2 << endl;
    correctCenterJacobian << 0.1, -0.1; //hand calculated value
    correctAxisPointJacobian << -0.1, 0.1; //hand calculated value
    double correctOrientation = 0.7854; //(radians) hand calculated value
    Values correct;
    Values tested;
    correct.insert(1, correctCenterJacobian);
    correct.insert(2, correctAxisPointJacobian);
    correct.insert(3, correctOrientation);
    tested.insert(1, H1);
    tested.insert(2, H2);
    tested.insert(3, orientation);
    if (correct.equals(tested, 0.00001)){
        cout << "ellipse2DOrientation PASSED" << endl;
    } else {
        cout << "ellipse2DOrientation FAILED" << endl;
        correct.print();
        tested.print();
    }
}

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

void syntheticTestOptimization(bool visualize, bool showEachStep, int levMarIterations){
    //Make correct object
    Point3 objectCenter(12,12,12);
    Rot3 objectOrientation(1,0,0,
              0,1,0,
              0,0,1);
    //Rot3 objectOrientation = zero.Yaw(0.3);
    //objectOrientation = objectOrientation.Roll(0.1);
    //objectOrientation = objectOrientation.Pitch(-0.3);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject correctObject(objectPose,objectAxes); //ground truth object

    //Make slightly wrong initial guess
    Point3 initialGuessCenter = objectCenter + Point3(0.2,0.2,-0.2);
    Rot3 initialGuessOrientation = objectOrientation.Yaw(0.2);
    initialGuessOrientation = initialGuessOrientation.Roll(0.2);
    initialGuessOrientation = initialGuessOrientation.Pitch(-0.2);
    Point2 initialGuessAxes(2.7,0.5);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    MserObject initialGuess(initialGuessPose, initialGuessAxes);

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

    //Create ideal measurements taken by cameras of correct object
    std::vector<MserMeasurement> measurements = createIdealMeasurements(cameras,correctObject);

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
        Vector3 objectColor = Vector3(255 /*-i*(255 / levMarIterations)*/,200-i*(255 / levMarIterations),200-i*(255 / levMarIterations)); //deeper shades mean more optimal objects
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

void testMeasurementFunction(){
    //Make camera
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 cameraPosition(0,0,10);// = objectCenter + Point3(0,0,100);
    Point3 up = Point3(0,1,0);
    SimpleCamera camera = SimpleCamera::Lookat(cameraPosition, Point3(0,0,0), up, *K);


    //Make object
    Point3 objectCenter1(0,0,0);
    Rot3 objectOrientation1(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes1(3,1);
    Pose3 objectPose1(objectOrientation1, objectCenter1);
    MserObject object1(objectPose1,objectAxes1);


    //Jacobian matrices
    Eigen::MatrixXd Dcamera(5,11);
    Matrix58 Dobject;
    //Take measurement
    MserMeasurement returnedMeasurement1 = measurementFunction(camera, object1, Dcamera, Dobject);
    /*
    //Hand calculated measurement
    MserMeasurement correctMeasurement(Pose2(320,240,0),Point2(150,50));
    //Check correctness
    if (gtsam::traits<MserMeasurement>::Equals(correctMeasurement,returnedMeasurement,0.001)){
        cerr << "TEST: MEASUREMENT FUNCTION TEST #1 PASSED" << endl;
        gtsam::traits<MserMeasurement>::Print(correctMeasurement, "CORRECT");
        gtsam::traits<MserMeasurement>::Print(returnedMeasurement, "RETURNED");
    } else {
        cerr << "TEST: MEASUREMENT FUNCTION TEST #1 FAILED" << endl;
        gtsam::traits<MserMeasurement>::Print(correctMeasurement, "CORRECT");
        gtsam::traits<MserMeasurement>::Print(returnedMeasurement, "RETURNED");
    } */
    cerr << "TEST 1 RESULTS: OBJECT AT ORIGIN" << endl; //
    gtsam::traits<MserMeasurement>::Print(returnedMeasurement1, "RETURNED");
    //test 2
    Point3 objectCenter2(1,0,0);
    Rot3 objectOrientation2(1,0,0,
                            0,1,0,
                            0,0,1);
    Point2 objectAxes2(3,1);
    Pose3 objectPose2(objectOrientation2, objectCenter2);
    MserObject object2(objectPose2,objectAxes2);

    MserMeasurement returnedMeasurement2 = measurementFunction(camera, object2, Dcamera, Dobject);
    cerr << "TEST 2 RESULTS: X POSITION INCREASED" << endl;
    gtsam::traits<MserMeasurement>::Print(returnedMeasurement2, "RETURNED");
    //test 3
    Point3 objectCenter3(0,1,0);
    Rot3 objectOrientation3(1,0,0,
                            0,1,0,
                            0,0,1);
    Point2 objectAxes3(3,1);
    Pose3 objectPose3(objectOrientation3, objectCenter3);
    MserObject object3(objectPose3,objectAxes3);

    MserMeasurement returnedMeasurement3 = measurementFunction(camera, object3, Dcamera, Dobject);
    cerr << "TEST 3 RESULTS: Y POSITION INCREASED" << endl;
    gtsam::traits<MserMeasurement>::Print(returnedMeasurement3, "RETURNED");
    //test 4
    Point3 objectCenter4(0,0,1);
    Rot3 objectOrientation4(1,0,0,
                            0,1,0,
                            0,0,1);
    Point2 objectAxes4(3,1);
    Pose3 objectPose4(objectOrientation4, objectCenter4);
    MserObject object4(objectPose4,objectAxes4);

    MserMeasurement returnedMeasurement4 = measurementFunction(camera, object4, Dcamera, Dobject);
    cerr << "TEST 4 RESULTS: Y POSITION INCREASED" << endl;
    gtsam::traits<MserMeasurement>::Print(returnedMeasurement4, "RETURNED");
}

//Draw all of the poses that we get from visual odometry
void testDisplayPoses(const InputManager& input){
    std::vector<Pose3> poses;
    input.getVOCameraPoses(poses);
    std::vector<MserObject> dummyObjects;
    Vector3 color = Vector3(0,0,0);
    std::vector<Vector3> colors;
    for (int p = 0; p < poses.size(); p++){
        poses[p].print();
        Point2 axes = Point2(0,0); //Hack: Draw "invisible" ellipse. Only axes will show.
        MserObject dummyObject = MserObject(poses[p],axes);
        dummyObjects.push_back(dummyObject);
        colors.push_back(color);
    }
    drawMserObjects(poses, dummyObjects, colors);
}

void realWorldTestOptimization(const InputManager& input){
    std::vector<MserTrack> tracks;// = getMserTracksFromCSV();
    std::vector<Pose3> allCameraPoses;// = getPosesFromBAL();
    input.getMSERMeasurementTracks(tracks);
    input.getVOCameraPoses(allCameraPoses);
    //shows only relevant camera poses; VERY INEFFICIENT!!!
    std::vector<Pose3> relevantCameraPoses;
    for (int t = 0; t < tracks.size(); t++){
        for (int f = 0; f < tracks[t].frameNumbers.size(); f++){ //the frame number corresponds to the pose index??
            relevantCameraPoses.push_back(allCameraPoses[tracks[t].frameNumbers[f]]);
        }
    }
    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks, allCameraPoses); //use all poses because this function expects to look through everything from getPosesFromBAL()
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

void testAll(const InputManager& input){
    testLocateObject();
    testEllipse2DOrientation();
    testPrintSuperimposedMeasurementImages(input);
    syntheticTestOptimization();
    realWorldTestOptimization(input);
    testMeasurementFunction();
    testDisplayPoses(input);
}