//
// Created by alex on 2/16/16.
//

#include "tests.h"
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

void testMserObjectDrawing(){
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject groundTruthObject(objectPose,objectAxes);
    //Make initial guess
    Point3 initialGuessCenter(0.2,1.5,-0.5);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.5);
    Point2 initialGuessAxes(1.7,0.5);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    MserObject initialGuess(initialGuessPose, initialGuessAxes);
    //Get measurements as well
    std::vector<SimpleCamera> cameras = alexCreateCameras(15,groundTruthObject.first.translation(),20); //make a bunch of cameras to pass to measurement function
    std::vector<MserMeasurement> measurements = createIdealMeasurements(cameras, groundTruthObject); //synthetic measurements directly from measurement function
    //std::vector<Pose3> poses;
    std::vector<MserObject> measurementsAsObjects; //needed to draw both measurements and resulting optimize object on same screen
    for (size_t i = 0; i < cameras.size(); i++){
        //poses.push_back(cameras[i].pose());
        MserObject temporary(cameras[i].pose(),measurements[i].second/30);
        measurementsAsObjects.push_back(temporary);
    }

    //Optimize
    Values result = expressionsOptimizationSynthetic(groundTruthObject, initialGuess, 100);
    MserObject returnedObject = result.at<MserObject>(Symbol('o',0)); //retrieve object
    std::vector<MserObject> objects;
    objects.push_back(groundTruthObject);
    objects.push_back(returnedObject);
    objects.push_back(initialGuess);
    objects.insert(objects.end(),measurementsAsObjects.begin(),measurementsAsObjects.end());
    std::vector<Pose3> cameraPoses;
    for (int i = 0; i < cameras.size(); i++){
        cameraPoses.push_back(cameras[i].pose());
    }
    drawMserObjects(cameraPoses, objects);
}

void testExpressionsOptimization(){
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject groundTruthObject(objectPose,objectAxes); //ground truth object

    //Make initial guess
    Point3 initialGuessCenter(0.1,0.1,0.1);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.2);
    Point2 initialGuessAxes(2.7,0.9);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    MserObject initialGuess(initialGuessPose, initialGuessAxes);

    //Check correctness
    Values correct;
    correct.insert(Symbol('o',0),groundTruthObject);
    Values result = expressionsOptimizationSynthetic(groundTruthObject, initialGuess, 100);

    if (correct.equals(result,0.1)){
        cout << "EXPRESSIONS OPTIMIZATION PASSED" << endl;
    } else {
        cout << "EXPRESSIONS OPTIMIZATION FAILED" << endl;
        correct.print("CORRECT OBJECT\n");
        result.print("RETURNED OBJECT\n");
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

void testExpressionsOptimizationWithBadInitialGuess(){
    //Make correct object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject groundTruthObject(objectPose,objectAxes); //ground truth object

    //Make initial guess
    Point3 initialGuessCenter(0.2,0.2,-0.2);
    Rot3 initialGuessOrientation = objectOrientation.yaw(0.2);
    Point2 initialGuessAxes(2.7,0.5);
    Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
    MserObject initialGuess(initialGuessPose, initialGuessAxes);

    //Check correctness
    Values correct;
    correct.insert(Symbol('o',0),groundTruthObject);
    Values result = expressionsOptimizationSynthetic(groundTruthObject, initialGuess, 100);
    if (correct.equals(result,0.1)){
        cout << "EXPRESSIONS OPTIMIZATION W/ BAD GUESS PASSED" << endl;
    } else {
        cout << "EXPRESSIONS OPTIMIZATION W/ BAD GUESS FAILED" << endl;
        correct.print("CORRECT OBJECT\n");
        result.print("RETURNED OBJECT\n");
    }

    MserObject returnedObject = result.at<MserObject>(Symbol('o',0));
    gtsam::traits<MserObject>::Print(returnedObject);
}

void testMeasurementFunction(){
    //Make object
    Point3 objectCenter(0,0,0);
    Rot3 objectOrientation(1,0,0,
                           0,1,0,
                           0,0,1);
    Point2 objectAxes(3,1);
    Pose3 objectPose(objectOrientation, objectCenter);
    MserObject object(objectPose,objectAxes);
    //Make camera
    Cal3_S2::shared_ptr K(new Cal3_S2(500.0, 500.0, 0.1, 640/2, 480/2));
    Point3 cameraPosition(0,0,10);// = objectCenter + Point3(0,0,100);
    Point3 up = Point3(0,1,0);
    SimpleCamera camera = SimpleCamera::Lookat(cameraPosition, objectCenter, up, *K);
    //Jacobian matrices
    Eigen::MatrixXd Dcamera(5,11);
    Matrix58 Dobject;
    //Take measurement
    MserMeasurement returnedMeasurement = measurementFunction(camera, object, Dcamera, Dobject);
    //Hand calculated measurement
    MserMeasurement correctMeasurement(Pose2(320,240,0),Point2(150,50));
    //Check correctness
    if (gtsam::traits<MserMeasurement>::Equals(correctMeasurement,returnedMeasurement,0.001)){
        cout << "MEASUREMENT FUNCTION TEST #1 PASSED" << endl;
    } else {
        cout << "MEASUREMENT FUNCTION TEST #1 FAILED" << endl;
        gtsam::traits<MserMeasurement>::Print(correctMeasurement, "CORRECT");
        gtsam::traits<MserMeasurement>::Print(returnedMeasurement, "RETURNED");
    }
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

void test3DReconstruction(const InputManager& input){
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
    testMserObjectDrawing();
    testPrintSuperimposedMeasurementImages(input);
    testExpressionsOptimization();
    testExpressionsOptimizationWithBadInitialGuess();
    testMeasurementFunction();
    testDisplayPoses(input);
    test3DReconstruction(input);
}