#include "testVisualization.h"
#include "testGeometry.h"
#include "testMeasurementFunction.h"
#include "Visualizer.h"
#include "MserTrack.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <sys/stat.h>

typedef std::vector<double> record_t;
typedef std::vector<record_t> data_t;

std::istream& operator >> (std::istream& ins, record_t& record){
    record.clear();
    std::string line;
    std::getline(ins, line);
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')){
        std::stringstream fs(field);
        double f = 0.0;
        fs >> f;
        record.push_back(f);
    }
    return ins;
}

std::istream& operator >> (std::istream& ins, data_t& data){
    data.clear();
    record_t record;
    while(ins >> record){
        data.push_back(record);
    }
    return ins;
}

std::vector<MserTrack> getMserTracksFromCSV(){
    std::ifstream infile("/home/alex/mser/mser_2d/mserMeasurements.csv");
    data_t data;
    infile >> data;
    infile.close();
    std::vector<MserTrack> tracks;
    for (int r = 1; r < data.size(); r++){ //start at row 1 because row 0 does not contain data
        MserTrack track;
        int numMSERS = data[r][1];
        for (int c = 2; c < 2 + 6*numMSERS; c = c + 6){
            int frameNum = (int) data[r][c];
            double x = data[r][c+1];
            double y = data[r][c+2];
            double a = data[r][c+3];
            double b = data[r][c+4];
            double theta = data[r][c+5];
            int end = data[r].size() - 1;
            int R = data[r][end - 2];
            int G = data[r][end - 1];
            int B = data[r][end];
            Pose2 pose(x,y,theta);
            Point2 axes(a,b);
            MserMeasurement measurement(pose,axes);
            track.frameNumbers.push_back(frameNum);
            track.measurements.push_back(measurement);
            track.colorR = R;
            track.colorG = G;
            track.colorB = B;
        }
        tracks.push_back(track);
    }
    return tracks;
}

std::vector<Pose3> getPosesFromBAL(){
    SfM_data mydata;
    string filename = "/home/alex/mser/datasets/fpv_bal_280_nf2.txt";
    readBAL(filename, mydata);
    cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();
    std::vector<Pose3> poses;
    BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
                    const Pose3 pose = camera.pose();
                    //pose.print("Camera pose:\n");
                    poses.push_back(pose);
                }
    return poses;
}

std::pair<std::vector<MserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurements(std::vector<MserTrack>& tracks, std::vector<Pose3>& VOposes){
    std::vector<MserObject> objects;
    std::vector<Vector3> colors;
    Cal3_S2::shared_ptr K(new Cal3_S2(470.0, 470.0, 0.1, 1280/2, 720/2));
    for (int t = 0; t < 40 /*tracks.size()*/; t++){
        std::vector<MserMeasurement> measurements = tracks[t].measurements;
        std::cout << "#" << t << " has " << measurements.size() << " measurements" << std::endl;
        std::vector<SimpleCamera> cameras;
        for (int m = 0; m < measurements.size(); m++){
            SimpleCamera camera(VOposes[tracks[t].frameNumbers[m]],*K);
            cameras.push_back(camera);
        }
        //Make initial guess
        Point3 other_initialGuessCenter = cameras[0].backproject(Point2(measurements[0].first.x(),measurements[0].first.y()),5); //totally guessing this depth....
        Rot3 other_initialGuessOrientation = cameras[0].pose().rotation();
        Pose3 other_initialGuessPose = Pose3(other_initialGuessOrientation, other_initialGuessCenter);
        Point2 initialGuessAxes(2,1); //another complete guess....need to use back project for better guess
        //Pose3 initialGuessPose = cameras[0].pose(); //(initialGuessOrientation, initialGuessCenter);
        //initialGuessPose.print("CAMERA POSE");
        cout << "X" << measurements[0].first.x() << "Y" << measurements[0].first.y()<< endl;
        //other_initialGuessPose.print("PROJECTED MSMT POSE");
        MserObject initialGuess(other_initialGuessPose, initialGuessAxes);
        Values result = expressionsOptimizationRealWorld(initialGuess, measurements, cameras);
        MserObject returnedObject = result.at<MserObject>(Symbol('o',0));
        gtsam::traits<MserObject>::Print(returnedObject);
        objects.push_back(returnedObject);
        gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
        colors.push_back(trackColor);
    }
    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair(objects,colors);
    return pair;
}

//Write video images with superimposed MSER measurements to disk in order to test/verify
void testPrintSuperimposedMeasurementImages(){
    std::vector<MserTrack> tracks = getMserTracksFromCSV();
    std::vector<Pose3> poses = getPosesFromBAL();
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
            cv::Point center = cv::Point(msmt.first.x(),msmt.first.y());
            cv::Size axes = cv::Size(msmt.second.x(),msmt.second.y());
            double angle = msmt.first.theta()*180/M_PI; //opencv wants angles in degrees
            cv::Scalar color = cv::Scalar(tracks[t].colorB,tracks[t].colorG,tracks[t].colorR);
            int thickness = 5;
            int startAngle = 0;
            int endAngle = 360;//draw entire ellipse
            cv::ellipse(allVideoFrames[frameNum],center,axes,angle,startAngle,endAngle,color,thickness);
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

//Draw all of the poses that we get from visual odometry
void testDisplayPoses(){
    std::vector<Pose3> poses = getPosesFromBAL();
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
    drawMserObjects(dummyObjects,colors);
}

//Helper function for displaying camera pose axes in same window as inferred MSER Objects. Adds dummy objects and colors to ends of provided vectors. You then call drawMserObjects() on the resulting vectors.
void addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(std::vector<MserObject>& objects, std::vector<Vector3>& colors){
    std::vector<Pose3> poses = getPosesFromBAL();
    Vector3 color(0,0,0);
    for (int p = 0; p < poses.size(); p++){
        //poses[p].print();
        Point2 axes = Point2(0,0); //Hack: Draw "invisible" ellipse. Only axes will show.
        MserObject dummyObject = MserObject(poses[p],axes);
        int frameNumber = p+1;
        objects.push_back(dummyObject);
        colors.push_back(color);
    }
}

int main() {
    //testAllVisualization();
    //testAllGeometry();
    testAllMeasurementFunction();
    //testPrintSuperimposedMeasurementImages();
    //testDisplayPoses();
    //Display camera poses using dummy MserObjects

    std::vector<MserTrack> tracks = getMserTracksFromCSV();
    std::vector<Pose3> poses = getPosesFromBAL();
    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks, poses);
    //drawMserObjects(pair.first,pair.second);

    //std::vector<MserObject> cameraPoseDummyObjects;
    //std::vector<Vector3> cameraPoseDummyColors;
    addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(pair.first,pair.second);
    //pair.first.insert(pair.first.end(),cameraPoseDummyObjects.begin(),cameraPoseDummyObjects.end());
    //pair.second.insert(pair.second.end(),cameraPoseDummyColors.begin(),cameraPoseDummyColors.end());
    drawMserObjects(pair.first,pair.second);
    return 0;
}





