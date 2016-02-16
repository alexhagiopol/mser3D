#include "testVisualization.h"
#include "testGeometry.h"
#include "testMeasurementFunction.h"
#include "Visualizer.h"
#include "MserTrack.h"
#include <opencv2/opencv.hpp>
#include <gtsam/geometry/triangulation.h>
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
    std::ifstream infile("/home/alex/mser/mser_2d/firstManualMeasurementGroup.csv");
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

//           objects                 colors
std::pair<std::vector<MserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurements(std::vector<MserTrack>& tracks, std::vector<Pose3>& VOposes){
    std::vector<MserObject> objects;
    std::vector<Vector3> colors;
    Cal3_S2::shared_ptr K(new Cal3_S2(857.483, 876.718, 0.1, 1280/2, 720/2)); //gopro camera calibration from http://www.theeminentcodfish.com/gopro-calibration/
    for (int t = 0; t < tracks.size(); t++){
        std::vector<MserMeasurement> measurements = tracks[t].measurements;
        std::cout << "#" << t << " has " << measurements.size() << " measurements" << std::endl;
        std::vector<SimpleCamera> cameras;
        std::vector<Point2> centroidMeasurements;
        std::vector<Point2> majorAxisMeasurements;
        std::vector<Point2> minorAxisMeasurements;
        for (int m = 0; m < measurements.size(); m++){
            MserMeasurement measurement = tracks[t].measurements[m];
            //make cameras with poses from Jing's Visual Odometry
            Pose3 pose = VOposes[tracks[t].frameNumbers[m]];
            SimpleCamera camera(pose,*K);
            cameras.push_back(camera);

            //extract contents of measurement
            double theta = measurement.first.theta();
            double majAxis  = measurement.second.x();
            double minAxis = measurement.second.y();
            double ctrX = measurement.first.translation().x();
            double ctrY = measurement.first.translation().y();

            //make vector of 2D centroid measurements
            centroidMeasurements.push_back(measurement.first.translation());
            measurement.first.print("MSER MEASUREMENT POSE \n");
            measurement.second.print("MSER MEASUREMENT AXES \n");

            //vector of 2D major axis measurements
            double majX = ctrX + majAxis*cos(theta);
            double majY = ctrY + majAxis*sin(theta);
            Point2 majorAxisMeasurement(majX,majY);
            majorAxisMeasurements.push_back(majorAxisMeasurement);
            majorAxisMeasurement.print("MAJOR AXIS MEASUREMENT \n");

            //vector of 2D minor axis measurements
            double minX = ctrX + minAxis*cos(theta + M_PI/2);
            double minY = ctrY + minAxis*sin(theta + M_PI/2);
            Point2 minorAxisMeasurement(minX,minY);
            minorAxisMeasurements.push_back(minorAxisMeasurement);
            minorAxisMeasurement.print("MINOR AXIS MEASUREMENT \n");
            cout << "THETA \n" << theta << endl;
        }

        /* //Crude initial guess.
        double depthGuess = 2.0; //make this better later....
        Point3 initialGuessCenter = cameras[0].backproject(Point2(measurements[0].first.x(),measurements[0].first.y()),depthGuess); //Guessed depth.
        Rot3 initialGuessOrientation = cameras[0].pose().rotation();
        Pose3 initialGuessPose = Pose3(initialGuessOrientation, initialGuessCenter);
        */

        //Axes initial guess via back projection of first camera
        /*
        Point2 majorAxisTipInImgFrame = measurements[0].first.translation() + Point2(measurements[0].second.x(),0);
        Point2 minorAxisTipInImgFrame = measurements[0].first.translation() + Point2(0,measurements[0].second.y());
        Point3 majorAxisTipInWorldFrame  = cameras[0].backproject(majorAxisTipInImgFrame,depthGuess);
        Point3 minorAxisTipInWorldFrame  = cameras[0].backproject(minorAxisTipInImgFrame,depthGuess);
        double majorAxisLengthInitialGuess = majorAxisTipInWorldFrame.distance(initialGuessCenter);
        double minorAxisLengthInitialGuess = minorAxisTipInWorldFrame.distance(initialGuessCenter);
        Point2 initialGuessAxes(majorAxisLengthInitialGuess,minorAxisLengthInitialGuess);
        */

        //Initial guess via triangulation.h methods
        Point3 initialGuessCentroid = gtsam::triangulatePoint3(cameras,centroidMeasurements,1e-9,true);
        Point3 initialGuessMajAxisPoint = gtsam::triangulatePoint3(cameras,majorAxisMeasurements,1e-9,true);
        Point3 initialGuessMinAxisPoint = gtsam::triangulatePoint3(cameras,minorAxisMeasurements,1e-9,true);

        initialGuessCentroid.print("CENTROID GUESS \n");
        initialGuessMajAxisPoint.print("MAJ AXIS POINT GUESS \n");
        initialGuessMinAxisPoint.print("MIN AXIS POINT GUESS \n");

        /*
        Point3 C_majA = initialGuessMajAxisPoint - initialGuessCentroid;
        Point3 C_minA = initialGuessMinAxisPoint - initialGuessCentroid;
        Point3 normalVector = C_majA.cross(C_minA);
        Point3 xVector(1,0,0);
        Point3 yVector(0,1,0);
        Point3 zVector(0,0,1);
        */



        Rot3 initialGuessOrientation = cameras[0].pose().rotation();
        Pose3 initialGuessPose = Pose3(initialGuessOrientation, initialGuessCentroid);
        double majAxisLengthEstimate = initialGuessCentroid.distance(initialGuessMajAxisPoint);
        double minAxisLengthEstimate = initialGuessCentroid.distance(initialGuessMinAxisPoint);
        Point2 initialGuessAxes = Point2(majAxisLengthEstimate,minAxisLengthEstimate);
        initialGuessAxes.print("AXES LENGTH GUESS");


        MserObject initialGuess(initialGuessPose, initialGuessAxes);
        //objects.push_back(initialGuess);
        //colors.push_back(gtsam::Vector3(0,0,0)); ///black initial guess

        /*
        for (int iter = 20; iter <= 20; iter += 1){
            cout << "OPTIMIZING WITH " << iter << " ITERATIONS" << endl;
            Values result = expressionsOptimizationRealWorld(initialGuess, measurements, cameras, iter);
            MserObject returnedObject = result.at<MserObject>(Symbol('o',0));
            objects.push_back(returnedObject);
            //gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
            colors.push_back(gtsam::Vector3(50 + iter*20,50 + iter*20,50 + iter*20)); //make everything darker on every iteration
        }
        */

        Values result = expressionsOptimizationRealWorld(initialGuess, measurements, cameras, 30);
        MserObject returnedObject = result.at<MserObject>(Symbol('o',0));
        objects.push_back(returnedObject);
        //gtsam::traits<MserObject>::Print(returnedObject);
        //gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
        gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
        colors.push_back(trackColor);

        cout << "FINISHED OPTIMIZING TRACK #" << t << " OF " << tracks.size() << endl;
    }
    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair(objects,colors);
    return pair;
}

std::vector<std::pair<Point3,Point3>> makeRayTracingPairs(std::vector<MserTrack>& tracks, std::vector<Pose3>& VOposes){
    std::vector<std::pair<Point3,Point3>> rayTracingPairs;
    for (int t = 0; t < tracks.size(); t++){
        std::vector<MserMeasurement> measurements = tracks[t].measurements;
        for (int m = 0; m < measurements.size(); m++){
            MserMeasurement measurement = tracks[t].measurements[m];
            Point2 centroid2D = measurement.first.translation();
            //Assume a camera and calibration - later on ask caller to provide cameras in this function and in inferObjectsFromRealMserMeasurements()
            Cal3_S2::shared_ptr K(new Cal3_S2(857.483, 876.718, 0.1, 1280/2, 720/2)); //gopro camera calibration from http://www.theeminentcodfish.com/gopro-calibration/
            SimpleCamera camera(VOposes[tracks[t].frameNumbers[m]],*K);
            Point3 rayEnd = camera.backproject(centroid2D,1000);
            Point3 rayStart = VOposes[tracks[t].frameNumbers[m]].translation();
            rayStart.print();
            rayEnd.print();

            std::pair<Point3,Point3> ray;
            ray.first = rayStart;
            ray.second = rayEnd;
            rayTracingPairs.push_back(ray);
        }
    }
    return rayTracingPairs;
};

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
    Visualizer myVisualizer = Visualizer();
    myVisualizer.drawMserObjects(poses, dummyObjects, colors);
}

void test3DReconstruction(bool showRays){
    std::vector<MserTrack> tracks = getMserTracksFromCSV();
    std::vector<Pose3> allCameraPoses = getPosesFromBAL();
    //shows only relevant camera poses
    std::vector<Pose3> relevantCameraPoses;
    for (int t = 0; t < tracks.size(); t++){
        for (int f = 0; f < tracks[t].frameNumbers.size(); f++){ //the frame number corresponds to the pose index??
            relevantCameraPoses.push_back(allCameraPoses[tracks[t].frameNumbers[f]]);
        }
    }

    std::pair<std::vector<MserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks, allCameraPoses); //use all poses because this funciton expects to look through everything from getPosesFromBAL()
    std::vector<std::pair<Point3,Point3>> rays;
    if (showRays) rays = makeRayTracingPairs(tracks, allCameraPoses);
    //Draw robot axes: causes seg fault for big track datasets because we cross the vertex memory limit.
    Visualizer myVisualizer = Visualizer();
    //myVisualizer.addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(poses, pair.first,pair.second);
    cout << "# cam poses " << relevantCameraPoses.size() << endl;
    cout << "# objects " << pair.first.size() << endl;
    cout << "# colors " << pair.second.size() << endl;
    cout << "# rays " << rays.size() << endl;
    myVisualizer.drawMserObjects(relevantCameraPoses, pair.first, pair.second, rays); //only display relevant poses
}

int main() {
    //testAllVisualization();
    //testAllGeometry();
    //testAllMeasurementFunction();
    //testPrintSuperimposedMeasurementImages();
    //testDisplayPoses();
    test3DReconstruction(true);
    return 0;
}





