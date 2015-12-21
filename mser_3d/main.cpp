#include "testVisualization.h"
#include "testGeometry.h"
#include "testMeasurementFunction.h"
#include "Visualizer.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>


typedef std::vector<double> record_t;
typedef std::vector<record_t> data_t;
struct mserTrack{
    int colorR; //colors to be assigned to final object
    int colorG;
    int colorB;
    std::vector<int> frameNumbers;
    std::vector<mserMeasurement> measurements;
};

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

std::vector<mserTrack> getMserTracksFromCSV(){
    std::ifstream infile("/home/alex/mser/mser_2d/mserMeasurements.csv");
    data_t data;
    infile >> data;
    infile.close();
    std::vector<mserTrack> tracks;
    for (int r = 1; r < data.size(); r++){ //start at row 1 because row 0 does not contain data
        mserTrack track;
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
            mserMeasurement measurement(pose,axes);
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

std::pair<std::vector<mserObject>,std::vector<Vector3>> inferObjectsFromRealMserMeasurements(std::vector<mserTrack>& tracks, std::vector<Pose3>& VOposes){
    std::vector<mserObject> objects;
    std::vector<Vector3> colors;
    Cal3_S2::shared_ptr K(new Cal3_S2(50.0, 50.0, 0.1, 640/2, 360/2));
    for (int t = 0; t < 60; t++){
        std::vector<mserMeasurement> measurements = tracks[t].measurements;
        std::cout << "#" << t << " has " << measurements.size() << " measurements" << std::endl;
        if ((t != 17) && (t != 33)){
            std::vector<SimpleCamera> cameras;
            for (int m = 0; m < measurements.size(); m++){
                SimpleCamera camera(VOposes[tracks[t].frameNumbers[m]],*K);
                cameras.push_back(camera);
            }

            //Make initial guess
            Point3 initialGuessCenter = cameras[0].backproject_from_camera(Point2(measurements[0].first.x(),measurements[0].first.y()),5); //totally guessing this depth....
            Rot3 initialGuessOrientation = cameras[0].pose().rotation();
            Point2 initialGuessAxes(2,1); //another complete guess....need to use back project for better guess
            Pose3 initialGuessPose(initialGuessOrientation, initialGuessCenter);
            mserObject initialGuess(initialGuessPose, initialGuessAxes);

            Values result = expressionsOptimizationRealWorld(initialGuess, measurements, cameras);
            mserObject returnedObject = result.at<mserObject>(Symbol('o',0));
            gtsam::traits<mserObject>::Print(returnedObject);
            objects.push_back(returnedObject);
            gtsam::Vector3 trackColor(tracks[t].colorR,tracks[t].colorG,tracks[t].colorB);
            colors.push_back(trackColor);
        }
    }
    std::pair<std::vector<mserObject>,std::vector<Vector3>> pair(objects,colors);
    return pair;
}

int main() {
    testAllVisualization();
    testAllGeometry();
    testAllMeasurementFunction();
    std::vector<mserTrack> tracks = getMserTracksFromCSV();
    std::vector<Pose3> poses = getPosesFromBAL();
    std::pair<std::vector<mserObject>,std::vector<Vector3>> pair = inferObjectsFromRealMserMeasurements(tracks, poses);
    drawMserObjects(pair.first,pair.second);
    return 0;
}





