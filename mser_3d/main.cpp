 
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h> 
#include <gtsam/slam/GeneralSFMFactor.h>
#include <iostream>

using namespace std;
using namespace gtsam;

int main() {
    SfM_data mydata;
    string filename = "/home/alex/mser/datasets/fpv_bal_280_nf2.txt";
    readBAL(filename, mydata);
    cout << boost::format("read %1% tracks on %2% cameras\n") % mydata.number_tracks() % mydata.number_cameras();
    BOOST_FOREACH(const SfM_Camera& camera, mydata.cameras){
        const Pose3 pose = camera.pose();
        pose.print("Camera pose:\n");
    }
    return 0;
}