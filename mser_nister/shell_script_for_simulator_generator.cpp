#include <fstream>
#include <stdio.h>
using namespace std;
int main(){
	ofstream myfile;
	myfile.open("nister_mser_pipeline.sh");

	myfile << "#!/bin/bash \n# Nister Shell Script \ncd Build \nmake clean \nmake nister";
	for (int i = 0; i <= 30; i++){
		char input[50];
		char output[50];
		sprintf(input, "../../images_simulator/1%08d.jpg",i);
		sprintf(output, " ../../images_output/1%08d.jpg",i);
		myfile << "\n./nister " << input << output;
	}
	myfile.close();
	return 0;
}