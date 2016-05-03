//
// Created by alex on 12/8/15.
//
#include "visualization.h"

using namespace gtsam;

GLuint loadBMP_custom(const char * imagepath){

    printf("Reading image %s\n", imagepath);

    // Data read from the header of the BMP file
    unsigned char header[54];
    unsigned int dataPos;
    unsigned int imageSize;
    unsigned int width, height;
    // Actual RGB data
    unsigned char * data;

    // Open the file
    FILE * file = fopen(imagepath,"rb");
    if (!file)							    {printf("%s could not be opened. Are you in the right directory ? Don't forget to read the FAQ !\n", imagepath); getchar(); return 0;}

    // Read the header, i.e. the 54 first bytes

    // If less than 54 bytes are read, problem
    if ( fread(header, 1, 54, file)!=54 ){
        printf("Not a correct BMP file: less than 54 bytes \n");
        return 0;
    }
    // A BMP files always begins with "BM"
    if ( header[0]!='B' || header[1]!='M' ){
        printf("Not a correct BMP file: doesn't start with BM\n");
        cout << "header[0] " << header[0] << " header[1] " << header[1] << endl;
        return 0;
    }
    // Make sure this is a 24bpp file
    if ( *(int*)&(header[0x1E])!=0  )         {printf("Not a correct BMP file\n"); cout << " MUST USE GIMP TO MAKE 24 BIT BMP *(int*)&(header[0x1E]) " << *(int*)&(header[0x1E]) << endl;     return 0;}
    if ( *(int*)&(header[0x1C])!=24 )         {printf("Not a correct BMP file\n"); cout << " MUST USE GIMP TO MAKE 24 BIT BMP *(int*)&(header[0x1C]) " << *(int*)&(header[0x1C]) << endl;    return 0;}

    // Read the information about the image
    dataPos    = *(int*)&(header[0x0A]);
    imageSize  = *(int*)&(header[0x22]);
    width      = *(int*)&(header[0x12]);
    height     = *(int*)&(header[0x16]);

    // Some BMP files are misformatted, guess missing information
    if (imageSize==0)    imageSize=width*height*3; // 3 : one byte for each Red, Green and Blue component
    if (dataPos==0)      dataPos=54; // The BMP header is done that way

    // Create a buffer
    data = new unsigned char [imageSize];

    // Read the actual data from the file into the buffer
    fread(data,1,imageSize,file);

    // Everything is in memory now, the file wan be closed
    fclose (file);

    // Create one OpenGL texture
    GLuint textureID;
    glGenTextures(1, &textureID);

    // "Bind" the newly created texture : all future texture functions will modify this texture
    glBindTexture(GL_TEXTURE_2D, textureID);

    // Give the image to OpenGL
    glTexImage2D(GL_TEXTURE_2D, 0,GL_RGB, width, height, 0, GL_BGR, GL_UNSIGNED_BYTE, data);

    // OpenGL has now copied the data. Free our own version
    delete [] data;

    // Poor filtering, or ...
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);

    // ... nice trilinear filtering.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glGenerateMipmap(GL_TEXTURE_2D);

    // Return the ID of the texture we just created
    return textureID;
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
            //rayStart.print();
            //rayEnd.print();

            std::pair<Point3,Point3> ray;
            ray.first = rayStart;
            ray.second = rayEnd;
            rayTracingPairs.push_back(ray);
        }
    }
    return rayTracingPairs;
}

int drawMserObjects(const InputManager& input, const std::vector<Pose3>& inputCameraPoses, const std::vector<MserObject>& inputObjects, const std::vector<Vector3>& inputColors, const std::vector<std::pair<Point3,Point3>>& rays){
    //make local copies to modify
    std::vector<MserObject> objects = inputObjects;
    std::vector<Vector3> colors = inputColors;
    //make easy representations of cmera pose axes
    addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(inputCameraPoses, objects, colors);

    cerr << "VISUALIZATION: Starting to draw MSER Objects" << endl;
    cerr << "VISUALIZATION: Objects size: " << objects.size() << endl;
    cerr << "VISUALIZATION: Colors size: " << colors.size() << endl;

    GLFWwindow *window;
    // Initialise GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    } else {
        cerr << "VISUALIZATION: GLFW initialized." << endl;
    }

    //Set up window.
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1024, 768, "VISUALIZATION MSER 3D", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "VISUALIZATION: Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n");
        glfwTerminate();
        return -1;
    } else {
        cerr << "VISUALIZATION: GLFW Window Opened." << endl;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW - OpenGL Extension Wrangler Library
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "VISUALIZATION: Failed to initialize GLEW\n");
        return -1;
    } else {
        cerr << "VISUALIZATION: GLEW initialized." << endl;
    }
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below

    //Set up random float generator for generating random colors
    random_device rd;
    mt19937 eng(0);
    uniform_real_distribution<float> distr(0, 1);

    //More OpenGL setup
    //glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); // Random color background
    glClearColor(1.0, 0.9, 0.9, 0.9); //Set color of background
    glEnable(GL_DEPTH_TEST); // Enable depth test
    glDepthFunc(GL_LESS);  // Accept fragment if it closer to the camera than the former one
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    GLuint programID = LoadShaders(input.VertexShaderPath().c_str(),
                                   input.FragmentShaderPath().c_str()); // Create and compile our GLSL program from the shaders
    GLuint MatrixID = glGetUniformLocation(programID, "MVP"); // Get a handle for our "MVP" uniform
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f,
                                            100.0f); // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units

    cerr << "VISUALIZATION: Shaders loaded, GLM pespective set, background color set." << endl;

    GLuint vertexbuffer; //create vertex buffer outside loop so it can be purged outside loop
    GLuint colorbuffer; //create color buffer outside loop so it can be purged outside loop

    // Camera matrix
    glm::mat4 View = glm::lookAt(
            glm::vec3(0, 10, 40), //camera position
            glm::vec3(0, 0, 0), // and looks at the target
            glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
    );
    glm::mat4 Model = glm::mat4(1.0f); // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 MVP = Projection * View *
                    Model; // Our ModelViewProjection : multiplication of our 3 matrices. Remember, matrix multiplication is the other way around

    //36 vertices makes for a decent-looking ellipse and keeps memory usage low.
    int numVerticesPerEllipse = 36;
    //(objects.size ellipses)(numVerticesPerEllipse triangles / ellipse)(3 points / triangle)(3 doubles / point) + (objects.size axes groups)(3 lines / axis group)(6 doubles / line) + (1 world axis group)(3 lines / world axis group)(6 doubles / world axis line) + (rays.size # rays)*(1 line / ray)*(3 doubles / line)
    long int vertexDataSize = objects.size()*numVerticesPerEllipse*3*3 + objects.size()*3*6 + 1*3*6 + rays.size()*1*6;

    cerr << "VISUALIZATION: Vertex data size = " << vertexDataSize << endl;

    GLfloat g_vertex_buffer_data[vertexDataSize];
    GLfloat g_color_buffer_data[vertexDataSize];

    //Draw 3D ellipses representing objects
    cerr << "VISUALIZATION: Computing ellipse vertices and colors for objects." << endl;
    for (int o = 0; o < objects.size(); o++) {
        float cubeR, cubeG, cubeB;
        if (colors.size() == objects.size()){
            cubeR = (float) colors[o][0] / 255;
            cubeG = (float) colors[o][1] / 255;
            cubeB = (float) colors[o][2] / 255;
        } else {
            cubeR = distr(eng);
            cubeG = distr(eng);
            cubeB = distr(eng);
        }
        for (int i = 0 + o*numVerticesPerEllipse*9; i < numVerticesPerEllipse*9 + o*numVerticesPerEllipse*9; i+=9) {
            // Object vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
            float rad_angle = (i/9) * 360/numVerticesPerEllipse * M_PI / 180;
            float next_rad_angle = (i/9 + 1) * 360/numVerticesPerEllipse * M_PI / 180;
            float ellipse_x_radius = objects[o].second.x();// / 40;
            float ellipse_y_radius = objects[o].second.y();// / 40;
            //cout << ellipse_x_radius << " " << ellipse_y_radius << endl;

            double scaleFactor = 1; //use this to make things further apart, closer together for easier visualization
            Point3 centerInWorldFrame(objects[o].first.x()/scaleFactor,objects[o].first.y()/scaleFactor,objects[o].first.z());
            Point3 axisTipInObjectFrame(cos(rad_angle) * ellipse_x_radius,sin(rad_angle) * ellipse_y_radius,0);
            Point3 nextAxisTipInObjectFrame(cos(next_rad_angle) * ellipse_x_radius,sin(next_rad_angle) * ellipse_y_radius,0);
            Rot3 objectRot = objects[o].first.rotation();
            Point3 axisTipInWorldFrame = objectRot*axisTipInObjectFrame + centerInWorldFrame;
            Point3 nextAxisTipInWorldFrame = objectRot*nextAxisTipInObjectFrame + centerInWorldFrame;

            g_vertex_buffer_data[i] = centerInWorldFrame.x();//objects[o].first.x();
            g_vertex_buffer_data[i+1] = centerInWorldFrame.y();//objects[o].first.y();
            g_vertex_buffer_data[i+2] = centerInWorldFrame.z();//objects[o].first.z();
            g_vertex_buffer_data[i+3] = axisTipInWorldFrame.x();//objects[o].first.x() + cos(rad_angle) * ellipse_x_radius;
            g_vertex_buffer_data[i+4] = axisTipInWorldFrame.y();//objects[o].first.y() + sin(rad_angle) * ellipse_y_radius;
            g_vertex_buffer_data[i+5] = axisTipInWorldFrame.z();//objects[o].first.z();
            g_vertex_buffer_data[i+6] = nextAxisTipInWorldFrame.x();//objects[o].first.x() + cos(next_rad_angle) * ellipse_x_radius;
            g_vertex_buffer_data[i+7] = nextAxisTipInWorldFrame.y();//objects[o].first.y() + sin(next_rad_angle) * ellipse_y_radius;
            g_vertex_buffer_data[i+8] = nextAxisTipInWorldFrame.z();//objects[o].first.z();

            //Set color of object
            g_color_buffer_data[i] = cubeR;
            g_color_buffer_data[i+1] = cubeG;
            g_color_buffer_data[i+2] = cubeB;
            g_color_buffer_data[i+3] = cubeR;
            g_color_buffer_data[i+4] = cubeG;
            g_color_buffer_data[i+5] = cubeB;
            g_color_buffer_data[i+6] = cubeR;
            g_color_buffer_data[i+7] = cubeG;
            g_color_buffer_data[i+8] = cubeB;
        }
    }

    //Draw axes lines in objects' frames. RGB correspond to XYZ.
    int objectAxisLength = 1;
    int vertexDataNum = objects.size()*numVerticesPerEllipse*3*3; //start where variable i left off in the previous loop
    cerr << "VISUALIZATION: Computing axes vertices and colors for objects." << endl;
    for (int o = 0; o < objects.size(); o++) {
        //Object frame:
        Point3 objectCenterInObjectFrame(0,0,0);
        Point3 objectXAxisTipInObjectFrame(objectAxisLength,0,0);
        Point3 objectYAxisTipInObjectFrame(0,objectAxisLength,0);
        Point3 objectZAxisTipInObjectFrame(0,0,objectAxisLength);
        //World frame:
        MserObject object = objects[o];
        Point3 objectCenterInWorldFrame = objectCenterInObjectFrame + object.first.translation();
        Point3 objectXAxisTipInWorldFrame = object.first.rotation()*objectXAxisTipInObjectFrame + objectCenterInWorldFrame;
        Point3 objectYAxisTipInWorldFrame = object.first.rotation()*objectYAxisTipInObjectFrame + objectCenterInWorldFrame;
        Point3 objectZAxisTipInWorldFrame = object.first.rotation()*objectZAxisTipInObjectFrame + objectCenterInWorldFrame;

        //First line: X axis
        //First point:
        g_vertex_buffer_data[vertexDataNum + 0] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 1] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 2] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 3] = objectXAxisTipInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 4] = objectXAxisTipInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 5] = objectXAxisTipInWorldFrame.z();
        //Second line: Y axis
        //First point:
        g_vertex_buffer_data[vertexDataNum + 6] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 7] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 8] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 9] = objectYAxisTipInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 10] = objectYAxisTipInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 11] = objectYAxisTipInWorldFrame.z();
        //Third line: Z axis
        //First point:
        g_vertex_buffer_data[vertexDataNum + 12] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 13] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 14] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 15] = objectZAxisTipInWorldFrame.x();
        g_vertex_buffer_data[vertexDataNum + 16] = objectZAxisTipInWorldFrame.y();
        g_vertex_buffer_data[vertexDataNum + 17] = objectZAxisTipInWorldFrame.z();

        //First line: X axis
        //First point:
        g_color_buffer_data[vertexDataNum + 0] = 1.0f;
        g_color_buffer_data[vertexDataNum + 1] = 0.0f;
        g_color_buffer_data[vertexDataNum + 2] = 0.0f;
        //Second point:
        g_color_buffer_data[vertexDataNum + 3] = 1.0f;
        g_color_buffer_data[vertexDataNum + 4] = 0.0f;
        g_color_buffer_data[vertexDataNum + 5] = 0.0f;

        //Second line: Y axis
        //First point:
        g_color_buffer_data[vertexDataNum + 6] = 0.0f;
        g_color_buffer_data[vertexDataNum + 7] = 1.0f;
        g_color_buffer_data[vertexDataNum + 8] = 0.0f;
        //Second point
        g_color_buffer_data[vertexDataNum + 9] = 0.0f;
        g_color_buffer_data[vertexDataNum + 10] = 1.0f;
        g_color_buffer_data[vertexDataNum + 11] = 0.0f;

        //Third line: Z axis
        //First point:
        g_color_buffer_data[vertexDataNum + 12] = 0.0f;
        g_color_buffer_data[vertexDataNum + 13] = 0.0f;
        g_color_buffer_data[vertexDataNum + 14] = 1.0f;
        //Second point:
        g_color_buffer_data[vertexDataNum + 15] = 0.0f;
        g_color_buffer_data[vertexDataNum + 16] = 0.0f;
        g_color_buffer_data[vertexDataNum + 17] = 1.0f;

        vertexDataNum += 18; //increment data position
    }

    cerr << "VISUALIZATION: Computing rays to object centroids." << endl;
    //Picking up where ellipses left off
    vertexDataNum = objects.size()*numVerticesPerEllipse*3*3 + objects.size()*3*6;

    for (int r = 0; r < rays.size(); r++){
        Point3 rayStart = rays[r].first;
        Point3 rayEnd = rays[r].second;

        //Ray points
        //First point:
        g_vertex_buffer_data[vertexDataNum + 0] = rayStart.x();
        g_vertex_buffer_data[vertexDataNum + 1] = rayStart.y();
        g_vertex_buffer_data[vertexDataNum + 2] = rayStart.z();
        //Second point:
        g_vertex_buffer_data[vertexDataNum + 3] = rayEnd.x();
        g_vertex_buffer_data[vertexDataNum + 4] = rayEnd.y();
        g_vertex_buffer_data[vertexDataNum + 5] = rayEnd.z();

        //Ray Colors
        //First point:
        g_color_buffer_data[vertexDataNum + 0] = 0.0f;
        g_color_buffer_data[vertexDataNum + 1] = 0.0f;
        g_color_buffer_data[vertexDataNum + 2] = 0.0f;
        //Second point:
        g_color_buffer_data[vertexDataNum + 3] = 0.0f;
        g_color_buffer_data[vertexDataNum + 4] = 0.0f;
        g_color_buffer_data[vertexDataNum + 5] = 0.0f;
        vertexDataNum += 6;
    }

    cerr << "VISUALIZATION: Computing vertices and colors for world axes." << endl;
    //Draw lines for world axes. RGB correspond to XYZ
    int worldAxisLength = 10;
    Point3 worldCenter(0,0,0);
    Point3 xAxisTip(worldAxisLength,0,0);
    Point3 yAxisTip(0,worldAxisLength,0);
    Point3 zAxisTip(0,0,worldAxisLength);

    //First line: X axis
    //First point:
    g_vertex_buffer_data[vertexDataSize - 18] = worldCenter.x();
    g_vertex_buffer_data[vertexDataSize - 17] = worldCenter.y();
    g_vertex_buffer_data[vertexDataSize - 16] = worldCenter.z();
    //Second point:
    g_vertex_buffer_data[vertexDataSize - 15] = xAxisTip.x();
    g_vertex_buffer_data[vertexDataSize - 14] = xAxisTip.y();
    g_vertex_buffer_data[vertexDataSize - 13] = xAxisTip.z();
    //Second line: Y axis
    //First point:
    g_vertex_buffer_data[vertexDataSize - 12] = worldCenter.x();
    g_vertex_buffer_data[vertexDataSize - 11] = worldCenter.y();
    g_vertex_buffer_data[vertexDataSize - 10] = worldCenter.z();
    //Second point:
    //First point:
    g_vertex_buffer_data[vertexDataSize - 9] = yAxisTip.x();
    g_vertex_buffer_data[vertexDataSize - 8] = yAxisTip.y();
    g_vertex_buffer_data[vertexDataSize - 7] = yAxisTip.z();
    //Third line: Z axis
    //First point
    g_vertex_buffer_data[vertexDataSize - 6] = worldCenter.x();
    g_vertex_buffer_data[vertexDataSize - 5] = worldCenter.y();
    g_vertex_buffer_data[vertexDataSize - 4] = worldCenter.z();
    //Second point:
    g_vertex_buffer_data[vertexDataSize - 3] = zAxisTip.x();
    g_vertex_buffer_data[vertexDataSize - 2] = zAxisTip.y();
    g_vertex_buffer_data[vertexDataSize - 1] = zAxisTip.z();

    //First line: X axis
    //First point:
    g_color_buffer_data[vertexDataSize - 18] = 1.0f;
    g_color_buffer_data[vertexDataSize - 17] = 0.0f;
    g_color_buffer_data[vertexDataSize - 16] = 0.0f;
    //Second point:
    g_color_buffer_data[vertexDataSize - 15] = 1.0f;
    g_color_buffer_data[vertexDataSize - 14] = 0.0f;
    g_color_buffer_data[vertexDataSize - 13] = 0.0f;

    //Second line: Y axis
    //First point:
    g_color_buffer_data[vertexDataSize - 12] = 0.0f;
    g_color_buffer_data[vertexDataSize - 11] = 1.0f;
    g_color_buffer_data[vertexDataSize - 10] = 0.0f;
    //Second point:
    g_color_buffer_data[vertexDataSize - 9] = 0.0f;
    g_color_buffer_data[vertexDataSize - 8] = 1.0f;
    g_color_buffer_data[vertexDataSize - 7] = 0.0f;

    //Third line: Z axis
    //First point:
    g_color_buffer_data[vertexDataSize - 6] = 0.0f;
    g_color_buffer_data[vertexDataSize - 5] = 0.0f;
    g_color_buffer_data[vertexDataSize - 4] = 1.0f;
    //Second point:
    g_color_buffer_data[vertexDataSize - 3] = 0.0f;
    g_color_buffer_data[vertexDataSize - 2] = 0.0f;
    g_color_buffer_data[vertexDataSize - 1] = 1.0f;
    // End lines for world axes

    cerr << "VISUALIZATION: Vertex computation done. Placing vertices and colors into color buffers..." << endl;
    //Place vertex info into a buffer
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    //Place color info into a buffer
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);

    //MAIN WINDOW ANIMATION LOOP: while loop start
    do{
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        glUseProgram(programID); // Use our shader
        // Compute the MVP matrix from keyboard and mouse input
        computeMatricesFromInputs(window);
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE,&MVP[0][0]); // Send our transformation to the currently bound shader in the "MVP" uniform
        // 1st attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void *) 0            // array buffer offset
        );
        // 2nd attribute buffer : colors
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                3,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void *) 0                          // array buffer offset
        );
        //MOST IMPORTANT DRAWING CODE IS HERE! We take what we stored in the vertex buffer and draw it.
        //Note that here we count VERTICES (e.g. 3 points per triangle or 2 points per line) NOT DOUBLES
        glDrawArrays(GL_TRIANGLES, 0, numVerticesPerEllipse * 3 * objects.size()); // Draw the numVerticesPerEllipse triangles per ellipse. Each triangle has 3 points. Hence numVerticesPerEllipse*3*objects.size(). Start at index 0.
        glDrawArrays(GL_LINES,numVerticesPerEllipse * 3 * objects.size(), numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2); //Draw object.size # of axes groups. Each axes group has 3 lines. Each line has 2 points.
        glDrawArrays(GL_LINES, numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2, numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2); //Draw world axes group. Group has 3 lines. Each line has 2 points.
        glDrawArrays(GL_LINES,numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2,numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2 + rays.size()*2); //Draw ray lines. Each ray line has 2 points.
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glfwSwapBuffers(window); // Swap buffers
        glfwPollEvents();
    }while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
            glfwWindowShouldClose(window) == 0 );
}

//Helper function for displaying camera pose axes in same window as inferred MSER Objects. Adds dummy objects and colors to ends of provided vectors. You then call drawMserObjects() on the resulting vectors.
void addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(const std::vector<Pose3>& cameraPoses, std::vector<MserObject>& objects, std::vector<Vector3>& colors){
    Vector3 color(0,0,0);
    for (int p = 0; p < cameraPoses.size(); p++){
        Point2 axes = Point2(0,0); //Hack: Draw "invisible" ellipse. Only axes will show.
        MserObject dummyObject = MserObject(cameraPoses[p],axes);
        int frameNumber = p+1;
        objects.push_back(dummyObject);
        colors.push_back(color);
    }
}

int drawTexturedMserObjects(const InputManager& input, const std::vector<Pose3>& inputCameraPoses, const std::vector<MserObject>& inputObjects, const std::vector<Vector3>& inputColors, const std::vector<std::pair<Point3,Point3>>& rays){
    //make local copies to modify
    std::vector<MserObject> objects = inputObjects;
    int originalObjectsSize = objects.size();
    std::vector<Vector3> colors = inputColors;
    //make easy representations of cmera pose axes
    addDummyObjectsAndColorsForDisplayingCameraAlongsideMserObjects(inputCameraPoses, objects, colors);

    cerr << "VISUALIZATION: Starting to draw MSER Objects" << endl;
    cerr << "VISUALIZATION: Objects size: " << objects.size() << endl;
    cerr << "VISUALIZATION: Colors size: " << colors.size() << endl;

    GLFWwindow *window;
    // Initialise GLFW
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return -1;
    } else {
        cerr << "VISUALIZATION: GLFW initialized." << endl;
    }

    //Set up window.
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(1024, 768, "VISUALIZATION MSER 3D", NULL, NULL);
    if (window == NULL) {
        fprintf(stderr, "VISUALIZATION: Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible.\n");
        glfwTerminate();
        return -1;
    } else {
        cerr << "VISUALIZATION: GLFW Window Opened." << endl;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW - OpenGL Extension Wrangler Library
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "VISUALIZATION: Failed to initialize GLEW\n");
        return -1;
    } else {
        cerr << "VISUALIZATION: GLEW initialized." << endl;
    }
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE); // Ensure we can capture the escape key being pressed below

    //Set up random float generator for generating random colors
    random_device rd;
    mt19937 eng(0);
    uniform_real_distribution<float> distr(0, 1);

    //More OpenGL setup
    //glClearColor(distr(eng), distr(eng), distr(eng), distr(eng)); // Random color background
    glClearColor(1.0, 0.9, 0.9, 0.9); //Set color of background
    glEnable(GL_DEPTH_TEST); // Enable depth test
    glDepthFunc(GL_LESS);  // Accept fragment if it closer to the camera than the former one
    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);
    GLuint programID = LoadShaders(input.VertexShaderPath().c_str(),
                                   input.FragmentShaderPath().c_str()); // Create and compile our GLSL program from the shaders
    GLuint MatrixID = glGetUniformLocation(programID, "MVP"); // Get a handle for our "MVP" uniform
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f,
                                            100.0f); // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units

    cerr << "VISUALIZATION: Shaders loaded, GLM pespective set, background color set." << endl;

    GLuint vertexbuffer; //create vertex buffer outside loop so it can be purged outside loop
    GLuint colorbuffer; //create color buffer outside loop so it can be purged outside loop
    GLuint uvbuffer;
    // Camera matrix
    glm::mat4 View = glm::lookAt(
            glm::vec3(0, 10, 40), //camera position
            glm::vec3(0, 0, 0), // and looks at the target
            glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
    );
    glm::mat4 Model = glm::mat4(1.0f); // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 MVP = Projection * View *
                    Model; // Our ModelViewProjection : multiplication of our 3 matrices. Remember, matrix multiplication is the other way around

    //36 vertices makes for a decent-looking ellipse and keeps memory usage low.
    int numVerticesPerEllipse = 36;
    //(objects.size ellipses)(numVerticesPerEllipse triangles / ellipse)(3 points / triangle)(3 doubles / point) + (objects.size axes groups)(3 lines / axis group)(6 doubles / line) + (1 world axis group)(3 lines / world axis group)(6 doubles / world axis line) + (rays.size # rays)*(1 line / ray)*(3 doubles / line)
    int textureSize = originalObjectsSize*2*3*3;
    long int vertexDataSize = textureSize +  objects.size()*numVerticesPerEllipse*3*3 + objects.size()*3*6 + 1*3*6 + rays.size()*1*6;

    cerr << "VISUALIZATION: Vertex data size = " << vertexDataSize << endl;

    GLfloat g_vertex_buffer_data[vertexDataSize];
    GLfloat g_color_buffer_data[vertexDataSize];
    GLfloat g_uv_buffer_data[originalObjectsSize*2*3*2];

    cerr << "Visualization: loading textures." << endl;
    GLuint Texture = loadBMP_custom("../../view.bmp");
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    for (int o = 0; o < originalObjectsSize; o++){
        float majAxisLength = objects[o].second.x();// / 40;
        float minAxisLength = objects[o].second.y();// / 40;

        Point3 centerInWorldFrame(objects[o].first.x(),objects[o].first.y(),objects[o].first.z());
        Point3 upperLeftInObjectFrame(-1*majAxisLength/2,majAxisLength/2,0);
        Point3 upperRightInObjectFrame(majAxisLength/2,majAxisLength/2,0);
        Point3 lowerLeftInObjectFrame(-1*majAxisLength/2,-1*majAxisLength/2,0);
        Point3 lowerRightInObjectFrame(majAxisLength/2,-1*majAxisLength/2,0);

        Rot3 objectRot = objects[o].first.rotation();
        Point3 upperLeftInWorldFrame = objectRot * upperLeftInObjectFrame + centerInWorldFrame;
        Point3 upperRightInWorldFrame = objectRot * upperRightInObjectFrame + centerInWorldFrame;
        Point3 lowerLeftInWorldFrame = objectRot * lowerLeftInObjectFrame + centerInWorldFrame;
        Point3 lowerRightInWorldFrame = objectRot * lowerRightInObjectFrame + centerInWorldFrame;

        //Draw square
        g_vertex_buffer_data[18*o] = lowerLeftInWorldFrame.x();
        g_vertex_buffer_data[18*o+1] = lowerLeftInWorldFrame.y();
        g_vertex_buffer_data[18*o+2] = lowerLeftInWorldFrame.z();
        g_vertex_buffer_data[18*o+3] = lowerRightInWorldFrame.x();
        g_vertex_buffer_data[18*o+4] = lowerRightInWorldFrame.y();
        g_vertex_buffer_data[18*o+5] = lowerRightInWorldFrame.z();
        g_vertex_buffer_data[18*o+6] = upperRightInWorldFrame.x();
        g_vertex_buffer_data[18*o+7] = upperRightInWorldFrame.y();
        g_vertex_buffer_data[18*o+8] = upperRightInWorldFrame.z();

        g_vertex_buffer_data[18*o+9] = upperRightInWorldFrame.x();
        g_vertex_buffer_data[18*o+10] = upperRightInWorldFrame.y();
        g_vertex_buffer_data[18*o+11] = upperRightInWorldFrame.z();
        g_vertex_buffer_data[18*o+12] = upperLeftInWorldFrame.x();
        g_vertex_buffer_data[18*o+13] = upperLeftInWorldFrame.y();
        g_vertex_buffer_data[18*o+14] = upperLeftInWorldFrame.z();
        g_vertex_buffer_data[18*o+15] = lowerLeftInWorldFrame.x();
        g_vertex_buffer_data[18*o+16] = lowerLeftInWorldFrame.y();
        g_vertex_buffer_data[18*o+17] = lowerLeftInWorldFrame.z();

        //texture
        g_uv_buffer_data[12*o] = 0;
        g_uv_buffer_data[12*o+1] = 0;
        g_uv_buffer_data[12*o+2]= 1;
        g_uv_buffer_data[12*o+3] = 0;
        g_uv_buffer_data[12*o+4] = 1;
        g_uv_buffer_data[12*o+5] = 1;
        g_uv_buffer_data[12*o+6] = 1;
        g_uv_buffer_data[12*o+7] = 1;
        g_uv_buffer_data[12*o+8] = 0;
        g_uv_buffer_data[12*o+9] = 1;
        g_uv_buffer_data[12*o+10] = 0;
        g_uv_buffer_data[12*o+11] = 0;

        //colors
        g_color_buffer_data[18*o] = 0;
        g_color_buffer_data[18*o+1] = 0;
        g_color_buffer_data[18*o+2] = 0;
        g_color_buffer_data[18*o+3] = 0;
        g_color_buffer_data[18*o+4] = 0;
        g_color_buffer_data[18*o+5] = 0;
        g_color_buffer_data[18*o+6] = 0;
        g_color_buffer_data[18*o+7] = 0;
        g_color_buffer_data[18*o+8] = 0;
        g_color_buffer_data[18*o+9] = 0;
        g_color_buffer_data[18*o+10] = 0;
        g_color_buffer_data[18*o+11] = 0;
        g_color_buffer_data[18*o+12] = 0;
        g_color_buffer_data[18*o+13] = 0;
        g_color_buffer_data[18*o+14] = 0;
        g_color_buffer_data[18*o+15] = 0;
        g_color_buffer_data[18*o+16] = 0;
        g_color_buffer_data[18*o+17] = 0;
    }


    //Draw 3D ellipses representing objects
    cerr << "VISUALIZATION: Computing ellipse vertices and colors for objects." << endl;
    for (int o = 0; o < objects.size(); o++) {
        cout << "object center " << objects[o].first.translation() << endl;
        float cubeR, cubeG, cubeB;
        if (colors.size() == objects.size()){
            cubeR = (float) colors[o][0] / 255;
            cubeG = (float) colors[o][1] / 255;
            cubeB = (float) colors[o][2] / 255;
        } else {
            cubeR = distr(eng);
            cubeG = distr(eng);
            cubeB = distr(eng);
        }

        for (int i = 0 + o*numVerticesPerEllipse*9; i < numVerticesPerEllipse*9 + o*numVerticesPerEllipse*9; i+=9) {
            // Object vertices. Three consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
            float rad_angle = (i/9) * 360/numVerticesPerEllipse * M_PI / 180;
            float next_rad_angle = (i/9 + 1) * 360/numVerticesPerEllipse * M_PI / 180;
            float ellipse_x_radius = 0;//objects[o].second.x();// / 40;
            float ellipse_y_radius = 0;//objects[o].second.y();// / 40;
            //cout << ellipse_x_radius << " " << ellipse_y_radius << endl;

            double scaleFactor = 1; //use this to make things further apart, closer together for easier visualization
            Point3 centerInWorldFrame(objects[o].first.x()/scaleFactor,objects[o].first.y()/scaleFactor,objects[o].first.z());
            Point3 axisTipInObjectFrame(cos(rad_angle) * ellipse_x_radius,sin(rad_angle) * ellipse_y_radius,0);
            Point3 nextAxisTipInObjectFrame(cos(next_rad_angle) * ellipse_x_radius,sin(next_rad_angle) * ellipse_y_radius,0);
            Rot3 objectRot = objects[o].first.rotation();
            Point3 axisTipInWorldFrame = objectRot*axisTipInObjectFrame + centerInWorldFrame;
            Point3 nextAxisTipInWorldFrame = objectRot*nextAxisTipInObjectFrame + centerInWorldFrame;

            g_vertex_buffer_data[textureSize + i] = centerInWorldFrame.x();//objects[o].first.x();
            g_vertex_buffer_data[textureSize +  i+1] = centerInWorldFrame.y();//objects[o].first.y();
            g_vertex_buffer_data[textureSize +  i+2] = centerInWorldFrame.z();//objects[o].first.z();
            g_vertex_buffer_data[textureSize +  i+3] = axisTipInWorldFrame.x();//objects[o].first.x() + cos(rad_angle) * ellipse_x_radius;
            g_vertex_buffer_data[textureSize +  i+4] = axisTipInWorldFrame.y();//objects[o].first.y() + sin(rad_angle) * ellipse_y_radius;
            g_vertex_buffer_data[textureSize +  i+5] = axisTipInWorldFrame.z();//objects[o].first.z();
            g_vertex_buffer_data[textureSize +  i+6] = nextAxisTipInWorldFrame.x();//objects[o].first.x() + cos(next_rad_angle) * ellipse_x_radius;
            g_vertex_buffer_data[textureSize +  i+7] = nextAxisTipInWorldFrame.y();//objects[o].first.y() + sin(next_rad_angle) * ellipse_y_radius;
            g_vertex_buffer_data[textureSize +  i+8] = nextAxisTipInWorldFrame.z();//objects[o].first.z();

            //Set color of object
            g_color_buffer_data[textureSize +  i] = cubeR;
            g_color_buffer_data[textureSize +  i+1] = cubeG;
            g_color_buffer_data[textureSize +  i+2] = cubeB;
            g_color_buffer_data[textureSize +  i+3] = cubeR;
            g_color_buffer_data[textureSize +  i+4] = cubeG;
            g_color_buffer_data[textureSize +  i+5] = cubeB;
            g_color_buffer_data[textureSize +  i+6] = cubeR;
            g_color_buffer_data[textureSize +  i+7] = cubeG;
            g_color_buffer_data[textureSize +  i+8] = cubeB;
        }
    }


    //Draw axes lines in objects' frames. RGB correspond to XYZ.
    int objectAxisLength = 1;
    int vertexDataNum = textureSize + objects.size()*numVerticesPerEllipse*3*3; //start where variable i left off in the previous loop
    cerr << "VISUALIZATION: Computing axes vertices and colors for objects." << endl;
    for (int o = 0; o < objects.size(); o++) {
        //Object frame:
        Point3 objectCenterInObjectFrame(0,0,0);
        Point3 objectXAxisTipInObjectFrame(objectAxisLength,0,0);
        Point3 objectYAxisTipInObjectFrame(0,objectAxisLength,0);
        Point3 objectZAxisTipInObjectFrame(0,0,objectAxisLength);
        //World frame:
        MserObject object = objects[o];
        Point3 objectCenterInWorldFrame = objectCenterInObjectFrame + object.first.translation();
        Point3 objectXAxisTipInWorldFrame = object.first.rotation()*objectXAxisTipInObjectFrame + objectCenterInWorldFrame;
        Point3 objectYAxisTipInWorldFrame = object.first.rotation()*objectYAxisTipInObjectFrame + objectCenterInWorldFrame;
        Point3 objectZAxisTipInWorldFrame = object.first.rotation()*objectZAxisTipInObjectFrame + objectCenterInWorldFrame;

        //First line: X axis
        //First point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 0] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 1] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 2] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 3] = objectXAxisTipInWorldFrame.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 4] = objectXAxisTipInWorldFrame.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 5] = objectXAxisTipInWorldFrame.z();
        //Second line: Y axis
        //First point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 6] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 7] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 8] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 9] = objectYAxisTipInWorldFrame.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 10] = objectYAxisTipInWorldFrame.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 11] = objectYAxisTipInWorldFrame.z();
        //Third line: Z axis
        //First point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 12] = objectCenterInWorldFrame.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 13] = objectCenterInWorldFrame.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 14] = objectCenterInWorldFrame.z();
        //Second point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 15] = objectZAxisTipInWorldFrame.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 16] = objectZAxisTipInWorldFrame.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 17] = objectZAxisTipInWorldFrame.z();

        //First line: X axis
        //First point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 0] = 1.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 1] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 2] = 0.0f;
        //Second point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 3] = 1.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 4] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 5] = 0.0f;

        //Second line: Y axis
        //First point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 6] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 7] = 1.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 8] = 0.0f;
        //Second point
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 9] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 10] = 1.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 11] = 0.0f;

        //Third line: Z axis
        //First point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 12] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 13] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 14] = 1.0f;
        //Second point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 15] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 16] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 17] = 1.0f;

        vertexDataNum += 18; //increment data position
    }

    cerr << "VISUALIZATION: Computing rays to object centroids." << endl;
    //Picking up where ellipses left off
    vertexDataNum = textureSize + objects.size()*numVerticesPerEllipse*3*3 + objects.size()*3*6;

    for (int r = 0; r < rays.size(); r++){
        Point3 rayStart = rays[r].first;
        Point3 rayEnd = rays[r].second;

        //Ray points
        //First point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 0] = rayStart.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 1] = rayStart.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 2] = rayStart.z();
        //Second point:
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 3] = rayEnd.x();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 4] = rayEnd.y();
        g_vertex_buffer_data[/*textureSize + */ vertexDataNum + 5] = rayEnd.z();

        //Ray Colors
        //First point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 0] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 1] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 2] = 0.0f;
        //Second point:
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 3] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 4] = 0.0f;
        g_color_buffer_data[/*textureSize + */ vertexDataNum + 5] = 0.0f;
        vertexDataNum += 6;
    }

    cerr << "VISUALIZATION: Computing vertices and colors for world axes." << endl;
    //Draw lines for world axes. RGB correspond to XYZ
    int worldAxisLength = 10;
    Point3 worldCenter(0,0,0);
    Point3 xAxisTip(worldAxisLength,0,0);
    Point3 yAxisTip(0,worldAxisLength,0);
    Point3 zAxisTip(0,0,worldAxisLength);

    //First line: X axis
    //First point:
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 18] = worldCenter.x();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 17] = worldCenter.y();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 16] = worldCenter.z();
    //Second point:
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 15] = xAxisTip.x();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 14] = xAxisTip.y();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 13] = xAxisTip.z();
    //Second line: Y axis
    //First point:
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 12] = worldCenter.x();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 11] = worldCenter.y();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 10] = worldCenter.z();
    //Second point:
    //First point:
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 9] = yAxisTip.x();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 8] = yAxisTip.y();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 7] = yAxisTip.z();
    //Third line: Z axis
    //First point
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 6] = worldCenter.x();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 5] = worldCenter.y();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 4] = worldCenter.z();
    //Second point:
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 3] = zAxisTip.x();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 2] = zAxisTip.y();
    g_vertex_buffer_data[/*textureSize + */ vertexDataSize - 1] = zAxisTip.z();

    //First line: X axis
    //First point:
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 18] = 1.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 17] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 16] = 0.0f;
    //Second point:
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 15] = 1.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 14] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 13] = 0.0f;

    //Second line: Y axis
    //First point:
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 12] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 11] = 1.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 10] = 0.0f;
    //Second point:
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 9] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 8] = 1.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 7] = 0.0f;

    //Third line: Z axis
    //First point:
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 6] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 5] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 4] = 1.0f;
    //Second point:
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 3] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 2] = 0.0f;
    g_color_buffer_data[/*textureSize + */ vertexDataSize - 1] = 1.0f;
    // End lines for world axes

    cerr << "VISUALIZATION: Vertex computation done. Placing vertices and colors into color buffers..." << endl;
    //Place vertex info into a buffer
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

    /*
    //Place color info into a buffer
    glGenBuffers(1, &colorbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_color_buffer_data), g_color_buffer_data, GL_STATIC_DRAW);
    */

    //MAIN WINDOW ANIMATION LOOP: while loop start
    do{
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear the screen
        glUseProgram(programID); // Use our shader
        // Compute the MVP matrix from keyboard and mouse input
        computeMatricesFromInputs(window);
        glm::mat4 ProjectionMatrix = getProjectionMatrix();
        glm::mat4 ViewMatrix = getViewMatrix();
        glm::mat4 ModelMatrix = glm::mat4(1.0);
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE,&MVP[0][0]); // Send our transformation to the currently bound shader in the "MVP" uniform


        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1st attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void *) 0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                2,                                // size : U+V => 2
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );

        // 2nd attribute buffer : colors
        /*
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
        glVertexAttribPointer(
                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                3,                                // size
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void *) 0                          // array buffer offset
        );
         */

        //MOST IMPORTANT DRAWING CODE IS HERE! We take what we stored in the vertex buffer and draw it.
        //Note that here we count VERTICES (e.g. 3 points per triangle or 2 points per line) NOT DOUBLES
        //glDrawArrays(GL_TRIANGLES,0,originalObjectsSize*2*3);
        glDrawArrays(GL_TRIANGLES, 0, originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size()); // Draw the numVerticesPerEllipse triangles per ellipse. Each triangle has 3 points. Hence numVerticesPerEllipse*3*objects.size(). Start at index 0.
        //glDrawArrays(GL_LINES,originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size(), originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2); //Draw object.size # of axes groups. Each axes group has 3 lines. Each line has 2 points.
        //glDrawArrays(GL_LINES, originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2, originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2); //Draw world axes group. Group has 3 lines. Each line has 2 points.
        //glDrawArrays(GL_LINES,originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2, originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2 + rays.size()*2); //Draw ray lines. Each ray line has 2 points.
        glDrawArrays(GL_LINES,originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size(), originalObjectsSize*2*3 + numVerticesPerEllipse * 3 * objects.size() + objects.size()*3*2 + 3*2 + rays.size()*2);

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glfwSwapBuffers(window); // Swap buffers
        glfwPollEvents();
    }while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
            glfwWindowShouldClose(window) == 0 );
}

void showTextures(){
    GLFWwindow* window;
    // Initialise GLFW
    if( !glfwInit() )
    {
        fprintf( stderr, "Failed to initialize GLFW\n" );

    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( 1024, 768, "Tutorial 05 - Textured Cube", NULL, NULL);
    if( window == NULL ){
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n" );
        glfwTerminate();

    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        fprintf(stderr, "Failed to initialize GLEW\n");

    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    GLuint VertexArrayID;
    glGenVertexArrays(1, &VertexArrayID);
    glBindVertexArray(VertexArrayID);

    // Create and compile our GLSL program from the shaders
    GLuint programID = LoadShaders( "../../TransformVertexShaderTutorial.vertexshader", "../../TextureFragmentShader.fragmentshader" );

    // Get a handle for our "MVP" uniform
    GLuint MatrixID = glGetUniformLocation(programID, "MVP");

    // Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
    glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.0f);
    // Camera matrix
    glm::mat4 View       = glm::lookAt(
            glm::vec3(4,3,3), // Camera is at (4,3,3), in World Space
            glm::vec3(0,0,0), // and looks at the origin
            glm::vec3(0,1,0)  // Head is up (set to 0,-1,0 to look upside-down)
    );
    // Model matrix : an identity matrix (model will be at the origin)
    glm::mat4 Model      = glm::mat4(1.0f);
    // Our ModelViewProjection : multiplication of our 3 matrices
    glm::mat4 MVP        = Projection * View * Model; // Remember, matrix multiplication is the other way around

    // Load the texture using any two methods
    GLuint Texture = loadBMP_custom("../../view.bmp");
    //GLuint Texture = loadDDS("uvtemplate.DDS");

    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureID  = glGetUniformLocation(programID, "myTextureSampler");

    // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
    // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
    static const GLfloat g_vertex_buffer_data[] = {
            0,0,0,
            1,0,0,
            1,1,0,
            1,1,0,
            0,1,0,
            0,0,0,
    };

    // Two UV coordinatesfor each vertex. They were created withe Blender.
    static const GLfloat g_uv_buffer_data[] = {
            0,0,
            1,0,
            1,1,
            1,1,
            0,1,
            0,0
    };

    GLuint vertexbuffer;
    glGenBuffers(1, &vertexbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_vertex_buffer_data), g_vertex_buffer_data, GL_STATIC_DRAW);

    GLuint uvbuffer;
    glGenBuffers(1, &uvbuffer);
    glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
    glBufferData(GL_ARRAY_BUFFER, sizeof(g_uv_buffer_data), g_uv_buffer_data, GL_STATIC_DRAW);

    do{

        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        glUseProgram(programID);

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

        // Bind our texture in Texture Unit 0
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, Texture);
        // Set our "myTextureSampler" sampler to user Texture Unit 0
        glUniform1i(TextureID, 0);

        // 1rst attribute buffer : vertices
        glEnableVertexAttribArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
        glVertexAttribPointer(
                0,                  // attribute. No particular reason for 0, but must match the layout in the shader.
                3,                  // size
                GL_FLOAT,           // type
                GL_FALSE,           // normalized?
                0,                  // stride
                (void*)0            // array buffer offset
        );

        // 2nd attribute buffer : UVs
        glEnableVertexAttribArray(1);
        glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
        glVertexAttribPointer(
                1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
                2,                                // size : U+V => 2
                GL_FLOAT,                         // type
                GL_FALSE,                         // normalized?
                0,                                // stride
                (void*)0                          // array buffer offset
        );

        // Draw the triangle !
        glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles

        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while( glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS &&
           glfwWindowShouldClose(window) == 0 );

    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbuffer);
    glDeleteBuffers(1, &uvbuffer);
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureID);
    glDeleteVertexArrays(1, &VertexArrayID);

    // Close OpenGL window and terminate GLFW
    glfwTerminate();
}

