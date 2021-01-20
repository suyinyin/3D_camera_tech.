#include "Objectdection.h"

// constructor
Objectdection::Objectdection(string& name_files, string& modelConfiguration, string& modelWeights){
     confThreshold = 0.6; // Confidence threshold
     nmsThreshold = 0.4;  // Non-maximum suppression threshold
     inpWidth = 416;  // Width of network's input image
     inpHeight = 416; // Height of network's input image

    // Load names of classes
    this->classesFile = name_files;
    ifstream ifs(classesFile.c_str());
    string line;
    while (getline(ifs, line)) classes.push_back(line);
    // Give the configuration and weight files for the model
    this->modelConfiguration = modelConfiguration;
    this->modelWeights = modelWeights;
    net = readNetFromDarknet(modelConfiguration, modelWeights);
    net.setPreferableBackend(DNN_BACKEND_OPENCV);
    net.setPreferableTarget(DNN_TARGET_CPU);
}


// copy constructor
Objectdection::Objectdection(const Objectdection& that) {
    confThreshold = that.confThreshold;
    nmsThreshold = that.nmsThreshold;
        inpWidth = that.inpWidth;
        inpHeight = that.inpHeight;
        classes = that.classes;
    modelConfiguration = that.modelConfiguration;
    modelWeights = that.modelWeights;
    net = that.net;
}

// assignment constructor
Objectdection& Objectdection::operator=(const Objectdection& that) {
    if (this != &that) {
        confThreshold = that.confThreshold;
        nmsThreshold = that.nmsThreshold;
        inpWidth = that.inpWidth;
        inpHeight = that.inpHeight;
        classes = that.classes;
        modelConfiguration = that.modelConfiguration;
        modelWeights = that.modelWeights;
        net = that.net;
    }
    return *this;
}

// destructor
Objectdection::~Objectdection() {
    // release memeory
}


// input image
void Objectdection::image_input(const std::string& image_path_input, const std::string& type, VideoCapture& cap) {
    int type_index = 0;
    if (type == "image")
        type_index = 0;
    else if (type == "video")
        type_index = 1;
    else if (type == "camera")
        type_index = 2;

    switch (type_index)
    {
        // open an image
        case 0: 
        {
            std::string image_path = image_path_input;
            ifstream ifile(image_path);
            if (!ifile) throw("error");
            cap.open(image_path);
            image_path.replace(image_path.end() - 4, image_path.end(), "_out.jpg");
            outputFile = image_path;
            break;
        }
        // open a video
        case 1:
        {
            // Open the video file
            std::string image_path = image_path_input;
            ifstream ifile(image_path);
            if (!ifile) throw("error");
            cap.open(image_path);
            image_path.replace(image_path.end() - 4, image_path.end(), "_out.avi");
            outputFile = image_path;
            break;
        }
        case 2:
        {
            cap.open(0);
            std::string image_path = image_path_input;
            image_path.replace(image_path.end() - 4, image_path.end(), "_out.avi");
            outputFile = image_path;
            break;
        }
        default: {
            cout << "Could not open the input image/video stream" << endl;
            break;
        } 
    }
}


// convert frame to 4D blob
void Objectdection::imageTo4D(cv::Mat& frame) {
    this->frame = frame;
    // Create a 4D blob from a frame.
    blobFromImage(frame, blob, 1 / 255.0, cv::Size(inpWidth, inpHeight), Scalar(0, 0, 0), true, false);

}

// Get the names of the output layers
vector<String> Objectdection::getOutputsNames()
{
    static vector<String> names;
    if (names.empty())
    {
        //Get the indices of the output layers, i.e. the layers with unconnected outputs
        vector<int> outLayers = net.getUnconnectedOutLayers();

        //get the names of all the layers in the network
        vector<String> layersNames = net.getLayerNames();

        // Get the names of the output layers in names
        names.resize(outLayers.size());
        for (size_t i = 0; i < outLayers.size(); ++i)
            names[i] = layersNames[outLayers[i] - 1];
    }
    return names;
}


// imput image to net
void Objectdection::compute_image() {
    //Sets the input to the network
    net.setInput(blob);
    // Runs the forward pass to get output of the output layers
    outs.clear();
    vector<String> s = getOutputsNames();
    std::cout << s[0] << std::endl;
    auto start = system_clock::now();
    net.forward(outs, getOutputsNames());
    auto end = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout << "duration: " << duration.count() << endl;
    

}


// Remove the bounding boxes with low confidence using non-maxima suppression
void Objectdection::postprocess(vector<Rect>& boxes_update)
{
    vector<int> classIds;
    vector<float> confidences;
    vector<Rect> boxes;
    for (size_t i = 0; i < outs.size(); ++i)
    {
        // Scan through all the bounding boxes output from the network and keep only the
        // ones with high confidence scores. Assign the box's class label as the class
        // with the highest score for the box.
        float* data = (float*)outs[i].data;
        for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        {
            Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            Point classIdPoint;
            double confidence;
            // Get the value and location of the maximum score
            minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            if (confidence > confThreshold)
            {
                int centerX = (int)(data[0] * frame.cols);
                int centerY = (int)(data[1] * frame.rows);
                int width = (int)(data[2] * frame.cols);
                int height = (int)(data[3] * frame.rows);
                int left = centerX - width / 2;
                int top = centerY - height / 2;

                classIds.push_back(classIdPoint.x);
                confidences.push_back((float)confidence);
                boxes.push_back(Rect(left, top, width, height));
            }
        }
    }

    // Perform non maximum suppression to eliminate redundant overlapping boxes with
    // lower confidences
    vector<int> indices;
    NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
    for (size_t i = 0; i < indices.size(); ++i)
    {
        int idx = indices[i];
        Rect box = boxes[idx];
        drawPred(classIds[idx], confidences[idx], box.x, box.y,box.x + box.width, box.y + box.height, frame);
        boxes_update.push_back(Rect(box.x, box.y, box.width, box.height));
    }
    namedWindow("out", WINDOW_NORMAL);
    while (cv::waitKey(30) != 27) {
        cv::imshow("out", frame);
    }

}

// Draw the predicted bounding box
void Objectdection::drawPred(int classId, float conf, int left, int top, int right, int bottom, Mat& frame)
{
    //Draw a rectangle displaying the bounding box
    rectangle(frame, Point(left, top), Point(right, bottom), Scalar(255, 178, 50), 3);

    //Get the label for the class name and its confidence
    string label = format("%.2f", conf);
    if (!classes.empty())
    {
        CV_Assert(classId < (int)classes.size());
        label = classes[classId] + ":" + label;
    }

    //Display the label at the top of the bounding box
    int baseLine;
    Size labelSize = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);
    top = max(top, labelSize.height);
    rectangle(frame, Point(left, top - round(1.5 * labelSize.height)), Point(left + round(1.5 * labelSize.width), top + baseLine), Scalar(255, 255, 255), FILLED);
    putText(frame, label, Point(left, top), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(0, 0, 0), 1);
}




