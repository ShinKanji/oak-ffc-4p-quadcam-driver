#include <opencv2/opencv.hpp>
#include "depthai/depthai.hpp"
#include <iostream>
#include <chrono>

static int clamp(int num, int v0, int v1) {
    return std::max(v0, std::min(num, v1));
}

int main() {

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoA = pipeline.create<dai::node::MonoCamera>();
    auto monoB = pipeline.create<dai::node::MonoCamera>();
    auto monoC = pipeline.create<dai::node::MonoCamera>();
    auto monoD = pipeline.create<dai::node::MonoCamera>();

    auto xoutA = pipeline.create<dai::node::XLinkOut>();
    auto xoutB = pipeline.create<dai::node::XLinkOut>();
    auto xoutC = pipeline.create<dai::node::XLinkOut>();
    auto xoutD = pipeline.create<dai::node::XLinkOut>();

    auto controlIn = pipeline.create<dai::node::XLinkIn>();

    controlIn->setStreamName("control");
    
    monoA->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    monoB->setBoardSocket(dai::CameraBoardSocket::CAM_B);
    monoC->setBoardSocket(dai::CameraBoardSocket::CAM_C);
    monoD->setBoardSocket(dai::CameraBoardSocket::CAM_D);

    monoA->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);
    monoB->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
    monoC->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
    monoD->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);

    xoutA->setStreamName("camera_A");
    xoutB->setStreamName("camera_B");
    xoutC->setStreamName("camera_C");
    xoutD->setStreamName("camera_D");

    // Properties
    monoA->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    monoB->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    monoC->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    monoD->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);

    // Linking
    monoA->out.link(xoutA->input);
    monoB->out.link(xoutB->input);
    monoC->out.link(xoutC->input);
    monoD->out.link(xoutD->input);

    controlIn->out.link(monoA->inputControl);
    controlIn->out.link(monoB->inputControl);
    controlIn->out.link(monoC->inputControl);
    controlIn->out.link(monoD->inputControl);

    // Sync together all 4 cameras that are connected to the OAK-FFC-4P:
    // Create an instance of BoardConfig
    dai::BoardConfig boardConfig;

    // Create a gpio map to store GPIO configurations
    std::unordered_map<int8_t, dai::BoardConfig::GPIO> gpio;

    // Configure GPIO[6] to be OUTPUT and set it to HIGH
    gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::OUTPUT, dai::BoardConfig::GPIO::HIGH);

    // Assign GPIO configurations to the boardConfig
    boardConfig.gpio = gpio;
    pipeline.setBoardConfig(boardConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Initialize control queues
    auto controlQueue = device.getInputQueue(controlIn->getStreamName());

    // Output queues will be used to get the grayscale frames
    auto qA = device.getOutputQueue(xoutA->getStreamName(), 1, false);
    auto qB = device.getOutputQueue(xoutB->getStreamName(), 1, false);
    auto qC = device.getOutputQueue(xoutC->getStreamName(), 1, false);
    auto qD = device.getOutputQueue(xoutD->getStreamName(), 1, false);

    int frameCount = 0;
    double fps = 0.0;
    auto start = std::chrono::high_resolution_clock::now();


    // Defaults and limits for manual focus/exposure controls
    int exp_time = 20000;
    int exp_min = 1;
    int exp_max = 33000;

    int sens_iso = 100;
    int sens_min = 100;
    int sens_max = 1600;
    dai::CameraControl init_ctrl;
    init_ctrl.setAntiBandingMode(dai::CameraControl::AntiBandingMode::AUTO);
    controlQueue->send(init_ctrl);

    while(true){

        auto inA = qA->get<dai::ImgFrame>();
        auto inB = qB->get<dai::ImgFrame>();
        auto inC = qC->get<dai::ImgFrame>();
        auto inD = qD->get<dai::ImgFrame>();

        cv::Mat frame1, frame2, frame3, frame4, stitched_frame;
        cv::resize(inA->getCvFrame(), frame1, cv::Size(320, 200));
        cv::resize(inB->getCvFrame(), frame2, cv::Size(320, 200));
        cv::resize(inC->getCvFrame(), frame3, cv::Size(320, 200));
        cv::resize(inD->getCvFrame(), frame4, cv::Size(320, 200));
        cv::hconcat(frame1, frame2, stitched_frame);
        cv::hconcat(stitched_frame, frame3, stitched_frame);
        cv::hconcat(stitched_frame, frame4, stitched_frame);

        cv::imshow("Stitched Frame", stitched_frame);

        frameCount++;
        auto end = std::chrono::high_resolution_clock::now();
        double elapsedSeconds = std::chrono::duration<double>(end - start).count();
        if (elapsedSeconds >= 1.0) {
            fps = frameCount / elapsedSeconds;
            frameCount = 0;
            start = end;
        }
        
        int key = cv::waitKey(1);
        if(key == 'e') {
            printf("Autoexposure enable\n");
            dai::CameraControl ctrl;
            ctrl.setAutoExposureEnable();
            controlQueue->send(ctrl);
        } else if(key == 'i' || key == 'o' || key == 'k' || key == 'l') {
            if(key == 'i') exp_time -= 500;
            if(key == 'o') exp_time += 500;
            if(key == 'k') sens_iso -= 100;
            if(key == 'l') sens_iso += 100;
            exp_time = clamp(exp_time, exp_min, exp_max);
            sens_iso = clamp(sens_iso, sens_min, sens_max);
            printf("Setting manual exposure, time: %d, iso: %d\n", exp_time, sens_iso);
            dai::CameraControl ctrl;
            ctrl.setManualExposure(exp_time, sens_iso);
            controlQueue->send(ctrl);
        } else if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
}
