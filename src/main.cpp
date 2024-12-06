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

    monoA->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
    monoB->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);
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

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Sync together all 4 cameras that are connected to the OAK-FFC-4P:
    // Create an instance of DeviceConfig
    dai::Device::Config c = pipeline.getDeviceConfig();
    // Configure GPIO[6] to be OUTPUT and set it to HIGH
    c.board.gpio[6] = dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::Direction::OUTPUT, dai::BoardConfig::GPIO::Level::HIGH);

    // Initialize control queues
    auto controlQueue = device.getInputQueue(controlIn->getStreamName());

    // Output queues will be used to get the frames
    auto qA = device.getOutputQueue(xoutA->getStreamName(), 1, false);
    auto qB = device.getOutputQueue(xoutB->getStreamName(), 1, false);
    auto qC = device.getOutputQueue(xoutC->getStreamName(), 1, false);
    auto qD = device.getOutputQueue(xoutD->getStreamName(), 1, false);

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

        if(false) {
            printf("Auto Exposure enabled\n");
            dai::CameraControl ctrl;
            ctrl.setAutoExposureEnable();
            controlQueue->send(ctrl);
        } else if(false) {
            int exp_time, sens_iso;
            printf("Setting manual exposure, time: %d, iso: %d\n", exp_time, sens_iso);
            dai::CameraControl ctrl;
            ctrl.setManualExposure(exp_time, sens_iso);
            controlQueue->send(ctrl);
        }
    }
}
