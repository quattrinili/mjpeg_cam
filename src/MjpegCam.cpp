#include "mjpeg_cam/MjpegCam.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

namespace mjpeg_cam
{

void clamp(int &val, int min, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
}

MjpegCam::MjpegCam(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle),
      sequence(0)
{
    readParameters();
    imagePub_ = nodeHandle_.advertise<sensor_msgs::CompressedImage>(camera_name + "/image_raw/compressed", 1);
    imageRawPub_ = nodeHandle_.advertise<sensor_msgs::Image>(camera_name + "/image_raw", 1);

    cinfoManager_ = new camera_info_manager::CameraInfoManager(nodeHandle, camera_name, camera_info_url);

    cameraInfoPub_ = nodeHandle_.advertise<sensor_msgs::CameraInfo>(camera_name + "/camera_info", 1);

    cam = new UsbCamera(device_name, width, height);

    try {
        setCameraParams();
    }
    catch (const char * e){
        std::cout << e << std::endl;
    }

    ROS_INFO("Successfully launched node.");
}

MjpegCam::~MjpegCam()
{
    delete cam;
    delete cinfoManager_;
}

bool MjpegCam::readAndPublishImage()
{
    try {
        int length;
        char *image = cam->grab_image(length);
        sensor_msgs::CompressedImage msg;
        msg.header.frame_id = camera_name;
        msg.header.seq = sequence++;
        msg.header.stamp = ros::Time::now();
        msg.format = "bgr8";
        msg.data.resize(length);
        std::copy(image, image + length, msg.data.begin());
        if (publish_image_raw) {
            cv::Mat decompressed_image = cv::imdecode(cv::Mat(msg.data), cv::IMREAD_UNCHANGED);
            sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(msg.header, msg.format, decompressed_image).toImageMsg();

            imageRawPub_.publish(image_msg);
        }
        imagePub_.publish(msg);
        //std::cout << "Image size in kB: " << length/1000 << std::endl;

        sensor_msgs::CameraInfo camera_info_msg = cinfoManager_->getCameraInfo();
        camera_info_msg.header.seq = msg.header.seq;
        camera_info_msg.header.stamp = msg.header.stamp; // Set timestamp
        cameraInfoPub_.publish(camera_info_msg);

        return true;
    }
    catch (const char *e) {
        std::cout << e << std::endl;
    }

    return false;
}
void MjpegCam::spin()
{
    ros::Rate loop_rate(framerate);
    while (nodeHandle_.ok()) {
        if (!readAndPublishImage())
            ROS_WARN("Could not publish image");

        loop_rate.sleep();
        ros::spinOnce();
    }
}

void MjpegCam::readParameters()
{
    nodeHandle_.param("device_name", device_name, std::string("/dev/video0"));
    nodeHandle_.param("camera_name", camera_name, std::string("usb_cam"));
    nodeHandle_.param("camera_info_url", camera_info_url, std::string(""));
    nodeHandle_.param("width", width, 640);
    nodeHandle_.param("width", width, 640);
    nodeHandle_.param("height", height, 480);
    nodeHandle_.param("framerate", framerate, 30);
    nodeHandle_.param("publish_image_raw", publish_image_raw, false);

    nodeHandle_.param("exposure", exposure, 128);
    nodeHandle_.param("autoexposure", autoexposure, true);
    nodeHandle_.param("brightness", brightness, 128);
}

bool MjpegCam::setCameraParams()
{
    if (cam == 0)
        return false;

    clamp(exposure, 0, 255);
    clamp(brightness, 0, 255);

    cam->set_v4l2_param("brightness", brightness);

    if (autoexposure) {
        cam->set_v4l2_param("exposure_auto", 3);
    }
    else {
        cam->set_v4l2_param("exposure_auto", 1);
        cam->set_v4l2_param("exposure_absolute", exposure);
    }


    return true;
}

void MjpegCam::setDynamicParams(int exposure, int brightness, bool autoexposure)
{
    this->exposure = exposure;
    this->brightness = brightness;
    this->autoexposure = autoexposure;
    setCameraParams();
}

} /* namespace */
