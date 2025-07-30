#include "mjpeg_cam/MjpegCam.hpp"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mjpeg_cam::MjpegCam, nodelet::Nodelet);

namespace mjpeg_cam
{

void clamp(int &val, int min, int max)
{
    if (val < min)
        val = min;
    if (val > max)
        val = max;
}

void MjpegCam::onInit()
{
    sequence = 0;
    ros::NodeHandle &nodeHandle = getPrivateNodeHandle();//getNodeHandle();

    readParameters(nodeHandle);
    imagePub_ = nodeHandle.advertise<sensor_msgs::CompressedImage>("image_raw/compressed", 1);
    imageRawPub_ = nodeHandle.advertise<sensor_msgs::Image>("image_raw", 1);

    cinfoManager_ = new camera_info_manager::CameraInfoManager(nodeHandle, camera_name, camera_info_url);

    cameraInfoPub_ = nodeHandle.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    cam = new UsbCamera(device_name, width, height);

    try {
        setCameraParams();
    }
    catch (const char * e){
        std::cout << e << std::endl;
    }
    
   
    //cb = boost::bind(&MjpegCam::dynamic_reconfigure_cb, _1, _2);
    server_.setCallback(cb);

    timer_ = nodeHandle.createTimer(ros::Duration(1.0/framerate), boost::bind(&MjpegCam::spin, this, _1));


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
void MjpegCam::spin(const ros::TimerEvent& event)
{
    if (!readAndPublishImage())
        ROS_WARN("Could not publish image");
}

void MjpegCam::readParameters(ros::NodeHandle &nodeHandle)
{
    nodeHandle.param("device_name", device_name, std::string("/dev/video0"));
    nodeHandle.param("camera_name", camera_name, std::string("usb_cam"));
    nodeHandle.param("camera_info_url", camera_info_url, std::string(""));
    nodeHandle.param("width", width, 640);
    nodeHandle.param("width", width, 640);
    nodeHandle.param("height", height, 480);
    nodeHandle.param("framerate", framerate, 30);
    nodeHandle.param("publish_image_raw", publish_image_raw, false);

    nodeHandle.param("exposure", exposure, 128);
    nodeHandle.param("autoexposure", autoexposure, true);
    nodeHandle.param("brightness", brightness, 128);
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

void MjpegCam::dynamic_reconfigure_cb(mjpeg_cam::mjpeg_camConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: \nExposure: %d \nBrightness: %d \nAutoexposure: %s",
             config.exposure,
             config.brightness,
             config.autoexposure?"True":"False");
    this->setDynamicParams(config.exposure, config.brightness, config.autoexposure);
}

} /* namespace */
