#ifndef MJPEG_CAM_MJPEGCAM_H
#define MJPEG_CAM_MJPEGCAM_H

#pragma once

#include "mjpeg_cam/MjpegCam.hpp"
#include "mjpeg_cam/UsbCamera.hpp"

// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>

#include <dynamic_reconfigure/server.h>
#include <mjpeg_cam/mjpeg_camConfig.h>

namespace mjpeg_cam
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class MjpegCam: public nodelet::Nodelet
{
public:
    /*!
     * Constructor.
     */
    MjpegCam() = default;

    /*!
     * Destructor.
     */
    ~MjpegCam();

    /*!
     * Enters an event loop to read the camera
     */
    void spin(const ros::TimerEvent& event);

    /*!
     * Set parameters that can be dynamically reconfigured
     */
    void setDynamicParams(int exposure, int brightness, bool autoexposure);

    /*!
     * Dynamic reconfigure callback
     */
    void dynamic_reconfigure_cb(mjpeg_cam::mjpeg_camConfig &config, uint32_t level);


private:
    /*!
     * Nodelet initialization.
     */

    virtual void onInit();

    /*!
     * Reads a single frame from the camera and publish to topic.
     */
    bool readAndPublishImage();

    /*!
     * Reads ROS parameters.
     */
    void readParameters(ros::NodeHandle &nodeHandle);

    /*!
     * Set camera parameters
     */
    bool setCameraParams();

    //! ROS timer
    ros::Timer timer_;

    //! ROS Image Publisher
    ros::Publisher imagePub_;
    ros::Publisher imageRawPub_;

    //! Camera info publisher
    ros::Publisher cameraInfoPub_;
    camera_info_manager::CameraInfoManager* cinfoManager_;

    //! Dynamic reconfigure server
    dynamic_reconfigure::Server<mjpeg_cam::mjpeg_camConfig> server_;
    dynamic_reconfigure::Server<mjpeg_cam::mjpeg_camConfig>::CallbackType cb;

    //! Camera Object
    UsbCamera *cam;
    unsigned int sequence;

    // Parameters
    std::string device_name;
    std::string camera_name;
    std::string camera_info_url;
    int width;
    int height;
    int framerate;
    bool publish_image_raw;
    int exposure;
    int brightness;
    bool autoexposure;
};

} /* namespace */

#endif // MJPEG_CAM_MJPEGCAM_H

