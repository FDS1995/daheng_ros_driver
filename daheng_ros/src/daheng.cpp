#include "daheng.h"



//-------------------------------------------------
/**
\brief Acquisition thread function
\param pParam       thread param, not used in this app
\return void*
*/
//-------------------------------------------------
void ProcGetImage(ros::NodeHandle &nh)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_implemented = false;

    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("camera", 1);
    header.frame_id = frame_id;
    camera_info_manager::CameraInfoManager cam_info_manager(nh, camera_name, camera_info_url);
    // Get the saved camera info if any
    cam_info_msg = cam_info_manager.getCameraInfo();
    cam_info_msg.header = header;

    while(nh.ok())
    {


        status = GXGetImage(g_device, &g_frame_data, 100);
        if(status == GX_STATUS_SUCCESS)
        {
            if(g_frame_data.nStatus == 0)
            {
                // printf("<Successful acquisition: Width: %d Height: %d>\n", g_frame_data.nWidth, g_frame_data.nHeight);
                // status = GXIsImplemented(g_device, GX_BUFFER_FRAME_INFORMATION, &is_implemented);
                // if(status == GX_STATUS_SUCCESS)
                // {
                //     if(true == is_implemented)
                //     {
                //         printf("<Frame number: %d>\n", GetCurFrameIndex());
                //     }
                // }

                //Save the raw data
                // SaveRawFile(g_frame_data.pImgBuf, g_frame_data.nWidth, g_frame_data.nHeight);

                //Convert the 8 bit raw data into the RGB data
                ProcessData(g_frame_data.pImgBuf, 
                        g_raw8_buffer, 
                        g_rgb_frame_data, 
                        g_frame_data.nWidth, 
                        g_frame_data.nHeight,
                        g_pixel_format,
                        g_color_filter);
                Mat frame(g_frame_data.nHeight, g_frame_data.nWidth, CV_8UC3, g_rgb_frame_data);
                if (pub.getNumSubscribers() > 0){
                    // Check if grabbed frame is actually filled with some content
                    if(!frame.empty()) {
                        msg = cv_bridge::CvImage(header, "rgb8", frame).toImageMsg();
                        // Create a default camera info if we didn't get a stored one on initialization
                        if (cam_info_msg.distortion_model == ""){
                            ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
                            cam_info_msg = get_default_camera_info_from_image(msg);
                            cam_info_manager.setCameraInfo(cam_info_msg);
                        }
                        // The timestamps are in sync thanks to this publisher
                        pub.publish(*msg, cam_info_msg, ros::Time::now());
                    }
                }
                //Save the RGB data
                // SavePPMFile(g_rgb_frame_data, g_frame_data.nWidth, g_frame_data.nHeight);
            }
        }
    }
    printf("<Acquisition thread Exit!>\n");
}

int main(int argc, char** argv)
{


    ros::init(argc, argv, "daheng");
    ros::NodeHandle nh;
    ros::NodeHandle _nh("~"); // to get the private params

    ROS_INFO_STREAM("Starting loading ros launch parameters....");

    _nh.param("camera_id", camera_id, std::string("left"));
    ROS_INFO_STREAM("Camera id: " << camera_id);

    _nh.param("camera_name", camera_name, std::string("camera"));
    ROS_INFO_STREAM("Camera name: " << camera_name);

    _nh.param("w_mode", w_mode, std::string("manual"));
    ROS_INFO_STREAM("white balance mode:manual or continuous?" << w_mode);

    _nh.param("w_red", w_red, 1.726);
    ROS_INFO_STREAM("white balance w_red" << w_red);

    _nh.param("w_green", w_green, 1.0);
    ROS_INFO_STREAM("white balance w_green" << w_green);

    _nh.param("w_blue", w_blue, 2.5);
    ROS_INFO_STREAM("white balance w_blue" << w_blue);

    _nh.param("gain_mode", gain_mode, std::string("manual"));
    ROS_INFO_STREAM("gain mode:manual or continuous?" << gain_mode);

    _nh.param("gain", gain, 10.0);
    ROS_INFO_STREAM("gain number:0-24" << gain);

    _nh.param("set_camera_fps", set_camera_fps, 20.0);
    ROS_INFO_STREAM("Setting camera FPS to: " << set_camera_fps);

    _nh.param("set_exposure_time", set_exposure_time, 30000.0);
    ROS_INFO_STREAM("Setting camera exposure time(us) to: " << set_exposure_time);

    _nh.param("frame_id", frame_id, std::string("camera"));
    ROS_INFO_STREAM("Publishing with frame_id: " << frame_id);

    _nh.param("camera_info_url", camera_info_url, std::string(""));
    ROS_INFO_STREAM("Provided camera_info_url: '" << camera_info_url << "'");



    ROS_INFO_STREAM("Initializing industrial camera based on the ros parameter......"); 


    uid_t user = 0;
    user = geteuid();
    if(user != 0)
    {
        ROS_INFO_STREAM("Please use sudo -s command to run this program in sudo!!!!......"); 
        return 0;
    }

    GX_STATUS status = GX_STATUS_SUCCESS;
    int ret = 0;
    GX_OPEN_PARAM open_param;
    //Initialize the camera library.
    status = GXInitLib();
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return 0;
    }
    //Get the number of enumerated camera
    uint32_t device_number = 0;
    status = GXUpdateDeviceList(&device_number, 1000);
    std::cout << "device_number: " << device_number << std::endl;
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return 0;
    }
    if(device_number <= 0)
    {
        printf("<No device>\n");
        return 0;
    }
    else
    {
        //Open the first device by index
        open_param.accessMode = GX_ACCESS_EXCLUSIVE;
        open_param.openMode = GX_OPEN_INDEX;

        if (camera_id == "left")
          open_param.pszContent = "1";
        else if (camera_id == "right")
          open_param.pszContent = "2";
        else
        {
          printf("<No device>\n");
          return 0;
        }

        status = GXOpenDevice(&open_param, &g_device);
        if(status == GX_STATUS_SUCCESS)
        {
            printf("<Open device success>\n");
        }
        else
        {
            printf("<Open devide fail>\n");
            return 0;			
        }
    }

   status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //Set the trigger mode to OFF
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //Get the Pixel format
    status = GXGetEnum(g_device, GX_ENUM_PIXEL_FORMAT, &g_pixel_format);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
    }
        
    //Color or Mono
    status = GXGetEnum(g_device, GX_ENUM_PIXEL_COLOR_FILTER, &g_color_filter);
    if(status != GX_STATUS_SUCCESS)
    {
        if (status != GX_STATUS_NOT_IMPLEMENTED)
        {
            GetErrorString(status);
        }
    }


    // //Set buffer quantity of acquisition queue
    // uint64_t nBufferNum = ACQ_BUFFER_NUM;
    // status = GXSetAcqusitionBufferNumber(g_device, nBufferNum);
    // if(status != GX_STATUS_SUCCESS)
    // {
    //     if (status != GX_STATUS_NOT_IMPLEMENTED)
    //     {
    //         GetErrorString(status);
    //     }
    // }

    //Set size of data transfer block
    status = GXSetInt(g_device, GX_DS_INT_STREAM_TRANSFER_SIZE, ACQ_TRANSFER_SIZE);
    if(status != GX_STATUS_SUCCESS)
    {
        if (status != GX_STATUS_NOT_IMPLEMENTED)
        {
            GetErrorString(status);
        }
    }

    //Set qty. of data transfer block
    status = GXSetInt(g_device, GX_DS_INT_STREAM_TRANSFER_NUMBER_URB, ACQ_TRANSFER_NUMBER_URB);
    if(status != GX_STATUS_SUCCESS)
    {
        if (status != GX_STATUS_NOT_IMPLEMENTED)
        {
            GetErrorString(status);
        }
    }

    //Set Balance White Mode : Continuous
    if(w_mode == "continuous")
    {
        ROS_INFO_STREAM("using auto white balance");
        status = GXSetEnum(g_device, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
    }
    else
    {
        // GX_FLOAT_RANGE ratioRange; 
        // status = GXGetFloatRange(g_device, GX_FLOAT_BALANCE_RATIO, &ratioRange); 
        ROS_INFO_STREAM("using manual white balance");
        //set white balance red channel
        status=GXSetEnum(g_device,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_RED); 
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }

        status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO, w_red); 
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
        //set white balance green channel
        status=GXSetEnum(g_device,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_GREEN); 
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
        status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO, w_green); 
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
        //set white balance blue channel
        status=GXSetEnum(g_device,GX_ENUM_BALANCE_RATIO_SELECTOR,GX_BALANCE_RATIO_SELECTOR_BLUE); 
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
        status = GXSetFloat(g_device, GX_FLOAT_BALANCE_RATIO, w_blue); 
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
    }
    if(gain_mode == "continuous")
    {
        //Set Gain Mode : Continuous
        ROS_INFO_STREAM("Using auto gain");
        status = GXSetEnum(g_device, GX_ENUM_GAIN_AUTO, GX_GAIN_AUTO_CONTINUOUS);
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
    }
    else
    {


        ROS_INFO_STREAM("using manual gain");
        status = GXSetEnum(g_device, GX_ENUM_GAIN_SELECTOR, GX_GAIN_SELECTOR_ALL);
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }
        status = GXSetFloat(g_device, GX_FLOAT_GAIN, gain);
        if(status != GX_STATUS_SUCCESS)
        {
            if (status != GX_STATUS_NOT_IMPLEMENTED)
            {
                GetErrorString(status);
            }
        }  
    }
    // enable frame rate setting 
    status = GXSetEnum(g_device, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    if(status != GX_STATUS_SUCCESS)
    {
        if (status != GX_STATUS_NOT_IMPLEMENTED)
        {
            GetErrorString(status);
        }
    }

    // setting the frame rate
    status = GXSetFloat(g_device, GX_FLOAT_ACQUISITION_FRAME_RATE, set_camera_fps);
    if(status != GX_STATUS_SUCCESS)
    {
        if (status != GX_STATUS_NOT_IMPLEMENTED)
        {
            GetErrorString(status);
        }
    }
    
    // setting exposure time (unit:us)
    status = GXSetFloat(g_device, GX_FLOAT_EXPOSURE_TIME, set_exposure_time);
    if(status != GX_STATUS_SUCCESS)
    {
        if (status != GX_STATUS_NOT_IMPLEMENTED)
        {
            GetErrorString(status);
        }
    }    

    ret = PreForImage();
    if(ret != 0)
    {
        printf("<Failed to prepare for acquire image>\n");
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }

    //Send the start  acquisition command
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_START);
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib();
        return 0;
    }


    ROS_INFO_STREAM("Opened the stream, starting to publish.");
    // Start acquisition thread, if thread create failed, exit this app
    // int nRet = pthread_create(&g_nAcquisitonThreadID, NULL, ProcGetImage, NULL);
    boost::thread cap_thread(ProcGetImage, nh);
    // boost::thread cap_thread(do_capture, nh);
    // if(nRet != 0)
    // {
    //     //Release the memory allocated
    //     UnPreForAcquisition();

    //     GXCloseDevice(g_hDevice);
    //     g_hDevice = NULL;
    //     GXCloseLib();

    //     printf("<Failed to create the acquisition thread, App Exit!>\n");
    //     ROS_ERROR_STREAM("Could not open the stream.");
    //     exit(nRet);
    // }



    ros::Rate r(set_camera_fps);
    while (nh.ok()) {

        // {
        //     std::lock_guard<std::mutex> g(q_mutex);
        //     if (!framesQueue.empty()){
        //         frame = framesQueue.front();
        //         framesQueue.pop();
        //     }
        // }

        // if (pub.getNumSubscribers() > 0){
        //     // Check if grabbed frame is actually filled with some content
        //     if(!frame.empty()) {
        //         // Flip the image if necessary
        //         msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
        //         // Create a default camera info if we didn't get a stored one on initialization
        //         if (cam_info_msg.distortion_model == ""){
        //             ROS_WARN_STREAM("No calibration file given, publishing a reasonable default camera info.");
        //             cam_info_msg = get_default_camera_info_from_image(msg);
        //             cam_info_manager.setCameraInfo(cam_info_msg);
        //         }
        //         // The timestamps are in sync thanks to this publisher
        //         pub.publish(*msg, cam_info_msg, ros::Time::now());
        //     }

        //     ros::spinOnce();
        // }
        r.sleep();
    }
    cap_thread.join();
}


