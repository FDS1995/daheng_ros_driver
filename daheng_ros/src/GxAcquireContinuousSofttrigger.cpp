//-------------------------------------------------------------
/**
\file      GxAcquireContinuousSofttrigger.cpp
\brief     sample to show how to acquire image by softtrigger.
\version   1.0.1809.9181
\date      2018-09-18
*/
//--------------------------------------------------------------
#include "GxIAPI.h"
#include "DxImageProc.h"
#include "CTimeCounter.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>

#define FRAMEINFOOFFSET 14
#define MEMORY_ALLOT_ERROR -1

GX_DEV_HANDLE g_device = NULL;                                    ///< The camera handle
GX_FRAME_DATA g_frame_data = { 0 };                               ///< The parameters of image acquisition 
void *g_raw8_buffer = NULL;                                       ///< The buffer for converting non-8-bit raw data into 8-bit data
void *g_rgb_frame_data = NULL;                                    ///< The buffer for converting 8 bit raw data into RGB data
int64_t g_pixel_format = GX_PIXEL_FORMAT_BAYER_GR8;               ///< The pixel format of current camera
int64_t g_color_filter = GX_COLOR_FILTER_NONE;                    ///< The parameters of bayer interpolation
pthread_t g_acquire_thread = 0;                                   ///< The ID of acquisition thread
bool g_get_image = false;                                         ///< The flag of acquisition thread: true running; false:exit
void *g_frameinfo_data = NULL;                                    ///< The buffer for Frame information
size_t g_frameinfo_datasize = 0;                                  ///< The length of frame information

// CTimeCounter g_time_counter;                                      ///< The timer

//Get the image size and allocate the memory.
int PreForImage();

//Free the resources
int UnPreForImage();

//The function of acquisition thread
void *ProcGetImage(void* param);

//Save the data to the PPM file
void SavePPMFile(void *image_buffer, size_t width, size_t height);

//Save the raw data file
void SaveRawFile(void *image_buffer, size_t width, size_t height);

//Convert the raw data to RGB data
void ProcessData(void *image_buffer, void *image_raw8_buffer, void *image_rgb_buffer, int image_width, int image_height, int pixel_format, int color_filter);

//Get the error message
void GetErrorString(GX_STATUS error_status);

//Get the index of current frame ID
int GetCurFrameIndex();

int main()
{
    uid_t user = 0;
    user = geteuid();
    if(user != 0)
    {
        printf("\n");
        printf("Please run this application with 'sudo -E ./GxAcquireContinuousSofttrigger' or"
                              " Start with root !\n");
        printf("\n");
        return 0;
    }

    printf("\n");
    printf("-------------------------------------------------------------\n");
    printf("sample to show how to acquire image by softtrigger.\n");
    #ifdef __x86_64__
    printf("version: 1.0.1802.8061\n");
    #elif __i386__
    printf("version: 1.0.1802.9061\n");
    #endif
    printf("-------------------------------------------------------------\n");
    printf("\n");
 
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
        open_param.pszContent = "1";  		
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

    //Set to continuous acquisition mode
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

    //Set the trigger mode to ON
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON);
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
	

    //Set TriggerSource to software trigger
    status = GXSetEnum(g_device, GX_ENUM_TRIGGER_SOURCE, GX_TRIGGER_SOURCE_SOFTWARE);
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

    //Prepare for image acquisition
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
    
    //Start the acquisition thread
    ret = pthread_create(&g_acquire_thread, 0, ProcGetImage, 0);
    if(ret != 0)
    {
        printf("<Failed to create the collection thread>\n");
        return 0;
    }

    printf("====================Menu================\n");
    printf("[X or x]:Exit\n");
    printf("[S or s]:Send softtrigger command\n");

    bool run = true;
    while(run == true)
    {
        int c = getchar();
        switch(c)
        {
            //exit
            case 'X':
            case 'x':
                run = false;
                break;

            //Send the TriggerSoftware command
            case 'S':
            case 's':
                status = GXSendCommand(g_device, GX_COMMAND_TRIGGER_SOFTWARE);
                printf("<The return value of softtrigger command: %d>\n", status);
                break;

            default:
                break;
        }	
    }
        
    //Prepare to stop image acquisition
    ret = UnPreForImage();
    if(ret != 0)
    {
        status = GXCloseDevice(g_device);
        if(g_device != NULL)
        {
            g_device = NULL;
        }
        status = GXCloseLib(); 
        return 0;
    }

    //Close the camera
    status = GXCloseDevice(g_device); 
    if(status != GX_STATUS_SUCCESS)
    {    
        return 0;
    }

    //Close the library
    status = GXCloseLib();

    return 0;
}

//-------------------------------------------------
/**
\Get the index of current frame
\return  int
*/
//-------------------------------------------------
int GetCurFrameIndex()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    status = GXGetBuffer(g_device, GX_BUFFER_FRAME_INFORMATION, (uint8_t*)g_frameinfo_data, &g_frameinfo_datasize);
    if(status != GX_STATUS_SUCCESS)
    {
        //Add codes here
    }

    int current_index = 0;
    memcpy(&current_index, (uint8_t*)g_frameinfo_data+FRAMEINFOOFFSET, sizeof(int));

    return current_index;
}

//-------------------------------------------------
/**
\ Get the image size and allocate the memory.
\return  int
*/
//-------------------------------------------------
int PreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;

    //Allocate the memory
    int64_t payload_size = 0;
    status = GXGetInt(g_device, GX_INT_PAYLOAD_SIZE, &payload_size); 
    if(status != GX_STATUS_SUCCESS)
    {
        GetErrorString(status);
        return status;
    }

    g_frame_data.pImgBuf = malloc(payload_size);
    if (g_frame_data.pImgBuf == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return MEMORY_ALLOT_ERROR;
    }
	
    //The buffer for converting non-8-bit raw data into 8-bit data
    g_raw8_buffer = malloc(payload_size);
    if (g_raw8_buffer == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return MEMORY_ALLOT_ERROR;
    }

    
    g_rgb_frame_data = malloc(payload_size * 3);
    if (g_rgb_frame_data == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return MEMORY_ALLOT_ERROR;
    }
    
    //Get the image size and allocate the memory.
    status = GXGetBufferLength(g_device, GX_BUFFER_FRAME_INFORMATION, &g_frameinfo_datasize);
    if(status != GX_STATUS_SUCCESS)
    { 
        //Add codes here
    }
    
    if(g_frameinfo_datasize > 0)
    {    
        g_frameinfo_data = malloc(g_frameinfo_datasize);
        if(g_frameinfo_data == NULL)
        {
            printf("<Failed to allocate memory>\n");
            return MEMORY_ALLOT_ERROR;
        }
    }
    return 0;
}

//-------------------------------------------------
/**
\brief Free resources
\return void
*/
//-------------------------------------------------
int UnPreForImage()
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    uint32_t ret = 0;

    g_get_image = false;
    ret = pthread_join(g_acquire_thread,NULL);
    if(ret != 0)
    {
        printf("<Failed to release resources>");
        return ret;
    }
	
    //Send the stop acquisition command
    status = GXSendCommand(g_device, GX_COMMAND_ACQUISITION_STOP);
    if(status != GX_STATUS_SUCCESS)
    {
        return status;
    }

    //Free the buffer
    if(g_frame_data.pImgBuf != NULL)
    {
        free(g_frame_data.pImgBuf);
	g_frame_data.pImgBuf = NULL;
    }

    if(g_raw8_buffer != NULL)
    {
        free(g_raw8_buffer);
        g_raw8_buffer = NULL;
    }

    if(g_rgb_frame_data != NULL)
    {
        free(g_rgb_frame_data);
        g_rgb_frame_data = NULL;
    }

    if(g_frameinfo_data != NULL)
    {
        free(g_frameinfo_data);
        g_frameinfo_data = NULL;
    }
 
    return 0;
}


//-------------------------------------------------
/**
\ The function of acquisition thread
\param param 
\return void*
*/
//-------------------------------------------------
void ProcGetImage(void* param)
{
    GX_STATUS status = GX_STATUS_SUCCESS;
    bool is_implemented = false;
    //
    g_get_image = true;

    while(g_get_image)
    {

        status = GXGetImage(g_device, &g_frame_data, 100);
        if(status == GX_STATUS_SUCCESS)
        {
            if(g_frame_data.nStatus == 0)
            {
                printf("<Successful acquisition: Width: %d Height: %d>\n", g_frame_data.nWidth, g_frame_data.nHeight);
                status = GXIsImplemented(g_device, GX_BUFFER_FRAME_INFORMATION, &is_implemented);
                if(status == GX_STATUS_SUCCESS)
                {
                    if(true == is_implemented)
                    {
                        printf("<Frame number: %d>\n", GetCurFrameIndex());
                    }
                }

                //Save the raw data
                SaveRawFile(g_frame_data.pImgBuf, g_frame_data.nWidth, g_frame_data.nHeight);

                //Convert the 8 bit raw data into the RGB data
                ProcessData(g_frame_data.pImgBuf, 
                        g_raw8_buffer, 
                        g_rgb_frame_data, 
                        g_frame_data.nWidth, 
                        g_frame_data.nHeight,
                        g_pixel_format,
                        g_color_filter);

                //Save the RGB data
                SavePPMFile(g_rgb_frame_data, g_frame_data.nWidth, g_frame_data.nHeight);
            }
        }
    }
}

//-------------------------------------------------
/**
\ Save the image data into the ppm file
\param image_buffer
\param width
\param height
\return void
*/ 
//-------------------------------------------------
void SavePPMFile(void *image_buffer, size_t width, size_t height)
{
    char name[64] = {0};

    static int rgb_file_index = 1;
    FILE* ff = NULL;

    sprintf(name, "RGB%d.ppm", rgb_file_index++);
    ff=fopen(name,"wb");
    if(ff != NULL)
    {
        fprintf(ff, "P6\n" "%d %d\n255\n", width, height);
        fwrite(image_buffer, 1, width * height * 3, ff);
        fclose(ff);
        ff = NULL;
        printf("<Save %s success>\n", name);
    }    
}

//-------------------------------------------------
/**
\ Save the raw image data into file
\param image_buffer 
\param width: image 
\param height:image 
\return void
*/
//-------------------------------------------------
void SaveRawFile(void *image_buffer, size_t width, size_t height)
{
    char name[64] = {0};

    static int raw_file_index = 1;
    FILE* ff = NULL;

    sprintf(name, "RAW%d.pgm", raw_file_index++);
    ff=fopen(name,"wb");
    if(ff != NULL)
    {
        fprintf(ff, "P5\n" "%u %u 255\n", width, height);
        fwrite(image_buffer, 1, width * height, ff);
        fclose(ff);
        ff = NULL;
        printf("<Save %s success>\n", name);
    }
}

//----------------------------------------------------------------------------------
/**
\  Convert the raw data to the RGB data
\param  [in] image_buffer  
\param  [in] image_raw8_buffer 
\param  [in,out] image_rgb_buffer 
\param  [in] image_width
\param  [in] image_height 
\param  [in] pixel_format 
\param  [in] color_filter
\return void
*/
//----------------------------------------------------------------------------------
void ProcessData(void *image_buffer, void *image_raw8_buffer, void *image_rgb_buffer, int image_width, int image_height, int pixel_format, int color_filter)
{
    switch(pixel_format)
    {
        //When the data format is 12 bits, it's converted to 4-11 bit.
        case GX_PIXEL_FORMAT_BAYER_GR12:
        case GX_PIXEL_FORMAT_BAYER_RG12:
        case GX_PIXEL_FORMAT_BAYER_GB12:
        case GX_PIXEL_FORMAT_BAYER_BG12:
            //Convert the 12-bit format image to the 8-bit format
            DxRaw16toRaw8(image_buffer, image_raw8_buffer, image_width, image_height, DX_BIT_4_11);	
            //Convert the Raw8 image to the RGB image
            DxRaw8toRGB24(image_raw8_buffer, image_rgb_buffer, image_width, image_height, RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(color_filter), false);		        
            break;

        //When the data format is 10 bits, it is converted to 2-9 bit.
        case GX_PIXEL_FORMAT_BAYER_GR10:
        case GX_PIXEL_FORMAT_BAYER_RG10:
        case GX_PIXEL_FORMAT_BAYER_GB10:
        case GX_PIXEL_FORMAT_BAYER_BG10:
            //Convert the 10-bit format image to the 8-bit format
            DxRaw16toRaw8(image_buffer, image_raw8_buffer, image_width, image_height, DX_BIT_2_9);
            //Convert the Raw8 image to the RGB image
            DxRaw8toRGB24(image_raw8_buffer, image_rgb_buffer, image_width, image_height,RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(color_filter),false);	
            break;

        case GX_PIXEL_FORMAT_BAYER_GR8:
        case GX_PIXEL_FORMAT_BAYER_RG8:
        case GX_PIXEL_FORMAT_BAYER_GB8:
        case GX_PIXEL_FORMAT_BAYER_BG8:
            ////Convert the Raw8 image to the RGB image
            // g_time_counter.Begin();
            DxRaw8toRGB24(image_buffer,image_rgb_buffer, image_width, image_height,RAW2RGB_NEIGHBOUR,
                 DX_PIXEL_COLOR_FILTER(color_filter),false);	
            // printf("<DxRaw8toRGB24 time consuming: %ld us>\n", g_time_counter.End());
            break;

        case GX_PIXEL_FORMAT_MONO12:
            //Convert the 12-bit format image to the 8-bit format
            DxRaw16toRaw8(image_buffer, image_raw8_buffer, image_width, image_height, DX_BIT_4_11);	
            //Convert the Raw8 image to the RGB image
            DxRaw8toRGB24(image_raw8_buffer, image_rgb_buffer, image_width, image_height, RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(NONE),false);		        
            break;

        case GX_PIXEL_FORMAT_MONO10:
            //Convert the 10-bit format image to the 8-bit format
            DxRaw16toRaw8(image_buffer, image_raw8_buffer, image_width, image_height, DX_BIT_4_11);	
            //Convert the Raw8 image to the RGB image
            DxRaw8toRGB24(image_raw8_buffer, image_rgb_buffer, image_width, image_height,RAW2RGB_NEIGHBOUR,
                DX_PIXEL_COLOR_FILTER(NONE),false);		        
            break;

        case GX_PIXEL_FORMAT_MONO8:
            //Convert the Raw8 image to the RGB image
            DxRaw8toRGB24(image_buffer, image_rgb_buffer, image_width, image_height,RAW2RGB_NEIGHBOUR,
            DX_PIXEL_COLOR_FILTER(NONE),false);		        
            break;

        default:
            break;
    }
}

//----------------------------------------------------------------------------------
/**
\brief Get the error message 
\param  error_status

\return void
*/
//----------------------------------------------------------------------------------
void GetErrorString(GX_STATUS error_status)
{
    char *error_info = NULL;
    size_t    size         = 0;
    GX_STATUS status      = GX_STATUS_SUCCESS;
	
    // Get the length of error information 
    status = GXGetLastError(&error_status, NULL, &size);
    error_info = new char[size];
    if (error_info == NULL)
    {
        printf("<Failed to allocate memory>\n");
        return ;
    }
	
    // Get the error message 
    status = GXGetLastError(&error_status, error_info, &size);
    if (status != GX_STATUS_SUCCESS)
    {                                        
        printf("<GXGetLastError called failed>\n");
    }
    else
    {
        printf("%s\n", (char*)error_info);
    }

    // Free the resources
    if (error_info != NULL)
    {
        delete[]error_info;
        error_info = NULL;
    }
}




