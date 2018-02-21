#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer

int main(int argc, char** argv)
{
  // Check if video source has been passed as a parameter
  //if(argv[1] == NULL) return 1;

  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);

  /*
  // Convert the passed as command line parameter index for the video device to an integer
  std::istringstream video_sourceCmd(argv[1]);
  int video_source;
  // Check if it is indeed a number
  if(!(video_sourceCmd >> video_source)) return 1;

  cv::VideoCapture cap(video_source);
  // Check if video device can be opened with the given index
  if(!cap.isOpened()) return 1;
  */
  cv::Mat frame;
  sensor_msgs::ImagePtr msg;

  // camera settings
  int                     iCameraCounts = 1;
    int                     iStatus=-1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;      // Device descriptions
    tSdkFrameHead           sFrameInfo;
    BYTE*			        pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage *iplImage = NULL;
    int                     channel=3;

    CameraSdkInit(1);

    // link devices
    CameraEnumerateDevice(&tCameraEnumList,&iCameraCounts);
    // No device connected
    if(iCameraCounts==0){
        return -1;
    }
  
  // Initialize cameras
    iStatus = CameraInit(&tCameraEnumList,-1,-1,&hCamera);
    // Initialize cameras fail
    if(iStatus!=CAMERA_STATUS_SUCCESS){
        return -1;
    }
  
  CameraGetCapability(hCamera,&tCapability);
  g_pRgbBuffer = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);
  CameraPlay(hCamera);
  
  if(tCapability.sIspCapacity.bMonoSensor){
        channel=1;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_MONO8);
    }else{
        channel=3;
        CameraSetIspOutFormat(hCamera,CAMERA_MEDIA_TYPE_BGR8);
    }

  ros::Rate loop_rate(5);
  while (nh.ok()) {
    /*
    cap >> frame;
    // Check if grabbed frame is actually full with some content
    if(!frame.empty()) {
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub.publish(msg);
      cv::waitKey(1);
    }
    */
    
    if(CameraGetImageBuffer(hCamera,&sFrameInfo,&pbyBuffer,1000) == CAMERA_STATUS_SUCCESS)
		{
		    CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer,&sFrameInfo);
		    if (iplImage)
            {
                cvReleaseImageHeader(&iplImage);
            }
            iplImage = cvCreateImageHeader(cvSize(sFrameInfo.iWidth,sFrameInfo.iHeight),IPL_DEPTH_8U,channel);
            cvSetData(iplImage,g_pRgbBuffer,sFrameInfo.iWidth*channel);
            // convert iplImage to OpenCV Mat
            #if 0
            cvShowImage("OpenCV Demo",iplImage);
            #else
            cv::Mat Iimag(iplImage);
            //imshow("OpenCV Demo",Iimag);
            #endif

            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", Iimag).toImageMsg();
            pub.publish(msg);
             cv::waitKey(1);

            // MUST release the buffer allocated from CameraReleaseImageBuffer after CameraGetImageBuffer.
			//Otherwise next CameraGetImageBuffer will be blocked until CameraReleaseImageBuffer been called to release the buffer.
			CameraReleaseImageBuffer(hCamera,pbyBuffer);

		}

    ros::spinOnce();
    loop_rate.sleep();
  }
}
