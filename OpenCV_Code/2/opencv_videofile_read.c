#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>

#define IMG_Width     640
#define IMG_Height    480

using namespace cv;
using namespace std; 

int main(void)
{
    int img_width, img_height;
    img_width = 640;
    img_height = 480;
    
    ///////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////OpenCV 변수 선언///////////////////////////////////////////
    
    Mat mat_image_org_color;
    Mat mat_image_org_gray;
    Mat mat_image_org_gary_result;
    Mat image;
	
    Scalar GREEN(0,255,0);
    Scalar RED(0,0,255);
    Scalar BLUE(255,0,0);
    Scalar YELLOW(0,255,255);
    //////////////////////////////////////////////////////////////////////////////////////


    VideoCapture cap("/home/pi/Team_aMAP/OpenCV_Code-s.mkv");

    if (!cap.isOpened())
    {
        cerr << "Error ! Can't Open the Camera \n";
        return -1;
    }
    printf("Image size[%3d,%3d]\n", img_width,img_height);
    
    namedWindow("Display window", CV_WINDOW_NORMAL);
    resizeWindow("Display window", img_width,img_height);
    moveWindow("Display window", 10, 10);
   
	
    while(1)
    {
       
      cap.read(mat_image_org_color);       		  	     
      if (!cap.isOpened())
      {
        cerr << "Empty Video \n";
        return -1;
      }
    
    imshow("Display window", mat_image_org_color);
    
    if(waitKey(10) > 0)
        break;
    }
    
    cap.release();
    destroyAllWindows();
    
   return 0;	
}
