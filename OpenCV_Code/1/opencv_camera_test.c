#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std; 

#define IMG_Width     640
#define IMG_Height    480


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


    VideoCapture cap(0);
    
    cap.set(CV_CAP_PROP_FRAME_WIDTH, img_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, img_height);
    
    if (!cap.isOpened())
    {
        cerr << "에러 - 카메라를 열 수 없습니다. \n";
        return -1;
    }
    cap.read(mat_image_org_color);
    	
    printf("Image size[%3d,%3d]\n", img_width,img_height);
    
    namedWindow("Display window", CV_WINDOW_NORMAL);
    resizeWindow("Display window", img_width,img_height);
    moveWindow("Display window", 10, 10);
   
	
    while(1)
    {
             		  	     
      if (!cap.isOpened())
      {
          cerr << "에러 - 카메라를 열 수 없습니다. \n";
        return -1;
      }
    cap.read(mat_image_org_color);
    
    imshow("Display window", mat_image_org_color);
    if(waitKey(5) > 0)
        break;
    }
    
    destroyAllWindows();
    
   return 0;	
}
