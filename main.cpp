//ou zheng
//2018/4/4



#include <iostream>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
//ou zheng
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
//Choudhisdfhish
#include <stdio.h>
//C++
#include <iostream>
#include <sstream>
#include <ctime>
#include <fstream>
#include <chrono>
#include <ctime>
using namespace cv;
using namespace std;

Point point1, point2; /* vertical points of the bounding box */
int drag = 0;
Rect rect; /* bounding box */
Mat img, roiImg; /* roiImg - the part of the image in the bounding box */
int select_flag = 1;
bool go_fast = false;
bool start=false;
int xLoc=0;
int yLoc=0;
int xLast=0;
int yLast=0;
int roix=0;
int roiy=0;
int roiWidth=0;
int roiHeight=0;

ofstream outfile;
string frameNumberString;
string fpsNumberString;
string timeNumberString;
Mat mytemplate;
Mat mytemplate2;
double timeFrame=0.0;
bool framePuse=false;

///------- template matching -----------------------------------------------------------------------------------------------

Mat TplMatch( Mat &img, Mat &mytemplate )
{
    Mat result;
   Rect region_of_interest = Rect(roix-roiWidth*2,roiy-roiHeight*2, roiWidth*5, roiHeight*5);
   //cout<<"roix"<<roix;
   //cout<<"roiy"<<roiy<<endl;;
   imshow("image roi", img(region_of_interest));
  
    matchTemplate( img(region_of_interest), mytemplate, result, CV_TM_SQDIFF_NORMED );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    return result;
}


///------- Localizing the best match with minMaxLoc ------------------------------------------------------------------------

Point minmax( Mat &result )
{
    double minVal, maxVal;
    Point  minLoc, maxLoc, matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    matchLoc = minLoc;
    
    return matchLoc;
}

///------- tracking --------------------------------------------------------------------------------------------------------

void track()
{
    if (select_flag)
    {

        //roiImg.copyTo(mytemplate);
        //         select_flag = false;
        go_fast = true;
    }
    
    //     imshow( "mytemplate", mytemplate ); waitKey(0);
    
    Mat result  =  TplMatch( img, mytemplate );
    Point match =  minmax( result );

    //re position roi
    match.x+=(roix-roiWidth*2);
    match.y+=(roiy-roiHeight*2);
    //cout<<sqrt(abs(xLast-match.x)^2+abs(yLast-match.y)^2)<<endl;
    if(start==false)
    {
        start=true;
        xLast=match.x;
        yLast=match.y;
    }
    // else if(sqrt(abs(xLast-match.x)^2+abs(yLast-match.y)^2)<2)

    // {
		roix=match.x;
        roiy= match.y;
 		xLast=match.x;
    	yLast=match.y;


  //    mytemplate2= img(Rect(match.x, match.y,  mytemplate.cols , mytemplate.rows ));
  //       //roiImg.copyTo(mytemplate);
  // imshow("mytemplate2", mytemplate2);
    	rectangle( img, match, Point( match.x + mytemplate.cols , match.y + mytemplate.rows ), CV_RGB(0, 255, 0), 0.5 );
    	rectangle( img,  Point( roix-roiWidth*2,roiy-roiHeight*2 ), Point( roix+roiWidth*3,roiy+roiHeight*3 ), CV_RGB(255, 0, 0), 0.5 );
        string displayInfor="1";
        putText(img, displayInfor.c_str(), cv::Point(match.x + mytemplate.cols+3 , match.y + mytemplate.rows),
               FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
    // }
    // else
    // {
    // rectangle( img, match, Point( match.x + mytemplate.cols , match.y + mytemplate.rows ), CV_RGB(0, 0, 255), 0.5 );
    //   string displayInfor="wrong";
    //    putText(img, displayInfor.c_str(), cv::Point(match.x + mytemplate.cols+3 , match.y + mytemplate.rows),
    //            FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
    // }
    // xCurrent=match.x-xLast;
    // yCurrent=match.y-yLast;
    
// //    if(match.x-xLoc!=0 ||match.y-yLoc!=0)
// //    {
//         outfile<<endl;
//         outfile<<frameNumberString<<","<<timeFrame<<","<<match.x-xLoc<<","<<match.y-yLoc<<","<<xCurrent<<","<<yCurrent<<",";
        
// //    }
   
   
   // /// latest match is the new template
   // Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
   // roiImg = img( ROI );
   // roiImg.copyTo(mytemplate);
   // imshow( "roiImg", roiImg ); //waitKey(0);
}


///------- MouseCallback function ------------------------------------------------------------------------------------------

void mouseHandler(int event, int x, int y, int flags, void *param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        /// left button clicked. ROI selection begins
        point1 = Point(x, y);
        drag = 1;
    }
    
    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        /// mouse dragged. ROI being selected
        Mat img1 = img.clone();
        point2 = Point(x, y);
        rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 3, 8, 0);
        imshow("image", img1);
    }
    
    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        point2 = Point(x, y);
        rect = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
        roix=point1.x;
        roiy= point1.y;

        roiWidth=x - point1.x;
        roiHeight=y - point1.y;
rectangle( img,  Point( point1.x, point1.y), Point( x,y ), CV_RGB(0, 255, 0), 0.5 );
      

        drag = 0;
        roiImg = img(rect);
        roiImg.copyTo(mytemplate);
        //imshow("MOUSE roiImg", roiImg); waitKey(0);
    }
    
    if (event == CV_EVENT_LBUTTONUP)
    {
        /// ROI selected
        select_flag = 1;
        drag = 0;
    }
    
}



///------- Main() ----------------------------------------------------------------------------------------------------------

int main( int argc, char** argv ){
    // show help
    if(argc<2){
        cout<<
        " Usage: tracker <video_name>\n"
        " examples:\n"
        " example_tracking_kcf Bolt/img/%04d.jpg\n"
        " example_tracking_kcf faceocc2.webm\n"
        << endl;
        return 0;
    }
    int k;
    /*
     ///open webcam
     VideoCapture cap(0);
     if (!cap.isOpened())
     return 1;*/
    
    ///open video file
    std::string video = argv[1];
    VideoCapture cap(video);
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];
    
    time (&rawtime);
    timeinfo = localtime(&rawtime);
    
    strftime(buffer,sizeof(buffer),"%d-%m-%Y %I:%M:%S",timeinfo);
    std::string str(buffer);
    
    std::cout << str;
    //open csv file
    cout<<"filename"<<video<<endl;
    string fileName="./file/"+video+"_"+str+".csv";
  
    outfile.open(fileName);
    outfile<<"frameNUM"<<","<<"time(s)"<<","<<"xdifferentfrom0"<<","<<"ydifferentfrom0"<<","<<"xdifferentfromlastx"<<","<<"ydifferentfromlasty"<<",";
    if ( !cap.isOpened() )
    {   cout << "Unable to open video file" << endl;    return -1;    }
    /*
     /// Set video to 320x240
     cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
     cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);*/
    
    cap >> img;
    
    //GaussianBlur( img, img, Size(7,7), 3.0 );
    imshow( "image", img );
    
    for(;;)
    {
        cout<<framePuse<<endl;
   
     //    if(framePuse)
     //    {
     //      //cvSetMouseCallback( "image", mouseHandler, NULL );
     //        continue;
     //    }
        // else
        // {
         cap >> img;
       //  resize(img, img, Size(1920, 1080));
        if ( img.empty() )
            break;
          if (waitKey(1)==27)
        {
            framePuse=!framePuse;
        }

       // // GaussianBlur( img, img, Size(7,7), 3.0 );
       //  stringstream ss;
       //  stringstream st;
       //  stringstream fps;
       //  rectangle(img, cv::Point(10, 2), cv::Point(450,20),
       //            cv::Scalar(255,255,255), -1);
       //  ss << cap.get(CAP_PROP_POS_FRAMES);
       //  fps << cap.get(CAP_PROP_FPS);
       //  st << cap.get( CAP_PROP_POS_MSEC);
       //   frameNumberString = ss.str();
       //  fpsNumberString = fps.str();
       //   timeNumberString = st.str();
       //  timeFrame=stod(timeNumberString)/1000;
       //  //  string timeNumberString = st.str();
       //  putText(img, frameNumberString.c_str(), cv::Point(15, 15),
       //          FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
       //  putText(img, fpsNumberString.c_str(), cv::Point(70, 15),
       //          FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
       //  putText(img, to_string(timeFrame), cv::Point(190, 15),
       //          FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
       //  // stop the program if no more images
       //  // Flip the frame horizontally and add blur
       //  //cv::flip( img, img, 1 );
        
        
       //  if ( rect.width == 0 && rect.height == 0 )
       //      cvSetMouseCallback( "image", mouseHandler, NULL );
       //  else
       //      track();
        
        imshow("image2", img);
       //  //  waitKey(100);   k = waitKey(75);
        //k = waitKey(go_fast ? 30 : 10000);
        //}


        waitKey(0)
        
        
        
        

            // break;
    }
    //delete capture object
    cap.release();
    outfile.close();
    return 0;
}