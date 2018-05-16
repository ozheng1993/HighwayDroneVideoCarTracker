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
int screenOgrinWidth=0;
int screenOgrinHeight=0;
int screenWidthScale=0;
int screenHeightScale=0;
int screenWidth=1920;
int screenHeight=1080;
int startLineX=0;
int endLineX=screenWidth-100;
int startLineY=0;
int endLineY=screenHeight;
ofstream outfile;
string frameNumberString;
string fpsNumberString;
string timeNumberString;
string totalFrameNumberString;
Mat mytemplate;
Mat mytemplate2;
double timeFrame=0.0;
bool framePuse=true;
bool frameTrack=true;
vector<Mat> carOrginTemplates;      // array of 10 images
vector<Mat> carTemplates;      // array of 10 images
vector<Mat> carLastTemplates;      // array of 10 images
vector<int> carX;
vector<int> carY;
vector<int> carLastX;
vector<int> carLastY;
vector<int> carWidth;
vector<int> carHeight;
vector<int> carStatus;//0=tag 1=tracking 2=lost 3= finised
//remove object
Mat inpainted;

vector<Mat> objectTemplates;      // array of 10 images
vector<int> objectX;
vector<int> objectY;
vector<int> objectLastX;
vector<int> objectLastY;
vector<int> objectWidth;
vector<int> objectHeight;
vector<int> objectStatus;//0=tag 1=tracking 2=lost 3= finised

vector<Mat> landmarkTemplates;      // array of 10 images
vector<int> landmarkX;
vector<int> landmarkY;
vector<int> landmarkLastX;
vector<int> landmarkLastY;
vector<int> landmarkWidth;
vector<int> landmarkHeight;
vector<int> landmarkStatus;//0=tag 1=tracking 2=lost 3= finised

double findDistanceBetweenTwoPoint(int x1,int y1,int x2,int y2)
{
    double distanceX=abs(x1-x2);
    double distanceY=abs(y1-y2);
    double distance=sqrt(distanceX*distanceX+distanceY*distanceY);
    return distance;
}
///------- template matching -----------------------------------------------------------------------------------------------

Mat TplMatch( Mat &img, Mat &mytemplate,int index,int x,int y,int width,int height,int temRoiX,int temRoiY,int temRoiW,int temRoiH )
{
    Mat result;
    Mat Roi;
    Rect region_of_interest;
    //    if((x+width*3)<screenWidth&&(x-width*2)>0)
    //    {
    //        cout<<"normal roi"<<endl;
    //       region_of_interest = Rect(x-width*2,y-height*2, width*5, height*5);
    //    }
    //    else
    //    {
    //        if((x-width*2)>0)
    //        {
    //            cout<<"right roi"<<endl;
    //        region_of_interest = Rect(x-width*2,y-height*2, screenWidth-(x-width*2), height*5);
    //        }
    //        else if((x+width*3)<screenWidth)
    //        {
    //             cout<<"left roi"<<endl;
    //             region_of_interest = Rect(1,y-height*2, width*7, height*5);
    //
    //        }
    //
    //    }
    
    region_of_interest = Rect(temRoiX,temRoiY, temRoiW, temRoiH);
    
    
    
    
    
    //cout<<"roix"<<roix;
    //cout<<"roiy"<<roiy<<endl;;
    string windowname="car roi of "+to_string(index);
    Roi=img(region_of_interest);
    // string displayInfor="car"+to_string(index)+"x"+to_string(x)+"y"+to_string(y)+"width"+to_string(width)+"height"+to_string(height);
    // //putText(Roi, displayInfor.c_str(), cv::Point(x+width , y + height),
    // FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
    
    matchTemplate( img(region_of_interest), mytemplate, result, CV_TM_SQDIFF_NORMED );
    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
    //imshow(windowname, Roi);
    return result;
}


///------- Localizing the best match with minMaxLoc ------------------------------------------------------------------------

Point minmax( Mat &result )
{
    double minVal, maxVal;
    Point  minLoc, maxLoc, matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );
    //    cout<<"min"<<minVal<<endl;
    //    cout<<"maxVal"<<maxVal<<endl;
    matchLoc = minLoc;
    
    return matchLoc;
}

///------- tracking --------------------------------------------------------------------------------------------------------

void track(int index)
{
    if (select_flag)
    {
        
        //roiImg.copyTo(mytemplate);
        //         select_flag = false;
        go_fast = true;
    }
    int tmpRoiX=0;
    int tmpRoiY=0;
    int tmpRoiW=0;
    int tmpRoiH=0;
    int tmpRoiScaleW=50;
    int tmpRoiScaleH=20;
    
    
    // cout<<"carX[index]"<<carX[index]<<endl;
    if(carX[index]+carWidth[index]+tmpRoiScaleW>=endLineX||carX[index]-tmpRoiScaleW/2<=carWidth[index]||carY[index]+carHeight[index]+tmpRoiScaleH>=endLineY||carY[index]-tmpRoiScaleH/2<=carHeight[index])
    {
        //cout<<"finished"<<endl;
        carStatus[index]=3;
        return ;
    }
    //finishe dete
    else if((carX[index]+carWidth[index]+tmpRoiScaleW)<endLineX&&(carX[index]-tmpRoiScaleW/2)>=0&&(carY[index]-tmpRoiScaleH/2)>0&&(carY[index]+carHeight[index]+tmpRoiScaleH)<endLineY)
    {
        // cout<<"normal roi"<<endl;
        tmpRoiX=carX[index]-tmpRoiScaleW/2;
        tmpRoiY=carY[index]-tmpRoiScaleH/2;
        tmpRoiW=carWidth[index]+tmpRoiScaleW;
        tmpRoiH=carHeight[index]+tmpRoiScaleH;
        // cout<<"tmpRoiX"<<tmpRoiX<<endl;
        //region_of_interest = Rect(x-width*2,y-height*2, width*5, height*5);
    }
    //    else
    //    {
    //        if((carX[index]-carWidth[index]*2)<=0)
    //        {
    //            //cout<<"carX[carX[index]-carWidth[index]*2]"<<carX[index]-carWidth[index]*2<<endl;
    //           cout<<"left roi"<<endl;
    //            tmpRoiX=0;
    //            tmpRoiY=carY[index]-carHeight[index]*2;
    //            tmpRoiW=carX[index]+carWidth[index]*2;
    //            tmpRoiH=carHeight[index]*5;
    //
    //            // region_of_interest = Rect(x-width*2,y-height*2, screenWidth-(x-width*2), height*5);
    //        }
    //        else if((carX[index]+carWidth[index]*3)>=endLineX)
    //        {
    //            // cout<<"right roi"<<endl;
    //
    //            tmpRoiX=carX[index]-carWidth[index]*2;
    //            tmpRoiY=carY[index]-carHeight[index]*2;
    //            tmpRoiW=endLineX-carX[index]+carWidth[index]*2;
    //            tmpRoiH=carHeight[index]*5;
    //            //region_of_interest = Rect(1,y-height*2, width*7, height*5);
    //
    //        }
    //
    //
    //
    //        if((carY[index]-carHeight[index]*3)<0)
    //        {
    //            //cout<<"carX[carX[index]-carWidth[index]*2]"<<carX[index]-carWidth[index]*2<<endl;
    //            //cout<<"left roi"<<endl;
    //            tmpRoiX=carX[index]-carWidth[index]*2;
    //            tmpRoiY=0;
    //            tmpRoiW=carWidth[index]*5;
    //            tmpRoiH=carHeight[index]*5;
    //
    //            // region_of_interest = Rect(x-width*2,y-height*2, screenWidth-(x-width*2), height*5);
    //        }
    //        else if((carY[index]+carHeight[index]*3)>=endLineY)
    //        {
    //            cout<<"down roi"<<endl;
    //
    //            tmpRoiX=carX[index]-carWidth[index]*2;
    //            tmpRoiY=carY[index]-carHeight[index]*2;
    //            tmpRoiW=carWidth[index]*5;
    //            tmpRoiH=endLineY-carY[index]+carHeight[index]*3;
    //            cout<<"x"<<tmpRoiX<<endl;
    //             cout<<"y"<<tmpRoiY<<endl;
    //             cout<<"w"<<tmpRoiW<<endl;
    //             cout<<"h"<<tmpRoiH<<endl;
    //            //region_of_interest = Rect(1,y-height*2, width*7, height*5);
    //        }
    //    }
    //
    //     imshow( "mytemplate", mytemplate ); waitKey(0);
    Mat result  =  TplMatch( img, carTemplates[index],index,carX[index],carY[index],carWidth[index],carHeight[index], tmpRoiX,tmpRoiY,tmpRoiW,tmpRoiH);
    Point match =  minmax( result );
    
    //re position roi
    match.x+=(carX[index]-tmpRoiScaleW/2);
    match.y+=(carY[index]-tmpRoiScaleH/2);
    //cout<<sqrt(abs(xLast-match.x)^2+abs(yLast-match.y)^2)<<endl;
    if(carStatus[index]==0)
    {
        carStatus[index]=1;
        carLastX[index]=match.x;
        carLastY[index]=match.y;
    }
    // else
    
    //    if(abs(carLastX[index]-match.x)>carWidth[index]&&abs(carLastY[index]-match.y)>carHeight[index])
    //    {
    //        cout<<"car"<<index<<"lost"<<endl;
    //        carStatus[index]=2;
    //    }
    
    
    //    if(carStatus[index]==4)
    //    {
    //
    //    }
    //
    
    if(abs(carLastX[index]-match.x)<carWidth[index]/2&&abs(carLastY[index]-match.y)<carHeight[index]/2&&carStatus[index]!=4)
    {
        
        
        carX[index]=match.x;
        carY[index]=match.y;
        //    mytemplate2= img(Rect(match.x, match.y,  mytemplate.cols , mytemplate.rows ));
        //       //roiImg.copyTo(mytemplate);
        // imshow("mytemplate2", mytemplate2);
        rectangle( img, match, Point( match.x + carWidth[index] , match.y + carHeight[index] ), CV_RGB(0, 255, 0), 0.5 );
        //rectangle( img,  Point( tmpRoiX,tmpRoiY), Point( tmpRoiX+tmpRoiW,tmpRoiY+tmpRoiH), CV_RGB(255, 0, 0), 0.5 );
        string displayInfor=to_string(index);
        putText(img, displayInfor.c_str(), cv::Point(match.x + carWidth[index] , match.y + carHeight[index]),
                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
        carLastX[index]=match.x;
        carLastY[index]=match.y;
        Rect tmpROI = cv::Rect( carLastX[index], carLastY[index], carWidth[index], carHeight[index]);
        Mat tempRoiImg = img( tmpROI );
        carLastTemplates[index]=tempRoiImg;
        // imshow("last", carLastTemplates[index]);
        //imshow("now", carTemplates[index]);
        // imshow("org",carOrginTemplates[index]);
        
        
    }
    else
    {
        //rectangle( img,Point( carLastX[index]-carWidth[index]/2,carLastY[index]-carHeight[index]/2 ), Point( carLastX[index] + carWidth[index]*1.5 , carY[index] + carHeight[index]*1.5 ), CV_RGB(255, 0, 0), 3 );
        
        ///rectangle( img, match, Point( match.x + carWidth[index] , match.y + carHeight[index] ), CV_RGB(0, 255, 0), 1 );
        //
        // rectangle( img, Point(carLastX[index],carLastY[index]), Point( carLastX[index] + carWidth[index] , carLastY[index]+ carHeight[index] ), CV_RGB(0, 155, 0), 1 );
        //
        //        string displayInfor=to_string(index)+"research";
        //        putText(img, displayInfor.c_str(), cv::Point( carLastX[index] + carWidth[index] , carY[index] + carHeight[index] ),
        //                FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
        //        carTemplates[index]=carLastTemplates[index];
        
        
        Mat searchResult  =  TplMatch( img, carLastTemplates[index],index,carX[index],carY[index],carWidth[index],carHeight[index], tmpRoiX,tmpRoiY,tmpRoiW,tmpRoiH);
        Point searchMatch =  minmax( searchResult );
        //re position roi
        searchMatch.x+=(carX[index]-tmpRoiScaleW/2);
        searchMatch.y+=(carY[index]-tmpRoiScaleH/2);
        //p framePuse=true;
        if(abs(carLastX[index]-searchMatch.x)<carWidth[index]/2 &&abs(carLastY[index]-searchMatch.y)<carHeight[index]/2)
        {
            //cout<<"normal"<<endl;
            //rectangle( img,Point( carLastX[index]-carWidth[index]/2,carLastY[index]-carHeight[index]/2 ), Point( carLastX[index] + carWidth[index]*1.5 , carY[index] + carHeight[index]*1.5 ), CV_RGB(255, 0, 0), 2 );
            
            Mat tempExhange;
            carLastTemplates[index].copyTo(tempExhange);
            //// carLastX[index]=searchMatch.x;
            // carLastY[index]=searchMatch.y;
            
            carTemplates[index]=tempExhange;
            
            //imshow("after", carTemplates[index]);
            // framePuse=true;
            
        }
        else
        {
            // rectangle( img,  searchMatch, Point( searchMatch.x + carWidth[index] , searchMatch.y + carHeight[index] ), CV_RGB(255, 0, 0), 0.5 );
            rectangle( img,  Point( tmpRoiX,tmpRoiY ), Point( tmpRoiX+tmpRoiW,tmpRoiY+tmpRoiH ), CV_RGB(255, 155, 155), 1 );
            string displayInfor=to_string(index)+" lost";
            putText(img, displayInfor.c_str(), cv::Point( carLastX[index] + carWidth[index]*3 , carY[index] + carHeight[index]*3 ),
                    FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
            cout<<"lost"<<endl;
            framePuse=true;
            carStatus[index]=2;
        }
        
        //        Mat searchResult  =  TplMatch( img, carTemplates[index],index,carX[index],carY[index],carWidth[index],carHeight[index], tmpRoiX,tmpRoiY,tmpRoiW,tmpRoiH);
        //        Point searchmatch =  minmax( result );
        
        //        Mat tmpRoiImg = img(rect);
        //        cout<<"x..............."<<carLastX[index]<<endl;
        //        cout<<"y..............."<<carLastY[index]<<endl;
        //
        //        roiImg.copyTo(carTemplates[index]);
        //        carLastTemplates[index].copyTo(carTemplates[index]);
        //                 carX[index]= carLastX[index];
        //                 carY[index]= carLastY[index];
        //                 carLastX[index]=match.x;
        //        //         carLastY[index]=match.y;
        //framePuse=true;
        
        
        
        //carStatus[index]=4;
        
        
        
        
    }
    // xCurrent=match.x-xLast;
    // yCurrent=match.y-yLast;
    
    // //    if(match.x-xLoc!=0 ||match.y-yLoc!=0)
    // //    {
    //         outfile<<endl;
    //         outfile<<frameNumberString<<","<<timeFrame<<","<<match.x-xLoc<<","<<match.y-yLoc<<","<<xCurrent<<","<<yCurrent<<",";
    //
    // //    }
    
    
    //    /// latest match is the new template
    //    if(stoi(frameNumberString)%30==0)
    //    {
    //        cout<<"update"<<endl;
    //            Rect ROI = cv::Rect( match.x, match.y, carWidth[index], carHeight[index]);
    //            roiImg = img( ROI );
    //            roiImg.copyTo(carTemplates[index]);
    //    }
    
    // imshow( "roiImg", roiImg ); //waitKey(0);
}










//
/////------- template matching -----------------------------------------------------------------------------------------------
//
//Mat objectTplMatch( Mat &img, Mat &mytemplate )
//{
//    Mat result;
//
//    matchTemplate( img, mytemplate, result, CV_TM_SQDIFF_NORMED );
//    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );
//
//    return result;
//}
//




void objectTrack(int index)
{
    if (select_flag)
    {
        //roiImg.copyTo(mytemplate);
        //         select_flag = false;
        go_fast = true;
    }
    int tmpRoiX=objectX[index]-objectWidth[index];
    int tmpRoiY=objectY[index]-objectHeight[index];
    int tmpRoiW=objectWidth[index]*3;
    int tmpRoiH=objectHeight[index]*3;
    //imshow( "mytemplate", objectTemplates[index] ); waitKey(0);
    Mat result  =  TplMatch( img,objectTemplates[index],index,objectX[index],objectY[index],objectWidth[index],objectHeight[index], tmpRoiX,tmpRoiY,tmpRoiW,tmpRoiH);
    //Mat result  =  objectTplMatch( img, objectTemplates[index]);
    Point match =  minmax( result );
    
    
    
    match.x+=(objectX[index]-objectWidth[index]);
    match.y+=(objectY[index]-objectHeight[index]);
    
    
    if(objectStatus[index]==0)
    {
        objectStatus[index]=1;
        objectLastX[index]=match.x;
        objectLastY[index]=match.y;
    }
    rectangle( img, match, Point( match.x + objectWidth[index] , match.y + objectHeight[index] ), CV_RGB(255, 255, 255), 0.5 );
    //rectangle( img, Point( tmpRoiX , tmpRoiY  ), Point( tmpRoiX + tmpRoiW , tmpRoiY + tmpRoiH ), CV_RGB(0, 255, 255), 0.5 );
    //std::cout << "match: " << match << endl;
    objectLastX[index]=objectX[index];
    objectLastY[index]= objectY[index];
    objectX[index]=match.x;
    objectY[index]=match.y;
    
    Mat inpaintMask = Mat::zeros(img.size(), CV_8U);
    rectangle( inpaintMask, match, Point( match.x + objectWidth[index] , match.y + objectHeight[index] ), CV_RGB(255, 255, 255), -1 );
    //rectangle( inpaintMask,  Point( point1.x, point1.y), Point( x,y ), CV_RGB(255, 255, 255),-1 );
    
    inpaint(img, inpaintMask, img, 1, INPAINT_TELEA);
    // imshow("inpainted image", inpainted);
    
    //    /// latest match is the new template
    //    Rect ROI = cv::Rect( match.x, match.y, mytemplate.cols, mytemplate.rows );
    //    roiImg = img( ROI );
    //    roiImg.copyTo(mytemplate);
    //    imshow( "roiImg", roiImg ); //waitKey(0);
}














///------- MouseCallback function ------------------------------------------------------------------------------------------

void mouseHandler(int event, int x, int y, int flags, void *param)
{
    if (event == CV_EVENT_LBUTTONDOWN && !drag)
    {
        /// left button clicked. ROI selection begins
        point1 = Point(x, y);
        drag = 1;
        cout<<"start dray"<<x<<"|"<<y<<endl;
        
    }
    if (event == CV_EVENT_MOUSEMOVE && !drag)
    {
         cout<<"mouse X: "<<x<<" y: "<<y<<endl;
    }
    if (event == CV_EVENT_MOUSEMOVE && drag)
    {
        /// mouse dragged. ROI being selected
        Mat img1 = img.clone();
        point2 = Point(x, y);
        rectangle(img1, point1, point2, CV_RGB(255, 0, 0), 0.5, 8, 0);
        cout<<"keep dray"<<x<<"|"<<y<<endl;
        imshow("image", img1);
    }
    
    if (event == CV_EVENT_LBUTTONUP && drag)
    {
        point2 = Point(x, y);
        cout<<"finishe dray"<<x<<"|"<<y<<endl;
        if(x - point1.x>0&&y - point1.y>0)
        {
            rect = Rect(point1.x, point1.y, x - point1.x, y - point1.y);
            roix=point1.x;
            roiy= point1.y;
            roiWidth=x - point1.x;
            roiHeight=y - point1.y;
        }
        else{
            cout<<"selsect again"<<endl;
        }
        
        //rectangle( img,  Point( point1.x, point1.y), Point( x,y ), CV_RGB(0, 255, 0), 0.5 );
        
        
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
    
    
    if(argc>2)
    {
        Mat framePre;
        for( int x = 0; x < stoi(argv[2]); x++ ) {
            int temp=x;
             int tempamount=stoi(argv[2]);
            cap>>framePre;
            cout<<"loading "<<temp<<" / "<<tempamount<<" frame "<<endl;
        }
    }
    
    
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
    outfile<<"frameNUM"<<","<<"time(s)"<<","<< "id"<<","<<"width"<<","<<"height"<<","<<"x"<<","<<"y"<<","<<"status"<<","<<"type"<<",";
    outfile<<endl;
    if ( !cap.isOpened() )
    {   cout << "Unable to open video file" << endl;    return -1;    }
    /*
     /// Set video to 320x240
     cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
     cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);*/
    
    cap >> img;
    
    //    screenOgrinWidth=img.cols;
    //    screenOgrinHeight=img.rows;
    //    screenWidthScale=screenOgrinWidth/screenWidth;
    //    screenHeightScale=screenOgrinHeight/screenHeight;
    //    screenWidth=img.cols;
    //    screenHeight=img.rows;
    //    cout << screenWidthScale<<endl;
    //    cout << img.rows<<endl;
    //   // resize(img, img, Size(screenWidth, screenHeight));
    //    //GaussianBlur( img, img, Size(7,7), 3.0 );
    //    namedWindow("image", WINDOW_NORMAL);
    //    resizeWindow("image", img.cols / 2, img.rows / 2);
    
    resize(img, img, Size(screenWidth, screenHeight), 0, 0, INTER_CUBIC);
    imshow( "image", img );
    
    while(1)
    {
        //cout<<framePuse<<endl;
        
        if(!framePuse)
        {
            
            
            
            //            for(int i=0;i<carTemplates.size();i++)
            //            {
            //
            //                if(carStatus[i]==2)
            //                {
            //                    carStatus[i]==4;
            //                }
            //            }
            
            
            //      //cvSetMouseCallback( "image", mouseHandler, NULL );
            //        continue;
            cap >> img;
            resize(img, img, Size(screenWidth, screenHeight), 0, 0, INTER_CUBIC);
            //resize(img, img, Size(screenWidth, screenHeight));
            //GaussianBlur( img, img, Size(7,7), 3.0 );
            stringstream ss;
            stringstream st;
            stringstream fps;
            stringstream tfn;
            rectangle(img, cv::Point(0, 2), cv::Point(450,20),
                      cv::Scalar(255,255,255), -1);
            ss << cap.get(CAP_PROP_POS_FRAMES);
            fps << cap.get(CAP_PROP_FPS);
            st << cap.get( CAP_PROP_POS_MSEC);
            tfn << cap.get( CAP_PROP_FRAME_COUNT);
            frameNumberString = ss.str();
            fpsNumberString = fps.str();
            timeNumberString = st.str();
            totalFrameNumberString=tfn.str();
            timeFrame=stod(timeNumberString)/1000;
            //  string timeNumberString = st.str();
            putText(img, totalFrameNumberString.c_str(), cv::Point(0, 15),
                    FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
            putText(img, frameNumberString.c_str(), cv::Point(85, 15),
                    FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
            putText(img, fpsNumberString.c_str(), cv::Point(190, 15),
                    FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
            putText(img, to_string(timeFrame), cv::Point(290, 15),
                    FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
            
            
            
            
            for(int i=0;i<objectTemplates.size();i++)
            {
                
                //                if(carStatus[i]==2)
                //                {
                //                    carStatus[i]=4;
                //                    outfile<<frameNumberString<<","<<timeFrame<<","<< i<<","<<carWidth[i]<<","<<carHeight[i]<<","<<carX[i]<<","<<carY[i]<<","<<carStatus[i]<<",";
                //                    outfile<<endl;
                //                }
                //cout<<"start track "<<i<<" car"<<endl;
                if(objectStatus[i]!=2&&objectStatus[i]!=4&&objectStatus[i]!=3&&!objectTemplates[i].empty())
                {
                    // cout<<"start check object"<<endl;
                    objectTrack(i);
                    
                    
                    
                    
                    
                    outfile<<frameNumberString<<","<<timeFrame<<","<< i<<","<<objectWidth[i]<<","<<objectHeight[i]<<","<<objectLastX[i]<<","<<objectLastY[i]<<","<<objectStatus[i]<<","<<"object"<<",";;
                    outfile<<endl;
                }
                
            }
            
            
            
            //imshow( "iaaftere", img );
            
            for(int i=0;i<carTemplates.size();i++)
            {
                
                if(carStatus[i]==2)
                {
                    carStatus[i]=4;
                    outfile<<frameNumberString<<","<<timeFrame<<","<< i<<","<<carWidth[i]<<","<<carHeight[i]<<","<<carX[i]<<","<<carY[i]<<","<<carStatus[i]<<","<<"car"<<",";
                    outfile<<endl;
                }
                //cout<<"start track "<<i<<" car"<<endl;
                else if(carStatus[i]!=2&&carStatus[i]!=4&&carStatus[i]!=5&&carStatus[i]!=3&&!carTemplates[i].empty())
                {
                    track(i);
                    outfile<<frameNumberString<<","<<timeFrame<<","<< i<<","<<carWidth[i]<<","<<carHeight[i]<<","<<carX[i]<<","<<carY[i]<<","<<carStatus[i]<<","<<"car"<<",";
                    outfile<<endl;
                }
                
            }
            
            ////object tracking
            
            
            
            
        }
        else{
            cvSetMouseCallback( "image", mouseHandler, NULL );
            frameTrack=false;
            for(int i=0;i<carTemplates.size();i++)
            {
                string imname="car:"+to_string(i);
                if(carStatus[i]!=2&&carStatus[i]!=3&&carStatus[i]!=4&&carStatus[i]!=5)
                {
                    rectangle(img, cv::Point(carX[i], carY[i]), cv::Point(carX[i]+carWidth[i],carY[i]+carHeight[i]),
                              cv::Scalar(0,255,0), 1);
                }
                //cout<<"car:"<<i<<"x:"<<carX[i]<<"y:"<<carY[i]<<"width"<<carWidth[i]<<"status"<<carStatus[i]<<endl;
                
                //                 imshow(imname, carTemplates[i]);
                //                 moveWindow(imname, 20,50*i);
                //                 rectangle(img, cv::Point(carX[i], carY[i]), cv::Point(carX[i]+carWidth[i],carY[i]+carHeight[i]),
                //                           cv::Scalar(0,255,0), 1);
            }
            
            
            
            
            
            for(int i=0;i<objectTemplates.size();i++)
            {
                string imname="object:"+to_string(i);
                if(objectStatus[i]!=2&&objectStatus[i]!=3&&objectStatus[i]!=4)
                {
                    rectangle(img, cv::Point(objectX[i], objectY[i]), cv::Point(objectX[i]+objectWidth[i],objectY[i]+objectHeight[i]),
                              cv::Scalar(125,125,125), 1);
                }
                //cout<<"car:"<<i<<"x:"<<carX[i]<<"y:"<<carY[i]<<"width"<<carWidth[i]<<"status"<<carStatus[i]<<endl;
                
                //                 imshow(imname, carTemplates[i]);
                //                 moveWindow(imname, 20,50*i);
                //                 rectangle(img, cv::Point(carX[i], carY[i]), cv::Point(carX[i]+carWidth[i],carY[i]+carHeight[i]),
                //                           cv::Scalar(0,255,0), 1);
            }
            
            
            
            
            
        }
        // else
        // {
        
        //  resize(img, img, Size(1920, 1080));
        if ( img.empty() )
            break;
        
        
        // GaussianBlur( img, img, Size(7,7), 3.0 );
        
        // stop the program if no more images
        // Flip the frame horizontally and add blur
        //cv::flip( img, img, 1 );
        
        
        //         if ( rect.width == 0 && rect.height == 0 )
        //
        //         else
        
        //        if(frameTrack)
        //
        //        {
        //           track(0);
        //        }
        ////
        imshow("image", img);
        //char key = waitKey(0);
        
        //imshow("image2", img);
        //  waitKey(100);   k = waitKey(75);
        k = waitKey(go_fast ? 30 : 10000);
        
        
        if(k == 'p')
        {
            framePuse=!framePuse;
        }
        //        else if(k=='t')
        //        {
        //            frameTrack!=frameTrack;s
        //        }
        else if(k=='t')
        {
            bool addNewcar=false;
            if(mytemplate.empty())
            {
                cout<<"please select"<<endl;
            }
            else
            {
                
                if(!carTemplates.empty())
                {
                    for(int i=0;i<carTemplates.size();i++)
                    {
                        
                        if(carStatus[i]==2)
                        {
                            double distance=0;
                            cout<<"find lost cat  "<<i<<" car"<<endl;
                            distance=findDistanceBetweenTwoPoint(roix,roiy,carLastX[i],carLastY[i]);
                            cout<<"distance between center for  "<<i<<"is "<<distance<<" to new point"<<endl;
                            if(distance<carWidth[i])
                            {
                                cout<<"new point assign to lost car  "<<i<<"is "<<distance<<" to new point"<<endl;
                                carStatus[i]=1;
                                carTemplates[i]=mytemplate;
                                carX[i]=roix;
                                carY[i]=roiy;
                                carLastX[i]=roix;
                                carLastY[i]=roiy;
                                carWidth[i]=roiWidth;
                                carHeight[i]=roiHeight;
                                addNewcar=true;
                                
                            }
                        }
                        else if(carStatus[i]==1)
                        {
                            double distance=0;
                            
                            distance=findDistanceBetweenTwoPoint(roix,roiy,carLastX[i],carLastY[i]);
                            
                            if(distance<carWidth[i]/2)
                            {
                                cout<<"re track car "<<endl;
                                carStatus[i]=1;
                                carTemplates[i]=mytemplate;
                                carX[i]=roix;
                                carY[i]=roiy;
                                carLastX[i]=roix;
                                carLastY[i]=roiy;
                                carWidth[i]=roiWidth;
                                carHeight[i]=roiHeight;
                                addNewcar=true;
                                
                            }
                        }
                        
                        
                    }
                }
                
                if(!addNewcar)
                {
                    cout<<"add car:"<<carTemplates.size()<<endl;
                    carTemplates.push_back(mytemplate);
                    carLastTemplates.push_back(mytemplate);
                    carOrginTemplates.push_back(mytemplate);
                    carX.push_back(roix);
                    carY.push_back(roiy);
                    carWidth.push_back(roiWidth);
                    carHeight.push_back(roiHeight);
                    carLastX.push_back(roix);
                    carLastY.push_back(roiy);
                    carStatus.push_back(0);//0=tag 1=tracking 2=lost 3= finised
                }
                
                
                
                
                
                
            }
            
        }
        
        
        
        
        
        
        
        
        
        else if(k=='a')
        {
            cout<<"add object:"<<objectTemplates.size()<<endl;
            objectTemplates.push_back(mytemplate);
            //carLastTemplates.push_back(mytemplate);
            //carOrginTemplates.push_back(mytemplate);
            objectX.push_back(roix);
            objectY.push_back(roiy);
            objectWidth.push_back(roiWidth);
            objectHeight.push_back(roiHeight);
            objectLastX.push_back(roix);
            objectLastY.push_back(roiy);
            objectStatus.push_back(0);//0=tag 1=tracking 2=lost 3= finised 4=lost and cannot find 5=removed
        }
        
        
        else if(k=='d')
        {
            for(int i=0;i<carTemplates.size();i++)
            {
                
//                if(carStatus[i]==2)
//                {
                    double distance=0;
                    cout<<"find lost cat  "<<i<<" car"<<endl;
                    distance=findDistanceBetweenTwoPoint(roix,roiy,carLastX[i],carLastY[i]);
                    //cout<<"distance between center for  "<<i<<"is "<<distance<<" to new point"<<endl;
                    if(distance<carWidth[i])
                    {
                        cout<<"remove  "<<i<<endl;
                        carStatus[i]=5;
                        outfile<<frameNumberString<<","<<timeFrame<<","<< i<<","<<carWidth[i]<<","<<carHeight[i]<<","<<carX[i]<<","<<carY[i]<<","<<carStatus[i]<<","<<"car"<<",";
                        outfile<<endl;
//                        carTemplates[i]=mytemplate;
//                        carX[i]=roix;
//                        carY[i]=roiy;
//                        carLastX[i]=roix;
//                        carLastY[i]=roiy;
//                        carWidth[i]=roiWidth;
//                        carHeight[i]=roiHeight;
                        //addNewcar=true;
                        
//                    }
                }
                
                
            }
        }
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        else if(k=='r')
        {
            if(carTemplates.size()>0)
            {
                int index=carTemplates.size()-1;
                cout<<"remove last add cat car:"<<carTemplates.size()<<endl;
                rectangle(img, cv::Point(carX[index], carY[index]), cv::Point(carX[index]+carWidth[index],carY[index]+carHeight[index]),
                          cv::Scalar(0,0,255), 1);
                String windowName="car:"+to_string(index);
                destroyWindow(windowName);
                carTemplates.pop_back();
                carLastTemplates.pop_back();
                carX.pop_back();
                carY.pop_back();
                carWidth.pop_back();
                carHeight.pop_back();
                carOrginTemplates.pop_back();
                carLastX.pop_back();
                carLastY.pop_back();
                carStatus.pop_back();//0=tag 1=tracking 2=lost 3= finised 4=lost&cannot find 5 =removed;
            }
            else{
                cout<<"no more car:"<<carTemplates.size()<<endl;
            }
            
        }
        else if (k == 27)
        {
            break;
            
        }
        
    }
    //delete capture object
    cap.release();
    outfile.close();
    return 0;
}

