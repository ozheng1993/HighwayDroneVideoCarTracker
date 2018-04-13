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
int screenWidth=1920;
int screenHeight=1080;
int startLine=1920;
int endLine=1700;
ofstream outfile;
string frameNumberString;
string fpsNumberString;
string timeNumberString;
Mat mytemplate;
Mat mytemplate2;
double timeFrame=0.0;
bool framePuse=true;
bool frameTrack=true;
vector<Mat> carTemplates;      // array of 10 images
vector<int> carX;
vector<int> carY;
vector<int> carLastX;
vector<int> carLastY;
vector<int> carWidth;
vector<int> carHeight;
vector<int> carStatus;//0=tag 1=tracking 2=lost 3= finised


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
   // imshow(windowname, Roi);
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
   // cout<<"carX[index]"<<carX[index]<<endl;
    if(carX[index]+carWidth[index]>=endLine||carX[index]<=carWidth[index])
    {
        cout<<"finished"<<endl;
        carStatus[index]=3;
       
    }
    //finishe dete
    if((carX[index]+carWidth[index]*3)<endLine&&(carX[index]-carWidth[index]*2)>0)
    {
       // cout<<"normal roi"<<endl;
        tmpRoiX=carX[index]-carWidth[index]*2;
        tmpRoiY=carY[index]-carHeight[index]*2;
        tmpRoiW=carWidth[index]*5;
        tmpRoiH=carHeight[index]*5;
       // cout<<"tmpRoiX"<<tmpRoiX<<endl;
        //region_of_interest = Rect(x-width*2,y-height*2, width*5, height*5);
    }
    else
    {
        if((carX[index]-carWidth[index]*2)<0)
        {
            //cout<<"carX[carX[index]-carWidth[index]*2]"<<carX[index]-carWidth[index]*2<<endl;
            //cout<<"left roi"<<endl;
            tmpRoiX=0;
            tmpRoiY=carY[index]-carHeight[index]*2;
            tmpRoiW=carX[index]+carWidth[index]*2;
            tmpRoiH=carHeight[index]*5;
            
           // region_of_interest = Rect(x-width*2,y-height*2, screenWidth-(x-width*2), height*5);
        }
        else if((carX[index]+carWidth[index]*3)>=endLine)
        {
           // cout<<"right roi"<<endl;
          
            tmpRoiX=carX[index]-carWidth[index]*2;
            tmpRoiY=carY[index]-carHeight[index]*2;
            tmpRoiW=endLine-carX[index]+carWidth[index]*2;
            tmpRoiH=carHeight[index]*5;
            //region_of_interest = Rect(1,y-height*2, width*7, height*5);
            
        }
        
    }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    //     imshow( "mytemplate", mytemplate ); waitKey(0);
    Mat result  =  TplMatch( img, carTemplates[index],index,carX[index],carY[index],carWidth[index],carHeight[index], tmpRoiX,tmpRoiY,tmpRoiW,tmpRoiH);
    Point match =  minmax( result );

    //re position roi
    match.x+=(carX[index]-carWidth[index]*2);
    match.y+=(carY[index]-carHeight[index]*2);
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
    
    if(abs(carLastX[index]-match.x)<carWidth[index]/5&&abs(carLastY[index]-match.y)<carHeight[index]/2)

     {
		carX[index]=match.x;
       carY[index]= match.y;
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
     }
     else
     {
       rectangle( img,Point( carLastX[index],carY[index] ), Point( carLastX[index] + carWidth[index] , carY[index] + carHeight[index] ), CV_RGB(0, 0, 255), 0.5 );
         rectangle( img,  Point( tmpRoiX,tmpRoiY ), Point( tmpRoiX+tmpRoiW,tmpRoiY+tmpRoiH ), CV_RGB(0, 0, 255), 0.5 );
       string displayInfor=to_string(index)+"lost,please retag";
        putText(img, displayInfor.c_str(), cv::Point(match.x + carWidth[index]*2 , match.y + carHeight[index]),
                 FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
//         carX[index]= carLastX[index];
//         carY[index]= carLastY[index];
//         carLastX[index]=match.x;
//         carLastY[index]=match.y;
                framePuse=true;
                carStatus[index]=2;
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
     outfile<<"frameNUM"<<","<<"time(s)"<<","<< "id"<<","<<"width"<<","<<"height"<<","<<"x"<<","<<"y"<<","<<"status"<<",";
    outfile<<endl;
    if ( !cap.isOpened() )
    {   cout << "Unable to open video file" << endl;    return -1;    }
    /*
     /// Set video to 320x240
     cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
     cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);*/
    
    cap >> img;
    
    //GaussianBlur( img, img, Size(7,7), 3.0 );
   imshow( "image", img );
    resize(img, img, Size(screenWidth, screenHeight));
    while(1)
    {
        //cout<<framePuse<<endl;
   
         if(!framePuse)
         {
             
     //      //cvSetMouseCallback( "image", mouseHandler, NULL );
     //        continue;
               cap >> img;
            resize(img, img, Size(screenWidth, screenHeight));
             stringstream ss;
             stringstream st;
             stringstream fps;
             rectangle(img, cv::Point(10, 2), cv::Point(450,20),
                       cv::Scalar(255,255,255), -1);
             ss << cap.get(CAP_PROP_POS_FRAMES);
             fps << cap.get(CAP_PROP_FPS);
             st << cap.get( CAP_PROP_POS_MSEC);
             frameNumberString = ss.str();
             fpsNumberString = fps.str();
             timeNumberString = st.str();
             timeFrame=stod(timeNumberString)/1000;
             //  string timeNumberString = st.str();
             putText(img, frameNumberString.c_str(), cv::Point(15, 15),
                     FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
             putText(img, fpsNumberString.c_str(), cv::Point(70, 15),
                     FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
             putText(img, to_string(timeFrame), cv::Point(190, 15),
                     FONT_HERSHEY_SIMPLEX, 0.5 , cv::Scalar(255,0,0));
             for(int i=0;i<carTemplates.size();i++)
             {
                 //cout<<"start track "<<i<<" car"<<endl;
                 if(carStatus[i]!=2&&carStatus[i]!=3&&!carTemplates[i].empty())
                 {
                    track(i);
                     outfile<<frameNumberString<<","<<timeNumberString<<","<< i<<","<<carWidth[i]<<","<<carHeight[i]<<","<<carX[i]<<","<<carY[i]<<","<<carStatus[i]<<",";
                     outfile<<endl;
                 }
                 
                 
             }
           
         }
         else{
              cvSetMouseCallback( "image", mouseHandler, NULL );
             frameTrack=false;
             for(int i=0;i<carTemplates.size();i++)
             {
                 string imname="car:"+to_string(i);
                 if(carStatus[i]!=2&&carStatus[i]!=3)
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
//            frameTrack!=frameTrack;
//        }
        else if(k=='a')
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
                            cout<<"distance between center is  "<<i<<"is "<<distance<<" to new point"<<endl;
                            if(distance<carWidth[i]/2)
                            {
                                cout<<"new point assign to lost car  "<<i<<"is "<<distance<<" to new point"<<endl;
                                carStatus[i]=1;
                                carTemplates[i]=mytemplate;
                                carX[i]=roix;
                                carY[i]=roiy;
                                carLastX[i]=roix;
                                carLastY[i]=roiy;
                                addNewcar=true;

                            }
                        }


                    }
                }

                
                
                
                
                
                
                if(!addNewcar)
                {
                    cout<<"add car:"<<carTemplates.size()<<endl;
                    carTemplates.push_back(mytemplate);
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
                carX.pop_back();
                carY.pop_back();
                carWidth.pop_back();
                carHeight.pop_back();
                carLastX.pop_back();
                carLastY.pop_back();
                carStatus.pop_back();//0=tag 1=tracking 2=lost 3= finised 4=search 5 =removed;
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
