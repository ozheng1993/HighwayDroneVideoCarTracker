# UAV Car tracker wiht openCV #
This repository contains OpenCV code and documents.
## How to download this code ##
` git clone git@github.com:ozheng1993/OpenCVHighwayCarTracker.git `.

## How to open in Ubuntu ##
1. Press ` Ctrl+Alt+T ` to open the Terminal. 
2. Nav to program root folder.
    1. run `ls ` to check where you at.
    2. run ` cd  folder name` to entry folder.
    3. for ou's friend run `cd Desktop/openCVTracker`.
3. Compile and run `` g++ -std=c++11 main.cpp `pkg-config --libs --cflags opencv` -o main ``.
4. Run `./ main videofilename\.
     1. add number after to skip frame.
## How to open in Mac ##
## For car ##
* Press `t`  to add car to the tracker.
* Press `r`  to remove the last car to the tracker.
* Press `p`  to puse/resume.
## For Object ##
* Press `a`  to add to object list. ps:this action will remove object from the frame too.
## Where is you data ##
Go to the root folder and open the folder named as `file`.
