#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <time.h>
#include <cstdio>



using namespace cv;



struct cornerInformation{
    float x;
    float y;
    float x3;
    float y3;
    float z3;
};

void fprintMatrix(Mat matrix, string name);
void fprintfVectorMat(vector< Mat> matrix, string name);
void fprintf2Point( vector< vector< Point2f> > Points, string name);
void fprintf3Point( vector< vector< Point3f> > Points, string name);
int frames();

int main()
{

    FileStorage fs( "calibracao.yml", FileStorage::WRITE );

    
    //////////////////////////////////////////////////////////////////////////////////////////////////
    //Set input params..
    int board_w, board_h;
    int n_boards;
    float measure=30;
    Size imageSize;

    vector< vector< Point2f> > imagePoints;
    vector< vector< Point3f> > objectPoints;

    board_w=8;
    board_h=6;
    n_boards = frames() - 1;


    fs << "frameCount" << n_boards-1;
    time_t rawtime; time(&rawtime);

    fs << "calibrationDate" << asctime(localtime(&rawtime));

 
    printf("w=%d h=%d n=%d %lfmm\n", board_w, board_h, n_boards, measure);
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////////////////////////////
    //image load
    //extraction image point and object point
    char str[100];
    for(int i=0; i< n_boards; ++i)
    {
        //image load
        sprintf(str,"p%d.jpg", i+1 );
        printf("%s\n", str);
        Mat img = imread(str);
        imageSize = Size(img.cols, img.rows);
        Mat gray;
        cvtColor(img, gray, CV_RGB2GRAY);
        vector< Point2f> corners;  

        //find chessboard corners
        bool sCorner = findChessboardCorners(gray, Size(board_w, board_h), corners);

        //if find corner success, then
        if(sCorner)
        {
            //corner point refine
            cornerSubPix(gray, corners, Size(11,11), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));
            //draw corner
            drawChessboardCorners(img, Size(board_w, board_h), corners, sCorner);
            if(corners.size() == board_w*board_h)
            {
                vector< Point2f> v_tImgPT;
                vector< Point3f> v_tObjPT;
                //save 2d coordenate and world coordinate
                for(int j=0; j< corners.size(); ++j)
                {
                    Point2f tImgPT;
                    Point3f tObjPT;

                    tImgPT.x = corners[j].x;
                    tImgPT.y = corners[j].y;

                    tObjPT.x = j%board_w*measure;
                    tObjPT.y = j/board_w*measure;
                    tObjPT.z = 0;

                    v_tImgPT.push_back(tImgPT);
                    v_tObjPT.push_back(tObjPT);     
                }
                imagePoints.push_back(v_tImgPT);
                objectPoints.push_back(v_tObjPT);
            }
        }
         // sprintf(str,"Detected%d.jpg",i+1);
         // imwrite(str,img);
        // imshow("pattern",img);
        // cvWaitKey(10);
    }
    //////////////////////////////////////////////////////////////////////////////////////////////////
   

    //////////////////////////////////////////////////////////////////////////////////////////////////
    //claibration part
    vector< Mat> rvecs, tvecs;
    Mat intrinsic_Matrix(3,3, CV_64F);
    Mat distortion_coeffs(8,1, CV_64F); 
    
    calibrateCamera(objectPoints, imagePoints, imageSize, intrinsic_Matrix, distortion_coeffs, rvecs, tvecs);
    

    fs << "intrinsic" << intrinsic_Matrix;
    fs << "distortion" << distortion_coeffs;
    //save part
    // fprintMatrix(intrinsic_Matrix, "intrinsic.txt");
    // fprintMatrix(distortion_coeffs, "distortion_coeffs.txt");

    // fprintfVectorMat(rvecs, "rotation.txt");
    // fprintfVectorMat(tvecs, "translation.txt");

    // FILE* fp=fopen("ptSize.txt","w");
    // fprintf(fp,"%d %d\n", board_w, board_h);
    // fclose(fp);


    for(int k = 0; k <= n_boards; k++)
    {
        std::string s;
        std::stringstream out;
        out << k;
        s = out.str();
        s = "p" + s + ".jpg";
        const char* a = s.c_str();
        //a << (char)s;
        std::remove(a);
    }

    return 0;
 //////////////////////////////////////////////////////////////////////////////////////////////////
}


// void fprintfVectorMat(vector< Mat> matrix, string name)
// {
//     FILE * fp;
//     fp = fopen(name.c_str() ,"w");
//     int i,j; 
//     printf("%s size %d, %d\n",name.c_str(),matrix.size(), matrix[0].cols, matrix[0].rows);
//     for(i=0; i< matrix.size(); ++i)
//     {
//         for(int j=0; j< matrix[i].rows; ++j)  
//         {
//             for(int k=0; k< matrix[i].cols; ++k)
//             {
//                 fprintf(fp,"%lf ", matrix[i].at<  double >(j,k)); 
//             }
//             fprintf(fp,"\n");
//         }
//         fprintf(fp,"\n");
//     }


//     fclose(fp);
// }

// void fprintMatrix(Mat matrix, string name)
// {
//     FILE * fp;
//     fp = fopen(name.c_str() ,"w");
//     int i,j; 
//     printf("%s size %d %d\n",name.c_str(), matrix.cols, matrix.rows);
//     for(i=0; i< matrix.rows; ++i)
//     {
//         for(j=0; j< matrix.cols; ++j)
//         {
//             fprintf(fp,"%lf ", matrix.at<  double >(i,j)); 
//         }
//         fprintf(fp,"\n");
//     }

//     fclose(fp);
// }

int frames()
{
    VideoCapture cap("calibracao.avi"); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
    {
       printf("leu n\n");
       return -1;
    } 

    int i = 1;
    int k = 0;
    
        
    while(true)
    {
        Mat frame;
        
        // get a new frame from camera
        bool success = cap.read(frame); 
        if(!success)
            break;

        if(i % 10 == 0)
        {
            std::string s;
            std::stringstream out;
            out << k;
            s = out.str();
            imwrite("p" + s + ".jpg", frame);
            k++;
        }

        if(waitKey(30) >= 0) break;
        i++;
    }

    // the camera will be deinitialized automatically in VideoCapture destructor
    return k;
}