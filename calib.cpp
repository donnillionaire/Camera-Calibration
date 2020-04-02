#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;

void help(char *argv[]){

}


int main (int argc, char* argv[]){
  int n_boards = 0;
  float image_sf = 0.5f;
  float delay = 1.f;
  int board_w = 0;
  int board_h =0;


if (argc < 4 || argc > 6){
  cout<<"\n Error: Wrong number of input parameters";
  help(argv);
  return -1;
}

board_w = atoi(argv[1]);
board_h = atoi( argv[2]);
n_boards = atoi(argv[3]);


if (argc > 4 ) delay = atof(argv[4]);
if (argc > 5 ) image_sf = atof(argv[5]);


int board_n = board_w * board_h;
cv::Size board_sz = cv::Size(board_w, board_h);

//open video stream
cv::VideoCapture capture(0);
//check for any error
if (!capture.isOpened()){
  cout<<"Couldn't open the camera\n";
  help(argv);
  return -1;
}

//STORAGE ALLOCATION
//
vector<vector<cv::Point2f> > image_points;
vector<vector<cv::Point3f> > object_points;


//capture corner views: loop until we got n_boards successful
//captured(all the corners on the board are found)

double last_captured_timestamp = 0;
cv::Size image_size;

while (image_points.size() < (size_t)n_boards){
  cv::Mat image0, image;
  capture>> image0;
  image_size = image0.size();
  cv::resize(image0,
	   image,
           cv::Size(),
           image_sf, 
           image_sf,
           cv::INTER_LINEAR
);

//Finding the board
//
  vector<cv::Point2f>corners;
  bool found = cv::findChessboardCorners( image, board_sz, corners);

//Drawing the board
//
 drawChessboardCorners( image, board_sz, corners, found);


// if we get a good board, add it to our data
double timestamp = (double)clock()/ CLOCKS_PER_SEC;

  if (found && last_captured_timestamp > 1) {
     last_captured_timestamp = timestamp; 
     image ^=cv::Scalar::all(255);
     cv::Mat mcorners(corners); // do not copy the data
     
     mcorners *=(1./image_sf);
     image_points.push_back(corners);
     object_points.push_back(vector<cv::Point3_<float>>());  
//	object_points.push_back(vector<uint>());
     vector<cv::Point3f> & opts = object_points.back();
     
     opts.resize(board_n);
     for(int j = 0; j < board_n, j++;){
         opts[j]= cv::Point3f((float) (j/board_w),(float)(j%board_w), 0.f);
     }
cout<<"Collected out"<<(int)image_points.size()<<"of"<<n_boards<<"needed chessboard images"<<endl;
     
  }

cv::imshow("calibration", image); //show if we did collect the image
  if ((cv::waitKey(30) & 255) ==255) return -1;

}
//END COLLECTION WHILE LOOP
 cv::destroyWindow("Calibration");
cout<<"\n\n***CALIBRATING THE CAMERA..\n"<<endl;

//CALIBRATE CAMERA
//
cv::Mat intrinsic_matrix, distortion_coeffs;
double err = cv::calibrateCamera(
              object_points,  
              image_points,
              image_size,
              intrinsic_matrix,
              distortion_coeffs, 
              cv::noArray(),
              cv::noArray(), 
              cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT
 );


//SAVE THE INTRINSICS AND DISTORTIONS MATRICES
cout<<"** DONE!\n\n Reprojection erro is " << err<< " \nStoring Intrinsics.xml and Distortion.xml files \n\n";
cv::FileStorage fs( "intrinsics.xml", cv::FileStorage::WRITE );

fs<<"image_width"<<image_size.width<<"image_height"<<image_size.height<<distortion_coeffs;
fs.release();

//LOADIG THE MATRICES TO THEIR RESPECTIVE FILES
fs.open("intrinsics.xml", cv::FileStorage::READ);
cout<<"\n image width: "<<(int)fs["image_width"];
cout<<"\n image  height:"<<(int)fs["image_height"];

//defining the intrinsic matrices loaded
cv::Mat intrinsic_matrix_loaded, distortion_coeffs_loaded;
fs["camera_matrix"]>>intrinsic_matrix_loaded;
fs["distortion_coefficients"]>>distortion_coeffs_loaded;

//cout<<"\n intrinsic matrix:" << intrinsic_matrix_loaded;
//cout<<"\n distortion coefficients" <<distortion_coeffs_loaded< distortion_coeffs_loaded;

//BUILDING THE UNDISTORTION MAP THAT WE WILL USE FOR ALL SUBSEQUENT FRAMES

cv::Mat map1, map2;
cv::initUndistortRectifyMap(
       intrinsic_matrix_loaded,
       distortion_coeffs_loaded,
       cv::Mat(),
       intrinsic_matrix_loaded,
       image_size,
       CV_16SC2,
       map1, map2
 );

//EVENTUALLY, WE RUN THE CAMERA ON THE SCREEN, SHOWING RAW AND UNDISTORTED IMAGE

for(;;){
 cv::Mat image, image0;
 capture>>image0;
 if(image0.empty()) break;
 cv::remap(image0,
           image,
           map1,
           map2,
           cv::INTER_LINEAR,
           cv::BORDER_CONSTANT,
           cv::Scalar()
);

cv::imshow("Undistorted", image);
if((cv::waitKey(30)& 255)==27)break;
}
return 0;    
}












