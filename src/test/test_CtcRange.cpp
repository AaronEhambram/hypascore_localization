#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include "hypascore_localization/contractors/CtcRange.hpp"

int box_x_max = 400.0;
int box_x_slider = 200.0;
int box_y_max = 400.0;
int box_y_slider = 200.0;
int box_x_uncertainty_max = 100.0;
int box_x_uncertainty_slider = 1.0;
int box_y_uncertainty_max = 100.0;
int box_y_uncertainty_slider = 1.0;
int range_max = 200.0;
int range_slider = 50.0;
std::string window_name = "Range Contractor";

void point_to_pixel(ibex::IntervalVector& hull, double& pixel_per_m, ibex::Vector& point, cv::Point2f& pixel)
{
  ibex::Vector tl(2); 
  tl[0] = hull[0].lb(); 
  tl[1] = hull[1].ub();  

  ibex::Vector offset_point = point-tl; 
  pixel.x = offset_point[0]*pixel_per_m; 
  pixel.y = -offset_point[1]*pixel_per_m;
}

std::pair<ibex::Vector,ibex::Vector> get_rect_corner_points(ibex::IntervalVector& box)
{
  ibex::Vector bl(2); 
  bl[0] = box[0].lb();
  bl[1] = box[1].lb(); 

  ibex::Vector tr(2); 
  tr[0] = box[0].ub();
  tr[1] = box[1].ub(); 

  return std::make_pair(bl,tr); 
}


void draw_box(cv::Mat& im, ibex::IntervalVector pose, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  std::pair<ibex::Vector,ibex::Vector> pose_box_corner_points = get_rect_corner_points(pose);
  cv::Point2f pose_bl,pose_tr; 
  point_to_pixel(hull, pixel_per_m,  pose_box_corner_points.first, pose_bl);
  point_to_pixel(hull, pixel_per_m,  pose_box_corner_points.second, pose_tr);
  cv::rectangle(im,pose_bl,pose_tr,color,thickness);
}

void draw_circle(cv::Mat& im, ibex::IntervalVector& c, ibex::Interval& r, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  ibex::Vector c_mid = c.mid(); 
  cv::Point2f c_mid_pix;
  point_to_pixel(hull, pixel_per_m,  c_mid, c_mid_pix);

  double r_lb_pix = r.ub()*pixel_per_m;

  cv::circle(im,c_mid_pix,r_lb_pix,color,thickness);
}

static void on_trackbar( int, void* )
{
  ibex::IntervalVector box(2); 
  box[0] = (double)(box_x_slider-200)/10.0;
  box[1] = (double)(box_y_slider-200)/10.0;
  box[0].inflate(box_x_uncertainty_slider/100.0);
  box[1].inflate(box_y_uncertainty_slider/100.0);

  ibex::Interval r(0,range_slider/100.0);
  ibex::IntervalVector c(2); 
  c[0] = 0; 
  c[1] = 0; 

  ibex::IntervalVector hull(2); 
  hull.set_empty(); 
  ibex::IntervalVector measure(2);
  ibex::IntervalVector r_vector(2); r_vector[0] = r | -r; r_vector[1] = r | -r;  
  hull = box | c+r_vector; 
  hull.inflate(0.1);

  int large_im_width = 800;
  int large_im_height = 800;
  cv::Scalar background_color(255,255,255); 
  cv::Mat im(cv::Size(large_im_width,large_im_height),CV_8UC3, background_color);
  double pixel_per_m = std::min(large_im_width/hull[0].diam(),large_im_height/hull[1].diam());

  draw_box(im,box,hull,pixel_per_m,cv::Scalar(0,0,0),1); 
  draw_circle(im,c,r,hull,pixel_per_m,cv::Scalar(0,0,0),1);

  codac::CtcRange ctc; 
  ctc.contract(box,c,r);

  draw_box(im,box,hull,pixel_per_m,cv::Scalar(0,255,0),1);

  cv::imshow(window_name,im);
}

int main()
{
  cv::namedWindow(window_name, cv::WINDOW_AUTOSIZE);

  char TrackbarName[50];
  snprintf( TrackbarName, sizeof(TrackbarName), "t_x x %f", 0.1 );
  cv::createTrackbar( TrackbarName, window_name, &box_x_slider, box_x_max, on_trackbar );

  char TrackbarName1[50];
  snprintf( TrackbarName1, sizeof(TrackbarName1), "t_y x %f", 0.1 );
  cv::createTrackbar( TrackbarName1, window_name, &box_y_slider, box_y_max, on_trackbar );

  char TrackbarName4[50];
  snprintf( TrackbarName4, sizeof(TrackbarName4), "x uncertainty x %f", 0.1 );
  cv::createTrackbar( TrackbarName4, window_name, &box_x_uncertainty_slider, box_x_uncertainty_max, on_trackbar );

  char TrackbarName5[50];
  snprintf( TrackbarName5, sizeof(TrackbarName5), "y uncertainty x %f", 0.1 );
  cv::createTrackbar( TrackbarName5, window_name, &box_y_uncertainty_slider, box_y_uncertainty_max, on_trackbar );

  char TrackbarName2[50];
  snprintf( TrackbarName2, sizeof(TrackbarName2), "range x %f", 0.1 );
  cv::createTrackbar( TrackbarName2, window_name, &range_slider, range_max, on_trackbar );

  on_trackbar(0,0);

  cv::waitKey(0); 
}