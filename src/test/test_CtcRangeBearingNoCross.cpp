#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include <queue>
#include "hypascore_localization/contractors/CtcRangeBearingNoCross.hpp"

int pose_x_max = 400.0;
int pose_x_slider = 200.0;
int pose_y_max = 400.0;
int pose_y_slider = 200.0;
int pose_psi_max = 400.0;
int pose_psi_slider = 200.0;
int range_max = 400.0;
int range_slider = 50.0;
int bearing_max = 400.0;
int bearing_slider = 200.0;

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

std::vector<ibex::Vector> get_corner_points(ibex::IntervalVector& box)
{
  std::vector<ibex::Vector> out;  
  ibex::Vector bl(2);
  ibex::Vector tl(2);
  ibex::Vector br(2);
  ibex::Vector tr(2);

  bl[0] = box[0].lb(); bl[1] = box[1].lb(); 
  tl[0] = box[0].lb(); tl[1] = box[1].ub(); 
  br[0] = box[0].ub(); br[1] = box[1].lb(); 
  tr[0] = box[0].ub(); tr[1] = box[1].ub(); 
  out.push_back(bl); 
  out.push_back(tl);
  out.push_back(br);
  out.push_back(tr);
  return out; 
} 

void draw_box(cv::Mat& im, ibex::IntervalVector pose, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  std::pair<ibex::Vector,ibex::Vector> pose_box_corner_points = get_rect_corner_points(pose);
  cv::Point2f pose_bl,pose_tr; 
  point_to_pixel(hull, pixel_per_m,  pose_box_corner_points.first, pose_bl);
  point_to_pixel(hull, pixel_per_m,  pose_box_corner_points.second, pose_tr);
  cv::rectangle(im,pose_bl,pose_tr,color,thickness);
}

void draw_line(cv::Mat& im, ibex::Vector a, ibex::Vector b, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  cv::Point2f a_px, b_px; 
  point_to_pixel(hull, pixel_per_m, a, a_px);
  point_to_pixel(hull, pixel_per_m, b, b_px);
  cv::line(im,a_px,b_px,color,thickness);
}

void draw_point(cv::Mat& im, ibex::Vector a, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  cv::Point2f a_px, b_px; 
  point_to_pixel(hull, pixel_per_m, a, a_px);
  cv::circle(im,a_px,1,color,thickness);
}

void draw_connection_lines_between_boxes(cv::Mat& im, ibex::IntervalVector a, ibex::IntervalVector b, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  std::vector<ibex::Vector> corners_a = get_corner_points(a);
  std::vector<ibex::Vector> corners_b = get_corner_points(b);
  for(size_t i = 0; i < corners_a.size(); ++i)
  {

    draw_line(im, corners_a[i], corners_b[i], hull, pixel_per_m, color, thickness); 
  }
} 

void draw_pose(cv::Mat& im, ibex::IntervalVector pose, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  draw_box(im,pose.subvector(0,1),hull,pixel_per_m,color,thickness);

  ibex::Vector position_mid = pose.subvector(0,1).mid();

  // lower orientation bound
  double psi_lb = pose[2].lb(); 
  ibex::Vector psi_lb_p(2); 
  psi_lb_p[0] = cos(psi_lb); psi_lb_p[1] = sin(psi_lb);
  ibex::Vector a = position_mid+psi_lb_p;
  draw_line(im, position_mid, a, hull, pixel_per_m, color, thickness); 

  // lower orientation bound
  double psi_ub = pose[2].ub(); 
  ibex::Vector psi_ub_p(2); 
  psi_ub_p[0] = cos(psi_ub); psi_ub_p[1] = sin(psi_ub);
  ibex::Vector b = position_mid+psi_ub_p;
  draw_line(im, position_mid, b, hull, pixel_per_m, color, thickness);
}

void draw_range_bearing(cv::Mat& im, ibex::IntervalVector pose, ibex::Interval r, ibex::Interval b, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  ibex::Vector position_mid = pose.subvector(0,1).mid();

  ibex::Interval alpha = pose[2].mid()+b; 
  ibex::Vector alpha_lb_p1(2),alpha_lb_p2(2),alpha_ub_p1(2),alpha_ub_p2(2); 
  alpha_lb_p1[0] = r.lb()*cos(alpha.lb()); alpha_lb_p1[1] = r.lb()*sin(alpha.lb());
  alpha_lb_p2[0] = r.ub()*cos(alpha.lb()); alpha_lb_p2[1] = r.ub()*sin(alpha.lb());
  alpha_ub_p1[0] = r.lb()*cos(alpha.ub()); alpha_ub_p1[1] = r.lb()*sin(alpha.ub());
  alpha_ub_p2[0] = r.ub()*cos(alpha.ub()); alpha_ub_p2[1] = r.ub()*sin(alpha.ub());
  ibex::Vector a1 = position_mid+alpha_lb_p1;
  ibex::Vector a2 = position_mid+alpha_lb_p2;
  ibex::Vector a3 = position_mid+alpha_ub_p1;
  ibex::Vector a4 = position_mid+alpha_ub_p2;
  draw_line(im, position_mid, a2, hull, pixel_per_m, color, thickness);
  draw_line(im, position_mid, a4, hull, pixel_per_m, color, thickness);
  draw_point(im, a1, hull, pixel_per_m, color, thickness*3);
  draw_point(im, a2, hull, pixel_per_m, color, thickness*3);
  draw_point(im, a3, hull, pixel_per_m, color, thickness*3);
  draw_point(im, a4, hull, pixel_per_m, color, thickness*3); 
}

static void on_trackbar( int, void* )
{
  ibex::IntervalVector pose(3);
  pose[0] = (double)(pose_x_slider-200)/10.0; 
  pose[1] = (double)(pose_y_slider-200)/10.0;
  pose[2] = (double)(pose_psi_slider-200)/(200)*M_PI; 
  pose[0].inflate(0.5);
  pose[1].inflate(0.5);
  pose[2].inflate(0.4);

  ibex::IntervalVector a(2);
  a[0] = 10.0; 
  a[1] = 0.0; 
  a[0].inflate(0.1);
  a[1].inflate(0.1);
  
  ibex::IntervalVector b(2);
  b[0] = 0.0; 
  b[1] = 10.0; 
  b[0].inflate(0.1);
  b[1].inflate(0.1);

  ibex::Interval range = (double)(range_slider)/10.0;
  range.inflate(0.05); 

  ibex::Interval bearing = (double)(bearing_slider-200)/(200)*M_PI;
  bearing.inflate(0.02);

  ibex::IntervalVector hull(2); 
  hull.set_empty(); 
  ibex::IntervalVector measure(2);
  measure[0] = range*cos(bearing); measure[1] = range*sin(bearing);  
  hull = pose.subvector(0,1) | pose.subvector(0,1)+measure | a | b; 
  hull.inflate(1.0);

  int large_im_width = 800;
  int large_im_height = 800;
  cv::Scalar background_color(255,255,255); 
  cv::Mat im(cv::Size(large_im_width,large_im_height),CV_8UC3, background_color);
  double pixel_per_m = std::min(large_im_width/hull[0].diam(),large_im_height/hull[1].diam());

  draw_pose(im, pose, hull, pixel_per_m, cv::Scalar(0,0,0), 1);
  draw_range_bearing(im, pose, range, bearing, hull, pixel_per_m, cv::Scalar(255,0,0), 1); 
  draw_box(im,a,hull,pixel_per_m,cv::Scalar(0,255,0),1); 
  draw_box(im,b,hull,pixel_per_m,cv::Scalar(0,255,0),1);
  draw_connection_lines_between_boxes(im, a, b, hull, pixel_per_m, cv::Scalar(0,255,0), 1);

  codac::CtcRangeBearingNoCross ctc; 
  ctc.contract(pose,range,bearing,a,b);

  draw_pose(im, pose, hull, pixel_per_m, cv::Scalar(0,0,255), 1);
  draw_range_bearing(im, pose, range, bearing, hull, pixel_per_m, cv::Scalar(0,0,255), 1); 
  draw_box(im,a,hull,pixel_per_m,cv::Scalar(0,0,255),-1); 
  draw_box(im,b,hull,pixel_per_m,cv::Scalar(0,0,255),-1);

  cv::imshow("No Range Bearing Cross Contractor",im);
}

int main(int argc, char **argv)
{
  cv::namedWindow("No Range Bearing Cross Contractor", cv::WINDOW_AUTOSIZE);

  char TrackbarName[50];
  snprintf( TrackbarName, sizeof(TrackbarName), "t_x x %f", 0.1 );
  cv::createTrackbar( TrackbarName, "No Range Bearing Cross Contractor", &pose_x_slider, pose_x_max, on_trackbar );

  char TrackbarName1[50];
  snprintf( TrackbarName1, sizeof(TrackbarName1), "t_y x %f", 0.1 );
  cv::createTrackbar( TrackbarName1, "No Range Bearing Cross Contractor", &pose_y_slider, pose_y_max, on_trackbar );

  char TrackbarName4[50];
  snprintf( TrackbarName4, sizeof(TrackbarName4), "psi x %f", 0.1 );
  cv::createTrackbar( TrackbarName4, "No Range Bearing Cross Contractor", &pose_psi_slider, pose_psi_max, on_trackbar );

  char TrackbarName2[50];
  snprintf( TrackbarName2, sizeof(TrackbarName2), "range x %f", 0.1 );
  cv::createTrackbar( TrackbarName2, "No Range Bearing Cross Contractor", &range_slider, range_max, on_trackbar );

  char TrackbarName3[50];
  snprintf( TrackbarName3, sizeof(TrackbarName3), "bearing x %f", 0.1 );
  cv::createTrackbar( TrackbarName3, "No Range Bearing Cross Contractor", &bearing_slider, bearing_max, on_trackbar );

  on_trackbar(0,0);

  cv::waitKey(0); 
}