#include "hypascore_localization/contractors/CtcBoundaryLine.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include <queue>

int pose_x_max = 400.0;
int pose_x_slider = 210.0;
int pose_y_max = 400.0;
int pose_y_slider = 210.0;
	
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

void draw_connection_lines_between_boxes(cv::Mat& im, ibex::IntervalVector a, ibex::IntervalVector b, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  std::vector<ibex::Vector> corners_a = get_corner_points(a);
  std::vector<ibex::Vector> corners_b = get_corner_points(b);
  for(size_t i = 0; i < corners_a.size(); ++i)
  {

    draw_line(im, corners_a[i], corners_b[i], hull, pixel_per_m, color, thickness); 
  }
} 

void sivia(codac::CtcBoundaryLine& ctc, ibex::IntervalVector pose, ibex::IntervalVector a, ibex::IntervalVector b, std::vector<ibex::IntervalVector>& intermediate_result)
{
  std::queue<ibex::IntervalVector> q;
  q.push(pose); 
  ibex::IntervalVector pose_i(2),a_i(2),b_i(2);
  double min_diam_length_ = 0.04; 
  while(!q.empty())
  {
    pose_i = q.front();
    a_i = a; b_i = b; 
    q.pop();
    ctc.contract(pose_i,a_i,b_i);
    if(!pose_i.is_empty())
    {
      if(pose_i.min_diam() >= min_diam_length_)
      {
        std::pair<ibex::IntervalVector,ibex::IntervalVector> bisected = pose_i.bisect(pose_i.extr_diam_index(false));
        q.push(bisected.first);
        q.push(bisected.second);
      }
      else
      {
        intermediate_result.push_back(pose_i); 
      }
    }
  }
}

static void on_trackbar( int, void* )
{

  codac::CtcBoundaryLine ctc;

  ibex::IntervalVector pose(2);
  pose[0] = (double)(pose_x_slider-200)/10.0; 
  pose[1] = (double)(pose_y_slider-200)/10.0; 
  pose[0].inflate(0.5);
  pose[1].inflate(0.5);

  ibex::IntervalVector a(2);
  a[0] = 10.0; 
  a[1] = 0.0; 
  a[0].inflate(0.01);
  a[1].inflate(0.01);
  
  ibex::IntervalVector b(2);
  b[0] = 0.0; 
  b[1] = 10.0; 
  b[0].inflate(0.01);
  b[1].inflate(0.01);

  ibex::IntervalVector hull(2); 
  hull.set_empty(); 
  hull = pose | a | b; 
  hull.inflate(1.0);

  int large_im_width = 1000;
  int large_im_height = 1000;
  cv::Scalar background_color(255,255,255); 
  cv::Mat im(cv::Size(large_im_width,large_im_height),CV_8UC3, background_color);
  double pixel_per_m = std::min(large_im_width/hull[0].diam(),large_im_height/hull[1].diam());

  draw_box(im,pose,hull,pixel_per_m,cv::Scalar(0,0,0),1);
  draw_box(im,a,hull,pixel_per_m,cv::Scalar(0,255,0),1); 
  draw_box(im,b,hull,pixel_per_m,cv::Scalar(0,255,0),1);

  draw_connection_lines_between_boxes(im, a, b, hull, pixel_per_m, cv::Scalar(0,255,0), 1);

  ctc.contract(pose,a,b);
  std::vector<ibex::IntervalVector> sivia_result;
  sivia(ctc, pose, a, b, sivia_result); 

  draw_box(im,pose,hull,pixel_per_m,cv::Scalar(0,0,255),-1);
  draw_box(im,a,hull,pixel_per_m,cv::Scalar(0,0,255),-1); 
  draw_box(im,b,hull,pixel_per_m,cv::Scalar(0,0,255),-1);

  // show sivia results
  for(ibex::IntervalVector& v : sivia_result)
  {
    draw_box(im,v,hull,pixel_per_m,cv::Scalar(0,250,100),1);
  }

  cv::imshow("Boundary Line Contractor",im);
}

int main(int argc, char **argv)
{
  cv::namedWindow("Boundary Line Contractor", cv::WINDOW_AUTOSIZE);

  char TrackbarName[50];
  snprintf( TrackbarName, sizeof(TrackbarName), "x-position x %f", 0.1 );
  cv::createTrackbar( TrackbarName, "Boundary Line Contractor", &pose_x_slider, pose_x_max, on_trackbar );

  char TrackbarName1[50];
  snprintf( TrackbarName1, sizeof(TrackbarName1), "y-position x %f", 0.1 );
  cv::createTrackbar( TrackbarName1, "Boundary Line Contractor", &pose_y_slider, pose_y_max, on_trackbar );

  cv::waitKey(0); 
}