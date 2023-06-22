#include "hypascore_localization/localization/CMMap.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <opencv2/calib3d.hpp>
#include "opencv2/imgproc.hpp"
#include <queue>
#include "codac-rob/codac_CtcConstell.h"
#include <algorithm>
#include "codac/codac_CtcPolar.h"

int uncert_x_max = 400.0;
int uncert_x_slider = 5.0;
int uncert_y_max = 400.0;
int uncert_y_slider = 5.0;
int uncert_psi_max = 314; 
int uncert_psi_slider = 0.0;

int pose_x_max = 400.0;
int pose_x_slider = 200.0;
int pose_y_max = 400.0;
int pose_y_slider = 200.0;
int pose_psi_max = 628; 
int pose_psi_slider = 314;

std::vector<WallGroundLine> map_; 
ibex::IntervalVector map_hull;
double obs_range = 8.0;
double angle_inc = 0.03;
	
// drawing functions
void point_to_pixel(ibex::IntervalVector& hull, double& pixel_per_m, ibex::Vector& point, cv::Point2f& pixel)
{
  ibex::Vector tl(2); 
  tl[0] = hull[0].lb(); 
  tl[1] = hull[1].ub();  

  ibex::Vector offset_point = point-tl; 
  pixel.x = offset_point[0]*pixel_per_m; 
  pixel.y = -offset_point[1]*pixel_per_m;
}

void param_to_pixel(ibex::IntervalVector& hull, double& pixel_per_angle, double& pixel_per_dist, ibex::Vector& point, cv::Point2f& pixel)
{
  ibex::Vector tl(2); 
  tl[0] = hull[0].lb(); 
  tl[1] = hull[1].ub();  

  ibex::Vector offset_point = point-tl; 
  pixel.x = offset_point[0]*pixel_per_angle; 
  pixel.y = -offset_point[1]*pixel_per_dist;
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

void draw_param_box(cv::Mat& im, ibex::IntervalVector param, ibex::IntervalVector hull, double pixel_per_angle, double pixel_per_dist, cv::Scalar color, int thickness = -1)
{
  std::pair<ibex::Vector,ibex::Vector> param_box_corner_points = get_rect_corner_points(param);
  cv::Point2f param_bl,param_tr; 
  param_to_pixel(hull, pixel_per_angle, pixel_per_dist,  param_box_corner_points.first, param_bl);
  param_to_pixel(hull, pixel_per_angle, pixel_per_dist,  param_box_corner_points.second, param_tr);
  cv::rectangle(im,param_bl,param_tr,color,thickness);
}

void draw_circle(cv::Mat& im, ibex::Vector center, int radius, ibex::IntervalVector hull, double pixel_per_m, cv::Scalar color, int thickness = -1)
{
  cv::Point2f p_px; 
  point_to_pixel(hull, pixel_per_m, center, p_px);
  cv::circle(im,p_px,radius,color,thickness);
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

  // draw the ground truth which is the mid of the
  /*cv::Scalar gt_color(0,200,0);
  draw_circle(im,position_mid,12,hull,pixel_per_m,gt_color,-1);
  double psi_mid = pose[2].mid(); 
  ibex::Vector psi_p(2); 
  psi_p[0] = 0.5*cos(psi_mid); psi_p[1] = 0.5*sin(psi_mid);
  ibex::Vector heading = position_mid+psi_p;
  draw_line(im, position_mid, heading, hull, pixel_per_m, gt_color, 4);*/
}

// generate map
void generate_map()
{
  map_hull = ibex::IntervalVector(2); 
  map_hull.set_empty();
  map_.resize(5); 
  // set the start and end points
  map_[0].start_point = Eigen::Vector2d(8,0);
  map_[0].end_point = Eigen::Vector2d(8,5);
  map_[1].start_point = map_[0].end_point;
  map_[1].end_point = Eigen::Vector2d(3,5);
  map_[2].start_point = map_[1].end_point;
  map_[2].end_point = Eigen::Vector2d(-1,7);
  map_[3].start_point = map_[2].end_point;
  map_[3].end_point = Eigen::Vector2d(-1,-2);
  map_[4].start_point = map_[3].end_point ;
  map_[4].end_point = Eigen::Vector2d(2,2);
  // compute normal vector and distance
  for(WallGroundLine& wgl : map_)
  {
    const Eigen::Vector2d& s = wgl.start_point;
    const Eigen::Vector2d& e = wgl.end_point;
    Eigen::Vector2d diff = e-s; 
    Eigen::Vector2d n(-diff(1),diff(0));
    n.normalize();
    double angle = atan2(n(1),n(0)); 
    if(angle > M_PI/2)
    {
      angle = angle - M_PI;
      n(0) = cos(angle);
      n(1) = sin(angle); 
    }
    else if(angle < -M_PI/2)
    {
      angle = angle + M_PI;
      n(0) = cos(angle);
      n(1) = sin(angle); 
    }
    double d = n.transpose()*s; 
    wgl.line_params = Eigen::Vector3d(n(0),n(1),d);
    //std::cout << wgl.line_params << std::endl << std::endl; 
    std::cout << "angle: " << angle << " d: " << d << std::endl;
    map_hull[0] = map_hull[0] | s(0);
    map_hull[0] = map_hull[0] | e(0);
    map_hull[1] = map_hull[1] | s(1);
    map_hull[1] = map_hull[1] | e(1);
  }
  std::cout << "---" << std::endl; 
}

void draw_local_measurements(cv::Mat& im, const std::map<WallGroundLine*,std::vector<Eigen::Vector2d>>& local_measurements, const std::vector<WallGroundLine>& local_lines, cv::Scalar color, int thickness = -1)
{
  ibex::IntervalVector hull(2); 
  hull = ibex::Vector::zeros(2); 
  hull.inflate(obs_range+1.0);

  int large_im_width = 800;
  int large_im_height = 800;
  cv::Scalar background_color(255,255,255); 
  im = cv::Mat(cv::Size(large_im_width,large_im_height),CV_8UC3, background_color);
  double pixel_per_m = std::min(large_im_width/hull[0].diam(),large_im_height/hull[1].diam());

  ibex::IntervalVector pose(3);
  pose = ibex::Vector::zeros(3); 
  draw_pose(im,pose,hull,pixel_per_m,background_color,1); 

  for(std::pair<WallGroundLine*,std::vector<Eigen::Vector2d>> pair : local_measurements)
  {

    for(int i = 1; i < pair.second.size()-1; i++)
    {
      const Eigen::Vector2d& p = pair.second[i];
      ibex::Vector x(2); 
      x[0] = p(0); 
      x[1] = p(1);
      draw_circle(im,x,4,hull,pixel_per_m,color,thickness); 
    }
    {const Eigen::Vector2d& p = pair.second[0];
    ibex::Vector x(2); 
    x[0] = p(0); 
    x[1] = p(1);
    draw_circle(im,x,4,hull,pixel_per_m,cv::Scalar(255,50,255),thickness);}
    {const Eigen::Vector2d& p = pair.second[pair.second.size()-1];
    ibex::Vector x(2); 
    x[0] = p(0); 
    x[1] = p(1);
    draw_circle(im,x,4,hull,pixel_per_m,cv::Scalar(0,0,255),thickness);}
  }

  for(const WallGroundLine& wgl : local_lines)
  {
    ibex::Vector s(2);
    ibex::Vector e(2); 
    s[0] = wgl.start_point(0); 
    s[1] = wgl.start_point(1); 
    e[0] = wgl.end_point(0); 
    e[1] = wgl.end_point(1);
    draw_line(im,s,e,hull,pixel_per_m,cv::Scalar(0,50,0),2); 
  }
}

void draw_hough_map(cv::Mat& im, std::vector<WallGroundLine>& map, ibex::IntervalVector hull, double pixel_per_angle, double pixel_per_dist, cv::Scalar color)
{
  for(const WallGroundLine& wgl : map)
  {
    ibex::Vector param(2); 
    param[0] = atan2(wgl.line_params(1),wgl.line_params(0));
    param[1] = wgl.line_params(2); 
    cv::Point2f p_px; 
    param_to_pixel(hull, pixel_per_angle, pixel_per_dist, param, p_px);
    cv::circle(im,p_px,4,color,-1);
  }
}

// generate local measurements
bool line_segments_intersect(Eigen::Vector2d& p1, Eigen::Vector2d& p2, Eigen::Vector2d& p3, Eigen::Vector2d& p4, Eigen::Vector2d& intersection)
{
  double& x1 = p1(0);
  double& y1 = p1(1);
  double& x2 = p2(0);
  double& y2 = p2(1);
  double& x3 = p3(0);
  double& y3 = p3(1);
  double& x4 = p4(0);
  double& y4 = p4(1);

  double t = ((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
  double u = ((x1-x3)*(y1-y2)-(y1-y3)*(x1-x2))/((x1-x2)*(y3-y4)-(y1-y2)*(x3-x4));
  
  if(0.0<=t && t<=1.0 && 0.0<=u && u<=1.0)
  {
    intersection(0) = x1+t*(x2-x1);
    intersection(1) = y1+t*(y2-y1);  
    return true;
  }
  return false;
}

bool sort_measure(Eigen::Vector2d a,Eigen::Vector2d b) { return (atan2(a(1),a(0))<atan2(b(1),b(0))); }
bool sort_measure_jump_contained(Eigen::Vector2d a,Eigen::Vector2d b) 
{ 
  double angle1 = atan2(a(1),a(0));
  double angle2 = atan2(b(1),b(0));
  if(angle1 < 0)
  {
    angle1 += 2.0*M_PI;
  }
  if(angle2 < 0)
  {
    angle2 += 2.0*M_PI;
  }
  return (angle1<angle2); 
}

void extract_local_line(const std::map<WallGroundLine*,std::vector<Eigen::Vector2d>>& local_measurements, std::vector<WallGroundLine>& local_lines)
{
  // choose first and last point in the list, compute line parameters
  for(std::pair<WallGroundLine*,std::vector<Eigen::Vector2d>> pair : local_measurements)
  {
    if(pair.second.size() > 2)
    {
      WallGroundLine local_wgl; 
      const Eigen::Vector2d& s = pair.second[0];
      const Eigen::Vector2d& e = pair.second[pair.second.size()-1];
      Eigen::Vector2d diff = e-s; 
      Eigen::Vector2d n(-diff(1),diff(0));
      n.normalize();
      double angle = atan2(n(1),n(0)); 
      if(angle > M_PI/2)
      {
        angle = angle - M_PI;
        n(0) = cos(angle);
        n(1) = sin(angle); 
      }
      else if(angle < -M_PI/2)
      {
        angle = angle + M_PI;
        n(0) = cos(angle);
        n(1) = sin(angle); 
      }
      double d = n.transpose()*s; 
      local_wgl.line_params = Eigen::Vector3d(n(0),n(1),d);
      local_wgl.start_point = s; 
      local_wgl.end_point = e; 
      local_lines.emplace_back(local_wgl);
    }
  }
}

void generate_local_measurements(const ibex::IntervalVector& pose, std::map<WallGroundLine*,std::vector<Eigen::Vector2d>>& local_measurements)
{
  Eigen::Affine2d m_T_l;
  m_T_l.translation()(0) = pose[0].mid(); 
  m_T_l.translation()(1) = pose[1].mid();
  m_T_l.linear() << cos(pose[2].mid()), -sin(pose[2].mid()), sin(pose[2].mid()), cos(pose[2].mid());
  // generate measurement rays
  int angle_steps = (int) (2*M_PI/angle_inc);
  std::vector<std::pair<WallGroundLine*,Eigen::Vector2d>> measurements(angle_steps);  
  #pragma omp parallel for 
  for(int i = 0; i < angle_steps; i++)
  {
    double angle = (double)i * angle_inc;
    Eigen::Vector2d l_start; l_start = Eigen::Vector2d::Zero(); 
    Eigen::Vector2d l_end = Eigen::Vector2d(cos(angle),sin(angle))*obs_range; 
    Eigen::Vector2d obs_start = m_T_l*l_start; 
    Eigen::Vector2d obs_end = m_T_l*l_end; 
    // check intersection with each map-line
    for(WallGroundLine& wgl : map_)
    {
      Eigen::Vector2d intersection; 
      if(line_segments_intersect(wgl.start_point,wgl.end_point,obs_start,obs_end,intersection))
      {
        Eigen::Vector2d l_intersection = m_T_l.inverse() * intersection; 
        if(measurements[i].first == NULL)
        {
          // first measurement
          measurements[i] = (std::make_pair(&wgl,l_intersection)); 
        }
        else
        {
          // measurement already there, select closest
          if(l_intersection.norm() < measurements[i].second.norm()) measurements[i] = (std::make_pair(&wgl,l_intersection));
        }
      }
    }
  }
  for(std::pair<WallGroundLine*,Eigen::Vector2d>& pair : measurements)
  {
    if(pair.first != NULL)
    {
      local_measurements[pair.first].push_back(pair.second);
    }
  }
  for(WallGroundLine& wgl : map_)
  {
    if(local_measurements.find(&wgl) != local_measurements.end())
    {
      ibex::Interval angle_span;
      angle_span.set_empty();  
      for(const Eigen::Vector2d p : local_measurements[&wgl])
      {
        // check if jump contained
        angle_span = angle_span | atan2(p(1),p(0));
      }
      if(angle_span.diam() <= M_PI)
      {
        std::sort(local_measurements[&wgl].begin(), local_measurements[&wgl].end(), sort_measure);
      }
      else
      {
        std::sort(local_measurements[&wgl].begin(), local_measurements[&wgl].end(), sort_measure_jump_contained);
      }
    }
  }
}

void transform_local_lines_to_map(const std::vector<WallGroundLine>& local_lines, ibex::IntervalVector& pose, cv::Mat& im, ibex::IntervalVector hull, double pixel_per_angle, double pixel_per_dist)
{
  ibex::Interval x_m = pose[0];
  ibex::Interval y_m = pose[1];
  ibex::Interval psi_m = pose[2];

  // build the constellation map 
  std::vector<ibex::IntervalVector> constell_map; 
  for(const WallGroundLine& wgl_map : map_)
  {
    double angle = atan2(wgl_map.line_params(1),wgl_map.line_params(0));
    double d = wgl_map.line_params(2); 
    ibex::IntervalVector param(2);
    param[0] = angle; param[0].inflate(0.0001); 
    param[1] = d; param[1].inflate(0.0001); 
    constell_map.push_back(param);
  }
  codac::CtcConstell ctc_constell(constell_map);

  for(const WallGroundLine& wgl : local_lines)
  {
    ibex::Interval angle_l(atan2(wgl.line_params(1),wgl.line_params(0)));
    angle_l.inflate(0.00001);
    ibex::Interval d_l(wgl.line_params(2));
    d_l.inflate(0.00001);
    ibex::Interval angle_m = psi_m+angle_l; // PROBLEM!!!! -> angle_m can be larger than pi/2 or smaller than -pi/2
    bool change_dm = false;
    bool add_mpi = false; 
    // Deal with the pi/2 problem of linesn by clipping the rotation always between -pi/2 and pi/2 
    if(angle_m.lb() > M_PI/2.0)
    {
      angle_m = angle_m - M_PI;
      change_dm = true; 
    }
    else if(angle_m.ub() < -M_PI/2.0)
    {
      angle_m = angle_m + M_PI;
      change_dm = true;
      add_mpi = true; 
    }
    else if(angle_m.contains(M_PI/2.0) || angle_m.contains(-M_PI/2.0))
    {
      continue; // ignore if orientation on the border: problem with empty set
    }
    codac::CtcPolar ctc_polar; 
    ibex::Interval nx, ny;
    ibex::Interval roh(1.0);
    ctc_polar.contract(nx,ny,roh,angle_m); 
    ibex::Interval nxx_m = nx*x_m;
    ibex::Interval nyy_m = ny*y_m;
    ibex::Interval sum = nxx_m + nyy_m;
    ibex::Interval d_m = d_l + sum;
    if(change_dm) // if the orientation was changes, d_m also changes!
    {
      d_m = -d_l + sum; 
    }

    ibex::IntervalVector line_param_m(2); 
    line_param_m[0] = angle_m;
    line_param_m[1] = d_m; 
    draw_param_box(im,line_param_m,hull,pixel_per_angle,pixel_per_dist,cv::Scalar(255,0,0),2); // parameter box in hough-space before contraction 
    ctc_constell.contract(line_param_m);

    draw_param_box(im,line_param_m,hull,pixel_per_angle,pixel_per_dist,cv::Scalar(0,100,0),2); // parameter box in hough-space after contraction 
    angle_m = line_param_m[0]; 
    d_m = line_param_m[1]; 

    // backward
    if(!change_dm) // consider the pi/2-problem as mentioned above
    {
      ibex::bwd_add(d_m,d_l,sum);
    }
    else
    {
      ibex::bwd_sub(d_m,sum,d_l);
    }
    
    ibex::bwd_add(sum,nxx_m,nyy_m);
    ibex::bwd_mul(nxx_m,nx,x_m);
    ibex::bwd_mul(nyy_m,ny,y_m);
    ctc_polar.contract(nx,ny,roh,angle_m);
    if(change_dm) // here consider the pi/2 problem for the orientation
    {
      if(add_mpi)
      {
        ibex::Interval angle_m_copy = angle_m;
        angle_m_copy = angle_m_copy-M_PI;  
        ibex::bwd_add(angle_m_copy,psi_m,angle_l);
        angle_m = angle_m_copy + M_PI;
      }
      else
      {
        ibex::Interval angle_m_copy = angle_m;
        angle_m_copy = angle_m_copy+M_PI;  
        ibex::bwd_add(angle_m_copy,psi_m,angle_l);
        angle_m = angle_m_copy - M_PI;
      }
    }
    else
    {
      ibex::bwd_add(angle_m,psi_m,angle_l);
    }
  }
  pose[0] = x_m; 
  pose[1] = y_m;
  pose[2] = psi_m;
}

static void on_trackbar( int, void* )
{
  ibex::IntervalVector pose(3);
  pose[0] = (double)(pose_x_slider)/10.0-20; 
  pose[1] = (double)(pose_y_slider)/10.0-20; 
  pose[2] = (double)(pose_psi_slider)/100.0-M_PI;
  pose[0].inflate((double)(uncert_x_slider)/10.0);
  pose[1].inflate((double)(uncert_y_slider)/10.0);
  pose[2].inflate((double)(uncert_psi_slider)/100.0);

  ibex::IntervalVector hull(2); 
  hull.set_empty(); 
  hull = pose.subvector(0,1) | map_hull; 
  hull.inflate(1.0);

  int large_im_width = 800;
  int large_im_height = 800;
  cv::Scalar background_color(255,255,255); 
  cv::Mat im(cv::Size(large_im_width,large_im_height),CV_8UC3, background_color);
  double pixel_per_m = std::min(large_im_width/hull[0].diam(),large_im_height/hull[1].diam());

  draw_pose(im,pose,hull,pixel_per_m,cv::Scalar(0,0,0),1);

  for(const WallGroundLine& wgl : map_)
  {
    ibex::Vector s(2); 
    ibex::Vector e(2); 
    s[0] = wgl.start_point(0); 
    s[1] = wgl.start_point(1);
    e[0] = wgl.end_point(0); 
    e[1] = wgl.end_point(1);
    draw_line(im,s,e,hull,pixel_per_m,cv::Scalar(100,100,100),2);
  }

  std::map<WallGroundLine*,std::vector<Eigen::Vector2d>> local_measurements; 
  cv::Mat im_local; 
  generate_local_measurements(pose, local_measurements); 

  // extract local line data 
  std::vector<WallGroundLine> local_lines; 
  extract_local_line(local_measurements,local_lines);
  draw_local_measurements(im_local,local_measurements,local_lines,cv::Scalar(0,200,0));

  // draw hough-space
  ibex::IntervalVector hough_hull(2);
  hough_hull.set_empty(); 
  for(const WallGroundLine& wgl : map_)
  {
    double angle = atan2(wgl.line_params(1),wgl.line_params(0));
    double d = wgl.line_params(2); 
    hough_hull[0] = hough_hull[0] | angle; 
    hough_hull[1] = hough_hull[1] | d; 
  }
  hough_hull[0].inflate(0.2);
  hough_hull[1].inflate(0.5); 
  int hough_im_width = 800;
  int hough_im_height = 800;
  cv::Mat hough_im(cv::Size(hough_im_width,hough_im_height),CV_8UC3, background_color);
  double pixel_per_angle = hough_im_width/hough_hull[0].diam();
  double pixel_per_dist = hough_im_height/hough_hull[1].diam();
  draw_hough_map(hough_im, map_, hough_hull, pixel_per_angle, pixel_per_dist, cv::Scalar(0,0,0));

  transform_local_lines_to_map(local_lines, pose, hough_im, hough_hull, pixel_per_angle, pixel_per_dist);

  draw_pose(im,pose,hull,pixel_per_m,cv::Scalar(0,200,0),3);

  cv::imshow("Constellation Contractor",im);
  cv::imshow("Constellation Contractor Local Measurements",im_local);
  cv::imshow("Constellation Contractor Hough Space",hough_im);
}

int main(int argc, char **argv)
{
  generate_map();
  cv::namedWindow("Constellation Contractor", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Constellation Contractor Local Measurements", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Constellation Contractor Hough Space", cv::WINDOW_AUTOSIZE);

  char TrackbarName[50];
  snprintf( TrackbarName, sizeof(TrackbarName), "x-uncertainty x %f", 0.1 );
  cv::createTrackbar( TrackbarName, "Constellation Contractor", &uncert_x_slider, uncert_x_max, on_trackbar );

  char TrackbarName1[50];
  snprintf( TrackbarName1, sizeof(TrackbarName1), "y-uncertainty x %f", 0.1 );
  cv::createTrackbar( TrackbarName1, "Constellation Contractor", &uncert_y_slider, uncert_y_max, on_trackbar );

  char TrackbarName2[50];
  snprintf( TrackbarName2, sizeof(TrackbarName2), "psi-uncertainty x %f", 0.01 );
  cv::createTrackbar( TrackbarName2, "Constellation Contractor", &uncert_psi_slider, uncert_psi_max, on_trackbar );

  char TrackbarName3[50];
  snprintf( TrackbarName3, sizeof(TrackbarName3), "x x %f", 0.1 );
  cv::createTrackbar( TrackbarName3, "Constellation Contractor", &pose_x_slider, pose_x_max, on_trackbar );

  char TrackbarName4[50];
  snprintf( TrackbarName4, sizeof(TrackbarName4), "y x %f", 0.1 );
  cv::createTrackbar( TrackbarName4, "Constellation Contractor", &pose_y_slider, pose_y_max, on_trackbar );

  char TrackbarName5[50];
  snprintf( TrackbarName5, sizeof(TrackbarName5), "psi x %f", 0.01 );
  cv::createTrackbar( TrackbarName5, "Constellation Contractor", &pose_psi_slider, pose_psi_max, on_trackbar );

  cv::waitKey(0); 
}