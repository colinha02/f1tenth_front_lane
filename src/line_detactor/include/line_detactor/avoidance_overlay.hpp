#pragma once
#include "bev_handoff/avoidance_handoff.hpp"
#include <opencv2/imgproc.hpp>
#include <cmath>
#include <chrono>
#include <utility>

namespace line_detactor
{
inline std::string drawAvoidancePreview(cv::Mat & image,const std_msgs::msg::Header & header,
  int width,int height,int padding,double max_delta,double max_age)
{
  if (!bev_handoff::avoidancePreviewEnabled()) {return {};}
  const auto plan=bev_handoff::latestAvoidancePreview();
  const std::string mode=bev_handoff::avoidanceControlRequested()?"APPLY ":"TEST ";
  const auto unavailable=[&](const std::string & why) {
    return mode+"WAIT: "+why;
  };
  if (!plan) {return unavailable("lane/depth");}
  const auto now=std::chrono::steady_clock::now();
  const double age=std::chrono::duration<double>(now-plan->lane_received_at).count();
  const double depth_age=std::chrono::duration<double>(now-plan->depth_captured_at).count();
  const double delta=std::abs(double(header.stamp.sec)-plan->header.stamp.sec+
    (double(header.stamp.nanosec)-plan->header.stamp.nanosec)*1e-9);
  const double display_delta=std::min(max_age,std::max(max_delta,plan->display_delta_sec));
  if (plan->header.frame_id!=header.frame_id || age<0 || age>max_age || depth_age<0 || depth_age>max_age || delta>display_delta) {
    return unavailable("stale/unsynced");
  }
  if (plan->width!=width || plan->height!=height || image.cols!=width+2*padding || image.rows!=height) {
    return unavailable("geometry");
  }
  auto plot=image(cv::Rect(padding,0,width,height));
  const auto pixel=[&](double x,double y) {
    return cv::Point(static_cast<int>(std::lround((plan->y_max-y)/plan->meter_per_pixel-.5)),
      static_cast<int>(std::lround((plan->x_max-x)/plan->meter_per_pixel-.5)));
  };
  for (const auto & box:plan->boxes) {
    cv::rectangle(plot,pixel(box.x0,box.y0),pixel(box.x1,box.y1),cv::Scalar(45,45,85),1);
  }
  for (const auto & box:plan->assumed_boxes) {
    cv::rectangle(plot,pixel(box.x0,box.y0),pixel(box.x1,box.y1),cv::Scalar(0,165,255),1);
    cv::drawMarker(plot,pixel((box.x0+box.x1)/2,(box.y0+box.y1)/2),
      cv::Scalar(0,165,255),cv::MARKER_CROSS,5,1,cv::LINE_AA);
  }
  const auto path=[&](const std::vector<cv::Point2d> & points,const cv::Scalar & color,int thickness) {
    for (std::size_t i=1;i<points.size();++i) {
      cv::line(plot,pixel(points[i-1].x,points[i-1].y),pixel(points[i].x,points[i].y),color,thickness,cv::LINE_AA);
    }
  };
  // No fan of global left/right candidates. Only show the attempted local
  // profile when blocked, or the selected complete multi-obstacle route.
  if (plan->selected.empty() && !plan->candidates.empty()) {
    path(plan->candidates.back().path,cv::Scalar(65,65,160),1);
  }
  path(plan->original,cv::Scalar(0,255,255),1);
  path(plan->selected,cv::Scalar(255,255,0),2);
  // Show the reason instead of a generic WAIT with zero speed/curvature.
  if (plan->selected.empty()) {return mode+plan->status;}
  // C=inferred corridor from centerline width (only for avoidance candidates).
  // Full details remain on /auto/avoidance_preview/status.
  return mode+plan->status.substr(0,plan->status.find(':'))+
    (plan->inferred_boundaries?" C":"")+
    cv::format(" n%zu L%.1f vref%.2f",plan->obstacle_count,plan->horizon_m,plan->recommended_speed);
}
// Extend only the display canvas. The 120x300 inference image and BEV geometry
// remain unchanged; this rear strip depicts tracked objects in the blind zone.
inline void appendPassagePreview(cv::Mat & image,int width,int height,int padding)
{
  if (!bev_handoff::avoidancePreviewEnabled() || width<=0 || height<=0 ||
      image.cols!=width+2*padding || image.rows!=height) {return;}
  const auto plan=bev_handoff::latestAvoidancePreview();
  const double mpp=plan && plan->meter_per_pixel>0 ? plan->meter_per_pixel:.01;
  constexpr double front=.07,rear=.43,half_width=.135,camera_x=-.05;
  const int extension=static_cast<int>(std::ceil((rear+.22)/mpp));
  cv::Mat extended;
  cv::copyMakeBorder(image,extended,0,extension,0,0,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
  image=std::move(extended);
  const double x_max=height*mpp,y_max=width*mpp/2;
  const auto px=[&](double x,double y) {
    return cv::Point(padding+static_cast<int>(std::lround((y_max-y)/mpp-.5)),
      static_cast<int>(std::lround((x_max-x)/mpp-.5)));
  };
  cv::line(image,{padding,height},{padding+width-1,height},cv::Scalar(65,65,65),1);
  if (plan && plan->width==width && plan->height==height &&
      std::chrono::duration<double>(std::chrono::steady_clock::now()-plan->lane_received_at).count()<.3) {
    for (const auto & box:plan->remembered_boxes) {
      if (box.x0>=0 || box.x1 < -rear-.22) {continue;}
      cv::rectangle(image,px(box.x0,box.y0),px(box.x1,box.y1),cv::Scalar(0,165,255),1);
    }
  }
  cv::rectangle(image,px(front,half_width),px(-rear,-half_width),cv::Scalar(0,255,0),1);
  cv::drawMarker(image,px(camera_x,0),cv::Scalar(255,255,255),cv::MARKER_CROSS,5,1);
  cv::putText(image,"27x50cm",px(-.12,-.19),cv::FONT_HERSHEY_SIMPLEX,.30,
    cv::Scalar(0,255,0),1,cv::LINE_AA);
}
}
