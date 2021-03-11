/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: filter.h 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#ifndef PCL_ROS2__FILTERS__FILTER_HPP_
#define PCL_ROS2__FILTERS__FILTER_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <pcl/filters/filter.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/exact_time.h"
#include "message_filters/sync_policies/approximate_time.h"

#include "pcl_msgs/msg/point_indices.hpp"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


namespace pcl_ros2
{
namespace sync_policies = message_filters::sync_policies;

/** \brief @b Filter represents the base filter class. Some generic 3D operations that are
  * applicable to all filters are defined here as static methods.
  * \author Radu Bogdan Rusu
  * \author Zahi Kakish (ROS 2)
  */
class Filter : public rclcpp::Node
{
public:
  typedef sensor_msgs::msg::PointCloud2 PointCloud2;

  typedef pcl::IndicesPtr IndicesPtr;
  typedef pcl::IndicesConstPtr IndicesConstPtr;
  
  typedef pcl_msgs::msg::PointIndices PointIndices;
  typedef pcl_msgs::msg::PointIndices::SharedPtr PointIndicesPtr;
  typedef pcl_msgs::msg::PointIndices::ConstSharedPtr PointIndicesConstPtr;

  explicit Filter(const rclcpp::NodeOptions & options);

protected:
  /** \brief the input PointCloud subscriber. */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_input_;

  /** \brief The message filter subscriber for PointCloud2. */
  message_filters::Subscriber<PointCloud2> sub_input_filter_;

  /** \brief The message filter subscriber for PointIndices. */
  message_filters::Subscriber<PointIndices> sub_indices_filter_;

  /** \brief The desired user filter field name. */
  std::string filter_field_name_;

  /** \brief The minimum allowed filter value a point will be considered from. */
  double filter_limit_min_;

  /** \brief The maximum allowed filter value a point will be considered from. */
  double filter_limit_max_;

  /** \brief Set to true if we want to return the data outside
    * (\a filter_limit_min_;\a filter_limit_max_). Default: false.
    */
  bool filter_limit_negative_;

  /** \brief The input TF frame the data should be transformed into,
    * if input.header.frame_id is different.
    */
  std::string tf_input_frame_;

  /** \brief The original data input TF frame. */
  std::string tf_input_orig_frame_;

  /** \brief The output TF frame the data should be transformed into,
    * if input.header.frame_id is different.
    */
  std::string tf_output_frame_;

  /** \brief Internal mutex. */
  std::mutex mutex_;

    /** \brief Virtual abstract filter method. To be implemented by every child.
    * \param input the input point cloud dataset.
    * \param indices a pointer to the vector of point indices to use.
    * \param output the resultant filtered PointCloud2
    */
  virtual void
  filter(
    const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices,
    PointCloud2 & output) = 0;

  /** \brief Lazy transport subscribe routine. */
  virtual void
  subscribe();

  /** \brief Lazy transport unsubscribe routine. */
  virtual void
  unsubscribe();

  /** \brief Nodelet initialization routine. */
  // virtual void
  // onInit();

  /** \brief Call the child filter () method, optionally transform the result, and publish it.
    * \param input the input point cloud dataset.
    * \param indices a pointer to the vector of point indices to use.
    */
  void
  computePublish(const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices);

  /** \brief The output PointCloud publisher. */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_output_;

  /** \brief Component parameters. */
  int max_queue_size_;
  bool use_indices_;
  bool latched_indices_;
  bool approximate_sync_;

  /** \brief Transform Listener with buffer. */
  tf2_ros::Buffer buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;


  /** \brief Test whether a given PointCloud message is "valid" (i.e., has points, and width and height are non-zero).
    * \param cloud the point cloud to test
    * \param topic_name an optional topic name (only used for printing, defaults to "input")
    */
  inline bool
  isValid(const PointCloud2::ConstSharedPtr & cloud, const std::string & topic_name = "input")
  {
    if (cloud->width * cloud->height * cloud->point_step != cloud->data.size()) {
      RCLCPP_WARN(this->get_logger(),
        "[Filter] Invalid PointCloud (data = %zu, width = %d, height = %d, step = %d) "
        "with stamp %f, and frame %s received!",
        cloud->data.size(), cloud->width, cloud->height, cloud->point_step,
        cloud->header.stamp.sec, cloud->header.frame_id.c_str());

      return false;
    }
    return true;
  }

  /** \brief Test whether a given PointIndices message is "valid" (i.e., has values).
    * \param indices the point indices message to test
    * \param topic_name an optional topic name (only used for printing, defaults to "indices")
    */
  inline bool
  isValid(const PointIndicesConstPtr & indices, const std::string & topic_name = "indices")
  {
    return true;
  }

private:
  /** \brief Pointer to a dynamic reconfigure service. */
  // boost::shared_ptr<dynamic_reconfigure::Server<pcl_ros::FilterConfig>> srv_;

  /** \brief Synchronized input, and indices.*/
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2,
    PointIndices>>> sync_input_indices_e_;
  std::shared_ptr<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2,
    PointIndices>>> sync_input_indices_a_;

  /** \brief Dynamic reconfigure service callback. */
  // virtual void
  // config_callback(pcl_ros::FilterConfig & config, uint32_t level);

  PointIndicesConstPtr indices_;

  /** \brief PointCloud2 + Indices data callback. */
  void
  input_indices_callback(
    const PointCloud2::SharedPtr cloud,
    const PointIndicesConstPtr indices);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pcl_ros2

#endif  // PCL_ROS2__FILTERS__FILTER_HPP_