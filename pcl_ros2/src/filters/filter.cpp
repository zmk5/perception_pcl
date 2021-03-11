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
 * $Id: filter.cpp 35876 2011-02-09 01:04:36Z rusu $
 *
 */

#include "pcl_ros2/filters/filter.hpp"
#include "pcl_ros2/transforms.hpp"

#include "rclcpp_components/register_node_macro.hpp"


namespace pcl_ros2
{

Filter::Filter(const rclcpp::NodeOptions & options)
: Node("filter", options), buffer_(this->get_clock())
{
  // Declare parameters that we care about only at startup
  this->declare_parameter<int>("max_queue_size", 3);

  // Declare optional parameters
  this->declare_parameter<bool>("use_indices", false);
  this->declare_parameter<bool>("latched_indices", false);
  this->declare_parameter<bool>("approximate_sync", false);

  // Get declared parameters
  this->get_parameter("max_queue_size", max_queue_size_);
  this->get_parameter("use_indices", use_indices_);
  this->get_parameter("latched_indices", latched_indices_);
  this->get_parameter("approximate_sync", approximate_sync_);

  // Publisher for PointCloud2
  pub_output_ = this->create_publisher<PointCloud2>("output", max_queue_size_);  // This QoS may cause issues.

  // Set transform listener.
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(buffer_);

  RCLCPP_DEBUG(this->get_logger(),
    "PCL Filter Component successfully created with the following parameters:\n"
    " - approximate_sync : %s\n"
    " - use_indices      : %s\n"
    " - latched_indices  : %s\n"
    " - max_queue_size   : %d",
    (approximate_sync_) ? "true" : "false",
    (use_indices_) ? "true" : "false",
    (latched_indices_) ? "true" : "false",
    max_queue_size_);

}


void
Filter::computePublish(const PointCloud2::ConstSharedPtr & input, const IndicesPtr & indices)
{
  PointCloud2 output;
  // Call the virtual method in the child
  // filter(input, indices, output);

  // PointCloud2::Ptr cloud_tf(new PointCloud2(output));     // set the output by default
  PointCloud2::SharedPtr cloud_tf = std::make_shared<PointCloud2>(output);  // Set the output by default
  // Check whether the user has given a different output TF frame
  if (!tf_output_frame_.empty() && output.header.frame_id != tf_output_frame_) {
    RCLCPP_DEBUG(this->get_logger(),
      "[Filter::computePublish] Transforming output dataset from %s to %s.",
      output.header.frame_id.c_str(), tf_output_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    // if (!pcl_ros2::transformPointCloud(tf_output_frame_, output, cloud_transformed, tf_listener_)) {
    if (!pcl_ros2::transformPointCloud(tf_output_frame_, output, cloud_transformed, buffer_)) {
      RCLCPP_ERROR(this->get_logger(),
        "[Filter::computePublish] Error converting output dataset from %s to %s.",
        output.header.frame_id.c_str(), tf_output_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }
  if (tf_output_frame_.empty() && output.header.frame_id != tf_input_orig_frame_) {
    // no tf_output_frame given, transform the dataset to its original frame
    RCLCPP_DEBUG(this->get_logger(),
      "[Filter::computePublish] Transforming output dataset from %s back to %s.",
      output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    // if (!pcl_ros2::transformPointCloud(
    //     tf_input_orig_frame_, output, cloud_transformed,
    //     tf_listener_))
    if (!pcl_ros2::transformPointCloud(
        tf_input_orig_frame_, output, cloud_transformed, buffer_))
    {
      RCLCPP_ERROR(this->get_logger(),
        "[Filter::computePublish] Error converting output dataset from %s back to %s.",
        output.header.frame_id.c_str(), tf_input_orig_frame_.c_str());
      return;
    }
    cloud_tf.reset(new PointCloud2(cloud_transformed));
  }

  // Copy timestamp to keep it
  cloud_tf->header.stamp = input->header.stamp;

  // Publish a shared ptr
  pub_output_->publish(*cloud_tf);
}


void
Filter::subscribe()
{
  // If we're supposed to look for PointIndices (indices)
  if (use_indices_) {
    // Subscribe to the input using a filter
    sub_input_filter_.subscribe(this, "input", rmw_qos_profile_sensor_data);
    sub_indices_filter_.subscribe(this, "indices", rmw_qos_profile_sensor_data);

    if (approximate_sync_) {
      sync_input_indices_a_ =
        std::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloud2,
          pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_a_->connectInput(sub_input_filter_, sub_indices_filter_);
      // sync_input_indices_a_->registerCallback(std::bind(&Filter::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
      sync_input_indices_a_->registerCallback(&Filter::input_indices_callback, this);
    } else {
      sync_input_indices_e_ =
         std::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<PointCloud2,
           pcl_msgs::msg::PointIndices>>>(max_queue_size_);
      sync_input_indices_e_->connectInput(sub_input_filter_, sub_indices_filter_);
      // sync_input_indices_e_->registerCallback(std::bind(&Filter::input_indices_callback, this, std::placeholders::_1, std::placeholders::_2));
      sync_input_indices_e_->registerCallback(&Filter::input_indices_callback, this);
    }
  } else {
    // Subscribe in an old fashion to input only (no filters)
    std::function<void(const PointCloud2::SharedPtr cloud)> bound_callback_func =
      std::bind(&Filter::input_indices_callback, this, std::placeholders::_1, PointIndicesConstPtr());
    sub_input_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input", max_queue_size_, bound_callback_func);
  }
}

void
Filter::unsubscribe()
{
  if (use_indices_) {
    sub_input_filter_.unsubscribe();
    sub_indices_filter_.unsubscribe();
  } else {
    sub_input_.reset();
  }
}

void
Filter::input_indices_callback(
  const PointCloud2::SharedPtr cloud,
  const PointIndicesConstPtr indices)
{
  // If cloud is given, check if it's valid
  if (!isValid(cloud)) {
    RCLCPP_ERROR(this->get_logger(), "[Filter::input_indices_callback] Invalid input!");
    return;
  }
  // If indices are given, check if they are valid
  if (indices && !isValid(indices)) {
    RCLCPP_ERROR(this->get_logger(), "[Filter::input_indices_callback] Invalid indices!");
    return;
  }

  /// DEBUG
  // if (indices) {
  //   NODELET_DEBUG(
  //     "[%s::input_indices_callback]\n"
  //     "                                 - PointCloud with %d data points (%s), stamp %f, and "
  //     "frame %s on topic %s received.\n"
  //     "                                 - PointIndices with %zu values, stamp %f, and "
  //     "frame %s on topic %s received.",
  //     getName().c_str(),
  //     cloud->width * cloud->height, pcl::getFieldsList(*cloud).c_str(),
  //     cloud->header.stamp.toSec(), cloud->header.frame_id.c_str(), pnh_->resolveName(
  //       "input").c_str(),
  //     indices->indices.size(), indices->header.stamp.toSec(),
  //     indices->header.frame_id.c_str(), pnh_->resolveName("indices").c_str());
  // } else {
  //   NODELET_DEBUG(
  //     "[%s::input_indices_callback] PointCloud with %d data points and frame %s on "
  //     "topic %s received.",
  //     getName().c_str(), cloud->width * cloud->height,
  //     cloud->header.frame_id.c_str(), pnh_->resolveName("input").c_str());
  // }

  // Check whether the user has given a different input TF frame
  tf_input_orig_frame_ = cloud->header.frame_id;
  PointCloud2::ConstSharedPtr cloud_tf;
  if (!tf_input_frame_.empty() && cloud->header.frame_id != tf_input_frame_) {
    RCLCPP_DEBUG(this->get_logger(),
      "[Filter::input_indices_callback] Transforming input dataset from %s to %s.",
      cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
    // Save the original frame ID
    // Convert the cloud into the different frame
    PointCloud2 cloud_transformed;
    // if (!pcl_ros2::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, tf_listener_)) {
    if (!pcl_ros2::transformPointCloud(tf_input_frame_, *cloud, cloud_transformed, buffer_)) {
      RCLCPP_ERROR(this->get_logger(),
        "[Filter::input_indices_callback] Error converting input dataset from %s to %s.",
        cloud->header.frame_id.c_str(), tf_input_frame_.c_str());
      return;
    }
    cloud_tf = std::make_shared<PointCloud2>(cloud_transformed);
  } else {
    cloud_tf = cloud;
  }

  // Need setInputCloud () here because we have to extract x/y/z
  IndicesPtr vindices;
  if (indices) {
    vindices.reset(new std::vector<int>(indices->indices));
  }

  computePublish(cloud_tf, vindices);
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(pcl_ros2::Filter)