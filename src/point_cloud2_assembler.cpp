/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/


#include "laser_assembler/base_assembler.h"
#include "sensor_msgs/point_cloud_conversion.hpp"

using namespace std ;

namespace laser_assembler
{

/**
 * \brief Maintains a history of incremental point clouds (usually from laser scans) and generates a point cloud upon request
 * \todo Clean up the doxygen part of this header
 * params
 *  * (Several params are inherited from BaseAssemblerSrv)
 */
class PointCloud2Assembler : public BaseAssembler<sensor_msgs::msg::PointCloud2>
{
public:
  PointCloud2Assembler(std::shared_ptr<rclcpp::Node> nh) : BaseAssembler<sensor_msgs::msg::PointCloud2>(nh, "max_clouds")
  {

  }

  ~PointCloud2Assembler()
  {

  }

  unsigned int GetPointsInScan(const sensor_msgs::msg::PointCloud2& scan)
  {
    return (scan.width * scan.height);
  }

  void ConvertToCloud(const string& fixed_frame_id, const sensor_msgs::msg::PointCloud2& scan_in, sensor_msgs::msg::PointCloud& cloud_out)
  {
    sensor_msgs::msg::PointCloud2 cloud_in;
    tf2_buffer_->transform(scan_in, cloud_in, fixed_frame_id);
    sensor_msgs::convertPointCloud2ToPointCloud(cloud_in, cloud_out);
    return ;
  }

private:

};

}

using namespace laser_assembler ;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("point_cloud_assembler");

  PointCloud2Assembler pc_assembler(node);
  pc_assembler.start("cloud");
  rclcpp::spin(node);

  return 0;
}
