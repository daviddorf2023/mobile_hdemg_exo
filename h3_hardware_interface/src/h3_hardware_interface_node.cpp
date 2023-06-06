/*LGPL-2.1*/
/*
 * Copyright (C) 2021  Technaid S.L. <www.technaid.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact:    <support@technaid.com>
 */


#include "h3_hardware_interface/h3_hardware_interface.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "h3_hardware_interface_node", ros::InitOption::AnonymousName);
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  int loop_hz;
  if (nh.getParam("loop_hz", loop_hz))
  {
    ROS_INFO("Loop frecuency retrieved from parameter server");
  }
  else
  {
    loop_hz = 100;
    ROS_WARN("Can't retrieve loop frecuency from parameter server, loop frecuency established to 100 hz");
  }

  ros::Time time_ps;
  ros::Duration duration(1 / loop_hz); // Initialized controller period
  ros::Rate loop_rate(loop_hz);

  h3_hardware_interface::H3Hardware exo_h3(nh);
  exo_h3.initialize();
  controller_manager::ControllerManager cm(&exo_h3, nh);

  while (ros::ok())
  {
    duration = ros::Time::now() - time_ps;

    exo_h3.read(ros::Time::now(), duration);
    cm.update(ros::Time::now(), duration);
    exo_h3.write(ros::Time::now(), duration);

    time_ps = ros::Time::now();
    loop_rate.sleep();
  }

  return 0;
}
