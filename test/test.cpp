/**
 * BSD 3-Clause LICENSE
 *
 * Copyright (c) 2018, Anirudh Topiwala
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without  
 * modification, are permitted provided that the following conditions are 
 * met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright 
 * notice, this list of conditions and the following disclaimer in the   
 * documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its 
 * contributors may be used to endorse or promote products derived from this 
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS 
 * IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 *  @file    test.cpp
 *  @author  Anirudh Topiwala
 *  @copyright BSD License
 *
 *  @brief Implementing publisher and subscriber node
 *
 *  @section DESCRIPTION
 *
 *  This program defines the subscriber
 *
 */

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <ros/service_client.h>
#include <gtest/gtest.h>
#include "beginner_tutorials/change_output_string.h"
#include <tf/tf.h>
/**
 * @brief      testing if the service exist and if its correcty changing the string
 *
 * @param[in]     TESTSuite
 * @param[in]     test_service_correct
 *
 * @return     none
 */
TEST(TESTSuite, testServiceCorrect) {
	
	// Creating NodeHandle
  ros::NodeHandle nh;

  // Create a srv object for service Change_String
  auto clt =
   nh.serviceClient<beginner_tutorials::change_output_string>("Change_String");

  // creating an object of Change_String
  beginner_tutorials::change_output_string srv;

  // assigning input to srv data
  srv.request.input = "New String";


  // checks if the service call works properly
  bool success = clt.call(srv);	

  EXPECT_TRUE(success);

  // compares the input and output string of the service
  EXPECT_STREQ("New String", srv.response.output.c_str());
}
/**
 * @brief      testing if the broadcaster is broadcasting values.
 *
 * @param[in]     TESTSuite
 * @param[in]     testBroadcaster
 *
 * @return     none
 */
TEST(TESTSuite, testBroadcaster) {
	
  // Creating NodeHandle
  ros::NodeHandle nh;

 tf::TransformListener listener;
 tf::StampedTransform transform;
 listener.waitForTransform("/world", "/talk", ros::Time(0), ros::Duration(10));
 listener.lookupTransform("/world", "/talk", ros::Time(0), transform);
 	
 auto x = transform.getOrigin().x();
 std::cout<< "x"<<x<<std::endl;
 EXPECT_EQ(x,5 );


}
