/*
Copyright (c) 2019-2020, Juan Miguel Jimeno
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef BODY_CONTROLLER_H
#define BODY_CONTROLLER_H

#include <geometry/geometry.h>
#include <quadruped_base/quadruped_base.h>
#include <quadruped_base/quadruped_leg.h>
#include <kinematics/kinematics.h>

namespace champ
{
    class BodyController
    {
        QuadrupedBase *base_;

        public:
            BodyController(QuadrupedBase &quadruped_base):
                base_(&quadruped_base)
            {
            }

            void poseCommand(geometry::Transformation (&foot_positions)[4], 
                             const champ::Pose &req_pose)
            {
                for(int i = 0; i < 4; i++)
                {
                    poseCommand(foot_positions[i], *base_->legs[i], req_pose);
                }
            }
            
            static void poseCommand(geometry::Transformation &foot_position, 
                                    champ::QuadrupedLeg &leg, 
                                    const champ::Pose &req_pose)
            {
                float delta_height = leg.zero_stance().Z() + req_pose.position.z;

                //create a new foot position from position of legs when stretched out
                foot_position = leg.zero_stance();

                //move the foot position to desired height of the robot
                foot_position.Translate(0.0f, 0.0f, -delta_height);

                //rotate the leg opposite the required orientation of the body
                foot_position.RotateZ(-req_pose.orientation.yaw);
                foot_position.RotateY(-req_pose.orientation.pitch);
                foot_position.RotateX(-req_pose.orientation.roll);
            
    
                champ::Kinematics::transformToHip(foot_position, leg);
            }
    };
}

#endif