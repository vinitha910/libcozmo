////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Vinitha Ranganeni
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

#ifndef SRC_ACTIONSPACE_ACTIONSPACE_H_
#define SRC_ACTIONSPACE_ACTIONSPACE_H_

#include <utility>

namespace libcozmo {
namespace actionspace {


// This class implements a 2D grid-based graph representation
class GenericActionspace {
    public:
        // constructor, sets min/max threshold
        // and initializes cozmo SDK handle
        GenericActionspace(const float& lin_min,
                           const float& lin_max,
                           const int& lin_samples,
                           const float& ang_min,
                           const float& ang_max,
                           const int& ang_samples,
                           const float& dur_min,
                           const float& dur_max,
                           const int& dur_samples,
                           const std::string& mesh_dir);
        
        // applies action on the SDK
        // action chosen index-wise
        void applyAction(const int& actionIdx);

        // generates a set of actions
        // that cozmo may handle
        void generateAction();

        // shows current actionspace
        System.String viewActionSpace();

    private:
        //cozmo robot handle
        libcozmo::Cozmo m_cozmo;
        // minimum linear velocity, mm/s in scale
        float m_lin_min;
        // maximum linear velocity, mm/s in scale
        float m_lin_max;
        // number of linear velocity samples to generate
        int m_lin_samples;
        //minimum angular velocity, mm/s in scale
        float m_ang_min;
        //maximum angular velocity, mm/s in scale
        float m_ang_max;
        //number of angular vel samples to generate
        int m_ang_samples;
        // minimum duration, s in scale
        float m_dur_min;
        // maximum duration, s in scale
        float m_dur_max;
        //number of duration samples to generate
        int m_dur_samples;
        // set of actions generated
        Action m_actions;
};

}  // namespace actionspace
}  // namespace libcozmo

#endif  // SRC_ACTIONSPACE_ACTIONSPACE_H_
