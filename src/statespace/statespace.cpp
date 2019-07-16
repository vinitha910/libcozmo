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

#include "statespace.h"
#include <assert.h>
#include <cmath>
#include "aikido/distance/SE2.hpp"
#include "aikido/statespace/SE2.hpp"
#include "cozmo_description/cozmo.hpp"


namespace grid_planner {
namespace statespace {
using aikido::statespace::SE2;

//newly added axis theta, [0,2pi], discretized by 2^n buckets
int Statespace::set_start_state(const int& x, const int& y, const int& theta) {
    if (is_valid_state(x, y, theta)) {
        m_start_id = get_state_id(x, y, theta);
        return m_start_id;
    }
    return -1;
}

int Statespace::set_goal_state(const int& x, const int& y, const int& theta) {
    if (is_valid_state(x, y, theta)) {
        m_goal_id = get_state_id(x, y, theta);
        return m_goal_id;
    }
    return -1;
}

int Statespace::create_new_state(const double& x, const double& y, const double& theta) {
    SE2::State s;
    Eigen::Isometry2d t = Eigen::Isometry2d::Identity();
    const Eigen::Rotation2D<double> rot(theta);
    t.linear() = rot.toRotationMatrix();
    Eigen::Vector2d trans;
    trans << x, y;
    t.translation() = trans;
    s.setIsometry(t);

    //get id
    //insert id, state mapping into the vector

    return s;
}

int Statespace::get_or_create_new_state(const int& x, const int& y, const int& theta) {
    //if s exists at given 3 axis, return s
    //else create one 
    
    int input_state_id = get_state_id(x, y, theta);


    else {
        return create_new_state(x, y, theta);
    }
}


void Statespace::get_path_coordinates(
    const std::vector<int>& path_state_ids,
    std::vector<std::pair<int, int> > *path_coordinates) const {
    for (int i = 0; i < path_state_ids.size(); ++i) {
        int state_id = path_state_ids[i];
        int x, y, theta;
        if (get_coord_from_state_id(state_id, &x, &y, &theta)) {
            path_coordinates->push_back(std::make_pair(x, y, theta));
        }
    }
}

int Statespace::get_state_id(const int& x, const int& y, const int& theta) const {
    assert(x < m_width);
    assert(y < m_height);
    //int theta = normalize_angle_rad(theta_rad);
    assert(theta <= m_bins);
    return (theta * m_width * m_height) + y * m_width + x; 
}

bool Statespace::get_coord_from_state_id(const int& state_id, int* x, int* y, int* theta) const {
    assert(state_id < m_occupancy_grid.size());
    int theta_val = state_id / (m_width * m_height);
    state_id = (state_id - theta * m_width * m_height);
    int y_val = state_id / m_width;
    int x_val = state_id - y_val * m_width;
    *x = x_val;
    *y = y_val;
    *theta = theta_val;

    return (is_valid_state(*x, *y, *theta));
}

bool Statespace::is_valid_state(const int& x, const int& y, const int& theta) const {
    if (!(x >= 0 && x < m_width && y >= 0 && y < m_height)) {
        return false;
    }

    if (!(theta >= 0 && theta <= m_bins)) {
        return false;
    }

    int state_id = get_state_id(x, y, theta);
    if (m_occupancy_grid[state_id] == 0) {
        return true;
    }
    return false;
}

double Statespace::get_action_cost(
        const SE2::State& source,
        const SE2::State& succ) {
    return distance(source, succ);
}

int Statespace::normalize_angle_rad(const int& theta_rad) {
    assert(m_bins % 2 == 0);
    
    // I hope this works
    if !(theta_rad >= 0 && theta_rad <= 2 * math.pi) {
        theta_rad = theta_rad % (2 * math.pi);
    }
    return theta_rad;
}

int Statespace::discrete_angle_to_continuous(const int& theta) {
    double rad = 2 * math.pi / m_bins * theta;
    return rad;
}

int Statespace::continuous_angle_to_discrete(cosnt int& theta_rad) {
    //placement into a bin
    int discrete = int(theta_rad / (2 * math.pi / m_bins));
    return discrete;
}

}  // namespace statespace
}  // namespace grid_planner

