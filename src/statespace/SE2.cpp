////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019, Brian Lee, Vinitha Ranganeni
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

#include "statespace/SE2.hpp"
#include <assert.h>
#include <cmath>
#include <iostream>

namespace libcozmo {
namespace statespace {

SE2::~SE2() {
    for (int i = 0; i < m_state_map.size(); ++i) {
        delete(m_state_map[i]);
    }
    m_state_map.clear();
}

int SE2::get_or_create_state(const StateSpace::State& _state) {
    const State state = static_cast<const State&>(_state);
    const auto state_id = m_state_to_id_map.find(state);
    if (state_id != m_state_to_id_map.end()) {
        return state_id->second;
    }

    StateSpace::State* new_state = create_state();
    copy_state(state, new_state);
    m_state_to_id_map[state] = m_state_map.size() - 1;
    return m_state_map.size() - 1;    
}

int SE2::get_or_create_state(
    const aikido::statespace::StateSpace::State& _state) {
    State discrete_state;
    continuous_state_to_discrete(_state, &discrete_state);
    return get_or_create_state(discrete_state);
}

void SE2::discrete_state_to_continuous(
    const StateSpace::State& _state,
    aikido::statespace::StateSpace::State* _continuous_state) const {
    const State state = static_cast<const State&>(_state);
    
    Eigen::VectorXd state_log(3);
    state_log.head<2>() =
        discrete_position_to_continuous(Eigen::Vector2i(state.x, state.y));
    state_log[2] =
        discrete_angle_to_continuous(state.theta);
    m_statespace->expMap(state_log, _continuous_state);
}

void SE2::continuous_state_to_discrete(
    const aikido::statespace::StateSpace::State& _state, 
    StateSpace::State* _discrete_state) const {
    Eigen::VectorXd log_state;
    m_statespace->logMap(&_state, log_state);

    const Eigen::Vector2i position = 
        continuous_position_to_discrete(log_state.head<2>());
    const int theta = continuous_angle_to_discrete(log_state[2]);
    
    State* discrete_state = static_cast<State*>(_discrete_state);
    *discrete_state = State(position.x(), position.y(), theta);
}

bool SE2::get_state_id(const StateSpace::State& _state, int* _state_id) const {
    const State state = static_cast<const State&>(_state);
    const auto state_id_iter = m_state_to_id_map.find(state);
    if (state_id_iter != m_state_to_id_map.end()) {
        *_state_id = state_id_iter->second;
        return true;
    }
    return false;
}

StateSpace::State* SE2::get_state(const int& _state_id) const {
    if (_state_id >= size()) {
        return nullptr;
    }

    return m_state_map[_state_id];
}

bool SE2::is_valid_state(const StateSpace::State& _state) const {
    const State& state = static_cast<const State&>(_state);
    if (!(state.theta >= 0 && state.theta < m_num_theta_vals)) {
        return false;
    }
    return true;
}

int SE2::size() const {
    return m_state_map.size();
}

double SE2::get_distance(
    const StateSpace::State& _state_1,
    const StateSpace::State& _state_2) const {
    aikido::statespace::SE2::State continuous_state_1;
    discrete_state_to_continuous(_state_1, &continuous_state_1);  
    aikido::statespace::SE2::State continuous_state_2;  
    discrete_state_to_continuous(_state_2, &continuous_state_2);  
    return m_distance_metric.distance(&continuous_state_1, &continuous_state_2);
}

double SE2::get_distance(
    const aikido::statespace::StateSpace::State& _state_1,
    const aikido::statespace::StateSpace::State& _state_2) const {
    return m_distance_metric.distance(&_state_1, &_state_2);
}

void SE2::copy_state(
    const StateSpace::State& _source, StateSpace::State* _destination) const {
    const State& source = static_cast<const State&>(_source);
    State* destination = static_cast<State*>(_destination);
    *destination = State(source.x, source.y, source.theta);
}

double SE2::get_resolution() const { return m_resolution; }

StateSpace::State* SE2::create_state() {
    m_state_map.push_back(new State());
    const auto state = m_state_map.back();
    return m_state_map.back();
}

double SE2::normalize_angle_rad(const double& theta_rad) const {
    assert(m_bins % 2 == 0);
    double normalized_theta_rad = theta_rad;
    if (abs(theta_rad) > 2.0 * M_PI) {
        normalized_theta_rad = normalized_theta_rad -
            static_cast<int>(normalized_theta_rad / (2.0 * M_PI)) * 2.0 * M_PI;
    }
    if (theta_rad < 0) {
        normalized_theta_rad += 2.0 * M_PI;
    }
    return normalized_theta_rad;
}

double SE2::discrete_angle_to_continuous(const int& theta) const {
    return normalize_angle_rad(theta * (2 * M_PI /m_num_theta_vals));
}

int SE2::continuous_angle_to_discrete(const double& theta_rad) const {
    const double bin_size = 2.0 * M_PI / static_cast<double>(m_num_theta_vals);
    const int theta =
        normalize_angle_rad(theta_rad + bin_size / 2.0) / (2.0 * M_PI)
            * static_cast<double>(m_num_theta_vals);
    return static_cast<int>(theta);
}

Eigen::Vector2d SE2::discrete_position_to_continuous(
    const Eigen::Vector2i& position) const {
    const double x_mm = position.x() * m_resolution + (m_resolution / 2.0);
    const double y_mm = position.y() * m_resolution + (m_resolution / 2.0);
    return Eigen::Vector2d(x_mm, y_mm);
}

Eigen::Vector2i SE2::continuous_position_to_discrete(
    const Eigen::Vector2d& position) const {
        
    const int x = static_cast<int>(floor(position.x() / m_resolution));
    const int y = static_cast<int>(floor(position.y() / m_resolution));
    return Eigen::Vector2i(x, y);
}

}  // namespace statespace
}  // namespace libcozmo

