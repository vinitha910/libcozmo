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


SE2::State::State(const int& x, const int& y, const int& theta) : \
    x(x), y(y), theta(theta) {}

bool SE2::State::operator== (const StateSpace::State& state) const {
    auto state_ = static_cast<const State&>(state);
    return x == state_.x && y == state_.y && theta == state_.theta;
}

Eigen::VectorXd SE2::State::vector() const {
    Eigen::VectorXd state_vector(3);
    state_vector << x, y, theta;
    return state_vector;
}

void SE2::State::from_vector(const Eigen::VectorXd& state) {
    if (state.size() != 3) {
        std::stringstream msg;
        msg << "state has incorrect size: expected 3"
            << ", got " << state.size() << ".\n";
        throw std::runtime_error(msg.str());
    }

    x = state[0];
    y = state[1];
    theta = state[2];
}

int SE2::State::X() const {
    return x;
}
int SE2::State::Y() const {
    return y;
}
int SE2::State::Theta() const {
    return theta;
}

SE2::ContinuousState::ContinuousState(
        const double& x, const double& y, const double& theta) : \
    x_mm(x), y_mm(y), theta_rad(theta) {}

bool SE2::ContinuousState::operator== (const StateSpace::ContinuousState& state) const {
    auto state_ = static_cast<const ContinuousState&>(state);
    return x_mm == state_.x_mm && y_mm == state_.y_mm && theta_rad == state_.theta_rad;
}

Eigen::VectorXd SE2::ContinuousState::vector() const {
    Eigen::VectorXd state_vector(3);
    state_vector << x_mm, y_mm, theta_rad;
    return state_vector;
}

void SE2::ContinuousState::from_vector(const Eigen::VectorXd& state) {
    if (state.size() != 3) {
        std::stringstream msg;
        msg << "state has incorrect size: expected 3"
            << ", got " << state.size() << ".\n";
        throw std::runtime_error(msg.str());
    }

    x_mm = state[0];
    y_mm = state[1];
    theta_rad = state[2];
}

double SE2::ContinuousState::X() const {
    return x_mm;
}

double SE2::ContinuousState::Y() const {
    return y_mm;
}

double SE2::ContinuousState::Theta() const {
    return theta_rad;
}

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
    const StateSpace::ContinuousState& _state) {
    State discrete_state;
    continuous_state_to_discrete(_state, &discrete_state);
    return get_or_create_state(discrete_state);
}

int SE2::get_or_create_state(
    const Eigen::VectorXd& _state) {
    if (_state.size() != 3) {
        std::stringstream msg;
        msg << "vector has incorrect size: expected 3"
            << ", got " << _state.size() << ".\n";
        throw std::runtime_error(msg.str());
    }
    State discrete_state(_state[0], _state[1], _state[2]);
    return get_or_create_state(discrete_state);
}

void SE2::discrete_state_to_continuous(
    const StateSpace::State& _state,
    StateSpace::ContinuousState* _continuous_state) const {
    const State state = static_cast<const State&>(_state);

    const Eigen::Vector2d position =
        discrete_position_to_continuous(Eigen::Vector2i(state.x, state.y));
    const double theta = discrete_angle_to_continuous(state.theta);

    ContinuousState* continuous_state =
        static_cast<ContinuousState*>(_continuous_state);
    *continuous_state = ContinuousState(position.x(), position.y(), theta);
}

void SE2::continuous_state_to_discrete(
    const StateSpace::ContinuousState& _state,
    StateSpace::State* _discrete_state) const {
    const ContinuousState state = static_cast<const ContinuousState&>(_state);
    const Eigen::Vector2i position =
        continuous_position_to_discrete(Eigen::Vector2d(state.x_mm, state.y_mm));
    const int theta = continuous_angle_to_discrete(state.theta_rad);

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
    ContinuousState continuous_state_1;
    discrete_state_to_continuous(_state_1, &continuous_state_1);
    ContinuousState continuous_state_2;
    discrete_state_to_continuous(_state_2, &continuous_state_2);
    return get_distance(continuous_state_1, continuous_state_2);
}

double SE2::get_distance(
    const StateSpace::ContinuousState& _state_1,
    const StateSpace::ContinuousState& _state_2) const {
    const ContinuousState state_1 =
        static_cast<const ContinuousState&>(_state_1);
    const ContinuousState state_2 =
        static_cast<const ContinuousState&>(_state_2); 
    Eigen::Vector3d diff;
    diff.head<2>() = state_1.vector().head<2>() - state_2.vector().head<2>();
    diff[2] = normalize_angle_rad(state_1.vector()[2] - state_2.vector()[2]);
    return diff.norm();     
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

