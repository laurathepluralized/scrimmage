/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_

#include <scrimmage/autonomy/Autonomy.h>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/eigen.h>

#include <map>
#include <vector>
#include <string>
#include <utility>

namespace py = pybind11;

namespace scrimmage {

struct EnvParams {
    std::vector<double> discrete_count;
    std::vector<std::pair<double, double>> continuous_extrema;
};

struct EnvValues {
    std::vector<int> discrete;
    std::vector<double> continuous;
};

namespace autonomy {

class ScrimmageOpenAIAutonomy : public scrimmage::Autonomy {
 public:
    ScrimmageOpenAIAutonomy();

    // normal overrides
    void init(std::map<std::string, std::string> &params) override;
    bool step_autonomy(double t, double dt) override;
    void get_action();

    // additional override
    virtual void set_environment() {}
    virtual std::pair<bool, double> calc_reward(double t, double dt);
    std::pair<double, double> reward_range;
    EnvParams action_space;
    EnvValues action;

    py::object py_obj_;
    py::object act_func_;
    bool learning_ = false;
};
}  // namespace autonomy
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
