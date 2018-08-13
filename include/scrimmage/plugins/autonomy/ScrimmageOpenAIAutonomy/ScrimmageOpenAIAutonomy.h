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
#include <scrimmage/sensor/Sensor.h>

#include <pybind11/pybind11.h>
#include <pybind11/embed.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

#include <string>
#include <memory>
#include <vector>
#include <thread>  // NOLINT
#include <utility>
#include <tuple>
#include <algorithm>
#include <limits>
#include <chrono>  // NOLINT
#include <map>

#include <boost/optional.hpp>
#include <boost/range/numeric.hpp>
#include <boost/range/adaptor/transformed.hpp>
#include <boost/range/adaptor/map.hpp>
#include <boost/range/algorithm/min_element.hpp>

namespace py = pybind11;
namespace sc = scrimmage;
namespace ba = boost::adaptors;
namespace br = boost::range;

namespace scrimmage {
namespace sensor {
    // not sure why cppclean is compaining about this, since the
    // compiler can't make any progress in compiling without this block
    // but cppclean doesn't have a // NOLINT type line-by-line ignore
    // thing from what I can tell
    class ScrimmageOpenAISensor;
}

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
    bool learning_ = false;
    py::object get_gym_space(const std::string &type);
    py::object create_space(py::list discrete_count, py::list continuous_minima, py::list continuous_maxima);
    void to_continuous(std::vector<std::pair<double, double>> &p,
                                           py::list &minima,
                                           py::list &maxima);
    void to_discrete(std::vector<double> &p, py::list &maxima);

    void create_action_space();
    void create_observation_space();

    py::object py_observation_;
    py::object py_observation_space_;
    py::object py_obj_;
    py::object act_func_;
    py::object py_action_space_;
    py::object py_action_;
    py::object asarray_;

 protected:
    bool combine_actors_ = false;
    bool global_sensor_ = false;
    using ExternalControlPtr = std::shared_ptr<sc::autonomy::ScrimmageOpenAIAutonomy>;
    using ScrimmageOpenAISensorPtr = std::shared_ptr<sc::sensor::ScrimmageOpenAISensor>;

    std::vector<ExternalControlPtr> ext_ctrl_vec_;
    std::vector<std::vector<ScrimmageOpenAISensorPtr> > ext_sensor_vec_;
};
}  // namespace autonomy
}  // namespace scrimmage
#endif  // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_SCRIMMAGEOPENAIAUTONOMY_SCRIMMAGEOPENAIAUTONOMY_H_
