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

// #include <scrimmage/python/py_bindings_lib.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/math/State.h>
#include <scrimmage/sensor/Sensor.h>

#include <iostream>
#include <limits>
#include <unordered_map>
#include <vector>

#include <boost/algorithm/string.hpp>

#include <chrono>  // NOLINT
#include <thread>  // NOLINT

namespace sp = scrimmage_proto;
namespace py = pybind11;
namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ScrimmageOpenAIAutonomy, ScrimmageOpenAIAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

ScrimmageOpenAIAutonomy::ScrimmageOpenAIAutonomy() :
    reward_range(-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()) {}

void ScrimmageOpenAIAutonomy::init(std::map<std::string, std::string> &params) {
    // import numpy asarray
    py::module np = py::module::import("numpy");
    asarray = np.attr("asarray");

    py::object np_array = np.attr("array");
    py::object np_float32 = np.attr("float32");


    // are we learning (true), or just using a trained model (false)?
    learning = scrimmage::get<bool>("learning", params, false);
    // get model python module
    const std::string module = params.at("module");
    py::object py_module = py::module::import(module.c_str());

    // model's action function
    const std::string py_act_fcn_str = params.at("act_fcn");
    py_act_fcn = py_module.attr(py_act_fcn_str.c_str());

    py::list continuous_minima;
    py::list continuous_maxima;
    py::list discrete_count;

    // Sensors (with base class ScrimmageOpenAISensor) for non-learning mode
    for (auto &sens : parent_->sensors()) {
        auto s_cast =
            std::dynamic_pointer_cast<scrimmage::sensor::ScrimmageOpenAISensor>(sens.second);
        if (s_cast) {
            s_cast->set_observation_space();
            auto obs_space = s_cast->observation_space;
            // to_continuous(s_cast->observation_space.continuous_extrema, continuous_minima, continuous_maxima);
            to_discrete(s_cast->observation_space.discrete_count, discrete_count);
            sensors.push_back(s_cast);
        }
    }

    auto create_obs = [&](py::list &discrete_count, py::list &continuous_maxima) -> py::object {

        int len_discrete = py::len(discrete_count);
        int len_continuous = py::len(continuous_maxima);

        py::array_t<int> discrete_array(len_discrete);
        py::array_t<double> continuous_array(len_continuous);

        if (len_discrete > 0 && len_continuous > 0) {
            py::list obs;
            obs.append(discrete_array);
            obs.append(continuous_array);
            return obs;
        } else if (len_continuous > 0) {
            return continuous_array;
        } else {
            return discrete_array;
        }
    };

    observation = create_obs(discrete_count, continuous_maxima);

    print_err_on_exit = false;
    return;
}

bool ScrimmageOpenAIAutonomy::step_autonomy(double t, double /*dt*/) {
    // clear previous step's actions
    action.discrete.clear();
    action.continuous.clear();

    if (!learning) {

        auto call_get_obs = [&](auto *data, uint32_t &beg_idx, auto sensor, int obs_size) {
            uint32_t end_idx = beg_idx + obs_size;
            if (end_idx != beg_idx) {
                sensor->get_observation(data, beg_idx, end_idx);
                beg_idx = end_idx;
            }
        };

        py::array_t<double> cont_obs = observation.cast<py::array_t<double>>();

        uint32_t cont_beg_idx = 0;
        double* r_cont = static_cast<double *>(cont_obs.request().ptr);

        for (auto &s : sensors) {
            auto obs_space = s->observation_space;
            call_get_obs(r_cont, cont_beg_idx, s, obs_space.continuous_extrema.size());
            observation = asarray(r_cont);
            // py::print(observation);
        }

        py::array_t<int> disc_actions;
        disc_actions = py::list();
        int* disc_action_data;

        const py::object thisaction = py_act_fcn(observation);
        disc_actions = asarray(thisaction);
        disc_action_data = static_cast<int*>(disc_actions.request().ptr);
        for (int ii = 0; ii < disc_actions.size(); ++ii) {
            action.discrete.push_back(disc_action_data[ii]);
        }
    }
    return true;
}

std::pair<bool, double> ScrimmageOpenAIAutonomy::calc_reward(double /*t*/, double /*dt*/) {
    return {false, 0.0};
}

}  // namespace autonomy
}  // namespace scrimmage
