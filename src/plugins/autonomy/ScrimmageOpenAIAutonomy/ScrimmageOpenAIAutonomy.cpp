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

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/math/State.h>
#include <scrimmage/sensor/Sensor.h>

#include <iostream>
#include <limits>

#include <boost/algorithm/string.hpp>

#include <chrono>  // NOLINT
#include <thread>  // NOLINT

namespace sp = scrimmage_proto;
namespace py = pybind11;

REGISTER_PLUGIN(scrimmage::Autonomy, scrimmage::autonomy::ScrimmageOpenAIAutonomy, ScrimmageOpenAIAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

ScrimmageOpenAIAutonomy::ScrimmageOpenAIAutonomy() :
    reward_range(-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()) {}

void ScrimmageOpenAIAutonomy::init(std::map<std::string, std::string> &params) {
    // import numpy asarray
    asarray = py::module::import("numpy").attr("asarray");

    // are we learning (true), or just using a trained model (false)?
    learning = scrimmage::get<bool>("learning", params, false);
    // get model python module
    const std::string module = params.at("module");
    py::object py_module = py::module::import(module.c_str());

    // model's action function
    const std::string py_act_fcn_str = params.at("act_fcn");
    py_act_fcn = py_module.attr(py_act_fcn_str.c_str());

    print_err_on_exit = false;
    return;
}

bool ScrimmageOpenAIAutonomy::step_autonomy(double t, double /*dt*/) {
    // clear previous step's actions
    action.discrete.clear();
    action.continuous.clear();

    if (!learning) {
        py::array_t<int> disc_actions;
        disc_actions = py::list();
        int* disc_action_data;

        const py::object thisaction = py_act_fcn(static_cast<int>(t));
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
