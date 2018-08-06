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

#include <scrimmage/entity/Entity.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/plugins/autonomy/ScrimmageOpenAIAutonomy/ScrimmageOpenAIAutonomy.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/math/State.h>
#include <scrimmage/sensor/Sensor.h>

#include <iostream>
#include <limits>
#include <cstddef>
#include <stdexcept>

#include <boost/algorithm/string.hpp>

#include <chrono> // NOLINT
#include <thread> // NOLINT

namespace sc = scrimmage;
namespace py = pybind11;
namespace sp = scrimmage_proto;

REGISTER_PLUGIN(sc::Autonomy,
        sc::autonomy::ScrimmageOpenAIAutonomy,
        ScrimmageOpenAIAutonomy_plugin)

namespace scrimmage {
namespace autonomy {

ScrimmageOpenAIAutonomy::ScrimmageOpenAIAutonomy() :
    reward_range(-std::numeric_limits<double>::infinity(),
                 std::numeric_limits<double>::infinity()) {}

void ScrimmageOpenAIAutonomy::init(std::map<std::string, std::string> &params) {
    asarray_ = py::module::import("numpy").attr("asarray");

    if (!learning_) {
        py::module module = py::module::import(params.at("module").c_str());
        py::object py_obj_class = module.attr(params.at("class").c_str());
        py_obj_ = py_obj_class();
        act_func_ = py_obj_.attr(params.at("act_func").c_str());
    }

    auto sensor_cb = [&](auto &msg) {

        obs_ = msg->data;

        /*
        py::array_t<int> disc_obs;
        py::array_t<double> cont_obs;
        py::list obs_list = obs_.cast<py::list>();
        disc_obs = obs_list[0].cast<py::array_t<int>>();
        cont_obs = obs_list[1].cast<py::array_t<double>>();

        uint32_t disc_beg_idx = 0;
        uint32_t cont_beg_idx = 0;
        int* r_disc = static_cast<int *>(disc_obs.request().ptr);
        double* r_cont = static_cast<double *>(cont_obs.request().ptr);
        */
    };

    print_err_on_exit = false;
    return;
}

bool ScrimmageOpenAIAutonomy::step_autonomy(double /*t*/, double /*dt*/) {

    if (!learning_) {
        py::object py_action = act_func_(obs_);
    } else {
    }

    return false;
}

void ScrimmageOpenAIAutonomy::get_action() {
    if (learning_) {
        return;
    }

    py::object py_obs = obs_;
    py::object py_action = act_func_(py_obs); // actor_func comes from the init function

    py::tuple action_list = py_action_.cast<py::list>();
    py::array_t<int> disc_actions;
    py::array_t<double> cont_actions;
    disc_actions = asarray_(action_list[0], py::str("int"));
    cont_actions = asarray_(action_list[1], py::str("float"));
    // The following two lines were the ones esquires3 pointed out as extra important here
    action.discrete.push_back(*static_cast<int*>(disc_actions.request().ptr));
    action.continuous.push_back(*static_cast<double*>(cont_actions.request().ptr));

    // action =  // something from py_openai_env.cpp
}

std::pair<bool, double> ScrimmageOpenAIAutonomy::calc_reward(double /*t*/, double /*dt*/) {
    return {false, 0.0};
}

}  // namespace autonomy
}  // namespace scrimmage
