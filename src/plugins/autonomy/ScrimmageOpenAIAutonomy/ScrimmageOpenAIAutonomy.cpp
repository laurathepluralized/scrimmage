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
        py::module mymodule = py::module::import(params.at("py_file").c_str());
        py::object py_obj_class = mymodule.attr(params.at("class").c_str());
        py_obj_ = py_obj_class();
        act_func_ = py_obj_.attr(params.at("act_func").c_str());
    }

    auto sensor_cb = [&](auto &msg) {

        py_observation_ = msg->data;

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
        py::object py_action = act_func_(py_observation_);
    } else {
    }

    return false;
}


py::object ScrimmageOpenAIAutonomy::get_gym_space(const std::string &type) {
    return py::module::import("gym").attr("spaces").attr(type.c_str());
}

py::object ScrimmageOpenAIAutonomy::create_space(
        py::list discrete_count,
        py::list continuous_minima,
        py::list continuous_maxima) {

    py::module np = py::module::import("numpy");
    py::object np_array = np.attr("array");
    py::object np_float32 = np.attr("float32");

    py::object gym_discrete_space = get_gym_space("Discrete");
    py::object gym_multidiscrete_space = get_gym_space("MultiDiscrete");
    py::object gym_box_space = get_gym_space("Box");
    py::object gym_tuple_space = get_gym_space("Tuple");

    py::object discrete_space = py::len(discrete_count) == 1 ?
        gym_discrete_space(discrete_count[0]) :
        gym_multidiscrete_space(discrete_count);

    py::object continuous_space = gym_box_space(
        np_array(continuous_minima),
        np_array(continuous_maxima),
        py::none(),
        np_float32);

    int len_discrete = py::len(discrete_count);
    int len_continuous = py::len(continuous_minima);
    if (len_discrete != 0 && len_continuous != 0) {
        py::list spaces;
        spaces.append(discrete_space);
        spaces.append(continuous_space);
        return gym_tuple_space(spaces);
    } else if (len_discrete != 0) {
        return discrete_space;
    } else if (len_continuous != 0) {
        return continuous_space;
    } else {
        // TODO: error handling
        return py::object();
    }
}


void ScrimmageOpenAIAutonomy::to_continuous(std::vector<std::pair<double, double>> &p,
                                       py::list &minima,
                                       py::list &maxima) {
    for (auto &value : p) {
        py::list min_max;
        minima.append(value.first);
        maxima.append(value.second);
    }
}

void ScrimmageOpenAIAutonomy::to_discrete(std::vector<double> &p, py::list &maxima) {
    for (auto &value : p) {
        maxima.append(value);
    }
}

void ScrimmageOpenAIAutonomy::create_action_space() {

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_count;
        py::list continuous_minima;
        py::list continuous_maxima;

        for (auto &a : ext_ctrl_vec_) {
            a->set_environment();
            to_discrete(a->action_space.discrete_count, discrete_count);
            to_continuous(a->action_space.continuous_extrema, continuous_minima, continuous_maxima);
        }

        py_action_space_ =
            create_space(discrete_count, continuous_minima, continuous_maxima);

    } else {

        py::list action_spaces;

        for (auto &a : ext_ctrl_vec_) {
            py::list discrete_count;
            py::list continuous_minima;
            py::list continuous_maxima;

            a->set_environment();
            to_discrete(a->action_space.discrete_count, discrete_count);
            to_continuous(a->action_space.continuous_extrema, continuous_minima, continuous_maxima);

            auto space =
                create_space(discrete_count, continuous_minima, continuous_maxima);
            action_spaces.append(space);
        }

        py::object tuple_space = get_gym_space("Tuple");
        py_action_space_ = tuple_space(action_spaces);
    }
}

void ScrimmageOpenAIAutonomy::create_observation_space() {

    py::object tuple_space = get_gym_space("Tuple");

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

    if (ext_ctrl_vec_.size() == 1 || combine_actors_) {
        py::list discrete_count;
        py::list continuous_minima;
        py::list continuous_maxima;

        bool done = false;
        for (auto &v : ext_sensor_vec_) {
            if (done) break;
            for (auto &s : v) {
                s->set_observation_space();
                to_discrete(s->observation_space.discrete_count, discrete_count);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);

                if (global_sensor_) {
                    done = true;
                    break;
                }
            }
        }

        py_observation_space_ =
            create_space(discrete_count, continuous_minima, continuous_maxima);

        py_observation_ = create_obs(discrete_count, continuous_maxima);

    } else {

        py::list observation_spaces;
        py::list obs;

        for (auto &v : ext_sensor_vec_) {
            for (auto &s : v) {
                py::list discrete_count;
                py::list continuous_minima;
                py::list continuous_maxima;

                s->set_observation_space();
                to_discrete(s->observation_space.discrete_count, discrete_count);
                to_continuous(s->observation_space.continuous_extrema, continuous_minima, continuous_maxima);

                auto space =
                    create_space(discrete_count, continuous_minima, continuous_maxima);
                observation_spaces.append(space);
                obs.append(create_obs(discrete_count, continuous_maxima));
            }
        }

        py_observation_space_ = tuple_space(observation_spaces);
        py_observation_ = obs;
    }
}





void ScrimmageOpenAIAutonomy::get_action() {
    if (learning_) {
        return;
    }

    py::object py_obs = py_observation_;
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
