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

#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControl.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/sensor/Sensor.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/motion/MotionModel.h>

#include <limits>

namespace sp = scrimmage_proto;

void ExternalControl::init(std::map<std::string, std::string> &params) {
    init_client(params.at("server_address"));

    double inf = std::numeric_limits<double>::infinity();
    send_env(-inf, inf);
}

bool ExternalControl::step_autonomy(double t, double dt) {
    return send_action_result(t, 0, false);
}

void ExternalControl::init_client(std::string server_address) {
    external_control_client_ = ExternalControlClient(
            grpc::CreateChannel(server_address, grpc::InsecureChannelCredentials()));
}


bool ExternalControl::send_action_result(double t, double reward, bool done) {
    sp::ActionResult action_result;
    sp::SpaceSample *obs = action_result.mutable_observations();

    for (auto &kv : parent_->sensors()) {
        auto msg = kv.second->sense<sp::SpaceSample>(t);
        if (msg) {
            sp::SpaceSample &sample = (*msg)->data;
            for (int i = 0; i < sample.value_size(); i++) {
                obs->add_value(sample.value(i));
            }
        }
    }

    action_result.set_reward(reward);
    action_result.set_done(done);

    return external_control_client_.send_action_result(action_result, desired_state_);
}

scrimmage_proto::Environment ExternalControl::send_env(double min_reward, double max_reward) {
    sp::Environment env;

    *env.mutable_action_spaces() = parent_->motion()->action_space_params();

    sp::SpaceParams *obs_space = env.mutable_observation_spaces();

    for (auto &kv : parent_->sensors()) {
        auto obs_space_params = kv.second->observation_space_params();
        if (obs_space_params) {
            for (const sp::SingleSpaceParams params : obs_space_params->params()) {
                *obs_space->add_params() = params;
            }
        }
    }

    env.set_min_reward(min_reward);
    env.set_max_reward(max_reward);
    external_control_client_.send_environment(env, desired_state_);

    return env;
}
