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

#include <scrimmage/plugins/metrics/RLSimpleMetrics/RLSimpleMetrics.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/msgs/RLTestMsgs.pb.h>

#include <iostream>
#include <limits>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::RLSimpleMetrics,
                RLSimpleMetrics_plugin)

namespace scrimmage {
namespace metrics {

RLSimpleMetrics::RLSimpleMetrics() {
}

void RLSimpleMetrics::init(std::map<std::string, std::string> &params) {
    auto reward_cb = [&] (scrimmage::MessagePtr<RLTestMsgs::Reward> msg) {
        auto data = msg->data;
        rewards_[data.sender_team_id()] += data.reward();
    };
    subscribe<RLTestMsgs::Reward>("GlobalNetwork", "RLSimpleScore", reward_cb);
}

bool RLSimpleMetrics::step_metrics(double t, double dt) {
    if (!initialized_) {
        for (auto &kv : *id_to_team_map_) {
            // initialize rewards for all teams
            rewards_[kv.second] = 0;
        }
        initialized_ = true;
    }
    return true;
}

void RLSimpleMetrics::calc_team_scores() {
    for (auto &rew: rewards_) {
        team_metrics_[rew.first]["reward"] = rew.second;
    }

    // list the headers we want put in the csv file
    headers_.push_back("reward");
}

void RLSimpleMetrics::print_team_summaries() {
    for (auto &rew: rewards_) {
        cout << "Team " << rew.first << "'s Reward: " << rew.second << endl;
        cout << sc::generate_chars("-", 70) << endl;
    }

}
} // namespace metrics
} // namespace scrimmage
