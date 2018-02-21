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

#include <scrimmage/plugins/metrics/Seed2Summary/Seed2Summary.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/metrics/Metrics.h>
#include <scrimmage/common/Random.h>

#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Subscriber.h>
#include <scrimmage/msgs/Collision.pb.h>
#include <scrimmage/msgs/Event.pb.h>
#include <scrimmage/msgs/SeedMessage.pb.h>
#include <scrimmage/msgs/OutDirMsg.pb.h>

#include <iostream>
#include <limits>
#include <string>

using std::cout;
using std::endl;
using std::string;
using std::to_string;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::Metrics,
                scrimmage::metrics::Seed2Summary,
                Seed2Summary_plugin)

namespace scrimmage {
namespace metrics {

Seed2Summary::Seed2Summary() {
}

void Seed2Summary::init(std::map<std::string, std::string> &params) {
    create_subscriber("SeedMessage");
    create_subscriber("OutDirMsg");
}

bool Seed2Summary::step_metrics(double t, double dt) {
    for (auto msg : subs_["SeedMessage"]->msgs<sc::Message<sm::SeedMessage>>()) {
        auto theseed = msg->data.sim_seed();
        seedmap_[t] = theseed;
    }
    return true;
}

void Seed2Summary::calc_team_scores() {
    int suffix = 0;
    string time_header;
    string seed_header;
    for (auto kv_seed : seedmap_) {
        time_header = "seed_time" + to_string(suffix);
        seed_header = "seed" + to_string(suffix);
        headers_.push_back(seed_header);
        headers_.push_back(time_header);

        // Put the seed entry on each team's line
        for (auto theent : *id_to_team_map_) {
            team_metrics_[theent.second][time_header] = kv_seed.first;
            team_metrics_[theent.second][seed_header] = kv_seed.second;
        }
        suffix++;
    }
}

void Seed2Summary::print_team_summaries() {
    string time_header;
    string seed_header;
    cout << "     Time Header     |     Seed Time     |     Seed Header     |     Seed     " << endl;
    int suffix = 0;
    for (auto kv_seed : seedmap_) {
        time_header = "seed_time" + to_string(suffix);
        seed_header = "seed" + to_string(suffix);
        cout << time_header << to_string(kv_seed.first) << seed_header << to_string(kv_seed.second) << endl;
        suffix++;
    }
}
} // namespace metrics
} // namespace scrimmage
