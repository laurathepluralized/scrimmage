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

#include <scrimmage/plugins/interaction/SeedMessenger/SeedMessenger.h>

#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/Random.h>
#include <scrimmage/math/State.h>

#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>
#include <scrimmage/msgs/SeedMessage.pb.h>
#include <scrimmage/msgs/OutDirMsg.pb.h>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::SeedMessenger,
                SeedMessenger_plugin)

namespace scrimmage {
namespace interaction {

SeedMessenger::SeedMessenger() {
}

bool SeedMessenger::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    create_publisher("SeedMessage");
    create_publisher("OutDirMsg");

    prev_seed_ = -1;
    current_seed_ = -1;
    auto msg = std::make_shared<sc::Message<sm::OutDirMsg>>();
    msg->data.set_path(mp_->log_dir());
    publish_immediate(0, pubs_["OutDirMsg"], msg);
    return true;
}


bool SeedMessenger::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {

    current_seed_ = random_->get_seed();
    if (prev_seed_ != current_seed_) {
        auto msg = std::make_shared<sc::Message<sm::SeedMessage> >();
        msg->data.set_sim_seed(random_->get_seed());
        publish_immediate(t, pubs_["SeedMessage"], msg);
    }
    prev_seed_ = current_seed_;

    return true;
}
} // namespace interaction
} // namespace scrimmage
