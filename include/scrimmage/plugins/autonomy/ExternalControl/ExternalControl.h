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

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_EXTERNALCONTROL_EXTERNALCONTROL_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_EXTERNALCONTROL_EXTERNALCONTROL_H_
#include <scrimmage/autonomy/Autonomy.h>
#include <scrimmage/plugins/autonomy/ExternalControl/ExternalControlClient.h>

#include <Eigen/Dense>

#include <map>
#include <string>

class ExternalControl : public scrimmage::Autonomy {
 public:
    virtual void init(std::map<std::string, std::string> &params);
    virtual bool step_autonomy(double t, double dt);
    virtual scrimmage_proto::SpaceParams action_space_params();

 protected:
    void init_client(std::string server_address);
    bool send_action_result(double t, double reward, bool done);
    bool send_env(double min_reward, double max_reward);

    ExternalControlClient external_control_client_;
    bool env_sent_ = false;
};

#endif // INCLUDE_SCRIMMAGE_PLUGINS_AUTONOMY_EXTERNALCONTROL_EXTERNALCONTROL_H_
