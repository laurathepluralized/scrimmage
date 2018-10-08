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
 * A int64 description goes here.
 *
 */

#ifndef INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHINTERACTION_H_
#define INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHINTERACTION_H_

#include <scrimmage/simcontrol/EntityInteraction.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/pubsub/Message.h>
#include <scrimmage/pubsub/Publisher.h>

#include <map>
#include <list>
#include <string>
#include <vector>
#include <deque>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/property_map/property_map.hpp>

namespace scrimmage {
namespace interaction {

class GraphInteraction : public scrimmage::EntityInteraction {
 public:
    GraphInteraction();
    bool init(std::map<std::string, std::string> &mission_params,
                std::map<std::string, std::string> &plugin_params) override;
    bool step_entity_interaction(std::list<scrimmage::EntityPtr> &ents,
                                    double t, double dt) override;

    unsigned int num_nodes() { return num_nodes_; }
 protected:
    struct GraphData {
        std::string Name;
    };

    struct NodeProperties {
        uint64_t osmid;           // d0 in non-simplified
        double x;               // d1 in non-simplified
        double y;               // d2 in non-simplified
//        std::string geometry;        // d3 in non-simplified
//        std::string xcoord;          // d4 in non-simplified
//        std::string ycoord;          // d5 in non-simplified
//        std::string highway;         // d6 in non-simplified
//        std::string ref;             // d7 in non-simplified

    };

    struct EdgeProperties {
        std::string geometry;        // d8 in non-simplified
        std::string highway;         // d9 in non-simplified
        double length;          // d10 in non-simplified
        // std::string oneway;          // d11 in non-simplified
        std::string osmid;           // d12 in non-simplified
        std::string name;            // d13 in non-simplified
        // std::string lanes;           // d14 in non-simplified
        // std::string ref;             // d15 in non-simplified
        // std::string bridge;          // d16 in non-simplified

    };

    typedef boost::adjacency_list<
        boost::vecS,
        boost::vecS,
        boost::directedS,
        NodeProperties, EdgeProperties> Graph;
    Graph g_;
 private:
    bool vis_graph_ = true;
    PublisherPtr pub_graph_;
    int id_ = 1;
    unsigned int num_nodes_ = 0;
};
} // namespace interaction
} // namespace scrimmage
#endif // INCLUDE_SCRIMMAGE_PLUGINS_INTERACTION_GRAPHINTERACTION_GRAPHINTERACTION_H_
