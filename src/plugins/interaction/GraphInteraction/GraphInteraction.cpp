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

#include <scrimmage/plugins/interaction/GraphInteraction/GraphInteraction.h>

#include <scrimmage/common/Utilities.h>
#include <scrimmage/common/FileSearch.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/parse/MissionParse.h>
#include <scrimmage/parse/ParseUtils.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>
#include <scrimmage/msgs/Graph.pb.h>


#include <memory>
#include <limits>
#include <iostream>
#include <fstream>
#include <deque>

#include <GeographicLib/LocalCartesian.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/optional.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_utility.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/dynamic_property_map.hpp>
#include <boost/property_map/property_map.hpp>

using std::cout;
using std::endl;
using std::stoi;


namespace fs = ::boost::filesystem;
namespace sc = scrimmage;
namespace sm = scrimmage_msgs;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::GraphInteraction,
                GraphInteraction_plugin)

namespace scrimmage {
namespace interaction {

GraphInteraction::GraphInteraction() {
}

std::ifstream& GotoLine(std::ifstream& file, unsigned int num) {
    file.seekg(std::ios::beg);
    for (unsigned int i = 0; i < num - 1; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    return file;
}

// Uses graph file to draw graph
bool GraphInteraction::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    pub_graph_ = advertise("GlobalNetwork", "Graph");
    std::string default_file_name = "default";
    std::string graph_file_name =
        sc::get<std::string>("graph_file", plugin_params, default_file_name);

    // If the data tag has been set for this plugin in the  mission file, get
    // the graph and label filenames from that instead of using those above.
    std::map<std::string, std::string> data_params;
    if (sc::parse_autonomy_data(plugin_params, data_params)) {
        FileSearch file_search;
        graph_file_name = sc::get<std::string>("graph_file", data_params,
                default_file_name);
        if (graph_file_name != default_file_name) {
            std::string graph_ext = fs::path(graph_file_name).extension().string();
            file_search.find_file(graph_file_name, graph_ext,
                    "SCRIMMAGE_DATA_PATH", graph_file_name);
        }
    }

    vis_graph_ = sc::get<bool>("visualize_graph", plugin_params, true);
    id_ = sc::get<int>("id", plugin_params, 1);
    if (graph_file_name == default_file_name)
        return true;

    std::ifstream graph_file(graph_file_name);

    boost::dynamic_properties dp(boost::ignore_other_properties);
    dp.property("osmid", boost::get(&NodeProperties::osmid, g_));
    dp.property("x", boost::get(&NodeProperties::x, g_));
    dp.property("y", boost::get(&NodeProperties::y, g_));
//    dp.property("geometry", boost::get(&NodeProperties::geometry, g_));
//    dp.property("xcoord", boost::get(&NodeProperties::xcoord, g_));
//    dp.property("ycoord", boost::get(&NodeProperties::ycoord, g_));
//    dp.property("highway", boost::get(&NodeProperties::highway, g_));
//    dp.property("ref", boost::get(&NodeProperties::ref, g_));

    dp.property("geometry", boost::get(&EdgeProperties::geometry, g_));
    dp.property("highway", boost::get(&EdgeProperties::highway, g_));
    dp.property("length", boost::get(&EdgeProperties::length, g_));
    // dp.property("oneway", boost::get(&EdgeProperties::oneway, g_));
    dp.property("osmid", boost::get(&EdgeProperties::osmid, g_));
    dp.property("name", boost::get(&EdgeProperties::name, g_));
    // dp.property("lanes", boost::get(&EdgeProperties::lanes, g_));
    // dp.property("ref", boost::get(&EdgeProperties::ref, g_));
    // dp.property("bridge", boost::get(&EdgeProperties::bridge, g_));

    if (graph_file.is_open()) {
        boost::read_graphml(graph_file, g_, dp);
        cout << "read graph file" << endl;
    }

    // Graph::vertex_iterator vertex_it, vertex_end;
    Graph::adjacency_iterator neigh_it, neigh_end;
    // std::tie(vertex_it, vertex_end) = boost::vertices(g_);
    std::pair<boost::adjacency_list<>::vertex_iterator,
        boost::adjacency_list<>::vertex_iterator> vs = boost::vertices(g_);

    uint64_t node_counter = 0;
    std::map<uint64_t, Eigen::Vector3d> nodes;
    std::map<uint64_t, uint64_t> osmid_to_boost_vert;
    std::map<uint64_t, std::pair<double, double>> osmid_to_lon_lat;

    auto graph_msg = std::make_shared<sc::Message<sm::Graph>>();
    graph_msg->data.set_id(id_);
    for (; vs.first != vs.second; ++vs.first) {
        // double longitude = boost::get(&NodeProperties::x, g_, *vs.first);
        // double latitude = boost::get(&NodeProperties::y, g_, *vs.first);
        // uint64_t this_osmid = boost::get(&NodeProperties::osmid, g_, *vs.first);
        double longitude = g_[*vs.first].x;
        double latitude = g_[*vs.first].y;
        double this_osmid = g_[*vs.first].osmid;
        osmid_to_boost_vert[this_osmid] = *vs.first;
        osmid_to_lon_lat[this_osmid].first = longitude;
        osmid_to_lon_lat[this_osmid].second = latitude;
        double x, y, z;
        parent_->projection()->Forward(latitude, longitude, 0.0, x, y, z);
        z = 200;  // dummy value for now
        nodes[this_osmid] = Eigen::Vector3d(x, y, z);
        auto node_ptr = graph_msg->data.add_nodes();
        node_ptr->set_id(this_osmid);
        sc::set(node_ptr->mutable_point(), nodes[this_osmid]);
        node_counter++;
        // TODO: Here or elsewhere, figure out why I can't just iterate over
        // edges in the same way I do with nodes. Why won't this work?!
        // cout << boost::out_edges(*vs.first, g_) << endl;
        // std::pair<boost::adjacency_list<>::edge_iterator,
        //     boost::adjacency_list<>::edge_iterator> es = boost::out_edges(*vs.first, g_);
        // for (; es.first != es.second; ++es.first) {
        //     cout << "out edge " << *es.first << endl;
        // }
    }
    cout << "final node count is " << node_counter << endl;
    cout << "boost says num_vertices is " << num_vertices(g_) << endl;



        /* // Read the nodes and edges */
        /* unsigned int edge_counter = 0; */
        /* while (std::getline(graph_file, line)) { */
        /*     std::vector<std::string> words; */
        /*     boost::split(words, line, [](char c) { return c == ' '; }); */
        /*  */
        /*     // is edge */
        /*         edge_counter++; */
        /*         int id_start = std::stoi(words[0]), id_end = std::stoi(words[1]); */
        /*         double length = std::stod(words[2]); */
        /*  */
        /*         bool edge_nodes_exist = */
        /*             nodes.count(id_start) > 0 && nodes.count(id_end) > 0; // reality check */
        /*         if (edge_nodes_exist) { */
        /*             auto edge_ptr = graph_msg->data.add_edges(); */
        /*             edge_ptr->set_start_node_id(id_start); */
        /*             edge_ptr->set_end_node_id(id_end); */
        /*             edge_ptr->set_weight(length); */
        /*  */
        /*             // Visualize the Edges */
        /*             if (vis_graph_) { */
        /*                 auto edge_shape = std::make_shared<scrimmage_proto::Shape>(); */
        /*                 edge_shape->set_persistent(true); */
        /*                 edge_shape->set_opacity(1.0); */
        /*                 scrimmage::set(edge_shape->mutable_color(), 0, 0, 0); */
        /*                 scrimmage::set(edge_shape->mutable_line()->mutable_start(), nodes[id_start]); */
        /*                 scrimmage::set(edge_shape->mutable_line()->mutable_end(), nodes[id_end]); */
        /*                 draw_shape(edge_shape); */
        /*             } */
        /*         } */
        /*     } */
        /* } */
        pub_graph_->publish(graph_msg);

        if (vis_graph_) {
            int counter_viz = 0;
            auto node_shape = std::make_shared<scrimmage_proto::Shape>();
            node_shape->set_persistent(true);
            for (auto node : nodes) {
                scrimmage::set(node_shape->mutable_pointcloud()->add_point(),
                        node.second[0], node.second[1], node.second[2]);
                sc::set(node_shape->mutable_pointcloud()->add_color(),
                        0, 0, 255);
                if (counter_viz % 1000 == 0) {
                    cout << counter_viz << endl;
                }
                counter_viz++;
            }
            node_shape->mutable_pointcloud()->set_size(6);
            draw_shape(node_shape);
        }

        graph_file.close();

    return true;
}

bool GraphInteraction::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (ents.empty()) {
        return true;
    }

    return true;
}

}  // namespace interaction
}  // namespace scrimmage
