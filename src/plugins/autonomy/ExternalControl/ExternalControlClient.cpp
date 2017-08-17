#include <scrimmage/plugins/ExternalControlClient.h>

bool ExternalControlClient::send_environment(
        scrimmage_proto::Environment &env, scrimmage::State &state) {

    sp::State reply;
    grpc::ClientContext context;
    grpc::Status status = stub_->SendEnvironment(context, env, &reply);

    if (status.ok()) {
        state.set_pos(sc::proto_2_vector3d(reply.position()));
        state.set_vel(sc::proto_2_vector3d(reply.velocity()));
        state.set_quat(sc::proto_2_quat(reply.orientation()));
        return true;
    } else {
        return false;
    }
}

bool ExternalControlClient::send_action_result(
        scrimmage_proto::ActionResult &action_result, scrimmage::State &state) {
    sp::State reply;
    grpc::ClientContext context;
    grpc::Status status = stub_->SendActionResult(context, action_result, &reply);

    if (status.ok()) {
        state.set_pos(sc::proto_2_vector3d(reply.position()));
        state.set_vel(sc::proto_2_vector3d(reply.velocity()));
        state.set_quat(sc::proto_2_quat(reply.orientation()));
        return true;
    } else {
        return false;
    }
}
