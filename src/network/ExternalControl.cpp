#include <scrimmage/network/ExternalControl.h>

grpc::Status ExternalControl::GetEnvironment(
        grpc::ServerContext* context,
        const google::protobuf::Empty *request,
        scrimmage_proto::Environment *reply) {


    ready = true;
    std::string prefix("Hello ");
    reply->set_message(prefix + request->name());
    return grpc::Status::OK;
}
