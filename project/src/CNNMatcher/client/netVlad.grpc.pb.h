// Generated by the gRPC C++ plugin.
// If you make any local change, they will be lost.
// source: netVlad.proto
#ifndef GRPC_netVlad_2eproto__INCLUDED
#define GRPC_netVlad_2eproto__INCLUDED

#include "netVlad.pb.h"

#include <grpc++/impl/codegen/async_stream.h>
#include <grpc++/impl/codegen/async_unary_call.h>
#include <grpc++/impl/codegen/method_handler_impl.h>
#include <grpc++/impl/codegen/proto_utils.h>
#include <grpc++/impl/codegen/rpc_method.h>
#include <grpc++/impl/codegen/service_type.h>
#include <grpc++/impl/codegen/status.h>
#include <grpc++/impl/codegen/stub_options.h>
#include <grpc++/impl/codegen/sync_stream.h>

namespace grpc {
class CompletionQueue;
class Channel;
class RpcService;
class ServerCompletionQueue;
class ServerContext;
}  // namespace grpc

namespace netVlad {

// The greeting service definition.
class NetConnect final {
 public:
  static constexpr char const* service_full_name() {
    return "netVlad.NetConnect";
  }
  class StubInterface {
   public:
    virtual ~StubInterface() {}
    // Sends a greeting
    virtual ::grpc::Status img2vec(::grpc::ClientContext* context, const ::netVlad::NetRequest& request, ::netVlad::NetResponse* response) = 0;
    std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::netVlad::NetResponse>> Asyncimg2vec(::grpc::ClientContext* context, const ::netVlad::NetRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReaderInterface< ::netVlad::NetResponse>>(Asyncimg2vecRaw(context, request, cq));
    }
  private:
    virtual ::grpc::ClientAsyncResponseReaderInterface< ::netVlad::NetResponse>* Asyncimg2vecRaw(::grpc::ClientContext* context, const ::netVlad::NetRequest& request, ::grpc::CompletionQueue* cq) = 0;
  };
  class Stub final : public StubInterface {
   public:
    Stub(const std::shared_ptr< ::grpc::ChannelInterface>& channel);
    ::grpc::Status img2vec(::grpc::ClientContext* context, const ::netVlad::NetRequest& request, ::netVlad::NetResponse* response) override;
    std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::netVlad::NetResponse>> Asyncimg2vec(::grpc::ClientContext* context, const ::netVlad::NetRequest& request, ::grpc::CompletionQueue* cq) {
      return std::unique_ptr< ::grpc::ClientAsyncResponseReader< ::netVlad::NetResponse>>(Asyncimg2vecRaw(context, request, cq));
    }

   private:
    std::shared_ptr< ::grpc::ChannelInterface> channel_;
    ::grpc::ClientAsyncResponseReader< ::netVlad::NetResponse>* Asyncimg2vecRaw(::grpc::ClientContext* context, const ::netVlad::NetRequest& request, ::grpc::CompletionQueue* cq) override;
    const ::grpc::RpcMethod rpcmethod_img2vec_;
  };
  static std::unique_ptr<Stub> NewStub(const std::shared_ptr< ::grpc::ChannelInterface>& channel, const ::grpc::StubOptions& options = ::grpc::StubOptions());

  class Service : public ::grpc::Service {
   public:
    Service();
    virtual ~Service();
    // Sends a greeting
    virtual ::grpc::Status img2vec(::grpc::ServerContext* context, const ::netVlad::NetRequest* request, ::netVlad::NetResponse* response);
  };
  template <class BaseClass>
  class WithAsyncMethod_img2vec : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithAsyncMethod_img2vec() {
      ::grpc::Service::MarkMethodAsync(0);
    }
    ~WithAsyncMethod_img2vec() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status img2vec(::grpc::ServerContext* context, const ::netVlad::NetRequest* request, ::netVlad::NetResponse* response) final override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    void Requestimg2vec(::grpc::ServerContext* context, ::netVlad::NetRequest* request, ::grpc::ServerAsyncResponseWriter< ::netVlad::NetResponse>* response, ::grpc::CompletionQueue* new_call_cq, ::grpc::ServerCompletionQueue* notification_cq, void *tag) {
      ::grpc::Service::RequestAsyncUnary(0, context, request, response, new_call_cq, notification_cq, tag);
    }
  };
  typedef WithAsyncMethod_img2vec<Service > AsyncService;
  template <class BaseClass>
  class WithGenericMethod_img2vec : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithGenericMethod_img2vec() {
      ::grpc::Service::MarkMethodGeneric(0);
    }
    ~WithGenericMethod_img2vec() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable synchronous version of this method
    ::grpc::Status img2vec(::grpc::ServerContext* context, const ::netVlad::NetRequest* request, ::netVlad::NetResponse* response) final override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
  };
  template <class BaseClass>
  class WithStreamedUnaryMethod_img2vec : public BaseClass {
   private:
    void BaseClassMustBeDerivedFromService(const Service *service) {}
   public:
    WithStreamedUnaryMethod_img2vec() {
      ::grpc::Service::MarkMethodStreamed(0,
        new ::grpc::StreamedUnaryHandler< ::netVlad::NetRequest, ::netVlad::NetResponse>(std::bind(&WithStreamedUnaryMethod_img2vec<BaseClass>::Streamedimg2vec, this, std::placeholders::_1, std::placeholders::_2)));
    }
    ~WithStreamedUnaryMethod_img2vec() override {
      BaseClassMustBeDerivedFromService(this);
    }
    // disable regular version of this method
    ::grpc::Status img2vec(::grpc::ServerContext* context, const ::netVlad::NetRequest* request, ::netVlad::NetResponse* response) final override {
      abort();
      return ::grpc::Status(::grpc::StatusCode::UNIMPLEMENTED, "");
    }
    // replace default version of method with streamed unary
    virtual ::grpc::Status Streamedimg2vec(::grpc::ServerContext* context, ::grpc::ServerUnaryStreamer< ::netVlad::NetRequest,::netVlad::NetResponse>* server_unary_streamer) = 0;
  };
  typedef WithStreamedUnaryMethod_img2vec<Service > StreamedUnaryService;
  typedef Service SplitStreamedService;
  typedef WithStreamedUnaryMethod_img2vec<Service > StreamedService;
};

}  // namespace netVlad


#endif  // GRPC_netVlad_2eproto__INCLUDED
