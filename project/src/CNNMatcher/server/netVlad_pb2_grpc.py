# Generated by the gRPC Python protocol compiler plugin. DO NOT EDIT!
import grpc

import netVlad_pb2 as netVlad__pb2


class NetConnectStub(object):
  """The greeting service definition.
  """

  def __init__(self, channel):
    """Constructor.

    Args:
      channel: A grpc.Channel.
    """
    self.img2vec = channel.unary_unary(
        '/netVlad.NetConnect/img2vec',
        request_serializer=netVlad__pb2.NetRequest.SerializeToString,
        response_deserializer=netVlad__pb2.NetResponse.FromString,
        )


class NetConnectServicer(object):
  """The greeting service definition.
  """

  def img2vec(self, request, context):
    """Sends a greeting
    """
    context.set_code(grpc.StatusCode.UNIMPLEMENTED)
    context.set_details('Method not implemented!')
    raise NotImplementedError('Method not implemented!')


def add_NetConnectServicer_to_server(servicer, server):
  rpc_method_handlers = {
      'img2vec': grpc.unary_unary_rpc_method_handler(
          servicer.img2vec,
          request_deserializer=netVlad__pb2.NetRequest.FromString,
          response_serializer=netVlad__pb2.NetResponse.SerializeToString,
      ),
  }
  generic_handler = grpc.method_handlers_generic_handler(
      'netVlad.NetConnect', rpc_method_handlers)
  server.add_generic_rpc_handlers((generic_handler,))
