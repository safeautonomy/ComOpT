# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/routing/proto/default_routing.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.common.proto import geometry_pb2 as modules_dot_common_dot_proto_dot_geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/routing/proto/default_routing.proto',
  package='apollo.routing',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n+modules/routing/proto/default_routing.proto\x12\x0e\x61pollo.routing\x1a#modules/common/proto/geometry.proto\"F\n\x0e\x44\x65\x66\x61ultRouting\x12\x0c\n\x04name\x18\x01 \x01(\t\x12&\n\x05point\x18\x02 \x03(\x0b\x32\x17.apollo.common.PointENU\"I\n\x0f\x44\x65\x66\x61ultRoutings\x12\x36\n\x0e\x64\x65\x66\x61ultrouting\x18\x01 \x03(\x0b\x32\x1e.apollo.routing.DefaultRouting')
  ,
  dependencies=[modules_dot_common_dot_proto_dot_geometry__pb2.DESCRIPTOR,])




_DEFAULTROUTING = _descriptor.Descriptor(
  name='DefaultRouting',
  full_name='apollo.routing.DefaultRouting',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='apollo.routing.DefaultRouting.name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='point', full_name='apollo.routing.DefaultRouting.point', index=1,
      number=2, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=100,
  serialized_end=170,
)


_DEFAULTROUTINGS = _descriptor.Descriptor(
  name='DefaultRoutings',
  full_name='apollo.routing.DefaultRoutings',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='defaultrouting', full_name='apollo.routing.DefaultRoutings.defaultrouting', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=172,
  serialized_end=245,
)

_DEFAULTROUTING.fields_by_name['point'].message_type = modules_dot_common_dot_proto_dot_geometry__pb2._POINTENU
_DEFAULTROUTINGS.fields_by_name['defaultrouting'].message_type = _DEFAULTROUTING
DESCRIPTOR.message_types_by_name['DefaultRouting'] = _DEFAULTROUTING
DESCRIPTOR.message_types_by_name['DefaultRoutings'] = _DEFAULTROUTINGS
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

DefaultRouting = _reflection.GeneratedProtocolMessageType('DefaultRouting', (_message.Message,), dict(
  DESCRIPTOR = _DEFAULTROUTING,
  __module__ = 'modules.routing.proto.default_routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.DefaultRouting)
  ))
_sym_db.RegisterMessage(DefaultRouting)

DefaultRoutings = _reflection.GeneratedProtocolMessageType('DefaultRoutings', (_message.Message,), dict(
  DESCRIPTOR = _DEFAULTROUTINGS,
  __module__ = 'modules.routing.proto.default_routing_pb2'
  # @@protoc_insertion_point(class_scope:apollo.routing.DefaultRoutings)
  ))
_sym_db.RegisterMessage(DefaultRoutings)


# @@protoc_insertion_point(module_scope)
