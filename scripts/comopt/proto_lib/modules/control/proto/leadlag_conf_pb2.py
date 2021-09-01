# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/control/proto/leadlag_conf.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/control/proto/leadlag_conf.proto',
  package='apollo.control',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n(modules/control/proto/leadlag_conf.proto\x12\x0e\x61pollo.control\"l\n\x0bLeadlagConf\x12(\n\x1binnerstate_saturation_level\x18\x01 \x01(\x01:\x03\x33\x30\x30\x12\x12\n\x05\x61lpha\x18\x02 \x01(\x01:\x03\x30.1\x12\x0f\n\x04\x62\x65ta\x18\x03 \x01(\x01:\x01\x31\x12\x0e\n\x03tau\x18\x04 \x01(\x01:\x01\x30')
)




_LEADLAGCONF = _descriptor.Descriptor(
  name='LeadlagConf',
  full_name='apollo.control.LeadlagConf',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='innerstate_saturation_level', full_name='apollo.control.LeadlagConf.innerstate_saturation_level', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(300),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='alpha', full_name='apollo.control.LeadlagConf.alpha', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0.1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='beta', full_name='apollo.control.LeadlagConf.beta', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(1),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tau', full_name='apollo.control.LeadlagConf.tau', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
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
  serialized_start=60,
  serialized_end=168,
)

DESCRIPTOR.message_types_by_name['LeadlagConf'] = _LEADLAGCONF
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LeadlagConf = _reflection.GeneratedProtocolMessageType('LeadlagConf', (_message.Message,), dict(
  DESCRIPTOR = _LEADLAGCONF,
  __module__ = 'modules.control.proto.leadlag_conf_pb2'
  # @@protoc_insertion_point(class_scope:apollo.control.LeadlagConf)
  ))
_sym_db.RegisterMessage(LeadlagConf)


# @@protoc_insertion_point(module_scope)
