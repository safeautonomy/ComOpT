# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/perception/camera/common/proto/object_template_meta_schema.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/perception/camera/common/proto/object_template_meta_schema.proto',
  package='apollo.perception.camera',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\nHmodules/perception/camera/common/proto/object_template_meta_schema.proto\x12\x18\x61pollo.perception.camera\"&\n\x03\x44im\x12\t\n\x01H\x18\x01 \x01(\x02\x12\t\n\x01W\x18\x02 \x01(\x02\x12\t\n\x01L\x18\x03 \x01(\x02\"Q\n\x0eObjectTemplate\x12\x13\n\x0bspeed_limit\x18\x01 \x01(\x02\x12*\n\x03\x64im\x18\x02 \x03(\x0b\x32\x1d.apollo.perception.camera.Dim\"\x8e\x06\n\x12ObjectTemplateMeta\x12\x39\n\x07unknown\x18\x01 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x41\n\x0funknown_movable\x18\x02 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x43\n\x11unknown_unmovable\x18\x03 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x35\n\x03\x63\x61r\x18\x04 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x35\n\x03van\x18\x05 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x37\n\x05truck\x18\x06 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x35\n\x03\x62us\x18\x07 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12\x39\n\x07\x63yclist\x18\x08 \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12>\n\x0cmotorcyclist\x18\t \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12<\n\ntricyclist\x18\n \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12<\n\npedestrian\x18\x0b \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12=\n\x0btrafficcone\x18\x0c \x01(\x0b\x32(.apollo.perception.camera.ObjectTemplate\x12!\n\x14max_dim_change_ratio\x18\x15 \x01(\x02:\x03\x30.1')
)




_DIM = _descriptor.Descriptor(
  name='Dim',
  full_name='apollo.perception.camera.Dim',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='H', full_name='apollo.perception.camera.Dim.H', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='W', full_name='apollo.perception.camera.Dim.W', index=1,
      number=2, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='L', full_name='apollo.perception.camera.Dim.L', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
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
  serialized_start=102,
  serialized_end=140,
)


_OBJECTTEMPLATE = _descriptor.Descriptor(
  name='ObjectTemplate',
  full_name='apollo.perception.camera.ObjectTemplate',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='speed_limit', full_name='apollo.perception.camera.ObjectTemplate.speed_limit', index=0,
      number=1, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dim', full_name='apollo.perception.camera.ObjectTemplate.dim', index=1,
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
  serialized_start=142,
  serialized_end=223,
)


_OBJECTTEMPLATEMETA = _descriptor.Descriptor(
  name='ObjectTemplateMeta',
  full_name='apollo.perception.camera.ObjectTemplateMeta',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='unknown', full_name='apollo.perception.camera.ObjectTemplateMeta.unknown', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='unknown_movable', full_name='apollo.perception.camera.ObjectTemplateMeta.unknown_movable', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='unknown_unmovable', full_name='apollo.perception.camera.ObjectTemplateMeta.unknown_unmovable', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='car', full_name='apollo.perception.camera.ObjectTemplateMeta.car', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='van', full_name='apollo.perception.camera.ObjectTemplateMeta.van', index=4,
      number=5, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='truck', full_name='apollo.perception.camera.ObjectTemplateMeta.truck', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='bus', full_name='apollo.perception.camera.ObjectTemplateMeta.bus', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='cyclist', full_name='apollo.perception.camera.ObjectTemplateMeta.cyclist', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='motorcyclist', full_name='apollo.perception.camera.ObjectTemplateMeta.motorcyclist', index=8,
      number=9, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='tricyclist', full_name='apollo.perception.camera.ObjectTemplateMeta.tricyclist', index=9,
      number=10, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='pedestrian', full_name='apollo.perception.camera.ObjectTemplateMeta.pedestrian', index=10,
      number=11, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='trafficcone', full_name='apollo.perception.camera.ObjectTemplateMeta.trafficcone', index=11,
      number=12, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='max_dim_change_ratio', full_name='apollo.perception.camera.ObjectTemplateMeta.max_dim_change_ratio', index=12,
      number=21, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=float(0.1),
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
  serialized_start=226,
  serialized_end=1008,
)

_OBJECTTEMPLATE.fields_by_name['dim'].message_type = _DIM
_OBJECTTEMPLATEMETA.fields_by_name['unknown'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['unknown_movable'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['unknown_unmovable'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['car'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['van'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['truck'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['bus'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['cyclist'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['motorcyclist'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['tricyclist'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['pedestrian'].message_type = _OBJECTTEMPLATE
_OBJECTTEMPLATEMETA.fields_by_name['trafficcone'].message_type = _OBJECTTEMPLATE
DESCRIPTOR.message_types_by_name['Dim'] = _DIM
DESCRIPTOR.message_types_by_name['ObjectTemplate'] = _OBJECTTEMPLATE
DESCRIPTOR.message_types_by_name['ObjectTemplateMeta'] = _OBJECTTEMPLATEMETA
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

Dim = _reflection.GeneratedProtocolMessageType('Dim', (_message.Message,), dict(
  DESCRIPTOR = _DIM,
  __module__ = 'modules.perception.camera.common.proto.object_template_meta_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.Dim)
  ))
_sym_db.RegisterMessage(Dim)

ObjectTemplate = _reflection.GeneratedProtocolMessageType('ObjectTemplate', (_message.Message,), dict(
  DESCRIPTOR = _OBJECTTEMPLATE,
  __module__ = 'modules.perception.camera.common.proto.object_template_meta_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.ObjectTemplate)
  ))
_sym_db.RegisterMessage(ObjectTemplate)

ObjectTemplateMeta = _reflection.GeneratedProtocolMessageType('ObjectTemplateMeta', (_message.Message,), dict(
  DESCRIPTOR = _OBJECTTEMPLATEMETA,
  __module__ = 'modules.perception.camera.common.proto.object_template_meta_schema_pb2'
  # @@protoc_insertion_point(class_scope:apollo.perception.camera.ObjectTemplateMeta)
  ))
_sym_db.RegisterMessage(ObjectTemplateMeta)


# @@protoc_insertion_point(module_scope)
