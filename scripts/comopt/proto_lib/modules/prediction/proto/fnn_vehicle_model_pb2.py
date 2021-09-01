# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: modules/prediction/proto/fnn_vehicle_model.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from modules.prediction.proto import fnn_model_base_pb2 as modules_dot_prediction_dot_proto_dot_fnn__model__base__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='modules/prediction/proto/fnn_vehicle_model.proto',
  package='apollo.prediction',
  syntax='proto2',
  serialized_options=None,
  serialized_pb=_b('\n0modules/prediction/proto/fnn_vehicle_model.proto\x12\x11\x61pollo.prediction\x1a-modules/prediction/proto/fnn_model_base.proto\"\xd5\x01\n\x0f\x46nnVehicleModel\x12\x11\n\tdim_input\x18\x01 \x01(\x05\x12/\n\x0csamples_mean\x18\x02 \x01(\x0b\x32\x19.apollo.prediction.Vector\x12.\n\x0bsamples_std\x18\x03 \x01(\x0b\x32\x19.apollo.prediction.Vector\x12\x11\n\tnum_layer\x18\x04 \x01(\x05\x12\'\n\x05layer\x18\x05 \x03(\x0b\x32\x18.apollo.prediction.Layer\x12\x12\n\ndim_output\x18\x06 \x01(\x05')
  ,
  dependencies=[modules_dot_prediction_dot_proto_dot_fnn__model__base__pb2.DESCRIPTOR,])




_FNNVEHICLEMODEL = _descriptor.Descriptor(
  name='FnnVehicleModel',
  full_name='apollo.prediction.FnnVehicleModel',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='dim_input', full_name='apollo.prediction.FnnVehicleModel.dim_input', index=0,
      number=1, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='samples_mean', full_name='apollo.prediction.FnnVehicleModel.samples_mean', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='samples_std', full_name='apollo.prediction.FnnVehicleModel.samples_std', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='num_layer', full_name='apollo.prediction.FnnVehicleModel.num_layer', index=3,
      number=4, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='layer', full_name='apollo.prediction.FnnVehicleModel.layer', index=4,
      number=5, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR),
    _descriptor.FieldDescriptor(
      name='dim_output', full_name='apollo.prediction.FnnVehicleModel.dim_output', index=5,
      number=6, type=5, cpp_type=1, label=1,
      has_default_value=False, default_value=0,
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
  serialized_start=119,
  serialized_end=332,
)

_FNNVEHICLEMODEL.fields_by_name['samples_mean'].message_type = modules_dot_prediction_dot_proto_dot_fnn__model__base__pb2._VECTOR
_FNNVEHICLEMODEL.fields_by_name['samples_std'].message_type = modules_dot_prediction_dot_proto_dot_fnn__model__base__pb2._VECTOR
_FNNVEHICLEMODEL.fields_by_name['layer'].message_type = modules_dot_prediction_dot_proto_dot_fnn__model__base__pb2._LAYER
DESCRIPTOR.message_types_by_name['FnnVehicleModel'] = _FNNVEHICLEMODEL
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

FnnVehicleModel = _reflection.GeneratedProtocolMessageType('FnnVehicleModel', (_message.Message,), dict(
  DESCRIPTOR = _FNNVEHICLEMODEL,
  __module__ = 'modules.prediction.proto.fnn_vehicle_model_pb2'
  # @@protoc_insertion_point(class_scope:apollo.prediction.FnnVehicleModel)
  ))
_sym_db.RegisterMessage(FnnVehicleModel)


# @@protoc_insertion_point(module_scope)
