# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: map_lane.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import map_id_pb2 as map__id__pb2
import map_geometry_pb2 as map__geometry__pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='map_lane.proto',
  package='apollo.hdmap',
  syntax='proto2',
  serialized_options=None,
  create_key=_descriptor._internal_create_key,
  serialized_pb=b'\n\x0emap_lane.proto\x12\x0c\x61pollo.hdmap\x1a\x0cmap_id.proto\x1a\x12map_geometry.proto\"\xcb\x01\n\x10LaneBoundaryType\x12\t\n\x01s\x18\x01 \x01(\x01\x12\x32\n\x05types\x18\x02 \x03(\x0e\x32#.apollo.hdmap.LaneBoundaryType.Type\"x\n\x04Type\x12\x0b\n\x07UNKNOWN\x10\x00\x12\x11\n\rDOTTED_YELLOW\x10\x01\x12\x10\n\x0c\x44OTTED_WHITE\x10\x02\x12\x10\n\x0cSOLID_YELLOW\x10\x03\x12\x0f\n\x0bSOLID_WHITE\x10\x04\x12\x11\n\rDOUBLE_YELLOW\x10\x05\x12\x08\n\x04\x43URB\x10\x06\"\x8a\x01\n\x0cLaneBoundary\x12\"\n\x05\x63urve\x18\x01 \x01(\x0b\x32\x13.apollo.hdmap.Curve\x12\x0e\n\x06length\x18\x02 \x01(\x01\x12\x0f\n\x07virtual\x18\x03 \x01(\x08\x12\x35\n\rboundary_type\x18\x04 \x03(\x0b\x32\x1e.apollo.hdmap.LaneBoundaryType\"1\n\x15LaneSampleAssociation\x12\t\n\x01s\x18\x01 \x01(\x01\x12\r\n\x05width\x18\x02 \x01(\x01\"\xee\t\n\x04Lane\x12\x1c\n\x02id\x18\x01 \x01(\x0b\x32\x10.apollo.hdmap.Id\x12*\n\rcentral_curve\x18\x02 \x01(\x0b\x32\x13.apollo.hdmap.Curve\x12\x31\n\rleft_boundary\x18\x03 \x01(\x0b\x32\x1a.apollo.hdmap.LaneBoundary\x12\x32\n\x0eright_boundary\x18\x04 \x01(\x0b\x32\x1a.apollo.hdmap.LaneBoundary\x12\x0e\n\x06length\x18\x05 \x01(\x01\x12\x13\n\x0bspeed_limit\x18\x06 \x01(\x01\x12$\n\noverlap_id\x18\x07 \x03(\x0b\x32\x10.apollo.hdmap.Id\x12(\n\x0epredecessor_id\x18\x08 \x03(\x0b\x32\x10.apollo.hdmap.Id\x12&\n\x0csuccessor_id\x18\t \x03(\x0b\x32\x10.apollo.hdmap.Id\x12\x37\n\x1dleft_neighbor_forward_lane_id\x18\n \x03(\x0b\x32\x10.apollo.hdmap.Id\x12\x38\n\x1eright_neighbor_forward_lane_id\x18\x0b \x03(\x0b\x32\x10.apollo.hdmap.Id\x12)\n\x04type\x18\x0c \x01(\x0e\x32\x1b.apollo.hdmap.Lane.LaneType\x12)\n\x04turn\x18\r \x01(\x0e\x32\x1b.apollo.hdmap.Lane.LaneTurn\x12\x37\n\x1dleft_neighbor_reverse_lane_id\x18\x0e \x03(\x0b\x32\x10.apollo.hdmap.Id\x12\x38\n\x1eright_neighbor_reverse_lane_id\x18\x0f \x03(\x0b\x32\x10.apollo.hdmap.Id\x12%\n\x0bjunction_id\x18\x10 \x01(\x0b\x32\x10.apollo.hdmap.Id\x12\x38\n\x0bleft_sample\x18\x11 \x03(\x0b\x32#.apollo.hdmap.LaneSampleAssociation\x12\x39\n\x0cright_sample\x18\x12 \x03(\x0b\x32#.apollo.hdmap.LaneSampleAssociation\x12\x33\n\tdirection\x18\x13 \x01(\x0e\x32 .apollo.hdmap.Lane.LaneDirection\x12=\n\x10left_road_sample\x18\x14 \x03(\x0b\x32#.apollo.hdmap.LaneSampleAssociation\x12>\n\x11right_road_sample\x18\x15 \x03(\x0b\x32#.apollo.hdmap.LaneSampleAssociation\x12.\n\x14self_reverse_lane_id\x18\x16 \x03(\x0b\x32\x10.apollo.hdmap.Id\"[\n\x08LaneType\x12\x08\n\x04NONE\x10\x01\x12\x10\n\x0c\x43ITY_DRIVING\x10\x02\x12\n\n\x06\x42IKING\x10\x03\x12\x0c\n\x08SIDEWALK\x10\x04\x12\x0b\n\x07PARKING\x10\x05\x12\x0c\n\x08SHOULDER\x10\x06\"B\n\x08LaneTurn\x12\x0b\n\x07NO_TURN\x10\x01\x12\r\n\tLEFT_TURN\x10\x02\x12\x0e\n\nRIGHT_TURN\x10\x03\x12\n\n\x06U_TURN\x10\x04\";\n\rLaneDirection\x12\x0b\n\x07\x46ORWARD\x10\x01\x12\x0c\n\x08\x42\x41\x43KWARD\x10\x02\x12\x0f\n\x0b\x42IDIRECTION\x10\x03'
  ,
  dependencies=[map__id__pb2.DESCRIPTOR,map__geometry__pb2.DESCRIPTOR,])



_LANEBOUNDARYTYPE_TYPE = _descriptor.EnumDescriptor(
  name='Type',
  full_name='apollo.hdmap.LaneBoundaryType.Type',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='UNKNOWN', index=0, number=0,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DOTTED_YELLOW', index=1, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DOTTED_WHITE', index=2, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SOLID_YELLOW', index=3, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SOLID_WHITE', index=4, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='DOUBLE_YELLOW', index=5, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CURB', index=6, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=150,
  serialized_end=270,
)
_sym_db.RegisterEnumDescriptor(_LANEBOUNDARYTYPE_TYPE)

_LANE_LANETYPE = _descriptor.EnumDescriptor(
  name='LaneType',
  full_name='apollo.hdmap.Lane.LaneType',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='CITY_DRIVING', index=1, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BIKING', index=2, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SIDEWALK', index=3, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='PARKING', index=4, number=5,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='SHOULDER', index=5, number=6,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1507,
  serialized_end=1598,
)
_sym_db.RegisterEnumDescriptor(_LANE_LANETYPE)

_LANE_LANETURN = _descriptor.EnumDescriptor(
  name='LaneTurn',
  full_name='apollo.hdmap.Lane.LaneTurn',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NO_TURN', index=0, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='LEFT_TURN', index=1, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='RIGHT_TURN', index=2, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='U_TURN', index=3, number=4,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1600,
  serialized_end=1666,
)
_sym_db.RegisterEnumDescriptor(_LANE_LANETURN)

_LANE_LANEDIRECTION = _descriptor.EnumDescriptor(
  name='LaneDirection',
  full_name='apollo.hdmap.Lane.LaneDirection',
  filename=None,
  file=DESCRIPTOR,
  create_key=_descriptor._internal_create_key,
  values=[
    _descriptor.EnumValueDescriptor(
      name='FORWARD', index=0, number=1,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BACKWARD', index=1, number=2,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
    _descriptor.EnumValueDescriptor(
      name='BIDIRECTION', index=2, number=3,
      serialized_options=None,
      type=None,
      create_key=_descriptor._internal_create_key),
  ],
  containing_type=None,
  serialized_options=None,
  serialized_start=1668,
  serialized_end=1727,
)
_sym_db.RegisterEnumDescriptor(_LANE_LANEDIRECTION)


_LANEBOUNDARYTYPE = _descriptor.Descriptor(
  name='LaneBoundaryType',
  full_name='apollo.hdmap.LaneBoundaryType',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='s', full_name='apollo.hdmap.LaneBoundaryType.s', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='types', full_name='apollo.hdmap.LaneBoundaryType.types', index=1,
      number=2, type=14, cpp_type=8, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _LANEBOUNDARYTYPE_TYPE,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=67,
  serialized_end=270,
)


_LANEBOUNDARY = _descriptor.Descriptor(
  name='LaneBoundary',
  full_name='apollo.hdmap.LaneBoundary',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='curve', full_name='apollo.hdmap.LaneBoundary.curve', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='length', full_name='apollo.hdmap.LaneBoundary.length', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='virtual', full_name='apollo.hdmap.LaneBoundary.virtual', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='boundary_type', full_name='apollo.hdmap.LaneBoundary.boundary_type', index=3,
      number=4, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=273,
  serialized_end=411,
)


_LANESAMPLEASSOCIATION = _descriptor.Descriptor(
  name='LaneSampleAssociation',
  full_name='apollo.hdmap.LaneSampleAssociation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='s', full_name='apollo.hdmap.LaneSampleAssociation.s', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='width', full_name='apollo.hdmap.LaneSampleAssociation.width', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
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
  serialized_start=413,
  serialized_end=462,
)


_LANE = _descriptor.Descriptor(
  name='Lane',
  full_name='apollo.hdmap.Lane',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  create_key=_descriptor._internal_create_key,
  fields=[
    _descriptor.FieldDescriptor(
      name='id', full_name='apollo.hdmap.Lane.id', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='central_curve', full_name='apollo.hdmap.Lane.central_curve', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='left_boundary', full_name='apollo.hdmap.Lane.left_boundary', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='right_boundary', full_name='apollo.hdmap.Lane.right_boundary', index=3,
      number=4, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='length', full_name='apollo.hdmap.Lane.length', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='speed_limit', full_name='apollo.hdmap.Lane.speed_limit', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='overlap_id', full_name='apollo.hdmap.Lane.overlap_id', index=6,
      number=7, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='predecessor_id', full_name='apollo.hdmap.Lane.predecessor_id', index=7,
      number=8, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='successor_id', full_name='apollo.hdmap.Lane.successor_id', index=8,
      number=9, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='left_neighbor_forward_lane_id', full_name='apollo.hdmap.Lane.left_neighbor_forward_lane_id', index=9,
      number=10, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='right_neighbor_forward_lane_id', full_name='apollo.hdmap.Lane.right_neighbor_forward_lane_id', index=10,
      number=11, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='type', full_name='apollo.hdmap.Lane.type', index=11,
      number=12, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='turn', full_name='apollo.hdmap.Lane.turn', index=12,
      number=13, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='left_neighbor_reverse_lane_id', full_name='apollo.hdmap.Lane.left_neighbor_reverse_lane_id', index=13,
      number=14, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='right_neighbor_reverse_lane_id', full_name='apollo.hdmap.Lane.right_neighbor_reverse_lane_id', index=14,
      number=15, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='junction_id', full_name='apollo.hdmap.Lane.junction_id', index=15,
      number=16, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='left_sample', full_name='apollo.hdmap.Lane.left_sample', index=16,
      number=17, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='right_sample', full_name='apollo.hdmap.Lane.right_sample', index=17,
      number=18, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='direction', full_name='apollo.hdmap.Lane.direction', index=18,
      number=19, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='left_road_sample', full_name='apollo.hdmap.Lane.left_road_sample', index=19,
      number=20, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='right_road_sample', full_name='apollo.hdmap.Lane.right_road_sample', index=20,
      number=21, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
    _descriptor.FieldDescriptor(
      name='self_reverse_lane_id', full_name='apollo.hdmap.Lane.self_reverse_lane_id', index=21,
      number=22, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      serialized_options=None, file=DESCRIPTOR,  create_key=_descriptor._internal_create_key),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _LANE_LANETYPE,
    _LANE_LANETURN,
    _LANE_LANEDIRECTION,
  ],
  serialized_options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=465,
  serialized_end=1727,
)

_LANEBOUNDARYTYPE.fields_by_name['types'].enum_type = _LANEBOUNDARYTYPE_TYPE
_LANEBOUNDARYTYPE_TYPE.containing_type = _LANEBOUNDARYTYPE
_LANEBOUNDARY.fields_by_name['curve'].message_type = map__geometry__pb2._CURVE
_LANEBOUNDARY.fields_by_name['boundary_type'].message_type = _LANEBOUNDARYTYPE
_LANE.fields_by_name['id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['central_curve'].message_type = map__geometry__pb2._CURVE
_LANE.fields_by_name['left_boundary'].message_type = _LANEBOUNDARY
_LANE.fields_by_name['right_boundary'].message_type = _LANEBOUNDARY
_LANE.fields_by_name['overlap_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['predecessor_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['successor_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['left_neighbor_forward_lane_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['right_neighbor_forward_lane_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['type'].enum_type = _LANE_LANETYPE
_LANE.fields_by_name['turn'].enum_type = _LANE_LANETURN
_LANE.fields_by_name['left_neighbor_reverse_lane_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['right_neighbor_reverse_lane_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['junction_id'].message_type = map__id__pb2._ID
_LANE.fields_by_name['left_sample'].message_type = _LANESAMPLEASSOCIATION
_LANE.fields_by_name['right_sample'].message_type = _LANESAMPLEASSOCIATION
_LANE.fields_by_name['direction'].enum_type = _LANE_LANEDIRECTION
_LANE.fields_by_name['left_road_sample'].message_type = _LANESAMPLEASSOCIATION
_LANE.fields_by_name['right_road_sample'].message_type = _LANESAMPLEASSOCIATION
_LANE.fields_by_name['self_reverse_lane_id'].message_type = map__id__pb2._ID
_LANE_LANETYPE.containing_type = _LANE
_LANE_LANETURN.containing_type = _LANE
_LANE_LANEDIRECTION.containing_type = _LANE
DESCRIPTOR.message_types_by_name['LaneBoundaryType'] = _LANEBOUNDARYTYPE
DESCRIPTOR.message_types_by_name['LaneBoundary'] = _LANEBOUNDARY
DESCRIPTOR.message_types_by_name['LaneSampleAssociation'] = _LANESAMPLEASSOCIATION
DESCRIPTOR.message_types_by_name['Lane'] = _LANE
_sym_db.RegisterFileDescriptor(DESCRIPTOR)

LaneBoundaryType = _reflection.GeneratedProtocolMessageType('LaneBoundaryType', (_message.Message,), {
  'DESCRIPTOR' : _LANEBOUNDARYTYPE,
  '__module__' : 'map_lane_pb2'
  # @@protoc_insertion_point(class_scope:apollo.hdmap.LaneBoundaryType)
  })
_sym_db.RegisterMessage(LaneBoundaryType)

LaneBoundary = _reflection.GeneratedProtocolMessageType('LaneBoundary', (_message.Message,), {
  'DESCRIPTOR' : _LANEBOUNDARY,
  '__module__' : 'map_lane_pb2'
  # @@protoc_insertion_point(class_scope:apollo.hdmap.LaneBoundary)
  })
_sym_db.RegisterMessage(LaneBoundary)

LaneSampleAssociation = _reflection.GeneratedProtocolMessageType('LaneSampleAssociation', (_message.Message,), {
  'DESCRIPTOR' : _LANESAMPLEASSOCIATION,
  '__module__' : 'map_lane_pb2'
  # @@protoc_insertion_point(class_scope:apollo.hdmap.LaneSampleAssociation)
  })
_sym_db.RegisterMessage(LaneSampleAssociation)

Lane = _reflection.GeneratedProtocolMessageType('Lane', (_message.Message,), {
  'DESCRIPTOR' : _LANE,
  '__module__' : 'map_lane_pb2'
  # @@protoc_insertion_point(class_scope:apollo.hdmap.Lane)
  })
_sym_db.RegisterMessage(Lane)


# @@protoc_insertion_point(module_scope)
