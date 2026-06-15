# generated from rosidl_generator_py/resource/_idl.py.em
# with input from jetson_camera_interfaces:msg/ProcessedImagePair.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ProcessedImagePair(type):
    """Metaclass of message 'ProcessedImagePair'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('jetson_camera_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'jetson_camera_interfaces.msg.ProcessedImagePair')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__processed_image_pair
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__processed_image_pair
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__processed_image_pair
            cls._TYPE_SUPPORT = module.type_support_msg__msg__processed_image_pair
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__processed_image_pair

            from sensor_msgs.msg import CompressedImage
            if CompressedImage.__class__._TYPE_SUPPORT is None:
                CompressedImage.__class__.__import_type_support__()

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ProcessedImagePair(metaclass=Metaclass_ProcessedImagePair):
    """Message class 'ProcessedImagePair'."""

    __slots__ = [
        '_header',
        '_raw_image',
        '_undistorted_image',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'raw_image': 'sensor_msgs/CompressedImage',
        'undistorted_image': 'sensor_msgs/CompressedImage',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'CompressedImage'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'CompressedImage'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        from sensor_msgs.msg import CompressedImage
        self.raw_image = kwargs.get('raw_image', CompressedImage())
        from sensor_msgs.msg import CompressedImage
        self.undistorted_image = kwargs.get('undistorted_image', CompressedImage())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.header != other.header:
            return False
        if self.raw_image != other.raw_image:
            return False
        if self.undistorted_image != other.undistorted_image:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @property
    def raw_image(self):
        """Message field 'raw_image'."""
        return self._raw_image

    @raw_image.setter
    def raw_image(self, value):
        if __debug__:
            from sensor_msgs.msg import CompressedImage
            assert \
                isinstance(value, CompressedImage), \
                "The 'raw_image' field must be a sub message of type 'CompressedImage'"
        self._raw_image = value

    @property
    def undistorted_image(self):
        """Message field 'undistorted_image'."""
        return self._undistorted_image

    @undistorted_image.setter
    def undistorted_image(self, value):
        if __debug__:
            from sensor_msgs.msg import CompressedImage
            assert \
                isinstance(value, CompressedImage), \
                "The 'undistorted_image' field must be a sub message of type 'CompressedImage'"
        self._undistorted_image = value
