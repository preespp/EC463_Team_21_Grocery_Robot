# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_interfaces:msg/OrderItem.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OrderItem(type):
    """Metaclass of message 'OrderItem'."""

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
            module = import_type_support('robot_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_interfaces.msg.OrderItem')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__order_item
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__order_item
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__order_item
            cls._TYPE_SUPPORT = module.type_support_msg__msg__order_item
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__order_item

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OrderItem(metaclass=Metaclass_OrderItem):
    """Message class 'OrderItem'."""

    __slots__ = [
        '_product_id',
        '_name',
        '_aisle',
        '_rack',
        '_shelf_level',
        '_qty',
        '_price',
        '_stock',
    ]

    _fields_and_field_types = {
        'product_id': 'string',
        'name': 'string',
        'aisle': 'string',
        'rack': 'int32',
        'shelf_level': 'int32',
        'qty': 'int32',
        'price': 'float',
        'stock': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.product_id = kwargs.get('product_id', str())
        self.name = kwargs.get('name', str())
        self.aisle = kwargs.get('aisle', str())
        self.rack = kwargs.get('rack', int())
        self.shelf_level = kwargs.get('shelf_level', int())
        self.qty = kwargs.get('qty', int())
        self.price = kwargs.get('price', float())
        self.stock = kwargs.get('stock', int())

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
        if self.product_id != other.product_id:
            return False
        if self.name != other.name:
            return False
        if self.aisle != other.aisle:
            return False
        if self.rack != other.rack:
            return False
        if self.shelf_level != other.shelf_level:
            return False
        if self.qty != other.qty:
            return False
        if self.price != other.price:
            return False
        if self.stock != other.stock:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def product_id(self):
        """Message field 'product_id'."""
        return self._product_id

    @product_id.setter
    def product_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'product_id' field must be of type 'str'"
        self._product_id = value

    @builtins.property
    def name(self):
        """Message field 'name'."""
        return self._name

    @name.setter
    def name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'name' field must be of type 'str'"
        self._name = value

    @builtins.property
    def aisle(self):
        """Message field 'aisle'."""
        return self._aisle

    @aisle.setter
    def aisle(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'aisle' field must be of type 'str'"
        self._aisle = value

    @builtins.property
    def rack(self):
        """Message field 'rack'."""
        return self._rack

    @rack.setter
    def rack(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rack' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rack' field must be an integer in [-2147483648, 2147483647]"
        self._rack = value

    @builtins.property
    def shelf_level(self):
        """Message field 'shelf_level'."""
        return self._shelf_level

    @shelf_level.setter
    def shelf_level(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'shelf_level' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'shelf_level' field must be an integer in [-2147483648, 2147483647]"
        self._shelf_level = value

    @builtins.property
    def qty(self):
        """Message field 'qty'."""
        return self._qty

    @qty.setter
    def qty(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'qty' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'qty' field must be an integer in [-2147483648, 2147483647]"
        self._qty = value

    @builtins.property
    def price(self):
        """Message field 'price'."""
        return self._price

    @price.setter
    def price(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'price' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'price' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._price = value

    @builtins.property
    def stock(self):
        """Message field 'stock'."""
        return self._stock

    @stock.setter
    def stock(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'stock' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'stock' field must be an integer in [-2147483648, 2147483647]"
        self._stock = value
