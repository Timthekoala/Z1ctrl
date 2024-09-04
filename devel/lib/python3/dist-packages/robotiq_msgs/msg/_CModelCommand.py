# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from robotiq_msgs/CModelCommand.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class CModelCommand(genpy.Message):
  _md5sum = "481503a99d995d0e403b7ee708c6862c"
  _type = "robotiq_msgs/CModelCommand"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """uint8 rACT
uint8 rGTO
uint8 rATR
uint8 rPR
uint8 rSP
uint8 rFR

"""
  __slots__ = ['rACT','rGTO','rATR','rPR','rSP','rFR']
  _slot_types = ['uint8','uint8','uint8','uint8','uint8','uint8']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       rACT,rGTO,rATR,rPR,rSP,rFR

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CModelCommand, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.rACT is None:
        self.rACT = 0
      if self.rGTO is None:
        self.rGTO = 0
      if self.rATR is None:
        self.rATR = 0
      if self.rPR is None:
        self.rPR = 0
      if self.rSP is None:
        self.rSP = 0
      if self.rFR is None:
        self.rFR = 0
    else:
      self.rACT = 0
      self.rGTO = 0
      self.rATR = 0
      self.rPR = 0
      self.rSP = 0
      self.rFR = 0

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_get_struct_6B().pack(_x.rACT, _x.rGTO, _x.rATR, _x.rPR, _x.rSP, _x.rFR))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 6
      (_x.rACT, _x.rGTO, _x.rATR, _x.rPR, _x.rSP, _x.rFR,) = _get_struct_6B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_6B().pack(_x.rACT, _x.rGTO, _x.rATR, _x.rPR, _x.rSP, _x.rFR))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      end = 0
      _x = self
      start = end
      end += 6
      (_x.rACT, _x.rGTO, _x.rATR, _x.rPR, _x.rSP, _x.rFR,) = _get_struct_6B().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_6B = None
def _get_struct_6B():
    global _struct_6B
    if _struct_6B is None:
        _struct_6B = struct.Struct("<6B")
    return _struct_6B
