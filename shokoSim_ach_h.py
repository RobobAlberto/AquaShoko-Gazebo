#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2017, Daniel M. Lofaro
# Copyright (c) 2017, Alberto Perez Nunez
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from ctypes import Structure,c_uint16,c_double,c_ubyte,c_uint32,c_int32,c_int16# */
import ach


# Ach channels
SHOKO_JOINT_COUNT         = 12
SHOKO_CHAN_NAME_REF       = 'shoko-ref'
SHOKO_CHAN_NAME_STATE     = 'shoko-state'

# Joint names
LSY = 0
LSP = 1
LEP = 2
RSY = 3
RSP = 4
REP = 5
LHY = 6
LHP = 7
LKP = 8
RHY = 9
RHP = 10
RKP = 11

SHOKO_CHAR_PARAM_BUFFER_SIZE = 30

# Modes
SHOKO_REF_MODE_FILTER   = 0 # reference to reference filter



# Make Reference Structure
class SHOKO_JOINT_REF(Structure):
  _pack_   = 1
  _fields_ = [
              ("ref"  , c_double),
              ("mode" , c_int16)]


class SHOKO_REF(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint" , SHOKO_JOINT_REF*SHOKO_JOINT_COUNT)]




class SHOKO_JOINT_STATE(Structure):
  _pack_   = 1
  _fields_ = [
              ("pos"    , c_double), # actuial position (rad)
              ("ref"    , c_double), # last commanded reference value (rad)
              ("vel"    , c_double), # angular velocity (rad/sec)
              ("torque" , c_double), # torque on motor (A)
              ("temp"   , c_double), # Temperature (C)
              ("voltage", c_double), # System voltage (V) 
              ("name"   , c_ubyte*SHOKO_CHAR_PARAM_BUFFER_SIZE)] # Space for human readiable name

class SHOKO_POWER_MIN_MAX(Structure):
  _pack_ = 1
  _fields_ = [
              ("max", c_double),
              ("min", c_double),
              ("ave", c_double)]

class SHOKO_POWER(Structure):
  _pack_ = 1
  _fields_ = [("voltage", SHOKO_POWER_MIN_MAX),
              ("current", SHOKO_POWER_MIN_MAX),
              ("temp"   , c_double), 
              ("power"  , c_double)]


class SHOKO_STATE(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint"   , SHOKO_JOINT_STATE*SHOKO_JOINT_COUNT),
              ("power"   , SHOKO_POWER),
              ("time"   , c_double)]




class SHOKO_JOINT_PARAM(Structure):
  _pack_   = 1
  _fields_ = [
              ("id"       , c_int16),  # ID of the servo
              ("ticks"    , c_int16),  # encoder ticks per rev
              ("offset"   , c_double), # offset from center (rad)
              ("dir"      , c_double), # direction +-1 
              ("toque"    , c_double), # torque conversion (A / unit)
              ("theta_max", c_double), # max theta (rad)
              ("theta_min", c_double)] # min theta (rad)

class SHOKO_PARAM(Structure):
  _pack_   = 1
  _fields_ = [
              ("joint", SHOKO_JOINT_PARAM*SHOKO_JOINT_COUNT),
              ("baud"     , c_int32),  # baud rate
              ("com"      , c_int16)] # COM port with dir (/dev..)

