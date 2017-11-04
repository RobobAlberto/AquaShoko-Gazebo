import shokoSim_ach_h as shoko
import sys
import numpy as np
import time
import os
import yaml
import time
import ach


ref     = shoko.SHOKO_REF()
state   = shoko.SHOKO_STATE()
param   = shoko.SHOKO_PARAM() 

robot   = None

ref_chan = ach.Channel(shoko.SHOKO_CHAN_NAME_REF)
state_chan = ach.Channel(shoko.SHOKO_CHAN_NAME_STATE)

ref_chan.flush()
state_chan.flush()


def init(com = None, baud=None):
  global ref, state, param
  com = None
  if(com == None):
    com = 0

  if(baud == None):
    baud = 1000000

  # setting up setting (but not using a setup file)
  for i in range(shoko.SHOKO_JOINT_COUNT):
    param.joint[i].ticks     = 4096
    param.joint[i].offset    = 0.0
    param.joint[i].dir       = 1.0
    param.joint[i].torque    = 0.0045  # 4.5 mA per unit
    param.joint[i].theta_max =  np.pi/2.0 # max theta in rad
    param.joint[i].theta_min = -np.pi/2.0 # min theta in rad
    param.baud               = baud # baud rate
    param.com                = com  # com port     

  param.joint[shoko.RSY].id = 11
  param.joint[shoko.RSP].id = 12
  param.joint[shoko.REP].id = 13
  
  param.joint[shoko.LSY].id = 21
  param.joint[shoko.LSP].id = 22
  param.joint[shoko.LEP].id = 23

  param.joint[shoko.RHY].id = 41
  param.joint[shoko.RHP].id = 42
  param.joint[shoko.RKP].id = 43


  param.joint[shoko.LHY].id = 31
  param.joint[shoko.LHP].id = 32
  param.joint[shoko.LKP].id = 33

