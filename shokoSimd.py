import shokoSim_ach as shoko
import numpy as np
import time


if __name__ == '__main__':
  shoko.init()
  
  T = 0.01
  end = False
  ii = 0
  while(not end):
    tick = time.time()
    # Get new Ach Ref message
    shoko.getRefData()

    # Set References 
    for i in range(shoko.shoko.SHOKO_JOINT_COUNT):  shoko.setRef(i,shoko.ref.joint[i].ref)

    # Send References
    shoko.syncRef()

    # Get and Update Enc values
    for i in range(shoko.shoko.SHOKO_JOINT_COUNT):
      enc = shoko.getEnc(i)
      if(None == enc):
        enc = -0.0
      shoko.state.joint[i].pos = enc

    # Update wall clock
    shoko.state.time = time.time()
    
    tock = time.time()


    # Put new Ach State message
    shoko.setStateData()

    if( ii > 10):
      print "dt = ", (tock - tick)
      ii = 0
    else:
      ii += 1
    time.sleep(T)

