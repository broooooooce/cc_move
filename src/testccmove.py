#!/usr/bin/env python

# Example of importing motion to enable function level movements 
# to cartesian coordinates.

# Bruce Stracener - University of Arkansas for Medical Sciences
# started 01/29/18


import motion           # contains ccmove function

#               x    y    z    ox    oy    oz     ow     speed    limb

motion.ccmove(1.1, .15, .65, 0.00, 0.70, 0.00, 0.700, 0.1500000, 'left')
motion.ccmove(1.1, .35, .65, 0.00, 0.70, 0.00, 0.700, 0.1500000, 'left')
motion.ccmove(1.1, .35, .45, 0.00, 0.70, 0.00, 0.700, 0.1500000, 'left')
motion.ccmove(1.1, .15, .45, 0.00, 0.70, 0.00, 0.700, 0.1500000, 'left')


#   the result is baxter draws a square in front of himself.
