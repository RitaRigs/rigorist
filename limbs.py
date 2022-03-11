# limbs.py
# Created: Sunday, 27th February 2022 8:46:50 pm
# Matthew Riche
# Last Modified: Sunday, 27th February 2022 8:58:10 pm
# Modified By: Matthew Riche

from . rmodule import *
from . placer import mirror_placer
from . import orient as ori

import pprint

class Limb(RMod):
    def __init__(self, name="C_Generic_RModule", dir_prefix='', mirror=True):
        '''
        A generic limb as a base, hinge, and end joint.  For an arm this will be shoulder, elbow,
        and wrist, and for a leg this will be hip, knee, and ankle.
        '''
        super().__init__(name=name, dir_prefix=dir_prefix, mirror=mirror)

        # The anatomy of these tuples is:
        #   [0] World Space Position
        #   [1] World Space Scale.
        #   [2] The name in the scene.
        #   [3] The colour (string that matches our colour dict.)
        #   [4] The key name.
        # The key name and the scene name being diffent allows for modules that inherit something 
        # generic like 'limb' and still reference placers correctly while displaying more accurate
        # names in the viewport.

        self.plan = {
            'base':{
                'pos':(0.0, 3.0, 4.0), 
                'name':'base',
                'placer':(1.0, 'orange'),
                'up_plc':{'pos':(0.0, 0.0, 7.0), 'size':0.4, 'colour':'white' },
                'aim':1,
                'up':0,
                'child':'hinge'
            },
            'hinge':{
                'pos':(0.0, 3.0, 0.0),
                'name':'hinge', 
                'placer':(1.0, 'orange'),
                'up_plc':{'pos':(7.0, 7.0, 7.0), 'size':0.4, 'colour':'white' },
                'aim':1,
                'up':0,
                'child':'end'
            },
            'end':{
                'pos':(0.0, 3.0, -4.0),
                'name':'end', 
                'placer':(1.0, 'orange'),
                'up_plc':{'pos':(7.0, 0.0, 0.0), 'size':0.4, 'colour':'white' },
                'aim':1,
                'up':0,
                'child':None
            }
        }

        # Get membership for the essential joints of a parent joint.
        self.base_joint = None
        self.hinge_joint = None
        self.end_joint = None

        return

    def build_module(self):
        '''
        Based upon placers in the scene, begin construction
        '''
        super().build_module()

        # Select clear to disallow any automatic parenting
        pm.select(cl=True)


class Arm(Limb):
    def __init__(self, name="C_Generic_RModule", dir_prefix=''):
        '''
        The least most complicated limb that is still acceptable in the rigging world--
        FK/IK switch, cleanly placed pole-vector, nothing else.
        '''
        super().__init__(name=name, dir_prefix=dir_prefix)

        self.plan['base']['pos'] = (20.0, 175.0, 0.0)
        self.plan['base']['name'] = 'shoulder'
        self.plan['hinge']['pos'] = (28.0, 145.0, 0.0)
        self.plan['hinge']['name'] = 'elbow'
        self.plan['end']['pos'] = (38.0, 115.0, 0.0)
        self.plan['end']['name'] = 'wrist'

        return

    def build_module(self):
        super().build_module()

        print("Arm Module built, as child of limb module.")


class Arms:
    def __init__(self, name=""):
        '''
        A pair of Arms.
        '''

        self._left_arm = Arm("L_arm")
        self._right_arm = Arm("R_arm")

        self._left_arm.build_placers()
        self._right_arm.build_placers()

        # Now we need to build expressions to make placers mirror on X.
        for key in self._left_arm.plan:
            mirror_placer(self._left_arm.plan[key]['placer_node'], 
            self._right_arm.plan[key]['placer_node'])
        
    def build(self):
        '''
        Build both arm modules.
        '''

        print("Building both arms...")
        self._left_arm.build_joints()
        self._right_arm.build_joints()

        return

class FKIKlimb(Limb):
    def __init__(self, name="FKIK_module", dir_prefix=''):
        '''
        creating FKIK joint chains with respective functions
        
        '''
        super().__init__(name=name, dir_prefix=dir_prefix)

    def build_module(self):
        
        super().build_module()

        shoulder = self.plan['base']['joint_node']
        wrist = self.plan['end']['joint_node']

        print("Nodes are: {} and {}".format(shoulder, wrist))

        pm.select(cl=True)
        
        self.ik=pm.ikHandle(sj=self.plan['base']['joint_node'] , ee=self.plan['end']['joint_node'] , sol='ikRPsolver', srp=True, see=True, s='sticky', jl=True, n=(self.side_prefix + self.name + 'IK'))
       

