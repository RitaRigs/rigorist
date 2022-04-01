# limbs.py
# Created: Sunday, 27th February 2022 8:46:50 pm
# Matthew Riche
# Last Modified: Sunday, 27th February 2022 8:58:10 pm
# Modified By: Matthew Riche

from . rmodule import *
from . placer import mirror_placer
from . import orient as ori
import  pymel.core.datatypes as dt
from . import controls as ctrl

import pprint

class Limb(RMod):
    def __init__(self, name="C_Generic_Limb", dir_prefix='', mirror=True):
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
                'aim':0,
                'up':1,
                'child':'hinge'
            },
            'hinge':{
                'pos':(0.0, 3.0, 0.0),
                'name':'hinge', 
                'placer':(1.0, 'orange'),
                'up_plc':{'pos':(7.0, 7.0, 7.0), 'size':0.4, 'colour':'white' },
                'aim':0,
                'up':1,
                'child':'end'
            },
            'end':{
                'pos':(0.0, 3.0, -4.0),
                'name':'end', 
                'placer':(1.0, 'orange'),
                'up_plc':{'pos':(7.0, 0.0, 0.0), 'size':0.4, 'colour':'white' },
                'aim':0,
                'up':1,
                'child':None
            }
        }

        return

    def build_module(self):
        '''
        Based upon placers in the scene, begin construction
        '''
        super().build_module()

        #creating the FK and IK joint chains
        self.fk_base_jnt = pm.duplicate(self.plan['base']['joint_node'])[0]
        self.fk_hinge_jnt = pm.listRelatives(self.fk_base_jnt, c=True)[0]
        self.fk_end_jnt = pm.listRelatives(self.fk_hinge_jnt, c=True)[0]

        self.ik_base_jnt = pm.duplicate(self.plan['base']['joint_node'])[0]
        self.ik_hinge_jnt = pm.listRelatives(self.ik_base_jnt, c=True)[0]
        self.ik_end_jnt = pm.listRelatives(self.ik_hinge_jnt, c=True)[0]

        #building FK controls and functions

        self.base_fk_control= ctrl.create_control(load_shape ='ring')
        pm.matchTransform(self.base_fk_control, self.fk_base_jnt)
        self.base_fk_null= ori.create_null(self.base_fk_control)
        pm.orientConstraint(self.base_fk_control,self.fk_base_jnt)
        
        self.hinge_fk_control= ctrl.create_control(load_shape ='ring')
        pm.matchTransform(self.hinge_fk_control, self.fk_hinge_jnt)
        self.hinge_fk_null= ori.create_null(self.hinge_fk_control)
        pm.orientConstraint(self.hinge_fk_control,self.fk_hinge_jnt)
        pm.parent(self.hinge_fk_null,self.base_fk_control)
        
        self.end_fk_control= ctrl.create_control(load_shape ='ring')
        pm.matchTransform(self.end_fk_control, self.fk_end_jnt)
        self.end_fk_null= ori.create_null(self.end_fk_control)
        pm.orientConstraint(self.end_fk_control,self.fk_end_jnt)
        pm.parent(self.end_fk_null,self.hinge_fk_control)

        #building the pole vector to eventually be constrained to the IK handle. The math will stay within this 
        #specific class but the pole vector locator can be used in other sub-classes
        base_pos = dt.Vector(self.plan['base']['placer_node'].getTranslation(space='world'))
        hinge_pos = dt.Vector(self.plan['hinge']['placer_node'].getTranslation(space='world'))
        end_pos = dt.Vector(self.plan['end']['placer_node'].getTranslation(space='world'))
        base_hinge_dist = (base_pos - hinge_pos)
        base_hinge_dist.normalize()
        hinge_end_dist = (end_pos - hinge_pos)
        hinge_end_dist.normalize()
        
        #creates pole vector according to joint placement
        pv = (base_hinge_dist + hinge_end_dist) * 20
        pv_pos = hinge_pos - pv

        self.pv_loc = pm.spaceLocator()
        self.pv_loc.translate.set(pv_pos)
        self.pv_loc.scaleX.set(2)
        self.pv_loc.scaleY.set(self.pv_loc.scaleX.get())
        self.pv_loc.scaleZ.set(self.pv_loc.scaleX.get())
        pm.makeIdentity(self.pv_loc, apply = True)

        #creating ik handle
        self.ik, self.eff = pm.ikHandle(sj=self.ik_base_jnt, ee=self.ik_end_jnt, sol='ikRPsolver', srp=True, see=True, s='sticky', jl=True, n=(self.side_prefix + 'limb_IK'))
        pm.poleVectorConstraint(self.pv_loc, self.ik)

        #creating ik control
        self.ik_control= ctrl.create_control(load_shape ='cube')
        pm.matchTransform(self.ik_control, self.ik_end_jnt, rot = False, pos = True)
        self.ik_null= ori.create_null(self.ik_control)

        pm.parentConstraint(self.ik_control,self.ik)

        #creating the pole vector control
        self.pv_control = ctrl.create_control(load_shape='jack')
        pm.matchTransform(self.pv_control,self.pv_loc)
        pm.makeIdentity(self.pv_control, apply=True)
        pm.parent(self.pv_loc, self.pv_control)

        #creating the FKIK switch
        self.base_blend = pm.createNode('pairBlend')
        self.hinge_blend = pm.createNode('pairBlend')
        self.end_blend = pm.createNode('pairBlend')

        self.fk_base_jnt.translate >> self.base_blend.inTranslate1
        self.ik_base_jnt.translate >> self.base_blend.inTranslate2
        self.base_blend.outTranslate >> self.plan['base']['joint_node'].translate
        self.fk_base_jnt.rotate >> self.base_blend.inRotate1
        self.ik_base_jnt.rotate >> self.base_blend.inRotate2
        self.base_blend.outRotate >> self.plan['base']['joint_node'].rotate

        self.fk_hinge_jnt.translate >> self.hinge_blend.inTranslate1
        self.ik_hinge_jnt.translate >> self.hinge_blend.inTranslate2
        self.hinge_blend.outTranslate >> self.plan['hinge']['joint_node'].translate
        self.fk_hinge_jnt.rotate >> self.hinge_blend.inRotate1
        self.ik_hinge_jnt.rotate >> self.hinge_blend.inRotate2
        self.hinge_blend.outRotate >> self.plan['hinge']['joint_node'].rotate

        self.fk_end_jnt.translate >> self.end_blend.inTranslate1
        self.ik_end_jnt.translate >> self.end_blend.inTranslate2
        self.end_blend.outTranslate >> self.plan['end']['joint_node'].translate
        self.fk_end_jnt.rotate >> self.end_blend.inRotate1
        self.ik_end_jnt.rotate >> self.end_blend.inRotate2
        self.end_blend.outRotate >> self.plan['end']['joint_node'].rotate

        if (self.base_blend.weight == 0, self.hinge_blend.weight== 0, self.end_blend.weight == 0):
            self.ik_control.visibility = 0
            self.base_fk_null.visibility = 1
        else:
            self.ik_control.visibility = 1
            self.base_fk_null.visibility = 0

        # Select clear to disallow any automatic parenting
        pm.select(cl=True)


class Arm(Limb):
    def __init__(self, name="L_Generic_Arm", dir_prefix=''):
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

        #rename fk joints, controls, and nulls
        self.fk_base_jnt.rename(self.side_prefix + 'FK_shoulder_jnt')
        self.fk_hinge_jnt.rename(self.side_prefix + 'FK_elbow_jnt')
        self.fk_end_jnt.rename(self.side_prefix + 'FK_wrist_jnt')

        self.base_fk_control.rename(self.side_prefix + 'FK_shoulder_CTRL')
        self.hinge_fk_control.rename(self.side_prefix + 'FK_elbow_CTRL')
        self.end_fk_control.rename(self.side_prefix + 'FK_wrist_CTRL')

        self.base_fk_null.rename(self.side_prefix + 'FK_shoulder_NULL')
        self.hinge_fk_null.rename(self.side_prefix + 'FK_elbow_NULL')
        self.end_fk_null.rename(self.side_prefix + 'FK_wrist_NULL')

        self.ik_base_jnt.rename(self.side_prefix + 'IK_shoulder_jnt')
        self.ik_hinge_jnt.rename(self.side_prefix + 'IK_elbow_jnt')
        self.ik_end_jnt.rename(self.side_prefix + 'IK_wrist_jnt')

        #rename ik joints, controls, handle, locator, and nulls
        pm.rename(self.eff,self.side_prefix + 'arm_EFF')
        self.pv_loc.rename(self.side_prefix + "arm_PV_loc")
        self.ik.rename(self.side_prefix + "arm_IK")
        self.ik_null.rename(self.side_prefix + "arm_IK_NULL")
        self.ik_control.rename(self.side_prefix + "arm_IK_CTRL")
        self.pv_control.rename(self.side_prefix + "arm_PV_CTRL")

        #rename fkik switch nodes and controls
        self.base_blend.rename(self.side_prefix + "FKIK_shoulder_BLEND")
        self.hinge_blend.rename(self.side_prefix + "FKIK_elbow_BLEND")
        self.end_blend.rename(self.side_prefix + "FKIK_wrist_BLEND")

        #arm defaults to FK mode
        self.base_blend.weight = 0
        self.hinge_blend.weight = 0
        self.end_blend.weight = 0

        print("Arm Module built, as child of limb module.")


class Arms:
    def __init__(self, name=""):
        '''
        A pair of Arms. This function does the mirroring.
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
    
class Leg(Limb):
    def __init__(self, name="L_Generic_Leg", dir_prefix=''):
        '''
        The least most complicated limb that is still acceptable in the rigging world--
        FK/IK switch, cleanly placed pole-vector, nothing else.
        '''
        super().__init__(name=name, dir_prefix=dir_prefix)

        self.plan['base']['pos'] = (11.0, 114.0, 0.0)
        self.plan['base']['name'] = 'hip'
        self.plan['hinge']['pos'] = (15.0, 66.0, 2.0)
        self.plan['hinge']['name'] = 'knee'
        self.plan['end']['pos'] = (17.0, 15.0, -4.0)
        self.plan['end']['name'] = 'ankle'

        return

    def build_module(self):
        super().build_module()

        #rename fk joints, controls, and nulls
        self.fk_base_jnt.rename(self.side_prefix + 'FK_hip_jnt')
        self.fk_hinge_jnt.rename(self.side_prefix + 'FK_knee_jnt')
        self.fk_end_jnt.rename(self.side_prefix + 'FK_ankle_jnt')

        self.base_fk_control.rename(self.side_prefix + 'FK_hip_CTRL')
        self.hinge_fk_control.rename(self.side_prefix + 'FK_knee_CTRL')
        self.end_fk_control.rename(self.side_prefix + 'FK_ankle_CTRL')

        self.base_fk_null.rename(self.side_prefix + 'FK_hip_NULL')
        self.hinge_fk_null.rename(self.side_prefix + 'FK_knee_NULL')
        self.end_fk_null.rename(self.side_prefix + 'FK_ankle_NULL')

        self.ik_base_jnt.rename(self.side_prefix + 'IK_hip_jnt')
        self.ik_hinge_jnt.rename(self.side_prefix + 'IK_knee_jnt')
        self.ik_end_jnt.rename(self.side_prefix + 'IK_ankle_jnt')

        #rename ik joints, controls, handle, locator, and nulls
        pm.rename(self.eff,self.side_prefix + 'leg_EFF')
        self.pv_loc.rename(self.side_prefix + "leg_PV_loc")
        self.ik.rename(self.side_prefix + "leg_IK")
        self.ik_null.rename(self.side_prefix + "leg_IK_NULL")
        self.ik_control.rename(self.side_prefix + "leg_IK_CTRL")
        self.pv_control.rename(self.side_prefix + "leg_PV_CTRL")

        #rename fkik switch nodes and controls
        self.base_blend.rename(self.side_prefix + "FKIK_hip_BLEND")
        self.hinge_blend.rename(self.side_prefix + "FKIK_knee_BLEND")
        self.end_blend.rename(self.side_prefix + "FKIK_ankle_BLEND")

        #leg defaults to IK mode
        self.base_blend.weight = 0
        self.hinge_blend.weight = 0
        self.end_blend.weight = 0

        print("Leg Module built, as child of limb module.")


class Legs:
    def __init__(self, name=""):
        '''
        A pair of Legs. This function does the mirroring.
        '''

        self._left_leg = Leg("L_leg")
        self._right_leg = Leg("R_leg")

        self._left_leg.build_placers()
        self._right_leg.build_placers()

        # Now we need to build expressions to make placers mirror on X.
        for key in self._left_leg.plan:
            mirror_placer(self._left_leg.plan[key]['placer_node'], 
            self._right_leg.plan[key]['placer_node'])
        
    def build(self):
        '''
        Build both leg modules.
        '''

        print("Building both legs...")
        self._left_leg.build_joints()
        self._right_leg.build_joints()

        return


