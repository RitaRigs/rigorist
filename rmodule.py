# rmodule.py
# Created: Tuesday, 25th January 2022 9:48:58 am
# Matthew Riche
# Last Modified: Friday, 18th February 2022 9:08:08 pm
# Modified By: Matthew Riche

'''
rmodule.py

The intent is to have a class instance that always has two build phases, one for placers and one 
for construction.
'''

import pymel.core as pm
from . import colour as cl

class RMod:
    def __init__(self):
        '''
        Generic module.  Each one will know where it's placers should go, and have a rather 
        vanilla build-script
        '''

        self.name = "Generic rmodule."
        self.side_prefix = "C_"
        self.dir_prefix = ""
        self.placer_list = [] # Placers identities to be built.
        self.dependencies = [] # Modules that must be built first.
        self.placer_nodes = [] # Placers in-scene currently as nodes.
        self.build_nodes = [] # Nodes in-scene built by this module.

        return

    def build_placers(self):
        '''
        Run through all the placers in placer_list and create them in the scene.
        '''

        print("Creating Placers...")

        return

    def build_module(self):
        '''
        Run through all build instructions to create this module in-scene.
        '''

        pm.warning("This is an instance of a generic parent class, and has no build instructions.")

        return

    def clean_placers(self):
        '''
        Delete all in-scene placers relating to this module.
        '''

        print("Deleting placers.")
        for placer in self.placer_nodes:
            pm.delete(placer)
            print("Cleaning up {}".format(placer.name()))
        return

    def __str__(self):
        '''
        When cast as string, just print the details to console.
        '''

        return ("(Rigorist Module: {}{}{})".format(self.side_prefix, self.dir_prefix, self.name))

    def __getitem__(self, i):
        '''
        When indexed, we just send back each piece of data.
        '''
        _as_list = ([self.name, self.side_prefix, self.dir_prefix, self.placer_list, 
            self.dependencies, self.placer_nodes])

        return _as_list[i]


class Limb(RMod):
    def __init__(self):
        '''
        A generic limb as a base, hinge, and end joint.  For an arm this will be shoulder, elbow,
        and wrist, and for a leg this will be hip, knee, and ankle.
        '''
        super().__init__()

        self.placer_list = [
            ((0.0, 3.0, 0.0), 10, 'base_joint', 'orange'),
            ((0.0, 2.0, 0.0), 8, 'hinge_joint', 'orange'),
            ((0.0, 1.0, 0.0), 10, 'end_joint', 'orange'),
        ]

        # Get membership for the essential joints of a parent joint.
        self.base_joint = None
        self.hinge_joint = None
        self.end_joint = None


def create_placer( pos=(0.0, 0.0, 0.0), size=10, name='RigIdPlacer', colour='blue'):
    '''
    create_placer
    Makes a nice way to visualize a placer that stands out.
    
    USAGE:
    create_placer( pos, size, name, colour)
    Arguments taken are the three vectors of the worlds space xform, the 
    visualized size, a name and a colour. Xform vectors are separate so that the
    use has the option of leaving one or two out.
    '''

    # "Placers are nurbs spheres with colour override.  These are intended 
    # basically as locators, but more clickable and visible."
    print ("Creating a placer named {}...").format(name)

    # Nurbs sphere placer is created and moved to the coords passed.
    new_placer = pm.sphere(polygon=0, radius=size, name=name)[0]
    print ("Moving nurbs sphere.")
    pm.move(pos, new_placer)

    # Disconnect the initial Shader
    initial_shader_grp = pm.PyNode('initialShadingGroup')

    try:
        pm.disconnectAttr
        (new_placer.getShape().instObjGroups[0], 
        initial_shader_grp.dagSetMembers[0]
        )
    except:
        print ("Tried to disconnected initial_shader.grp, it was already gone.")

    # Set up colour override
    cl.change_colour(new_placer, colour)

    print ("{} has been created.").format(new_placer)

    return new_placer
