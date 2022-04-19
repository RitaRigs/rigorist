# orient.py
# Created: Monday, 28th February 2022 9:44:43 pm
# Matthew Riche
# Last Modified: Monday, 28th February 2022 9:48:03 pm
# Modified By: Matthew Riche

from locale import normalize
import pymel.core as pm
import pymel.core.datatypes as dt


def get_vector(subject, target):
    '''
    Get the vector between two objects
    '''

    subject_pos = subject.getTranslation(space='world')
    target_pos = target.getTranslation(space='world')

    aim_vector = (target_pos - subject_pos)
    aim_vector.normalize()

    return aim_vector
    

def aim_at(subject, target, up_vector=(0.0, 0.0, 1.0), up_object=None, aim_axis=0, up_axis=2):
    '''
    Aims the subject node down the target node, and twists it to the provided up-vector.
    Axis involved are specificially defined as 0-2, for x-z.
    
    usage:
    aim_at(PyNode, PyNode, up_vector=(float, float, float), aim_axis=int, up_axis=int)
    '''

    fix_determinant = False # A flag to fix negative determinants

    subject_position = subject.getTranslation(space='world')
    target_position = target.getTranslation(space='world')

    aim_vector = get_vector(subject, target)

    if(up_object is None):
        up_vector_scoped = dt.Vector(up_vector) # named to separate it from the arg.
        up_vector_scoped.normalize()
    else:
        up_vector_scoped = get_vector(subject, up_object)

    last_vector = up_vector_scoped.cross(aim_vector)
    last_vector.normalize()

    # Some combination of chosen axis will create a negative determinent.  This gets reconciled by
    # the scale becoming negative.
    flip_combos = [(0,1), (1,2), (2,0)]
    if((aim_axis, up_axis) in flip_combos):
        fix_determinant = True # With this flag, we know to re-calc the last_axis.

    up_vector_scoped = aim_vector.cross(last_vector)
    up_vector_scoped.normalize()

    # Now the three vectors are ready, their position inside the matrix has to get chosen.
    axes = ['x', 'y', 'z']

    if(fix_determinant):
        # By re-discovering the cross product at this point, we "reverse" it (not negate it!) which
        # will prevent a negative scale in certain arrangements of the matrix (half of the total 
        # possible configurations have this problem.)
        last_vector = aim_vector.cross(up_vector_scoped)
        last_vector.normalize()
        
    if(aim_axis == 0):
        x_row = list(aim_vector)
        axes.remove('x')
    elif(aim_axis == 1):
        y_row = list(aim_vector)
        axes.remove('y')
    elif(aim_axis == 2):
        z_row = list(aim_vector)
        axes.remove('z')
    else:
        pm.error("Bad axis int given for aim_axis: {}; should be 0,1,2 for x,y,z.".format(aim_axis))

    if(up_axis == 0):
        x_row = list(up_vector_scoped)
        axes.remove('x')
    elif(up_axis == 1):
        y_row = list(up_vector_scoped)
        axes.remove('y')
    elif(up_axis == 2):
        z_row = list(up_vector_scoped)
        axes.remove('z')
    else:
        pm.error("Bad axis int given for up_axis: {}; should be 0,1,2 for x,y,z.".format(up_axis))

    if(axes[0] == 'x'):
        x_row = list(last_vector)
    elif(axes[0] == 'y'):
        y_row = list(last_vector)
    elif(axes[0] == 'z'):
        z_row = list(last_vector)
    else:
        pm.error("Failed to determine last vector.  This is a bug.")

    # Append fourth values to the matrix' right side.
    for row in [x_row, y_row, z_row]:
        row.append(0.0)

    # Create the bottom row of the matrix using the transform of the subject, plus a final 1.0
    trans_row = list(subject_position)
    trans_row.append(1.0)

    new_matrix = dt.Matrix(x_row, y_row, z_row, trans_row)
    subject.setMatrix(new_matrix, worldSpace=True)

    return

def create_null(subject):
    '''
    Create an empty trans node, and match transform to a target,
    Then place the target beneath it.
    '''

    null_trans = pm.createNode('transform', n=(subject.name() + '_null'))
    pm.matchTransform(null_trans, subject, scl=False)

    pm.parent(subject, null_trans)

    return null_trans