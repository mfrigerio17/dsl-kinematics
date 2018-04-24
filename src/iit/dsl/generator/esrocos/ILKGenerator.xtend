package iit.dsl.generator.esrocos

import iit.dsl.generator.Common
import iit.dsl.generator.common.TreeUtils
import iit.dsl.generator.common.gr.AttachedFrame
import iit.dsl.generator.common.gr.RobotPoseUtils.JointSuccessorPose
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint

class ILKGenerator
{
    public new(QueryProcessor q) {
        this.query= q
        this.tree = q.treeUtils
    }

    def public lua()
    '''
    return {
        { op='model_const', args={ «FOR kk: query.constantPoses SEPARATOR","»'«kk.name»'«ENDFOR»} },
        «FOR c : query.jointPoses SEPARATOR"," AFTER","»
            { op='model_T_joint_local', name='«c.name()»', jtype='«c.joint.typestr»', dir='«c.directionTag»', input=«c.joint.coordinateIdx» }
        «ENDFOR»

        «FOR c : query.composes SEPARATOR ","AFTER","»
            { op='compose', args={ '«c.arg1.name»', '«c.arg2.name»', '«c.result.name»' } }
        «ENDFOR»

        «FOR c : query.outputs SEPARATOR ","»
            { op='output', '«c.name»' }
        «ENDFOR»
    }
    '''

    def private dispatch typestr(RevoluteJoint j)  { return "revolute" }
    def private dispatch typestr(PrismaticJoint j) { return "prismatic" }
    def private int coordinateIdx(Joint j) {
        tree.successor(j).ID - 1
    }

    def private directionTag(JointSuccessorPose pose) {
        val targetKind = pose.target().role()
        if( targetKind == AttachedFrame.FrameRole.link ) {
            return "a_x_b"
        } else {
            return "b_x_a"
        }
    }

    private QueryProcessor query
    private TreeUtils tree

    private static extension Common helper = Common.getInstance()
}