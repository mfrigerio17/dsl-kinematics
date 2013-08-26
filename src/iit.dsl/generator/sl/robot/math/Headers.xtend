package iit.dsl.generator.sl.robot.math

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.common.Vector3D
import iit.dsl.generator.sl.Common
import iit.dsl.generator.common.TreeUtils

class Headers {

    def public gjac_declare(Robot robot) '''
        // This is a set of flags used by SL to determine which joint (column
        //  index of this matrix) contributes to the velocity of which
        //  end-effector (row index)
        int Jlist[1 + «robot.chainEndLinks.size»][1 + «robot.joints.size»];
    '''
    def public gjac_math(Robot robot) {
        val code = new StringBuffer
        val leafs = robot.chainEndLinks
        var ee_i = 1
        for(leaf : leafs) {
            code.append('''bzero(Jlist[«ee_i»], sizeof(Jlist[«ee_i»]));''')
            code.append("\n");
            val chain = TreeUtils::chainToBase(leaf)
            for(l : chain) {
               code.append('''Jlist[«ee_i»][«Common::jointEnumID(l.connectingJoint)»] = 1;''')
               code.append("\n");
            }
            ee_i = ee_i + 1
        }
        return code
    }

    def public contact_gjac_declare(Robot robot) '''
        // This is a set of flags used by SL to determine which joint (column
        //  index of this matrix) contributes to the velocity of which
        //  link (row index)
        int Jlist[1 + «robot.links.size»][1 + «robot.joints.size»];
    '''
    def public contact_gjac_math(Robot robot) {
        val code = new StringBuffer
        for(link : robot.links) {
            val lID = Common::linkEnumID(link)
            code.append('''bzero(Jlist[«lID»], sizeof(Jlist[«lID»]));''')
            code.append("\n");
            val chain = TreeUtils::chainToBase(link)
            for(l : chain) {
               code.append('''Jlist[«lID»][«Common::jointEnumID(l.connectingJoint)»] = 1;''')
               code.append("\n");
            }
        }
        return code
    }

    def public prismatic_joints(Robot robot) '''
        «FOR j : robot.joints»
            prismatic_joint_flag[«j.arrayIdx + 1»] = «j.prismaticFlag»;
        «ENDFOR»
    '''
    def private dispatch prismaticFlag(RevoluteJoint joint) '''0'''
    def private dispatch prismaticFlag(PrismaticJoint joint)'''1'''

    def public floating_base(Robot robot)
    '''const int floating_base_flag = «IF robot.base.floating»1«ELSE»0«ENDIF»;'''

    def public opengl(Robot robot) '''
        #define RAD2DEG (57.3)

        static double x,y,z; // support vars

        «IF robot.base.floating»
            // The state of the floating base
            glPushMatrix();
            glTranslated((GLdouble)basec[0].x[1],(GLdouble)basec[0].x[2],(GLdouble)basec[0].x[3]);
            glRotated((GLdouble)114.5916*ArcCos(baseo[0].q[1]),(GLdouble)baseo[0].q[2],(GLdouble)baseo[0].q[3],(GLdouble)baseo[0].q[4]);
        «ENDIF»

        «opengl_depthVisit(robot.base)»

        «IF robot.base.floating»
            // pops the first matrix related to the state of the base
            glPopMatrix();
        «ENDIF»
    '''

    def private opengl_depthVisit(AbstractLink link) '''
        «IF link.childrenList.children.size == 0»
            // Draw the end effector
            glPushMatrix();
            x = eff[«opengl_ee_count = opengl_ee_count + 1»].x[_X_];
            y = eff[«opengl_ee_count»].x[_Y_];
            z = eff[«opengl_ee_count»].x[_Z_];
            glRotated(
                    (GLdouble)RAD2DEG*acos(z),(GLdouble)-y,(GLdouble)x,(GLdouble)0);
            myDrawGLElement(«100+opengl_ee_count», (double)Sqrt(x*x + y*y + z*z), 0);
            glPopMatrix();
        «ENDIF»
        «FOR childSpec : link.childrenList.children»
            «val j = childSpec.joint»
            // Joint «j.name»

            glPushMatrix();
            «drawJointCode(j)»

            «opengl_depthVisit(childSpec.link)»

            glPopMatrix();
        «ENDFOR»
    '''
    private int opengl_ee_count = 0;


    def private drawJointCode(Joint joint)
        '''
        «val vec = Vector3D::convert(joint.refFrame.translation)»
        «val rot = Vector3D::convert(joint.refFrame.rotation)»
        glPushMatrix();
        // Align the Z axis along the direction between the two joints, to display
        //  the link correctly ('myDrawGLElement()' draws along the Z axis)
        «alignZwithLink(vec)»
        myDrawGLElement(«Common::jointEnumID(joint)», «Vector3D::norm(joint.refFrame.translation)», 1);
        glPopMatrix();

        // move to the next joint, the same parameters as in the kinematics model file
        glTranslated((GLdouble)«vec.x», (GLdouble)«vec.y», (GLdouble)«vec.z»);
        glRotated((GLdouble)(RAD2DEG*«rot.x»), (GLdouble)1.0, (GLdouble)0.0, (GLdouble)0.0);
        glRotated((GLdouble)(RAD2DEG*«rot.y»), (GLdouble)0.0, (GLdouble)1.0, (GLdouble)0.0);
        glRotated((GLdouble)(RAD2DEG*«rot.z»), (GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);

        // move according to the joint state
        «jointStateMove(joint)»
        '''


    def private dispatch jointStateMove(RevoluteJoint joint)
        '''glRotated((GLdouble)RAD2DEG*state[«Common::jointEnumID(joint)»].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);'''

    def private dispatch jointStateMove(PrismaticJoint joint)
        '''glTranslated((GLdouble)0.0, (GLdouble)0.0, (GLdouble)state[«Common::jointEnumID(joint)»].th);'''

    def private alignZwithLink(Vector3D vec) {
        val angle = java::lang::Math::acos(vec.z)
        return '''glRotated((GLdouble)(RAD2DEG*«angle»), (GLdouble)-(«vec.y»), (GLdouble)«vec.x», (GLdouble)0.0);'''
    }

    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common
}