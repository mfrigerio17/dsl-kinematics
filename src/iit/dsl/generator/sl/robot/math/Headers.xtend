package iit.dsl.generator.sl.robot.math

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Vector3
import iit.dsl.kinDsl.Expr
import iit.dsl.generator.common.Vector3D
import iit.dsl.generator.sl.Common
import iit.dsl.generator.common.TreeUtils
import iit.dsl.generator.common.Parameters
import iit.dsl.generator.cpp.ParametrizationHelper

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

    def public opengl(Robot robot) {
        return new OpenGL_h(robot).body()
    }

    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common
}


/**
 * Class dedicated to the generation of OpenGL.h
 */
class OpenGL_h {
    new(Robot r) {
        robot = r
        parametric = Parameters::kinematicsIsParametric(robot)
        if(parametric) {
            paramsHelper = new ParametrizationHelper(robot)
        }
    }

    def public body()
        '''
        «val ns = iit::dsl::generator::cpp::Common::enclosingNamespacesQualifier(robot)»
        #define RAD2DEG (57.3)

        static double x,y,z; // support vars
        «IF parametric»
            «IF paramsHelper.angleParams.size > 0»
                «ns»::«paramsHelper.anglesStructVarDeclaration()» _angles;
            «ENDIF»
            «IF paramsHelper.lengthParams.size > 0»
                «ns»::«paramsHelper.lengthsStructVarDeclaration()» _lengths;
            «ENDIF»
            «FOR l : paramsHelper.lengthParams»
                _lengths.«paramsHelper.structField(l)» = «ns»::SL::paramsGetter -> «paramsHelper.getterFunction(l)»();
            «ENDFOR»
            «FOR l : paramsHelper.angleParams»
                _angles.«paramsHelper.structField(l)» = «ns»::SL::paramsGetter -> «paramsHelper.getterFunction(l)»();
            «ENDFOR»
        «ENDIF»

        // The state of the base
        glPushMatrix();
        glTranslated((GLdouble)basec[0].x[1],(GLdouble)basec[0].x[2],(GLdouble)basec[0].x[3]);
        glRotated((GLdouble)114.5916*ArcCos(baseo[0].q[1]),(GLdouble)baseo[0].q[2],(GLdouble)baseo[0].q[3],(GLdouble)baseo[0].q[4]);

        «opengl_depthVisit(robot.base)»

        // pops the first matrix related to the state of the base
        glPopMatrix();
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
        «val vec = new VectorString(joint.refFrame.translation, "_lengths", paramsHelper)»
        «val rot = new VectorString(joint.refFrame.rotation, "_angles", paramsHelper)»
        glPushMatrix();
        // Align the Z axis along the direction between the two joints, to display
        //  the link correctly ('myDrawGLElement()' draws along the Z axis)
        «alignZwithLink(joint)»
        myDrawGLElement(::«Common::jointEnumID(joint)», «distance(joint)», 1);
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
        '''glRotated((GLdouble)RAD2DEG*state[::«Common::jointEnumID(joint)»].th,(GLdouble)0.0, (GLdouble)0.0, (GLdouble)1.0);'''

    def private dispatch jointStateMove(PrismaticJoint joint)
        '''glTranslated((GLdouble)0.0, (GLdouble)0.0, (GLdouble)state[::«Common::jointEnumID(joint)»].th);'''

    def private alignZwithLink(Joint j)
    {
        var CharSequence firstItem
        val pos = j.refFrame.translation
        // If there is no translation along x nor y, then the current OpenGL frame
        //  has already the z axis aligned with the line connecting the two joints
        if( !Parameters::isParameter(pos.x)  &&
            !Parameters::isParameter(pos.y) )
        {
            if(pos.x.asFloat == 0.0  &&  pos.y.asFloat == 0.0) {
                return '''// nothing to do'''
            }
        }

        val str = new VectorString(pos, "_lengths", paramsHelper)
        firstItem = '''(RAD2DEG*std::acos( «str.z»/(«str.norm()») ))'''

        return
        '''glRotated((GLdouble)«firstItem», (GLdouble)-(«str.y»), (GLdouble)«str.x», (GLdouble)0.0);'''
    }


    def private distance(Joint j) {
        val vec = j.refFrame.translation
        if( ! Parameters::isParametric(vec) ) {
            return Vector3D::norm(vec)
        } else {
            return new VectorString(vec, "_lengths", paramsHelper).norm()
        }
    }

    private Robot robot
    private boolean parametric
    private ParametrizationHelper paramsHelper = null
    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common
}

/**
 * Simple wrapper that converts the 3 elements of a vector into strings.
 * The class supports constants and parameters. Constant elements (ie floats)
 * are simply converted to the string representation of the number, while
 * parameters are converted with the code that accesses a field of a struct.
 */
class VectorString {
    public String x
    public String y
    public String z

    new(Vector3 vec, String struct, ParametrizationHelper paramsHelper)
    {
        if( ! Parameters::isParametric(vec) ) {
            val tmp = Vector3D::convert(vec)
            x = tmp.x.toString
            y = tmp.y.toString
            z = tmp.z.toString
        } else {
            val iter = Parameters::getParameters(vec).iterator
            if(Parameters::isParameter(vec.x)) {
                x = struct + "." + paramsHelper.structField(iter.next)
                if( (vec.x as Expr).identifier.minus ) {
                    x = "-" + x;
                }
            } else {
                x = helper.asFloat(vec.x).toString;
            }
            if(Parameters::isParameter(vec.y)) {
                y = struct + "." + paramsHelper.structField(iter.next)
                if( (vec.y as Expr).identifier.minus ) {
                    y = "-" + y;
                }
            } else {
                y = helper.asFloat(vec.y).toString;
            }
            if(Parameters::isParameter(vec.z)) {
                z = struct + "." + paramsHelper.structField(iter.next)
                if( (vec.z as Expr).identifier.minus ) {
                    z = "-" + z;
                }
            } else {
                z = helper.asFloat(vec.z).toString;
            }
        }
    }

    def public norm() {
        return '''std::sqrt(«x»*«x» + «y»*«y» + «z»*«z»)'''
    }

    private static iit.dsl.generator.Common helper = new iit.dsl.generator.Common();
}