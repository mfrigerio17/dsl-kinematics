package iit.dsl.generator.misc

import iit.dsl.kinDsl.Robot

import iit.dsl.generator.Common
import iit.dsl.generator.Utilities
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.InertiaParams
import iit.dsl.kinDsl.FloatLiteral

class Misc {
    static Misc singleton = null

    def static Misc getInstance() {
        if(singleton == null) {
            singleton = new Misc()
        }
        return singleton
    }



    private extension Common common = new Common()

    private new() { }

    /**
     * Generates and xml URDF description of the robot, as specified in the ROS
     * documentation.
     *
     * The URDF file format requires the inertia tensor to be expressed in a reference frame
     * with origin in the center of mass. Therefore this generator takes the inertia tensor
     * expressed in the link-default-frame and translates it to the COM before printing the values.
     * In addition, the URDF format requires the elements of the inertia tensor,
     * and not the inertia moments as in the Kinematics-DSL format; therefore
     * the signs of the centrifugal moments are swapped.
     */
    def public URDF_ROS_model(Robot robot) '''
    <robot name="«robot.name»">
    «FOR link : robot.abstractLinks»
        «val inertia_lf = link.linkFrameInertiaParams /*inertia params expressed in the default link frame*/»
        «val inertia = getURDFInertiaParams(inertia_lf)»
        <link name="«link.name»">
            <inertial>
                <origin xyz="«inertia_lf.com.listCoordinates()»"/>
                <mass value="«inertia.mass.str»"/>
                <inertia ixx="«inertia.ix.str»" iyy="«inertia.iy.str»" izz="«inertia.iz.str»" ixy="«inertia.ixy.str»" ixz="«inertia.ixz.str»" iyz="«inertia.iyz.str»"/>
            </inertial>
        </link>
    «ENDFOR»
    «FOR joint : robot.joints»
        «val frame = joint.refFrame»
        <joint name="«joint.name»" type="«joint.typeString»">
            <origin xyz="«frame.translation.listCoordinates()»" rpy="«frame.rotation.x.asFloat» «frame.rotation.y.asFloat» «frame.rotation.z.asFloat»"/>
            <parent link="«joint.predecessorLink.name»"/>
            <child  link="«joint.successorLink.name»"/>
            <limit effort="30" velocity="1.0"/>
            <axis xyz="0 0 1"/>
        </joint>
    «ENDFOR»
    </robot>
    '''
    def private getURDFInertiaParams(InertiaParams inertia_lf) {
        val com = inertia_lf.com
        val inertia_com = // params expressed in the frame centered in the COM
             Utilities::rototranslate(inertia_lf, com.x.asFloat,com.y.asFloat,com.z.asFloat,0,0,0,false)

        // Invert the centrifugal moments, because URDF wants the elements of
        //  the inertia tensor, not the moments
        inertia_com.setIxy( (inertia_com.ixy as FloatLiteral).invert )
        inertia_com.setIxz( (inertia_com.ixz as FloatLiteral).invert )
        inertia_com.setIyz( (inertia_com.iyz as FloatLiteral).invert )
        return inertia_com
    }

    /**
     * Generates the content of an SD/FAST input file representing the given robot
     */
    def SDFAST_model(Robot robot) '''
        #  This is an SD/FAST input file describing the robot «robot.name»
        #
        #  This file has been automatically generated by the robotics code generator
        #   developed by Marco Frigerio at the Italian Institute of Technology.
        #      November 2012

        language = c
        gravity = 0 0 -9.8

        «IF robot.base.floating»
            # Floating base is not supported yet!!! #
        «ELSE»
            «val iparams = sdfast_inertiaParams(robot.base)»
            body = «robot.base.name»
              inb = $ground
              joint = weld
              mass = «iparams.mass.str»
              inertia =
                «iparams.ix.str» «iparams.ixy.str» «iparams.ixz.str»
                «iparams.ixy.str» «iparams.iy.str» «iparams.iyz.str»
                «iparams.ixz.str» «iparams.iyz.str» «iparams.iz.str»
              bodytojoint = 0 0 0
        «ENDIF»

        «FOR link : robot.links»
            «val iparams = sdfast_inertiaParams(link)»
            body = «link.name»
              inb = «link.parent.name»
              joint = «sdfast_jointType(link.connectingJoint)»
              pin = 0 0 1
              mass = «iparams.mass.str»
              inertia =
                «iparams.ix.str» «iparams.ixy.str» «iparams.ixz.str»
                «iparams.ixy.str» «iparams.iy.str» «iparams.iyz.str»
                «iparams.ixz.str» «iparams.iyz.str» «iparams.iz.str»
              bodytojoint = «sdfast_bodyCOMToJointVector(link)»
              inbtojoint = «sdfast_parentBodyToJointVector(link)»

        «ENDFOR»
    '''
    def private dispatch sdfast_jointType(PrismaticJoint j) '''slider'''
    def private dispatch sdfast_jointType(RevoluteJoint  j) '''pin'''
    def private InertiaParams sdfast_inertiaParams(AbstractLink body) {
        val iparams = body.linkFrameInertiaParams

        val iparams_new = /*inertia params expressed in the frame centered in the COM*/
                Utilities::rototranslate(
                iparams,
                iparams.com.x.asFloat,
                iparams.com.y.asFloat,
                iparams.com.z.asFloat,
                0,0,0,false //TODO compute the angles
            )

        return iparams_new
    }
    def private sdfast_bodyCOMToJointVector(AbstractLink link) {
        val x = Utilities::invert(link.inertiaParams.com.x.asFloat)
        val y = Utilities::invert(link.inertiaParams.com.y.asFloat)
        val z = Utilities::invert(link.inertiaParams.com.z.asFloat)
        return '''«x» «y» «z»'''
    }
    def private sdfast_parentBodyToJointVector(AbstractLink body) {
        return body.connectingJoint.refFrame.translation.listCoordinates
    }


}
