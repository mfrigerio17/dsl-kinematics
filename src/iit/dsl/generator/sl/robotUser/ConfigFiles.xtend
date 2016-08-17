package iit.dsl.generator.sl.robotUser

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.sl.Utilities

class ConfigFiles {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    /**
    Configuration file with Inertia parameters
    It is assumed that the provided moments of inertia strictly follow their definition,
    so that the off-diagonal elements of the inertia tensor have a minus sign. It is also
    assumed that the configuration file for SL expects the inertia tensor elements, so we
    put that minus in front of the values
    */
    def linkParameters(Robot robot)'''
        «val auxparb = getLinkFrameInertiaParams(robot.base)»
        «val bparams = Utilities::tuneForSL(auxparb)»
        BASE «'\t'»«bparams.mass.str»   «bparams.com.x.str» «bparams.com.y.str» «bparams.com.z.str»   «bparams.ix.str» «bparams.ixy.str» «bparams.ixz.str» «bparams.iy.str» «bparams.iyz.str» «bparams.iz.str»   0.1 0 0 0
        «FOR link : robot.links»
            «val auxpar = getLinkFrameInertiaParams(link)»
            «val params = Utilities::tuneForSL(auxpar)»
            «link.connectingJoint.name» «'\t'»«params.mass.str»   «params.com.x.str» «params.com.y.str» «params.com.z.str»   «params.ix.str» «params.ixy.str» «params.ixz.str» «params.iy.str» «params.iyz.str» «params.iz.str»   0.1 0 0 0
        «ENDFOR»
        '''

    def gains(Robot robot) '''
        «FOR Joint joint : robot.joints»
            «joint.name»   600.0    10.0    0.0   1000
        «ENDFOR»
    '''

    def sensorCalibration(Robot robot) '''
        «FOR Joint joint : robot.joints»
            «joint.name»   0 0 0 0 0 1 1
        «ENDFOR»
    '''

    def whichDOFs(Robot robot) '''
        /* this file contains a list of DOFs that are computed by each processor
           Note: the file is parsed according to keywords, after the keyword,
           the values need to come in the correct sequence */

           /* format: keyword, <list of joint names for a processor> */

        task_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        task_sim_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        motor_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        motor_sim_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        vision_servo
        vision_sim_servo
        invdyn_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        invdyn_sim_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»
    '''

    def sensorOffset(Robot robot) '''
        /* this file contains the specification for position offsets and min/max
           position values (min and max AFTER offset was subtracted. Additionally,
           the file allows to set a default posture for the robot.
           Note: the file is parsed according to keywords, after the keyword,
                 the values need to come in the correct sequence */
        /* format: keyword, min , max, default, rest, weight, offset  */
        /* Please edit this file according to your needs */

       «FOR Joint joint : robot.joints»
            «joint.name» -2.0  2.0  0.0  0.0  0.1  0.0
       «ENDFOR»
    '''

    def sensorFilters(Robot robot) '''
        «FOR Joint joint : robot.joints»
            «joint.name»   100  100  100  100
        «ENDFOR»

        «FOR s : Utilities::defaultMiscSensors»
            «s»«'\t\t'»100  100  100  100
        «ENDFOR»
    '''
}