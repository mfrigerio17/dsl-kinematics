package iit.dsl.generator.sl.robot

import iit.dsl.kinDsl.Robot

import iit.dsl.generator.cpp.Names
import iit.dsl.generator.sl.Utilities
import iit.dsl.generator.sl.Common

class Headers {

    def SL_user(Robot robot)'''
        #ifndef _SL_USER_«robot.name.toUpperCase»_H_
        #define _SL_USER_«robot.name.toUpperCase»_H_

        #include "SL.h"

        /*! the robot name */
        #define ROBOT_NAME "«robot.name»"

        /*! links of the robot */
        enum RobotLinks {
            PLACEHOLDER = 0,
            «FOR link : robot.links»
                «Common::linkEnumID(link)»,
            «ENDFOR»
            «Common::DUMMY_EE_LINK_ID»,
            N_ROBOT_LINKS
        };

        /*! endeffector information */
        enum RobotEndeffectors {
          ENDEFF=1,
          N_ROBOT_ENDEFFECTORS
        };

        /*! vision variables */
        enum VisionCameras {
          N_VISION_CAMERAS
        };

        enum ColorBlobs {
          BLOB1=1,
          N_COLOR_BLOBS
        };

        /*! define the DOFs of this robot */
        enum RobotDOFs {
            BASE=0,
            «FOR joint : robot.joints»
                «Common::jointEnumID(joint)»,
            «ENDFOR»
            N_ROBOT_DOFS
        };

        «val sens = Utilities::defaultMiscSensors»
        /*! define miscellenous sensors of this robot */
        enum RobotMiscSensors {
            «sens.get(0)»=1,
            «FOR s : sens.drop(1)»
                «s»,
            «ENDFOR»
            N_ROBOT_MISC_SENSORS
        };

        /*! number of degrees-of-freedom of robot */
        #define N_DOFS (N_ROBOT_DOFS-1)

        /*! N_DOFS + fake DOFS, needed for parameter estimation;
           fake DOFS come from creating endeffector information */
        #define N_DOFS_EST (N_DOFS+31)

        /*! N_DOFS to be excluded from parameter estimation (e.g., eye joints);
           these DOFS must be the last DOFS in the arrays */
        #define N_DOFS_EST_SKIP 0

        /*! number of links of the robot */
        #define N_LINKS    (N_ROBOT_LINKS-1)

        /*! number of miscellaneous sensors */
        #define N_MISC_SENSORS   (N_ROBOT_MISC_SENSORS-1)

        /*! number of endeffectors */
        #define N_ENDEFFS  (N_ROBOT_ENDEFFECTORS-1)

        /*! number of cameras used */
        #define N_CAMERAS (N_VISION_CAMERAS-1)

        /*! number of blobs that can be tracked by vision system */
        #define MAX_BLOBS (N_COLOR_BLOBS-1)

        /*! vision default post processing */
        #define VISION_DEFAULT_PP "vision_default.pp"

        /*! the servo rate used by the I/O with robot: this limits the
           servo rates of all other servos */
        #define  SERVO_BASE_RATE 1000

        /*! divisor to obtain task servo rate (task servo can run slower than
           base rate, but only in integer fractions */
        #define  TASK_SERVO_RATIO   R1TO4
        //! #define  TASK_SERVO_RATIO   R1TO1

        /* settings for D/A debugging information -- see SL_oscilloscope.c */
        #define   D2A_CM      1
        #define   D2A_CT      2
        #define   D2A_CV      3
        #define   D2A_CR      4

        #ifdef __cplusplus
        extern "C" {
        #endif

        extern int    real_time_flag;
        extern double force_biases[N_ENDEFFS+1][N_CART+1];

        #ifdef __cplusplus
        }
        #endif
        #endif  /* _SL_USER_«robot.name.toUpperCase»_H_ */
    '''

    def public robcogenGlobals(Robot robot) '''
        «val robodir = Names$Files::folder(robot)»
        «val ns = iit::dsl::generator::cpp::Common::enclosingNamespacesQualifier(robot)»
        #ifndef _IIT_«robot.name.toUpperCase»_SL__ROBOGEN_GLOBALS_H_
        #define _IIT_«robot.name.toUpperCase»_SL__ROBOGEN_GLOBALS_H_

        #include <iit/robots/«robodir»/«Names$Files::mainHeader(robot)».h>
        #include <iit/robots/«robodir»/«Names$Files::parametersHeader(robot)».h>
        #include <iit/robots/«robodir»/«Names$Files::transformsHeader(robot)».h>
        #include <iit/robots/«robodir»/«Names$Files$RBD::fwdDynHeader(robot)».h>
        #include <iit/robots/«robodir»/«Names$Files$RBD::invDynHeader(robot)».h>

        #include <SL.h>
        #include <SL_user.h>

        namespace robot = «ns»;

        «iit::dsl::generator::cpp::Common::enclosingNamespacesOpen(robot)»
        namespace SL {

        extern «ns»::HomogeneousTransforms* homogeneousTransforms;
        extern «ns»::MotionTransforms* motionTransforms;
        extern «ns»::ForceTransforms* forceTransforms;
        extern «ns»::«Names$Namespaces::dynamics»::ForwardDynamics* fwdDynEngine;
        extern «ns»::«Names$Namespaces::dynamics»::InverseDynamics* invDynEngine;

        «IF robot.base.floating»
            extern «ns»::HomogeneousTransforms::MatrixType world_X_base;
        «ENDIF»

        inline void updateEndeffectorsParams(SL_endeff* eff) {
            //TODO
        }

        void createDefaultTransformsAndDynamics();

        «IF robot.base.floating»
            void update__world_X_base(const SL_Cstate& base_pos,const SL_quat& base_orient);
        «ENDIF»

        }
        «iit::dsl::generator::cpp::Common::enclosingNamespacesClose(robot)»



        #endif
    '''

    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

}