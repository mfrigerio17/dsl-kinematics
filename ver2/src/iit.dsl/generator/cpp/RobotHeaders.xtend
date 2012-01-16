package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint

class RobotHeaders {
    def static mainHeaderFileName(Robot robot) {
        return robot.name + "_declarations";
    }

    def main(Robot robot)'''
        #ifndef IIT_«mainHeaderFileName(robot).toUpperCase()»_H_
        #define IIT_«mainHeaderFileName(robot).toUpperCase()»_H_

        #include <Eigen/Dense>

        namespace iit {
        namespace «robot.name» {

        typedef Eigen::Matrix<double, «robot.joints.size», 1> Column«robot.joints.size»d;
        typedef Column«robot.joints.size»d JointState;

        enum JointIdentifiers {
            «Common::jointIdentifier(robot.joints.get(0))» = 0
            «FOR Joint j : robot.joints.drop(1)»
            , «Common::jointIdentifier(j)»
            «ENDFOR»
        };

        }
        }
        #endif
        '''
}