package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint

class RobotHeaders {
    def main(Robot robot)'''
        #ifndef IIT_«Names$Files::mainHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files::mainHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>

        namespace iit {
        namespace «Names$Namespaces::rob(robot)» {

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