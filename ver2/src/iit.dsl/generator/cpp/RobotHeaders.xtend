package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint


class RobotHeaders {

    def main(Robot robot)'''
        #ifndef IIT_«Names$Files::mainHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files::mainHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        static const int JointSpaceDimension = «robot.joints.size»;
        static const int jointsCount = «robot.joints.size»;
        /** The total number of rigid bodies of this robot, including the base */
        static const int linksCount  = «robot.links.size + 1»;

        typedef Eigen::Matrix<double, «robot.joints.size», 1> Column«robot.joints.size»d;
        typedef Column«robot.joints.size»d JointState;

        enum JointIdentifiers {
            «Common::jointIdentifier(robot.joints.get(0))» = 0
            «FOR Joint j : robot.joints.drop(1)»
            , «Common::jointIdentifier(j)»
            «ENDFOR»
        };

        enum LinkIdentifiers {
            «Common::linkIdentifier(robot.base)» = 0
            «FOR l : robot.links»
            , «Common::linkIdentifier(l)»
            «ENDFOR»
        };

        }
        }
        #endif
        '''
}