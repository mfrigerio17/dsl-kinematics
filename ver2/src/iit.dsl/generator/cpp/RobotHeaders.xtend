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

    def jacobiansHeader(Robot robot) '''
        #ifndef «robot.name.toUpperCase()»_JACOBIANS_H_
        #define «robot.name.toUpperCase()»_JACOBIANS_H_

        #include <iit/robots/«robot.name»_declarations.h>
        #include <iit/rbd/JStateDependentMatrix.h>

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        template<int COLS>
        class JacobianT : public «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», 6, COLS> {
            private:
                typedef «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», 6, COLS> Base;
            public:
                JacobianT(Base::JointStateSetter setter) : Base(setter) {}
        };


        /**
         * The namespace with the Jacobian matrices for the robot «robot.name»
         */
        namespace «Names$Namespaces::jacobians» {
        namespace «Names$Namespaces::internal» {
        /* Declarations */

        } //namespace '«Names$Namespaces::internal»'

        /**
         * Call this function once at initialization time, to prepare the
         * transform matrices.
         **/
        void initAll();

        } // end of namespace '«Names$Namespaces::jacobians»'
        } // end of namespace '«Names$Namespaces::rob(robot)»'
        } // end of namespace '«Names$Namespaces::enclosing»'

        #endif
        '''
}