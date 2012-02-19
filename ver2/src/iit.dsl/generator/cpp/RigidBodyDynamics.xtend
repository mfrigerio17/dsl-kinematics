package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.FixedRobotBase
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Link

import com.google.inject.Inject

class RigidBodyDynamics {

    @Inject extension iit.dsl.generator.Common common

    def static headerFileName(Robot r)'''«r.name»_dynamics'''
    def static sourceFileName(Robot r)'''«r.name»_dynamics'''
    def static className(Robot r) '''Dynamics'''

    def mainHeader(Robot robot) '''
        #ifndef IIT_RBD_«Names$Files$RBD::header(robot).toUpperCase()»_DYNAMICS_H_
        #define IIT_RBD_«Names$Files$RBD::header(robot).toUpperCase()»_DYNAMICS_H_

        #include <Eigen/Dense>
        #include <iit/robots/«Names$Files::mainHeader(robot)».h>
        #include <iit/robots/«Names$Files::transformsHeader(robot)».h>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/CoordinateTransform.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/MotionSubspaceMx.h>
        #include <iit/rbd/utils.h>

        namespace iit {
        namespace «Names$Namespaces::rob(robot)» {

        typedef iit::rbd::InertiaMatrixDense InertiaMatrix;
        typedef iit::rbd::MotionSubspaceMxSparse SubspaceMx;
        typedef iit::rbd::SparseColumnd JointVelocity;


        class «className(robot)» {
        public:
            «className(robot)»();
            /**
             * The inverse dynamics routine for this robot
             */
            void id(const JointState& q, const JointState& qd, const JointState& qdd, JointState& torques);
        public:
            iit::rbd::SparseColumnd gravity;

        private:
            iit::rbd::Matrix66d spareMx; // support variable
            JointVelocity vJ;
            «FOR Joint j : robot.joints»
                // Joint '«j.name»' :
                SubspaceMx «j.subspaceMxName»;
            «ENDFOR»
            «FOR l : robot.links»
                // Link '«l.name»' :
                InertiaMatrix «inertiaMxName(l)»;
                iit::rbd::VelocityVector «velocityName(l)»;
                iit::rbd::VelocityVector «accelerationName(l)»;
                iit::rbd::ForceVector  «forceName(l)»;
            «ENDFOR»

        };

        }
        }

        #endif
        '''

        def inverseDynamicsImplementation(Robot robot)'''
            #include "«Names$Files$RBD::header(robot)».h"
            #ifndef EIGEN_NO_DEBUG
                #include <iostream>
            #endif

            using namespace std;
            using namespace iit::rbd;
            using namespace «fullNSQualifier(robot)»;

            «fullNSQualifier(robot)»::«className(robot)»::«className(robot)»() {
            #ifndef EIGEN_NO_DEBUG
                std::cout << "Robot «robot.name», «className(robot)»::«className(robot)»()" << std::endl;
                std::cout << "Compiled with Eigen debug active" << std::endl;
            #endif
                gravity.resize(6);
                gravity.insert(5) = 9.81;
                vJ.resize(6);

                «FOR Joint j : robot.joints»
                    «jointSubspaceMx(j)»
                «ENDFOR»
                «FOR l : robot.links»
                    «inertiaMxName(l)».fill(«l.inertiaParams.mass», Vector3d(«l.inertiaParams.com.x.str»,«l.inertiaParams.com.y.str»,«l.inertiaParams.com.z.str»),
                        Utils::buildInertiaTensor(«l.inertiaParams.ix»,«l.inertiaParams.iy»,«l.inertiaParams.iz»,«l.inertiaParams.ixy»,«l.inertiaParams.ixz»,«l.inertiaParams.iyz»));
                «ENDFOR»

                «Names$Namespaces::transforms6D»::initAll(); // initializes coordinates transforms
            }

            void «fullNSQualifier(robot)»::«className(robot)»::id(const JointState& q, const JointState& qd, const JointState& qdd, JointState& torques) {
                «val sortedLinks = robot.abstractLinks.sortBy(link | getID(link))»
                «FOR AbstractLink l : sortedLinks»
                    «inverseDynamicsPass1(l)»

                «ENDFOR»
                «FOR AbstractLink l : sortedLinks.reverse()»
                    «inverseDynamicsPass2(l)»

                «ENDFOR»
            } '''

    def private fullNSQualifier(Robot r) '''«Names$Namespaces::enclosing»::«Names$Namespaces::rob(r)»'''

    def private dispatch jointSubspaceMx(PrismaticJoint j) '''
        «j.subspaceMxName».resize(6);
        «j.subspaceMxName».insert(5) = 1.0;
    '''
    def private dispatch jointSubspaceMx(RevoluteJoint j) '''
        «j.subspaceMxName».resize(6);
        «j.subspaceMxName».insert(2) = 1.0;
    '''


    def private dispatch inverseDynamicsPass1(FixedRobotBase base)''''''
    def private dispatch inverseDynamicsPass1(Link l)'''
           // First pass, link '«l.name»'
           «val parentLink = l.parent»
           «val myJoint = l.connectingJoint»
           «val velocity = l.velocityName»
           «val acceler = l.accelerationName»
           «val transform = child_X_parent__mxName(parentLink, l)»
           «transform»(q); // updates the transform with the joint status
           «IF parentLink.getID() == 0»
               «velocity» = «myJoint.subspaceMxName» * qd(«myJoint.ID-1»); // «velocity» = vJ, for the first link of a fixed base robot
               «acceler» = («transform» * gravity) + («myJoint.subspaceMxName()» * qdd(«myJoint.ID-1»)).toDense();
               Utils::fillAsForceCrossProductMx(«velocity», spareMx); // this could be optimized..
               «forceName(l)» = «inertiaMxName(l)» * «acceler» + (spareMx * «inertiaMxName(l)» * «velocity»);
           «ELSE»
               vJ = «myJoint.subspaceMxName» * qd(«myJoint.ID-1»);
               «velocity» = («transform» * «parentLink.velocityName») + vJ.toDense();
               Utils::fillAsMotionCrossProductMx(«velocity», spareMx); // this could be optimized..
               «acceler» = («transform» * «parentLink.accelerationName») +
                        («myJoint.subspaceMxName» * qdd(«myJoint.ID-1»)).toDense() +
                        (spareMx * vJ);
               «forceName(l)» = «inertiaMxName(l)» * «acceler» + (-spareMx.transpose() * «inertiaMxName(l)» * «velocity»);
           «ENDIF»
           '''

    def private dispatch inverseDynamicsPass2(FixedRobotBase base)''''''
    def private dispatch inverseDynamicsPass2(Link l)'''
        // Second pass, link '«l.name»'
        «val parentLink = l.parent»
        «val myJoint = l.connectingJoint»
        torques(«myJoint.ID-1») = «myJoint.subspaceMxName».transpose().dot(«l.forceName»);
        «IF parentLink.ID != 0»
            «parentLink.forceName» = «parentLink.forceName» + «child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
        «ENDIF»
        '''

    def private child_X_parent__mxName(AbstractLink parent, AbstractLink child) '''
        «Names$Namespaces::transforms6D»::fr_«child.name»_X_fr_«parent.name»'''

    def testMain(Robot robot) '''
        #include <cmath>
        #include <iostream>

        #include "«Names$Files$RBD::header(robot)».h"

        using namespace std;
        using namespace «fullNSQualifier(robot)»;
        using namespace iit::rbd;

        int main(int argc, char** argv) {
            «Names$Types::jointState» q, qd, qdd, tau;
            «FOR Joint j : robot.joints»
            q(«j.getID()-1»)   = std::atof(argv[«j.getID()»]);
            qd(«j.getID()-1»)  = std::atof(argv[«j.getID() + robot.joints.size»]);
            qdd(«j.getID()-1») = std::atof(argv[«j.getID() + robot.joints.size + robot.joints.size»]);
            «ENDFOR»

            «className(robot)» foo;

            foo.id(q,qd,qdd,tau);
            std::cout << tau << std::endl;
            return 0;
        }'''


    def inertiaMatrixHeader(Robot robot)'''
        #ifndef IIT_«Names$Files$RBD::inertiaMatrixHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files$RBD::inertiaMatrixHeader(robot).toUpperCase()»_H_

        #include <iit/rbd/JStateDependentMatrix>
        #include <iit/robots/«Names$Files::mainHeader(robot)».h>

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        typedef «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «robot.joints.size», «robot.joints.size»> «Names$Types::jspaceMLocal»;

        extern «Names$Types::jspaceMLocal» jspaceM;
        void initJointSpaceInertiaMatrix();
        namespace «Names$Namespaces::internal» {
            void jspaceM_setJointStatus(const «Names$Types::jointState»& «Common::jointsStateVarName», «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState», «robot.joints.size», «robot.joints.size»>& mx»);
        }

        }
        }
        #endif
        '''

    def inertiaMatrixSource(Robot robot) '''
        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"

        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::transforms6D»;

        «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jspaceMLocal» «Names$Namespaces$Qualifiers::robot(robot)»::jspaceM(«Names$Namespaces::internal»::jspaceM_setJointStatus);

        «Names$Namespaces$Qualifiers::robot(robot)»::initJointSpaceInertiaMatrix() {
            jspaceM.setZero();
        }

        «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::internal»::jspaceM_setJointStatus(const «Names$Types::jointState»& «Common::jointsStateVarName», «Names$Types::jstateDependentMatrix()»<«Names$Types::jointState» jState, «robot.joints.size», «robot.joints.size»>& M») {

        }
    '''

}
