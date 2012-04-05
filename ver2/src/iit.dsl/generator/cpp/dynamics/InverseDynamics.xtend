package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.FloatingRobotBase
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.FixedRobotBase
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Link

import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Utils

import java.util.List
import iit.dsl.generator.cpp.kinematics.Transforms

class InverseDynamics {
	extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def static className(Robot r) '''InverseDynamics'''

    def mainHeader(Robot robot) '''
        «val jState = Names$Types::jointState»
        #ifndef IIT_RBD_«Names$Files$RBD::header(robot).toUpperCase()»_DYNAMICS_H_
        #define IIT_RBD_«Names$Files$RBD::header(robot).toUpperCase()»_DYNAMICS_H_

        #include <Eigen/Dense>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/utils.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"

        namespace iit {
        namespace «Names$Namespaces::rob(robot)» {

        typedef iit::rbd::InertiaMatrixDense InertiaMatrix;
        typedef iit::rbd::SparseColumnd JointVelocity;


        class «className(robot)» {
        public:
            «className(robot)»();
            /** The inverse dynamics routine for this robot */
            void id(const «jState»& q, const «jState»& qd, const «jState»& qdd, «jState»& torques);
            /** \name Gravity terms
             *  The torques acting on the joints due to gravity, for a specific configuration.
             *  In order to do gravity compensation, torques with the opposite sign should be applied.
             */ ///@{
            void G_terms(const «jState»& q, «jState»& torques);
            void G_terms(«jState»& torques);
            ///@}
            /** \name Centrifugal and Coriolis terms
             * The torques acting on the joints due to centrifugal and Coriolis effects, for a
             * specific configuration.
             */ ///@{
            void C_terms(const «jState»& q, const «jState»& qd, «jState»& torques);
            void C_terms(const «jState»& qd, «jState»& torques);
            ///@}
            /** Updates the kinematics transforms used by the inverse dynamics routine. */
            void setJointStatus(const «jState»& q) const;
        public:
            iit::rbd::SparseColumnd gravity;

        private:
            iit::rbd::Matrix66d spareMx; // support variable
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
            «val jState = Names$Types::jointState»
            «val sortedLinks = robot.abstractLinks.sortBy(link | getID(link))»
            #include "«Names$Files$RBD::header(robot)».h"
            #ifndef EIGEN_NO_DEBUG
                #include <iostream>
            #endif
            «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot)»
            using namespace std;
            using namespace iit::rbd;
            using namespace «nsqualifier»;

            «nsqualifier»::«className(robot)»::«className(robot)»() {
            #ifndef EIGEN_NO_DEBUG
                std::cout << "Robot «robot.name», «className(robot)»::«className(robot)»()" << std::endl;
                std::cout << "Compiled with Eigen debug active" << std::endl;
            #endif
                gravity.resize(6);
                gravity.insert(5) = 9.81;

                «FOR l : robot.links»
                    «velocityName(l)».setZero();
                «ENDFOR»
                «FOR l : robot.links»
                    «inertiaMxName(l)».fill(«l.inertiaParams.mass», Vector3d(«l.inertiaParams.com.x.str»,«l.inertiaParams.com.y.str»,«l.inertiaParams.com.z.str»),
                        Utils::buildInertiaTensor(«l.inertiaParams.ix»,«l.inertiaParams.iy»,«l.inertiaParams.iz»,«l.inertiaParams.ixy»,«l.inertiaParams.ixz»,«l.inertiaParams.iyz»));
                «ENDFOR»

                «Names$Namespaces::transforms6D»::initAll(); // initializes coordinates transforms
            }

            void «nsqualifier»::«className(robot)»::id(const «jState»& q, const «jState»& qd, const «jState»& qdd, «jState»& torques) {
                «FOR AbstractLink l : sortedLinks»
                    «inverseDynamicsPass1(l)»

                «ENDFOR»
                «FOR AbstractLink l : sortedLinks.reverse()»
                    «inverseDynamicsPass2(l)»

                «ENDFOR»
            }


            «methodsDefinitions(robot)»
            '''

    def private dispatch subspaceIndex(PrismaticJoint j) {
        return 5;
    }
    def private dispatch subspaceIndex(RevoluteJoint j) {
        return 2;
    }

    def private dispatch inverseDynamicsPass1(FixedRobotBase base)''''''
    def private dispatch inverseDynamicsPass1(FloatingRobotBase base)'''// **** WARNING: floating base is not supported yet //TODO'''
    def private dispatch inverseDynamicsPass1(Link l)'''
           // First pass, link '«l.name»'
           «val parentLink = l.parent»
           «val myJoint = l.connectingJoint»
           «val velocity = l.velocityName»
           «val acceler = l.accelerationName»
           «val transform = Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(parentLink, l)»
           «transform»(q); // updates the transform with the joint status
           «val jid = myJoint.arrayIdx»
           «val subspaceIdx = myJoint.subspaceIndex»
           «IF parentLink.getID() == 0»
               «velocity»(«subspaceIdx») = qd(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot
               «acceler» = («transform» * gravity);
               «acceler»(«subspaceIdx») += qdd(«jid»);
               Utils::fillAsForceCrossProductMx(«velocity», spareMx); // this could be optimized..
               «forceName(l)» = «inertiaMxName(l)» * «acceler» + ((spareMx * «inertiaMxName(l)»).col(«subspaceIdx») * qd(«jid»));
           «ELSE»
               «velocity» = («transform» * «parentLink.velocityName»);
               «velocity»(«subspaceIdx») += qd(«jid»);

               Utils::fillAsMotionCrossProductMx(«velocity», spareMx); // this could be optimized..

               «acceler» = («transform» * «parentLink.accelerationName») + (spareMx.col(«subspaceIdx») * qd(«jid»));
               «acceler»(«subspaceIdx») += qdd(«jid»);

               «forceName(l)» = «inertiaMxName(l)» * «acceler» + (-spareMx.transpose() * «inertiaMxName(l)» * «velocity»);
           «ENDIF»
           '''

    def private dispatch inverseDynamicsPass2(FixedRobotBase base)''''''
    def private dispatch inverseDynamicsPass2(FloatingRobotBase base)'''// **** WARNING: floating base is not supported yet //TODO'''
    def private dispatch inverseDynamicsPass2(Link l)'''
        // Second pass, link '«l.name»'
        «val parentLink = l.parent»
        «val joint      = l.connectingJoint»
        torques(«joint.arrayIdx») = «l.forceName»(«joint.subspaceIndex»);
        «IF parentLink.ID != 0»
            «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«Transforms::child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
        «ENDIF»
        '''

    def private C_terms__docs_parameters() '''
        /**
         * \param q the joint status vector that specifies the robot configuration
         * \param qd the joint velocities vector
         * \param torques will be filled with the (generalized) forces acting on the joints
         */
        '''

    def methodsDefinitions(Robot robot) '''
        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot)»
        «val jState = Names$Types::jointState»
        «val sortedLinks = robot.links.sortBy(link | getID(link))»
        «val updateTransformsCode = setJointStatusCode(sortedLinks)»
        /**
         * \param q the joint status vector that specifies the robot configuration
         * \param torques will be filled with the (generalized) forces due to the gravity
         */
        void «nsqualifier»::«className(robot)»::G_terms(const «jState»& q, «jState»& torques) {
            «updateTransformsCode»
            G_terms(torques);
        }

        /**
         * This version of the function uses the current configuration of the robot.
         * It is provided for the sake of efficiency, in case the kinematics transforms
         * of the robot have already been updated elsewhere with the most recent
         * configuration, so that it is useless to compute them again here.
         * \param torques will be filled with the (generalized) forces due to the gravity
         */
        void «nsqualifier»::«className(robot)»::G_terms(«jState»& torques) {
            «FOR Link l : sortedLinks»
                // Link '«l.name»'
                «val parentLink = l.parent»
                «val acceler    = l.accelerationName»
                «val transform  = Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(parentLink, l)»
                «IF parentLink.getID() == 0»
                    «acceler» = «transform».col(«Utils::Z_L») * ( - «Names$Namespaces::enclosing»::«Names$Namespaces::rbd»::g);
                    «forceName(l)» = «inertiaMxName(l)» * «acceler»;
                «ELSE»
                    «acceler» = («transform» * «parentLink.accelerationName»);
                    «forceName(l)» = «inertiaMxName(l)» * «acceler»;
                «ENDIF»
            «ENDFOR»
            «FOR Link l : sortedLinks.reverseView()»
                // Link '«l.name»'
                «val parentLink = l.parent»
                «val joint      = l.connectingJoint»
                torques(«joint.arrayIdx») = «l.forceName»(«joint.subspaceIndex»);
                «IF parentLink.ID != 0»
                    «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«Transforms::child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
                «ENDIF»
            «ENDFOR»
        }

        «C_terms__docs_parameters»
        void «nsqualifier»::«className(robot)»::C_terms(const «jState»& q, const «jState»& qd, «jState»& torques) {
            «updateTransformsCode»
            C_terms(qd, torques);
        }
        «C_terms__docs_parameters»
        void «nsqualifier»::«className(robot)»::C_terms(const «jState»& qd, «jState»& torques) {
            «FOR Link l : sortedLinks»
                «val parentLink = l.parent»
                «val acceler    = l.accelerationName»
                «val myJoint    = l.connectingJoint»
                «val jid        = myJoint.arrayIdx»
                «val subspaceIdx= myJoint.subspaceIndex»
                «val velocity   = l.velocityName»
                «val transform  = Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(parentLink, l)»

                // Link '«l.name»'
                «IF parentLink.equals(robot.base)»
                    // velocity:
                    «velocity»(«subspaceIdx») = qd(«jid»);
                    // force:
                    Utils::fillAsForceCrossProductMx(«velocity», spareMx);
                    «forceName(l)» = (spareMx * «inertiaMxName(l)»).col(«subspaceIdx») * qd(«jid»);
                «ELSE»
                    // velocity:
                    «velocity» = («transform» * «parentLink.velocityName»);
                    «velocity»(«subspaceIdx») += qd(«jid»);
                    // acceleration and force:
                    Utils::fillAsMotionCrossProductMx(«velocity», spareMx);
                    «IF parentLink.parent.equals(robot.base)»
                        «acceler» = (spareMx.col(«subspaceIdx») * qd(«jid»));
                    «ELSE»
                        «acceler» = («transform» * «parentLink.accelerationName») + (spareMx.col(«subspaceIdx») * qd(«jid»));
                    «ENDIF»
                    «forceName(l)» = «inertiaMxName(l)» * «acceler» + (-spareMx.transpose() * «inertiaMxName(l)» * «velocity»);
                «ENDIF»
            «ENDFOR»
            «FOR Link l : sortedLinks.reverseView()»

                // Link '«l.name»'
                «val parentLink = l.parent»
                «val joint      = l.connectingJoint»
                torques(«joint.arrayIdx») = «l.forceName»(«joint.subspaceIndex»);
                «IF parentLink.ID != 0»
                    «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«Transforms::child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
                «ENDIF»
            «ENDFOR»
        }

        /**
         * \param q the joint status vector that specifies the robot configuration
         */
        void «nsqualifier»::«className(robot)»::setJointStatus(const «jState»& q) const {
            «updateTransformsCode»
        }
    '''

    def private setJointStatusCode(List<Link> sortedLinks) '''
        «FOR Link l : sortedLinks»
            «Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(l.parent, l)»(q);
        «ENDFOR»
    '''


    def main_benchmarkID(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::header(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static void fillState(«robotNS»::«Names$Types::jointState»& q, «robotNS»::«Names$Types::jointState»& qd, «robotNS»::«Names$Types::jointState»& qdd);
        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject);

        /* This main is supposed to be used to test the inverse dynamics routines */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the number of tests to perform" << endl;
                return -1;
            }

            //Make sure all the transforms for this robot are initialized
            «robotNS»::«Names$Namespaces::transforms6D»::initAll();
            «robotNS»::«Names$Namespaces::transforms6D»::«Names$Namespaces::T6D_force»::initAll();

            int numOfTests = std::atoi(argv[1]);
            double me[numOfTests];
            int iterations[numOfTests];

            double t0, duration, my_total;
            my_total = 0;

            «robotNS»::«Names$Types::jointState» q, qd, qdd, tau;
            «robotNS»::«className(robot)» myDynamics;

            int t=0,i=0;

            std::srand(std::time(NULL)); // initialize random number generator

            int numOfIterations = 1;
            for(t=0; t<numOfTests; t++) {
                my_total = 0;
                numOfIterations = numOfIterations * 10;
                iterations[t] = numOfIterations;

                for(i=0; i<numOfIterations; i++) {
                    fillState(q, qd, qdd);
                    t0 = std::clock();
                    myDynamics.id(q, qd, qdd, tau);
                    duration = std::clock() - t0;
                    my_total += duration;
                }
                me[t] = my_total/CLOCKS_PER_SEC;
            }

           matlabLog(numOfTests, iterations, me, "inv_dyn");

            for(t=0; t<numOfTests; t++) {
                cout << me[t] << endl;
            }

            return 0;
        }


        void fillState(«robotNS»::«Names$Types::jointState»& q, «robotNS»::«Names$Types::jointState»& qd, «robotNS»::«Names$Types::jointState»& qdd) {
            static const double max = 12.3;
            «FOR Joint j : robot.joints»
                q(«j.getID()-1»)   = ( ((double)std::rand()) / RAND_MAX) * max;
                qd(«j.getID()-1»)  = ( ((double)std::rand()) / RAND_MAX) * max;
                qdd(«j.getID()-1») = ( ((double)std::rand()) / RAND_MAX) * max;
            «ENDFOR»
        }

        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject) {
            «val prefix = robot.name.toLowerCase + "_test"»
            std::string fileName = "«robot.name»_" + subject + "_speed_test_data.m";
            ofstream out(fileName.c_str());
            out << "«prefix».robot       = '«robot.name»';" << std::endl;
            out << "«prefix».description = 'test of the speed of the calculation of: " << subject << "';" << std::endl;
            out << "«prefix».software    = 'code generated from the Kinematic DSL & Co.';" << std::endl;

            // Current date/time based on current system
            time_t now = std::time(0);
            tm* localtm = std::localtime(&now); // Convert now to tm struct for local timezone
            char timeStr[64];
            std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d  %X",localtm);
            out << "«prefix».date = '" << timeStr << "';" << std::endl;

            out << "«prefix».iterations = [";
            for(int t=0; t<numOfTests; t++) {
                out << " " << iterations[t];
            }
            out << "];" << endl;
            out << "«prefix».times = [";
            for(int t=0; t<numOfTests; t++) {
                out << " " << tests[t];
            }
            out << "];" << endl;
            out.close();
        }
        '''






    def testMain(Robot robot) '''
        #include <cmath>
        #include <iostream>

        #include "«Names$Files$RBD::header(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace iit::rbd;

        int main(int argc, char** argv) {
            «Names$Types::jointState» q, qd, qdd, tau, tau2;
            «FOR Joint j : robot.joints»
            q(«j.getID()-1»)   = std::atof(argv[«j.getID()»]);
            qd(«j.getID()-1»)  = std::atof(argv[«j.getID() + robot.joints.size»]);
            qdd(«j.getID()-1») = std::atof(argv[«j.getID() + robot.joints.size + robot.joints.size»]);
            «ENDFOR»

            «className(robot)» foo;

            foo.id(q,qd,qdd,tau);
            std::cout << "Full inverse dynamics terms:" << std::endl << tau << std::endl;
            foo.G_terms(q, tau);
            std::cout << std::endl << "Gravity terms:" << std::endl << tau << std::endl;
            foo.C_terms(q, qd, tau2);
            std::cout << std::endl << "Centrifugal/Coriolis terms:" << std::endl << tau2 << std::endl;

            std::cout << std::endl << "-G + C:" << std::endl << -tau + tau2 << std::endl;
            return 0;
        }'''
}