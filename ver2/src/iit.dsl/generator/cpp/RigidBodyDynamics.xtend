package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.FixedRobotBase
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Link

import java.util.List
import org.eclipse.xtend2.lib.StringConcatenation
import iit.dsl.kinDsl.FloatingRobotBase

class RigidBodyDynamics {

    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def static className(Robot r) '''Dynamics'''

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
           «val transform = Names$Namespaces::transforms6D + "::" + child_X_parent__mxName(parentLink, l)»
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
            «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
        «ENDIF»
        '''

    def private child_X_parent__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«child.name»_X_fr_«parent.name»'''
    def private parent_X_child__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«parent.name»_X_fr_«child.name»'''

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
                «val transform  = Names$Namespaces::transforms6D + "::" + child_X_parent__mxName(parentLink, l)»
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
                    «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
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
                «val transform  = Names$Namespaces::transforms6D + "::" + child_X_parent__mxName(parentLink, l)»

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
                    «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
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
            «Names$Namespaces::transforms6D + "::" + child_X_parent__mxName(l.parent, l)»(q);
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


    def inertiaMatrixHeader(Robot robot)'''
        #ifndef IIT_«Names$Files$RBD::inertiaMatrixHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files$RBD::inertiaMatrixHeader(robot).toUpperCase()»_H_

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/JStateDependentMatrix.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::header(robot)».h"

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        //typedef «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «robot.joints.size», «robot.joints.size»> «Names$Types::jspaceMLocal»;

        class «Names$Types::jspaceMLocal» : public «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «robot.joints.size», «robot.joints.size»>
        {
            private:
                typedef «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «robot.joints.size», «robot.joints.size»> Base;
            public:
                typedef Base::Scalar Scalar;
                typedef Base::Index Index;
                typedef Eigen::Matrix<double, «robot.joints.size», «robot.joints.size»> MatrixType;
            public:
                «Names$Types::jspaceMLocal»();
                ~«Names$Types::jspaceMLocal»() {}

                const «Names$Types::jspaceMLocal»& operator()(const «Names$Types::jointState»&);

                //need to redeclare these because previous overloading hides the base class versions
                const Scalar& operator()(Index row, Index col) const;
                Scalar& operator()(Index row, Index col);

                const MatrixType& getL();
                const MatrixType& getLinv();
                const MatrixType& getInv();
            private:
                // The inertia tensor of each link
                «FOR l : robot.links»
                    InertiaMatrix «inertiaName(l)»;
                «ENDFOR»
                // The composite-inertia tensor for each link
                «FOR l : robot.links»
                    «IF l.childrenList.children.empty»
                        const InertiaMatrix& «inertiaCompositeName(l)»;
                    «ELSE»
                        InertiaMatrix «inertiaCompositeName(l)»;
                    «ENDIF»
                «ENDFOR»

                MatrixType L;
                MatrixType Linv;
                MatrixType inverse;
        };


        inline const «Names$Types::jspaceMLocal»::Scalar&
        «Names$Types::jspaceMLocal»::operator()(Index row, Index col) const {
            return this->Base::operator() (row,col);
        }

        inline «Names$Types::jspaceMLocal»::Scalar&
        «Names$Types::jspaceMLocal»::operator()(Index row, Index col) {
            return this->Base::operator() (row,col);
        }

        // The joint space inertia matrix of this robot
        extern «Names$Types::jspaceMLocal» «Names$GlobalVars::jsInertia»;


        }
        }
        #endif
        '''

    def inertiaMatrixSource(Robot robot) '''
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        //using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::transforms6D»;

        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot)»
        «val classname = Names$Types::jspaceMLocal»

        // The joint space inertia matrix of this robot
        «nsqualifier»::«Names$Types::jspaceMLocal» «nsqualifier»::«Names$GlobalVars::jsInertia»;

        «val endLinks = chainEndLinks(robot)»
        //Implementation of default constructor
        «nsqualifier»::«classname»::«classname»()
        «IF endLinks.size() > 0»: «inertiaCompositeName(endLinks.get(0))»(«inertiaName(endLinks.get(0))»)
        «FOR l : endLinks.drop(1)», «inertiaCompositeName(l)»(«inertiaName(l)»)«ENDFOR»
        «ENDIF»
        {
            //Make sure all the transforms for this robot are initialized
            «nsqualifier»::«Names$Namespaces::transforms6D»::initAll();
            «nsqualifier»::«Names$Namespaces::transforms6D»::«Names$Namespaces::T6D_force»::initAll();

            this->setZero();
            // Initialize the 6D inertia tensor of each body of the robot
            «FOR l : robot.links»
                «inertiaName(l)».fill(«l.inertiaParams.mass», «Names$Namespaces::rbd»::Vector3d(«l.inertiaParams.com.x.str»,«l.inertiaParams.com.y.str»,«l.inertiaParams.com.z.str»),
                        «Names$Namespaces::rbd»::Utils::buildInertiaTensor(«l.inertiaParams.ix»,«l.inertiaParams.iy»,«l.inertiaParams.iz»,«l.inertiaParams.ixy»,«l.inertiaParams.ixz»,«l.inertiaParams.iyz»));
            «ENDFOR»
        }

        #define DATA operator()

        const «nsqualifier»::«classname»& «nsqualifier»::«classname»::operator()(const «Names$Types::jointState»& state) {
            «val sortedLinks = robot.links.sortBy(link | getID(link)).reverse»
            static «Names$Namespaces::rbd»::ForceVector F;

            // Precomputes only once the coordinate transforms:
            «FOR l : sortedLinks»
                «val parent = l.parent»
                «IF !(parent.equals(robot.base))»
                    «Names$Namespaces::T6D_force»::«parent_X_child__mxName(parent, l)»(state);
                    «child_X_parent__mxName(parent, l)»(state);
                «ENDIF»
            «ENDFOR»

            // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
            «FOR l : sortedLinks»

                // Link «l.name» //

                «val parent = l.parent»
                «IF !(parent.equals(robot.base))»
                    «inertiaCompositeName(parent)» = «inertiaName(parent)» + «Names$Namespaces::T6D_force»::«parent_X_child__mxName(parent, l)» * «inertiaCompositeName(l)» * «child_X_parent__mxName(parent, l)»;
                «ENDIF»

                «val linkJoint = getJoint(parent, l)»
                «IF linkJoint instanceof PrismaticJoint»
                    F = «inertiaCompositeName(l)».col(5); // multiplication by the joint subspace matrix, assuming 1 DoF joint
                    DATA(«linkJoint.ID-1», «linkJoint.ID-1») = F.row(5)(0,0);
                «ELSE»
                    F = «inertiaCompositeName(l)».col(2); // multiplication by the joint subspace matrix, assuming 1 DoF joint
                    DATA(«linkJoint.ID-1», «linkJoint.ID-1») = F.row(2)(0,0);
                «ENDIF»

                «val chain = chainToBase(l)»
                «inertiaMatrix_lastStep(chain, linkJoint.ID-1)»
            «ENDFOR»

            return *this;
        }

        #undef DATA

        const «nsqualifier»::«classname»::MatrixType& «nsqualifier»::«classname»::getL() {
            «LTLfactorization(robot)»
            return L;
        }

        const «nsqualifier»::«classname»::MatrixType& «nsqualifier»::«classname»::getLinv() {
            //assumes L has been compute already
            «Linverse(robot)»
            return Linv;
        }

        const «nsqualifier»::«classname»::MatrixType& «nsqualifier»::«classname»::getInv() {
            //assumes Linv has been compute already
            «Minverse(robot)»
            return inverse;
        }
    '''

    def private inertiaCompositeName(AbstractLink l) '''Ic_«l.name»'''
    def private inertiaName(AbstractLink l) '''I_«l.name»'''

    /**
     * Constructs the chain (list) of links connecting the argument to the
     * robot base, except the base itself
     */
    def private List<AbstractLink> chainToBase(AbstractLink l) {
        val chain = buildChain(l, (l.eContainer() as Robot).base)
        chain.remove(chain.size() - 1) // removes the last element, which is the base
        return chain
    }

    def private inertiaMatrix_lastStep(List<AbstractLink> chainToBase, int rowIndex) {
        val strBuff = new StringConcatenation()

        var AbstractLink parent
        var Joint parentJ
        for( link : chainToBase ) {
            parent = link.parent
            if( ! parent.equals( (parent.eContainer() as Robot).base ) ) {
                parentJ = getConnectingJoint(parent);
                strBuff.append('''
                F = «Names$Namespaces::T6D_force»::«parent_X_child__mxName(parent, link)» * F;
                DATA(«rowIndex», «parentJ.ID-1») = F.transpose().«IF parentJ instanceof PrismaticJoint»col(5)«ELSE»col(2)«ENDIF»(0,0);
                DATA(«parentJ.ID-1», «rowIndex») = DATA(«rowIndex», «parentJ.ID-1»); //the matrix is symmetric
                ''');
            }
        }
        return strBuff;
    }

    def inertiaMatrixTestMain(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::jacobiansHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static void fillState(«robotNS»::«Names$Types::jointState»& q);
        // Speed test functions:
        static void speedTest_M(int numOfTests);  // 'M' is the joint space inertia matrix
        static void speedTest_inverseM(int numOfTests);
        static void speedTest_N(); // 'N' is the null space projector

        static void testInverse();

        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject);

        /* This main is supposed to be used to test the joint space inertia matrix routines */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the number of tests to perform" << endl;
                return -1;
            }
            int numOfTests = std::atoi(argv[1]);

            «robotNS»::«Names$Namespaces::jacobians»::initAll();
            std::srand(std::time(NULL)); // initialize random number generator

            //speedTest_M(numOfTests);
            //speedTest_inverseM(numOfTests);
            speedTest_N();
            //testInverse();

            return 0;
        }


        static void fillState(«robotNS»::«Names$Types::jointState»& q) {
            static const double max = 50;
            «FOR Joint j : robot.joints»
                q(«j.getID()-1») = ( ((double)std::rand()) / RAND_MAX) * max;
            «ENDFOR»
        }

        static void speedTest_M(int numOfTests) {
            «robotNS»::«Names$Types::jointState» q;
            std::srand(std::time(NULL)); // initialize random number generator

            double t0, duration, total;
            total = 0;
            int iterations[numOfTests+1];//use indexes 1..numOfTests
            double tests[numOfTests];
            iterations[0] = 1;
            for(int t=0; t<numOfTests; t++) {
                total = 0;
                iterations[t+1] = iterations[t] * 10;

                for(int i=0; i<iterations[t+1]; i++) {
                    fillState(q);

                    t0 = std::clock();
                    «robotNS»::«Names$GlobalVars::jsInertia»(q);// this is actually performing computations
                    duration = std::clock() - t0;
                    total += duration;

                }
                tests[t] = total / CLOCKS_PER_SEC;
            }

            for(int t=0; t<numOfTests; t++) {
                cout << tests[t] << endl;
            }

            matlabLog(numOfTests, iterations, tests, "M");
        }

        static void speedTest_inverseM(int numOfTests) {
            «robotNS»::«Names$Types::jointState» q;
            std::srand(std::time(NULL)); // initialize random number generator

            double t0, duration, total;
            total = 0;
            int iterations[numOfTests+1];//use indexes 1..numOfTests
            double tests[numOfTests];
            iterations[0] = 1;
            for(int t=0; t<numOfTests; t++) {
                total = 0;
                iterations[t+1] = iterations[t] * 10;

                for(int i=0; i<iterations[t+1]; i++) {
                    fillState(q);

                    t0 = std::clock();
                    «robotNS»::«Names$GlobalVars::jsInertia»(q);// this is actually performing computations
                    «robotNS»::«Names$GlobalVars::jsInertia».getL();
                    «robotNS»::«Names$GlobalVars::jsInertia».getLinv();
                    «robotNS»::«Names$GlobalVars::jsInertia».getInv();
                    duration = std::clock() - t0;
                    total += duration;

                }
                tests[t] = total / CLOCKS_PER_SEC;
            }

            for(int t=0; t<numOfTests; t++) {
                cout << tests[t] << endl;
            }

            matlabLog(numOfTests, iterations, tests, "M_inverse");
        }

        static void testInverse() {
            «val M = robotNS + "::" + Names$GlobalVars::jsInertia»
            «robotNS»::«Names$Types::jointState» q;
            fillState(q);

            cout << endl << "M:" << endl << «M»(q) << endl;
            cout << endl << "L:" << endl << «M».getL() << endl; // L gets computed

            const «robotNS»::«Names$Types::jspaceMLocal»::MatrixType& Linv = «M».getLinv(); // computes the inverse
            cout << endl << "L inverse:" << endl << Linv << endl;

            const «robotNS»::«Names$Types::jspaceMLocal»::MatrixType& Minv = «M».getInv();
            cout << endl << "M inverse:" << endl << Minv << endl;

            «robotNS»::«Names$Types::jspaceMLocal»::MatrixType id;
            id = «M» * Linv * Linv.transpose();
            std::cout << endl << "M * L^{-1} * L^{-T}:" << endl <<
                  (id.array().abs() < 1E-6).select(0, id) << std::endl;

            id = «M» * Minv;
            std::cout << endl << "M * M^{-1}:" << endl <<
                  (id.array().abs() < 1E-6).select(0, id) << std::endl;
        }

        static void speedTest_N() {
            «robotNS»::«Names$Types::jointState» q;

            Eigen::Matrix<double, «robot.joints.size», «robot.joints.size»> I;
            I.setIdentity();

            // convenient aliases
            «IF robot.name.equals("Fancy")»
                «robotNS»::«Names$Types::jacobianLocal»<«robot.joints.size»>& J = «robotNS»::«Names$Namespaces::jacobians»::fr_FancyBase_J_ee;
            «ELSE»
                «robotNS»::«Names$Types::jacobianLocal»<«robot.joints.size»> deleteMe(NULL);
                «robotNS»::«Names$Types::jacobianLocal»<«robot.joints.size»>& J = deleteMe; //«robotNS»::«Names$Namespaces::jacobians»:: ... //TODO fill!!! ******************
            «ENDIF»
            «robotNS»::«Names$Types::jspaceMLocal»& M = «robotNS»::«Names$GlobalVars::jsInertia»;
            const «robotNS»::«Names$Types::jspaceMLocal»::MatrixType& Minv = «robotNS»::«Names$GlobalVars::jsInertia».getInv();

            std::srand(std::time(NULL)); // initialize random number generator
            static const int numOfTests = 5;
            double t0, duration, total;
            total = 0;
            int iterations[numOfTests+1];//use indexes 1..numOfTests
            double tests[numOfTests];
            iterations[0] = 1;
            for(int t=0; t<numOfTests; t++) {
                total = 0;
                iterations[t+1] = iterations[t] * 10;

                for(int i=0; i<iterations[t+1]; i++) {
                    fillState(q);

                    t0 = std::clock();
                        M(q); // this is actually performing computations
                        J(q); // this is actually performing computations
                        M.getL();
                        M.getLinv();
                        M.getInv(); // this is actually updating the inverse 'Minv'
                    duration = std::clock() - t0;
                    total += duration;

                }
                tests[t] = total / CLOCKS_PER_SEC;
            }

            for(int t=0; t<numOfTests; t++) {
                cout << tests[t] << endl;
            }
            matlabLog(numOfTests, iterations, tests, "N_proj");

            //cout << J << endl << Minv << endl << endl;
            //cout << J * Minv * J.transpose() << endl;
            //cout << I - J.transpose() * (J * Minv * J.transpose()).inverse() * J*Minv << endl;
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
                out << " " << iterations[t+1];
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

    def LTLfactorization(Robot robot) '''
        L = «Names$GlobalVars::jsInertia».triangularView<Eigen::Lower>();
        «FOR Joint joint : robot.joints.reverseView»
            «val row = joint.getID()-1»
            // Joint «joint.name», index «row» :
            L(«row», «row») = std::sqrt(L(«row», «row»));
            «val chainToBase = joint.predecessorLink.chainToBase»
            «FOR ancestor : chainToBase»
                «val col = ancestor.connectingJoint.getID - 1»
                L(«row», «col») = L(«row», «col») / L(«row», «row»);
            «ENDFOR»
            «FOR ancestor : chainToBase»
                «val i = ancestor.connectingJoint.getID-1»
                «val secondChain = ancestor.chainToBase»
                «FOR ancestor2 : secondChain»
                    «val j = ancestor2.connectingJoint.getID-1»
                    L(«i», «j») = L(«i», «j») - L(«row», «i») * L(«row», «j»);
                «ENDFOR»
            «ENDFOR»

        «ENDFOR»
    '''

    def Linverse(Robot robot) '''
        «FOR jo : robot.joints»
            «val i = jo.ID-1»
            Linv(«i», «i») = 1 / L(«i», «i»);
        «ENDFOR»
        «FOR jo : robot.joints.drop(1)»
            «val link = jo.successorLink»
            «val chain = getChainJoints(buildChain(jo.predecessorLink, robot.base))»
            «val i = jo.ID-1»
            «FOR jo2 : chain»
                «val j = jo2.ID-1»
                «val subChain = getChainJoints(buildChain(jo2.successorLink, link))»
                Linv(«i», «j») = - Linv(«j», «j») * («FOR jo3 : subChain»«val k = jo3.ID-1»(Linv(«i», «k») * L(«k», «j»)) + «ENDFOR»0);
            «ENDFOR»
        «ENDFOR»
    '''

    def Minverse(Robot robot) {
        val strBuff = new StringConcatenation()
        for(jo_i : robot.joints) {
            val i = jo_i.arrayIdx
            // Get the chain containing all the joints in the chain up to the base, including 'jo_i' itself
            val chain = getChainJoints(buildChain(jo_i.successorLink, robot.base))
            for(jo_j : chain) {
                val j = jo_j.arrayIdx
                strBuff.append('''inverse(«i», «j») = ''')

                // Get the chain containing all the joints from the base to hoint 'jo_j' itself
                val chain2 = getChainJoints(buildChain(robot.base, jo_j.successorLink))
                for(jo_k : chain2) {
                    val k = jo_k.arrayIdx
                    strBuff.append(''' + (Linv(«i», «k») * Linv(«j», «k»))''')
                }
                strBuff.append(";\n")
                if(i != j) {
                    strBuff.append('''inverse(«j», «i») = inverse(«i», «j»);
                    ''')
                }
            }
        }
        return strBuff
    }

}
