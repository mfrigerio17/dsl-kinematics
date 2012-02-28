package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.FixedRobotBase
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.Link

import iit.dsl.kinDsl.RobotBase
import java.util.List
import org.eclipse.xtend2.lib.StringConcatenation

class RigidBodyDynamics {

    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

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

            void «nsqualifier»::«className(robot)»::id(const JointState& q, const JointState& qd, const JointState& qdd, JointState& torques) {
                «val sortedLinks = robot.abstractLinks.sortBy(link | getID(link))»
                «FOR AbstractLink l : sortedLinks»
                    «inverseDynamicsPass1(l)»

                «ENDFOR»
                «FOR AbstractLink l : sortedLinks.reverse()»
                    «inverseDynamicsPass2(l)»

                «ENDFOR»
            } '''

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
           «val transform = Names$Namespaces::transforms6D + "::" + child_X_parent__mxName(parentLink, l)»
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
            «parentLink.forceName» = «parentLink.forceName» + «Names$Namespaces::transforms6D»::«child_X_parent__mxName(parentLink, l)».transpose() * «l.forceName»;
        «ENDIF»
        '''

    def private child_X_parent__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«child.name»_X_fr_«parent.name»'''
    def private parent_X_child__mxName(AbstractLink parent, AbstractLink child) '''
        fr_«parent.name»_X_fr_«child.name»'''

    def testMain(Robot robot) '''
        #include <cmath>
        #include <iostream>

        #include "«Names$Files$RBD::header(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
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

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/JStateDependentMatrix.h>
        #include <iit/robots/«Names$Files::mainHeader(robot)».h>
        #include <iit/robots/«Names$Files$RBD::header(robot)».h>

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
            public:
                «Names$Types::jspaceMLocal»();
                ~«Names$Types::jspaceMLocal»() {}

                const «Names$Types::jspaceMLocal»& operator()(const «Names$Types::jointState»&);

                //need to redeclare these because previous overloading hides the base class versions
                const Scalar& operator()(Index row, Index col) const;
                Scalar& operator()(Index row, Index col);

                const Eigen::Matrix<double, «robot.joints.size», «robot.joints.size»>& getL();
            private:
                // The composite inertia matrix for each link
                «FOR l : robot.links»
                    InertiaMatrix «inertiaCompositeName(l)»;
                «ENDFOR»

                Eigen::Matrix<double, «robot.joints.size», «robot.joints.size»> L;
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
        #include <iit/robots/«Names$Files::transformsHeader(robot)».h>
        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        //using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::transforms6D»;

        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot)»
        «val classname = Names$Types::jspaceMLocal»

        // The joint space inertia matrix of this robot
        «nsqualifier»::«Names$Types::jspaceMLocal» «nsqualifier»::«Names$GlobalVars::jsInertia»;

        //Implementation of default constructor
        «nsqualifier»::«classname»::«classname»() {
            //Make sure all the transforms for this robot are initialized
            «nsqualifier»::«Names$Namespaces::transforms6D»::initAll();
            «nsqualifier»::«Names$Namespaces::transforms6D»::«Names$Namespaces::T6D_force»::initAll();

            this->setZero();
            «FOR l : robot.links»
                «inertiaCompositeName(l)».fill(«l.inertiaParams.mass», «Names$Namespaces::rbd»::Vector3d(«l.inertiaParams.com.x.str»,«l.inertiaParams.com.y.str»,«l.inertiaParams.com.z.str»),
                        «Names$Namespaces::rbd»::Utils::buildInertiaTensor(«l.inertiaParams.ix»,«l.inertiaParams.iy»,«l.inertiaParams.iz»,«l.inertiaParams.ixy»,«l.inertiaParams.ixz»,«l.inertiaParams.iyz»));
            «ENDFOR»
        }

        #define DATA operator()

        const «nsqualifier»::«classname»& «nsqualifier»::«classname»::operator()(const «Names$Types::jointState»& state) {
            «val sortedLinks = robot.links.sortBy(link | getID(link)).reverse»
            static «Names$Namespaces::rbd»::ForceVector F;

            // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
            «FOR l : sortedLinks»


                // Link «l.name» //

                «val parent = l.parent»
                «IF !(parent instanceof RobotBase)»
                    «inertiaCompositeName(parent)» = «inertiaCompositeName(parent)» + «Names$Namespaces::T6D_force»::«parent_X_child__mxName(parent, l)»(state) * «inertiaCompositeName(l)» * «child_X_parent__mxName(parent, l)»(state);
                «ENDIF»

                «val linkJoint = getJoint(parent, l)»
                «IF linkJoint instanceof PrismaticJoint»
                    F = «inertiaCompositeName(l)».col(5); // multiplication by the joint subspace matrix, assuming 1 DoF joint
                    DATA(«linkJoint.ID-1», «linkJoint.ID-1») = F.row(2)(0,0);
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

        const Eigen::Matrix<double, «robot.joints.size», «robot.joints.size»>& «nsqualifier»::«classname»::getL() {
            «LTLfactorization(robot)»
            return L;
        }
    '''

    def private inertiaCompositeName(AbstractLink l) '''Ic_«l.name»'''

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
                F = «Names$Namespaces::T6D_force»::«parent_X_child__mxName(parent, link)»(state) * F;
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

        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static void fillState(«robotNS»::«Names$Types::jointState»& q);

        /* This main is supposed to be used to test the joint space inertia matrix routines */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the number of tests to perform" << endl;
                return -1;
            }
            int numOfTests = std::atoi(argv[1]);

            «robotNS»::«Names$Types::jointState» q;

            std::srand(std::time(NULL)); // initialize random number generator
            fillState(q);

            double t0, duration, total;
            total = 0;
            int numOfIterations = 0;
            double tests[numOfTests];
            for(int t=0; t<numOfTests; t++) {
                total = 0;
                numOfIterations = std::pow(10,t+1);

                for(int i=0; i<numOfIterations; i++) {
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

            return 0;
        }


        void fillState(«robotNS»::«Names$Types::jointState»& q) {
            static const double max = 12.3;
            «FOR Joint j : robot.joints»
                q(«j.getID()-1») = ( ((double)std::rand()) / RAND_MAX) * max;
            «ENDFOR»
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
                «FOR ancestor2 : chainToBase»
                    «val j = ancestor2.connectingJoint.getID-1»
                    L(«i», «j») = L(«i», «j») - L(«row», «i») * L(«row», «j»);
                «ENDFOR»
            «ENDFOR»

        «ENDFOR»
    '''

}
