package iit.dsl.generator.cpp.dynamics

import iit.dsl.generator.cpp.Names

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.Robot

import java.util.List
import org.eclipse.xtend2.lib.StringConcatenation
import iit.dsl.generator.cpp.kinematics.Transforms

class JointsSpaceInertia {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def inertiaMatrixHeader(Robot robot)'''
        #ifndef IIT_«Names$Files$RBD::inertiaMatrixHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files$RBD::inertiaMatrixHeader(robot).toUpperCase()»_H_

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/JStateDependentMatrix.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$LinkInertias::header(robot)».h"

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {
        namespace «Names$Namespaces::dynamics» {
        «val className = Names$Types::jspaceMLocal»
        /**
         * The type of the Joint Space Inertia Matrix (JSIM) of the robot «robot.name».
         */
        class «className» : public «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «robot.joints.size», «robot.joints.size»>
        {
            private:
                typedef «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «robot.joints.size», «robot.joints.size»> Base;
            public:
                typedef Base::Scalar Scalar;
                typedef Base::Index Index;
                typedef Eigen::Matrix<double, «robot.joints.size», «robot.joints.size»> MatrixType;
            public:
                «className»();
                ~«className»() {}

                const «Names$Types::jspaceMLocal»& operator()(const «Names$Types::jointState»&);

                //need to redeclare these because previous overloading hides the base class versions
                const Scalar& operator()(Index row, Index col) const;
                Scalar& operator()(Index row, Index col);

                /**
                 * Computes and saves the matrix L of the L^T L factorization of this JSIM.
                 */
                void computeL();
                /**
                 * Computes and saves the inverse of this JSIM.
                 * This function assumes that computeL() has been called already, since it
                 * uses L to compute the inverse. The algorithm takes advantage of the branch
                 * induces sparsity of the robot, if any.
                 */
                void computeInverse();
                /**
                 * Returns an unmodifiable reference to the matrix L. See also computeL()
                 */
                const MatrixType& getL() const;
                /**
                 * Returns an unmodifiable reference to the inverse of this JSIM
                 */
                const MatrixType& getInverse() const;
            protected:
                /**
                 * Computes and saves the inverse of the matrix L. See also computeL()
                 */
                void computeLInverse();
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


        inline const «className»::Scalar&
        «className»::operator()(Index row, Index col) const {
            return this->Base::operator() (row,col);
        }

        inline «className»::Scalar&
        «className»::operator()(Index row, Index col) {
            return this->Base::operator() (row,col);
        }

        inline const «className»::MatrixType& «className»::getL() const {
            return L;
        }

        inline const «className»::MatrixType& «className»::getInverse() const {
            return inverse;
        }

        // The joint space inertia matrix of this robot
        extern «Names$Types::jspaceMLocal» «Names$GlobalVars::jsInertia»;


        }
        }
        }
        #endif
        '''

    def inertiaMatrixSource(Robot robot) '''
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        //using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::transforms6D»;

        «val robo_ns_qualifier = Names$Namespaces$Qualifiers::robot(robot)»
        «val robodyn_ns_qualifier = robo_ns_qualifier + "::" + Names$Namespaces::dynamics»
        «val classname = Names$Types::jspaceMLocal»
        «val class_qualifier = robodyn_ns_qualifier + "::" + classname»

        // The joint space inertia matrix of this robot
        «robodyn_ns_qualifier»::«Names$Types::jspaceMLocal» «robodyn_ns_qualifier»::«Names$GlobalVars::jsInertia»;

        «val endLinks = chainEndLinks(robot)»
        //Implementation of default constructor
        «class_qualifier»::«classname»()
        «IF endLinks.size() > 0»: «inertiaCompositeName(endLinks.get(0))»(«inertiaName(endLinks.get(0))»)
        «FOR l : endLinks.drop(1)», «inertiaCompositeName(l)»(«inertiaName(l)»)«ENDFOR»
        «ENDIF»
        {
            //Make sure all the transforms for this robot are initialized
            «robo_ns_qualifier»::«Names$Namespaces::transforms6D»::initAll();
            «robo_ns_qualifier»::«Names$Namespaces::transforms6D»::«Names$Namespaces::T6D_force»::initAll();

            this->setZero();
            // Initialize the 6D inertia tensor of each body of the robot
            «LinkInertias::className(robot)» linkInertias;
            «FOR l : robot.links»
                «inertiaName(l)» = linkInertias.«LinkInertias::tensorGetterName(l)»();
            «ENDFOR»
        }

        #define DATA operator()

        const «class_qualifier»& «class_qualifier»::operator()(const «Names$Types::jointState»& state) {
            «val sortedLinks = robot.links.sortBy(link | getID(link)).reverse»
            static «Names$Namespaces::rbd»::ForceVector F;

            // Precomputes only once the coordinate transforms:
            «FOR l : sortedLinks»
                «val parent = l.parent»
                «IF !(parent.equals(robot.base))»
                    «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(parent, l)»(state);
                    «Transforms::child_X_parent__mxName(parent, l)»(state);
                «ENDIF»
            «ENDFOR»

            // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
            «FOR l : sortedLinks»

                // Link «l.name» //

                «val parent = l.parent»
                «IF !(parent.equals(robot.base))»
                    «inertiaCompositeName(parent)» = «inertiaName(parent)» + «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(parent, l)» * «inertiaCompositeName(l)» * «Transforms::child_X_parent__mxName(parent, l)»;
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

        void «class_qualifier»::computeL() {
            «LTLfactorization(robot)»
        }

        void «class_qualifier»::computeInverse() {
            computeLInverse();

            «Minverse(robot)»
        }

        void «class_qualifier»::computeLInverse() {
            //assumes L has been computed already
            «Linverse(robot)»
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
                F = «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(parent, link)» * F;
                DATA(«rowIndex», «parentJ.ID-1») = F.transpose().«IF parentJ instanceof PrismaticJoint»col(5)«ELSE»col(2)«ENDIF»(0,0);
                DATA(«parentJ.ID-1», «rowIndex») = DATA(«rowIndex», «parentJ.ID-1»); //the matrix is symmetric
                ''');
            }
        }
        return strBuff;
    }

    def main_test(Robot robot) '''
        «val jsim = Names$GlobalVars::jsInertia»
        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::dynamics»;
        using namespace «Names$Namespaces$Qualifiers::iit_rbd»;

        int main(int argc, char** argv) {
            if(argc < «robot.joints.size + 1») {
                cerr << "Please specify a float value for each joint of the robot" << endl;
                return -1;
            }
            «Names$Types::jointState» q;
            «FOR Joint j : robot.joints»
                q(«j.getID()-1»)   = std::atof(argv[«j.getID()»]);
            «ENDFOR»

            «jsim»(q);
            «jsim».computeL();
            «jsim».computeInverse();
            cout << "Joint Space Inertia Matrix M" << endl << «jsim» << endl << endl;
            cout << "L" << endl << «jsim».getL() << endl;
            cout << "Inverse of M" << endl << «jsim».getInverse() << endl << endl;
            return 0;
        }'''

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
                    «robotNS»::«Names$GlobalVars::jsInertia».computeL();
                    «robotNS»::«Names$GlobalVars::jsInertia».computeInverse();
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

                // Get the chain containing all the joints from the base to joint 'jo_j' itself
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