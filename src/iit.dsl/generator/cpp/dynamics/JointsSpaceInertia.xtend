package iit.dsl.generator.cpp.dynamics

import java.util.List
import java.util.ArrayList

import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Common
import iit.dsl.generator.cpp.kinematics.Transforms
import iit.dsl.generator.common.TreeUtils

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.Robot


class JointsSpaceInertia {
    def inertiaMatrixHeader(Robot robot)'''
        «loadInfo(robot)»
        #ifndef IIT_«Names$Files$RBD::jsimHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files$RBD::jsimHeader(robot).toUpperCase()»_H_

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/JStateDependentMatrix.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»
        namespace «Names$Namespaces::dynamics» {
        /**
         * The type of the Joint Space Inertia Matrix (JSIM) of the robot «robot.name».
         */
        class «className» : public «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «dofs», «dofs»>
        {
            private:
                typedef «Names$Types::jstateDependentMatrix()»<«Names$Namespaces$Qualifiers::robot(robot)»::«Names$Types::jointState», «dofs», «dofs»> Base;
            public:
                typedef Base::Scalar Scalar;
                typedef Base::Index Index;
                typedef Eigen::Matrix<double,«dofs»,«dofs»> MatrixType;
                «val typename_blockF = "BlockF_t"»
                «val typename_blockFixedBase = "BlockFixedBase_t"»
                «IF floatingBase»
                    /** The type of the F sub-block of the floating-base JSIM */
                    typedef const Eigen::Block<const MatrixType,6,«jointDOFs»> «typename_blockF»;
                    /** The type of the fixed-base sub-block of the JSIM */
                    typedef const Eigen::Block<const MatrixType,«jointDOFs»,«jointDOFs»> «typename_blockFixedBase»;
                «ENDIF»
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
                 * induced sparsity of the robot, if any.
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

                «IF floatingBase»
                    /**
                     * The spatial composite-inertia tensor of the robot base,
                     * ie the inertia of the whole robot for the current configuration.
                     * According to the convention of this class about the layout of the
                     * floating-base JSIM, this tensor is the 6x6 upper left corner of
                     * the JSIM itself.
                     * \return the 6x6 InertiaMatrix that correspond to the spatial inertia
                     *   tensor of the whole robot, according to the last joints configuration
                     *   used to update this JSIM
                     */
                    const InertiaMatrix& getWholeBodyInertia() const;
                    /**
                     * The matrix that maps accelerations in the actual joints of the robot
                     * to the spatial force acting on the floating-base of the robot.
                     * This matrix is the F sub-block of the JSIM in Featherstone's notation.
                     * \return the 6x«jointDOFs» upper right block of this JSIM
                     */
                    const «typename_blockF» getF() const;
                    /**
                     * The submatrix of this JSIM related only to the actual joints of the
                     * robot (as for a fixed-base robot).
                     * This matrix is the H sub-block of the JSIM in Featherstone's notation.
                     * \return the «jointDOFs»x«jointDOFs» lower right block of this JSIM,
                     *   which correspond to the fixed-base JSIM
                     */
                    const «typename_blockFixedBase» getFixedBaseBlock() const;
                «ENDIF»
            protected:
                /**
                 * Computes and saves the inverse of the matrix L. See also computeL()
                 */
                void computeLInverse();
            private:
                // The composite-inertia tensor for each link
                «FOR l : links»
                    InertiaMatrix «inertiaCompositeName(l)»;
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

        «IF floatingBase»
            inline const InertiaMatrix& «className»::getWholeBodyInertia() const {
                return «inertiaCompositeName(robot.base)»;
            }

            inline const «className»::«typename_blockF» «className»::getF() const {
                return block<6,«jointDOFs»>(0,6);
            }

            inline const «className»::«typename_blockFixedBase» «className»::getFixedBaseBlock() const{
                return block<«jointDOFs»,«jointDOFs»>(6,6);
            }
        «ENDIF»

        // The joint space inertia matrix of this robot
        extern «Names$Types::jspaceMLocal» «Names$GlobalVars::jsInertia»;


        }
        «Common::enclosingNamespacesClose(robot)»
        #endif
        '''

    def inertiaMatrixSource(Robot robot) '''
        «loadInfo(robot)»
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::jsimHeader(robot)».h"

        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::transforms6D»;

        «val robo_ns_qualifier = Names$Namespaces$Qualifiers::robot(robot)»
        «val robodyn_ns_qualifier = robo_ns_qualifier + "::" + Names$Namespaces::dynamics»
        «val class_qualifier = robodyn_ns_qualifier + "::" + className»

        // The joint space inertia matrix of this robot
        «robodyn_ns_qualifier»::«Names$Types::jspaceMLocal» «robodyn_ns_qualifier»::«Names$GlobalVars::jsInertia»;

        //Implementation of default constructor
        «class_qualifier»::«className»()
        {
            //Make sure all the transforms of the robot are initialized
            «robo_ns_qualifier»::«Names$Namespaces::transforms6D»::initAll();
            «robo_ns_qualifier»::«Names$Namespaces::transforms6D»::«Names$Namespaces::T6D_force»::initAll();
            //Initialize the matrix itself
            this->setZero();
            // Initialize the 6D composite-inertia tensor of each body of the robot
            «LinkInertias::className(robot)» linkInertias;
            «FOR l : links»
                «inertiaCompositeName(l)» = linkInertias.«LinkInertias::tensorGetterName(l)»();
            «ENDFOR»
        }

        #define DATA operator()
        #define F(j) (block<6,1>(0,(j)+6))

        const «class_qualifier»& «class_qualifier»::operator()(const «Names$Types::jointState»& state) {
            «val sortedLinks = robot.links.sortBy(link | link.ID).reverse /* do not consider the robot base */»
            «IF !floatingBase»
                static «Names$Namespaces$Qualifiers::iit_rbd»::ForceVector F;
            «ENDIF»

            // Precomputes only once the coordinate transforms:
            «FOR l : sortedLinks»
                «val parent = l.parent»
                «IF floatingBase || !(parent.equals(robot.base))»
                    «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(parent, l)»(state);
                    «Transforms::child_X_parent__mxName(parent, l)»(state);
                «ENDIF»
            «ENDFOR»

            // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
            «FOR l : sortedLinks»

                // Link «l.name»:
                «val parent = l.parent»
                «IF floatingBase || !(parent.equals(robot.base))»
                    «inertiaCompositeName(parent)» = «inertiaCompositeName(parent)» + «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(parent, l)» * «inertiaCompositeName(l)» * «Transforms::child_X_parent__mxName(parent, l)»;
                «ENDIF»

                «val jt = getJoint(parent, l)»
                «val F = getF(jt)»
                «val jointIndex = jsimIndex(jt)»
                «F» = «inertiaCompositeName(l)».col(«Common::spatialVectIndex(jt)»);
                DATA(«jointIndex», «jointIndex») = «F».row(«Common::spatialVectIndex(jt)»)(0,0);

                «val chain = TreeUtils::chainToBase(l)»
                «inertiaMatrix_lastStep(chain, jt)»
                «IF floatingBase»
                    «F» = «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(robot.base, chain.last)» * «F»;
                «ENDIF»
            «ENDFOR»

            «IF floatingBase»
                // Copies the upper-right block into the lower-left block, after transposing
                block<«jointDOFs», 6>(6,0) = (block<6, «jointDOFs»>(0,6)).transpose();
                // The composite-inertia of the whole robot is the upper-left quadrant of the JSIM
                block<6,6>(0,0) = «inertiaCompositeName(currRobot.base)»;
            «ENDIF»
            return *this;
        }

        #undef DATA
        #undef F

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

    /*
     * Saves in the members of this instance the relevant information about
     * the given robot
     */
    def private void loadInfo(Robot robot) {
        currRobot = robot
        dofs = robot.DOFs
        jointDOFs = robot.jointDOFs
        floatingBase = robot.base.floating
        if(floatingBase) {
            links = robot.abstractLinks
        } else {
            links = new ArrayList<AbstractLink>()
            links.addAll(robot.links)
        }

        className = Names$Types::jspaceMLocal
    }
    /*
     * Code to update all and only the required spatial vectors transforms
     * required by the composite-rigid-body algorithm for the JSIM
     */
    def private parentChildTransform(AbstractLink p, AbstractLink c) '''
        «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(p, c)»(state);
        «Transforms::child_X_parent__mxName(p, c)»(state);
        «FOR child : c.childrenList.children»
            «parentChildTransform(c, child.link)»
        «ENDFOR»
    '''

    def private getF(Joint j) {
        if( !floatingBase) {
            return '''F'''
        } else {
            return '''F(«Common::jointIdentifier(j)»)'''
        }
    }

    def private jsimIndex(Joint j) {
        if(floatingBase) {
            return '''«Common::jointIdentifier(j)»+6'''
        } else {
            return Common::jointIdentifier(j)
        }
    }

    def private inertiaCompositeName(AbstractLink l) '''Ic_«l.name»'''

    def private inertiaMatrix_lastStep(List<AbstractLink> chainToBase, Joint rowJoint) {
        val strBuff = new StringConcatenation()

        var AbstractLink parent
        var Joint parentJ
        var CharSequence F
        var CharSequence col
        val row = jsimIndex(rowJoint)
        for( link : chainToBase ) {
            parent = link.parent
            F      = getF(rowJoint)
            if( ! parent.equals( currRobot.base ) ) {
                parentJ = parent.connectingJoint;
                col     = jsimIndex(parentJ)
                strBuff.append('''
                «F» = «Names$Namespaces::T6D_force»::«Transforms::parent_X_child__mxName(parent, link)» * «F»;
                DATA(«row», «col») = «F».transpose().col(«Common::spatialVectIndex(parentJ)»)(0,0);
                DATA(«col», «row») = DATA(«row», «col»);
                ''');
            }
        }
        return strBuff;
    }

    def main_test(Robot robot) '''
        «val jsim = Names$GlobalVars::jsInertia»
        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::jsimHeader(robot)».h"

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
        #include "«Names$Files$RBD::jsimHeader(robot)».h"

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

    def private LTLfactorization(Robot robot) '''
        L = «Names$GlobalVars::jsInertia».triangularView<Eigen::Lower>();
        «FOR Joint joint : robot.joints.reverseView»
            «val row = joint.getID()-1»
            // Joint «joint.name», index «row» :
            L(«row», «row») = std::sqrt(L(«row», «row»));
            «val chainToBase = TreeUtils::chainToBase(joint.predecessorLink)»
            «FOR ancestor : chainToBase»
                «val col = ancestor.connectingJoint.arrayIdx»
                L(«row», «col») = L(«row», «col») / L(«row», «row»);
            «ENDFOR»
            «FOR ancestor : chainToBase»
                «val i = ancestor.connectingJoint.arrayIdx»
                «val secondChain = TreeUtils::chainToBase(ancestor)»
                «FOR ancestor2 : secondChain»
                    «val j = ancestor2.connectingJoint.arrayIdx»
                    L(«i», «j») = L(«i», «j») - L(«row», «i») * L(«row», «j»);
                «ENDFOR»
            «ENDFOR»

        «ENDFOR»
    '''

    def private Linverse(Robot robot) '''
        «FOR jo : robot.joints»
            «val i = jo.ID-1»
            Linv(«i», «i») = 1 / L(«i», «i»);
        «ENDFOR»
        «FOR jo : robot.joints.drop(1)»
            «val link = jo.successorLink»
            «val chain = getChainJoints(TreeUtils::buildChain(jo.predecessorLink, robot.base))»
            «val i = jo.ID-1»
            «FOR jo2 : chain»
                «val j = jo2.ID-1»
                «val subChain = getChainJoints(TreeUtils::buildChain(jo2.successorLink, link))»
                Linv(«i», «j») = - Linv(«j», «j») * («FOR jo3 : subChain»«val k = jo3.ID-1»(Linv(«i», «k») * L(«k», «j»)) + «ENDFOR»0);
            «ENDFOR»
        «ENDFOR»
    '''

    def private Minverse(Robot robot) {
        val strBuff = new StringConcatenation()
        for(jo_i : robot.joints) {
            val i = jo_i.arrayIdx
            // Get the chain containing all the joints in the chain up to the base, including 'jo_i' itself
            val chain = getChainJoints(TreeUtils::buildChain(jo_i.successorLink, robot.base))
            for(jo_j : chain) {
                val j = jo_j.arrayIdx
                strBuff.append('''inverse(«i», «j») = ''')

                // Get the chain containing all the joints from the base to joint 'jo_j' itself
                val chain2 = getChainJoints(TreeUtils::buildChain(robot.base, jo_j.successorLink))
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




    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    private Robot currRobot
    private int dofs
    private int jointDOFs
    private List<AbstractLink> links
    private boolean floatingBase
    private String className


}