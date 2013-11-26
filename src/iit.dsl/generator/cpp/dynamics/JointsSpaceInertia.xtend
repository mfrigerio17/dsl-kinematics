package iit.dsl.generator.cpp.dynamics

import java.util.List
import java.util.ArrayList

import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Common
import iit.dsl.generator.common.TreeUtils

import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.Robot
import iit.dsl.generator.common.Transforms


class JointsSpaceInertia {
    def inertiaMatrixHeader(Robot robot)'''
        «loadInfo(robot)»
        #ifndef IIT_«Names$Files$RBD::jsimHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files$RBD::jsimHeader(robot).toUpperCase()»_H_

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/StateDependentMatrix.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»
        namespace «Names$Namespaces::dynamics» {

        /**
         * The type of the Joint Space Inertia Matrix (JSIM) of the robot «robot.name».
         */
        «val superType = Names$Namespaces$Qualifiers::iit_rbd() + "::" + Names$Types$IIT_RBD::stateDependentMatrix(jointStateQualifiedTypeName, dofs.toString, dofs.toString, className)»
        class «className» : public «superType»
        {
            private:
                typedef «superType» Base;
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
                «className»(«LinkInertias::className(robot)»&, «Names$Types$Transforms::spatial_force»&);
                ~«className»() {}

                const «className»& update(const «Names$Types::jointState»&);


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
                «LinkInertias::className(robot)»& linkInertias;
                «Names$Types$Transforms::spatial_force»* «forceTransformsMember»;

                // The composite-inertia tensor for each link
                «FOR l : links»
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



        }
        «Common::enclosingNamespacesClose(robot)»
        #endif
        '''

    def inertiaMatrixSource(Robot robot, iit.dsl.coord.coordTransDsl.Model transformsModel) '''
        «loadInfo(robot)»
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::jsimHeader(robot)».h"

        «val robo_ns_qualifier = Names$Namespaces$Qualifiers::robot(robot)»
        «val robodyn_ns_qualifier = robo_ns_qualifier + "::" + Names$Namespaces::dynamics»
        «val class_qualifier = robodyn_ns_qualifier + "::" + className»

        //Implementation of default constructor
        «class_qualifier»::«className»(«LinkInertias::className(robot)»& inertiaProperties, «Names$Types$Transforms::spatial_force»& forceTransforms) :
            linkInertias(inertiaProperties),
            «forceTransformsMember»( &forceTransforms ),
            «FOR l : robot.chainEndLinks SEPARATOR ','»
                «inertiaCompositeName(l)»(linkInertias.«LinkInertias::tensorGetterName(l)»())
            «ENDFOR»
        {
            //Initialize the matrix itself
            this->setZero();
        }

        #define DATA operator()
        #define F(j) (block<6,1>(0,(j)+6))

        const «class_qualifier»& «class_qualifier»::update(const «Names$Types::jointState»& state) {
            «val sortedLinks = robot.links.sortBy(link | link.ID).reverse /* do not consider the robot base */»
            «IF !floatingBase»
                static «Names$Namespaces$Qualifiers::iit_rbd»::ForceVector F;
            «ENDIF»

            // Precomputes only once the coordinate transforms:
            «FOR l : sortedLinks»
                «val parent = l.parent»
                «IF floatingBase || !(parent.equals(robot.base))»
                    «val parent_X_child = Transforms::getTransform(transformsModel, parent, l)»
                    «Xforce(parent_X_child)»(state);
                «ENDIF»
            «ENDFOR»

            // Initializes the composite inertia tensors
            «FOR l : links»
                «IF ! l.childrenList.children.empty»
                    «inertiaCompositeName(l)» = linkInertias.«LinkInertias::tensorGetterName(l)»();
                «ENDIF»
            «ENDFOR»

            // "Bottom-up" loop to update the inertia-composite property of each link, for the current configuration
            «FOR l : sortedLinks»

                // Link «l.name»:
                «val parent = l.parent»
                «val parent_X_child = Transforms::getTransform(transformsModel, parent, l)»
                «IF floatingBase || !(parent.equals(robot.base))»
                    «inertiaCompositeName(parent)» = «inertiaCompositeName(parent)» + «Xforce(parent_X_child)» * «inertiaCompositeName(l)» * («Xforce(parent_X_child)»).transpose();
                «ENDIF»

                «val jt = getJoint(parent, l)»
                «val F = getF(jt)»
                «val jointIndex = jsimIndex(jt)»
                «F» = «inertiaCompositeName(l)».col(«Common::spatialVectIndex(jt)»);
                DATA(«jointIndex», «jointIndex») = «F».row(«Common::spatialVectIndex(jt)»)(0,0);

                «val chain = TreeUtils::chainToBase(l)»
                «inertiaMatrix_lastStep(chain, jt, transformsModel)»
                «IF floatingBase»
                    «val base_X_last = Transforms::getTransform(transformsModel, robot.base, chain.last)»
                    «F» = «Xforce(base_X_last)» * «F»;
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
        jointStateQualifiedTypeName = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Types::jointState
    }


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
    def private forceTransformsMember() '''frcTransf'''
    def private Xforce(iit.dsl.coord.coordTransDsl.Transform t)
        '''«forceTransformsMember» -> «iit::dsl::coord::generator::cpp::EigenFiles::memberName(t)»'''

    def private inertiaMatrix_lastStep(
        List<AbstractLink> chainToBase,
        Joint rowJoint,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    {
        val strBuff = new StringConcatenation()

        var AbstractLink parent
        var Joint parentJ
        var CharSequence F
        var CharSequence col
        var iit.dsl.coord.coordTransDsl.Transform parent_X_child
        val row = jsimIndex(rowJoint)
        for( link : chainToBase ) {
            parent = link.parent
            F      = getF(rowJoint)
            if( ! parent.equals( currRobot.base ) ) {
                parentJ = parent.connectingJoint;
                col     = jsimIndex(parentJ)
                parent_X_child = Transforms::getTransform(transformsModel, parent, link)
                strBuff.append('''
                «F» = «Xforce(parent_X_child)» * «F»;
                DATA(«row», «col») = «F».transpose().col(«Common::spatialVectIndex(parentJ)»)(0,0);
                DATA(«col», «row») = DATA(«row», «col»);
                ''');
            }
        }
        return strBuff;
    }

    def main_test(Robot robot) '''
        «val jsim = "jsim"»
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

            «Names$Types$Transforms::spatial_force» ft;
            JSIM «jsim»(ft);
            «jsim»(q);
            «jsim».computeL();
            «jsim».computeInverse();
            cout << "Joint Space Inertia Matrix M" << endl << «jsim» << endl << endl;
            cout << "L" << endl << «jsim».getL() << endl;
            cout << "Inverse of M" << endl << «jsim».getInverse() << endl << endl;
            return 0;
        }'''


    def private LTLfactorization(Robot robot) '''
        L = this -> triangularView<Eigen::Lower>();
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
    private String jointStateQualifiedTypeName

}