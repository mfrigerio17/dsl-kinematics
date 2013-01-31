package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.PrismaticJoint
import java.util.List
import iit.dsl.kinDsl.Joint
import org.eclipse.xtend2.lib.StringConcatenation
import iit.dsl.generator.common.TreeUtils

class Jsim {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def private inertiaCompositeName(AbstractLink l) '''Ic_«l.name»'''
    def private inertiaName(AbstractLink l) '''inertia_lf_«l.name».tensor6D'''

    private static String jsim_varName = "H"
    private static String jsim_inv_varName = "Hinv"

    def jsim_init_code(Robot robot) '''
        «jsim_varName» = zeros(«robot.joints.size»,«robot.joints.size»);
        «jsim_inv_varName» = «jsim_varName»;
        L = «jsim_varName»;
        Linv = «jsim_varName»;
        «val endLinks = chainEndLinks(robot)»
        «FOR l : endLinks»
            «inertiaCompositeName(l)» = «inertiaName(l)»;
        «ENDFOR»
    '''

    def jsim_update_code(Robot robot) '''
        «val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)»
        % "Bottom-up" loop to update the inertia-composite property of each link,
        %  for the current configuration
        «val sortedLinks = robot.links.sortBy(link | getID(link)).reverse»
        «FOR l : sortedLinks»
            % Link «l.name»
            «val parent = l.parent»
            «IF !(parent.equals(robot.base))»
                «val parent_XFrc_child = iit::dsl::coord::generator::matlab::Generator::identifier(
                    iit::dsl::generator::common::Transforms::getTransform(transformsModel, parent, l),
                    iit::dsl::coord::generator::Utilities$MatrixType::_6D_FORCE)»
                «val child_X_parent = iit::dsl::coord::generator::matlab::Generator::identifier(
                    iit::dsl::generator::common::Transforms::getTransform(transformsModel, l, parent),
                    iit::dsl::coord::generator::Utilities$MatrixType::_6D)»
                «inertiaCompositeName(parent)» = «inertiaName(parent)» + «parent_XFrc_child» * «inertiaCompositeName(l)» * «child_X_parent»;
            «ENDIF»

            % Assuming 1 DoF joints, the multiplication by the motion subspace matrix results in the selection of a column/row
            «val linkJoint = getJoint(parent, l)»
            «IF linkJoint instanceof PrismaticJoint»
                F = «inertiaCompositeName(l)»(:,6);
                «jsim_varName»(«linkJoint.ID», «linkJoint.ID») = F(6);
            «ELSE»
                F = «inertiaCompositeName(l)»(:,3);
                «jsim_varName»(«linkJoint.ID», «linkJoint.ID») = F(3);
            «ENDIF»

            «val chain = iit::dsl::generator::common::TreeUtils::chainToBase(l)»
            «jsim_lastStep(chain, linkJoint.ID, transformsModel)»
        «ENDFOR»
	'''

    def private jsim_lastStep(List<AbstractLink> chainToBase, int rowIndex,
        iit.dsl.coord.coordTransDsl.Model transModel)
    {
        val strBuff = new StringConcatenation()

        var AbstractLink parent
        var Joint parentJ
        for( link : chainToBase ) {
            parent = link.parent
            if( ! parent.equals( (parent.eContainer() as Robot).base ) ) {
                parentJ = getConnectingJoint(parent);
                strBuff.append('''
                F = «iit::dsl::coord::generator::matlab::Generator::identifier(
                    iit::dsl::generator::common::Transforms::getTransform(transModel, parent, link),
                    iit::dsl::coord::generator::Utilities$MatrixType::_6D_FORCE)» * F;
                tmp = F';
                «jsim_varName»(«rowIndex», «parentJ.ID») = tmp«IF parentJ instanceof PrismaticJoint»(:,6)«ELSE»(:,3)«ENDIF»;
                «jsim_varName»(«parentJ.ID», «rowIndex») = «jsim_varName»(«rowIndex», «parentJ.ID»); % the matrix is symmetric
                ''');
            }
        }
        return strBuff;
    }

    def jsim_inverse_code(Robot robot) '''
        % The following code computes the lower triangular matrix L such that
        %  «jsim_varName» = L' L  (LTL factorization)
        % Then it computes the inverse of L and the inverse of «jsim_varName»

        % LTL factorization
        «LTLfactorization(robot)»

        % Inverse of L
        «Linverse(robot)»

        % Inverse of «jsim_varName»
        «JSIMinverse(robot)»
    '''

    def LTLfactorization(Robot robot) '''
        L = tril(«jsim_varName»); % lower triangular
        «FOR Joint joint : robot.joints.reverseView»
            «val row = joint.getID()»
            % Joint «joint.name», index «row» :
            L(«row», «row») = sqrt(L(«row», «row»));
            «val chainToBase = iit::dsl::generator::common::TreeUtils::chainToBase(joint.predecessorLink)»
            «FOR ancestor : chainToBase»
                «val col = ancestor.connectingJoint.getID»
                L(«row», «col») = L(«row», «col») / L(«row», «row»);
            «ENDFOR»
            «FOR ancestor : chainToBase»
                «val i = ancestor.connectingJoint.getID»
                «val secondChain = iit::dsl::generator::common::TreeUtils::chainToBase(ancestor)»
                «FOR ancestor2 : secondChain»
                    «val j = ancestor2.connectingJoint.getID»
                    L(«i», «j») = L(«i», «j») - L(«row», «i») * L(«row», «j»);
                «ENDFOR»
            «ENDFOR»

        «ENDFOR»
    '''
    def Linverse(Robot robot) '''
        «FOR jo : robot.joints»
            «val i = jo.ID»
            Linv(«i», «i») = 1 / L(«i», «i»);
        «ENDFOR»
        «FOR jo : robot.joints.drop(1)»
            «val link = jo.successorLink»
            «val chain = getChainJoints(TreeUtils::buildChain(jo.predecessorLink, robot.base))»
            «val i = jo.ID»
            «FOR jo2 : chain»
                «val j = jo2.ID»
                «val subChain = getChainJoints(TreeUtils::buildChain(jo2.successorLink, link))»
                Linv(«i», «j») = - Linv(«j», «j») * («FOR jo3 : subChain»«val k = jo3.ID»(Linv(«i», «k») * L(«k», «j»)) + «ENDFOR»0);
            «ENDFOR»
        «ENDFOR»
    '''

    def JSIMinverse(Robot robot) {
        val strBuff = new StringConcatenation()

        for(jo_i : robot.joints) {
            val i = jo_i.ID
            // For all the joints in the chain up to the base, including 'jo_i' itself ...
            val chain = getChainJoints(TreeUtils::buildChain(jo_i.successorLink, robot.base))
            for(jo_j : chain) {
                val j = jo_j.ID
                strBuff.append('''«jsim_inv_varName»(«i», «j») = ''')

                // For all the joints from the base to joint 'jo_j' itself ...
                val chain2 = getChainJoints(TreeUtils::buildChain(robot.base, jo_j.successorLink))
                for(jo_k : chain2) {
                    val k = jo_k.ID
                    strBuff.append(''' + (Linv(«i», «k») * Linv(«j», «k»))''')
                }
                strBuff.append(";\n")
                // Exploits the symmetry to fill the other elements of the matrix
                if(i != j) {
                    strBuff.append('''«jsim_inv_varName»(«j», «i») = «jsim_inv_varName»(«i», «j»);''')
                    strBuff.append("\n")
                }
            }
        }
        return strBuff
    }

}
