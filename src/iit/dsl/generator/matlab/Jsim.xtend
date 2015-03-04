package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.AbstractLink

import iit.dsl.generator.common.TreeUtils

import java.util.List

import org.eclipse.xtend2.lib.StringConcatenation


class Jsim
{
    public static CharSequence updateFunctionName = '''updateJSIM'''
    public static CharSequence invertFunctionName = '''invertJSIM'''

    extension iit.dsl.generator.Common common = iit.dsl.generator.Common::getInstance()

    def private inertiaCompositeName(AbstractLink l) '''Ic_«l.name»'''
    def private inertiaName(AbstractLink l) '''lf_«l.name».tensor6D'''

    private static String jsim_varName = "H"
    private static String jsim_inv_varName = "Hinv"

    private static CharSequence xfVarName = '''force_transforms'''
    private static CharSequence ipVarName = '''inertia_props'''

    def jsim_update_code(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        «loadInfo(robot)»
        function «returnValues()» = «updateFunctionName»(«ipVarName», «xfVarName»)

        % Initialization of the composite-inertia matrices
        «FOR l : robot.links»
            «inertiaCompositeName(l)» = «ipVarName».«inertiaName(l)»;
        «ENDFOR»
        «IF floatingBase»
            «inertiaCompositeName(robot.base)» = «ipVarName».«inertiaName(robot.base)»;
        «ENDIF»

        % "Bottom-up" loop to update the inertia-composite property of each link,
        %  for the current configuration
        «val sortedLinks = robot.links.sortBy(link | getID(link)).reverse»
        «FOR l : sortedLinks»

            % Link «l.name»
            «val parent = l.parent»
            «IF floatingBase || !(parent.equals(currRobot.base))»
                «val parent_XFrc_child = iit::dsl::coord::generator::matlab::Generator::identifier(
                    iit::dsl::generator::common::Transforms::getTransform(transformsModel, parent, l),
                    iit::dsl::coord::generator::Utilities$MatrixType::_6D_FORCE)»
                «inertiaCompositeName(parent)» = «inertiaCompositeName(parent)» + «xfVarName».«parent_XFrc_child» * «inertiaCompositeName(l)» * «xfVarName».«parent_XFrc_child»';
            «ENDIF»

            «val linkJoint = getJoint(parent, l)»
            «val idx = Common::spatialVectorIndex(linkJoint)»
            «getF(linkJoint)» = «inertiaCompositeName(l)»(:,«idx»);
            «jsim_varName»(«linkJoint.ID», «linkJoint.ID») = «getF(linkJoint, idx)»;

            «val chain = iit::dsl::generator::common::TreeUtils::chainToBase(l)»
            «jsim_lastStep(chain, linkJoint, transformsModel)»
        «ENDFOR»
	'''

    def private jsim_lastStep(List<AbstractLink> chainToBase, Joint currentJ,
        iit.dsl.coord.coordTransDsl.Model transModel)
    {
        val strBuff = new StringConcatenation()

        var AbstractLink parent
        var Joint parentJ
        val F = getF(currentJ)
        for( link : chainToBase )
        {
            parent = link.parent
            val parent_X_link = xfVarName + "." +
                iit::dsl::coord::generator::matlab::Generator::identifier(
                    iit::dsl::generator::common::Transforms::getTransform(transModel, parent, link),
                    iit::dsl::coord::generator::Utilities$MatrixType::_6D_FORCE)

            if( ! parent.equals(currRobot.base) )
            {
                parentJ = parent.connectingJoint
                val Jidx = Common::spatialVectorIndex(parentJ)
                strBuff.append('''
                «F» = «parent_X_link» * «F»;
                «jsim_varName»(«parentJ.ID», «currentJ.ID») = «jsim_varName»(«currentJ.ID», «parentJ.ID») = «getF(currentJ, Jidx)»;

                ''');
            }
            else if(floatingBase)
            {
                strBuff.append('''
                «F» = «parent_X_link» * «F»;
                ''');
            }
        }
        return strBuff;
    }

    def private getF(Joint j)
    {
        if( ! floatingBase) {
            return '''F'''
        } else {
            return '''F(:,«j.ID»)'''
        }
    }
    def private getF(Joint j, int el)
    {
        if( ! floatingBase) {
            return '''F(«el»)'''
        } else {
            return '''F(«el»,«j.ID»)'''
        }
    }

    def private returnValues() {
        if(floatingBase) {
            return '''[H «inertiaCompositeName(currRobot.base)» F]'''
        }
        return '''H'''
    }

    def jsim_inverse_code(Robot robot) '''
        function [«jsim_inv_varName» L Linv] = «invertFunctionName»(«jsim_varName»)

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


    /*
     * Saves in the members of this instance the relevant information about
     * the given robot
     */
    def private void loadInfo(Robot robot)
    {
        currRobot = robot
        floatingBase = robot.base.floating
    }

    private Robot currRobot
    private boolean floatingBase
}
