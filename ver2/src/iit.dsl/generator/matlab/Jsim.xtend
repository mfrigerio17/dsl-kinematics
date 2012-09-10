package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.PrismaticJoint
import java.util.List
import iit.dsl.kinDsl.Joint
import org.eclipse.xtend2.lib.StringConcatenation

class Jsim {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def private inertiaCompositeName(AbstractLink l) '''Ic_«l.name»'''
    def private inertiaName(AbstractLink l) '''inertia_lf_«l.name».tensor6D'''

    def jsim_init_code(Robot robot) '''
        H = zeros(«robot.joints.size»,«robot.joints.size»);
        Hinv = H;
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
                «val parent_X_child = iit::dsl::generator::common::Transforms::l1_X_l2__defaultName(transformsModel, parent, l)»
                «val child_X_parent = iit::dsl::generator::common::Transforms::l2_X_l1__defaultName(transformsModel, parent, l)»
                «inertiaCompositeName(parent)» = «inertiaName(parent)» + asForceTransform(«parent_X_child») * «inertiaCompositeName(l)» * «child_X_parent»;
            «ENDIF»

            % Assuming 1 DoF joints, the multiplication by the motion subspace matrix results in the selection of a column/row
            «val linkJoint = getJoint(parent, l)»
            «IF linkJoint instanceof PrismaticJoint»
                F = «inertiaCompositeName(l)»(:,6);
                H(«linkJoint.ID», «linkJoint.ID») = F(6);
            «ELSE»
                F = «inertiaCompositeName(l)»(:,3);
                H(«linkJoint.ID», «linkJoint.ID») = F(3);
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
                F = asForceTransform(«iit::dsl::generator::common::Transforms::l1_X_l2__defaultName(transModel, parent, link)») * F;
                tmp = F';
                H(«rowIndex», «parentJ.ID») = tmp«IF parentJ instanceof PrismaticJoint»(:,6)«ELSE»(:,3)«ENDIF»;
                H(«parentJ.ID», «rowIndex») = H(«rowIndex», «parentJ.ID»); % the matrix is symmetric
                ''');
            }
        }
        return strBuff;
    }
}