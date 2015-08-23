package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.coord.generator.Utilities

class CompositeInertia
{
    public static final CharSequence functionName = '''compositeInertia'''

    def public functionBody(Robot robot)
    '''
        function «retValueName» = «functionName»(«ipVarName», «xfVarName», transformsType)

        % Computes the spatial composite inertia of each link of the robot.
        % Arguments:
        % - «ipVarName» : the structure with the inertia properties
        % - «xfVarName» : the structure with the spatial coordinate transformation matrices
        % - transformsType : a string specifying which is the type of the given
        %      coordinate transforms, either velocity ('motion') or force ('force').
        %      Optional argument, default is 'force'.

        if nargin < 3
            transformsType = 'force';
        end

        %
        % Initialization of the composite-inertia matrices
        %
        «FOR l : robot.links»
            «ciField(l)» = «ipVarName».«InertiaProperties::fieldName(l)».«InertiaProperties::fieldName_spatialInertia»;
        «ENDFOR»
        «IF robot.base.floating»
            «ciField(robot.base)» = «ipVarName».«InertiaProperties::fieldName(robot.base)».«InertiaProperties::fieldName_spatialInertia»;
        «ENDIF»

        %
        % Leafs-to-root pass to update the composite inertia of
        %     each link, for the current configuration:
        %
        «val sortedLinks   = robot.links.sortBy(link | link.ID).reverse»

        if strcmp(transformsType, 'motion')  % we have transforms for motion vectors
        «val motionTransformsMap = Common::getParentToChildTransforms(robot, Utilities$MatrixType::_6D, xfVarName)»
        «FOR l : sortedLinks»

            % Contribution of link «l.name»
            «val parent = l.parent»
            «IF robot.base.floating || !(parent.equals(robot.base))»
                «val child_XM_parent = motionTransformsMap.get(l)»
                «ciField(parent)» = «ciField(parent)» + «child_XM_parent»' * «ciField(l)» * «child_XM_parent»;
            «ENDIF»

        «ENDFOR»

        else % we have transforms for force vectors
        «val forceTransformsMap = Common::getChildtoParentTransforms(robot, Utilities$MatrixType::_6D_FORCE, xfVarName)»
        «FOR l : sortedLinks»

            % Contribution of link «l.name»
            «val parent = l.parent»
            «IF robot.base.floating || !(parent.equals(robot.base))»
                «val parent_XF_child = forceTransformsMap.get(l)»
                «ciField(parent)» = «ciField(parent)» + «parent_XF_child» * «ciField(l)» * «parent_XF_child»';
            «ENDIF»

        «ENDFOR»

        end
    '''

    def private ciField(AbstractLink l) {
        return retValueName + "." + l.inertiaC
    }

    extension iit.dsl.generator.Common common = iit.dsl.generator.Common::getInstance()
    extension SpatialQuantitiesNames varnames = SpatialQuantitiesNames::getInstance()

    private static CharSequence xfVarName = '''xf'''
    private static CharSequence ipVarName = '''ip'''
    private static CharSequence retValueName = '''ci'''
}