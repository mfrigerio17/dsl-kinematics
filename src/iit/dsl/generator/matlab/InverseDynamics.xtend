package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Link
import iit.dsl.coord.generator.Utilities
import iit.dsl.kinDsl.AbstractLink

class InverseDynamics
{
    public static CharSequence functionName = '''inverseDynamics'''

    // TODO: possibly, in the future, optimize the spatial cross product (i.e.
    //  avoid code that computes the whole 6x6 matrix)

    public def body(Robot robot) '''
        «val updateXMFunc = iit::dsl::coord::generator::matlab::Generator::updateFunctionName(Utilities$MatrixType::_6D)»
        «IF robot.base.floating»
            function [tau «robot.base.acceleration»] = «functionName»(«ipVarName», «xmVarName», «robot.base.velocity», gravity, qd, qdd, q)

            if nargin == 7   % the joint status is also an argument
                xm = «updateXMFunc»(xm, q);
            end
        «ELSE»
            function tau = «functionName»(«ipVarName», «xmVarName», qd, qdd, q)

            if nargin == 5   % the joint status is also an argument
                xm = «updateXMFunc»(xm, q);
            end

            g = 9.81;
        «ENDIF»

        «val sortedLinks = robot.links.sortBy(link | getID(link))»
        «val transformsMap = Common::getParentToChildTransforms(robot, Utilities$MatrixType::_6D, xmVarName)»
        «FOR Link l : sortedLinks»
            «val parent   = l.parent»
            «val myJoint  = l.connectingJoint»
            «val velocity = l.velocity»
            «val acceler  = l.acceleration»
            «val child_X_parent = transformsMap.get(l)»
            «val jid         = Common::arrayIndex(myJoint)»
            «val subspaceIdx = Common::spatialVectorIndex(myJoint)»

            % First pass, link '«l.name»'
            «velocity» = zeros(6,1);
            «acceler» = zeros(6,1);

            «IF robot.base.floating»
                «velocity» = «child_X_parent» * «parent.velocity»;
                «velocity»(«subspaceIdx») += qd(«jid»);

                vcross = vcross_mx(«velocity»);

                «IF parent.equals(robot.base)/* parent is the floating base */»
                    «acceler» = vcross(:,«subspaceIdx») * qd(«jid»);
                «ELSE»
                    «acceler» = «child_X_parent» * «parent.acceleration» + (vcross(:,«subspaceIdx») * qd(«jid»));
                «ENDIF»
                «acceler»(«subspaceIdx») += qdd(«jid»);

                «l.force» = «l.spatialInertia» * «acceler» + (-vcross' * «l.spatialInertia» * «velocity»);
            «ELSE»
                «IF parent.equals(robot.base)»
                    «acceler» = «child_X_parent»(:,6) * g; % TODO hide 6
                    «acceler»(«subspaceIdx») += qdd(«jid»);
                    «velocity»(«subspaceIdx») = qd(«jid»);
                    vcross = vcross_mx(«velocity»);

                    «l.force» = «l.spatialInertia» * «acceler» + (( -vcross' * «l.spatialInertia»)(:,«subspaceIdx») * qd(«jid»));
                «ELSE»
                    «velocity» = ((«child_X_parent») * «parent.velocity»);
                    «velocity»(«subspaceIdx») += qd(«jid»);

                    vcross = vcross_mx(«velocity»);

                    «acceler» = «child_X_parent» * «parent.acceleration» + (vcross(:,«subspaceIdx») * qd(«jid»));
                    «acceler»(«subspaceIdx») += qdd(«jid»);

                    «l.force» = «l.spatialInertia» * «acceler» + (-vcross' * «l.spatialInertia» * «velocity»);
                «ENDIF»
            «ENDIF»
        «ENDFOR»

        «IF robot.base.floating»
            %
            % The force exerted on the floating base by the links
            %
            «val base = robot.base»
            vcross = vcross_mx(«base.velocity»);
            «base.force» = - vcross' * «base.spatialInertia» * «base.velocity»;


            %
            % Pass 2. Compute the composite inertia and the spatial forces
            %
            ci = «CompositeInertia::functionName»(«ipVarName», «xmVarName», 'motion');
            «FOR l : sortedLinks.reverseView()»
                «val parent = l.parent»
                «val child_X_parent = transformsMap.get(l)»
                «parent.force» = «parent.force» + «child_X_parent»' * «l.force»;
            «ENDFOR»

            %
            % The base acceleration due to the force due to the movement of the links
            %
            «robot.base.acceleration» = - inverse(ci.«robot.base.inertiaC») * «robot.base.force»; % TODO inverse

            %
            % Pass 3. Compute the joint forces while propagating back the floating base acceleration
            %
            tau = zeros(«robot.DOFs - 6», 1);
            «FOR l : sortedLinks»
                «val parent = l.parent»
                «val joint  = l.connectingJoint»
                «val child_X_parent = transformsMap.get(l)»
                «val idx = Common::spatialVectorIndex(joint)»
                «l.acceleration» = «child_X_parent» * «parent.acceleration»;
                tau(«Common::arrayIndex(joint)») = ci.«l.inertiaC»(«idx»,:) * «l.acceleration» + «l.force»(«idx»);

            «ENDFOR»

            «robot.base.acceleration» = «robot.base.acceleration» + gravity;
        «ELSE»
            %
            % Pass 2. Compute the joint torques while back propagating the spatial forces
            %
            tau = zeros(«robot.DOFs»,1);
            «FOR l : sortedLinks.reverseView»
                % Link '«l.name»'
                «val parent = l.parent»
                «val joint  = l.connectingJoint»
                tau(«Common::arrayIndex(joint)») = «l.force»(«Common::spatialVectorIndex(joint)»);
                «IF ( ! parent.equals(robot.base))»
                    «val child_X_parent = transformsMap.get(l)»
                    «parent.force» = «parent.force» + «child_X_parent»' * «l.force»;
                «ENDIF»
            «ENDFOR»
        «ENDIF»

        function vc = vcross_mx(v)
            vc = [   0    -v(3)  v(2)   0     0     0    ;
                     v(3)  0    -v(1)   0     0     0    ;
                    -v(2)  v(1)  0      0     0     0    ;
                     0    -v(6)  v(5)   0    -v(3)  v(2) ;
                     v(6)  0    -v(4)   v(3)  0    -v(1) ;
                    -v(5)  v(4)  0     -v(2)  v(1)  0    ];
    '''

    def private spatialInertia(AbstractLink l)
    '''«ipVarName».«InertiaProperties::fieldName(l)».«InertiaProperties::fieldName_spatialInertia»'''


    extension iit.dsl.generator.Common common = iit.dsl.generator.Common::getInstance()
    extension SpatialQuantitiesNames varnames = SpatialQuantitiesNames::getInstance()

    private static CharSequence xmVarName = '''xm'''
    private static CharSequence ipVarName = '''ip'''
}