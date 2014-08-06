package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink

import iit.dsl.generator.cpp.dynamics.LinkInertias
import iit.dsl.kinDsl.ChildSpec

class Misc {
    def header(Robot robot) '''
        #ifndef IIT_ROBCOGEN__«robot.name.toUpperCase()»_MISCELLANEOUS_H_
        #define IIT_ROBCOGEN__«robot.name.toUpperCase()»_MISCELLANEOUS_H_

        #include "«Names$Files$RBD::inertiaHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»

        /** \name Center of mass calculation
         * Computes the Center Of Mass (COM) position of the whole robot, in
         * base coordinates.
         *
         * Common parameters are the inertia properties of the robot and the set
         * of homogeneous coordinate transforms. If a joint status variable is
         * also passed, then the transforms are updated accordingly; otherwise,
         * they are not modified before being used.
         */
        ///@{
        /**
         * \param inertia the inertia properties of the links of the robot
         * \param transforms the homogeneous coordinate transforms of the robot
         * \return the position of the Center Of Mass of the whole robot, expressed
         *         in base coordinates
         */
        «Names$Types::vector3d» getWholeBodyCOM(
            const «Names$Namespaces::dynamics»::«LinkInertias::className(robot)»& inertia,
            const «Names$Types$Transforms::homogeneous»& transforms);
        /**
         * \param inertia the inertia properties of the links of the robot
         * \param q the joint status vector describing the configuration of the robot
         * \param transforms the homogeneous coordinate transforms of the robot
         * \return the position of the Center Of Mass of the whole robot, expressed
         *         in base coordinates
         */
        «Names$Types::vector3d» getWholeBodyCOM(
            const «Names$Namespaces::dynamics»::«LinkInertias::className(robot)»& inertia,
            const «Names$Types::jointState»& q,
            «Names$Types$Transforms::homogeneous»& transforms);
        ///@}

        «Common::enclosingNamespacesClose(robot)»

        #endif
    '''

    def source(Robot robot) '''
        #include <iit/rbd/utils.h>
        #include "«Names$Files::miscHeader(robot)».h"

        using namespace «Common::enclosingNamespacesQualifier(robot)»;
        using namespace «Common::enclosingNamespacesQualifier(robot)»::«Names$Namespaces::dynamics»;

        «Names$Types::vector3d» «Common::enclosingNamespacesQualifier(robot)»::getWholeBodyCOM(
            const «LinkInertias::className(robot)»& inertiaProps,
            const «Names$Types$Transforms::homogeneous»& ht)
        {
            «val tmpX = "tmpX"»
            «val cumSum = "tmpSum"»
            «Names$Types::vector3d» «cumSum»(«Names$Types::vector3d»::Zero());

            «IF robot.base.floating»
                «cumSum» += inertiaProps.«LinkInertias::comGetterName(robot.base)»() * inertiaProps.«LinkInertias::massGetterName(robot.base)»();
            «ENDIF»

            «Names$Types$Transforms::homogeneous»::MatrixType «tmpX»(«Names$Types$Transforms::homogeneous»::MatrixType::Identity());
            «depthVisit(robot.base, tmpX, cumSum)»

            return «cumSum» / inertiaProps.getTotalMass();
        }

        «Names$Types::vector3d» «Common::enclosingNamespacesQualifier(robot)»::getWholeBodyCOM(
            const «LinkInertias::className(robot)»& inertiaProps,
            const «Names$Types::jointState»& q,
            «Names$Types$Transforms::homogeneous»& ht)
        {
            // First updates the coordinate transforms that will be used by the routine
            «FOR l : robot.abstractLinks»
                «FOR childSpec : l.childrenList.children»
                    ht.«iit::dsl::generator::cpp::kinematics::Transforms::parent_X_child__mxName(l, childSpec.link)»(q);
                «ENDFOR»
            «ENDFOR»

            // The actual calculus
            return getWholeBodyCOM(inertiaProps, ht);
        }
    '''

    def private tempXName(ChildSpec spec) '''base_X_«spec.joint.name»_chain'''

    def private CharSequence depthVisit(
        AbstractLink current,
        CharSequence base_X_current,
        CharSequence sumVariableName)
    {
        val children = current.childrenList.children
        if(children.size == 0) {
            return null
        }
        val branching = children.size > 1
        var CharSequence base_X_child = base_X_current
        val text = new StringBuffer()

        if(branching) {
            for(childSpec : children) {
                text.append('''«Names$Types$Transforms::homogeneous»::MatrixType «tempXName(childSpec)»;''')
                text.append("\n")
            }
            text.append("\n\n")
        }

        for(childSpec : children) {
            val child = childSpec.link
            if(branching) {
                base_X_child = tempXName(childSpec)
            }

            text.append('''
            «base_X_child» = «base_X_current» * ht.«iit::dsl::generator::cpp::kinematics::Transforms::parent_X_child__mxName(current,child)»;
            «sumVariableName» += inertiaProps.«LinkInertias::massGetterName(child)»() *
                    ( «Names$Namespaces$Qualifiers::iit_rbd»::Utils::transform(«base_X_child», inertiaProps.«LinkInertias::comGetterName(child)»()));

            «depthVisit(child, base_X_child, sumVariableName)»''')
        }

        return text
    }

    private extension iit.dsl.generator.Common utils = iit::dsl::generator::Common::getInstance()
}