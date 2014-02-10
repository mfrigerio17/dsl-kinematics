package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.Common
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.common.Parameters
import iit.dsl.kinDsl.InertiaParams
import iit.dsl.kinDsl.Var

class LinkInertias {

    def public static className(Robot r) '''InertiaProperties'''
    def public static updateParamsMethodName() '''updateParameters'''
    def public static tensorGetterName(AbstractLink l) '''getTensor_«l.name»'''
    def public static massGetterName(AbstractLink l)   '''getMass_«l.name»'''
    def public static comGetterName(AbstractLink l)   '''getCOM_«l.name»'''

    extension Common common = new Common()

	def header(Robot robot) '''
        «val parametric = Parameters::massPropertiesAreParametric(robot)»
        «val paramsClass = InertiaParameters::className»
        #ifndef IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files$RBD::inertiaHeader(robot).toUpperCase()»_H_
        #define IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files$RBD::inertiaHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/utils.h>

        #include "«Names$Files::mainHeader(robot)».h"
        «IF parametric»
            #include "«Names$Files$RBD::massParametersHeader(robot)».h"
        «ENDIF»

        «iit::dsl::generator::cpp::Common::enclosingNamespacesOpen(robot)»
        /**
         * This namespace encloses classes and functions related to the Dynamics
         * of the robot «robot.name».
         */
        namespace «Names$Namespaces::dynamics» {

        typedef «Names$Namespaces$Qualifiers::iit_rbd»::InertiaMatrixDense InertiaMatrix;

        «val links = getRelevantLinks(robot)»
        class «className(robot)» {
            public:
                «className(robot)»(«IF parametric»const «paramsClass»&«ENDIF»);
                ~«className(robot)»();
                «FOR l : links»
                    const InertiaMatrix& «tensorGetterName(l)»() const;
                «ENDFOR»
                «FOR l : links»
                    double «massGetterName(l)»() const;
                «ENDFOR»
                «FOR l : links»
                    const «Names$Types::vector3d»& «comGetterName(l)»() const;
                «ENDFOR»
                double getTotalMass() const;

                «IF parametric»
                    /*!
                     * Forces the update of the runtime inertia parameters of the robot
                     * «robot.name».
                     *
                     * This function uses in turn the getter methods of the «InertiaParameters::className»
                     * member of this instance. All the inertia properties contained in
                     * this instance that are defined in terms of non-constant parameters,
                     * will be updated.
                     */
                    void «updateParamsMethodName()»();
                «ENDIF»
            private:
                «IF parametric»
                    const «paramsClass»* «memberName_paramsGetter»;
                «ENDIF»

                «FOR l : links»
                    InertiaMatrix «memberName_tensor(l)»;
                «ENDFOR»
                «FOR l : links»
                    «Names$Types::vector3d» «memberName_com(l)»;
                «ENDFOR»
        };


        inline «className(robot)»::~«className(robot)»() {}

        «FOR l : links»
            inline const InertiaMatrix& «className(robot)»::«tensorGetterName(l)»() const {
                return this->«memberName_tensor(l)»;
            }
        «ENDFOR»
        «FOR l : links»
            inline double «className(robot)»::«massGetterName(l)»() const {
                return this->«memberName_tensor(l)».getMass();
            }
        «ENDFOR»
        «FOR l : links»
            inline const «Names$Types::vector3d»& «className(robot)»::«comGetterName(l)»() const {
                return this->«memberName_com(l)»;
            }
        «ENDFOR»

        inline double «className(robot)»::getTotalMass() const {
            return «FOR l : links SEPARATOR " + "»«value(l.inertiaParams.mass)»«ENDFOR»;
        }

        }
        «iit::dsl::generator::cpp::Common::enclosingNamespacesClose(robot)»

        #endif
	'''

    def source(Robot robot) '''
        «val nsqualifier    = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
        «val classqualifier = nsqualifier + "::" + className(robot)»
        «val links = getRelevantLinks(robot)»
        «val parametric = Parameters::massPropertiesAreParametric(robot)»
        #include "«Names$Files$RBD::inertiaHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::iit_rbd»;

        «classqualifier»::«className(robot)»(«IF parametric»const «InertiaParameters::className»& pGetter«ENDIF»)
            «IF parametric»
                : «memberName_paramsGetter»(& pGetter)
            «ENDIF»
        {
            «FOR l : links»
                «val params = getLinkFrameInertiaParams(l)»
                «memberName_com(l)» = «comRHS(params)»;
                «memberName_tensor(l)».fill(
                    «value(l.inertiaParams.mass)»,
                    «memberName_com(l)»,
                    «buildTensorExpr(l.inertiaParams)» );

            «ENDFOR»
        }

        «IF parametric»
            void «classqualifier»::«updateParamsMethodName()»() {
                «FOR l : links»
                    «updateRuntimeParamsCode(l)»
                «ENDFOR»
            }
        «ENDIF»
    '''

    def private updateRuntimeParamsCode(AbstractLink link) '''
        «val inertia = getLinkFrameInertiaParams(link)»
        «val parametricFlags = Parameters::whoIsParametric(inertia)»
        «IF( parametricFlags.all )»
            «memberName_com(link)» = «comRHS(inertia)»;
                «memberName_tensor(link)».fill(
                    «value(inertia.mass)»,
                    «memberName_com(link)»,
                    «buildTensorExpr(inertia)»);
        «ELSE»
            «IF( parametricFlags.mass)»
                «memberName_tensor(link)».changeMass(«value(inertia.mass)»);
            «ENDIF»
            «IF( parametricFlags.com)»
                «memberName_com(link)» = «comRHS(inertia)»;
                «memberName_tensor(link)».changeCOM(«memberName_com(link)»);
            «ENDIF»
            «IF( parametricFlags.tensor)»
                «memberName_tensor(link)».changeRotationalInertia(
                    «buildTensorExpr(inertia)»);
            «ENDIF»
        «ENDIF»
    '''

    def private comRHS(InertiaParams in)
        '''«Names$Types::vector3d»(«value(in.com.x)»,«value(in.com.y)»,«value(in.com.z)»)'''
    def private buildTensorExpr(InertiaParams inertia)
        '''
        Utils::buildInertiaTensor(
                «value(inertia.ix)»,
                «value(inertia.iy)»,
                «value(inertia.iz)»,
                «value(inertia.ixy)»,
                «value(inertia.ixz)»,
                «value(inertia.iyz)»)'''

    def private value(Var v) {
        if( Parameters::isParameter(v) ) {
            val p = Parameters::asParameter(v)
            return '''«memberName_paramsGetter»->«InertiaParameters::getterFunctionName(p)»()'''
        }
        // The Var is either a constant value or a simple expression of a constant
        return '''«v.asFloat»'''
    }

    def private memberName_tensor(AbstractLink l) '''tensor_«l.name»'''
    def private memberName_com(AbstractLink l) '''com_«l.name»'''
    def private memberName_paramsGetter() '''paramsGetter'''

    def private getRelevantLinks(Robot robot) {
        if(robot.base.floating) {
            return robot.abstractLinks //include the base body
        } else {
            return robot.links
        }
    }


}