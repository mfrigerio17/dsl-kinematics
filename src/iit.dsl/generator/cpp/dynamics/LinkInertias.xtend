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
        «val paramsClass = InertiaParameters::getNamespace() + "::" + InertiaParameters::className»
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
                    const double& «massGetterName(l)»() const;
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
                     * This function uses in turn the getter methods of the «InertiaParameters::namespace»::«InertiaParameters::className»
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
                    double «memberName_mass(l)»;
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
            inline const double& «className(robot)»::«massGetterName(l)»() const {
                return this->«memberName_mass(l)»;
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

        «classqualifier»::«className(robot)»(«IF parametric»const «InertiaParameters::namespace»::«InertiaParameters::className»& pGetter«ENDIF»)
            :
            «IF parametric»
                «memberName_paramsGetter»(& pGetter),
            «ENDIF»
            «FOR l : links SEPARATOR ','»
                «memberName_mass(l)»(«value(l.inertiaParams.mass)»)
            «ENDFOR»
        {
            «FOR l : links»
                «val params = getLinkFrameInertiaParams(l)»
                «memberName_com(l)» = «comRHS(params)»;
                «memberName_tensor(l)».fill(
                    «memberName_mass(l)», «memberName_com(l)»,
                    Utils::buildInertiaTensor(
                        «value(params.ix)»,
                        «value(params.iy)»,
                        «value(params.iz)»,
                        «value(params.ixy)»,
                        «value(params.ixz)»,
                        «value(params.iyz)»));

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
        «IF(Parameters::isParametric(link.inertiaParams))»
            «val inertia = getLinkFrameInertiaParams(link)»
            «IF( Parameters::isParameter(inertia.mass))»
                «memberName_mass(link)» = «value(inertia.mass)»;
            «ENDIF»
            «IF( Parameters::isParametric(inertia.com))»
                «memberName_com(link)» = «comRHS(inertia)»;
            «ENDIF»
            «memberName_tensor(link)».fill(
                «memberName_mass(link)», «memberName_com(link)»,
                «IF( anyParametricMoment(inertia))»
                    Utils::buildInertiaTensor(
                        «value(inertia.ix)»,
                        «value(inertia.iy)»,
                        «value(inertia.iz)»,
                        «value(inertia.ixy)»,
                        «value(inertia.ixz)»,
                        «value(inertia.iyz)»)
                «ELSE»
                    «memberName_tensor(link)».get3x3Tensor()
                «ENDIF»
                );
        «ENDIF»
    '''

    def private comRHS(InertiaParams in)
        '''«Names$Types::vector3d»(«value(in.com.x)»,«value(in.com.y)»,«value(in.com.z)»)'''

    def private value(Var v) {
        if( Parameters::isParameter(v) ) {
            val p = Parameters::asParameter(v)
            return '''«memberName_paramsGetter»->«InertiaParameters::getterFunctionName(p)»()'''
        }
        // The Var is either a constant value or a simple expression of a constant
        return '''«v.asFloat»'''
    }

    def private boolean anyParametricMoment(InertiaParams in) {
        return
            Parameters::isParameter(in.ix)  ||
            Parameters::isParameter(in.iy)  ||
            Parameters::isParameter(in.iz)  ||
            Parameters::isParameter(in.ixy) ||
            Parameters::isParameter(in.ixz) ||
            Parameters::isParameter(in.iyz)
    }

    def private memberName_mass(AbstractLink l) '''mass_«l.name»'''
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