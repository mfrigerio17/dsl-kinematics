package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.Common
import iit.dsl.kinDsl.AbstractLink

class LinkInertias {

    def static className(Robot r) '''LinkInertias'''
    def static tensorGetterName(AbstractLink l) '''getTensor_«l.name»'''
    def static massGetterName(AbstractLink l)   '''getMass_«l.name»'''
    def static comGetterName(AbstractLink l)   '''getCOM_«l.name»'''

    extension Common common = new Common()

	def header(Robot robot) '''
        #ifndef IIT_RBD_«Names$Files$LinkInertias::header(robot).toUpperCase()»_H_
        #define IIT_RBD_«Names$Files$LinkInertias::header(robot).toUpperCase()»_H_

        #include <Eigen/Dense>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/utils.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {
        namespace «Names$Namespaces::dynamics» {

        typedef «Names$Namespaces$Qualifiers::iit_rbd»::InertiaMatrixDense InertiaMatrix;


        class «className(robot)» {
            public:
                «className(robot)»();
                ~«className(robot)»();
                «FOR l : robot.links»
                    const InertiaMatrix& «tensorGetterName(l)»() const;
                «ENDFOR»
                «FOR l : robot.links»
                    const double& «massGetterName(l)»() const;
                «ENDFOR»
                «FOR l : robot.links»
                    const «Names$Types::vector3d»& «comGetterName(l)»() const;
                «ENDFOR»
            private:
                «FOR l : robot.links»
                    InertiaMatrix «memberName_tensor(l)»;
                «ENDFOR»
                «FOR l : robot.links»
                    double «memberName_mass(l)»;
                «ENDFOR»
                «FOR l : robot.links»
                    «Names$Types::vector3d» «memberName_com(l)»;
                «ENDFOR»
        };


        inline «className(robot)»::~«className(robot)»() {}

        «FOR l : robot.links»
            inline const InertiaMatrix& «className(robot)»::«tensorGetterName(l)»() const {
                return this->«memberName_tensor(l)»;
            }
        «ENDFOR»
        «FOR l : robot.links»
            inline const double& «className(robot)»::«massGetterName(l)»() const {
                return this->«memberName_mass(l)»;
            }
        «ENDFOR»
        «FOR l : robot.links»
            inline const «Names$Types::vector3d»& «className(robot)»::«comGetterName(l)»() const {
                return this->«memberName_com(l)»;
            }
        «ENDFOR»

        }
        }
        }

        #endif
	'''

    def source(Robot robot) '''
        «val nsqualifier    = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
        «val classqualifier = nsqualifier + "::" + className(robot)»
        «val links = robot.links»
        #include "«Names$Files$LinkInertias::header(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::iit_rbd»;

        «classqualifier»::«className(robot)»() :
            «memberName_mass(links.get(0))»(«links.get(0).inertiaParams.mass»)
            «FOR l:links.drop(1)»
                , «memberName_mass(l)»(«l.inertiaParams.mass»)
            «ENDFOR»
        {
            «FOR l : links»
                «val params = getLinkFrameInertiaParams(l)»
                «memberName_com(l)» = «Names$Types::vector3d»(«params.com.x.str»,«params.com.y.str»,«params.com.z.str»);
                «memberName_tensor(l)».fill(«memberName_mass(l)», «memberName_com(l)»,
                    Utils::buildInertiaTensor(«params.ix»,«params.iy»,«params.iz»,«params.ixy»,«params.ixz»,«params.iyz»));

            «ENDFOR»
        }
    '''

    def private memberName_mass(AbstractLink l) '''mass_«l.name»'''
    def private memberName_tensor(AbstractLink l) '''tensor_«l.name»'''
    def private memberName_com(AbstractLink l) '''com_«l.name»'''
}