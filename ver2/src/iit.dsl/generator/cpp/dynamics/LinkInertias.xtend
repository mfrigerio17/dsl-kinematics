package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.Common
import iit.dsl.kinDsl.AbstractLink

class LinkInertias {

    def static className(Robot r) '''LinkInertias'''
    def static getterName(AbstractLink l) '''get_«l.name»'''

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
                    const InertiaMatrix& «getterName(l)»() const;
                «ENDFOR»
            private:
                «FOR l : robot.links»
                    InertiaMatrix «inertiaMxName(l)»;
                «ENDFOR»
        };


        inline «className(robot)»::~«className(robot)»() {}

        «FOR l : robot.links»
            inline const InertiaMatrix& «className(robot)»::«getterName(l)»() const {
                return this->«inertiaMxName(l)»;
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
        #include "«Names$Files$LinkInertias::header(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::iit_rbd»;

        «classqualifier»::«className(robot)»() {
            «FOR l : robot.links»
                «inertiaMxName(l)».fill(«l.inertiaParams.mass», Vector3d(«l.inertiaParams.com.x.str»,«l.inertiaParams.com.y.str»,«l.inertiaParams.com.z.str»),
                    Utils::buildInertiaTensor(«l.inertiaParams.ix»,«l.inertiaParams.iy»,«l.inertiaParams.iz»,«l.inertiaParams.ixy»,«l.inertiaParams.ixz»,«l.inertiaParams.iyz»));
            «ENDFOR»
        }
    '''
}