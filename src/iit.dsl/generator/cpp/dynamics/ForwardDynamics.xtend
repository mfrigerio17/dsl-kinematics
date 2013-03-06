package iit.dsl.generator.cpp.dynamics

import java.util.List

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Common
import iit.dsl.generator.cpp.kinematics.Transforms
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Link
import iit.dsl.generator.cpp.RobotHeaders

class ForwardDynamics {
    def static className(Robot r) '''ForwardDynamics'''

    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def public headerContent(Robot robot) '''
        «val rbd_ns = Names$Namespaces$Qualifiers::iit_rbd»
        «val jState = Names$Types::jointState»
        «val extF_t = Names$Types::extForces»
        #ifndef IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files$RBD::abaHeader(robot).toUpperCase()»_H_
        #define IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files$RBD::abaHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/utils.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$LinkInertias::header(robot)».h"
        #include "«Names$Files::linkDataMapHeader(robot)».h"

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {
        namespace «Names$Namespaces::dynamics» {

        typedef «rbd_ns»::InertiaMatrixDense InertiaMatrix;

        /**
         * The Forward Dynamics routine for the robot «robot.name».
         *
         * The parameters common to most of the methods are the joint status \c q, the
         * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
         * will be filled with the computed values. Overloaded methods without the \c q
         * parameter use the current configuration of the robot; they are provided for
         * the sake of efficiency, in case the kinematics transforms of the robot have
         * already been updated elsewhere with the most recent configuration (eg by a
         * call to setJointStatus()), so that it would be useless to compute them again.
         */
        class «className(robot)» {
        public:
            typedef «RobotHeaders::linkDataMap_type()»<«rbd_ns»::ForceVector> «extF_t»;
        public:
            «className(robot)»();
            /** \name Forward dynamics
             * The Articulated-Body-Algorithm to compute the joint accelerations
             */ ///@{
            /**
             * \param q the joint status vector
             * \param qd the joint velocities vector
             * \param tau the joint forces (torque or force)
             * \param qdd the joint accelerations vector (output parameter).
                           These values are computed by this function.
             * \param fext the external forces, optional. Each force must be
             *              expressed in the reference frame of the link it is
             *              exerted on.
             */
            void fd(const «jState»& q, const «jState»& qd, const «jState»& tau,
                           «jState»& qdd, const «extF_t»& fext = zeroExtForces);
            void fd(const «jState»& qd, const «jState»& tau,«jState»& qdd,
                                          const «extF_t»& fext = zeroExtForces);
            ///@}

            /** Updates all the kinematics transforms used by this instance. */
            void setJointStatus(const «jState»& q) const;
        private:
            typedef «rbd_ns»::ForceVector Force;
            typedef «rbd_ns»::VelocityVector Velocity;

            «rbd_ns»::Matrix66d spareMx; // support variable
            «FOR l : robot.links»
                // Link '«l.name»' :
                InertiaMatrix «artInertiaName(l)»;
                Velocity «accelerationName(l)»;
                Velocity «velocityName(l)»;
                Velocity «cTermName(l)»;
                Force    «biasForceName(l)»;

                «rbd_ns»::Column6d «UTermName(l)»;
                double «DTermName(l)»;
                double «uTermName(l)»;
            «ENDFOR»

            «LinkInertias::className(robot)» «linkInertiasMemeberName»;
        private:
            static const «extF_t» zeroExtForces;
        };

        inline void «className(robot)»::setJointStatus(const «jState»& q) const {
            «setJointStatusCode(robot.links)»
        }

        inline void «className(robot)»::fd(const «jState»& q,
                                           const «jState»& qd,
                                           const «jState»& tau,
                                           «jState»& qdd,
                                           const «extF_t»& fext/* = zeroExtForces */)
        {
            setJointStatus(q);
            fd(qd, tau, qdd, fext);
        }

        }
        }
        }

        #endif
    '''
    def private linkInertiasMemeberName() '''inpar'''

    def public implementationFileContent(Robot robot) '''
        «val jState = Names$Types::jointState»
        «val extF_t = Names$Types::extForces»
        #include "«Names$Files$RBD::abaHeader(robot)».h"

        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
        using namespace «Names$Namespaces$Qualifiers::iit_rbd»;

        // Initialization of static-const data
        const «nsqualifier»::«className(robot)»::«extF_t»
            «nsqualifier»::«className(robot)»::zeroExtForces(Force::Zero());

        «nsqualifier»::«className(robot)»::«className(robot)»() {
            «FOR l : robot.links»
                «velocityName(l)».setZero();
                «cTermName(l)».setZero();
            «ENDFOR»

            «Names$Namespaces::transforms6D»::initAll(); // initializes coordinates transforms
        }

        void «nsqualifier»::«className(robot)»::fd(const «jState»& qd,
                                                   const «jState»& tau,
                                                   «jState»& qdd,
                                                   const «extF_t»& fext/* = zeroExtForces */)
        {
            «ABABody(robot)»
        }
    '''


    def private ABABody(Robot robot) '''
        «val rbd_ns = Names$Namespaces$Qualifiers::iit_rbd»
        «val sortedLinks = robot.links.sortBy(link | getID(link))»

        «FOR l : sortedLinks»
            «artInertiaName(l)» = «linkInertiasMemeberName».«LinkInertias::tensorGetterName(l)»();
        «ENDFOR»
        «FOR l : sortedLinks»
            «biasForceName(l)» = - fext[«Common::linkIdentifier(l)»];
        «ENDFOR»
        // ---------------------- FIRST PASS ---------------------- //
        «FOR l : sortedLinks»
            «val parent   = l.parent»
            «val joint    = l.connectingJoint»
            «val velocity = l.velocityName»
            «val cterm    = cTermName(l)»
            «val biasF    = biasForceName(l)»
            «val jid      = Common::jointIdentifier(joint)»
            «val child_X_parent = Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(parent, l)»

            «val subspaceIdx = Common::spatialVectIndex(joint)»
            // + Link «l.name»
            «IF parent.equals(robot.base)»
                //  - The spatial velocity:
                «velocity»(«subspaceIdx») = qd(«jid»);

                //  - The bias force term:
                Utils::fillAsForceCrossProductMx(«velocity», spareMx);
                «biasF» += spareMx * «artInertiaName(l)».col(«subspaceIdx») * qd(«jid»);
            «ELSE»
                //  - The spatial velocity:
                «velocity» = («child_X_parent» * «parent.velocityName»);
                «velocity»(«subspaceIdx») += qd(«jid»);

                //  - The velocity-product acceleration term:
                Utils::fillAsMotionCrossProductMx(«velocity», spareMx);
                «cterm» = (spareMx.col(«subspaceIdx») * qd(«jid»));

                //  - The bias force term:
                «biasF» += -spareMx.transpose() * «artInertiaName(l)» * «velocity»;
            «ENDIF»
        «ENDFOR»

        // ---------------------- SECOND PASS ---------------------- //
        InertiaMatrix Ia;
        Force pa;
        «FOR l : sortedLinks.reverseView»
            «val joint = l.connectingJoint»
            «val subspaceIdx = Common::spatialVectIndex(joint)»
            «val U = UTermName(l)»
            «val D = DTermName(l)»
            «val u = uTermName(l)»
            «val p = biasForceName(l)»
            «val I = artInertiaName(l)»
            «val parent = l.parent»
            «val child_X_parent = Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(parent, l)»

            // + Link «l.name»
            «U» = «I».col(«subspaceIdx»);
            «D» = «U».row(«subspaceIdx»)(0,0);
            «u» = tau(«Common::jointIdentifier(joint)») - «p».row(«subspaceIdx»)(0,0);

            «IF !parent.equals(robot.base)»
                Ia = «I» - «U»/«D» * «U».transpose();
                pa = «p» + Ia * «cTermName(l)» + «U» * «u»/«D»;

                «artInertiaName(parent)» += «child_X_parent».transpose() * Ia * «child_X_parent»;
                «biasForceName(parent)» += «child_X_parent».transpose() * pa;
            «ENDIF»
        «ENDFOR»

        // ---------------------- THIRD PASS ---------------------- //
        Velocity atmp;
        «FOR l : sortedLinks»
            «val parent = l.parent»
            «val joint  = l.connectingJoint»
            «val jid    = Common::jointIdentifier(joint)»
            «val child_X_parent = Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(parent, l)»
            «IF parent.equals(robot.base)»
                atmp = «child_X_parent».col(«Names$Namespaces$Qualifiers::iit_rbd»::LZ) * («rbd_ns»::g);
            «ELSE»
                atmp = «child_X_parent» * «accelerationName(parent)» + «cTermName(l)»;
            «ENDIF»
            qdd(«jid») = («uTermName(l)» - «UTermName(l)».transpose() * atmp) / «DTermName(l)»;
            «accelerationName(l)» = atmp;
            «accelerationName(l)».row(«Common::spatialVectIndex(joint)»)(0,0) += qdd(«jid»);

        «ENDFOR»
    '''

    def private setJointStatusCode(List<Link> sortedLinks) '''
        «FOR l : sortedLinks»
            «Names$Namespaces::transforms6D + "::" + Transforms::child_X_parent__mxName(l.parent, l)»(q);
        «ENDFOR»
    '''

    def private cTermName(AbstractLink l) '''«l.name»_c'''
    def private artInertiaName(AbstractLink l) '''«l.name»_AI'''
    def private biasForceName(AbstractLink l) '''«l.name»_p'''
    def private UTermName(AbstractLink l) '''«l.name»_U'''
    def private DTermName(AbstractLink l) '''«l.name»_D'''
    def private uTermName(AbstractLink l) '''«l.name»_u'''

}