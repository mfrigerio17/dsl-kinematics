package iit.dsl.generator.cpp.dynamics

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.Link
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.RobotHeaders
import iit.dsl.generator.cpp.Common
import java.util.List
import iit.dsl.generator.common.Transforms
import java.util.Map
import java.util.HashMap

class InverseDynamics {

    def static className(Robot r) '''InverseDynamics'''

    def mainHeader(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        «loadInfo(robot, transformsModel)»
        #ifndef IIT_«robot.name.toUpperCase()»_«Names$Files$RBD::invDynHeader(robot).toUpperCase()»_H_
        #define IIT_«robot.name.toUpperCase()»_«Names$Files$RBD::invDynHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/utils.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files::linkDataMapHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»
        namespace «Names$Namespaces::dynamics» {

        /**
         * The Inverse Dynamics routine for the robot «robot.name».
         *
         * In addition to the full Newton-Euler algorithm, specialized versions to compute
         * only certain terms are provided.
         * The parameters common to most of the methods are the joint status \c q, the
         * joint velocities \c qd and the accelerations \c qdd. The \c «name_jointsForce» parameter
         * will be filled with the computed values.
         * Additional overloaded methods are provided without the \c q parameter. These
         * methods use the current configuration of the robot; they are provided for the
         * sake of efficiency, in case the kinematics transforms of the robot have already
         * been updated elsewhere with the most recent configuration (eg by a call to
         * setJointStatus()), so that it is useless to compute them again.
         */
        class «className(robot)» {
        public:

            typedef «rbd_ns»::ForceVector Force;
            typedef «rbd_ns»::VelocityVector Velocity;
            typedef «rbd_ns»::VelocityVector Acceleration;
            typedef «rbd_ns»::InertiaMatrixDense InertiaMatrix;
            typedef «RobotHeaders::linkDataMap_type()»<«rbd_ns»::ForceVector> «extF_t»;
        public:
            /**
             * Default constructor
             * \param tr the container of all the spatial motion transforms of
             *     the robot «robot.name», which will be used by this instance
             *     to compute inverse-dynamics.
             */
            «className(robot)»(«Names$Types$Transforms::spatial_motion»& tr);

            «val params ='''«fParam_qd», «fParam_qdd», «fParam_fext» = zeroExtForces'''»
            «IF floatingBase»
                /** \name Inverse dynamics
                 * The full algorithm for inverse dynamics for this robot
                 */ ///@{
                void id(
                    «fParam_tau», «fParam_basea»,
                    «fParam_gravity», «fParam_basev»,
                    «fParam_q», «fParam_qd», «fParam_qdd»,
                    «fParam_fext» = zeroExtForces);
                void id(
                    «fParam_tau», «fParam_basea»,
                    «fParam_gravity», «fParam_basev»,
                    «fParam_qd», «fParam_qdd»,
                    «fParam_fext» = zeroExtForces);
                ///@}
                /** \name Inverse dynamics, fully actuated base
                 * The inverse dynamics algorithm for the floating base robot,
                 * in the assumption of a fully actuated base
                 */ ///@{
                void id_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_gravity», «fParam_basev», «fParam_basea_const»,
                    «fParam_q», «params»);
                void id_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_gravity», «fParam_basev», «fParam_basea_const»,
                    «params»);
                ///@}
                /** \name Gravity terms, fully actuated base
                 */
                ///@{
                void G_terms_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_gravity», «fParam_q»);
                void G_terms_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_gravity»);
                ///@}
                /** \name Centrifugal and Coriolis terms, fully actuated base
                 *
                 * These functions take only velocity inputs, that is, they assume
                 * a zero spatial acceleration of the base (in addition to zero acceleration
                 * at the actuated joints).
                 * Note that this is NOT the same as imposing zero acceleration
                 * at the virtual 6-dof-floting-base joint, which would result, in general,
                 * in a non-zero spatial acceleration of the base, due to velocity
                 * product terms.
                 */
                ///@{
                void C_terms_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_basev», «fParam_q», «fParam_qd»);
                void C_terms_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_basev», «fParam_qd»);
                ///@}
            «ELSE»
                /** \name Inverse dynamics
                 * The full Newton-Euler algorithm for inverse dynamics for this robot
                 * \param fext the external forces acting on the links. Each external
                 *        force must be expressed in the frame of the link it is
                 *        exerted on.
                 */
                ///@{
                void id(
                    «fParam_tau»,
                    «fParam_q», «fParam_qd», «fParam_qdd»,
                    «fParam_fext» = zeroExtForces);
                void id(
                    «fParam_tau»,
                    «fParam_qd», «fParam_qdd»,
                    «fParam_fext» = zeroExtForces);
                ///@}

                /** \name Gravity terms
                 * The joint forces (linear or rotational) required to compensate
                 * for the effect of gravity, in a specific configuration.
                 */
                ///@{
                void G_terms(«fParam_tau», «fParam_q»);
                void G_terms(«fParam_tau»);
                ///@}

                /** \name Centrifugal and Coriolis terms
                 * The forces (linear or rotational) acting on the joints due to centrifugal and
                 * Coriolis effects, for a specific configuration.
                 */
                ///@{
                void C_terms(«fParam_tau», «fParam_q», «fParam_qd»);
                void C_terms(«fParam_tau», «fParam_qd»);
                ///@}
            «ENDIF»
            /** Updates all the kinematics transforms used by the inverse dynamics routine. */
            void setJointStatus(«fParam_q») const;

        protected:
            «IF floatingBase»
                void secondPass_fullyActuated(«fParam_tau»);
            «ELSE»
                void firstPass(«fParam_qd», «fParam_qdd»);
                void secondPass(«fParam_tau»);
            «ENDIF»

        private:
            «rbd_ns»::Matrix66d spareMx; // support variable
            «IF floatingBase»
                // The robot base
                InertiaMatrix «robot.base.inertia»;
                InertiaMatrix «robot.base.inertiaC»;
                Force         «robot.base.force»;
                // The composite inertia tensors
                «FOR l : robot.links»
                    «IF l.childrenList.children.empty»
                        const InertiaMatrix& «l.inertiaC»;
                    «ELSE»
                        InertiaMatrix «l.inertiaC»;
                    «ENDIF»
                «ENDFOR»
            «ENDIF»

            «FOR l : robot.links»
                // Link '«l.name»' :
                InertiaMatrix «l.inertia»;
                Velocity      «l.velocity»;
                Acceleration  «l.acceleration»;
                Force         «l.force»;
            «ENDFOR»

            «Names$Types$Transforms::spatial_motion»* «motionTransformsMember»;
        private:
            static const «extF_t» zeroExtForces;
        };

        inline void «className(robot)»::setJointStatus(«fParam_q») const
        {
            «setJointStatusCode()»
        }

        «IF floatingBase»
            inline void «className(robot)»::id(
                «fParam_tau», «fParam_basea»,
                «fParam_gravity», «fParam_basev»,
                «fParam_q», «fParam_qd», «fParam_qdd»,
                «fParam_fext»)
            {
                setJointStatus(q);
                id(«name_jointsForce», «robot.base.acceleration», g, «robot.base.velocity»,
                   qd, qdd, fext);
            }

            inline void «className(robot)»::G_terms_fully_actuated(
                «fParam_basef», «fParam_tau»,
                «fParam_gravity», «fParam_q»)
            {
                setJointStatus(q);
                G_terms_fully_actuated(«name_baseWrench», «name_jointsForce», g);
            }

            inline void «className(robot)»::C_terms_fully_actuated(
                «fParam_basef», «fParam_tau»,
                «fParam_basev», «fParam_q», «fParam_qd»)
            {
                setJointStatus(q);
                C_terms_fully_actuated(«name_baseWrench», «name_jointsForce», «robot.base.velocity», qd);
            }

            inline void «className(robot)»::id_fully_actuated(
                    «fParam_basef», «fParam_tau»,
                    «fParam_gravity», «fParam_basev», «fParam_basea_const»,
                    «fParam_q», «fParam_qd», «fParam_qdd», «fParam_fext»)
            {
                setJointStatus(q);
                id_fully_actuated(«name_baseWrench», «name_jointsForce», g, «robot.base.velocity»,
                    «name_baseAccel», qd, qdd, fext);
            }
        «ELSE»
            inline void «className(robot)»::G_terms(«fParam_tau», «fParam_q»)
            {
                setJointStatus(q);
                G_terms(«name_jointsForce»);
            }

            inline void «className(robot)»::C_terms(«fParam_tau», «fParam_q», «fParam_qd»)
            {
                setJointStatus(q);
                C_terms(«name_jointsForce», qd);
            }

            inline void «className(robot)»::id(
                «fParam_tau»,
                «fParam_q», «fParam_qd», «fParam_qdd»,
                «fParam_fext»)
            {
                setJointStatus(q);
                id(«name_jointsForce», qd, qdd, fext);
            }
        «ENDIF»

        }
        «Common::enclosingNamespacesClose(robot)»

        #endif
        '''

    def inverseDynamicsImplementation(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
            #include "«Names$Files$RBD::invDynHeader(robot)».h"
            #include "«Names$Files$RBD::inertiaHeader(robot)».h"
            #ifndef EIGEN_NO_DEBUG
                #include <iostream>
            #endif
            «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
            using namespace std;
            using namespace «Names$Namespaces$Qualifiers::iit_rbd»;
            using namespace «nsqualifier»;

            // Initialization of static-const data
            const «nsqualifier»::«className(robot)»::«extF_t»
            «nsqualifier»::«className(robot)»::zeroExtForces(Force::Zero());

            «nsqualifier»::«className(robot)»::«className(robot)»(«Names$Types$Transforms::spatial_motion»& transforms) :
                «IF floatingBase»
                    «FOR l : robot.chainEndLinks SEPARATOR ',' AFTER ','»
                        «l.inertiaC»(«l.inertia»)
                     «ENDFOR»
                «ENDIF»

               «motionTransformsMember»( & transforms )
            {
            #ifndef EIGEN_NO_DEBUG
                std::cout << "Robot «robot.name», «className(robot)»::«className(robot)»()" << std::endl;
                std::cout << "Compiled with Eigen debug active" << std::endl;
            #endif

                «LinkInertias::className(robot)» linkInertias;
                «IF floatingBase»
                    «robot.base.inertia» = linkInertias.«LinkInertias::tensorGetterName(robot.base)»();
                «ENDIF»
                «FOR l : robot.links»
                    «l.inertia» = linkInertias.«LinkInertias::tensorGetterName(l)»();
                «ENDFOR»

                «FOR l : robot.links»
                    «l.velocity».setZero();
                «ENDFOR»
            }

            «IF floatingBase»
                «methodsDefinition_floatingBase()»
            «ELSE»
                «methodsDefinition_fixedBase()»
            «ENDIF»
            '''

    private static CharSequence name_baseWrench  = '''baseWrench'''
    private static CharSequence name_jointsForce = '''jForces'''
    private static CharSequence name_baseAccel   = '''baseAccel'''
    def private fParam_q()   '''const «jState»& q'''
    def private fParam_qd()  '''const «jState»& qd'''
    def private fParam_qdd() '''const «jState»& qdd'''
    def private fParam_tau() '''«jState»& «name_jointsForce»'''
    def private fParam_fext()'''const «extF_t»& fext'''
    def private fParam_basea()   '''Acceleration& «robot.base.acceleration»'''
    def private fParam_basea_const()   '''const Acceleration& «name_baseAccel»'''
    def private fParam_basev()   '''const Velocity& «robot.base.velocity»'''
    def private fParam_basef()   '''Force& «name_baseWrench»'''
    def private fParam_gravity() '''const Acceleration& g'''



    def private methodsDefinition_fixedBase() '''
        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
        void «nsqualifier»::«className(robot)»::id(
            «fParam_tau»,
            «fParam_qd», «fParam_qdd»,
            «fParam_fext»)
        {
            firstPass(qd, qdd);
            // Add the external forces:
            «addFextCode()»
            secondPass(«name_jointsForce»);
        }

        void «nsqualifier»::«className(robot)»::G_terms(«fParam_tau»)
        {
            «fixedBase_pass1_gravityTerms()»

            secondPass(«name_jointsForce»);
        }

        void «nsqualifier»::«className(robot)»::C_terms(«fParam_tau», «fParam_qd»)
        {
            «fixedBase_pass1_Cterms()»

            secondPass(«name_jointsForce»);
        }


        void «nsqualifier»::«className(robot)»::firstPass(«fParam_qd», «fParam_qdd»)
        {
            «fixedBase_pass1()»
        }

        void «nsqualifier»::«className(robot)»::secondPass(«fParam_tau»)
        {
            «fixedBase_pass2()»
        }
    '''

    def private methodsDefinition_floatingBase() '''
        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
        void «nsqualifier»::«className(robot)»::id(
            «fParam_tau», «fParam_basea»,
            «fParam_gravity», «fParam_basev»,
            «fParam_qd», «fParam_qdd»,
            «fParam_fext»)
        {
            «IF floatingBase»
                «robot.base.inertiaC» = «robot.base.inertia»;
                «FOR l : robot.links»
                    «IF ! l.childrenList.children.empty»
                        «l.inertiaC» = «l.inertia»;
                    «ENDIF»
                «ENDFOR»
            «ENDIF»

            «floatingBase_pass1()»

            // Add the external forces:
            «addFextCode()»

            «floatingBase_pass2»

            «floatingBase_pass3»

            «robot.base.acceleration» += g;
        }


        void «nsqualifier»::«className(robot)»::G_terms_fully_actuated(
            «fParam_basef», «fParam_tau»,
            «fParam_gravity»)
        {
            const Acceleration& «robot.base.acceleration» = -g;

            «fixedBase_pass1_gravityTerms()»

            «robot.base.force» = «robot.base.inertia» * «robot.base.acceleration»;

            secondPass_fullyActuated(«name_jointsForce»);

            «name_baseWrench» = «robot.base.force»;
        }

        void «nsqualifier»::«className(robot)»::C_terms_fully_actuated(
            «fParam_basef», «fParam_tau»,
            «fParam_basev», «fParam_qd»)
        {
            «fixedBase_pass1_Cterms()»

            Utils::fillAsForceCrossProductMx(«robot.base.velocity», spareMx);
            «robot.base.force» = spareMx * «robot.base.inertia» * «robot.base.velocity»;

            secondPass_fullyActuated(«name_jointsForce»);

            «name_baseWrench» = «robot.base.force»;
        }

        void «nsqualifier»::«className(robot)»::id_fully_actuated(
                «fParam_basef», «fParam_tau»,
                «fParam_gravity», «fParam_basev», «fParam_basea_const»,
                «fParam_qd», «fParam_qdd», «fParam_fext»)
        {
            Acceleration «robot.base.acceleration» = «name_baseAccel» -g;

            «fixedBase_pass1()»

            // The base
            Utils::fillAsForceCrossProductMx(«robot.base.velocity», spareMx);
            «robot.base.force» = «robot.base.inertia» * «robot.base.acceleration» + spareMx * «robot.base.inertia» * «robot.base.velocity»;


            secondPass_fullyActuated(«name_jointsForce»);

            «name_baseWrench» = «robot.base.force»;
        }


        void «nsqualifier»::«className(robot)»::secondPass_fullyActuated(«fParam_tau»)
        {
            «fixedBase_pass2()»
        }
    '''


    def private fixedBase_pass1_gravityTerms() '''
        «FOR Link l : sortedLinks»
            // Link '«l.name»'
            «val parent  = l.parent»
            «val acceler = l.acceleration»
            «val child_X_parent = transformsMap.get(l)»
            «IF parent.equals(robot.base) && !floatingBase»
                «acceler» = («child_X_parent»).col(«rbd_ns»::LZ) * «rbd_ns»::g;
            «ELSE»
                «acceler» = («child_X_parent») * «parent.acceleration»;
            «ENDIF»
            «l.force» = «l.inertia» * «acceler»;
        «ENDFOR»
     '''

    def private fixedBase_pass1_Cterms() '''
        «FOR Link l : sortedLinks»
            «val parent   = l.parent»
            «val joint    = l.connectingJoint»
            «val velocity = l.velocity»
            «val acceler  = l.acceleration»
            «val child_X_parent = transformsMap.get(l)»
            «val jid = Common::jointIdentifier(joint)»
            «val subspaceIdx = iit::dsl::generator::cpp::Common::spatialVectIndex(joint)»
            // Link '«l.name»'
            «IF floatingBase»
                «velocity» = ((«child_X_parent») * «parent.velocity»);
                «velocity»(«subspaceIdx») += qd(«jid»);
                Utils::fillAsMotionCrossProductMx(«velocity», spareMx);
                «IF parent.equals(robot.base)»
                    «acceler» = (spareMx.col(«subspaceIdx») * qd(«jid»));
                «ELSE»
                    «acceler» = ((«child_X_parent») * «parent.acceleration») + (spareMx.col(«subspaceIdx») * qd(«jid»));
                «ENDIF»
                «l.force» = «l.inertia» * «acceler» + (-spareMx.transpose() * «l.inertia» * «velocity»);
            «ELSE»
                «IF parent.equals(robot.base)»
                    «velocity»(«subspaceIdx») = qd(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot

                    Utils::fillAsForceCrossProductMx(«velocity», spareMx);

                    «l.force» = ((spareMx * «l.inertia»).col(«subspaceIdx») * qd(«jid»));
                «ELSE»
                    «velocity» = ((«child_X_parent») * «parent.velocity»);
                    «velocity»(«subspaceIdx») += qd(«jid»);

                    Utils::fillAsMotionCrossProductMx(«velocity», spareMx);

                    «IF parent.parent.equals(robot.base)»
                        «acceler» = (spareMx.col(«subspaceIdx») * qd(«jid»));
                    «ELSE»
                        «acceler» = ((«child_X_parent») * «parent.acceleration») + (spareMx.col(«subspaceIdx») * qd(«jid»));
                    «ENDIF»

                    «l.force» = «l.inertia» * «acceler» + (-spareMx.transpose() * «l.inertia» * «velocity»);
                «ENDIF»
            «ENDIF»

        «ENDFOR»
    '''

    def private fixedBase_pass1() '''
        «FOR Link l : sortedLinks»
            «val parent   = l.parent»
            «val myJoint  = l.connectingJoint»
            «val velocity = l.velocity»
            «val acceler  = l.acceleration»
            «val child_X_parent = transformsMap.get(l)»
            «val jid = Common::jointIdentifier(myJoint)»
            «val subspaceIdx = iit::dsl::generator::cpp::Common::spatialVectIndex(myJoint)»
            // First pass, link '«l.name»'
            «IF parent.equals(robot.base) && !floatingBase»
                «acceler» = («child_X_parent»).col(«rbd_ns»::LZ) * «rbd_ns»::g;
                «acceler»(«subspaceIdx») += qdd(«jid»);
                «velocity»(«subspaceIdx») = qd(«jid»);   // «velocity» = vJ, for the first link of a fixed base robot
                Utils::fillAsForceCrossProductMx(«velocity», spareMx);

                «l.force» = «l.inertia» * «acceler» + ((spareMx * «l.inertia»).col(«subspaceIdx») * qd(«jid»));
            «ELSE»
                «velocity» = ((«child_X_parent») * «parent.velocity»);
                «velocity»(«subspaceIdx») += qd(«jid»);

                Utils::fillAsMotionCrossProductMx(«velocity», spareMx);

                «acceler» = ((«child_X_parent») * «parent.acceleration») + (spareMx.col(«subspaceIdx») * qd(«jid»));
                «acceler»(«subspaceIdx») += qdd(«jid»);

                «l.force» = «l.inertia» * «acceler» + (-spareMx.transpose() * «l.inertia» * «velocity»);
            «ENDIF»

        «ENDFOR»
    '''



    def private fixedBase_pass2() '''
        «FOR l : sortedLinks.reverseView»
            // Link '«l.name»'
            «val parent = l.parent»
            «val joint  = l.connectingJoint»
            «name_jointsForce»(«Common::jointIdentifier(joint)») = «l.force»(«iit::dsl::generator::cpp::Common::spatialVectIndex(joint)»);
            «IF ( ! parent.equals(robot.base)) || floatingBase»
                «val child_X_parent = transformsMap.get(l)»
                «parent.force» = «parent.force» + («child_X_parent»).transpose() * «l.force»;
            «ENDIF»
        «ENDFOR»
    '''

    def private floatingBase_pass1() '''
        «FOR Link l : sortedLinks»
            «val parent   = l.parent»
            «val myJoint  = l.connectingJoint»
            «val velocity = l.velocity»
            «val acceler  = l.acceleration»
            «val child_X_parent = transformsMap.get(l)»
            «val jid = Common::jointIdentifier(myJoint)»
            «val subspaceIdx = iit::dsl::generator::cpp::Common::spatialVectIndex(myJoint)»
            // First pass, link '«l.name»'
            «velocity» = ((«child_X_parent») * «parent.velocity»);
            «velocity»(«subspaceIdx») += qd(«jid»);

            Utils::fillAsMotionCrossProductMx(«velocity», spareMx);

            «IF parent.equals(robot.base)/* parent is the floating base */»
                «acceler» = (spareMx.col(«subspaceIdx») * qd(«jid»));
            «ELSE»
                «acceler» = ((«child_X_parent») * «parent.acceleration») + (spareMx.col(«subspaceIdx») * qd(«jid»));
            «ENDIF»
            «acceler»(«subspaceIdx») += qdd(«jid»);

            «l.force» = «l.inertia» * «acceler» + (-spareMx.transpose() * «l.inertia» * «velocity»);

        «ENDFOR»
        // The force exerted on the floating base by the links
        «val base = robot.base»
        Utils::fillAsMotionCrossProductMx(«base.velocity», spareMx);
        «base.force» = -spareMx.transpose() * «base.inertia» * «base.velocity»;

    '''

    def private floatingBase_pass2() '''
        «FOR l : sortedLinks.reverseView()»
            «val parent = l.parent»
            «val child_X_parent = transformsMap.get(l)»
            «parent.inertiaC» = «parent.inertiaC» + («child_X_parent»).transpose() * «l.inertiaC» * («child_X_parent»);
            «parent.force» = «parent.force» + («child_X_parent»).transpose() * «l.force»;

        «ENDFOR»
    '''

    def private floatingBase_pass3() '''
        // The base acceleration due to the force due to the movement of the links
        «robot.base.acceleration» = - «robot.base.inertiaC».inverse() * «robot.base.force»;

        «FOR l : sortedLinks»
            «val parent = l.parent»
            «val joint  = l.connectingJoint»
            «val child_X_parent = transformsMap.get(l)»
            «val idx = iit::dsl::generator::cpp::Common::spatialVectIndex(joint)»
            «l.acceleration» = «child_X_parent» * «parent.acceleration»;
            «name_jointsForce»(«Common::jointIdentifier(joint)») = («l.inertiaC».row(«idx») * «l.acceleration» + «l.force»(«idx»));

        «ENDFOR»
    '''

    def private setJointStatusCode() '''
        «FOR l : sortedLinks»
            («transformsMap.get(l)»)(q);
        «ENDFOR»
    '''

    def private addFextCode() '''
        «IF floatingBase»
            «robot.base.force» -= fext[«Common::linkIdentifier(robot.base)»];
        «ENDIF»
        «FOR l : sortedLinks»
            «l.force» -= fext[«Common::linkIdentifier(l)»];
        «ENDFOR»
    '''


    def private motionTransformsMember() '''xm'''
    def private Xmotion(iit.dsl.coord.coordTransDsl.Transform t)
        '''«motionTransformsMember»->«iit::dsl::coord::generator::cpp::EigenFiles::memberName(t)»'''

    def private void loadInfo(Robot rob, iit.dsl.coord.coordTransDsl.Model transforms)
    {
        if(robot == rob) return;
        robot = rob
        dofs = robot.DOFs
        jointDOFs = robot.jointDOFs
        floatingBase = robot.base.floating
        transformsModel = transforms
        sortedLinks = robot.links.sortBy(link | getID(link))
        for(l : robot.links) {
            transformsMap.put(l, Xmotion(Transforms::getTransform(transformsModel, l, l.parent)))
        }
    }

    def main_benchmarkID(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::invDynHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static void fillState(«robotNS»::«Names$Types::jointState»& q, «robotNS»::«Names$Types::jointState»& qd, «robotNS»::«Names$Types::jointState»& qdd);
        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject);

        /* This main is supposed to be used to test the inverse dynamics routines */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the number of tests to perform" << endl;
                return -1;
            }

            int numOfTests = std::atoi(argv[1]);
            double me[numOfTests];
            int iterations[numOfTests];

            double t0, duration, my_total;
            my_total = 0;

            «robotNS»::«Names$Types::jointState» q, qd, qdd, tau;
            «robotNS»::«Names$Namespaces::dynamics»::«className(robot)» myDynamics;

            int t=0,i=0;

            std::srand(std::time(NULL)); // initialize random number generator

            int numOfIterations = 1;
            for(t=0; t<numOfTests; t++) {
                my_total = 0;
                numOfIterations = numOfIterations * 10;
                iterations[t] = numOfIterations;

                for(i=0; i<numOfIterations; i++) {
                    fillState(q, qd, qdd);
                    t0 = std::clock();
                    myDynamics.id(q, qd, qdd, tau);
                    duration = std::clock() - t0;
                    my_total += duration;
                }
                me[t] = my_total/CLOCKS_PER_SEC;
            }

           matlabLog(numOfTests, iterations, me, "inv_dyn");

            for(t=0; t<numOfTests; t++) {
                cout << me[t] << endl;
            }

            return 0;
        }


        void fillState(«robotNS»::«Names$Types::jointState»& q, «robotNS»::«Names$Types::jointState»& qd, «robotNS»::«Names$Types::jointState»& qdd) {
            static const double max = 12.3;
            «FOR Joint j : robot.joints»
                q(«j.getID()-1»)   = ( ((double)std::rand()) / RAND_MAX) * max;
                qd(«j.getID()-1»)  = ( ((double)std::rand()) / RAND_MAX) * max;
                qdd(«j.getID()-1») = ( ((double)std::rand()) / RAND_MAX) * max;
            «ENDFOR»
        }

        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject) {
            «val prefix = robot.name.toLowerCase + "_test"»
            std::string fileName = "«robot.name»_" + subject + "_speed_test_data.m";
            ofstream out(fileName.c_str());
            out << "«prefix».robot       = '«robot.name»';" << std::endl;
            out << "«prefix».description = 'test of the speed of the calculation of: " << subject << "';" << std::endl;
            out << "«prefix».software    = 'code generated from the Kinematic DSL & Co.';" << std::endl;

            // Current date/time based on current system
            time_t now = std::time(0);
            tm* localtm = std::localtime(&now); // Convert now to tm struct for local timezone
            char timeStr[64];
            std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d  %X",localtm);
            out << "«prefix».date = '" << timeStr << "';" << std::endl;

            out << "«prefix».iterations = [";
            for(int t=0; t<numOfTests; t++) {
                out << " " << iterations[t];
            }
            out << "];" << endl;
            out << "«prefix».times = [";
            for(int t=0; t<numOfTests; t++) {
                out << " " << tests[t];
            }
            out << "];" << endl;
            out.close();
        }
        '''






    def main_test(Robot robot) '''
        #include <cmath>
        #include <iostream>

        #include "«Names$Files$RBD::invDynHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»;
        using namespace «Names$Namespaces$Qualifiers::robot(robot)»::«Names$Namespaces::dynamics»;
        using namespace «Names$Namespaces$Qualifiers::iit_rbd»;

        int main(int argc, char** argv) {
            «Names$Types::jointState» q, qd, qdd, tau, tau2;
            «FOR Joint j : robot.joints»
            q(«j.getID()-1»)   = std::atof(argv[«j.getID()»]);
            qd(«j.getID()-1»)  = std::atof(argv[«j.getID() + robot.joints.size»]);
            qdd(«j.getID()-1») = std::atof(argv[«j.getID() + robot.joints.size + robot.joints.size»]);
            «ENDFOR»

            «Names$Types$Transforms::spatial_motion» transforms;
            «className(robot)» foo(transforms);

            foo.id(q,qd,qdd,tau);
            std::cout << "Full inverse dynamics terms:" << std::endl << tau << std::endl;
            foo.G_terms(q, tau);
            std::cout << std::endl << "Gravity terms:" << std::endl << tau << std::endl;
            foo.C_terms(q, qd, tau2);
            std::cout << std::endl << "Centrifugal/Coriolis terms:" << std::endl << tau2 << std::endl;

            std::cout << std::endl << "-G + C:" << std::endl << -tau + tau2 << std::endl;
            return 0;
        }'''

    def main_sine_task(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        «val robotdynNS = robotNS + "::" + Names$Namespaces::dynamics»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files$RBD::invDynHeader(robot)».h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static void matlabLog(«robotNS»::«Names$Types::jointState»* q,
        «robotNS»::«Names$Types::jointState»* qd,
        «robotNS»::«Names$Types::jointState»* qdd,
        «robotNS»::«Names$Types::jointState»* tau,
        int count);

        static const double vis_coef = 0.1;
        static const double sampF = 250;
        static const double deltaT = 1/sampF;

        /* Applies a sinusoidal trajectory to the joints, to test the inverse dynamics routine */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the duration of the task" << endl;
                return -1;
            }
            double duration = std::atof(argv[1]);
            int last = duration/deltaT;


            «FOR Joint j : robot.joints»
                double offs_«j.name» = 0;
                double ampl_«j.name» = 0;
                double phas_«j.name» = 0;
                double freq_«j.name» = 1;

            «ENDFOR»
            «robotNS»::«Names$Types::jointState»* q   = new «robotNS»::«Names$Types::jointState»[last];
            «robotNS»::«Names$Types::jointState»* qd  = new «robotNS»::«Names$Types::jointState»[last];
            «robotNS»::«Names$Types::jointState»* qdd = new «robotNS»::«Names$Types::jointState»[last];
            «robotNS»::«Names$Types::jointState»* tau = new «robotNS»::«Names$Types::jointState»[last];

            «robotdynNS»::«className(robot)» myDynamics;

            «Names$Namespaces$Qualifiers::iit_rbd»::ForceVector forceVec;
            forceVec.setZero();
            «robotdynNS»::«Names$Types::extForces» extForces(forceVec);

            std::srand(std::time(NULL)); // initialize random number generator

            double t = 0;
            for(int i=0; i<last; i++) {
                «FOR Joint j : robot.joints»
                    q[i](«robotNS»::«Common::jointIdentifier(j)») = offs_«j.name» + ampl_«j.name» *
                            std::sin(2 * M_PI * freq_«j.name» * t + phas_«j.name»);

                    qd[i](«robotNS»::«Common::jointIdentifier(j)») = ampl_«j.name» * 2 * M_PI * freq_«j.name» *
                            std::cos(2 * M_PI * freq_«j.name» * t + phas_«j.name»);

                    qdd[i](«robotNS»::«Common::jointIdentifier(j)») = - ampl_«j.name» * (4 * M_PI * M_PI * freq_«j.name»* freq_«j.name») *
                            std::sin(2 * M_PI * freq_«j.name» * t + phas_«j.name»);

                    forceVec.setZero();
                    forceVec(«iit::dsl::generator::cpp::Common::spatialVectIndex(j)») = - vis_coef * qd[i](«robotNS»::«Common::jointIdentifier(j)»);
                    extForces[«robotNS»::«Common::linkIdentifier(j.successorLink)»] = forceVec;
                «ENDFOR»
                myDynamics.id(q[i], qd[i], qdd[i], extForces, tau[i]);
                t += deltaT;
            }

           matlabLog(q, qd, qdd, tau, last);

            delete[] q;
            delete[] qd;
            delete[] qdd;
            delete[] tau;
            return 0;
        }

        static void matlabLog(«robotNS»::«Names$Types::jointState»* q,
            «robotNS»::«Names$Types::jointState»* qd,
            «robotNS»::«Names$Types::jointState»* qdd,
            «robotNS»::«Names$Types::jointState»* tau,
            int count)
        {
            «val prefix = robot.name.toLowerCase + "_test"»
            std::string fileName = "«robot.name»_sine_task_ID.m";
            ofstream out(fileName.c_str());
            out << "«prefix».robot       = '«robot.name»';" << std::endl;
            out << "«prefix».description = 'sine trajectory simulation with computation of Inverse Dynamics';" << std::endl;
            out << "«prefix».software    = 'code generated from the Kinematic DSL & Co.';" << std::endl;

            // Current date/time based on current system
            time_t now = std::time(0);
            tm* localtm = std::localtime(&now); // Convert now to tm struct for local timezone
            char timeStr[64];
            std::strftime(timeStr, sizeof(timeStr), "%Y-%m-%d  %X",localtm);
            out << "«prefix».date = '" << timeStr << "';" << std::endl;

            out << "«prefix».q = [";
            for(int t=0; t<count; t++) {
                out << "["
                «FOR Joint j : robot.joints»
                    << " " << q[t](«robotNS»::«Common::jointIdentifier(j)»)
                «ENDFOR»
                << "];" << endl;
            }
            out << "];";

            out << "«prefix».qd = [";
            for(int t=0; t<count; t++) {
                out << "["
                «FOR Joint j : robot.joints»
                    << " " << qd[t](«robotNS»::«Common::jointIdentifier(j)»)
                «ENDFOR»
                << "];" << endl;
            }
            out << "];";

            out << "«prefix».qdd = [";
            for(int t=0; t<count; t++) {
                out << "["
                «FOR Joint j : robot.joints»
                    << " " << qdd[t](«robotNS»::«Common::jointIdentifier(j)»)
                «ENDFOR»
                << "];" << endl;
            }
            out << "];";

            out << "«prefix».tau = [";
            for(int t=0; t<count; t++) {
                out << "["
                «FOR Joint j : robot.joints»
                    << " " << tau[t](«robotNS»::«Common::jointIdentifier(j)»)
                «ENDFOR»
                << "];" << endl;
            }
            out << "];";

            out.close();
        }
    '''

    private Robot robot
    private int dofs
    private int jointDOFs
    private boolean floatingBase
    private iit.dsl.coord.coordTransDsl.Model transformsModel
    private List<Link> sortedLinks
    private Map<Link, CharSequence> transformsMap = new HashMap<Link, CharSequence>();

    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    private extension VariableNames = new VariableNames

    private static String rbd_ns = Names$Namespaces$Qualifiers::iit_rbd
    private static String jState = Names$Types::jointState
    private static String extF_t = Names$Types::extForces
}