package iit.dsl.generator.cpp.dynamics

import java.util.List

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Link
import iit.dsl.generator.cpp.Names
import iit.dsl.generator.cpp.Common
import iit.dsl.generator.cpp.RobotHeaders
import iit.dsl.generator.common.Transforms
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.Joint

class ForwardDynamics {
    def static className(Robot r) '''ForwardDynamics'''


    def public headerContent(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        «loadInfo(robot)»
        #ifndef IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files$RBD::fwdDynHeader(robot).toUpperCase()»_H_
        #define IIT_ROBOT_«robot.name.toUpperCase()»_«Names$Files$RBD::fwdDynHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>
        #include <iit/rbd/rbd.h>
        #include <iit/rbd/InertiaMatrix.h>
        #include <iit/rbd/utils.h>

        #include "«Names$Files::mainHeader(robot)».h"
        #include "«Names$Files::transformsHeader(robot)».h"
        #include "«Names$Files$RBD::inertiaHeader(robot)».h"
        #include "«Names$Files::linkDataMapHeader(robot)».h"

        «Common::enclosingNamespacesOpen(robot)»
        namespace «Names$Namespaces::dynamics» {

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
            // Convenient type aliases:
            typedef «RobotHeaders::linkDataMap_type()»<«rbd_ns»::ForceVector> «extF_t»;
            typedef «rbd_ns»::ForceVector Force;
            typedef «rbd_ns»::VelocityVector Velocity;
            typedef «rbd_ns»::VelocityVector Acceleration;
        public:
            /**
             * Default constructor
             * \param in the inertia properties of the links
             * \param tr the container of all the spatial motion transforms of
             *     the robot «robot.name», which will be used by this instance
             *     to compute the dynamics.
             */
            «className(robot)»(«LinkInertias::className(robot)»& in, «Names$Types$Transforms::spatial_motion»& tr);
            /** \name Forward dynamics
             * The Articulated-Body-Algorithm to compute the joint accelerations
             */ ///@{
            /**
             * \param qdd the joint accelerations vector (output parameter).
             «IF floatingBase»
                  * \param «robot.base.acceleration»
                  * \param «robot.base.velocity»
                  * \param g the gravity acceleration vector, expressed in the
                  *          base coordinates
             «ENDIF»
             * \param q the joint status vector
             * \param qd the joint velocities vector
             * \param tau the joint forces (torque or force)
             * \param fext the external forces, optional. Each force must be
             *              expressed in the reference frame of the link it is
             *              exerted on.
             */
             «val params ='''const «jState»& qd, const «jState»& tau, const «extF_t»& fext = zeroExtForces'''»
             «val fb_out_params = '''«jState»& qdd, Acceleration& «robot.base.acceleration», // output parameters'''»
             «val fb_in_params  = '''const Velocity& «robot.base.velocity», const Acceleration& g'''»
            «IF floatingBase»
                void fd(
                   «fb_out_params»,
                   «fb_in_params»,
                   const «jState»& q, «params»);
                void fd(
                    «fb_out_params»,
                    «fb_in_params»,
                    «params»);
            «ELSE»
                void fd(
                    «jState»& qdd, // output parameter
                    const «jState»& q, «params»);
                void fd(
                    «jState»& qdd, // output parameter
                    «params»);
            «ENDIF»
            ///@}

            /** Updates all the kinematics transforms used by this instance. */
            void setJointStatus(const «jState»& q) const;

        private:
            «LinkInertias::className(robot)»* «linkInertiasMember»;
            «Names$Types$Transforms::spatial_motion»* «motionTransformsMember»;

            «rbd_ns»::Matrix66d vcross; // support variable
            «IF robot.anyPrismaticJoint»
                «rbd_ns»::Matrix66d Ia_p;   // support variable, articulated inertia in the case of a prismatic joint
            «ENDIF»
            «IF robot.anyRevoluteJoint»
                «rbd_ns»::Matrix66d Ia_r;   // support variable, articulated inertia in the case of a revolute joint
            «ENDIF»
            «IF floatingBase»
                // Link '«robot.base.name»'
                «rbd_ns»::Matrix66d «artInertiaName(robot.base)»;
                Force «biasForceName(robot.base)»;
            «ENDIF»

            «FOR l : robot.links»
                // Link '«l.name»' :
                «rbd_ns»::Matrix66d «artInertiaName(l)»;
                Velocity «l.acceleration»;
                Velocity «l.velocity»;
                Velocity «cTermName(l)»;
                Force    «biasForceName(l)»;

                «rbd_ns»::Column6d «UTermName(l)»;
                double «DTermName(l)»;
                double «uTermName(l)»;
            «ENDFOR»
        private:
            static const «extF_t» zeroExtForces;
        };

        inline void «className(robot)»::setJointStatus(const «jState»& q) const {
            «setJointStatusCode(robot.links, transformsModel)»
        }

        «IF floatingBase»
            inline void «className(robot)»::fd(
                «fb_out_params»,
                «fb_in_params»,
                const «jState»& q,
                const «jState»& qd,
                const «jState»& tau,
                const «extF_t»& fext/* = zeroExtForces */)
            {
                setJointStatus(q);
                fd(qdd, «robot.base.acceleration», «robot.base.velocity», g, qd, tau, fext);
            }
        «ELSE»
            inline void «className(robot)»::fd(
                «jState»& qdd,
                const «jState»& q,
                const «jState»& qd,
                const «jState»& tau,
                const «extF_t»& fext/* = zeroExtForces */)
            {
                setJointStatus(q);
                fd(qdd, qd, tau, fext);
            }
        «ENDIF»

        }
        «Common::enclosingNamespacesClose(robot)»

        #endif
    '''

    def public implementationFileContent(
        Robot robot,
        iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        «loadInfo(robot)»
        #include "«Names$Files$RBD::fwdDynHeader(robot)».h"

        #include <Eigen/Cholesky>
        #include <iit/rbd/robcogen_commons.h>

        «val nsqualifier = Names$Namespaces$Qualifiers::robot(robot) + "::" + Names$Namespaces::dynamics»
        using namespace «rbd_ns»;

        // Initialization of static-const data
        const «nsqualifier»::«className(robot)»::«extF_t»
            «nsqualifier»::«className(robot)»::zeroExtForces(Force::Zero());

        «nsqualifier»::«className(robot)»::«className(robot)»(«LinkInertias::className(robot)»& inertia, «Names$Types$Transforms::spatial_motion»& transforms) :
            «linkInertiasMember»( & inertia ),
            «motionTransformsMember»( & transforms )
        {
            «FOR l : robot.links»
                «l.velocity».setZero();
                «cTermName(l)».setZero();
            «ENDFOR»

            vcross.setZero();
            «IF robot.anyPrismaticJoint»
                Ia_p.setZero();
            «ENDIF»
            «IF robot.anyRevoluteJoint»
                Ia_r.setZero();
            «ENDIF»

        }

        void «nsqualifier»::«className(robot)»::fd(
            «jState»& qdd,
            «IF floatingBase»
                Acceleration& «robot.base.acceleration»,
                const Velocity& «robot.base.velocity»,
                const Acceleration& g,
            «ENDIF»
            const «jState»& qd,
            const «jState»& tau,
            const «extF_t»& fext/* = zeroExtForces */)
        {
            «ABABody(robot, transformsModel)»
        }
    '''

    def private linkInertiasMember() '''inertiaProps'''
    def private motionTransformsMember() '''motionTransforms'''
    def private Xmotion(iit.dsl.coord.coordTransDsl.Transform t)
        '''«motionTransformsMember»-> «iit::dsl::coord::generator::cpp::EigenFiles::memberName(t)»'''


    def private ABABody(Robot robot, iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        «val rbd_ns = Names$Namespaces$Qualifiers::iit_rbd»
        «val sortedLinks = robot.links.sortBy(link | getID(link))»

        «IF floatingBase»
            «artInertiaName(robot.base)» = «linkInertiasMember»->«LinkInertias::tensorGetterName(robot.base)»();
            «biasForceName(robot.base)» = - fext[«Common::linkIdentifier(robot.base)»];
        «ENDIF»
        «FOR l : sortedLinks»
            «artInertiaName(l)» = «linkInertiasMember»->«LinkInertias::tensorGetterName(l)»();
            «biasForceName(l)» = - fext[«Common::linkIdentifier(l)»];
        «ENDFOR»
        // ---------------------- FIRST PASS ---------------------- //
        // Note that, during the first pass, the articulated inertias are really
        //  just the spatial inertia of the links (see assignments above).
        //  Afterwards things change, and articulated inertias shall not be used
        //  in functions which work specifically with spatial inertias.
        «FOR l : sortedLinks»
            «val parent   = l.parent»
            «val joint    = l.connectingJoint»
            «val velocity = l.velocity»
            «val cterm    = cTermName(l)»
            «val biasF    = biasForceName(l)»
            «val jid      = Common::jointIdentifier(joint)»
            «val child_X_parent = Xmotion(Transforms::getTransform(transformsModel, l, parent))»

            «val subspaceIdx = Common::spatialVectIndex_no_ns(joint)»
            // + Link «l.name»
            «IF parent.equals(robot.base) && (!floatingBase)»
                //  - The spatial velocity:
                «velocity»(«subspaceIdx») = qd(«jid»);

                //  - The bias force term:
                «IF joint.prismatic»
                    // The first joint is prismatic, no bias force term
                «ELSE»
                    «biasF» += vxIv(qd(«jid»), «l.artInertiaName»);
                «ENDIF»
            «ELSE»
                //  - The spatial velocity:
                «velocity» = («child_X_parent») * «parent.velocity»;
                «velocity»(«subspaceIdx») += qd(«jid»);

                //  - The velocity-product acceleration term:
                motionCrossProductMx(«velocity», vcross);
                «cterm» = vcross.col(«subspaceIdx») * qd(«jid»);

                //  - The bias force term:
                «biasF» += vxIv(«velocity», «l.artInertiaName»);
            «ENDIF»
        «ENDFOR»

        «IF floatingBase»
            // + The floating base body
            «biasForceName(robot.base)» += vxIv(«robot.base.velocity», «robot.base.artInertiaName»);
        «ENDIF»

        // ---------------------- SECOND PASS ---------------------- //
        Matrix66d IaB;
        Force pa;
        «FOR l : sortedLinks.reverseView»
            «val joint = l.connectingJoint»
            «val subspaceIdx = Common::spatialVectIndex_no_ns(joint)»
            «val U = UTermName(l)»
            «val D = DTermName(l)»
            «val u = uTermName(l)»
            «val p = biasForceName(l)»
            «val I = artInertiaName(l)»
            «val parent = l.parent»
            «val child_X_parent = Xmotion(Transforms::getTransform(transformsModel, l, parent))»
            «check_joint(joint)»

            // + Link «l.name»
            «u» = tau(«Common::jointIdentifier(joint)») - «p»(«subspaceIdx»);
            «U» = «I».col(«subspaceIdx»);
            «D» = «U»(«subspaceIdx»);

            «IF !parent.equals(robot.base) || floatingBase»
                «IF joint instanceof PrismaticJoint»
                    compute_Ia_prismatic(«I», «U», «D», Ia_p);  // same as: Ia_p = «I» - «U»/«D» * «U».transpose();
                    pa = «p» + Ia_p * «cTermName(l)» + «U» * «u»/«D»;
                    ctransform_Ia_prismatic(Ia_p, «child_X_parent», IaB);
                «ELSE»
                    compute_Ia_revolute(«I», «U», «D», Ia_r);  // same as: Ia_r = «I» - «U»/«D» * «U».transpose();
                    pa = «p» + Ia_r * «cTermName(l)» + «U» * «u»/«D»;
                    ctransform_Ia_revolute(Ia_r, «child_X_parent», IaB);
                «ENDIF»
                «artInertiaName(parent)» += IaB;
                «biasForceName(parent)» += («child_X_parent»).transpose() * pa;
            «ENDIF»
        «ENDFOR»

        «IF floatingBase»
            // + The acceleration of the floating base «robot.base.name», without gravity
            Eigen::LLT<Matrix66d> llt(«artInertiaName(robot.base)»);
            «robot.base.acceleration» = - llt.solve(«biasForceName(robot.base)»);  // «robot.base.acceleration» = - IA^-1 * «biasForceName(robot.base)»
        «ENDIF»

        // ---------------------- THIRD PASS ---------------------- //
        «FOR l : sortedLinks»
            «val parent = l.parent»
            «val joint  = l.connectingJoint»
            «val jid    = Common::jointIdentifier(joint)»
            «val child_X_parent = Xmotion(Transforms::getTransform(transformsModel, l, parent))»
            «IF parent.equals(robot.base) && (!floatingBase)»
                «l.acceleration» = («child_X_parent»).col(LZ) * («rbd_ns»::g);
            «ELSE»
                «l.acceleration» = («child_X_parent») * «acceleration(parent)» + «cTermName(l)»;
            «ENDIF»
            qdd(«jid») = («uTermName(l)» - «UTermName(l)».dot(«l.acceleration»)) / «DTermName(l)»;
            «l.acceleration»(«Common::spatialVectIndex_no_ns(joint)») += qdd(«jid»);

        «ENDFOR»

        «IF floatingBase»
            // + Add gravity to the acceleration of the floating base
            «robot.base.acceleration» += g;
        «ENDIF»
    '''

    def private setJointStatusCode(List<Link> sortedLinks, iit.dsl.coord.coordTransDsl.Model transformsModel)
    '''
        «FOR l : sortedLinks»
            «val child_X_parent = Xmotion(Transforms::getTransform(transformsModel, l, l.parent))»
            («child_X_parent»)(q);
        «ENDFOR»
    '''

    /* This one is just to make sure we don't keep using this code generator
     * should a new joint type was introduced in RobCoGen
     */
    def private check_joint(Joint j) {
        if( ! (j instanceof PrismaticJoint ||
               j instanceof RevoluteJoint) )
        {
            throw new RuntimeException("during forward dynamics code generation: unknown joint type")
        }
    }

    def private cTermName(AbstractLink l) '''«l.name»_c'''
    def private artInertiaName(AbstractLink l) '''«l.name»_AI'''
    def private biasForceName(AbstractLink l) '''«l.name»_p'''
    def private UTermName(AbstractLink l) '''«l.name»_U'''
    def private DTermName(AbstractLink l) '''«l.name»_D'''
    def private uTermName(AbstractLink l) '''«l.name»_u'''

    /*
     * Saves in the members of this instance the relevant information about
     * the given robot
     */
    def private void loadInfo(Robot robot) {
        if(currRobot == robot) return;
        currRobot = robot
        dofs = robot.DOFs
        jointDOFs = robot.jointDOFs
        floatingBase = robot.base.floating
        /*
        if(floatingBase) {
            links = robot.abstractLinks
        } else {
            links = new ArrayList<AbstractLink>()
            links.addAll(robot.links)
        }*/

    }

    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    private extension VariableNames varNames = new VariableNames()

    private Robot currRobot
    private int dofs
    private int jointDOFs
    private boolean floatingBase

    private static String rbd_ns = Names$Namespaces$Qualifiers::iit_rbd
    private static String jState = Names$Types::jointState
    private static String extF_t = Names$Types::extForces

/*
    private static List<CharSequence> fd_fb_params = Arrays::asList(
        '''const «rbd_ns»::VelocityVector& v0''',
        '''const «rbd_ns»::VelocityVector& g''')

    private static List<CharSequence> fd_common_params = Arrays::asList(
        '''const «jState»& qd''',
        '''const «jState»& tau''',
        '''«jState»& qdd''',
        '''const «extF_t»& fext'''
        )
*/

}