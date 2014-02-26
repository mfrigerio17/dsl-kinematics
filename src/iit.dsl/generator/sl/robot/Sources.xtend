package iit.dsl.generator.sl.robot

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.cpp.dynamics.LinkInertias
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.cpp.Names
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.sl.Common
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.generator.sl.Utilities

class Sources {
    public static CharSequence world_X_base = '''SL::world_X_base'''

    /**
     * The code of the source file with kinematics computations (equivalent to
     * the old SL_kinematics_body.h)
     */
    def public kinematics(Robot robot) '''
        #include <iit/commons/SL/eigen_conversion.h>
        #include <iit/commons/SL/rbd_conversion.h>
        #include <iit/commons/SL/joint_status_conversion.h>

        «val include_dir = "iit/robots/" + Names$Files::folder(robot)»
        #include <«include_dir»/«Names$Files::mainHeader(robot)».h>
        #include <«include_dir»/«Names$Files::transformsHeader(robot)».h>
        #include <«include_dir»/«Names$Files::jacobiansHeader(robot)».h>
        #include <«include_dir»/«Names$Files::traitsHeader(robot)».h>
        #include <«include_dir»/«Names$Files$RBD::inertiaHeader(robot)».h>

        #include <iit/rbd/utils.h>

        #include "«Common::robogenGlobalsFileName(robot)».h"

        #include <iostream>

        // the system headers
        #include "SL_system_headers.h"
        #include "SL.h"
        #include "SL_user.h"
        #include "SL_common.h"
        #include "mdefs.h"
        #include "SL_kinematics.h"
        #include "utility_macros.h"

        using namespace «iit::dsl::generator::cpp::Common::enclosingNamespacesQualifier»;
        using namespace «iit::dsl::generator::cpp::Common::enclosingNamespacesQualifier(robot)»;


        static «Names$Types::jointState» q;
        static «Names$Namespaces::dynamics»::«LinkInertias::className(robot)» inertias;

        typedef typename commons::SL::SLtoRobogen<Traits> SLtoRGen;

        /*!
         * Original documentation:
         *
         *        computes the m*cog, rotation axis, and local coord.sys. orgin for
         *        every link. This information can be used to compute the COG and
         *        COG jacobians, assuming the mass and center of mass parameters are
         *        properly identified.
         *
         * \param[in]     state   : the state containing th, thd, thdd
         * \param[in]     basec   : the position state of the base
         * \param[in]     baseo   : the orientational state of the base
         * \param[in]     endeff  : the endeffector parameters
         * \param[out]    Xmcog   : array of mass*cog vectors
         * \param[out]    Xaxis   : array of rotation axes
         * \param[out]    Xorigin : array of coord.sys. origin vectors
         * \param[out]    Xlink   : array of link position
         * \param[out]    Ahmat   : homogeneous transformation matrices of each link
         *
         *
         * Marco's notes:
         * I am not really sure what is the difference between Xorigin and Xlink
         */
        void linkInformation(
                SL_Jstate *state,
                SL_Cstate *basec,
                SL_quat *baseo,
                SL_endeff *eff,
                double **Xmcog, double **Xaxis, double **Xorigin, double **Xlink,
                double ***Ahmat, double ***Ahmatdof)
        {
            «val rob_ns = Names$Namespaces$Qualifiers::robot(robot)»
            // Convenient alias of the global variable
            «rob_ns»::«Names$Types$Transforms::homogeneous»& HT = * «rob_ns»::SL::homogeneousTransforms;
            // Copy the joint status
            SLtoRGen::pos(state, q);
            // Support vector
            Eigen::Matrix<double,4,1> tmp_vec;

            «IF robot.base.floating»
                «kinematics_depthVisit(rob_ns, robot.base, world_X_base)»
            «ELSE»
                «val tmpX = "tmpX"»
                static «rob_ns»::«Names$Types$Transforms::homogeneous»::MatrixType «tmpX» = «rob_ns»::«Names$Types$Transforms::homogeneous»::MatrixType::Identity();
                «kinematics_depthVisit(rob_ns, robot.base, tmpX)»
            «ENDIF»
        }
    '''

    def private tempXName(ChildSpec spec) '''tmpX_«spec.joint.ID»'''

    def private CharSequence kinematics_depthVisit(
        CharSequence rob_ns,
        AbstractLink parent,
        CharSequence world_X_parent)
    {
        val children = parent.childrenList.children
        if(children.size == 0) {
            return '''//TODO  add the code for the endeffector links!!'''
        }
        val branching = children.size > 1
        var CharSequence world_X_link = world_X_parent
        val text = new StringBuffer()
        if(branching) {
            for(childSpec : children) {
                text.append('''«rob_ns»::«Names$Types$Transforms::homogeneous»::MatrixType «tempXName(childSpec)»;''')
                text.append("\n")
            }
            text.append("\n\n")
        }

        for(childSpec : children) {
            val child = childSpec.link
            if(branching) {
                world_X_link = tempXName(childSpec)
            }

            text.append('''
            // The transform from «child.name» to world
            «world_X_link» = «world_X_parent» * HT.«iit::dsl::generator::cpp::kinematics::Transforms::parent_X_child__mxName(parent,child)»(q);

            «kinematics_linkBlock(child, childSpec.joint, world_X_link)»

            «kinematics_depthVisit(rob_ns, child, world_X_link)»''')
        }

        return text
    }

    /*
     * Code for the Xaxis, Xorigin, Xlink, Amath and Xmcog variables, for the
     * given link/joint.
     */
    def private kinematics_linkBlock(AbstractLink link, Joint dof, CharSequence world_X_link) '''
        commons::SL::copy(Xaxis  [::«Common::jointEnumID(dof)»], «iit_rbd_ns»::Utils::zAxis(«world_X_link») );
        commons::SL::copy(Xorigin[::«Common::jointEnumID(dof)»], «iit_rbd_ns»::Utils::positionVector(«world_X_link») );

        commons::SL::copy(Xlink[::«Common::linkEnumID(link)»], «iit_rbd_ns»::Utils::positionVector(«world_X_link») );
        commons::SL::copy(Ahmat[::«Common::linkEnumID(link)»], «world_X_link»);

        tmp_vec.block<3,1>(0,0) = inertias.«LinkInertias::comGetterName(link)»() * inertias.«LinkInertias::massGetterName(link)»();
        tmp_vec(3) = inertias.«LinkInertias::massGetterName(link)»();
        commons::SL::copy(Xmcog[::«Common::jointEnumID(dof)»], «world_X_link» * tmp_vec);
    '''



    def public dynamics(Robot robot) '''
        «val dyn_ns = Names$Namespaces::dynamics»
        «val rbd_ns = Names$Namespaces$Qualifiers::iit_rbd()»
        «val floating = robot.base.floating»
        /************************************************************* STRT CPYHDR
        *
        * SL - realtime robot control and simulation framework
        * (c) 2010 Stefan Schaal, all rights reserved
        *
        * Copy, use and distribution of SL, both in source and binary form is
        * not permitted without explicit permission by the copyright holder.
        *
        * Please contact Stefan Schaal <sschaal@usc.edu>
        * for licensing information.
        *
        *
        ************************************************************* EOF CPYHDR */

        «val include_dir = Names$Files::folder(robot)»
        #include <iit/robots/«include_dir»/«Names$Files::mainHeader(robot)».h>
        #include <iit/robots/«include_dir»/«Names$Files$RBD::fwdDynHeader(robot)».h>
        #include <iit/robots/«include_dir»/«Names$Files::traitsHeader(robot)».h>

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/utils.h>

        #include <iit/commons/SL/rbd_conversion.h>
        #include <iit/commons/matrix_utils.h>
        #include <iit/commons/SL/rbd_conversion.h>
        #include <iit/commons/SL/joint_status_conversion.h>
        #include <iit/commons/SL/generic_dynamics.h>

        #include "«Common::robogenGlobalsFileName(robot)».h"

        #include "SL_system_headers.h"
        #include "SL.h"
        #include "SL_dynamics.h"


        // global variables
        int    freeze_base               = FALSE;
        double freeze_base_pos[N_CART+1] = {0.0,0.0,0.0,0.0};
        double freeze_base_quat[N_QUAT+1] = {0.0,1.0,0.0,0.0,0.0};


        using namespace «iit::dsl::generator::cpp::Common::enclosingNamespacesQualifier»;
        using namespace «iit::dsl::generator::cpp::Common::enclosingNamespacesQualifier(robot)»;


        int init_dynamics( void )
        {
            setDefaultEndeffector();
            return TRUE;
        }

        /*!
         * Inverse dynamics
         *
         * Original documentation (Sept 2010):
         *
         * Standard Newton Euler inverse dynamics, which switches automatically between
         * floating base and fixed base robots
         *
         * \param[in]     cstate  : the current state (pass NULL to use only desired state)
         * \param[in,out] lstate  : the desired state
         * \param[in]     endeff  : the endeffector parameters
         * \param[in]     cbase   : the position state of the base
         * \param[in]     obase   : the orientational state of the base
         *
         * Returns:
         * The appropriate feedforward torques are added in the uff component of the lstate
         * structure.
         *
         */
        void SL_InvDyn(SL_Jstate *cstate, SL_DJstate *lstate, SL_endeff *leff,
              SL_Cstate *cbase, SL_quat *obase)
        {
            «IF floating»
                commons::SL::inverse_dynamics<Traits>::
                floating_base(*SL::invDynEngine, SL::world_X_base.block<3,3>(0,0),
                        cstate, lstate, cbase, obase);
            «ELSE»
                iit::commons::SL::inverse_dynamics<Traits>::
                    fixed_base(*SL::invDynEngine, cstate, lstate);
            «ENDIF»
        }


        /*!
         * Forward Dynamics
         *
         * Original documentation (date June 1999)
         *
         *         computes the forward dynamics accelerations
         *
         *
         *  \param[in,out] lstate  : the state containing th, thd, thdd, and receiving the
         *                           appropriate u
         *  \param[in,out] cbase   : the position state of the base
         *  \param[in,out] obase   : the orientational state of the base
         *  \param[in]     ux      : the external forces acting on each joint,
         *                           in world coordinates, e.g., as computed from contact
         *                           forces
         *  \param[in]     endeff  : the endeffector parameters
         *
         */
        void SL_ForDyn(
            SL_Jstate *lstate,
            SL_Cstate *cbase, SL_quat *obase,
            SL_uext *ux, SL_endeff *leff)
        {
            «IF floating»
                static «dyn_ns»::ForwardDynamics::ExtForces extForces(«rbd_ns»::ForceVector::Zero());
                // TODO convert the external forces!

                commons::SL::forward_dynamics<Traits>::
                        floating_base(*SL::fwdDynEngine, extForces,
                        SL::world_X_base.block<3,3>(0,0), lstate, cbase, obase);

                if(freeze_base) {
                    //trunk_a.setZero();
                    //trunk_v.setZero();
                    commons::SL::baseVelToSL(iit::rbd::VelocityVector::Zero(), *cbase, *obase);
                    commons::SL::baseAccelToSL(iit::rbd::VelocityVector::Zero(), *cbase, *obase);
                }
            «ELSE»
                iit::commons::SL::forward_dynamics<Traits>::
                    fixed_base(*SL::fwdDynEngine, lstate);
            «ENDIF»
        }


        /*!
         *  Original documentation (date  Sept 2010)
         *
         *
         * computes the generalized joint forces due to friction and spring terms, i.e.,
         * the sum of all forces that act per joint independently of all others. The sign
         * of the terms is as if they were on the LEFT side of the RBD equations:
         *
         * M qdd + C qd + G + f = u
         *
         *
         *  \param[in] state       : the joint state of the robot
         *  \param[in] li          : the link parameters for this joint
         *
         *  returns the generalized joint force for this joint due friction and spring terms
         *
         */
        double compute_independent_joint_forces(SL_Jstate state, SL_link li)
        {
          double f=0;

          f = state.thd*li.vis +
            COULOMB_FUNCTION(state.thd)*li.coul +
            state.th*li.stiff +
            li.cons;

          return f;
        }
    '''


    /**
     * The code for SL_user_common
     */
    def SL_user_common(Robot robot)'''
        #include "SL.h"
        #include "SL_user.h"
        #include "SL_common.h"
        #include "SL_dynamics.h" // only because of setDefaultEndeffector()


        char joint_names[][20]= {
            {"BASE"}
            «FOR Joint joint : robot.joints»
            ,{"«joint.name»"}
            «ENDFOR»
        };

        char cart_names[][20]= {
            {"dummy"},
            {"ENDEFF"}
        };

        char link_names[][20]= {
            {"BASE"},
            «FOR link : robot.links»
                {"LNK_«link.name»"},
            «ENDFOR»
            {"LNK_dummy_ee"}
        };
        // don't really know what this is for, but it is required for linking...
        char blob_names[][20]= {
          {"dummy"},
          {"BLOB1"},
          {"BLOB2"},
          {"BLOB3"},
          {"BLOB4"},
          {"BLOB5"},
          {"BLOB6"}
        };

        char misc_sensor_names[][20]= {
            {"dummy"},
            «FOR s : Utilities::defaultMiscSensors SEPARATOR ','»
                {"«s»"}
            «ENDFOR»
        };

        int link2endeffmap[] = {0,«Common::DUMMY_EE_LINK_ID»};
        double test_goal[N_CART+1];
        int    no_user_interaction_flag=FALSE;
        int    real_time_flag=FALSE;
        char   initial_task_name[100];

        /* the following include must be the last line of the variable declaration section */
        #include "SL_user_common.h"   /* do not erase!!! */


        void setDefaultEndeffector(void)
        {
            int i;
            for (i=1; i<=N_ENDEFFS; ++i) {
                endeff[i].m       = 0.0;
                endeff[i].mcm[_X_]= 0.0;
                endeff[i].mcm[_Y_]= 0.0;
                endeff[i].mcm[_Z_]= 0.0;
                endeff[i].x[_X_]  = 0.0;
                endeff[i].x[_Y_]  = 0.0;
                endeff[i].x[_Z_]  = 0.0;
                endeff[i].a[_A_]  = 0.0;
                endeff[i].a[_B_]  = 0.0;
                endeff[i].a[_G_]  = 0.0;
            }
        }

        void setRealRobotOptions(void)
        {
            if (!real_robot_flag) {
                sprintf(config_files[CONFIGFILES],"ConfigFilesSim.cf");
            } else {
                sprintf(config_files[CONFIGFILES],"ConfigFiles.cf");
            }

            // update the config file names
            read_config_files(config_files[CONFIGFILES]);
        }
        '''


    private extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    private static String iit_rbd_ns = Names$Namespaces$Qualifiers::iit_rbd

}