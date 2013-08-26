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
            SL::fromSLState(state, q);
            // Support vector
            Eigen::Matrix<double,4,1> tmp_vec;

            «kinematics_depthVisit(rob_ns, robot.base, world_X_base)»
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
        commons::SL::copy(Xaxis  [::«dof.name»], «iit_rbd_ns»::Utils::zAxis(«world_X_link») );
        commons::SL::copy(Xorigin[::«dof.name»], «iit_rbd_ns»::Utils::positionVector(«world_X_link») );

        commons::SL::copy(Xlink[::«Common::linkEnumID(link)»], «iit_rbd_ns»::Utils::positionVector(«world_X_link») );
        commons::SL::copy(Ahmat[::«Common::linkEnumID(link)»], «world_X_link»);

        tmp_vec.block<3,1>(0,0) = inertias.«LinkInertias::comGetterName(link)»() * inertias.«LinkInertias::massGetterName(link)»();
        tmp_vec(3) = inertias.«LinkInertias::massGetterName(link)»();
        commons::SL::copy(Xmcog[::«dof.name»], «world_X_link» * tmp_vec);
    '''


    /**
     * The code for SL_user_common
     */
    def SL_user_common(Robot robot)'''
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