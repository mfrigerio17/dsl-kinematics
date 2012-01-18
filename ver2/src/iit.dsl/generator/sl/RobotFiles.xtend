package iit.dsl.generator.sl

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint

import com.google.inject.Inject
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.kinDsl.Link

class RobotFiles {
    @Inject extension iit.dsl.generator.Common common
    @Inject extension iit.dsl.generator.sl.Common slCommon

    def dynModel(Robot robot) '''
        { (* Base Coordinate System *)
        {jointID,{ID=0}},
        {floatingBase,{«IF robot.base.floating»«ELSE»0«ENDIF»}},
        {notUsed,{}},
        {notUsed,{}},
        {successors,{1}},
        {inertia,GenInertiaMatrixS["links",ID,1]},
        {massCenterMass,GenMCMVectorS["links",ID,1]},
        {mass,GenMassS["links",ID]},
        {baseVariables,GenBaseVariablesS["basec","baseo",ID]},
        {extForce,GenExtForceS["uex",ID]}
        }
        «FOR Joint j : robot.joints»
            { (* Joint «j.name» *)
            {jointID,{ID=«j.ID»}},
            «j.jointAxis»,
            {translation,{«j.refFrame.translation.x.str»,«j.refFrame.translation.y.str»,«j.refFrame.translation.z.str»}},
            {rotationMatrix,{«j.refFrame.rotation.x.str»,«j.refFrame.rotation.y.str»,«j.refFrame.rotation.z.str»}},
            {successors,{«j.successors»}},
            {inertia,GenInertiaMatrixS["links",ID,1]},
            {massCenterMass,GenMCMVectorS["links",ID,1]},
            {mass,GenMassS["links",ID]},
            {jointVariables,GenVariablesS["state",ID]},
            {extForce,GenExtForceS["uex",ID]}
            }
        «ENDFOR»
        { (* dummy to draw endeffector *)
        {jointID,{ID=104}},
        {jointAxis,{0,0,0}},
        {translation,{eff$1$$x[[1]],eff$1$$x[[2]],eff$1$$x[[3]]}},
        {rotationMatrix,{eff$1$$a[[1]],eff$1$$a[[2]],eff$1$$a[[3]]}},
        {successors,{}},
        {inertia,{{0,0,0},{0,0,0},{0,0,0}}},
        {massCenterMass,{eff$1$$mcm[[1]],eff$1$$mcm[[2]],eff$1$$mcm[[3]]}},
        {mass,{eff$1$$m}},
        {jointVariables,{0,0,0,0,0}},
        {extForce,{0,0,0,0,0,0}}
        }
        '''

    def private dispatch jointAxis(PrismaticJoint j)'''
        {jointAxis,{0,0,0,0,0,1}}'''
    def private dispatch jointAxis(RevoluteJoint j)'''
        {jointAxis,{0,0,1}}'''
    def private getSuccessors(Joint j)'''
        «val children =  j.successorLink.childrenList.children»
        «IF children.empty»
        104
        «ELSE»
            «children.head().joint.ID»
            «FOR ChildSpec child : children.drop(1)»
                ,«child.joint.ID»
            «ENDFOR»
        «ENDIF»'''

    def mathematicaNotebook(Robot robot, String slRootPath) '''
        «val robotName = robot.name»
        (* Beginning of Notebook Content *)
        Notebook[{
        Cell[BoxData[
         RowBox[{
          RowBox[{
          "SetDirectory", "[", "\"\<«slRootPath»/RigidBodyDynamics/\>\"", "]"}],
          ";"}]], "Input",
         CellChangeTimes->{{3.4691655408842077`*^9, 3.4691655651588182`*^9}, {
          3.469166315104549*^9, 3.4691663196428537`*^9}, {3.499409252158552*^9,
          3.499409256220811*^9}, {3.525146342503011*^9, 3.52514635857464*^9}, {
          3.525152703171839*^9, 3.52515271342125*^9}}],

        Cell[BoxData[
         RowBox[{"<<", "RigidBodyDynamics.m"}]], "Input",
         CellChangeTimes->{{3.499409280874213*^9, 3.499409284632689*^9},
           3.4994102006706963`*^9, {3.4994103390985413`*^9, 3.499410339567994*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{
          RowBox[{"SetDirectory", "[", "\"\<«slRootPath»/«robot.robotFolderName»/math\>\"", "]"}],
          ";"}]], "Input",
         CellChangeTimes->{
          3.469165616187872*^9, 3.487616125166923*^9, 3.499410202079053*^9, {
           3.52514637103411*^9, 3.525146374304959*^9}}],

        Cell["", "Text",
         CellChangeTimes->{{3.487621177793989*^9, 3.487621180100277*^9}}],

        Cell[BoxData[
         RowBox[{"OpenGLKinematics", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582726736*^9, 3.4691656162179947`*^9, {3.525146378593598*^9,
           3.525146383686726*^9}}],

        Cell[BoxData[
         RowBox[{"InvDynNE", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.4691655827608423`*^9, 3.4691656162535877`*^9, {3.499409294531172*^9,
           3.4994092946474867`*^9}, {3.525146389247444*^9, 3.525146392188701*^9},
           3.525151813183488*^9, {3.525152860858181*^9, 3.525152863572134*^9}, {
           3.525155668635*^9, 3.525155669011446*^9}, {3.525155860832596*^9,
           3.525155863184689*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{"InvDynArt", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.4691655827923203`*^9, 3.469165616284869*^9, {3.525146395907315*^9,
           3.525146401733008*^9}}],

        Cell[BoxData[
         RowBox[{"LinkEndpointKinematics", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582826668*^9, 3.469165616318798*^9, {3.525146404960537*^9,
           3.525146410197776*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{"GeometricJacobian", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",",
           RowBox[{"{", "104", "}"}], ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582859626*^9, 3.469165616352508*^9, {3.487618257581111*^9,
           3.48761825911502*^9}, {3.499409314363538*^9, 3.499409332230188*^9}, {
           3.499410238750249*^9, 3.499410249789199*^9}, 3.499410607628263*^9, {
           3.525146413537066*^9, 3.525146416492277*^9}}],

        Cell[BoxData[
         RowBox[{"ForDynArt", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{{3.525146419655585*^9, 3.52514642229861*^9}}],

        Cell[BoxData[
         RowBox[{"ForDynComp", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582894937*^9, 3.4691656164047832`*^9, 3.499410251421309*^9,
           3.499410355748085*^9, {3.52514642559848*^9, 3.525146428415546*^9}},
         AspectRatioFixed->True],

        Cell[BoxData[
         RowBox[{"LinkInformation", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\""}], "]"}]], "Input",
         CellChangeTimes->{
          3.469165582926524*^9, 3.469165616435227*^9, {3.525146431186875*^9,
           3.525146434575663*^9}}],

        Cell["\<\
        Note that the list below is {1,2,3,4,5,6,7,8,9,10,11,12}, but each number \
        replaced by the successor link\
        \>", "Text"],

        Cell[BoxData[""], "Input",
         CellChangeTimes->{3.46916558296457*^9, 3.469165616469927*^9,
          3.487620831126145*^9}],

        Cell[BoxData[
         RowBox[{"ParmEst", "[",
          RowBox[{"\"\<«robotName».dyn\>\"", ",", "\"\<«robotName»\>\"", ",",
           RowBox[{"{",
            RowBox[{"0", ",", "0", ",",
             RowBox[{"-", "gravity"}]}], "}"}]}], "]"}]], "Input",
         CellChangeTimes->{
          3.4691655829948797`*^9, 3.469165616154977*^9, {3.525146437669165*^9,
           3.525146439684971*^9}}],

        Cell[BoxData["\t"], "Input",
         CellChangeTimes->{3.499409450658988*^9}],

        Cell[CellGroupData[{

        Cell[BoxData["\[AliasDelimiter]"], "Input",
         CellChangeTimes->{3.525146451826103*^9}],

        Cell[BoxData["\[AliasDelimiter]"], "Output",
         CellChangeTimes->{3.525151787140445*^9, 3.525151820416446*^9,
          3.525151900535952*^9, 3.525151967371533*^9, 3.525152058497048*^9,
          3.525152134681759*^9, 3.525152240817702*^9, 3.525152720084728*^9,
          3.525152880902732*^9, 3.525152995312067*^9, 3.525155871299821*^9,
          3.525157395534119*^9}]
        }, Open  ]]
        },
        WindowToolbars->{},
        CellGrouping->Automatic,
        WindowSize->{671, 773},
        WindowMargins->{{63, Automatic}, {22, Automatic}},
        PrivateNotebookOptions->{"ColorPalette"->{RGBColor, 128}},
        ShowSelection->True,
        ShowCellLabel->True,
        ShowCellTags->False,
        RenderingOptions->{"ObjectDithering"->True,
        "RasterDithering"->False},
        CharacterEncoding->"MacintoshAutomaticEncoding",
        FrontEndVersion->"7.0 for Linux x86 (32-bit) (February 25, 2009)",
        StyleDefinitions->"Default.nb"
        ]
        (* End of Notebook Content *)

        (* Internal cache information *)
        (*CellTagsOutline
        CellTagsIndex->{}
        *)
        (*CellTagsIndex
        CellTagsIndex->{}
        *)
        (*NotebookFileOutline
        Notebook[{
        Cell[545, 20, 395, 8, 32, "Input"],
        Cell[943, 30, 230, 4, 32, "Input"],
        Cell[1176, 36, 259, 6, 32, "Input"],
        Cell[1438, 44, 82, 1, 31, "Text"],
        Cell[1523, 47, 239, 5, 32, "Input"],
        Cell[1765, 54, 571, 12, 32, "Input"],
        Cell[2339, 68, 328, 8, 32, "Input"],
        Cell[2670, 78, 268, 6, 32, "Input"],
        Cell[2941, 86, 441, 8, 32, "Input"],
        Cell[3385, 96, 274, 6, 32, "Input"],
        Cell[3662, 104, 397, 9, 32, "Input"],
        Cell[4062, 115, 236, 5, 32, "Input"],
        Cell[4301, 122, 130, 3, 51, "Text"],
        Cell[4434, 127, 114, 2, 32, "Input"],
        Cell[4551, 131, 326, 8, 32, "Input"],
        Cell[4880, 141, 70, 1, 32, "Input"],
        Cell[CellGroupData[{
        Cell[4975, 146, 85, 1, 32, "Input"],
        Cell[5063, 149, 340, 5, 31, "Output"]
        }, Open  ]]
        }
        ]
        *)

        (* End of internal cache information *)'''

    def makefileUnix(Robot robot) '''
        «val nameUpperCase = robot.name.toUpperCase»
        «val nameLowerCase = robot.name.toLowerCase»
        INCLUDES = -I../src -I../include -I../math \
        -I$(MY_INCLUDES) -I/usr/X11/include \
        -I/usr/local/glut/include

        CFLAGS     = $(OPTIMIZE_CC_FLAGS) $(INCLUDES) -D$(MACHTYPE)
        SRCDIR     = ../src
        LDFLAGS    = $(LAB_LIBDIR)
        LIBDIR     = $(MYLIBDIR)/$(MACHTYPE)
        HEADERDIR  = $(MYINCLUDEPATH)
        LIBRARIES  =
        BINDIR     = .

        SRCS_COMMON  = \
        SL_user_commands.c \
        SL_user_common.c \
        SL_kinematics.c \
        SL_dynamics.c \
        SL_invDynNE.c \
        SL_invDynArt.c \
        SL_forDynComp.c \
        SL_forDynArt.c

        OBJS_COMMON  = \
        SL_user_commands.o \
        SL_user_common.o \
        SL_kinematics.o \
        SL_dynamics.o \
        SL_invDynNE.o \
        SL_invDynArt.o \
        SL_forDynComp.o \
        SL_forDynArt.o

        SRCS_X«nameUpperCase» = \
        SL_main.c \
        SL_user_common.c

        OBJS_X«nameUpperCase» = \
        SL_main.o \
        SL_user_common.o

        LIBS_X«nameUpperCase» = -lSLcommon -lutility -lX11 -lm

        SOURCES  = $(SRCS_COMMON) SL_parm_estimate.c SL_user_simulation.c SL_user_openGL.c SL_main.c SL_user_task.c SL_user_sensor_proc_unix.c SL_user_motor.c SL_user_vision.c
        OBJECTS  = $(OBJS_COMMON) SL_parm_estimate.o SL_user_simulation.o SL_user_openGL.o SL_main.o SL_user_task.o SL_user_sensor_proc_unix.o SL_user_motor.o SL_user_vision.o

        HEADERS =

        LIB_MOTOR     = -lSLmotor -lSLcommon -lutility $(COMM_LIBRARIES) -lm
        LIB_VISION    = -lSLvision -lSLcommon -llwpr -lutility $(COMM_LIBRARIES) -lm

        KeepUpToDateCopy( SL_kinematics.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_dynamics.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_forDynArt.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_forDynComp.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_invDynNE.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_invDynArt.c, $(LAB_ROOT)/SL/src, $(SRCDIR))
        KeepUpToDateCopy( SL_parm_estimate.c, $(LAB_ROOT)/SL/src, $(SRCDIR))

        ProgramListTarget( x«nameLowerCase», $(OBJS_X«nameUpperCase»), $(LIBS_X«nameUpperCase») )
        ProgramListTarget( xmotor, $(OBJS_COMMON) SL_user_motor.o SL_user_sensor_proc_unix.o ,$(LIB_MOTOR) )
        ProgramListTarget( xvision, $(OBJS_COMMON) SL_user_vision.o, $(LIB_VISION) )

        LibraryListAddTarget( «nameLowerCase», $(OBJS_COMMON), )
        LibraryListAddTarget( «nameLowerCase»_openGL, SL_user_openGL.o ,  )
        LibraryListAddTarget( «nameLowerCase»_task, SL_user_task.o ,  )
        LibraryListAddTarget( «nameLowerCase»_simulation, SL_user_simulation.o ,  )

        NormalObjRule( $(OBJECTS) )'''


    def SL_user_dot_h(Robot robot)'''
        #ifndef _SL_USER_H_
        #define _SL_USER_H_

        /*! the robot name */
        #define ROBOT_NAME "«robot.name»"
        /*! the force bias file */
        #define FORCE_BIAS_FILE "ForceBiases.cf"

        /*! links of the robot */
        enum RobotLinks {
            PLACEHOLDER = 0,
            «FOR Link link : robot.links»
            «link.name.toUpperCase()»,
            «ENDFOR»
            N_ROBOT_LINKS
        };

        /*! endeffector information */
        enum RobotEndeffectors {
          ENDEFF=1,
          N_ROBOT_ENDEFFECTORS
        };

        /*! vision variables */
        enum VisionCameras {
          N_VISION_CAMERAS
        };

        enum ColorBlobs {
          BLOB1=1,
          N_COLOR_BLOBS
        };

        /*! define the DOFs of this robot */
        enum RobotDOFs {
            BASE=0,
            «FOR Joint joint : robot.joints»
            «joint.name»,
            «ENDFOR»
            N_ROBOT_DOFS
        };

        /*! define miscellenous sensors of this robot */
        enum RobotMiscSensors {
          B_Q_0=1,
          B_Q_1,
          B_Q_2,
          B_Q_3,

          B_Qd_0,
          B_Qd_1,
          B_Qd_2,
          B_Qd_3,

          B_Qdd_0,
          B_Qdd_1,
          B_Qdd_2,
          B_Qdd_3,

          B_Ad_A,
          B_Ad_B,
          B_Ad_G,

          B_Add_A,
          B_Add_B,
          B_Add_G,

          B_X,
          B_Y,
          B_Z,

          B_Xd,
          B_Yd,
          B_Zd,

          B_Xdd,
          B_Ydd,
          B_Zdd,

          TIME_MOTOR,

          N_ROBOT_MISC_SENSORS
        };

        /*! number of degrees-of-freedom of robot */
        #define N_DOFS (N_ROBOT_DOFS-1)

        /*! N_DOFS + fake DOFS, needed for parameter estimation;
           fake DOFS come from creating endeffector information */
        #define N_DOFS_EST (N_DOFS+31)

        /*! N_DOFS to be excluded from parameter estimation (e.g., eye joints);
           these DOFS must be the last DOFS in the arrays */
        #define N_DOFS_EST_SKIP 0

        /*! number of links of the robot */
        #define N_LINKS    (N_ROBOT_LINKS-1)

        /*! number of miscellaneous sensors */
        #define N_MISC_SENSORS   (N_ROBOT_MISC_SENSORS-1)

        /*! number of endeffectors */
        #define N_ENDEFFS  (N_ROBOT_ENDEFFECTORS-1)

        /*! number of cameras used */
        #define N_CAMERAS (N_VISION_CAMERAS-1)

        /*! number of blobs that can be tracked by vision system */
        #define MAX_BLOBS (N_COLOR_BLOBS-1)

        /*! vision default post processing */
        #define VISION_DEFAULT_PP "vision_default.pp"

        /*! the servo rate used by the I/O with robot: this limits the
           servo rates of all other servos */
        #define  SERVO_BASE_RATE 1000

        /*! divisor to obtain task servo rate (task servo can run slower than
           base rate, but only in integer fractions */
        #define  TASK_SERVO_RATIO   R1TO4
        //! #define  TASK_SERVO_RATIO   R1TO1

        /* settings for D/A debugging information -- see SL_oscilloscope.c */
        #define   D2A_CM      1
        #define   D2A_CT      2
        #define   D2A_CV      3
        #define   D2A_CR      4

        #ifdef __cplusplus
        extern "C" {
        #endif

        /* function prototypes */
        int readTestGoal(void);
        void compute_cog_jacobian(void);
        void compute_cog_velocity(void);
        void worldToBase(double *xw, SL_Cstate *cbase, SL_quat *obase, double *xl);
        void baseToWorld(double *xl, SL_Cstate *cbase, SL_quat *obase, double *xw);
        void checkJointOffsets(int verbose);
        void compute_cog_kinematics(int **status, int flag, int use_z, int use_des,
                       Matrix Jccogp, Matrix NJccog);
        void compute_constraint_cog_kinematics(Matrix NJc, int use_z, int **status, int use_des,
                          Matrix Jccog,
                          Matrix Jccogp, Matrix NcJccog,
                          Matrix NJccog);
        void compute_constraint_kinematics(int **status, int use_des,
                          Matrix Jc, Matrix Jcp, Matrix NJc);
        void worldToBaseVelocity(double *xw, double *vw,
                    SL_Cstate *cbase, SL_quat *obase, double *vl);
        void dumpTerrain(void);
        void donePlanning(void);
        void setLegForceControlGains(int leg, double **gains);
        void setLegForceControlSetPoints(int leg, double *f_d);
        int  readForceBiases(char *fname);
        void readInitPos(Vector cartPos);
        int  readFootIKCache(void);
        int  footIKCached(int leg, Vector x, Vector q_bf, Vector q_br,
                 double *error_bf, double *error_br, int *rc, double *knee);

        void base_error_control(int *leg_status, double *delta_pos, double *delta_orient,
                   int use_des, double *delta_th);
        int  convertTerrainIDtoString(int ID, char *string);
        void inverseDynamicsFloat(double dt, iMatrix status, int use_des, SL_DJstate *joint_ref_state,
                     double *xdd_ref, double *add_ref, Matrix fc);
        void computeAvoidanceTorques(int leg, int use_des, double *f_foot,
                    double *f_knee, double *torque);

        /* external variables */
        #define REG_RADIUS 10   //!< (was 20) radius of point to include in terrain regression
        #define REG_DOWN    3   //!< down sampling of points in terrain regression
        #define REG_CRADIUS 2   //!< regression radius for computing the contact normal

        extern double test_goal[];
        extern int    no_user_interaction_flag;
        extern int    real_time_flag;
        extern char   initial_task_name[];
        extern double test_mocap_marker[];
        extern SL_Jstate joint_mocap_state[];
        extern double foot_radius;
        extern double force_biases[N_ENDEFFS+1][N_CART+1];

        #ifdef __cplusplus
        }
        #endif
        #endif  /* _SL_USER_H_ */'''


    def SL_user_common_dot_c(Robot robot)'''
        char joint_names[][20]= {
            {"BASE"}
            «FOR Joint joint : robot.joints»
            , «joint.name»
            «ENDFOR»
        };

        char cart_names[][20]= {
            {"dummy"},
            {"ENDEFF"},
        };'''
}