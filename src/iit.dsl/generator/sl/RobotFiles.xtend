package iit.dsl.generator.sl

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.ChildSpec
import iit.dsl.kinDsl.Link
import iit.dsl.kinDsl.PILiteral
import iit.dsl.kinDsl.FloatLiteral
import iit.dsl.kinDsl.PlainExpr
import iit.dsl.kinDsl.MultExpr
import iit.dsl.kinDsl.ConstantLiteral
import iit.dsl.kinDsl.DivExpr


class RobotFiles {

    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    extension iit.dsl.generator.sl.Common slCommon = new iit.dsl.generator.sl.Common()

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
            {translation,{«jointTranslParams(j)»}},
            {rotationMatrix,{«jointRotParams(j)»}},
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

    def private jointTranslParams(Joint j)
    '''«val tr=j.refFrame.translation»«string(tr.x)», «string(tr.y)», «string(tr.z)»'''
    def private jointRotParams(Joint j)
    '''«val rot=j.refFrame.rotation»«string(rot.x)», «string(rot.y)», «string(rot.z)»'''

    def private dispatch string(FloatLiteral id)'''«id.value.str»'''
    def private dispatch string(PlainExpr expr) '''«string(expr.identifier)»'''
    def private dispatch string(MultExpr expr)  '''«expr.mult.str» «string(expr.identifier)»'''
    def private dispatch string(DivExpr expr)   '''«string(expr.identifier)»/«expr.div»'''
    def private dispatch string(ConstantLiteral id)  '''«id.str»'''
    def private dispatch string(PILiteral pi)   '''«IF pi.minus»-«ENDIF»Pi'''

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


    private static String DUMMY_EE_LINK_ID = "DUMMY_EE"

    def SL_user_dot_h(Robot robot)'''
        #ifndef _SL_USER_«robot.name.toUpperCase»_H_
        #define _SL_USER_«robot.name.toUpperCase»_H_

        #include "SL.h"

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
            «DUMMY_EE_LINK_ID»,
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

        «val sens = Utilities::defaultMiscSensors»
        /*! define miscellenous sensors of this robot */
        enum RobotMiscSensors {
            «sens.get(0)»=1,
            «FOR s : sens.drop(1)»
                «s»,
            «ENDFOR»
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
        #endif  /* _SL_USER_«robot.name.toUpperCase»_H_ */'''


    def SL_user_common_dot_c(Robot robot)'''
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
            «FOR s : Utilities::defaultMiscSensors SEPARATOR ','»
                {"«s»"}
            «ENDFOR»
        };

        int link2endeffmap[] = {0,«DUMMY_EE_LINK_ID»};
        '''

    def Makefile(Robot robot) '''
        «val nameLower = robot.name.toLowerCase»
        «val String TAB = "\t"»
        DIR_SRCS = src
        DIR_HEADERS = include
        DIR_OBJS = $(MACHTYPE)
        DIR_LIBS = $(DIR_OBJS)
        DIR_BINS = $(DIR_OBJS)
        DIR_DEPS = $(DIR_OBJS)

        DIR_INSTALL_LIBS = $(SL_ROOT)/lib/$(MACHTYPE)
        DIR_INSTALL_BINS = $(SL_ROOT)/«robotUserFolderName(robot)»/$(MACHTYPE)

        DIR_SLCORE = $(SL_ROOT)/SL
        DIR_SLCORE_SRCS = $(SL_ROOT)/SL/src

        # Where to find SL libraries and data acquisition libraries:
        DIR_SL_LIBS  = $(SL_ROOT)/lib/$(MACHTYPE)
        DIR_DAQ_LIBS = $(SL_ROOT)/lib/$(MACHTYPE)

        include $(DIR_SLCORE)/Makefile.common


        INCLUDE_PATHS = include/ \
        math/ \
        $(SL_ROOT)/include/ \
        $(SL_ROOT)/utilities/include/ \
        /usr/local/glut/include/ \
        /usr/X11/include/ \
        /usr/local/RoboLLI/include \
        $(IIT_IO_ROOT) $(FANCY_IO_ROOT)

        CPPFLAGS += $(patsubst %,-I %,$(INCLUDE_PATHS))
        CXXFLAGS += -g -Wall -O3 -march=native -mtune=native -D $(MACHTYPE) -D EIGEN_NO_DEBUG -D UNIX # -D CLMC
        LDFLAGS  += -L$(SL_ROOT)/lib/$(MACHTYPE) -L/opt/local/lib -L/sw/lib -L/usr/X11/lib -L/usr/local/RoboLLI/lib

        LIBRARIES = «nameLower» «nameLower»_openGL «nameLower»_task «nameLower»_simulation
        BINARIES  = x«nameLower» xmotor

        # ------- #
        # SOURCES #
        # ------- #
        # Source files common for any architecture:

        SL_SRCS = SL_kinematics.c \
        SL_dynamics.c \
        SL_invDynNE.c \
        SL_invDynArt.c \
        SL_forDynComp.c \
        SL_forDynArt.c

        COMMON_SRCS = SL_user_commands.c SL_user_common.c\
        $(SL_SRCS)

        SRCS_«nameLower» = $(COMMON_SRCS)
        SRCS_«nameLower»_openGL = SL_user_openGL.c
        SRCS_«nameLower»_task = SL_user_task.c
        SRCS_«nameLower»_simulation = SL_user_simulation.c

        SRCS_x«nameLower»  = SL_main.c SL_user_common.c
        SRCS_xr«nameLower» = SL_user_common.c
        SRCS_xmotor  = $(COMMON_SRCS) SL_user_motor.c
        SRCS_xrmotor = $(COMMON_SRCS) SL_user_motor.c

        #
        # Architecture dependent source files:

        ifeq ($(MACHTYPE),i486xeno)
            # For XENO machine:
            COMMON_SRCS   +=
            SRCS_«nameLower»_task += SL_user_task_xeno.c
            SRCS_x«nameLower»   +=
            SRCS_xr«nameLower»  += SL_rmain.c
            SRCS_xmotor   +=
            SRCS_xrmotor  += SL_user_sensor_proc_xeno.c
        else
            # Plain UNIX architecture:
            SRCS_«nameLower»_task +=
            SRCS_xr«nameLower»    +=
            SRCS_xmotor   += SL_user_sensor_proc_unix.c
            SRCS_xrmotor  +=
        endif


        # Prepend 'SRCS_' to the name of all the libraries and binaries
        #SRCS_GROUPS  = $(patsubst %,SRCS_%,$(LIBRARIES))
        SRCS_GROUPS += $(patsubst %,SRCS_%,$(BINARIES) $(LIBRARIES))

        # This uses each of the 'SRCS_<module>' names as a variable
        #  and gets the corresponding value (which is a list of source files)
        SOURCES = $(foreach GROUP,$(SRCS_GROUPS),$(value $(GROUP)))


        # Function that takes a target name (either a lib or a binary) and returns
        # the related object files.
        # To be called with '$(call targetToObjects,<name>)'
        # First, removes the file extension from source files, then prepends the
        #  correct directory and adds the extension '.o'.
        targetToObjects = $(patsubst %,$(DIR_OBJS)/%.o, $(basename $(SRCS_$(1))) )

        # Use 'sort' because it removes duplicates
        #OBJECTS  = $(sort $(patsubst %.c,$(DIR_OBJS)/%.o,$(filter %.c,$(SOURCES))))
        #OBJECTS += $(sort $(patsubst %.cpp,$(DIR_OBJS)/%.o,$(filter %.cpp,$(SOURCES))))
        ALL_OBJECTS = $(sort $(foreach TARGET,$(LIBRARIES) $(BINARIES),$(call targetToObjects,$(TARGET))))

        SL_OBJECTS = $(patsubst %.c,$(DIR_OBJS)/%.o,$(SL_SRCS))

        OBJECTS = $(filter-out $(SL_OBJECTS),$(ALL_OBJECTS))

        DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJECTS) $(SL_OBJECTS))


        # ------------------ #
        # Required libraries #
        # ------------------ #
        SYS_LIBS = m readline curses

        LIBS_x«nameLower»_SL    = SLcommon utility
        LIBS_xr«nameLower»_SL   = SLcommon utility
        LIBS_xmotor_SL  = SLmotor SLcommon utility
        LIBS_xrmotor_SL = SLmotor SLcommon utility

        # Real-robot executables might need the data I/O libraries:
        #  (but actually you control the real-robot only with a xeno machine,
        #   so these vars default to empty).
        LIBS_xr«nameLower»_DAQ   =
        LIBS_xrmotor_DAQ =

        LIBS_x«nameLower»  = $(LIBS_x«nameLower»_SL)  $(LIBS_x«nameLower»_DAQ) X11
        LIBS_xr«nameLower» = $(LIBS_x«nameLower»_SL)  $(LIBS_xr«nameLower»_DAQ) X11
        LIBS_xmotor  = $(LIBS_xmotor_SL)  $(LIBS_xmotor_DAQ)  nsl
        LIBS_xrmotor = $(LIBS_xrmotor_SL) $(LIBS_xrmotor_DAQ) nsl

        ifeq ($(MACHTYPE),i486xeno) # For XENO machine:

        else  # Plain UNIX architecture:

            LIBS_xrmotor_SL +=
            SYS_LIBS += pthread rt

        endif

        # Function that takes a target name (either a lib or a binary) and returns
        # the linker options with the required libraries.
        targetToLibs = $(patsubst %,-l%,$(LIBS_$(1)) $(SYS_LIBS))

        # These return the files names of non-system libraries, i.e. libraries that might
        #  be rebuilt by this Makefile itself (i.e. SL and DAQ libraries)
        targetToDAQLibNames = $(patsubst %,$(DIR_DAQ_LIBS)/lib%.a,$(LIBS_$(1)_DAQ))
        targetToSLLibNames  = $(patsubst %,$(DIR_SL_LIBS)/lib%.a,$(LIBS_$(1)_SL))
        targetToRebuildableLibs = $(call targetToSLLibNames,$(1)) $(call targetToDAQLibNames,$(1))


        # ------- #
        # TARGETS #
        # ------- #
        LIBRARY_FILE_PATTERN = $(DIR_LIBS)/lib%.a
        STATIC_LIBS = $(patsubst %,$(LIBRARY_FILE_PATTERN),$(LIBRARIES))
        LIBS_TO_INSTALL = $(patsubst %,$(DIR_INSTALL_LIBS)/%,$(notdir $(STATIC_LIBS)))

        BINARY_FILE_PATTERN = $(DIR_BINS)/%
        EXECUTABLES = $(patsubst %,$(BINARY_FILE_PATTERN),$(BINARIES))
        BINS_TO_INSTALL = $(patsubst %,$(DIR_INSTALL_BINS)/%,$(notdir $(EXECUTABLES)))


        all : $(EXECUTABLES) $(STATIC_LIBS)

        install : all $(LIBS_TO_INSTALL) $(BINS_TO_INSTALL)

        # These 'order-only' prerequisites *ensure* (????) that all the SL stuff gets
        # always eventually updated before building HyQ stuff.
        $(STATIC_LIBS) : | make_folders SL_install_headers
        $(OBJECTS)     : | make_folders SL_install_headers
        $(EXECUTABLES) : | make_folders SL_core

        # ------------ #
        # OBJECT FILES #
        # ------------ #
        objects : $(SL_OBJECTS) $(OBJECTS)

        # Helping "macros" for .o and .d targets
        # With the -MMD flag $(CXX) produces dependencies information as a side
        # effect of compilation (check the compiler man for details).
        # Such ouptut should also include all the required SL-core headers
        # (such that if any of those headers changes, some object here will be rebuilt)
        COMPILE = $(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@

        .SECONDEXPANSION:
        $(OBJECTS) : $(DIR_OBJS)/%.o : $$(wildcard $(DIR_SRCS)/%.c*)
        «TAB»$(COMPILE)

        $(SL_OBJECTS) : $(DIR_OBJS)/%.o : $(DIR_SLCORE_SRCS)/%.c
        «TAB»$(COMPILE)


        # ---------------------- #
        # LIBRARIES AND BINARIES #
        # ---------------------- #
        .SECONDEXPANSION:
        $(STATIC_LIBS) : $(LIBRARY_FILE_PATTERN) : $$(call targetToObjects,%)
        «TAB»@echo " * Building library $@ ..."
        «TAB»@$(REMOVE) $@
        «TAB»@ar cr $@ $^

        .SECONDEXPANSION:
        $(EXECUTABLES) : $(BINARY_FILE_PATTERN) : $$(call targetToRebuildableLibs,%) $$(call targetToObjects,%)
        «TAB»@echo " + Building binary $@ ..."
        «TAB»$(CXX) $(CXXFLAGS) $(LDFLAGS) $(filter %.o,$^) $(call targetToLibs,$*)  -o $@


        # ---------- #
        # INSTALLING #
        # ---------- #
        $(LIBS_TO_INSTALL) : $(DIR_INSTALL_LIBS)/% : $(DIR_LIBS)/%
        «TAB»@echo " * Installing library $@ ..."
        «TAB»@$(REMOVE) $@
        «TAB»@$(COPY) $< $@
        «TAB»@chmod ugo-wx $@

        $(BINS_TO_INSTALL) : $(DIR_INSTALL_BINS)/% : $(DIR_BINS)/%
        «TAB»@echo " * Installing binary $@ ..."
        «TAB»@$(REMOVE) $@
        «TAB»@$(COPY) $< $@
        «TAB»@chmod ugo-w $@


        -include $(DEPS_FILES)


        # -p to avoid warning if directory exists (can be improved with checks)
        make_folders :
        «TAB»@mkdir -p $(DIR_OBJS) $(DIR_LIBS) $(DIR_BINS) $(DIR_DEPS)


        SL_install_headers :
        «TAB»$(MAKE) -C $(DIR_SLCORE) install_headers

        SL_core :
        «TAB»$(MAKE) -C $(DIR_SLCORE) all install

        # TODO: add invocation of make in the lwpr and utility sub projects
        ######################################


        makedebug :
        «TAB»@echo $(call targetToRebuildableLibs,xrmotor)

        clean :
        «TAB»$(REMOVE) $(SL_OBJECTS) $(OBJECTS) $(DEPS_FILES) $(STATIC_LIBS) $(EXECUTABLES)

        remove_sl_srcs :
        «TAB»$(REMOVE) $(patsubst %,src/%,$(SL_SRCS))

        .PHONY = all install clean makedebug SL_core SL_install_headers make_folders remove_sl_srcs
    '''
}