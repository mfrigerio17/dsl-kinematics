package iit.dsl.generator.sl

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.cpp.Names

class RobotUserFiles {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    extension iit.dsl.generator.sl.Common slCommon = new iit.dsl.generator.sl.Common()


	/**
    Configuration file with Inertia parameters
    It is assumed that the provided moments of inertia strictly follow their definition,
    so that the off-diagonal elements of the inertia tensor have a minus sign. It is also
    assumed that the configuration file for SL expects the inertia tensor elements, so we
    put that minus in front of the values
    */
    def linkParameters(Robot robot)'''
        «val bparams = Utilities::tuneForSL(robot.base.inertiaParams)»
        BASE «'\t'»«bparams.mass.str»   «bparams.com.x.str» «bparams.com.y.str» «bparams.com.z.str»   «bparams.ix.str» «bparams.ixy.str» «bparams.ixz.str» «bparams.iy.str» «bparams.iyz.str» «bparams.iz.str»   0.1 0 0 0
        «FOR link : robot.links»
            «val params = Utilities::tuneForSL(link.inertiaParams)»
            «link.connectingJoint.name» «'\t'»«params.mass.str»   «params.com.x.str» «params.com.y.str» «params.com.z.str»   «params.ix.str» «params.ixy.str» «params.ixz.str» «params.iy.str» «params.iyz.str» «params.iz.str»   0.1 0 0 0
        «ENDFOR»
        '''

    def confFile_gains(Robot robot) '''
        «FOR Joint joint : robot.joints»
            «joint.name»   100.0    1.0    0.0   1000
        «ENDFOR»
    '''

    def confFile_sensorCalibration(Robot robot) '''
        «FOR Joint joint : robot.joints»
            «joint.name»   0 0 0 0 0 1 1
        «ENDFOR»
    '''

    def confFile_whichDOFs(Robot robot) '''
        /* this file contains a list of DOFs that are computed by each processor
           Note: the file is parsed according to keywords, after the keyword,
           the values need to come in the correct sequence */

           /* format: keyword, <list of joint names for a processor> */

        task_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        task_sim_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        motor_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        motor_sim_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        vision_servo
        vision_sim_servo
        invdyn_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»

        invdyn_sim_servo
            «FOR Joint joint : robot.joints»
                «joint.name»
            «ENDFOR»
    '''

    def confFile_sensorOffset(Robot robot) '''
        /* this file contains the specification for position offsets and min/max
           position values (min and max AFTER offset was subtracted. Additionally,
           the file allows to set a default posture for the robot.
           Note: the file is parsed according to keywords, after the keyword,
                 the values need to come in the correct sequence */
        /* format: keyword, min , max, default, rest, weight, offset  */
        /* Please edit this file according to your needs */

       «FOR Joint joint : robot.joints»
            «joint.name» 0.0  0.0  0.0  0.0  0.1  0.0
       «ENDFOR»
    '''

    def confFile_sensorFilters(Robot robot) '''
        «FOR Joint joint : robot.joints»
            «joint.name»   100  100  100  100
        «ENDFOR»

        «FOR s : Utilities::defaultMiscSensors»
            «s»«'\t\t'»100  100  100  100
        «ENDFOR»
    '''

    def imakefileUnix(Robot robot) '''
        «val nameUpperCase = robot.name.toUpperCase»
        «val nameLowerCase = robot.name.toLowerCase»
        INCLUDES = -I../src \
        -I../include \
        -I$(MY_INCLUDES) \
        -I$(LAB_INCLUDES) \
        -I$(LAB_ROOT)/«nameLowerCase»/include \
        -I$(LAB_ROOT)/«nameLowerCase»/math \
        -I/sw/include \
        -I/usr/X11/include \
        -I/usr/local/glut/include \
        -I$(LAB_ROOT)/common \
        -I/usr/local/include/eigen3/ \

        CFLAGS    = $(OPTIMIZE_CC_FLAGS) $(INCLUDES) -DEIGEN_NO_DEBUG -D$(MACHTYPE)
        SRCDIR    = ../src
        LIBDIR    = $(MY_LIBDIR)/$(MACHTYPE)
        HEADERDIR = $(MY_INCLUDES)
        LDFLAGS   = -L$(MY_LIBDIR)/$(MACHTYPE) $(LAB_LIBDIR)
        LIBRARIES =
        BINDIR    = .

        LIB_TASK   = -l«nameLowerCase»_task       -l«nameLowerCase» -lSLtask -lSLcommon -lutility  -lm
        LIB_OPENGL = -l«nameLowerCase»_openGL     -l«nameLowerCase» -lSLopenGL -lSLcommon -lutility -lm $(OPENGL_LIBRARIES) -lXinerama -lX11
        LIB_SIM    = -l«nameLowerCase»_simulation -l«nameLowerCase» -lSLsimulation -lSLcommon -lutility $(COMM_LIBRARIES) -lm -llwpr

        SRCS_X«nameUpperCase»  = \
            initUserTasks.c \
            sample_task.c
        OBJS_X«nameUpperCase»  = \
            initUserTasks.o \
            sample_task.o

        SRCS_XOPENGL  = \
            initUserGraphics.c \
             userGraphics.c

        OBJS_XOPENGL  = \
            initUserGraphics.o \
            userGraphics.o \

        SRCS_XSIM  = \
               initUserSimulation.c

        OBJS_XSIM  = \
               initUserSimulation.o

        SOURCES = $(SRCS_X«nameUpperCase») $(SRCS_XOPENGL) $(SRCS_XSIM)
        OBJECTS = $(OBJS_X«nameUpperCase») $(OBJS_XOPENGL) $(OBJS_XSIM) $(MYOBJS)

        CPP_OBJECTS=«robot.benchmarkIDFileName».o «robot.main_inertiaM_filename».o

        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/x«nameLowerCase»,$(BINDIR))
        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/xmotor,$(BINDIR))
        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/xvision,$(BINDIR))

        CPPProgramListTarget( xtask, $(OBJS_X«nameUpperCase») $(CPP_OBJS_X«nameUpperCase»),$(LIB_TASK) )
        CPPProgramListTarget( xopengl, $(OBJS_XOPENGL), $(LIB_OPENGL) )
        CPPProgramListTarget( xsimulation, $(OBJS_XSIM), $(LIB_SIM) )

        CPPProgramListTarget(benchID, «robot.benchmarkIDFileName».o $(OBJS_XSIM), $(LIB_SIM))
        CPPProgramListTarget(jsim, «robot.main_inertiaM_filename».o $(OBJS_XSIM), $(LIB_SIM))

        NormalObjRule( $(OBJECTS) )
        NormalCPPObjRule( $(CPP_OBJECTS) )'''


    def main_benchmarkID(Robot robot) '''
        #include <iostream>
        #include <fstream>
        #include <ctime>
        #include <cstdlib>

        #include "SL.h"
        #include "SL_user.h"
        #include "SL_kinematics.h"
        #include "SL_dynamics.h"

        using namespace std;

        static void fillState(SL_DJstate* desiredState);
        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject);

        /* This main is supposed to be used to test the inverse dynamics routines */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the number of tests to perform" << endl;
                return -1;
            }
            int numOfTests = std::atoi(argv[1]);
            double sl[numOfTests];
            int iterations[numOfTests];

            double t0, duration, sl_total;
            sl_total = 0;

            int t=0,i=0;

            SL_Jstate currentState[N_ROBOT_DOFS];
            SL_DJstate desiredState[N_ROBOT_DOFS];
            SL_endeff endeffector[N_ROBOT_ENDEFFECTORS];
            SL_Cstate basePosition;
            SL_quat baseOrient;

            init_kinematics();
            if( ! init_dynamics() ) {
                cerr << "Error in init_dynamics()" << endl;
                exit(-1);
            }

            for(i=0; i<N_ROBOT_DOFS; i++) {
                currentState[i].th   = 0;
                currentState[i].thd  = 0;
                currentState[i].thdd = 0;

                desiredState[i].th   = 0;
                desiredState[i].thd  = 0;
                desiredState[i].thdd = 0;

                desiredState[i].uff  = 0;
                desiredState[i].uex  = 0;
            }

            // Zeroes out every end effector:
            for(int e=1; e<=N_ENDEFFS; e++) {
                endeffector[e].m = 0;
                for(i=0; i<=N_CART; i++) {
                    endeffector[e].x[i]   = 0;
                    endeffector[e].a[i]  = 0;
                }
            }


            for(i=0; i<=N_CART; i++) {
                basePosition.x[i]   = 0;
                basePosition.xd[i]  = 0;
                basePosition.xdd[i] = 0;
            }
            baseOrient.q[_Q0_] = 1;
            baseOrient.q[_Q1_] = 0;
            baseOrient.q[_Q2_] = 0;
            baseOrient.q[_Q3_] = 0;

            // This restores the default end effector parameters (ie non zero values as opposed
            //  to what we did above): for some reason this makes inverse dynamics much slower (???)
            //setDefaultEndeffector();

            std::srand(std::time(NULL)); // initialize random number generator

            /*
            // Prints numerical results, for comparison
            fillState(desiredState);
            SL_InvDynNE(NULL, desiredState, endeffector, &basePosition, &baseOrient);
            cout << "SL:" << endl
            «FOR Joint j : robot.joints»
                << desiredState[«j.name»].uff << endl
            «ENDFOR»
                ;
           return 1;
           //*/

            int numOfIterations = 1;
            for(t=0; t<numOfTests; t++) {
                sl_total = 0;
                numOfIterations = numOfIterations * 10;
                iterations[t] = numOfIterations;

                for(i=0; i<numOfIterations; i++) {
                    fillState(desiredState);

                    t0 = std::clock();
                    SL_InvDynNE(NULL, desiredState, endeffector, &basePosition, &baseOrient);
                    duration = std::clock() - t0;
                    sl_total += duration;
                }
                sl[t] = sl_total/CLOCKS_PER_SEC;
            }


           for(int t=0; t<numOfTests; t++) {
                cout << sl[t] << endl;
            }
            matlabLog(numOfTests, iterations, sl, "inv_dyn");

            return TRUE;
        }


        void fillState(SL_DJstate* desiredState) {
            static const double max = 12.3;
            for(int i=0; i<N_ROBOT_DOFS; i++) {
                desiredState[i].th   = ( ((double)std::rand()) / RAND_MAX) * max;
                desiredState[i].thd  = ( ((double)std::rand()) / RAND_MAX) * max;
                desiredState[i].thdd = ( ((double)std::rand()) / RAND_MAX) * max;
            }
        }

        static void matlabLog(int numOfTests, int* iterations, double* tests, const std::string& subject) {
            «val prefix = robot.name.toLowerCase + "_test"»
            std::string fileName = "«robot.name»_" + subject + "_speed_test_data.m";
            ofstream out(fileName.c_str());
            out << "«prefix».robot       = '«robot.name»';" << std::endl;
            out << "«prefix».description = 'test of the speed of the calculation of: " << subject << "';" << std::endl;
            out << "«prefix».software    = 'SL';" << std::endl;

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



        def main_inertiaM(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        «val robotdynNS = robotNS + "::" + Names$Namespaces::dynamics»
        «val jsimT_qualified = robotdynNS + "::" + Names$Types::jspaceMLocal»
        «val jsim = robotdynNS + "::" + Names$GlobalVars::jsInertia»
        «val jointDOFs = robot.jointDOFs»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include <iit/rbd/rbd.h>
        #include <iit/rbd/utils.h>
        #include <iit/robots/«Names$Files::folder(robot)»/«Names$Files$RBD::jsimHeader(robot)».h>

        #include "SL.h"
        #include "SL_user.h"
        #include "SL_kinematics.h"
        #include "SL_dynamics.h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static SL_Jstate currentState[N_ROBOT_DOFS];
        static SL_endeff endeffector[N_ROBOT_ENDEFFECTORS];
        static SL_Cstate basePosition;
        static SL_quat baseOrient;
        static SL_uext   ux[N_DOFS+1];
        static Matrix rbdM;
        static Vector rbdCG;

        static void fillState(«robotNS»::«Names$Types::jointState»& q, SL_Jstate* SLState);
        static void SL_init();

        /* This main is supposed to be used to test the joint space inertia matrix routines */
        int main(int argc, char**argv)
        {
            «robotNS»::«Names$Types::jointState» q;

            SL_init();
            std::srand(std::time(NULL)); // initialize random number generator
            fillState(q, currentState);
            SL_ForDynComp(currentState, &basePosition, &baseOrient, ux, endeff, rbdM, rbdCG);
            «jsim»(q);

            «jsimT_qualified» SLM;

            // Copies the matrix of SL into an Eigen matrix, to make it easier to print, compare, etc.
            «IF robot.base.floating»
                // Copy the joint-space part of the matrix.
                // Cannot use for loops because the joint ordering might be different in SL
                «FOR Joint jo : robot.joints»
                    «FOR Joint ji : robot.joints»
                        SLM(«robotNS»::«iit::dsl::generator::cpp::Common::jointIdentifier(jo)»+6,«robotNS»::«iit::dsl::generator::cpp::Common::jointIdentifier(ji)»+6) = rbdM[«jo.name»][«ji.name»];
                    «ENDFOR»
                «ENDFOR»
                // Copy the 6x6 block with the composite inertia of the whole robot:
                int r,c;
                for(r=0; r<6; r++) {
                    for(c=0; c<6; c++) {
                        SLM(r,c) = rbdM[r+1+«jointDOFs»][c+1+«jointDOFs»];
                    }
                }
                // re-arrange blocks to match the convention of the generated dynamics code
                «Names$Namespaces::rbd»::Matrix33d temp;
                temp = SLM.block<3,3>(0,0);
                SLM.block<3,3>(0,0) = SLM.block<3,3>(3,3);
                SLM.block<3,3>(3,3) = temp;

                SLM.block<3,3>(0,3) = SLM.block<3,3>(3,0);
                SLM.block<3,3>(3,0) = SLM.block<3,3>(0,3).transpose();

                // Copy the remaining blocks:
                for(r=0; r<6; r++) {
                    for(c=6; c<«jointDOFs»; c++) {
                        SLM(r,c) = rbdM[r+1+«jointDOFs»][c+1-6];
                    }
                }
                for(r=0; r<6; r++) {
                    «FOR Joint j : robot.joints»
                        SLM(r,«robotNS»::«iit::dsl::generator::cpp::Common::jointIdentifier(j)»+6) = rbdM[r+1+«jointDOFs»][«j.name»];
                    «ENDFOR»
                }
                // Deal with different convention about spatial vectors (linear/angular part)
                Eigen::Matrix<double,3,«jointDOFs»> tempF;
                tempF = SLM.block<3,«jointDOFs»>(0,6);
                SLM.block<3,«jointDOFs»>(0,6) = SLM.block<3,«jointDOFs»>(3,6);
                SLM.block<3,«jointDOFs»>(3,6) = tempF;
                // F and F^T blocks
                SLM.block<«jointDOFs»,6>(6,0) = SLM.block<6,«jointDOFs»>(0,6).transpose();
            «ELSE»
                «FOR Joint jo : robot.joints»
                    «FOR Joint ji : robot.joints»
                        SLM(«robotNS»::«iit::dsl::generator::cpp::Common::jointIdentifier(jo)»,«robotNS»::«iit::dsl::generator::cpp::Common::jointIdentifier(ji)») = rbdM[«jo.name»][«ji.name»];
                    «ENDFOR»
                «ENDFOR»
            «ENDIF»

            rbd::Utils::CwiseAlmostZeroOp<«jsimT_qualified»::Scalar> almostZero(1E-4);

            cout << "SL:" << endl << SLM.unaryExpr(almostZero) << endl;
            cout << "Me:" << endl << «jsim».unaryExpr(almostZero)  << endl;

            //«jsimT_qualified»::MatrixType diff = SLM - «robotdynNS»::«Names$GlobalVars::jsInertia»;
            //cout << "difference:" << endl << diff.unaryExpr(almostZero) << endl;

            «IF robot.base.floating»
                //cout << SLM.block<6,6>(0,0).unaryExpr(almostZero) << endl;
                //cout << «jsim».block<6,6>(0,0).unaryExpr(almostZero) << endl;
            «ENDIF»
            return TRUE;
        }


        void fillState(«robotNS»::«Names$Types::jointState»& q, SL_Jstate* SLState) {
            static const double max = 12.3;
            «FOR Joint j : robot.joints»
                q(«j.getID()-1») = ( ((double)std::rand()) / RAND_MAX) * max;
                SLState[«j.name»].th = q(«j.getID()-1»);
            «ENDFOR»
        }

        static void SL_init() {
            init_kinematics();
            init_dynamics();

            bzero((void *)&basePosition,sizeof(basePosition));
            bzero((void *)&baseOrient,sizeof(baseOrient));
            bzero((void *)ux,sizeof(SL_uext)*N_DOFS+1);
            setDefaultEndeffector(); // the the default end-effector parameters

            baseOrient.q[_Q0_] = 1;
            baseOrient.q[_Q1_] = 0;
            baseOrient.q[_Q2_] = 0;
            baseOrient.q[_Q3_] = 0;

            rbdM = my_matrix(1,N_DOFS+6,1,N_DOFS+6);
            rbdCG = my_vector(1,N_DOFS+6);
            mat_zero(rbdM);
            vec_zero(rbdCG);

            freeze_base = TRUE;//
        }
        '''


    def Makefile(Robot robot) '''
        «val nameLower = robot.name.toLowerCase»
        «val String TAB = "\t"»
        DIR_SRCS = src
        DIR_OBJS = $(MACHTYPE)
        DIR_LIBS = $(DIR_OBJS)
        DIR_BINS = $(DIR_OBJS)
        DIR_DEPS = $(DIR_OBJS)

        DIR_SLCORE = $(SL_ROOT)/SL
        DIR_SL_LIBS = $(SL_ROOT)/lib/$(MACHTYPE)

        include $(DIR_SLCORE)/Makefile.common

        INCLUDE_PATHS = src \
        $(SL_ROOT)/include \
        $(SL_ROOT)/SLRemote/include \
        $(SL_ROOT)/«nameLower»/include \
        $(SL_ROOT)/«nameLower»/math \
        /sw/include \
        /usr/X11/include \
        /usr/local/glut/include

        CPPFLAGS += $(patsubst %,-I %,$(INCLUDE_PATHS))
        CPPFLAGS += -I/usr/local/include/eigen3/
        CXXFLAGS += -g -Wall -O3 -march=native -mtune=native -D $(MACHTYPE) -D UNIX -D EIGEN_NO_DEBUG
        LDFLAGS  += -L$(SL_ROOT)/lib/$(MACHTYPE) -L/opt/local/lib -L/sw/lib -L/usr/X11/lib

        BINARIES = xtask xopengl xsimulation benchID jsimTest

        # ------- #
        # SOURCES #
        # ------- #
        # Source files common for any architecture:

        SRCS_xtask   = initUserTasks.c sample_task.c
        SRCS_xopengl = initUserGraphics.c
        SRCS_xsimulation = initUserSimulation.c

        SRCS_benchID  = «robot.benchmarkIDFileName».cpp  «Names$Files::transformsHeader(robot)».cpp «Names$Files$RBD::source(robot)».cpp initUserSimulation.c
        SRCS_jsimTest = «robot.main_inertiaM_filename».cpp «Names$Files::transformsHeader(robot)».cpp «Names$Files$RBD::jsimHeader(robot)».cpp

        #
        # Architecture dependent source files:
        ifeq ($(MACHTYPE),i486xeno)
            # For XENO machine:
        else
            # Plain UNIX architecture:
        endif

        targetToObjects = $(patsubst %,$(DIR_OBJS)/%.o, $(basename $(SRCS_$(1))) )

        OBJECTS    = $(sort $(foreach TARGET,$(BINARIES),$(call targetToObjects,$(TARGET)))) # sort removes duplicates
        DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJECTS))

        # ------------------ #
        # Required libraries #
        # ------------------ #

        LIBS_OPENGL = glut GL GLU Xmu
        LIBS_SYS = m readline curses

        LIBS_xtask_SL = SLtask SLcommon «nameLower»_task «nameLower» utility #SLRemote
        LIBS_xopengl_SL = SLopenGL SLcommon «nameLower»_openGL «nameLower» utility
        LIBS_xsimulation_SL = SLsimulation SLcommon lwpr «nameLower»_simulation «nameLower» utility

        LIBS_benchID  = $(LIBS_xtask_SL) $(LIBS_xsimulation_SL) rbd
        LIBS_jsimTest = $(LIBS_xtask_SL) $(LIBS_xsimulation_SL) rbd

        LIBS_xtask   = $(LIBS_xtask_SL)
        LIBS_xopengl = $(LIBS_xopengl_SL) $(LIBS_OPENGL) Xinerama X11
        LIBS_xsimulation  = $(LIBS_xsimulation_SL) nsl


        ifeq ($(MACHTYPE),i486xeno) # For XENO machine:

            LIBS_xtask_SL +=
            LIBS_xopengl_SL +=
            LIBS_xsimulation_SL +=

            LIBS_SYS += iitio canfestival canfestival_unix native rtdk analogy rtdm

        else  # Plain UNIX architecture:
            LIBS_SYS += pthread rt
        endif

        # Function that takes a target name (either a lib or a binary) and returns
        # the linker options with the required libraries.
        targetToLibs = $(patsubst %,-l%,$(LIBS_$(1)) $(LIBS_SYS))

        targetToSLLibNames = $(patsubst %,$(DIR_SL_LIBS)/lib%.a,$(LIBS_$(1)_SL))

        SL_REQUIRED_LIBS = $(sort $(foreach BIN,$(BINARIES),$(call targetToSLLibNames,$(BIN))))


        # ------- #
        # Targets #
        # ------- #
        BINARY_FILE_PATTERN = $(DIR_BINS)/%
        EXECUTABLES = $(patsubst %,$(BINARY_FILE_PATTERN),$(BINARIES))

        all : $(EXECUTABLES)

        $(EXECUTABLES) : | make_folders make_«nameLower»
        $(OBJECTS) : | make_folders

        .SECONDEXPANSION:
        $(EXECUTABLES) : $(BINARY_FILE_PATTERN) : $$(call targetToSLLibNames,%) $$(call targetToObjects,%)
        «TAB»$(CXX) $(CXXFLAGS) $(LDFLAGS) $(filter %.o,$^) $(call targetToLibs,$*)  -o $@

        .SECONDEXPANSION:
        $(OBJECTS) : $(DIR_OBJS)/%.o : $$(wildcard $(DIR_SRCS)/%.c*)
        «TAB»$(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@


        ##$(SL_REQUIRED_LIBS) : make_«nameLower»

        make_«nameLower» :
        «TAB»$(MAKE) -C $(SL_ROOT)/«nameLower» all install


        -include $(DEPS_FILES)

        # -p to avoid warning if directory exists (can be improved with checks)
        make_folders :
        «TAB»@mkdir -p $(DIR_OBJS) $(DIR_LIBS) $(DIR_BINS) $(DIR_DEPS)

        clean :
        «TAB»$(REMOVE) $(OBJECTS) $(DEPS_FILES) $(EXECUTABLES)

        .PHONY = clean all make_«nameLower» make_folders
    '''

    def CMakeLists(Robot robot) '''
        # Auto-generated CMake file
        #
        #
        «val namelow = robot.name.toLowerCase»
        # Project configuration
        cmake_minimum_required(VERSION 2.8)
        project(«namelow»User)
        set(EXECUTABLE_OUTPUT_PATH ..)

        set(EIGEN_ROOT   $ENV{EIGEN_ROOT}   CACHE PATH "Path to Eigen headers")

        set(COMMON_LIBS «namelow»common SLcommon utilities)

        # Include directories
        include_directories(${SL_ROOT}/utilities/include)
        include_directories(${SL_ROOT}/lwpr/include)
        include_directories(${SL_ROOT}/SL/include)
        include_directories(${SL_ROOT}/SLRemote/include)
        include_directories(${SL_ROOT}/${ROBOT}/math)
        include_directories(${SL_ROOT}/${ROBOT}/include)
        include_directories(include)

        include_directories(${IIT_RBD_ROOT})
        include_directories(${EIGEN_ROOT})

        if (XENO)
          # Add executable
          add_executable(xenotask
            ${SOURCE_SL_TASK}
            ${SOURCE_TASK}
            src/initUserTasks.c
            src/sample_task.c)

          target_link_libraries(xenotask
            ${COMMON_LIBS}
            ${XENOTASK_LIBS})

          # Add executable
          add_executable(xenoopengl
            ${SOURCE_SL_OPENGL}
            ${SOURCE_OPENGL}
            src/initUserGraphics.c)

          target_link_libraries(xenoopengl
            ${COMMON_LIBS}
            ${XENOOPENGL_LIBS})
        else (XENO)
            # Executables:

          add_executable(xtask
            ${SOURCE_SL_TASK}
            ${SOURCE_TASK}
            src/initUserTasks.c)

          target_link_libraries(xtask
            ${COMMON_LIBS}
            ${XTASK_LIBS})

          # Add executable
          add_executable(xopengl
            ${SOURCE_SL_OPENGL}
            ${SOURCE_OPENGL}
            src/initUserGraphics.c)

          target_link_libraries(xopengl
            «namelow»common SLcommon utilities
            ${XOPENGL_LIBS})

          # Add executable
          add_executable(xsim
            ${SOURCE_SL_SIM}
            ${SOURCE_SIM}
            src/initUserSimulation.c)

          target_link_libraries(xsim
            ${COMMON_LIBS}
            ${XSIM_LIBS})
        endif (XENO)
    '''
}