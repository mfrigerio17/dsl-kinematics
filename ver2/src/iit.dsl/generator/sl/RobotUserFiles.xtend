package iit.dsl.generator.sl

import iit.dsl.kinDsl.Robot

import com.google.inject.Inject
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.cpp.Names

class RobotUserFiles {
    @Inject extension iit.dsl.generator.Common common
    @Inject extension iit.dsl.generator.sl.Common slCommon


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

        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/x«nameLowerCase»,$(BINDIR))
        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/xmotor,$(BINDIR))
        InstallProgram($(LAB_ROOT)/«nameLowerCase»/$(MACHTYPE)/xvision,$(BINDIR))

        CPPProgramListTarget( xtask, $(OBJS_X«nameUpperCase») $(CPP_OBJS_X«nameUpperCase»),$(LIB_TASK) )
        CPPProgramListTarget( xopengl, $(OBJS_XOPENGL), $(LIB_OPENGL) )
        CPPProgramListTarget( xsimulation, $(OBJS_XSIM), $(LIB_SIM) )

        NormalObjRule( $(OBJECTS) )

        MYSRCS = «robot.benchmarkFileName()».cpp «robot.name»_dynamics.cpp
        MYOBJS = «robot.benchmarkFileName()».o   «robot.name»_dynamics.o
        MYLIB = /usr/local/lib/librbd.a

        CPPProgramListTarget(foo, $(MYOBJS) $(OBJS_XSIM)  $(MYLIB), $(LIB_SIM))
        NormalCPPObjRule( $(MYOBJS) )'''


    def benchmarkMain(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include "«robot.name»_dynamics.h"

        #include "SL.h"
        #include "SL_user.h"
        #include "SL_kinematics_body.h" //"SL_kinematics.h" does not work
        #include "SL_dynamics.h"

        using namespace std;
        using namespace «Names$Namespaces::enclosing»;

        static void fillState(«robotNS»::«Names$Types::jointState»& q, «robotNS»::«Names$Types::jointState»& qd, «robotNS»::«Names$Types::jointState»& qdd, SL_DJstate* desiredState);

        /* This main is supposed to be used to test the inverse dynamics routines */
        int main(int argc, char**argv)
        {
            if(argc < 2) {
                cerr << "Please provide the number of tests to perform" << endl;
                return -1;
            }
            int numOfIterations =0;//= std::atoi(argv[1]);
            int numOfTests = std::atoi(argv[1]);
            double sl[numOfTests];
            double me[numOfTests];

            double t0, duration, sl_total, my_total;
            sl_total = 0;
            my_total = 0;

            «robotNS»::«Names$Types::jointState» q, qd, qdd, tau;
            «robotNS»::Dynamics myDynamics;

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

            setDefaultEndeffector();
            for(i=0; i<=N_CART; i++) {
                basePosition.x[i]   = 0;
                basePosition.xd[i]  = 0;
                basePosition.xdd[i] = 0;
            }
            baseOrient.q[_Q0_] = 1;
            baseOrient.q[_Q1_] = 0;
            baseOrient.q[_Q2_] = 0;
            baseOrient.q[_Q3_] = 0;


            std::srand(std::time(NULL)); // initialize random number generator


            // Prints numerical results, for comparison
            fillState(q, qd, qdd, desiredState);
            SL_InvDynNE(NULL, desiredState, endeffector, &basePosition, &baseOrient);
            myDynamics.id(q, qd, qdd, tau);

            cout << "SL:" << endl
            «FOR Joint j : robot.joints»
                << desiredState[«j.name»].uff << endl
            «ENDFOR»
                ;
           cout << "Me:" << endl << tau << endl;
           return 1;

            ofstream out("«robot.name.toLowerCase()»_testdata.m");
            out << "«robot.name.toLowerCase()»_test.iterations = [";

            for(t=0; t<numOfTests; t++) {
                sl_total = 0;
                my_total = 0;
                numOfIterations = std::pow(10,t+1);
                out << numOfIterations << " ";

                for(i=0; i<numOfIterations; i++) {
                    fillState(q, qd, qdd, desiredState);

                    t0 = std::clock();
                    SL_InvDynNE(NULL, desiredState, endeffector, &basePosition, &baseOrient);
                    duration = std::clock() - t0;
                    sl_total += duration;

                    t0 = std::clock();
                    myDynamics.id(q, qd, qdd, tau);
                    duration = std::clock() - t0;
                    my_total += duration;
                }
                sl[t] = sl_total/CLOCKS_PER_SEC;
                me[t] = my_total/CLOCKS_PER_SEC;
            }


            out << "];" << endl;
            // SL times
            out << "«robot.name.toLowerCase()»_test.sl = [";
            for(t=0; t<numOfTests; t++) {
                out << sl[t] << " ";
            }
            out << "];" << endl;
            // My times
            out << "«robot.name.toLowerCase()»_test.me = [";
            for(t=0; t<numOfTests; t++) {
                out << me[t] << " ";
            }
            out << "];" << endl;

            for(t=0; t<numOfTests; t++) {
                cout << "SL: " << sl[t] << "\t Me: " << me[t] << endl;
            }

            return TRUE;
        }


        void fillState(«robotNS»::«Names$Types::jointState»& q, «robotNS»::«Names$Types::jointState»& qd, «robotNS»::«Names$Types::jointState»& qdd, SL_DJstate* desiredState) {
            static const double max = 12.3;
        «FOR Joint j : robot.joints»
            q(«j.getID()-1»)   = ( ((double)std::rand()) / RAND_MAX) * max;
            qd(«j.getID()-1»)  = ( ((double)std::rand()) / RAND_MAX) * max;
            qdd(«j.getID()-1») = ( ((double)std::rand()) / RAND_MAX) * max;

            desiredState[«j.name»].th   = q(«j.getID()-1»);
            desiredState[«j.name»].thd  = qd(«j.getID()-1»);
            desiredState[«j.name»].thdd = qdd(«j.getID()-1»);
        «ENDFOR»
        }'''



        def main_inertiaM(Robot robot) '''
        «val robotNS = Names$Namespaces::rob(robot)»
        #include <iostream>
        #include <fstream>
        #include <ctime>

        #include "«Names$Files$RBD::inertiaMatrixHeader(robot)».h"

        #include "SL.h"
        #include "SL_user.h"
        #include "SL_kinematics_body.h" //"SL_kinematics.h" does not work
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
            «robotNS»::«Names$GlobalVars::jsInertia»(q);

            «robotNS»::«Names$Types::jspaceMLocal» SLM;

            // Copies the matrix of SL into an Eigen matrix
            «FOR Joint jo : robot.joints»
                «FOR Joint ji : robot.joints»
                SLM(«robotNS»::«jo.name»,«robotNS»::«ji.name») = rbdM[«jo.name»][«ji.name»];
                «ENDFOR»
            «ENDFOR»

            cout << "SL:" << endl << (SLM.array().abs() < 1E-4).select(0, SLM) << endl;
            cout << "Me:" << endl <<
                    («robotNS»::«Names$GlobalVars::jsInertia».array().abs() < 1E-4).select(0,«robotNS»::«Names$GlobalVars::jsInertia»)  << endl;

            //cout << SLM.block<6,6>(0,0) << endl;
            //cout << «robotNS»::«Names$GlobalVars::jsInertia».block<6,6>(0,0) << endl;

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
}