package iit.dsl.generator.sl

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.generator.cpp.Names

class RobotUserFiles {
    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()


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

        def main_compare_fwd_dyn(Robot robot) '''
            «val robotNS = Names$Namespaces::rob(robot)»
            «val jstate = Names$Types::jointState»
            #include <iostream>
            #include <fstream>
            #include <ctime>

            #include <iit/robots/«Names$Files::folder(robot)»/«Names$Files::mainHeader(robot)».h>
            #include <iit/robots/«Names$Files::folder(robot)»/«Names$Files$RBD::fwdDynHeader(robot)».h>

            #include <SL.h>
            #include <SL_kinematics.h>
            #include <SL_dynamics.h>
            #include <SL_user.h>

            #include "matlab_log.h"

            using namespace std;
            using namespace «Names$Namespaces::enclosing»;
            using namespace «Names$Namespaces::enclosing»::«robotNS»;
            using namespace «Names$Namespaces::enclosing»::«robotNS»::«Names$Namespaces::dynamics»;

            static void fillState(«jstate»& q, «jstate»& qd, «jstate»& qdd,
                    SL_Jstate* state);

            /* This main is supposed to be used to test the inverse dynamics routines */
            int main(int argc, char**argv) {
                if(argc < 2) {
                    cerr << "Please provide the number of tests to perform" << endl;
                    return -1;
                }
                int numOfTests = std::atoi(argv[1]);
                double sl[numOfTests];
                double me[numOfTests];
                int iterations[numOfTests];

                double t0, sl_total, me_total;
                me_total = 0;
                sl_total = 0;

                int t=0,i=0,avg=0;

                «jstate» q, qd, qdd, tau;
                ForwardDynamics fwdyn;


                SL_Jstate currentState[N_ROBOT_DOFS];
                SL_endeff endeffector[N_ROBOT_ENDEFFECTORS];
                SL_Cstate basePosition;
                SL_quat baseOrient;
                SL_uext extForces[N_ROBOT_DOFS];

                init_kinematics();
                if (!init_dynamics()) {
                    cerr << "Error in init_dynamics()" << endl;
                    exit(-1);
                }

                for (i = 0; i < N_ROBOT_DOFS; i++) {
                    currentState[i].th = 0;
                    currentState[i].thd = 0;
                    currentState[i].thdd = 0;
                    currentState[i].u = 0;
                    bzero(extForces[i].f, sizeof(extForces[i].f));
                    bzero(extForces[i].t, sizeof(extForces[i].t));
                }

                endeffector[ENDEFF].m = 0; // zero mass -> no endeffector
                for (i = 0; i <= N_CART; i++) {
                    basePosition.x[i] = 0;
                    basePosition.xd[i] = 0;
                    basePosition.xdd[i] = 0;

                    endeffector[ENDEFF].mcm[i] = 0;
                    endeffector[ENDEFF].cf[i] = 0;
                    endeffector[ENDEFF].ct[i] = 0;
                    endeffector[ENDEFF].x[i] = 0;
                    endeffector[ENDEFF].a[i] = 0;
                }
                bzero(baseOrient.q  , sizeof(baseOrient.q));
                bzero(baseOrient.qd , sizeof(baseOrient.qd));
                bzero(baseOrient.qdd, sizeof(baseOrient.qdd));
                bzero(baseOrient.ad , sizeof(baseOrient.ad));
                bzero(baseOrient.add, sizeof(baseOrient.add));
                baseOrient.q[_Q0_] = 1;

                std::srand(std::time(NULL)); // initialize random number generator

            //      // Prints numerical results, for comparison
            //      fillState(q, qd, tau, currentState);
            //      SL_ForDynArt(currentState, &basePosition, &baseOrient, extForces, endeffector);
            //      fwdyn.fd(q, qd, tau, qdd);
            //      cout << "SL:" << endl
            //      «FOR Joint j : robot.joints AFTER ";"»
            //         << currentState[::«j.name»].thdd << endl«ENDFOR»
            //      cout << endl << qdd << endl;
            //      return 1;

                int numOfIterations = 1;
                int sl_elapsed = 0;
                int me_elapsed = 0;
                static const int avgFactor = 10;
                for(t=0; t<numOfTests; t++) {
                    sl_total = 0;
                    me_total = 0;
                    numOfIterations = numOfIterations * 10;
                    iterations[t] = numOfIterations;

                    for(i=0; i<numOfIterations; i++) {
                        fillState(q, qd, tau, currentState);

                        sl_elapsed = 0;
                        me_elapsed = 0;

                        for(avg=0; avg < avgFactor; avg++) {
                            t0 = std::clock();
                            SL_ForDynArt(currentState, &basePosition, &baseOrient, extForces, endeffector);
                            sl_elapsed += (std::clock() - t0); // add the time elapsed during the above call

                            t0 = std::clock();
                            fwdyn.fd(q, qd, tau, qdd);
                            me_elapsed += (std::clock() - t0);
                        }
                        // Add to the total time the average
                        sl_total += (sl_elapsed / avgFactor);
                        me_total += (me_elapsed / avgFactor);

                    }
                    // The total execution time, in seconds
                    sl[t] = sl_total/CLOCKS_PER_SEC;
                    me[t] = me_total/CLOCKS_PER_SEC;
                }


                    cout << "SL:";
                    for(int t=0; t<numOfTests; t++) {
                        cout << " " << sl[t];
                    }
                    cout << endl;

                    cout << "Robogen:";
                    for(int t=0; t<numOfTests; t++) {
                        cout << " " << me[t];
                    }
                    cout  << endl;

                    std::string roboname("«robot.name»");
                    std::string algo("forward dynamics - articulated body algorithm");
                    matlabLog(numOfTests, iterations, sl, roboname, algo, "SL");
                    matlabLog(numOfTests, iterations, me, roboname, algo, "RoboGen");

                    return 0;
            }

            void fillState(«jstate»& q, «jstate»& qd, «jstate»& tau, SL_Jstate* state) {
                static const double max = 5;
                «FOR Joint j : robot.joints»
                    «val jid = robotNS + "::" + iit::dsl::generator::cpp::Common::jointIdentifier(j)»
                    q(«jid»)   = ( ((double)std::rand()) / RAND_MAX) * max;
                    qd(«jid»)  = ( ((double)std::rand()) / RAND_MAX) * max;
                    tau(«jid») = ( ((double)std::rand()) / RAND_MAX) * max;

                    state[::«j.name»].th  =   q(«jid»);
                    state[::«j.name»].thd =  qd(«jid»);
                    state[::«j.name»].u   = tau(«jid»);

                «ENDFOR»
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
                «Names$Namespaces$Qualifiers::iit_rbd»::Matrix33d temp;
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



}