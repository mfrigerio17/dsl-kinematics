package iit.dsl.generator.cpp;

import java.util.Arrays;
import java.util.List;

import iit.dsl.generator.cpp.config.IConfigurator;

import iit.dsl.kinDsl.Robot;

public class Names {
    private static IConfigurator.Names.Files files = null;
    private static IConfigurator.Names.Namespaces nspaces = null;

    public static void setConfigurators(IConfigurator.Names.Files filesCfg) {
        files = filesCfg;
    }
    public static void setConfigurators(IConfigurator.Names.Namespaces nspacesCfg) {
        nspaces = nspacesCfg;
    }

    public static void setConfigurators(
            IConfigurator.Names.Files filesCfg,
            IConfigurator.Names.Namespaces nspacesCfg)
    {
        files = filesCfg;
        nspaces = nspacesCfg;
    }

    static public class Files {
        static public String folder(Robot robot) {
            return (files == null? robot.getName().toLowerCase() : files.folder(robot));
        }
        static public String mainHeader(Robot robot) {
            return (files == null? "declarations" : files.h_declarations(robot));
        }
        static public String linkDataMapHeader(Robot robot) {
            return (files == null? "link_data_map" : files.h_linkDataMap(robot));
        }
        static public String jointDataMapHeader(Robot robot) {
            return (files == null? "joint_data_map" : files.h_jointDataMap(robot));
        }
        static public String transformsHeader(Robot robot) {
            return (files == null? "transforms" : files.h_transforms(robot));
        }
        static public String jacobiansHeader(Robot robot) {
            return (files == null? "jacobians" : files.h_jacobians(robot));
        }
        static public String transformsSource(Robot robot) {
            return (files == null? "transforms" : files.src_transforms(robot));
        }


        static public class RBD {
            static public String header(Robot r) {
                return (files == null? "inverse_dynamics" : files.h_invdyn(r));
            }
            static public String source(Robot r) {
                return (files == null? "inverse_dynamics" : files.src_invdyn(r));
            }
            static public String jsimHeader(Robot r) {
                return (files == null? "jsim" : files.h_jsim(r));
            }
            static public String inertiaHeader(Robot r) {
                return (files == null? "inertia_params" : files.h_inertias(r));
            }
            static public String inertiaSource(Robot r) {
                return (files == null? "inertia_params" : files.src_inertias(r));
            }


            static public String testMain(Robot r) {
                return "test";
            }
            static public String jsimTestMain(Robot r) {
                return "jsim_test";
            }
            static public String main_benchmarkID(Robot r) {
                return "benchmarkID";
            }
            static public String main_sine_task_ID(Robot r) {
                return "main_sine_task_ID";
            }
            static public String main_jsim_test(Robot r) {
                return "test_jsim";
            }
            static public String abaHeader(Robot r) {
                return "forward_dynamics";
            }
        }

    }

    static public class Namespaces {
        static public String transforms6D =
                nspaces==null? "motion_transf" : nspaces.T6D_motion();
        static public String T6D_force =
                nspaces==null? "force_transf" :  nspaces.T6D_force();
        static public String THomogeneous =
                nspaces==null? "homogeneous_transf" : nspaces.THomogeneous();

        static public List<String> enclosing() {
            return
            nspaces==null?
                    Arrays.asList(new String[]{"iit"}) :
                        nspaces.enclosing();
        }

        static public String rob(Robot r) {
            return nspaces==null? r.getName() : nspaces.robot(r);
        }

        static public String dynamics  = nspaces==null? "dyn"  : nspaces.dynamics();
        static public String jacobians = nspaces==null? "jacs" : nspaces.jacobians();
        static public String internal = "internal";

        static public class Qualifiers {
            static public String robot(Robot rob) {
                return Common.enclosingNamespacesQualifier() + "::" + rob(rob);
            }
            static public String roboJacs(Robot rob) {
                return robot(rob) + "::" + jacobians;
            }
            static public String iit_rbd() {
                if(nspaces==null) return "iit::rbd";
                String ret = "";
                List<String> names = nspaces.iit_rbd();
                int len = names.size();
                for(int i=0; i<len; i++ ) {
                    ret += names.get(i);
                    if(! (i==len-1) ) ret += "::";
                }
                return ret;
            }
        }

    }

    static public class Types {
        static public String jointState = "JointState";
        public static String jstateDependentMatrix(Robot model, int matrixSize) {
            return "iit::rbd::JStateDependentMatrix<" + jointState + ", " + matrixSize + ", " + matrixSize + ">";
        }
        public static String jstateDependentMatrix() {
            return "iit::rbd::JStateDependentMatrix";
        }
        public static String jacobianLocal = "JacobianT";
        public static String jspaceMLocal = "JSpaceInertiaMatrix";
        public static String vector3d = Namespaces.Qualifiers.iit_rbd() + "::Vector3d";
        public static String extForces = "ExtForces";
    }

    static public class GlobalVars {
        static public String jsInertia = "jsim";
    }
}
