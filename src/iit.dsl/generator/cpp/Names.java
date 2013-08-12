package iit.dsl.generator.cpp;

import java.util.List;

import iit.dsl.generator.cpp.config.DefaultConfigurator;
import iit.dsl.generator.cpp.config.IConfigurator;

import iit.dsl.kinDsl.Robot;

public class Names {
    private static IConfigurator.Names.Files files = null;
    private static IConfigurator.Names.Namespaces nspaces = null;
    private static IConfigurator.Names.ClassesAndTypes types = null;

    static {
        DefaultConfigurator config = new DefaultConfigurator();
        setConfigurators(config, config, config);
    }

    public static void setConfigurators(IConfigurator.Names.Files filesCfg)
    {
        if(filesCfg == null) {
            //TODO log warning, refusing to set null configurator
            return;
        }
        files = filesCfg;
    }
    public static void setConfigurators(IConfigurator.Names.Namespaces nspacesCfg)
    {
        if(nspacesCfg == null) {
            //TODO log warning, refusing to set null configurator
            return;
        }
        nspaces = nspacesCfg;
    }
    public static void setConfigurators(IConfigurator.Names.ClassesAndTypes typesCfg)
    {
        if(typesCfg == null) {
            //TODO log warning, refusing to set null configurator
            return;
        }
        types = typesCfg;
    }


    public static void setConfigurators(
            IConfigurator.Names.Files filesCfg,
            IConfigurator.Names.Namespaces nspacesCfg,
            IConfigurator.Names.ClassesAndTypes typesCfg)
    {
        files = filesCfg;
        nspaces = nspacesCfg;
        types = typesCfg;
    }

    static public class Files {
        static public String folder(Robot robot) {
            return files.folder(robot);
        }
        static public String mainHeader(Robot robot) {
            return files.h_declarations(robot);
        }
        static public String linkDataMapHeader(Robot robot) {
            return files.h_linkDataMap(robot);
        }
        static public String jointDataMapHeader(Robot robot) {
            return files.h_jointDataMap(robot);
        }
        static public String transformsHeader(Robot robot) {
            return files.h_transforms(robot);
        }
        static public String parametersHeader(Robot robot) {
            return files.h_parameters(robot);
        }

        static public String jacobiansHeader(Robot robot) {
            return files.h_jacobians(robot);
        }
        static public String transformsSource(Robot robot) {
            return files.src_transforms(robot);
        }


        static public class RBD {
            static public String header(Robot r) {
                return files.h_invdyn(r);
            }
            static public String source(Robot r) {
                return files.src_invdyn(r);
            }
            static public String jsimHeader(Robot r) {
                return files.h_jsim(r);
            }
            static public String inertiaHeader(Robot r) {
                return files.h_inertias(r);
            }
            static public String inertiaSource(Robot r) {
                return files.src_inertias(r);
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
        static public List<String> enclosing() {
            return nspaces.enclosing();
        }

        static public String rob(Robot r) {
            return nspaces.robot(r);
        }

        static public String dynamics  = nspaces.dynamics();
        static public String jacobians = nspaces.jacobians();
        static public String internal = "internal";

        static public class Qualifiers {
            static public String robot(Robot rob) {
                return Common.enclosingNamespacesQualifier() + "::" + rob(rob);
            }
            static public String roboJacs(Robot rob) {
                return robot(rob) + "::" + jacobians;
            }
            static public String iit_rbd() {
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

        public static String jacobianLocal = "JacobianT";
        public static String jspaceMLocal = "JSIM";
        public static String vector3d = Namespaces.Qualifiers.iit_rbd() + "::Vector3d";
        public static String extForces = "ExtForces";

        /**
         * The names of the classes that contain the various coordinate transforms.
         * The code of such classes is generated by generators in the Transforms-DSL
         * package
         * @author Marco Frigerio
         */
        static public class Transforms {
            static public String homogeneous() {
                return types.transforms_homogeneous();
            }
            static public String spatial_motion() {
                return types.transforms_spatial_motion();
            }
            static public String spatial_force() {
                return types.transforms_spatial_force();
            }
        }

        static public class IIT_RBD {
            static public String stateDependentMatrix(
                    String state, String rows, String cols, String matrix) {
                return "StateDependentMatrix<" +state+ ", " +rows+ ", " +cols+ ", " +matrix+ ">";
            }
            static public String jacobianBase(String state, String cols, String matrix) {
                return "JacobianBase<" + state + ", " + cols + ", " + matrix + ">";
            }
        }
    }

    static public class GlobalVars {
        static public String jsInertia = "jsim";
    }
}
