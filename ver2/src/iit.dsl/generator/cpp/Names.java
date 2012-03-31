package iit.dsl.generator.cpp;

import iit.dsl.kinDsl.Robot;

public class Names {
    static public class Files {
        static public String folder(Robot robot) {
            return robot.getName().toLowerCase();
        }
        static public String mainHeader(Robot robot) {
            return "declarations";
        }

        static public String transformsHeader(Robot robot) {
            return "transforms";
        }
        static public String transformsSource(Robot robot) {
            return "transforms";
        }

        static public String jacobiansHeader(Robot robot) {
            return "jacobians";
        }

        static public class RBD {
            static public String header(Robot r) {
                return "dynamics";
            }
            static public String source(Robot r) {
                return "dynamics";
            }
            static public String testMain(Robot r) {
                return "main_test";
            }
            static public String inertiaMatrixHeader(Robot r) {
                return "jspaceM";
            }
            static public String inertiaMatrixTestMain(Robot r) {
                return "jspaceM_test";
            }
            static public String main_benchmarkID(Robot r) {
                return "main_benchmarkID";
            }
        }
    }

    static public class Namespaces {
        static final public String transforms6D = "transforms6D";
        static final public String T6D_force = "force";
        static final public String enclosing = "iit";
        static final public String rbd = "rbd";
        static public String rob(Robot r) {
            return r.getName();
        }
        static final public String jacobians = "jacs";
        static final public String internal = "internal";

        static public class Qualifiers {
            static public String robot(Robot rob) {
                return enclosing + "::" + rob(rob);
            }
            static public String roboJacs(Robot rob) {
                return robot(rob) + "::" + jacobians;
            }
        }

    }

    static public class Types {
        static final public String jointState = "JointState";
        public static String jstateDependentMatrix(Robot model, int matrixSize) {
            return "iit::rbd::JStateDependentMatrix<" + jointState + ", " + matrixSize + ", " + matrixSize + ">";
        }
        public static String jstateDependentMatrix() {
            return "iit::rbd::JStateDependentMatrix";
        }
        public static final String jacobianLocal = "JacobianT";
        public static final String jspaceMLocal = "JSpaceInertiaMatrix";
    }

    static public class GlobalVars {
        static final public String jsInertia = "jspaceM";
    }
}
