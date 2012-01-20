package iit.dsl.generator.cpp;

import iit.dsl.kinDsl.Robot;

public class Names {
    static public class Files {
        static public String mainHeader(Robot robot) {
            return robot.getName() + "_declarations";
        }

        static public String transformsHeader(Robot robot) {
            return robot.getName() + "_transforms";
        }

        static public class RBD {
            static public String header(Robot r) {
                return r.getName() + "_dynamics";
            }
            static public String source(Robot r) {
                return r.getName() + "_dynamics";
            }
            static public String testMain(Robot r) {
                return r.getName() + "_main";
            }
        }
    }

    static public class Namespaces {
        static final public String transforms6D = "transforms6D";
        static final public String enclosingQualifier = "iit";
        static public String rob(Robot r) {
            return r.getName();
        }
    }

    static public class TypeNames {
        static final public String jointState = "JointState";
    }
}
