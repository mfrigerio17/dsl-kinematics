package iit.dsl.generator.cpp;

import iit.dsl.kinDsl.Robot;

public class Names {
    static class Files {
        static String mainHeader(Robot robot) {
            return robot.getName() + "_declarations";
        }

        static String transformsHeader(Robot robot) {
            return robot.getName() + "_transforms";
        }

        static class RBD {
            static String header(Robot r) {
                return r.getName() + "_dynamics";
            }
            static String source(Robot r) {
                return r.getName() + "_dynamics";
            }
        }
    }

    static class Namespaces {
        static final String transforms6D = "transforms6D";
        static final String enclosingQualifier = "iit";
        static String rob(Robot r) {
            return r.getName();
        }
    }
}
