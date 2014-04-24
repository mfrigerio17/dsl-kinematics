package iit.dsl.generator.cpp.config;

import java.util.List;

import iit.dsl.kinDsl.Robot;

/**
 * Configuration interface for the C++ code generation.
 *
 * The various sub-interfaces describe all the information required by the code
 * generator software of this package.
 * @author Marco Frigerio
 *
 */
public interface IConfigurator {
    /**
     * Various paths required to configure the code generation
     */
    public interface Paths {
        public String maximaCodeTransforms();
        public String maximaCodeJacobians();
        public String maximaLibs();
    }
    /**
     * Miscellaneous identifiers, like file names and namespace names
     */
    public interface Names {
        /**
         * File names
         */
        public interface Files {
            public String folder(Robot robot);

            public String h_declarations(Robot robot);
            public String h_linkDataMap(Robot robot);
            public String h_jointDataMap(Robot robot);
            public String h_transforms(Robot robot);
            public String h_parameters(Robot robot);
            public String h_mass_parameters(Robot robot);
            public String h_jacobians(Robot robot);

            public String h_inertias(Robot r);
            public String h_invdyn(Robot r);
            public String h_fwddyn(Robot r);
            public String h_jsim(Robot r);


            public String src_transforms(Robot robot);
            public String src_jacobians(Robot robot);

            public String src_inertias(Robot r);
            public String src_invdyn(Robot r);
            public String src_fwddyn(Robot r);
            public String src_jsim(Robot r);
        }

        /**
         * Namespace names
         */
        public interface Namespaces {
            public List<String> iit_rbd();
            /**
             * The list of namespaces that will enclose the generated code.
             * These names represent the outermost, user-specific namespaces,
             * in addition to those created by the generators of this package.
             * @return the list of enclosing namespaces, ordered from the
             *    outermost to the innermost.
             */
            public List<String> enclosing();
            public String robot(Robot r);

            public String dynamics();
        }

        /**
         * Type names, class names, etc.
         * @author Marco Frigerio
         */
        public interface ClassesAndTypes {
            public String transforms_homogeneous();
            public String transforms_spatial_motion();
            public String transforms_spatial_force();
        }

    }
}
