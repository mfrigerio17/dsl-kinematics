package iit.dsl.generator.cpp.config

import iit.dsl.generator.cpp.config.IConfigurator
import iit.dsl.kinDsl.Robot
import java.util.ArrayList

/**
 * A default implementation of the different configuration interfaces defined
 * within the IConfigurator interface.
 *
 * This implementation has hardcoded, default values for the various names
 * to be returned by the functions. Therefore, note that using this configurator
 * without overriding any of its functions is likely not to work, especially
 * because of the functions returning paths.
 */
class DefaultConfigurator implements
    IConfigurator$Names$Files,
    IConfigurator$Names$Namespaces,
    IConfigurator$Names$ClassesAndTypes,
    IConfigurator$Paths

{

    override folder(Robot robot) {
        robot.getName().toLowerCase()
    }

    override h_declarations(Robot robot) {
        "declarations"
    }

    override h_fwddyn(Robot r) {
        "forward_dynamics"
    }

    override h_inertias(Robot r) {
        "inertia_params"
    }

    override h_invdyn(Robot r) {
        "inverse_dynamics"
    }

    override h_jacobians(Robot robot) {
        "jacobians"
    }

    override h_jointDataMap(Robot robot) {
        "joint_data_map"
    }

    override h_jsim(Robot r) {
        "jsim"
    }

    override h_linkDataMap(Robot robot) {
        "link_data_map"
    }

    override h_transforms(Robot robot) {
        "transforms"
    }

    override h_parameters(Robot robot) {
        "parameters_utils"
    }

    override src_fwddyn(Robot r) {
        h_fwddyn(r)
    }

    override src_inertias(Robot r) {
        h_inertias(r)
    }

    override src_invdyn(Robot r) {
        h_invdyn(r)
    }

    override src_jacobians(Robot r) {
        h_jacobians(r)
    }

    override src_jsim(Robot r) {
        h_jsim(r)
    }

    override src_transforms(Robot r) {
        h_transforms(r)
    }




    // NAMESPACES //

    override dynamics() {
        "dyn"
    }

    override enclosing() {
        val foo = new ArrayList<String>()
        foo.add("iit")
        return foo
    }

    override iit_rbd() {
        val foo = new ArrayList<String>()
        foo.add("iit")
        foo.add("rbd")
        return foo
    }

    override robot(Robot r) {
        r.name
    }



    // TYPE NAMES //

    override transforms_homogeneous() {
        "HomogeneousTransforms"
    }

    override transforms_spatial_force() {
        "ForceTransforms"
    }

    override transforms_spatial_motion() {
        "MotionTransforms"
    }


    // PATHS //

    override maximaCodeJacobians() {
        "gen_code/maxima"
    }

    override maximaCodeTransforms() {
        maximaCodeJacobians()
    }

    override maximaLibs() {
        "maxima_libs"
    }



}