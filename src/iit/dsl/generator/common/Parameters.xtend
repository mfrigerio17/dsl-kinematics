package iit.dsl.generator.common

import iit.dsl.kinDsl.Vector3
import iit.dsl.kinDsl.ParameterLiteral
import iit.dsl.kinDsl.Var
import iit.dsl.kinDsl.Expr
import iit.dsl.kinDsl.Identifier
import iit.dsl.kinDsl.Robot

import java.util.ArrayList
import java.util.HashSet
import java.util.List
import iit.dsl.kinDsl.InertiaParams
import java.util.Set
import iit.dsl.generator.Common
import iit.dsl.kinDsl.FloatLiteral

/**
 * Utilities related to the parameters (i.e. non-constants) possibly included
 * in the robot description files
 */
class Parameters {
    /**
     * \name Parameters-group names
     *
     * The name of the group containing the length/angular parameters of a
     * robot.
     * Such a name will be used in the document of the Motion-DSL and therefore
     * in the one of the Transforms-DSL related to a robot, and will appear
     * also in the generated code (eg as the type of the C++ struct containing
     * the actual values of the parameters)
     */
    ///@{
    def public static String lengthsGroupName() { "lengths" }
    def public static String anglesGroupName()  { "angles" }
    ///@}

    def public static ParameterLiteral asParameter(Var v) {
        if( isParameter(v) ) {
            return cast(v)
        }

        if(v instanceof FloatLiteral) {
            throw new RuntimeException("Cannot cast the constant " +
                v.value + " to a ParameterLiteral")
        }
        if(v instanceof Expr) {
            throw new RuntimeException("Cannot cast the expression " +
                v.identifier.varname + " to a ParameterLiteral")
        }
        throw new RuntimeException("Could not cast the given variable " +
                "to a ParameterLiteral")
    }

    /**
     * Returns a list of the ParameterLiteral instances referenced by the given
     * vector.
     *
     * The list will be empty if the components of the vector are all
     * constants. Two parameters are considered to be the same if they have the
     * same name. The returned list does not contain duplicates.
     */
    def static getParameters(Vector3 vec) {
        var String pxName = ""
        var String pyName = ""
        val list = new ArrayList<ParameterLiteral>()
        if(isParameter(vec.x)) {
            list.add( (vec.x as Expr).identifier as ParameterLiteral)
            pxName = (vec.x as Expr).identifier.varname
        }
        if(isParameter(vec.y)) {
            pyName = (vec.y as Expr).identifier.varname
            if( ! pxName.equals(pyName) ) {
                list.add( (vec.y as Expr).identifier as ParameterLiteral)
            }
        }
        if(isParameter(vec.z)) {
            val pzName = (vec.z as Expr).identifier.varname
            if( (!pxName.equals(pzName))  &&  (!pyName.equals(pzName)) ) {
                list.add( (vec.z as Expr).identifier as ParameterLiteral)
            }
        }
        return list
    }

    /**
     * Tells whether one or more components of the given vector is a user
     * parameter
     */
    def static boolean isParametric(Vector3 vec) {
        return isParameter(vec.x) || isParameter(vec.y) || isParameter(vec.z)
    }

    /**
     * Tells whether the pose of one ore more joint-frames of the given robot
     * depends on a user parameter.
     * @return true if there is at least one joint whose geometric attributes
     *     (position and orientation of the coordinate frame, wrt to the
     *     supporting link) is parametric; false otherwise (ie all the geometric
     *     information for the robot amounts to numerical constants)
     */
    def static boolean kinematicsIsParametric(Robot robot) {
        for(j : robot.joints) {
            if( isParametric(j.refFrame.translation) ) return true
            if( isParametric(j.refFrame.rotation) )    return true
        }
        return false
    }

    /**
     * Fills two lists with the length and angular parameters of the joints of
     * the given robot.
     * The lists will not contain duplicates, e.g. multiple occurrences of the
     * same parameter, which may appear in multiple locations in the robot
     * model. Two parameters are different if they have a different name.
     */
    def static getJointsParameters(Robot robot,
        List<ParameterLiteral> lengths,
        List<ParameterLiteral> angles)
    {
        val lengthsSet  = new HashSet<String>()
        val anglesSet   = new HashSet<String>()
        var List<ParameterLiteral> lengthsTmplist = null
        var List<ParameterLiteral>  anglesTmplist = null

        for(j : robot.joints) {
            lengthsTmplist = getParameters(j.refFrame.translation)
             anglesTmplist = getParameters(j.refFrame.rotation)
            for(p : lengthsTmplist) {
                if( ! lengthsSet.contains(p.varname) ) {
                    lengthsSet.add(p.varname)
                    lengths.add(p)
                }
            }
            for(p : anglesTmplist) {
                if( ! anglesSet.contains(p.varname) ) {
                    anglesSet.add(p.varname)
                    angles.add(p)
                }
            }
        }
    }

    /**
     * Converts a length parameter of the Kinematics-DSL to one of the
     * Transforms-DSL
     */
    def public static iit.dsl.coord.coordTransDsl.ParameterLiteral
        convertLengthParam(
            iit.dsl.coord.coordTransDsl.Model transforms,
            ParameterLiteral param)
    {
        val fqn = new iit.dsl.coord.generator.ParameterFQN
        fqn.groupName = lengthsGroupName
        fqn.paramName = param.varname
        return getParam(transforms, fqn)
    }
    /**
     * Converts an agular parameter of the Kinematics-DSL to one of the
     * Transforms-DSL
     */
    def public static iit.dsl.coord.coordTransDsl.ParameterLiteral
        convertAngleParam(
            iit.dsl.coord.coordTransDsl.Model transforms,
            ParameterLiteral param)
    {
        val fqn = new iit.dsl.coord.generator.ParameterFQN
        fqn.groupName = anglesGroupName
        fqn.paramName = param.varname
        return getParam(transforms, fqn)
    }

    /**
     * Returns a ParameterLiteral of the Transforms-DSL, given a model and
     * the fully qualified name of the parameter itself
     */
    def public static iit.dsl.coord.coordTransDsl.ParameterLiteral
        getParam(
            iit.dsl.coord.coordTransDsl.Model transforms,
            iit.dsl.coord.generator.ParameterFQN fqn)
    {
        return iit::dsl::coord::generator::Common::getParameterFromName(transforms, fqn)
    }

    /**
     * \name Parameters-group getters
     * These methods return the iit.dsl.coord.coordTransDsl.ParametersDeclaration
     * instance that corresponds either to the length or the angular parameters
     * of the robot.
     */
    ///@{
    def public static iit.dsl.coord.coordTransDsl.ParametersDeclaration
        getLengthParamsGroup(iit.dsl.coord.coordTransDsl.Model transforms)
    {
        iit::dsl::coord::generator::Common::getParametersGroupFromName(transforms, lengthsGroupName)
    }
    def public static iit.dsl.coord.coordTransDsl.ParametersDeclaration
        getAngleParamsGroup(iit.dsl.coord.coordTransDsl.Model transforms)
    {
        iit::dsl::coord::generator::Common::getParametersGroupFromName(transforms, anglesGroupName)
    }

    def public static iit.dsl.coord.coordTransDsl.ParametersDeclaration
        getLengthParamsGroup(Robot robot)
    {
        getLengthParamsGroup(
            iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        )
    }
    def public static iit.dsl.coord.coordTransDsl.ParametersDeclaration
        getAngleParamsGroup(Robot robot)
    {
        getAngleParamsGroup(
            iit::dsl::generator::common::Transforms::getTransformsModel(robot)
        )
    }
    ///@}


    def public static boolean isParametric(InertiaParams inertia) {
        return isParameter(inertia.mass) ||
                isParametric(inertia.com) ||
                isParameter(inertia.ix)||isParameter(inertia.iy)||
                isParameter(inertia.iz)||isParameter(inertia.ixy)||
                isParameter(inertia.ixz)||isParameter(inertia.iyz)
    }

    def public static ParametricInertiaFlags whoIsParametric(InertiaParams inertia)
    {
        val ret = new ParametricInertiaFlags
        ret.mass = isParameter(inertia.mass)
        ret.com  = isParametric(inertia.com)
        ret.tensor = isParameter(inertia.ix) || isParameter(inertia.iy) ||
                     isParameter(inertia.iz) || isParameter(inertia.ixy)||
                     isParameter(inertia.ixz)|| isParameter(inertia.iyz)
        return ret
    }

    def public static boolean massPropertiesAreParametric(Robot robot) {
        for(l : common.abstractLinks(robot)) {
            if( isParametric(l.inertiaParams) ) return true
        }
        return false
    }


    def public static  List<ParameterLiteral> getInertiaParameters(Robot robot)
    {
        val list = new ArrayList<ParameterLiteral>
        val namesPool = new HashSet<String>
        if(common.isFloating(robot.base)) {
            getParameters(robot.base.inertiaParams, list, namesPool)
        }
        for(l : robot.links) {
            getParameters(l.inertiaParams, list, namesPool)
        }
        return list
    }

    /**
     * The list of parameters (ie non-constant values) referenced by the given
     * inertia properties of a link.
     *
     * Note that parameters with the same name are considered to be the same
     * thing, regardless the field (mass, COM or moments) in which they appear,
     * and won't be added multiple times. A common parameter in the mass, COM
     * or the inertia moments, conceptually does not make sense (because you
     * would have the same parameter representing simultaneously different
     * physical quantities, but syntactically is possible, in the documents.
     */
    def public static List<ParameterLiteral> getParameters(InertiaParams inertia)
    {
        val list = new ArrayList<ParameterLiteral>
        val namesPool = new HashSet<String>

        getParameters(inertia, list, namesPool)

        return list
    }

    def private static getParameters(
        InertiaParams inertia,
        List<ParameterLiteral> list,
        Set<String> namesPool)
    {
        addParameterIfAny(inertia.mass, list, namesPool)

        addParameterIfAny(inertia.com.x, list, namesPool)
        addParameterIfAny(inertia.com.y, list, namesPool)
        addParameterIfAny(inertia.com.z, list, namesPool)

        addParameterIfAny(inertia.ix,  list, namesPool)
        addParameterIfAny(inertia.iy,  list, namesPool)
        addParameterIfAny(inertia.iz,  list, namesPool)
        addParameterIfAny(inertia.ixy, list, namesPool)
        addParameterIfAny(inertia.ixz, list, namesPool)
        addParameterIfAny(inertia.iyz, list, namesPool)
    }

    def private static boolean addParameterIfAny(
        Var v,
        List<ParameterLiteral> list,
        Set<String> namesSet)
    {
        if( ! isParameter(v) ) return false
        val param = cast(v)
        if( namesSet.contains(param.varname) ) return false

        namesSet.add(param.varname)
        list.add(param)
    }


    def public static dispatch boolean isParameter(Var x) { false }
    def public static dispatch boolean isParameter(Expr x) { isParameter(x.identifier) }
    def public static dispatch boolean isParameter(Identifier x) { false }
    def public static dispatch boolean isParameter(ParameterLiteral x) { true }

    def private static ParameterLiteral cast(Var x) {
        return ((x as Expr).identifier as ParameterLiteral)
    }

    private static Common common = new Common
}

class ParametricInertiaFlags {
    public boolean mass;
    public boolean com;
    public boolean tensor;
    def public boolean any() { return mass || com || tensor }
    def public boolean none(){ return !any() }
    def public boolean all() { return mass && com && tensor }
}