package iit.dsl.generator.matlab

import java.util.Map
import java.util.HashMap

import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.Link
import iit.dsl.coord.generator.Utilities

class Common
{
    def public static dispatch spatialVectorIndex(RevoluteJoint joint) {
        return 3
    }
    def public static dispatch spatialVectorIndex(PrismaticJoint joint) {
        return 6
    }

    def public static arrayIndex(Joint j) {
        return j.ID;
    }
    /**
     * A map of the identifiers of all the transforms in the form \c child_X_parent.
     *
     * @param robot the robot of interest
     * @param mxtype the type of the transforms which is requested
     * @param structName the name of the Matlab structure whose fields are assumed
     *        to be the coordinate transformation matrices
     * @return a map that associates any link \c l of the robot to the Matlab expression
     *         which would evaluate to the transform \c l_X_parent. Such an expression
     *         is simply a reference to a field of the given structure.
     */
    def public static Map<Link, CharSequence>
    getParentToChildTransforms( Robot robot, Utilities$MatrixType mxtype, CharSequence structName )
    {
        return getTransformsMap(robot, mxtype, structName, true)
    }
    /**
     * A map of the identifiers of all the transforms in the form \c parent_X_child.
     *
     * @param robot the robot of interest
     * @param mxtype the type of the transforms which is requested
     * @param structName the name of the Matlab structure whose fields are assumed
     *        to be the coordinate transformation matrices
     * @return a map that associates any link \c l of the robot to the Matlab expression
     *         which would evaluate to the transform \c parent_X_l. Such an expression
     *         is simply a reference to a field of the given structure.
     */
    def public static Map<Link, CharSequence>
    getChildtoParentTransforms( Robot robot, Utilities$MatrixType mxtype, CharSequence structName )
    {
        return getTransformsMap(robot, mxtype, structName, false)
    }



    def private static Map<Link, CharSequence>
    getTransformsMap(
        Robot robot,
        Utilities$MatrixType mxtype,
        CharSequence structName,
        boolean parentToChild)
    {
        val transformsMap = new HashMap<Link, CharSequence>();
        val transformsModel = iit::dsl::generator::common::Transforms::getTransformsModel(robot)

        for(l : robot.links)
        {
            var iit.dsl.coord.coordTransDsl.Transform t
            if( parentToChild ) {
                t = iit::dsl::generator::common::Transforms::getTransform(transformsModel, l, l.parent)
            } else {
                t = iit::dsl::generator::common::Transforms::getTransform(transformsModel, l.parent, l)
            }
            val identifier = iit::dsl::coord::generator::matlab::Generator::identifier(t, mxtype)
            transformsMap.put(l, structName + "." + identifier)
        }
        return transformsMap
    }

    private static extension iit.dsl.generator.Common common = iit.dsl.generator.Common::getInstance()
}