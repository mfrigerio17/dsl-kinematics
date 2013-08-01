package iit.dsl.generator.common

import iit.dsl.TransformsAccessor
import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.generator.Common
import iit.dsl.generator.common.TransformsDSLsUtils

class Transforms {
    private static TransformsDSLsUtils transformsDSLsUtils = new TransformsDSLsUtils()
    private static TransformsAccessor transformsAccessor = new TransformsAccessor()
    private static iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()
    private static Common common = new Common()


    def static iit.dsl.coord.coordTransDsl.Model getTransformsModel(
        Robot robot,
        iit.dsl.transspecs.transSpecs.DesiredTransforms userTransforms)
    {
        return  transformsAccessor.getModel(
            transformsDSLsUtils.coordinateTransformsDSLDoc(robot, userTransforms).toString() )
    }

    def static iit.dsl.coord.coordTransDsl.Model getTransformsModel(Robot robot) {
        val modelDoc = transformsDSLsUtils.coordinateTransformsDSLDoc(robot, null)
        return  transformsAccessor.getModel(modelDoc.toString());
    }

    def static iit.dsl.coord.coordTransDsl.Transform getTransform(
        iit.dsl.coord.coordTransDsl.Model model, AbstractLink left, AbstractLink right)
    {
        return coordTransCommon.getTransform(model,
                common.getFrameName(left).toString(),
                common.getFrameName(right).toString())
    }

    def static l1_X_l2__defaultName(iit.dsl.coord.coordTransDsl.Model model,
        AbstractLink l1, AbstractLink l2)
    {
        return coordTransCommon.name(
            coordTransCommon.getTransform(model,
                common.getFrameName(l1).toString(),
                common.getFrameName(l2).toString())  )
    }

    def static l2_X_l1__defaultName(iit.dsl.coord.coordTransDsl.Model model,
        AbstractLink l1, AbstractLink l2)
    {
        return coordTransCommon.name(
            coordTransCommon.getTransform(model,
                common.getFrameName(l2).toString(),
                common.getFrameName(l1).toString())  )
    }





}