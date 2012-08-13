package iit.dsl.generator.matlab

import iit.dsl.kinDsl.Robot
import iit.dsl.generator.Common
import iit.dsl.TransformsAccessor
import java.io.File
import iit.dsl.generator.FramesTransforms


class PlotFrames {
    private static String transformsModelPath = "generated_code/misc" // default value
    private static TransformsAccessor transformsAccessor = new TransformsAccessor()

    def static setTransformsModelPath(String path) {
        transformsModelPath = path;
    }



    extension Common common = new Common()
    private Robot robot
    private iit.dsl.coord.coordTransDsl.Model transforms
    private iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()

    new(Robot rob) {
        robot = rob
        val transformsModelFile = new File(
            transformsModelPath + "/" + FramesTransforms::fileName(robot))
        transforms = transformsAccessor.getTransformsModel(robot, transformsModelFile)
    }

	def plotFramesCode() '''
        scaling = 0.1;
        baseColor = [0,0,1];

        x = scaling * [1;0;0];
        y = scaling * [0;1;0];

        plotRefFrame(x, y, [0;0;0], baseColor);
        «val count = robot.links.size + 1»
        «FOR l : robot.links»
            «val T    = coordTransCommon.getTransform(transforms, robot.base.frameName.toString(), l.frameName.toString())»
            «IF T == null»
                % I could not find the transform from base to link «l.name»; are you generating it?
            «ELSE»
                «val name = coordTransCommon.name(T)»
                x = scaling * «name»(1:3,1);
                y = scaling * «name»(1:3,2);
                pos = «name»(1:3,4);  % no scaling

                plotRefFrame(x, y, pos, baseColor * («count»-«l.ID»)/«count»);
            «ENDIF»
        «ENDFOR»

        axis equal;
        '''
}