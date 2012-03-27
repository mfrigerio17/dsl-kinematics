package iit.dsl.generator.maxima

import com.google.inject.Inject
import org.eclipse.xtend2.lib.StringConcatenation


import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.RevoluteJoint
import iit.dsl.kinDsl.PrismaticJoint
import iit.dsl.coord.coordTransDsl.Model
import iit.dsl.TransformsAccessor

import iit.dsl.generator.Jacobian
import iit.dsl.generator.FramesTransforms
import java.io.File

class Jacobians {
    static String transformsModelPath = "generated_code/misc" // default value

    def static setTransformsModelPath(String path) {
        transformsModelPath = path;
    }

    extension iit.dsl.generator.Common common = new iit.dsl.generator.Common()
    TransformsAccessor transformsAccessor = new TransformsAccessor()
    @Inject iit.dsl.coord.generator.Maxima coordsMaxima //use injection otherwise you have to manually initialize its sub-members

    def static fileName(Robot robot) '''«robot.name»_jacobians'''

    def jacobian(Jacobian J) {
        val iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()

        val transformsModelFile = new File(
            transformsModelPath + "/" + FramesTransforms::fileName(J.robot));
        val Model transforms = transformsAccessor.getTransformsModel(J.robot, transformsModelFile)

        val baseFrameName = J.baseFrame.name;

        // The "longest" transform, between the base link and the moving link
        val base_X_ee = coordTransCommon.getTransform(transforms, baseFrameName, J.movingFrame.name)
        if(base_X_ee == null) {
            throw new RuntimeException("Cannot generate the jacobian file: could not find the transform " +
                J.baseFrame.name + "_X_" + J.movingFrame.name + " (robot " + J.robot.name + ")");
        }
        var maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(base_X_ee)
        // Some variables of help for the code generation
        val variablesList = coordsMaxima.argsListText(base_X_ee)
        val jacName   = '''«J.getName()»(«variablesList»)'''
        val eePosName = '''«J.movingFrame.name»_pos_wrt_«J.baseFrame.name»(«variablesList»)'''

        var iit.dsl.coord.coordTransDsl.Transform transform
        val StringConcatenation strBuff = new StringConcatenation();
        strBuff.append('''
            «eePosName» := posVector(«maximaTransformLiteral»);
            «jacName» := addcol(matrix()
                ''');

        for(Joint el : J.jointsChain) {
            // Get the transform 'base_X_<current link>'
            val tmpFrameName = common.getFrameName(el).toString();
            transform = coordTransCommon.getTransform(transforms, baseFrameName, tmpFrameName)
            if(transform == null) {
                throw new RuntimeException("Cannot generate the jacobian file: could not find the transform " +
                    baseFrameName + "_X_" + tmpFrameName + " (robot " + J.robot.name + ")");
            }
            maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(transform)
            if(el instanceof RevoluteJoint) {
                strBuff.append(
                '''    , GJacobianColumn(«eePosName», zaxis(«maximaTransformLiteral»), posVector(«maximaTransformLiteral»))
                ''');
            } else if (el instanceof PrismaticJoint) {
                strBuff.append(
                '''    , GJacobianColumn_prism(zaxis(«maximaTransformLiteral»))
                ''');
            } else {
                throw new RuntimeException("Unknown joint type")
            }
        }
        strBuff.append(''');
        ''');

        return strBuff

    }


}
