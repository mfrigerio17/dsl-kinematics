package iit.dsl.generator.maxima

import com.google.inject.Inject
import org.eclipse.xtend2.lib.StringConcatenation

import iit.dsl.kinDsl.RefFrame
import iit.dsl.kinDsl.AbstractLink
import iit.dsl.kinDsl.Robot
import iit.dsl.TransformsAccessor

import iit.dsl.coord.coordTransDsl.Model
import iit.dsl.coord.coordTransDsl.impl.CoordTransDslFactoryImpl
import iit.dsl.coord.coordTransDsl.CoordTransDslFactory

class Jacobians {
    @Inject TransformsAccessor transformsAccessor
    @Inject iit.dsl.coord.generator.Maxima coordsMaxima //use injection otherwise you have to manually initialize its sub-members
    @Inject extension iit.dsl.generator.Common common

    def jacName(String baseFrameName, String targetFrameName) '''«baseFrameName»_J_«targetFrameName»'''

    def jacobian(AbstractLink base, AbstractLink targetLink) {
        return jacobian(base, targetLink, base.defaultFrame, targetLink.defaultFrame)
    }
    def jacobian(AbstractLink base, RefFrame targetFrame) {
        return jacobian(base, common.getContainingLink((base.eContainer() as Robot),targetFrame), base.defaultFrame, targetFrame)
    }
    def jacobian(RefFrame baseFrame, AbstractLink movingLink ) {
        return jacobian(common.getContainingLink((movingLink.eContainer() as Robot),baseFrame), movingLink, baseFrame, movingLink.defaultFrame)
    }
    def jacobian(Robot robot, RefFrame baseFrame, RefFrame movingFrame) {
        return jacobian(
            robot.getContainingLink(baseFrame),
            robot.getContainingLink(movingFrame),
            baseFrame, movingFrame
        )
    }

	def private jacobian(AbstractLink base, AbstractLink targetLink, RefFrame baseFr, RefFrame targetFr) {
        if(baseFr == null || targetLink == null || baseFr == null || targetFr == null) {
            throw(new RuntimeException("All arguments but be non-null"))
        }

        //Factory for the coordinate-transformation-DSL classes
        val CoordTransDslFactory factory = CoordTransDslFactoryImpl::init()
        var iit.dsl.coord.coordTransDsl.Frame eeFrame  = factory.createFrame()
        var iit.dsl.coord.coordTransDsl.Frame bsFrame  = factory.createFrame()
        var iit.dsl.coord.coordTransDsl.Frame tmpFrame = factory.createFrame()
        bsFrame.setName(baseFr.name)
        eeFrame.setName(targetFr.name)
        // Get the model with the coordinate transforms of this robot
        val Robot robot = base.eContainer() as Robot
        val Model transforms = transformsAccessor.getTransformsModel(robot)
        val iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()

        // The "longest" transform, between the base link and the moving link
        val base_X_ee = coordTransCommon.getTransform(transforms, bsFrame, eeFrame)
        if(base_X_ee == null) {
            throw new RuntimeException("Cannot generate the jacobian file, the transform " +
                bsFrame.name + "_X_" + eeFrame.name + " is missing");
        }
        var maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(base_X_ee)
        // Some variables of help for the code generation
        val variablesList = coordsMaxima.argsListText(base_X_ee)
        val jacName   = '''«jacName(bsFrame.name, eeFrame.name)»(«variablesList»)'''
        val eePosName = '''«eeFrame.name»_pos_wrt_«bsFrame.name»(«variablesList»)'''

        val chain = common.buildChain(base, targetLink).drop(1) //drops the base itself
        var iit.dsl.coord.coordTransDsl.Transform transform
        val StringConcatenation strBuff = new StringConcatenation();
        strBuff.append('''
            «eePosName» := posVector(«maximaTransformLiteral»);
            «jacName» := addcol(matrix()
                ''');
        for(AbstractLink el : chain) {
            // Get the transform 'base_X_<current link>'
            tmpFrame.setName(common.getFrameName(el).toString())
            transform = coordTransCommon.getTransform(transforms, bsFrame, tmpFrame)
            if(transform == null) {
                throw new RuntimeException("Cannot generate the jacobian file, the transform " +
                    bsFrame.name + "_X_" + tmpFrame.name + " is missing");
            }
            maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(transform)
            strBuff.append(
            '''    , GJacobianColumn(«eePosName», zaxis(«maximaTransformLiteral»), posVector(«maximaTransformLiteral»))
            ''');
        }
        strBuff.append(''');''');

        return strBuff
    }
}