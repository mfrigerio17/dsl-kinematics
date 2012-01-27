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
import iit.dsl.coord.coordTransDsl.VariableLiteral

import java.util.Set
import java.util.HashSet

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

        val Robot robot = base.eContainer() as Robot
        val Model transforms = transformsAccessor.getTransformsModel(robot)

        val iit.dsl.coord.generator.Common coordTransCommon = new iit.dsl.coord.generator.Common()
        val StringConcatenation strBuff = new StringConcatenation();

        var iit.dsl.coord.coordTransDsl.Transform transform
        var String maximaTransformLiteral

        transform = coordTransCommon.getTransform(transforms, bsFrame, eeFrame)
        if(transform == null) {
            throw new RuntimeException("Cannot generate the jacobian file, the transform " +
                bsFrame.name + "_X_" + eeFrame.name + " is missing");
        }
        maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(transform)
        val jacName = jacName(bsFrame.name, eeFrame.name)
        val String tempJStr = "tempJ"
        strBuff.append('''
            «tempJStr» : matrix();
            eePos : posVector(«maximaTransformLiteral»);
            '''
        );

        var i = 0
        
        val chain = common.buildChain(base, targetLink).drop(1) //drops the base itself
        val Set<VariableLiteral> variables = new HashSet<VariableLiteral>()
        for(AbstractLink el : chain) {
            tmpFrame.setName(common.getFrameName(el).toString())
            transform = coordTransCommon.getTransform(transforms, bsFrame, tmpFrame)
            variables.addAll(coordTransCommon.getVars(transform))
            if(transform == null) {
                throw new RuntimeException("Cannot generate the jacobian file, the transform " +
                    bsFrame.name + "_X_" + tmpFrame.name + " is missing");
            }
            maximaTransformLiteral = coordsMaxima.getTransformFunctionLiteral(transform)
            strBuff.append('''
                pos«i»  : posVector(«maximaTransformLiteral»);
                axis«i» : zaxis(«maximaTransformLiteral»);
                «tempJStr»  : addcol(«tempJStr», GJacobianColumn(eePos, axis«i», pos«i»));
            ''');
            i = i + 1
        }

        
        return strBuff
    }
}