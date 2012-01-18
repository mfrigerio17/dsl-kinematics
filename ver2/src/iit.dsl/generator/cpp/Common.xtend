package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Joint
import iit.dsl.kinDsl.Robot

class Common {
    def public static jointIdentifier(Joint j) {
        return j.name.toUpperCase()
    }

    def public static namespace(Robot robot)'''«robot.name»'''
}