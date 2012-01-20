package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Joint

class Common {
    def public static jointIdentifier(Joint j) {
        return j.name.toUpperCase()
    }
}