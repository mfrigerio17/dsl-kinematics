package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot
import iit.dsl.kinDsl.Joint


class RobotHeaders {
    private iit.dsl.generator.Common common = new iit.dsl.generator.Common()

    def main(Robot robot)'''
        #ifndef IIT_«Names$Files::mainHeader(robot).toUpperCase()»_H_
        #define IIT_«Names$Files::mainHeader(robot).toUpperCase()»_H_

        #include <Eigen/Dense>

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        static const int JointSpaceDimension = «robot.joints.size»;
        static const int jointsCount = «robot.joints.size»;
        /** The total number of rigid bodies of this robot, including the base */
        static const int linksCount  = «robot.links.size + 1»;

        typedef Eigen::Matrix<double, «robot.joints.size», 1> Column«robot.joints.size»d;
        typedef Column«robot.joints.size»d JointState;

        enum JointIdentifiers {
            «Common::jointIdentifier(robot.joints.get(0))» = 0
            «FOR Joint j : robot.joints.drop(1)»
            , «Common::jointIdentifier(j)»
            «ENDFOR»
        };

        enum LinkIdentifiers {
            «Common::linkIdentifier(robot.base)» = 0
            «FOR l : robot.links»
            , «Common::linkIdentifier(l)»
            «ENDFOR»
        };

        }
        }
        #endif
        '''

    def static linkDataMap_type() '''LinkDataMap'''
    def linkDataMap(Robot robot) '''
        «val className = linkDataMap_type()»
        #ifndef IIT_«robot.name.toUpperCase()»_«Names$Files::linkDataMapHeader(robot).toUpperCase()»_H_
        #define IIT_«robot.name.toUpperCase()»_«Names$Files::linkDataMapHeader(robot).toUpperCase()»_H_

        #include "«Names$Files::mainHeader(robot)».h"

        namespace «Names$Namespaces::enclosing» {
        namespace «Names$Namespaces::rob(robot)» {

        /**
         * A very simple container to associate a generic data item to each link
         */
        template<typename T> class «className» {
        private:
            T data[linksCount];
        public:
            «className»() {};
            «className»(const T& defaultValue);
            «className»(const «className»& rhs);
            «className»& operator=(const «className»& rhs);
                  T& operator[](LinkIdentifiers which);
            const T& operator[](LinkIdentifiers which) const;
        private:
            void copydata(const «className»& rhs);
        };

        template<typename T> inline
        «className»<T>::«className»(const T& value) {
            data[«Common::linkIdentifier(robot.base)»] = value;
            «FOR l : robot.links»
                data[«Common::linkIdentifier(l)»] = value;
            «ENDFOR»
        }

        template<typename T> inline
        «className»<T>::«className»(const «className»& rhs)
        {
            copydata(rhs);
        }

        template<typename T> inline
        «className»<T>& «className»<T>::operator=(const «className»& rhs)
        {
            if(&rhs != this) {
                copydata(rhs);
            }
            return *this;
        }

        template<typename T> inline
        T& «className»<T>::operator[](LinkIdentifiers l) {
            return data[l];
        }

        template<typename T> inline
        const T& «className»<T>::operator[](LinkIdentifiers l) const {
            return data[l];
        }

        template<typename T> inline
        void «className»<T>::copydata(const «className»& rhs) {
            data[«Common::linkIdentifier(robot.base)»] = rhs[«Common::linkIdentifier(robot.base)»];
            «FOR l : robot.links»
                data[«Common::linkIdentifier(l)»] = rhs[«Common::linkIdentifier(l)»];
            «ENDFOR»
        }

        template<typename T> inline
        std::ostream& operator<<(std::ostream& out, const «className»<T>& map) {
            out
            «FOR l : common.abstractLinks(robot)»
                << "   «l.name» = "
                << map[«Common::linkIdentifier(l)»]
            «ENDFOR»
            ;
            return out;
        }

        }
        }
        #endif
    '''
}