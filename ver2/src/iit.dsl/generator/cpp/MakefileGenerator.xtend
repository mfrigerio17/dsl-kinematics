package iit.dsl.generator.cpp

import iit.dsl.kinDsl.Robot

class MakefileGenerator {
    static String basePath_eigen = "/usr/local/include/eigen3/"
    static String basePath_iitRBD = "/home/marco/iit/code/git/rbd/"

    def makefileBody(Robot robot) '''
        «val String TAB = "\t"»
        «val dir_src=""»
        REMOVE = rm -f

        DIR_OUT=./bin
        DIR_OBJS=$(DIR_OUT)
        DIR_DEPS=$(DIR_OBJS)

        CPPFLAGS = -I«basePath_eigen» -I«basePath_iitRBD»
        CXXFLAGS = -g -Wall -O3 -march=native -mtune=native -D EIGEN_NO_DEBUG

        SRCS = $(wildcard «dir_src»*.cpp)
        OBJS = $(patsubst %.cpp,$(DIR_OBJS)/%.o,$(SRCS))

        LIB = lib«robot.name.toLowerCase()».a

        all : $(OBJS)

        # ------------ #
        # OBJECT FILES #
        # ------------ #
        COMPILE = $(CXX) $(CPPFLAGS) -MMD -MF $(DIR_DEPS)/$*.d  $(CXXFLAGS) -c $< -o $@

        $(OBJS) : $(DIR_OBJS)/%.o : «dir_src»%.cpp
        «TAB»$(COMPILE)


        # ------- #
        # LIBRARY #
        # ------- #
        lib: $(DIR_OUT)/$(LIB)
        $(DIR_OUT)/$(LIB) : $(OBJS)  #TODO exclude object files with main()
        «TAB»@echo " * Building library $@ ($(notdir $^))"
        «TAB»@ar cr $@ $^

        DEPS_FILES = $(patsubst $(DIR_OBJS)/%.o,$(DIR_DEPS)/%.d,$(OBJS))
        -include $(DEPS_FILES)

        clean :
        «TAB»$(REMOVE) $(OBJS) $(DEPS_FILES) $(DIR_OUT)/$(LIB)

        .PHONY = all lib clean
	'''
}