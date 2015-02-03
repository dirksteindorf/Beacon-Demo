
MIRA_REQUIRE_PACKAGE(DemoPath)

###############################################################################

MIRA_ADD_DOCUMENTATION(DemoPath
	DIRS
		doc
		include
	DEPENDS
		#RobotDataTypes
)

###############################################################################

INCLUDE_DIRECTORIES(${CMAKE_CURRENT_LIST_DIR}/include)

###############################################################################

MIRA_ADD_LIBRARY(GoalSetter
	SHARED
	#PACKAGE $PACKAGE$
	SOURCE
		src/GoalSetter.C

	LINK_LIBS
		MIRABase
		MIRAFramework
)
