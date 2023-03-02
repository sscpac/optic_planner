cmake_minimum_required(VERSION 3.5)
project (optic_planner)

find_package(ament_cmake REQUIRED)


set( CMAKE_MODULE_PATH    ${CMAKE_MODULE_PATH}
                          ${CMAKE_CURRENT_SOURCE_DIR}/cmake/modules)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")


find_package(Cgl)
find_package(Cbc)

set(dependencies
    rclcpp
)

set(optic_SOURCE_DIR src/optic)
set(parsePDDL_SOURCE_DIR src/parsing)
set(val_SOURCE_DIR src/validate)

include_directories(
  ${optic_SOURCE_DIR}
  ${val_SOURCE_DIR}
  ${parsePDDL_SOURCE_DIR}
)

find_program(BISON_EXECUTABLE bison)

if(NOT EXISTS ${BISON_EXECUTABLE})
	message("Could not find bison executable.")
endif(NOT EXISTS ${BISON_EXECUTABLE})

FIND_PROGRAM(FLEX_EXECUTABLE flex)

if(NOT EXISTS ${FLEX_EXECUTABLE})
        message("Could not find flex executable.")
endif(NOT EXISTS ${FLEX_EXECUTABLE})

add_custom_command(
       OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
       COMMAND
        ${FLEX_EXECUTABLE}
       ARGS
       -+  --outfile=${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.lex
       DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.lex
)

add_custom_command(
       OUTPUT
        ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
       COMMAND
        ${BISON_EXECUTABLE}
       ARGS
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.yacc -o ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
       DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/pddl+.yacc ${CMAKE_CURRENT_BINARY_DIR}/lex.yy.cc
       COMMAND
        ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/fixyywrap ${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/${parsePDDL_SOURCE_DIR}/
)


set(libParsePDDL_SRCS
      ${CMAKE_CURRENT_BINARY_DIR}/pddl+.cpp
      ${parsePDDL_SOURCE_DIR}/ptree.cpp
      ${parsePDDL_SOURCE_DIR}/DebugWriteController.cpp
)

add_library(ParsePDDL SHARED ${libParsePDDL_SRCS})

set(libInst_SRCS
      ${val_SOURCE_DIR}/SimpleEval.cpp
      ${val_SOURCE_DIR}/FastEnvironment.cpp
      ${val_SOURCE_DIR}/instantiation.cpp
      ${val_SOURCE_DIR}/typecheck.cpp
      ${val_SOURCE_DIR}/TypedAnalyser.cpp
      ${val_SOURCE_DIR}/FuncAnalysis.cpp
      ${val_SOURCE_DIR}/TIM.cpp
      ${val_SOURCE_DIR}/TimSupport.cpp
      ${val_SOURCE_DIR}/TypeStripWC.cpp
)

add_library(Inst SHARED ${libInst_SRCS})

target_link_libraries(Inst ParsePDDL)


install(TARGETS ParsePDDL
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

set(optic_build_srcs
    ${optic_SOURCE_DIR}/choosepreconditions.cpp
    ${optic_SOURCE_DIR}/compressionsafescheduler.cpp
    ${optic_SOURCE_DIR}/totalordertransformer.cpp
    ${optic_SOURCE_DIR}/partialordertransformer.cpp
    ${optic_SOURCE_DIR}/FFEvent.cpp
    ${optic_SOURCE_DIR}/FFSolver.cpp
    ${optic_SOURCE_DIR}/globals.cpp
    ${optic_SOURCE_DIR}/lpscheduler.cpp
    ${optic_SOURCE_DIR}/PreferenceHandler.cpp
    ${optic_SOURCE_DIR}/RPGBuilder.cpp
    ${optic_SOURCE_DIR}/RPGBuilderAnalysis.cpp
    ${optic_SOURCE_DIR}/RPGBuilderEvaluation.cpp
    ${optic_SOURCE_DIR}/RPGBuilderNumerics.cpp
    ${optic_SOURCE_DIR}/temporalanalysis.cpp
    ${optic_SOURCE_DIR}/minimalstate.cpp
    ${optic_SOURCE_DIR}/temporalconstraints.cpp
    ${optic_SOURCE_DIR}/numericanalysis.cpp
    ${optic_SOURCE_DIR}/solver.cpp
    ${optic_SOURCE_DIR}/solver-clp.cpp
    ${optic_SOURCE_DIR}/NNF.cpp
    )

add_library(opticCommon SHARED ${optic_build_srcs})
target_link_libraries(opticCommon ParsePDDL Inst ${CBC_LIBRARIES})

ADD_EXECUTABLE(optic_planner ${optic_SOURCE_DIR}/opticMain.cpp)
target_link_libraries(optic_planner
  opticCommon
  ${CGL_LIBRARIES}
)

install(TARGETS optic_planner opticCommon ParsePDDL Inst
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(opticCommon ParsePDDL Inst)
ament_export_dependencies(${dependencies})
ament_package()