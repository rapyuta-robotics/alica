if(EXISTS ${WORKING_DIRECTORY}/${FileToCheck})
  message(STATUS "${FileToCheck} exists. So do not generate it.")
else()
  message(STATUS "${FileToCheck} doesn't exist.")
  set(re2cInput ${WORKING_DIRECTORY}/libgringo/src/input/nongroundlexer.xh)
  set(re2cOutput ${WORKING_DIRECTORY}/libgringo/src/input/nongroundlexer.hh)
  execute_process(
    COMMAND re2c -o ${re2cOutput} ${re2cInput}
  )
endif()