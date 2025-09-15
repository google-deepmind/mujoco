file(READ "${TARGET_FILE}" FILE_CONTENTS)

string(REGEX REPLACE
  "cmake_minimum_required\\(VERSION [0-9\\.]+\\)"
  "cmake_minimum_required(VERSION ${DESIRED_VERSION})"
  NEW_CONTENTS
  "${FILE_CONTENTS}"
)

file(WRITE "${TARGET_FILE}" "${NEW_CONTENTS}")
