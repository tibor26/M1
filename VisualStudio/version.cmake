
# Get the latest version number, commit hash
execute_process(
	COMMAND git describe --tags --long --dirty
	WORKING_DIRECTORY ${GIT_DIR}
	OUTPUT_VARIABLE GIT_DESCRIBE
	OUTPUT_STRIP_TRAILING_WHITESPACE)
if ("${GIT_DESCRIBE}" MATCHES "^$")
message(WARNING "Counld't run \"git describe\". Is git installed?")
set(GIT_DESCRIBE "........-...-g0000000")  # init variable to match regex
set(VERSION_DATE "...")
else()
execute_process(
	COMMAND git log -1 --format=%cd --date=format:%y%m%d
	WORKING_DIRECTORY ${GIT_DIR}
	OUTPUT_VARIABLE VERSION_DATE
	OUTPUT_STRIP_TRAILING_WHITESPACE)
endif()

# version number (git tag), last 6 characters
string( REGEX REPLACE "-.+$" "" VERSION_NUMBER ${GIT_DESCRIBE} )
string( SUBSTRING "${VERSION_NUMBER}" 0 6 VERSION_NUMBER)
# number of commits since last tag, append D if state is dirty
string( REGEX REPLACE "^.+-([0-9]+)-.+$" "\\1" VERSION_OFFSET ${GIT_DESCRIBE} )
if ( "${GIT_DESCRIBE}" MATCHES "^.+-dirty$" )
string (APPEND VERSION_OFFSET "D")
endif()
string( SUBSTRING "${VERSION_OFFSET}" 0 6 VERSION_OFFSET)
# git commit hash, 6 characters, uppercase
string( REGEX REPLACE "^.+-[0-9]+-g([0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f][0-9a-f])[0-9a-f](-dirty)?$" "\\1" VERSION_COMMIT ${GIT_DESCRIBE} )
string( TOUPPER ${VERSION_COMMIT} VERSION_COMMIT)

message(GIT_DESCRIBE="${GIT_DESCRIBE}")
message(VERSION_NUMBER="${VERSION_NUMBER}")
message(VERSION_OFFSET="${VERSION_OFFSET}")
message(VERSION_COMMIT="${VERSION_COMMIT}")
message(VERSION_DATE="${VERSION_DATE}")

configure_file(${SRC} ${DST} @ONLY)
