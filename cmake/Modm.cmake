# This file handles the integration of the modm library into the CMake build process.
# It fetches the modm library using FetchContent, sets up a Python virtual environment,
# installs dependencies from requirements.txt, and then uses lbuild to generate
# the necessary build files for modm based on project.xml.

# ==============================================================================
# Setup Python Environment for modm using uv
# ==============================================================================

# 1. Suche nach dem uv-Executable auf dem Host-System
find_program(UV_EXECUTABLE uv REQUIRED)

# 2. Pfade und Python-Version definieren
set(PYTHON_VERSION "3.12")
set(VENV_PATH "${CMAKE_SOURCE_DIR}/.venv")
set(REQUIREMENTS_TXT "${CMAKE_SOURCE_DIR}/requirements.txt")

if(WIN32)
  set(VENV_PYTHON "${VENV_PATH}/Scripts/python.exe")
else()
  set(VENV_PYTHON "${VENV_PATH}/bin/python")
endif()

# 3. Virtual Environment mit Python 3.12 erstellen (falls nicht vorhanden)
if(NOT EXISTS "${VENV_PATH}")
    message(STATUS "Creating Python ${PYTHON_VERSION} virtual environment using uv...")
    execute_process(
        COMMAND "${UV_EXECUTABLE}" venv --python "${PYTHON_VERSION}" "${VENV_PATH}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMAND_ERROR_IS_FATAL ANY
    )
endif()

# 4. Abhängigkeiten aus requirements.txt via uv installieren
# (uv prüft extrem schnell, ob Pakete bereits installiert sind)
if(EXISTS "${REQUIREMENTS_TXT}")
    message(STATUS "Installing/updating Python dependencies with uv...")
    execute_process(
        COMMAND "${UV_EXECUTABLE}" pip install -r "${REQUIREMENTS_TXT}" --python "${VENV_PYTHON}"
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMAND_ERROR_IS_FATAL ANY
    )
else()
    message(WARNING "requirements.txt not found at: ${REQUIREMENTS_TXT}")
endif()

# Optional: Setze den Python-Executable-Pfad für spätere CMake-Schritte (z. B. lbuild Aufrufe)
set(Python3_EXECUTABLE "${VENV_PYTHON}" CACHE FILEPATH "Python 3 interpreter in venv" FORCE)