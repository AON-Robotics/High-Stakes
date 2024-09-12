@echo off

@REM Check that Docker is running
@docker ps
If %ERRORLEVEL% neq 0 goto ERROR

@REM Make sure M.CSS is actually available locally
git submodule update --init --recursive --remote

@REM Remove old Docker images and output folder
docker image rm m4rqu1705/doxygen_mcss_image:latest
rmdir /s /q "./docs/output"

@REM Use Dockerfile to build image and run container
docker build --tag m4rqu1705/doxygen_mcss_image:latest -f ./docs/Dockerfile .
docker run --name doxygen_mcss_container m4rqu1705/doxygen_mcss_image:latest

@REM Copy generated website into the AON repository
docker cp doxygen_mcss_container:/root/docs/output/ ./docs/
call "./docs/output/html/index.html"

@REM Clean up containers after running everything
docker stop doxygen_mcss_container
docker rm doxygen_mcss_container

goto END

@REM Section used for general error handling when docker is not running.
:ERROR
echo ========================================================================================
echo "[>>] The Docker Engine is not running! Try opening Docker Desktop to start running it."
echo ========================================================================================

@REM End of program. Reactivating console output.
:END
@echo on