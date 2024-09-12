# Retrieves a docker base image that runs Python 3.10 in Alpine 3.15
FROM python:3.10-alpine3.15

# Make sure to update package lists and upgrade Alpine
RUN apk --update-cache upgrade

# Install dependencies that are essential to run MCSS. Ended up only needing doxygen for this
RUN apk add doxygen

# Upgrade pip before beginning to install required packages
RUN pip install --upgrade pip

# Install pip packages to run MCSS
RUN pip install jinja2 Pygments

# Change working directory to /root in order to contain the folders in a single spot.
WORKDIR /root

# Copy relevant files to generate documentation
COPY ./docs ./docs
COPY ./include/aon ./include/aon

# Little trick to change styling color from blue to green!!
#  Headings
RUN sed -i "s/#a5c9ea/#339966/g" ./docs/m.css/css/m-dark+documentation.compiled.css
#  Links
RUN sed -i "s/#5b9dd9/#66cc99/g" ./docs/m.css/css/m-dark+documentation.compiled.css

# Prepare log file for output warnings output
RUN mkdir -p ./docs/output/ && touch ./docs/output/warnings.txt

# Finally generate documentation
RUN python docs/m.css/documentation/doxygen.py docs/Doxyfile-mcss