# 📝 docs

Documentation will be available in the form of a [static
website](./output/html/index.html) similar to Okapi's. Later on it will be
available as a Latex PDF document for the engineering notebook.

---

## ✏️ Writing Documentation

[The following
document](https://github.com/AON-Robotics/AON_2020/blob/206bab2ad4cf6b485fddb6721877de483b46a739/Odometry/Doxygen%20Documentation.pdf),
written for the 2020-2021 season (VRC Changeup), is a reference guide with
examples to get started with Doxygen. Download the document and open it with a
PDF reader to make the most out of the links and the table of contents.

<a href="https://doxygen.nl/"><p align="center"> <img
  src="https://www.doxygen.nl/images/doxygen.png" alt="Doxygen Logo"
  height="50">
</p></a>

[Doxygen](https://doxygen.nl/) is a tool enables you to write comments with
particular formats which later on serves as a reference for other programmers on
what parameters particular functions and methods input and what is their output.
It's best to also show examples of how to use these in order for beginners and
new team members to get a feeling on how to use your tools.

---

## 🔨 Generating Documentation

<a href="https://www.docker.com/"><p align="center"> <img
  src="https://www.docker.com/sites/default/files/d8/2019-07/horizontal-logo-monochromatic-white.png"
  alt="Docker Logo" height="100">
</p></a>

In order to generate the documentation you need to download and install [Docker
Desktop](https://www.docker.com/get-started). This tool will enable you to
easily download all the important dependencies without having to worry about
configuring them at all, or if your environment is correctly set up. A brief
intro to Docker can be seen [in the official
website](https://www.docker.com/why-docker) and
[here](https://youtu.be/Gjnup-PuquQ)

Afterwards, make sure you've installed Docker by opening your terminal of choice
and running `docker --version`. If it works you're ready to generate the
documentation.

Inside the Command Prompt use `cd` to go to `2021-Tipping-Point` local
repository. If using Windows run the following command:

> `call "./docs/windows.bat"`

If using a Unix-based OS, specially one running a bash terminal, run the
following command:

> `sudo ./docs/unix.sh`

The first time you run this after turning on your computer might take a while to
set up. In my case it took around 1 minute. It's important to note that the
Docker image will occupy around 100 MB of space as of the latest version in
January 10, 2021.

After a while it should finish and automatically open an `index.html` file for
you with the latest updates to the code's documentation. In case it doesn't
work, the files will be inside `docs/output/html`, which can be used to later
deploy the static website with GitHub pages.

After the first time running the command, the documentation will get generated
in a matter of seconds since Docker would have cached the image by then.

---

### 📑 Summary

 1. Download and install [Docker Desktop](https://www.docker.com/get-started).
 2. Open up a terminal and run the following commands:
    * `cd Documents/AON_Robotics/2021-Tipping-Point`. Make sure to change the
      folder address depending on your particular case.
    * `call "./docs/windows.bat"` in Windows or `sudo ./docs/unix.sh` in
      Unix-based systems.
 3. Open `2021-Tipping-Point/docs/output/html/index.html` in your browser to see
    the documentation.
