# Readme Template
This template file details the format for a readme file for a package/repo. A
readme file should start with the title of the package/repo/topic (as above) and
continue with the sections outlined below.

## Table of Contents

 * [Purpose](#purpose)
 * [Functionality](#functionality)
 * [Use](#Use)
 * [Dependencies](#dependencies)

## Purpose <a name="purpose"></a>
This section should detail the main idea of the package/repo/topic

This should start with a short (1-2 line) statement of purpose (as above). A
longer description should follow. The idea is that someone who is somewhat
familiar with the package/repo/topic can read the short statement to jog their
memory about the purpose. The longer description of purpose is for when the
short description doesn't suffice. For example, someone unfamiliar with the
referenced content should be able to read the purpose section and have a pretty
good idea whether or not the package/repo/etc. will do what they need it to do.
However, this shouldn't be too long or contain very many details of *how* the
purpose is accomplished.

Things to think about:
*  What does the package/repo/topic/etc. accomplish?
*  Why does it do what it does?
*  Why was this content developed?

## Functionality <a name="functionality"></a>
This section details the *how* of the package/etc. While the source code should
be self-documenting, and thus the main source of in-depth documentation, this
section should be pretty heavy on the details that are important to
understanding the way that the code/etc. works. Ideally, after reading this
section someone should be well equipped to then read through either lower-level
readme documents (which would be linked in this document) or the source code
without confusion or extra effort to understand what is going on.

Things to think about:
*  How does this package/etc. actually accomplish what it should?
*  What are key details for understanding the code?
*  What are some things that might be confusing to others reading the code?
*  What were things that took a while to develop or debug?
*  Which lower-level parts do what?
*  Which things run as ROS nodes?
*  What is the overall class structure (has-a and is-a relationships)?
*  What role do config files play?

## Use <a name="use"></a>
This section should detail how to *use* the content. Reading this section should
allow others to feel like they can install, start, and use the package/etc.
without any major hiccups. Try to be as thorough as possible, including things
like ROS topics that are published/subscribed to, ROS parameters that are used,
and details of the content of config files and what changing their parameters
accomplishes.

Things to think about:
*  What do you do to get this package/etc. set up to use?
*  What do you change for different configurations?
*  What is the process to run this package/etc.?
*  What needs to be run before and after this package?
*  How does someone interact with the running code to achieve what they want?
*  How does someone use the class(es) in this package/library/etc. for what they want to do?
*  What are key functions/methods that are exposed to the user and how do they work?

## Dependencies <a name="dependencies"></a>
This section lists the other packages/libraries that are needed for running or
using this package/etc. This should also include instructions (or links to
instructions) for installing these libraries, particularly if they don't follow
a typical install method. However, it's always a good idea to include more
instructions than less. Even just mentioning that a library is installed via
apt or pip is better than nothing, but the best is actually listing the
command(s) that are needed.

Things to think about:
*  What do I have installed that this needs?
*  What includes/imports/etc. are used?
*  What tools do I use to build/install/run?
*  What tools might be helpful, even if they're not necessary?
