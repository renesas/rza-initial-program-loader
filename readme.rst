Overview
--------

Initial Program Loader (IPL) for Renesas RZ/A Series

IPL is a boot program sample code for RZ/A Series.
When the RZ/A MPU is started, the IPL initializes peripheral I/O modules, Flash memory, DDR SDRAM etc., and starts the application software developed in Renesas FSP environment with the execution state of AARch64, Exception Level EL3.
IPL consists of functions such as PLL and clock control driver, peripheral I/O module driver, and Serial Flash Memory driver, etc.

Current Release
---------------

`IPL v1.0.0`_

Supported MPUs
--------------
RZ/A3UL

Supported RZ/A MPU Kits
-----------------------
- RZ/A3UL-Evaluation-Board-Kit (RTK9763U02S01000BE)

Development Environment
-----------------------
e² studio is used for build and program execution environment.
Please refer to the application note(https://github.com/renesas/rza-initial-program-loader/tree/main/application_note) for the procedure for building the environment and how to use it.

Related Links
-------------
RZ/A3UL Software Package :  https://www.renesas.com/us/en/software-tool/rza3ul-software-package

RZ Product Information: www.renesas.com/rza3ul

e² studio : www.renesas.com/e2studio

Support: www.renesas.com/support

.. _IPL v1.0.0: https://github.com/renesas/rza-initial-program-loader/releases/tag/v1.0.0

