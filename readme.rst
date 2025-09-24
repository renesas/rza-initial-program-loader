Overview
============================

Initial Program Loader (IPL) for Renesas RZ/A Series

IPL is a boot program sample code for RZ/A Series.
When the RZ/A MPU is started, the IPL initializes peripheral I/O modules, Flash memory, DDR SDRAM, etc., and starts the application software developed in Renesas FSP environment with the execution state of AARch64, Exception Level EL3.
IPL consists of functions such as PLL and clock control driver, peripheral I/O module driver, Serial Flash Memory driver, and Octa Memory Control driver, etc.

Current Release
---------------------------
IPL v2.2.0

Supported MPUs
----------------------------
* RZ/A3UL
* RZ/A3M

Supported RZ/A MPU Kits
----------------------------
* RZ/A3UL-Evaluation-Board-Kit (QSPI version)

  Model number:

  * RTK9763U02S01000BE (J-Link lite not included)
  * RTK9763U02S01002BE (J-Link Lite included)

* RZ/A3UL-Evaluation-Board-Kit (Octal-SPI version)

  Model number:

  * RTK9763U02S01001BE (J-Link lite not included)
  * RTK9763U02S01003BE (J-Link Lite included)

* EK-RZA3M Kit

  Model number:

  * RTK9EKZA3MS10001BE

Development Environment
----------------------------
e² studio is used for build and program execution environment.
Please refer to the `application note <./application_note/_index.md>`_ for the procedure for building the environment and how to use it.

Related Links
----------------------------
* RZ/A3UL Software Package: `www.renesas.com/software-tool/rza3ul-software-package <https://www.renesas.com/software-tool/rza3ul-software-package>`_
* RZ Product Information: `www.renesas.com/en/products/microcontrollers-microprocessors/rz-mpus <https://www.renesas.com/en/products/microcontrollers-microprocessors/rz-mpus>`_
* e² studio: `www.renesas.com/e2studio <https://www.renesas.com/e2studio>`_
* Support: `www.renesas.com/support <https://www.renesas.com/support>`_


