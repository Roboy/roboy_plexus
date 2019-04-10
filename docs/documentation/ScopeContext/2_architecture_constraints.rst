Architecture Constraints
========================


.. _runtime_interfaces:

Technical Constraints / Runtime Interface Requirements
------------------------------------------------------

Hardware Constraints
----
.. csv-table:: 
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "Altera FPGA", "All code is highly specific to the Altera FPGA. Intel has bought Altera and aims to integrate their SoC with FPGAs. We are on the right horse!"
  "Intel RealSense", "Only the intel real sense can sense it.."

Hardware Constraints

Software Constraints
----
.. csv-table:: 
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "Altera FPGA", "All code is highly specific to the Altera FPGA. Intel has bought Altera and aims to integrate their SoC with FPGAs. We are on the right horse!"
  "Intel RealSense", "Only the intel real sense can sense it..."

Software Constraints

Operating System Constraints
----
.. csv-table:: 
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "Windows 8 or higher", "Due to the Intel RealSense SDK only being supported on Windows, we are stuck with Windows"

Operating System Constraints

Programming Constraints
----
.. csv-table:: 
  :header: "Constraint Name", "Description"
  :widths: 20, 60

  "CouchDB", "We have to use the CouchDB because the type of data we have to store changes at runtime..."

Programming Constraints


.. _conventions:
