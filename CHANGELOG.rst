^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package adi_imu
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2025-09-12)
------------------
* **Core Features:**

  * Support for multiple ADIS IMU devices including:

    * ``adis16465-1``, ``adis16465-2``, ``adis16465-3``
    * ``adis16467-1``, ``adis16467-2``, ``adis16467-3``
    * ``adis16470``
    * ``adis16475-1``, ``adis16475-2``, ``adis16475-3``
    * ``adis16477-1``, ``adis16477-2``, ``adis16477-3``
    * ``adis16500``
    * ``adis16501``
    * ``adis16505-1``, ``adis16505-2``, ``adis16505-3``
    * ``adis16507-1``, ``adis16507-2``, ``adis16507-3``
    * ``adis16545-1``, ``adis16545-2``, ``adis16545-3``
    * ``adis16547-1``, ``adis16547-2``, ``adis16547-3``
    * ``adis16550``
    * ``adis16575-2``, ``adis16575-3``
    * ``adis16576-2``, ``adis16576-3``
    * ``adis16577-2``, ``adis16577-3``

  * Supported ROS distribution: Humble Hawksbill (Ubuntu 22.04)
  * Supported platforms: x86_64, ARM64

  * Dynamic ADIS device detection and configuration at runtime
  * Real-time data publishing via ROS 2 topics:

    * IMU data (acceleration, angular velocity, temperature)
    * Diagnostics data with device-specific error flags
    * Device identification information

  * Dynamic parameter configuration via ROS 2 parameters
  * Configurable options to enable/disable diagnostic and identification publishers

* **Integration:**

  * Launch files for common use cases:

    * IMU with Madgwick filter and RViz visualization
    * ToF and IMU sensor fusion with RViz

  * Buffer-based sensor data acquisition for efficient processing
  * Proper timestamp handling from device when available

* **Documentation and Testing**

  * Comprehensive user documentation
  * Integration tests for data publishers
  * System tests for device functionality
  * API documentation with doxygen

* **Infrastructure**

  * CI/CD workflows for multiple ROS 2 distributions (Humble, Jazzy, Rolling)
  * Code quality enforcement with linting tools
  * Contributor guidelines and communication standards

* Contributors: Adrian-Stanea, Budai Robert, Dan Nechita, Ramona Bolboaca, Ramona Gradinariu, Robert, Robert Budai, VHolonec, Vasile Holonec, rbudai
