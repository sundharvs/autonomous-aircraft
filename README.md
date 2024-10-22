# Autonomous Aircraft

This project aims to develop an autonomous aircraft in the [X-Plane 11](https://www.x-plane.com/) simulator, grounded in traditional aircraft control methods, with advanced features like see-and-avoid conflict prevention using computer vision, and ATC communication using LLMs, among others.

The end goal is to autonomously fly a VFR route from cold and dark to shutdown on the [VATSIM](https://vatsim.net/) ATC network in airspace shared with other human pilots.

*Note: This project is a continuous work in progress which represents my long-term interests and goals. For well-defined, structured, complete projects please see my other [pinned repositories](https://github.com/sundharvs).*

Currently, a basic cascaded PID control architecture has been implemented, allowing the aircraft to track heading and altitude setpoints, as well as localizer, glideslope, and flare path guidance. In the future, this will be upgraded to a nonlinear total energy controller (TEC.py) in the longitudinal axis to better address pitch-airspeed coupling.

A demo of the autoland feature can be found [here](https://youtu.be/ZjRcqrGQgLU?si=VoMbAAM7fezcrjHu):

[![Autoland Demo](https://img.youtube.com/vi/ZjRcqrGQgLU/0.jpg)](https://www.youtube.com/watch?v=ZjRcqrGQgLU)

