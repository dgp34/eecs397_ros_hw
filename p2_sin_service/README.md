# p2_sin_service

This package addresses P2:  Create a service that applies amplitudes and frequencies to the sin_commander built in P1.

## How to Run

Use the launch file to start the controller, simulator, and commander. In a separate terminal, start the sin client separately using rosrun p2_sin_service sin_client. It will prompt for an amplitude and frequency. It assumes proper input.  If the client needs to be ended, use of ctrl-z will end it faster than ctrl-c.  There is no built-in end input.
