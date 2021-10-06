# Heliostat
Arduino sketch for a heliostat (a mirror that moves with the sun throughout the day)

## What is a Heliostat?
A heliostat is a mirror that is placed outdoors and reoriented throughout the day so that it always reflects sunlight at the same point.  
[Heliostat - Wikipedia](https://en.wikipedia.org/wiki/Heliostat)  
[Video of a similar Heliostat](https://www.youtube.com/watch?v=khykXoAcDw4)  
Heliostats are an efficient way to use "daylighting" to passively light or heat and indoor area, by placing the heliostat outdoors and shining it in a window all day. They are also used in [concentrated solar power plants](https://en.wikipedia.org/wiki/Concentrated_solar_power_plant) to concentrate the sun's rays at a central heating tower.

## Approach
- Algorithms exist that precisely predict the position of the sun in the sky, given the date, time, and latitude/longitude.
- One such algorithm is provided by Joseph Michalsky in the 1988 article [The Astronomical Almanac's algorithm for approximate solar position (1950–2050)](https://www.researchgate.net/publication/222131147_The_Astronomical_Almanac's_algorithm_for_approximate_solar_position_1950-2050) which calculates the sun's azimuth (degrees from true North) and altitude (degrees up from the horizon) in the sky to within 0.01 degrees until 2050.
- Using a Real Time Clock module, the heliostat is able to internally keep track of the date and time. 
- The user must enter latitude/longitude coordinates, or use a GPS module to obtain them. (I manually entered coordinates for this project.)
- Knowing time and geographic coordinates allows the heliostat to calculate the sun's position in the sky. However, the heliostat does not yet know it's own orientation, or where to aim the sun's reflection. More on that later.
- The physical structure of the heliostat is a mirror whose orientation can be adjusted by stepper motors. I chose to have one motor control the azimuth of the mirror's normal vector, and one control the altitude. This is similar to those bathroom vanity mirrors which can rotate on a vertical post, and also tilt up and down.
- The heliostat has several button commands which allow the user to move the mirror slowly using the two motors, and to calibrate the heliostat.
- To set up and calibrate the heliostat, place the heliosat on the ground, perfectly level, and either physically, or with the motor control buttons, rotate the heliostat so that the tilting axis of the mirror is pointing at true north. With the motor control buttons, adjust the altitude of the mirror so that it is completely level. Press the calibrate button. The heliostat now knows which way is north, and which way is up, and can point at "real" azimuth/elevation vectors. From this point on, do not move or adjust the heliostat except with the control buttons.
- Now, whenever you adjust the mirror's orientation with the motor control buttons, the heliostat, which knows the sun's position and the mirror's orientation, will calculate the current direction of the reflected light. This is how the heliostat is aimed. Once the target is chosen, as the day progresses, the heliostat will reorient the mirror to continue pointing at that target. The target can be adjusted at any time.