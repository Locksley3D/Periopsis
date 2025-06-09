# Periopsis


![to_be_continued_2025-May-06_12-33-56AM-000_CustomizedView50947251801](https://github.com/user-attachments/assets/13d5c7c9-0423-4e39-a03e-8fd4a0e73485)

Periopsis is a DIY room scale 3D Lidar scanner that is comprised of a 2D Lidar system and a moving base with the purpose of creating an affordable, modable, and easily integrable room scanning system.

Note: In my design, I used an RLS RE36 Absolute SSI encoder to keep track of the base position. I made this choice due to what I had at hand, but this project doesn't require such an expensive encoder. Any encoder with high PPR should be fine. My encoder was 12-bit, so 4096 points per rotation, which is higher than needed. So feel free to use any encoder you wish. You could even skip the encoder and just trust the stepper, but I decided that an encoder wasn't a bad idea for something that needs accuracy.
