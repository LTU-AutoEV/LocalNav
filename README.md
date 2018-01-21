# Localized Laser Object Detection

![img](https://challengepost-s3-challengepost.netdna-ssl.com/photos/production/software_photos/000/590/679/datas/gallery.jpg)

## Inspiration

Our goal is to improve road safety for individuals in autonomous and semi-autonomous vehicles.

## What it does

We use a 3D laser rangefinder (also known as 3D-LIDAR) to build a virtual mesh of the environment surrounding the vehicle. We then analyze this environment to plan vehicle movements and avoid obstacles autonomously.

## How we built it

We read binary point cloud data from a 3D lidar, then after filtering with the Moving Least Squares algorithm we build a greedy mesh from the points. We then squash this to a 2.5D height map using the PointCloudLibrary, which we use to generate a 2D OccupancyGrid. Finally, a pathfinder can use this occupancy grid to pathfind around objects autonomously and in real-time. The system is built on top of the Robot Operating System (ROS) and Ubuntu linux 16.04.

## Challenges we ran into

The combination of extensive (6+ hr) compilation times, outdated library documentation, link time template errors, and poorly implemented APIs provided by ROS slowed the project to a crawl at times.

http://TheseAreNotTheDocsYouAreLookingFor.com/

## Accomplishments that we're proud of

The aforementioned challenges provided excellent experience in peer programming, team collaboration, problem solving and Google-fu. We are also proud of the working prototype of the system we were able to create despite the challenges.

## What we learned

We learned to work as a team to adapt to changing conditions in a high pressure environment.

## What's next for Localized Laser Object Detection

We intend to use the system to provide an open and accessible way to for others get involved with autonomous driving systems.
