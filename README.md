# Monte Carlo Localization (a.k.a Particle Filter Localization)

A simple repository to understand deeply the implementation of the Monte Carlo Localization (MCL) in modern C++, and further the adaptive Monte Carlo Localization (AMCL). I use this to debug the MCL implementation of some Robotic projects.

Some results in different cases:

1. Round movement with 8 landmarks with 1000 particles:

<img src="./docs/example.gif" alt="ACL round movement" width="320"/>

2. Round movement in a corridor landmarks with 1000 particles:

<img src="./docs/corridor_round.gif" alt="moving circle in corridor" width="300"/>
<img src="./docs/corridor_round2.gif" alt="moving circle in corridor2" width="300"/>

3. Straight movement in a corridor landmarks with 1000 particles:

<img src="./docs/corridor_straight.gif" alt="moving forward in corridor" width="300"/>
<img src="./docs/corridor_straight2.gif" alt="moving forward in corridor2" width="300"/>

## Getting Started

Compiling the code:

```
mkdir build && cd build
cmake ..
make
```

Running the binary:

```
./mcl
```

## Generating a GIF from the images

```
ffmpeg -i images/steps%d.jpg -vf "fps=10,scale=640:-1:flags=lanczos" -t 5 output3.gif
```

or

```
ffmpeg -framerate 10 -i images/steps%d.jpg -t 10 -vf scale=640x480 -metadata title="Monte Carlo Localization in Corridor Straight" -metadata author="Yongkie Wiyogo" corridor_straight.gif
```
